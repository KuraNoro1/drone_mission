#!/usr/bin/env python3
"""
统一视觉检测 - 根据任务状态自动切换模式
- 投放区: YOLO 桶检测 → /tmp/vision_pipe (二进制)
- 侦察区: YOLO 桶检测 + HSV 色块分析 → /tmp/recon_pipe (文本)
- 返航降落: YOLO H 检测 → /tmp/h_pipe (文本)
- 读取 /tmp/mission_cmd 自动切换 (C++ 状态通知)
- 仿真模式自动启动 gz_gst_bridge 相机桥
"""

import cv2
import torch
import struct
import os
import sys
import time
import queue
import threading
import numpy as np
import fcntl
from collections import defaultdict
from datetime import datetime

# ─────────────────────────── 配置 ───────────────────────────
BUCKET_MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "../yolo/best.pt")
H_MODEL_PATH      = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "../yolo/best_H.pt")
YOLOV5_REPO       = "/home/hy/yolov5"

VISION_PIPE = "/tmp/vision_pipe"
RECON_PIPE  = "/tmp/recon_pipe"
H_PIPE      = "/tmp/h_pipe"
CMD_PIPE    = "/tmp/mission_cmd"

STREAM_URL = "tcp://127.0.0.1:5000"
IMG_SIZE = 416
CONF_THRESH = 0.6
H_CONF_THRESH = 0.5

SIM_MODE = False   # 由启动脚本传入 --sim
GST_BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "gz_gst_bridge.py")

# ── 真机 IMX219 (1280x720, f=3.04mm, pixel≈2.24µm binned) ──
CAM_FX_REAL  = 1357.0; CAM_FY_REAL  = 1357.0
CAM_CX_REAL  = 640.0;  CAM_CY_REAL  = 360.0

# ── 仿真向下相机 (640x640, FOV=60°) ──
CAM_FX_SIM   = 554.26; CAM_FY_SIM   = 554.26
CAM_CX_SIM   = 320.0;  CAM_CY_SIM   = 320.0

# 运行时根据 --sim 自动切换
CAM_FX = CAM_FX_REAL; CAM_FY = CAM_FY_REAL
CAM_CX = CAM_CX_REAL; CAM_CY = CAM_CY_REAL
ALT_PIPE = "/tmp/altitude_pipe"
CURRENT_ALTITUDE = 1.5
ALT_TIMESTAMP = 0.0
ALT_LOCK = threading.Lock()
ALT_MAX_AGE = 1.0

MAX_BUCKETS = 5
MARKER_MIN_AREA = 30
MARGIN = 5
EXPAND_RATIO = 2.5

COLOR_RANGES = {
    'red':     ((0, 40, 30),   (10, 255, 255)),
    'red2':    ((160, 40, 30), (180, 255, 255)),
    'orange':  ((10, 40, 30),  (20, 255, 255)),
    'yellow':  ((25, 40, 30),  (35, 255, 255)),
    'green':   ((35, 40, 30),  (80, 255, 255)),
    'blue':    ((100, 40, 30), (130, 255, 255)),
    'purple':  ((135, 40, 30), (160, 255, 255)),
}
IGNORE_COLORS = {'white', 'gray', 'black'}
COLOR_NAMES_CN = {
    'red': '红色', 'orange': '橙色', 'yellow': '黄色',
    'green': '绿色', 'blue': '蓝色', 'purple': '紫色',
}

THRESH_15_20 = 30
THRESH_20_25 = 55

MNT_LX, MNT_LY = -0.07, 0.001
MNT_RX, MNT_RY =  0.07, -0.001
CAM_DX, CAM_DY =  0.15, 0.0
WORLD_R = 0.10
# ────────────────────────────────────────────────────────────

USE_DISPLAY = False  # Jetson 真机默认无显示器, --display 开启
DISPLAY_FPS = 15

TEST_IMGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../TestImgs")

# ── 任务状态枚举 ──
class MissionMode:
    DROP = 0
    RECON = 1
    RTL = 2
    H_LAND = 3
    OTHER = 4

def parse_mission_state(state_str):
    s = state_str.strip().upper()
    if s in ("DROP_SEARCH", "DROP_VISUAL_SERVO", "TRANSIT_TO_DROP"):
        return MissionMode.DROP
    if s == "RECON_SCAN":
        return MissionMode.RECON
    if s == "RTL":
        return MissionMode.RTL
    if s == "LANDED":
        return MissionMode.H_LAND
    return MissionMode.OTHER

# ── 颜色分析 ──
def find_color_buckets(frame, min_area=80):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    buckets = []
    for name, (lo, hi) in COLOR_RANGES.items():
        if name in IGNORE_COLORS: continue
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
        k = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < min_area: continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            sz = int(max(w, h) * EXPAND_RATIO)
            bx = max(0, cx - sz // 2); by = max(0, cy - sz // 2)
            bw = min(frame.shape[1] - bx, sz)
            bh = min(frame.shape[0] - by, sz)
            if bw > 30 and bh > 30 and bw < frame.shape[1] * 0.8:
                buckets.append((bx, by, bx + bw, by + bh))
    return buckets

def merge_buckets(buckets, iou_thresh=0.3):
    if not buckets: return []
    buckets = sorted(buckets, key=lambda b: (b[2]-b[0])*(b[3]-b[1]), reverse=True)
    merged = []
    for b in buckets:
        ok = True
        for m in merged:
            x1 = max(b[0], m[0]); y1 = max(b[1], m[1])
            x2 = min(b[2], m[2]); y2 = min(b[3], m[3])
            if x2 > x1 and y2 > y1:
                ia = (x2-x1)*(y2-y1)
                if ia / min((b[2]-b[0])*(b[3]-b[1]), (m[2]-m[0])*(m[3]-m[1])) > iou_thresh:
                    ok = False; break
        if ok: merged.append(b)
    return merged

def analyze_colors(roi, min_area=80):
    if roi is None or roi.size == 0: return {}
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    total = roi.shape[0] * roi.shape[1]
    color_map = defaultdict(int)
    for name, (lo, hi) in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
        k = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        s = sum(cv2.contourArea(c) for c in contours if cv2.contourArea(c) >= min_area)
        if s > 0: color_map[name] += s
    if 'red' in color_map and 'red2' in color_map:
        color_map['red'] += color_map.pop('red2')
    elif 'red2' in color_map:
        color_map['red'] = color_map.pop('red2')
    final = {}
    for c, a in color_map.items():
        p = (a / total) * 100.0
        if p > 0.5: final[c] = round(p, 2)
    return dict(sorted(final.items(), key=lambda x: x[1], reverse=True))

def classify_bucket_id(pixel_width, cx, cy):
    with ALT_LOCK:
        alt = CURRENT_ALTITUDE
        alt_ts = ALT_TIMESTAMP
    if alt <= 0.1 or time.time() - alt_ts > ALT_MAX_AGE:
        alt = 1.5
    dx = cx - CAM_CX
    dy = cy - CAM_CY
    cos_theta = 1.0 / np.sqrt(1.0 + (dx * dx) / (CAM_FX * CAM_FX) + (dy * dy) / (CAM_FY * CAM_FY))
    if cos_theta < 0.1:
        cos_theta = 0.1
    slantDist = alt / cos_theta
    real_d = (pixel_width * slantDist) / CAM_FX
    if 0.125 <= real_d <= 0.175:
        return 1
    if 0.175 < real_d <= 0.225:
        return 2
    if 0.225 < real_d <= 0.325:
        return 3
    if pixel_width < THRESH_15_20:
        return 1
    if pixel_width < THRESH_20_25:
        return 2
    return 3

def compute_mount_pixels(altitude):
    if altitude < 0.1:
        altitude = 0.1
    offsetL = np.sqrt((MNT_LX - CAM_DX) ** 2 + (MNT_LY - CAM_DY) ** 2)
    offsetR = np.sqrt((MNT_RX - CAM_DX) ** 2 + (MNT_RY - CAM_DY) ** 2)
    slantL = np.sqrt(altitude * altitude + offsetL * offsetL)
    slantR = np.sqrt(altitude * altitude + offsetR * offsetR)
    uL = CAM_CX + CAM_FX * (MNT_LX - CAM_DX) / altitude
    vL = CAM_CY + CAM_FY * (MNT_LY - CAM_DY) / altitude
    uR = CAM_CX + CAM_FX * (MNT_RX - CAM_DX) / altitude
    vR = CAM_CY + CAM_FY * (MNT_RY - CAM_DY) / altitude
    r = WORLD_R * CAM_FX / ((slantL + slantR) * 0.5)
    return uL, vL, uR, vR, r

def draw_mount_points(img):
    with ALT_LOCK:
        alt = CURRENT_ALTITUDE
        alt_ts = ALT_TIMESTAMP
    if alt <= 0.1 or time.time() - alt_ts > ALT_MAX_AGE:
        alt = 1.5
    pts = compute_mount_pixels(alt)
    uL, vL, uR, vR, radius = pts
    h, w = img.shape[:2]
    sx, sy = w / 640.0, h / 640.0
    r = int(max(radius * sx, 3))
    uLs, vLs = int(uL * sx), int(vL * sy)
    uRs, vRs = int(uR * sx), int(vR * sy)
    cv2.circle(img, (uLs, vLs), r, (0, 0, 255), 2)
    cv2.circle(img, (uLs, vLs), 3, (0, 0, 255), -1)
    cv2.putText(img, "L", (uLs - 20, vLs - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.circle(img, (uRs, vRs), r, (0, 255, 0), 2)
    cv2.circle(img, (uRs, vRs), 3, (0, 255, 0), -1)
    cv2.putText(img, "R", (uRs - 20, vRs - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(img, f"H={alt:.1f}m r={r}px", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# ── 全局状态 ──
current_mode = MissionMode.OTHER
mode_lock = threading.Lock()
running = True

raw_queue = queue.Queue(maxsize=1)
disp_queue = queue.Queue(maxsize=1)
popup_queue = queue.Queue(maxsize=1)
save_queue = queue.Queue(maxsize=10)  # 图片保存队列, 非阻塞
vision_pipe_queue = queue.Queue(maxsize=1)
recon_pipe_queue  = queue.Queue(maxsize=1)
h_pipe_queue      = queue.Queue(maxsize=1)

vision_pipe_fd = None
recon_pipe_fd  = None
h_pipe_fd      = None

save_dir = ""          # 当前保存目录
save_frame_interval = 5  # 每 N 帧保存一张, 平衡存储和数据量
save_frame_count = 0

bucket_model = None
h_model = None
bridge_proc = None

# ── 相机桥 ──
def start_camera_bridge():
    global bridge_proc
    if not SIM_MODE: return
    import subprocess
    cmd = ["/usr/bin/python3", GST_BRIDGE, "--tcp", "5000", "--no-display"]
    print(f"[bridge] 启动: {' '.join(cmd)}")
    bridge_proc = subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr)
    print(f"[bridge] PID={bridge_proc.pid}")
    time.sleep(3)

def stop_camera_bridge():
    global bridge_proc
    if bridge_proc and bridge_proc.poll() is None:
        print("[bridge] 正在关闭...")
        bridge_proc.terminate()
        try:
            bridge_proc.wait(timeout=3)
        except:
            bridge_proc.kill()
        print("[bridge] 已关闭")

# ── 模型加载 ──
def load_bucket_model():
    global bucket_model
    if bucket_model is not None: return
    print("加载桶检测模型...")
    bucket_model = torch.hub.load(YOLOV5_REPO, 'custom', path=BUCKET_MODEL_PATH,
                                   source='local', device='0', force_reload=False)
    bucket_model.conf = CONF_THRESH
    bucket_model.classes = [0]
    print("桶模型就绪")

def load_h_model():
    global h_model
    if h_model is not None: return
    print("加载 H 检测模型...")
    h_model = torch.hub.load(YOLOV5_REPO, 'custom', path=H_MODEL_PATH,
                              source='local', device='0', force_reload=False)
    h_model.conf = H_CONF_THRESH
    h_model.classes = [0]
    print("H 模型就绪")

def maybe_unload_bucket_model():
    global bucket_model
    if bucket_model is not None:
        del bucket_model
        bucket_model = None
        torch.cuda.empty_cache()
        print("已释放桶模型显存")

def maybe_unload_h_model():
    global h_model
    if h_model is not None:
        del h_model
        h_model = None
        torch.cuda.empty_cache()
        print("已释放 H 模型显存")

# ── 管道初始化 ──
def open_pipe(path, mode):
    if not os.path.exists(path):
        try:
            os.mkfifo(path)
        except FileExistsError:
            pass
    fd = open(path, mode)
    flags = fcntl.fcntl(fd.fileno(), fcntl.F_GETFL)
    fcntl.fcntl(fd.fileno(), fcntl.F_SETFL, flags | os.O_NONBLOCK)
    return fd

def init_pipes():
    global vision_pipe_fd, recon_pipe_fd, h_pipe_fd
    vision_pipe_fd = open_pipe(VISION_PIPE, 'wb')
    print("vision_pipe 就绪")
    recon_pipe_fd  = open_pipe(RECON_PIPE, 'wb')
    print("recon_pipe 就绪")
    h_pipe_fd      = open_pipe(H_PIPE, 'wb')
    print("h_pipe 就绪")

# ── 任务状态读取线程 ──
def mission_cmd_reader():
    global current_mode
    if not os.path.exists(CMD_PIPE):
        os.mkfifo(CMD_PIPE)
    fd = open_pipe(CMD_PIPE, 'rb')
    buf = b""
    while running:
        try:
            data = os.read(fd.fileno(), 256)
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    state = line.decode('utf-8').strip()
                    if state:
                        new_mode = parse_mission_state(state)
                        with mode_lock:
                            if new_mode != current_mode:
                                print(f"[STATE] {state} → mode={new_mode}")
                                current_mode = new_mode
            else:
                time.sleep(0.05)
        except BlockingIOError:
            time.sleep(0.05)
        except Exception as e:
            print(f"[CMD] 错误: {e}")
            time.sleep(1)

# ── 高度读取线程 ──
def altitude_reader():
    global CURRENT_ALTITUDE, ALT_TIMESTAMP
    if not os.path.exists(ALT_PIPE):
        print(f"[ALT] {ALT_PIPE} 不存在")
        return
    try:
        alt_fd = os.open(ALT_PIPE, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return
    buf = b""
    while running:
        try:
            data = os.read(alt_fd, 256)
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        h = float(line.strip())
                        with ALT_LOCK:
                            CURRENT_ALTITUDE = h
                            ALT_TIMESTAMP = time.time()
                    except ValueError: pass
            else:
                time.sleep(0.05)
        except BlockingIOError:
            time.sleep(0.05)

# ── 采集线程 ──
def capture_worker():
    global running
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print("无法连接 TCP 视频流")
        running = False
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print("采集线程已启动")
    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.005); continue
        while not raw_queue.empty():
            try: raw_queue.get_nowait()
            except queue.Empty: break
        raw_queue.put(frame)
    cap.release()
    print("采集线程退出")

# ── 推理线程 ──
def inference_worker():
    global running, current_mode
    print("推理线程已启动")
    frame_count = 0

    while running:
        if raw_queue.empty():
            time.sleep(0.001); continue

        try:
            frame = raw_queue.get()
            frame_count += 1

            with mode_lock: mode = current_mode

            if mode == MissionMode.DROP:
                process_drop(frame, frame_count)
            elif mode == MissionMode.RECON:
                process_recon(frame, frame_count)
            elif mode in (MissionMode.RTL, MissionMode.H_LAND):
                process_h(frame, frame_count)
            else:
                push_display(frame)

            if mode != MissionMode.RECON and USE_DISPLAY:
                while not popup_queue.empty():
                    try: popup_queue.get_nowait()
                    except queue.Empty: break
                popup_queue.put([])
        except Exception as e:
            print(f"[推理异常] {e}", flush=True)

    print("推理线程退出")

def process_drop(frame, frame_count):
    global bucket_model
    if bucket_model is None:
        load_bucket_model()
        maybe_unload_h_model()

    results = bucket_model(frame, size=IMG_SIZE)
    detections = results.xyxy[0].cpu().numpy()
    target_dets = detections[detections[:, 5] == 0] if len(detections) > 0 else []

    bucket_list = []
    result_frame = frame.copy()

    for box in target_dets:
        x1, y1, x2, y2, conf, cls = box
        cx = (x1 + x2) / 2.0; cy = (y1 + y2) / 2.0
        bid = classify_bucket_id(x2 - x1, cx, cy)
        bucket_list.append((bid, cx, cy))
        cv2.rectangle(result_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
        cv2.circle(result_frame, (int(cx), int(cy)), 5, (0,0,255), -1)
        cv2.putText(result_frame, f"bucket{bid}", (int(x1), int(y1)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    if len(bucket_list) == 0:
        if frame_count % 30 == 0: print("[DROP] None")
    else:
        for bid, cx, cy in bucket_list:
            print(f"[DROP] bucket{bid}: ({cx:.1f}, {cy:.1f})")

    while not vision_pipe_queue.empty():
        try: vision_pipe_queue.get_nowait()
        except queue.Empty: break
    vision_pipe_queue.put((len(bucket_list), bucket_list))

    push_save(result_frame, "DROP")
    push_display(result_frame)

def process_recon(frame, frame_count):
    global bucket_model
    if bucket_model is None:
        load_bucket_model()
        maybe_unload_h_model()

    results = bucket_model(frame, size=IMG_SIZE)
    detections = results.xyxy[0].cpu().numpy()
    yolo_boxes = []
    if len(detections) > 0:
        for box in detections[detections[:, 5] == 0]:
            yolo_boxes.append(tuple(box.tolist())[:5])
    yolo_boxes.sort(key=lambda b: b[0])

    if len(yolo_boxes) < MAX_BUCKETS:
        color_b = merge_buckets(find_color_buckets(frame), 0.3)
        color_boxes = [(b[0], b[1], b[2], b[3], 0.0) for b in color_b]
        all_boxes = yolo_boxes + color_boxes
        all_boxes.sort(key=lambda b: b[0])
        merged = []
        for box in all_boxes:
            x1, y1, x2, y2, conf = box
            ok = True
            for m in merged:
                mx1, my1, mx2, my2, _ = m
                ix1 = max(x1,mx1); iy1 = max(y1,my1)
                ix2 = min(x2,mx2); iy2 = min(y2,my2)
                if ix2>ix1 and iy2>iy1:
                    ia=(ix2-ix1)*(iy2-iy1)
                    a1=(x2-x1)*(y2-y1); a2=(mx2-mx1)*(my2-my1)
                    if ia/min(a1,a2)>0.3: ok=False; break
            if ok: merged.append((x1,y1,x2,y2,conf))
        final_boxes = merged
    else:
        final_boxes = yolo_boxes

    result_frame = frame.copy()
    pipe_parts = []

    for idx, box in enumerate(final_boxes):
        if idx >= MAX_BUCKETS: break
        bid = idx + 1
        x1, y1, x2, y2, conf = box
        x1i, y1i = max(0, int(x1)), max(0, int(y1))
        x2i = min(frame.shape[1], int(x2))
        y2i = min(frame.shape[0], int(y2))
        roi = frame[y1i+MARGIN:y2i-MARGIN, x1i+MARGIN:x2i-MARGIN]

        if roi.size == 0:
            pipe_parts.append(f"{bid}:empty")
            draw_rect(result_frame, x1i, y1i, x2i, y2i, f"B{bid} [EMPTY]", (0,200,200), 1)
            continue

        props = analyze_colors(roi, MARKER_MIN_AREA)
        if props:
            ps = ",".join(f"{c}:{p:.1f}" for c,p in props.items())
            pipe_parts.append(f"{bid}:{ps}")
            cn_str = ",".join(f"{COLOR_NAMES_CN.get(c,c)}:{p:.1f}" for c,p in props.items())
            print(f"[RECON] B{bid}: {cn_str}")

            y_off = y1i - 10
            for c, p in list(props.items())[:3]:
                cn = COLOR_NAMES_CN.get(c, c)
                cv2.putText(result_frame, f"{cn}:{p:.1f}%", (x1i, y_off),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
                y_off -= 18
            draw_rect(result_frame, x1i, y1i, x2i, y2i, f"B{bid} [DETECTED]", (0,255,0), 3)
        else:
            pipe_parts.append(f"{bid}:empty")
            draw_rect(result_frame, x1i, y1i, x2i, y2i, f"B{bid} [EMPTY]", (0,200,200), 1)

    pipe_msg = ";".join(pipe_parts) if pipe_parts else "None"
    while not recon_pipe_queue.empty():
        try: recon_pipe_queue.get_nowait()
        except queue.Empty: break
    recon_pipe_queue.put(pipe_msg)

    if USE_DISPLAY:
        popups = []
        for bid in range(1, len(final_boxes) + 1):
            if bid > MAX_BUCKETS: break
            box = final_boxes[bid - 1]
            x1, y1, x2, y2, _ = box
            x1i, y1i = max(0, int(x1)), max(0, int(y1))
            x2i = min(frame.shape[1], int(x2))
            y2i = min(frame.shape[0], int(y2))
            roi = frame[y1i:y2i, x1i:x2i]
            if roi.size > 0:
                popups.append(cv2.resize(roi, (160, 160)))
        while not popup_queue.empty():
            try: popup_queue.get_nowait()
            except queue.Empty: break
        popup_queue.put(popups)

    push_save(result_frame, "RECON")
    push_display(result_frame)

def draw_rect(frame, x1, y1, x2, y2, label, color, thickness):
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
    cv2.putText(frame, label, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

def process_h(frame, frame_count):
    global h_model
    if h_model is None:
        load_h_model()
        maybe_unload_bucket_model()

    results = h_model(frame, size=IMG_SIZE)
    detections = results.xyxy[0].cpu().numpy()
    h_dets = detections[detections[:, 5] == 0] if len(detections) > 0 else []

    msg = "None"
    result_frame = frame.copy()

    if len(h_dets) > 0:
        best = h_dets[h_dets[:, 4].argmax()]
        x1, y1, x2, y2, conf, cls = best
        cx = (x1 + x2) / 2.0; cy = (y1 + y2) / 2.0
        msg = f"{cx:.2f},{cy:.2f}"
        cv2.rectangle(result_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,0,255), 2)
        cv2.circle(result_frame, (int(cx), int(cy)), 5, (0,255,255), -1)
        cv2.putText(result_frame, f"H ({cx:.1f},{cy:.1f})", (int(x1), int(y1)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        h, w = frame.shape[:2]
        cv2.line(result_frame, (w//2, 0), (w//2, h), (100,100,100), 1)
        cv2.line(result_frame, (0, h//2), (w, h//2), (100,100,100), 1)

    print(f"[H] {msg}")
    while not h_pipe_queue.empty():
        try: h_pipe_queue.get_nowait()
        except queue.Empty: break
    h_pipe_queue.put(msg)

    push_display(result_frame)

def push_display(frame):
    if not USE_DISPLAY: return
    while not disp_queue.empty():
        try: disp_queue.get_nowait()
        except queue.Empty: break
    disp_queue.put(frame)

def push_save(frame, mode_name):
    global save_frame_count
    save_frame_count += 1
    if save_frame_count % save_frame_interval != 0:
        return
    if save_queue.full():
        try:
            save_queue.get_nowait()
        except queue.Empty:
            pass
    save_queue.put((frame.copy(), mode_name))

# ── 管道发送线程 ──
def pipe_sender_worker():
    global running
    print("管道发送线程已启动")
    while running:
        sent = False
        if not vision_pipe_queue.empty():
            count, blist = vision_pipe_queue.get()
            try:
                fd = vision_pipe_fd
                fd.write(struct.pack('B', count))
                for bid, cx, cy in blist:
                    fd.write(struct.pack('B', bid))
                    fd.write(struct.pack('ff', cx, cy))
                fd.flush()
                sent = True
            except (BlockingIOError, BrokenPipeError, OSError): pass
        if not recon_pipe_queue.empty():
            msg = recon_pipe_queue.get()
            try:
                recon_pipe_fd.write((msg + "\n").encode())
                recon_pipe_fd.flush()
                sent = True
            except (BlockingIOError, BrokenPipeError, OSError): pass
        if not h_pipe_queue.empty():
            msg = h_pipe_queue.get()
            try:
                h_pipe_fd.write((msg + "\n").encode())
                h_pipe_fd.flush()
                sent = True
            except (BlockingIOError, BrokenPipeError, OSError): pass
        if not sent:
            time.sleep(0.002)

# ── 显示线程 ──
def display_worker():
    global running
    if not USE_DISPLAY: return
    print("显示线程已启动")
    cv2.namedWindow("Unified Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Unified Detection", 960, 540)
    delay = 1.0 / 15
    popup_windows = []
    while running:
        if not disp_queue.empty():
            frame = disp_queue.get()
            cv2.imshow("Unified Detection", frame)
        if not popup_queue.empty():
            popups = popup_queue.get()
            for i, roi in enumerate(popups):
                wn = f'Bucket_{i}'
                if wn not in popup_windows:
                    popup_windows.append(wn)
                cv2.imshow(wn, roi)
                cv2.moveWindow(wn, 150 + i * 180, 50)
            for i in range(len(popups), len(popup_windows)):
                wn = f'Bucket_{i}'
                if cv2.getWindowProperty(wn, cv2.WND_PROP_VISIBLE) >= 0:
                    cv2.destroyWindow(wn)
            popup_windows = popup_windows[:len(popups)]
        k = cv2.waitKey(1)
        if k == 27:
            running = False
            break
        if not disp_queue.empty() or not popup_queue.empty():
            continue
        time.sleep(delay)
    cv2.destroyAllWindows()
    print("显示线程退出")

# ── 图片保存线程 (非阻塞, 不影响主推理流程) ──
def ensure_save_dir():
    global save_dir
    try:
        today = datetime.now().strftime("%Y%m%d")
        base = os.path.join(TEST_IMGS_DIR, f"data_{today}")
        idx = 1
        while True:
            d = f"{base}_{idx}"
            if not os.path.exists(d):
                os.makedirs(d, exist_ok=True)
                break
            idx += 1
        save_dir = d
        print(f"[SAVE] 图片保存目录: {save_dir}")
    except Exception as e:
        print(f"[SAVE] 创建目录失败: {e}")

def save_worker():
    global running, save_dir
    print("图片保存线程已启动")
    while running:
        if save_queue.empty():
            time.sleep(0.01)
            continue
        try:
            frame, mode_name = save_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        try:
            if save_dir == "":
                ensure_save_dir()
            ts = datetime.now().strftime("%H%M%S_%f")
            fname = os.path.join(save_dir, f"{mode_name}_{ts}.jpg")
            cv2.imwrite(fname, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        except Exception as e:
            print(f"[SAVE] 保存失败: {e}")
    print("图片保存线程退出")

# ── 主程序 ──
def main():
    global running, SIM_MODE, USE_DISPLAY

    SIM_MODE = "--sim" in sys.argv
    USE_DISPLAY = "--display" in sys.argv

    global CAM_FX, CAM_FY, CAM_CX, CAM_CY
    if SIM_MODE:
        CAM_FX, CAM_FY = CAM_FX_SIM, CAM_FY_SIM
        CAM_CX, CAM_CY = CAM_CX_SIM, CAM_CY_SIM
    else:
        CAM_FX, CAM_FY = CAM_FX_REAL, CAM_FY_REAL
        CAM_CX, CAM_CY = CAM_CX_REAL, CAM_CY_REAL

    if SIM_MODE:
        print("[MODE] 仿真模式 (自动启动 gz_gst_bridge, 640x640)")
    else:
        print("[MODE] 真机模式 (依赖外部视频流, 1280x720)")

    start_camera_bridge()

    print("预加载检测模型...")
    load_bucket_model()

    init_pipes()

    threads = []
    t_cmd = threading.Thread(target=mission_cmd_reader, daemon=True); t_cmd.start(); threads.append(t_cmd)
    t_alt = threading.Thread(target=altitude_reader, daemon=True); t_alt.start(); threads.append(t_alt)
    t_cap = threading.Thread(target=capture_worker, daemon=True); t_cap.start(); threads.append(t_cap)
    t_inf = threading.Thread(target=inference_worker, daemon=True); t_inf.start(); threads.append(t_inf)
    t_pipe = threading.Thread(target=pipe_sender_worker, daemon=True); t_pipe.start(); threads.append(t_pipe)
    if USE_DISPLAY:
        t_disp = threading.Thread(target=display_worker, daemon=True); t_disp.start(); threads.append(t_disp)

    t_save = threading.Thread(target=save_worker, daemon=True); t_save.start(); threads.append(t_save)

    print("=" * 50)
    print("  统一视觉检测系统")
    print(f"  模式: {'仿真' if SIM_MODE else '真机'}")
    print("  读取 /tmp/mission_cmd 自动切换")
    print("  投放区 → 侦察区 → H降落")
    print("  Ctrl+C 退出")
    print("=" * 50)

    try:
        while running:
            with mode_lock: m = current_mode
            mode_names = {MissionMode.DROP: "投放区", MissionMode.RECON: "侦察区",
                          MissionMode.RTL: "返航降落", MissionMode.H_LAND: "H降落",
                          MissionMode.OTHER: "待命"}
            # print every 10s if mode didn't change
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n用户中断")
        running = False
    finally:
        stop_camera_bridge()
        for t in threads:
            t.join(timeout=1)
        if vision_pipe_fd: vision_pipe_fd.close()
        if recon_pipe_fd:  recon_pipe_fd.close()
        if h_pipe_fd:      h_pipe_fd.close()
        maybe_unload_bucket_model()
        maybe_unload_h_model()
        print("程序已退出")

if __name__ == "__main__":
    main()
