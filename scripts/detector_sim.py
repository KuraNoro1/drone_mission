#!/usr/bin/env python3
"""
仿真视觉检测 (detector_sim) - 多桶检测版
- 自动启动 gz_gst_bridge (Gazebo 相机 → TCP :5000), 退出时自动关闭
- 检测所有桶，根据直径映射为桶1(15cm)、桶2(20cm)、桶3(25cm)
- 管道发送：数量(1字节) + 每个桶 (ID 1字节 + cx float + cy float)
"""

import cv2
import torch
import struct
import os
import sys
import time
import queue
import threading
import signal
import subprocess
import numpy as np
import fcntl

# ================== 配置 ==================
MODEL_PATH = os.path.expanduser("~/yolo_test/best.pt")
YOLOV5_REPO = os.path.expanduser(f"~{os.environ.get('SUDO_USER', '')}/yolov5")
PIPE_PATH = "/tmp/vision_pipe"
IMG_SIZE = 416
CONF_THRESH = 0.6
TARGET_CLASS = 0

STREAM_URL = "tcp://127.0.0.1:5000"
USE_DISPLAY = "--display" in sys.argv
DISPLAY_FPS = 15

THRESH_15_20 = 30
THRESH_20_25 = 55

# ── 相机内参 (与 C++ computeMountPixels 一致) ──
FX = 554.26
ALT_PIPE = "/tmp/altitude_pipe"
CURRENT_ALTITUDE = 1.5  # 默认值, 管道更新后覆盖
ALT_TIMESTAMP = 0.0
ALT_LOCK = threading.Lock()
ALT_MAX_AGE = 1.0  # 高度数据最大过期时间(秒)

GST_BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gz_gst_bridge.py")
# ==========================================

raw_queue  = queue.Queue(maxsize=1)
disp_queue = queue.Queue(maxsize=1)
pipe_queue = queue.Queue(maxsize=1)

running = True
pipe_w = None
bridge_proc = None

if not torch.cuda.is_available():
    print("CUDA 不可用, 使用 CPU")
else:
    print(f"CUDA 可用, 设备: {torch.cuda.get_device_name(0)}")

if not os.path.exists(YOLOV5_REPO):
    print(f"YOLOv5 仓库不存在: {YOLOV5_REPO}"); sys.exit(1)
if not os.path.exists(MODEL_PATH):
    print(f"模型文件不存在: {MODEL_PATH}"); sys.exit(1)

print("正在加载模型...")
try:
    model = torch.hub.load(YOLOV5_REPO, 'custom',
                           path=MODEL_PATH, source='local',
                           device='0', force_reload=True)
    model.conf = CONF_THRESH
    model.classes = [TARGET_CLASS]
    print("模型加载成功")
except Exception as e:
    print(f"模型加载失败: {e}"); sys.exit(1)

if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)
    print(f"创建管道 {PIPE_PATH}")

print("等待 C++ 程序连接管道...")
pipe_w = open(PIPE_PATH, 'wb')
fd = pipe_w.fileno()
flags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
print("C++ 已连接")

def classify_bucket_id(pixel_width, cx, cy):
    """含透视补偿的桶直径估算"""
    with ALT_LOCK:
        alt = CURRENT_ALTITUDE
        alt_ts = ALT_TIMESTAMP
    if alt <= 0.1 or time.time() - alt_ts > ALT_MAX_AGE:
        alt = 1.5
    dx = cx - 320.0  # CX=320 for sim 640x640
    dy = cy - 320.0
    cos_theta = 1.0 / np.sqrt(1.0 + (dx * dx) / (FX * FX) + (dy * dy) / (FX * FX))
    if cos_theta < 0.1:
        cos_theta = 0.1
    slantDist = alt / cos_theta
    real_diameter_m = (pixel_width * slantDist) / FX

    if 0.125 <= real_diameter_m <= 0.175:
        return 1   # 15cm
    elif 0.175 < real_diameter_m <= 0.225:
        return 2   # 20cm
    elif 0.225 < real_diameter_m <= 0.325:
        return 3   # 25cm
    else:
        # 回退到像素阈值判断
        if pixel_width < THRESH_15_20:
            return 1
        elif pixel_width < THRESH_20_25:
            return 2
        else:
            return 3

def start_camera_bridge():
    global bridge_proc
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
        except subprocess.TimeoutExpired:
            bridge_proc.kill()
        print("[bridge] 已关闭")

def capture_worker():
    global running
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print(f"无法连接 {STREAM_URL}")
        running = False
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print("采集线程已启动")
    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.005)
            continue
        while not raw_queue.empty():
            try: raw_queue.get_nowait()
            except queue.Empty: break
        raw_queue.put(frame)
    cap.release()
    print("采集线程退出")

def inference_worker():
    global running
    print("推理线程已启动")
    while running:
        if raw_queue.empty():
            time.sleep(0.001)
            continue
        frame = raw_queue.get()
        results = model(frame, size=IMG_SIZE)
        detections = results.xyxy[0].cpu().numpy()

        if len(detections) > 0:
            target_dets = detections[detections[:, 5] == TARGET_CLASS]
        else:
            target_dets = []

        bucket_list = []
        result_frame = frame.copy()

        for box in target_dets:
            x1, y1, x2, y2, conf, cls = box
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            pixel_width = x2 - x1
            bucket_id = classify_bucket_id(pixel_width, cx, cy)
            bucket_list.append((bucket_id, cx, cy))

            cv2.rectangle(result_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            cv2.circle(result_frame, (int(cx), int(cy)), 5, (0,0,255), -1)
            cv2.putText(result_frame, f"bucket{bucket_id}", (int(x1), int(y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        if len(bucket_list) == 0:
            if int(time.time() * 2) % 60 == 0:
                print("None", flush=True)
        else:
            for bid, cx, cy in bucket_list:
                print(f"bucket{bid}: ({cx:.1f}, {cy:.1f})")

        while not pipe_queue.empty():
            try: pipe_queue.get_nowait()
            except queue.Empty: break
        pipe_queue.put((len(bucket_list), bucket_list))

        if USE_DISPLAY:
            while not disp_queue.empty():
                try: disp_queue.get_nowait()
                except queue.Empty: break
            disp_queue.put(result_frame)
    print("推理线程退出")

def pipe_sender_worker():
    global running, pipe_w
    print("管道发送线程已启动")
    while running:
        if pipe_queue.empty():
            time.sleep(0.002)
            continue
        count, bucket_list = pipe_queue.get()
        try:
            pipe_w.write(struct.pack('B', count))
            for bucket_id, cx, cy in bucket_list:
                pipe_w.write(struct.pack('B', bucket_id))
                pipe_w.write(struct.pack('ff', cx, cy))
            pipe_w.flush()
        except (BlockingIOError, BrokenPipeError, OSError):
            pass
        except Exception as e:
            print(f"管道发送错误: {e}")
            running = False
            break
    print("管道发送线程退出")

def display_worker():
    global running
    if not USE_DISPLAY:
        return
    print("显示线程已启动")
    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Detection", 960, 540)
    delay = 1.0 / DISPLAY_FPS
    while running:
        if not disp_queue.empty():
            frame = disp_queue.get()
            cv2.imshow("Detection", frame)
            if cv2.waitKey(1) == 27:
                running = False
                break
        else:
            time.sleep(delay)
    cv2.destroyAllWindows()
    print("显示线程退出")

# ---------- 高度管道读取线程 ----------
def altitude_reader():
    global running, CURRENT_ALTITUDE, ALT_TIMESTAMP
    if not os.path.exists(ALT_PIPE):
        print(f"[ALT] {ALT_PIPE} 不存在, 等待 C++ 创建...")
        for _ in range(50):
            if os.path.exists(ALT_PIPE):
                break
            time.sleep(0.2)
    try:
        alt_fd = os.open(ALT_PIPE, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        print(f"[ALT] 无法打开 {ALT_PIPE}, 使用默认高度={CURRENT_ALTITUDE}m")
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
                    except ValueError:
                        pass
            else:
                time.sleep(0.05)
        except BlockingIOError:
            time.sleep(0.05)
    os.close(alt_fd)
    print("高度读取线程退出")

def cleanup(sig=None, frame=None):
    global running
    running = False

def main():
    global running, pipe_w
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    start_camera_bridge()

    threads = []
    t_cap = threading.Thread(target=capture_worker, daemon=True)
    t_cap.start(); threads.append(t_cap)
    t_infer = threading.Thread(target=inference_worker, daemon=True)
    t_infer.start(); threads.append(t_infer)
    t_pipe = threading.Thread(target=pipe_sender_worker, daemon=True)
    t_pipe.start(); threads.append(t_pipe)
    if USE_DISPLAY:
        t_disp = threading.Thread(target=display_worker, daemon=True)
        t_disp.start(); threads.append(t_disp)
    t_alt = threading.Thread(target=altitude_reader, daemon=True)
    t_alt.start(); threads.append(t_alt)

    print("所有线程已启动，按 Ctrl+C 退出")
    try:
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n用户中断")
        running = False
    finally:
        for t in threads:
            t.join(timeout=1)
        if pipe_w:
            pipe_w.close()
        stop_camera_bridge()
        print("程序已退出")

if __name__ == "__main__":
    main()
