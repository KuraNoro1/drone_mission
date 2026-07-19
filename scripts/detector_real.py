#!/usr/bin/env python3
"""
Jetson Nano 视觉检测 - 投放区专用（输出桶ID+坐标）
- 检测所有桶，根据直径映射为桶1(15cm)、桶2(20cm)、桶3(25cm)
- 管道发送：数量(1字节) + 每个桶 (ID 1字节 + cx float + cy float)
- 无桶时发送数量0
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

# ================== 用户配置 ==================
MODEL_PATH = os.path.expanduser("~/yolo_test/best.pt")
YOLOV5_REPO = os.path.expanduser("~/yolov5")
PIPE_PATH = "/tmp/vision_pipe"
IMG_SIZE = 416
CONF_THRESH = 0.6
TARGET_CLASS = 0

STREAM_URL = "tcp://127.0.0.1:5000"

USE_DISPLAY = True
DISPLAY_FPS = 15

# 直径分类像素阈值（需根据实际飞行高度标定）
THRESH_15_20 = 30
THRESH_20_25 = 55

# ── 相机内参 (与 C++ computeMountPixels 一致, 真机需实测更新) ──
FX = 554.26
FY = 554.26
CX = 320
CY = 320
CAM_DX, CAM_DY = 0.15, 0.0
MNT_LX, MNT_LY = -0.07, 0.001
MNT_RX, MNT_RY = 0.07, -0.001
WORLD_R = 0.10
ALT_PIPE = "/tmp/altitude_pipe"
CURRENT_ALTITUDE = 1.5  # 默认值, 管道更新后覆盖
ALT_LOCK = threading.Lock()
# ==============================================

raw_queue = queue.Queue(maxsize=1)
disp_queue = queue.Queue(maxsize=1)
pipe_queue = queue.Queue(maxsize=1)

running = True
pipe_w = None

# ---------- 检查 CUDA ----------
if not torch.cuda.is_available():
    print("CUDA 不可用，使用 CPU")
else:
    print(f"CUDA 可用，设备: {torch.cuda.get_device_name(0)}")

# ---------- 加载模型 ----------
if not os.path.exists(YOLOV5_REPO):
    print(f"YOLOv5 仓库不存在: {YOLOV5_REPO}")
    sys.exit(1)
if not os.path.exists(MODEL_PATH):
    print(f"模型文件不存在: {MODEL_PATH}")
    sys.exit(1)

print("正在加载模型...")
try:
    model = torch.hub.load(YOLOV5_REPO, 'custom',
                           path=MODEL_PATH, source='local',
                           device='0', force_reload=True)
    model.conf = CONF_THRESH
    model.classes = [TARGET_CLASS]
    print("模型加载成功")
except Exception as e:
    print(f"模型加载失败: {e}")
    sys.exit(1)

# ---------- 创建命名管道（非阻塞） ----------
if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)
    print(f"创建管道 {PIPE_PATH}")

print("等待 C++ 程序连接管道...")
pipe_w = open(PIPE_PATH, 'wb')
fd = pipe_w.fileno()
flags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
print("C++ 已连接")

# ---------- 直径分类与桶ID映射 ----------
def classify_bucket_id(pixel_width):
    """根据像素宽度和实际高度估算真实直径, 再分类桶ID
       与老方案 ros2_yolov8/detect.py 公式一致"""
    with ALT_LOCK:
        alt = CURRENT_ALTITUDE
    if alt <= 0.1:
        alt = 1.5

    real_diameter_m = (pixel_width * alt) / FX

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


# ---------- 挂载点像素投影 (与 C++ computeMountPixels 一致) ----------
def compute_mount_pixels(altitude):
    if altitude < 0.1:
        altitude = 0.1
    uL = CX + FX * (MNT_LX - CAM_DX) / altitude
    vL = CY + FY * (MNT_LY - CAM_DY) / altitude
    uR = CX + FX * (MNT_RX - CAM_DX) / altitude
    vR = CY + FY * (MNT_RY - CAM_DY) / altitude
    radius = WORLD_R * FX / altitude
    return uL, vL, uR, vR, radius


def draw_mount_points(img):
    """在图像上绘制挂载点投影标记 (与 gz_gst_bridge.py _drawMountCircles 风格一致)"""
    with ALT_LOCK:
        alt = CURRENT_ALTITUDE
    pts = compute_mount_pixels(alt)
    if pts is None:
        return
    uL, vL, uR, vR, radius = pts

    h, w = img.shape[:2]
    scale_x = w / 640.0
    scale_y = h / 640.0
    r = int(max(radius * scale_x, 3))

    uL_s = int(uL * scale_x)
    vL_s = int(vL * scale_y)
    uR_s = int(uR * scale_x)
    vR_s = int(vR * scale_y)

    # 左挂载点 — 红色圆圈 + 中心点
    cv2.circle(img, (uL_s, vL_s), r, (0, 0, 255), 2)
    cv2.circle(img, (uL_s, vL_s), 3, (0, 0, 255), -1)
    cv2.putText(img, "L", (uL_s - 20, vL_s - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # 右挂载点 — 绿色圆圈 + 中心点
    cv2.circle(img, (uR_s, vR_s), r, (0, 255, 0), 2)
    cv2.circle(img, (uR_s, vR_s), 3, (0, 255, 0), -1)
    cv2.putText(img, "R", (uR_s - 20, vR_s - r - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # HUD: 高度 + 半径
    cv2.putText(img, f"H={alt:.2f}m r={r}px",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(img, f"L:({uL:.0f},{vL:.0f}) R:({uR:.0f},{vR:.0f})",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

# ---------- 采集线程 ----------
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
            time.sleep(0.005)
            continue
        while not raw_queue.empty():
            try:
                raw_queue.get_nowait()
            except queue.Empty:
                break
        raw_queue.put(frame)

    cap.release()
    print("采集线程退出")

# ---------- 推理线程 ----------
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
            bucket_id = classify_bucket_id(pixel_width)
            bucket_list.append((bucket_id, cx, cy))

            cv2.rectangle(result_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(result_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(result_frame, f"bucket{bucket_id}", (int(x1), int(y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        draw_mount_points(result_frame)

        if len(bucket_list) == 0:
            print("None")
        else:
            for bid, cx, cy in bucket_list:
                print(f"bucket{bid}: ({cx:.1f}, {cy:.1f})")

        while not pipe_queue.empty():
            try:
                pipe_queue.get_nowait()
            except queue.Empty:
                break
        pipe_queue.put((len(bucket_list), bucket_list))

        if USE_DISPLAY:
            while not disp_queue.empty():
                try:
                    disp_queue.get_nowait()
                except queue.Empty:
                    break
            disp_queue.put(result_frame)

    print("推理线程退出")

# ---------- 管道发送线程 ----------
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

# ---------- 显示线程 ----------
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
    global running, CURRENT_ALTITUDE
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
                    except ValueError:
                        pass
            else:
                time.sleep(0.05)
        except BlockingIOError:
            time.sleep(0.05)
    os.close(alt_fd)
    print("高度读取线程退出")

# ---------- 主程序 ----------
def main():
    global running, pipe_w

    threads = []
    t_cap = threading.Thread(target=capture_worker, daemon=True)
    t_cap.start()
    threads.append(t_cap)

    t_infer = threading.Thread(target=inference_worker, daemon=True)
    t_infer.start()
    threads.append(t_infer)

    t_pipe = threading.Thread(target=pipe_sender_worker, daemon=True)
    t_pipe.start()
    threads.append(t_pipe)

    if USE_DISPLAY:
        t_disp = threading.Thread(target=display_worker, daemon=True)
        t_disp.start()
        threads.append(t_disp)

    t_alt = threading.Thread(target=altitude_reader, daemon=True)
    t_alt.start()
    threads.append(t_alt)

    print("所有线程已启动，按 Ctrl+C 退出")
    print("检测到桶时输出：bucketID (cx, cy)；无桶时输出 None")

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
        print("程序已退出")

if __name__ == "__main__":
    main()
