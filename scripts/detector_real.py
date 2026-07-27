#!/usr/bin/env python3
"""
Jetson Nano 视觉检测 - 投放区专用（输出桶ID+坐标）
- 检测所有桶，根据直径映射为桶1(15cm)、桶2(20cm)、桶3(25cm)
- 管道发送：数量(1字节) + 每个桶 (ID 1字节 + cx float + cy float)
- 无桶时发送数量0

[v2 改动]
  1. 删掉 model.fuse() — YOLOv5 v7.0 AutoShape 已自动融合
  2. results.render()[0].copy() — 解决只读数组无法绘图
  3. 跳帧缓存 last_result — 画面不闪烁
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
SKIP_FRAMES = 2  # [v2] 每3帧推理一次

THRESH_15_20 = 30
THRESH_20_25 = 55

FX = 554.26
FY = 554.26
CX = 320
CY = 320
CAM_DX, CAM_DY = 0.15, 0.0
MNT_LX, MNT_LY = -0.07, 0.001
MNT_RX, MNT_RY = 0.07, -0.001
WORLD_R = 0.10
ALT_PIPE = "/tmp/altitude_pipe"
CURRENT_ALTITUDE = 1.5
ALT_LOCK = threading.Lock()
# ==============================================

raw_queue = queue.Queue(maxsize=2)
disp_queue = queue.Queue(maxsize=2)
pipe_queue = queue.Queue(maxsize=2)

running = True
pipe_w = None

if not torch.cuda.is_available():
    print("CUDA 不可用，使用 CPU")
else:
    print(f"CUDA 可用，设备: {torch.cuda.get_device_name(0)}")

if not os.path.exists(YOLOV5_REPO):
    print(f"YOLOv5 仓库不存在: {YOLOV5_REPO}"); sys.exit(1)
if not os.path.exists(MODEL_PATH):
    print(f"模型文件不存在: {MODEL_PATH}"); sys.exit(1)

print("正在加载模型...")
try:
    # 清除 Windows 训练残留在 Linux 上的损坏缓存
    for cache_ext in ['.cache', '.cache.lock']:
        f = MODEL_PATH + cache_ext
        if os.path.exists(f):
            os.remove(f)

    model = torch.hub.load(YOLOV5_REPO, 'custom',
                           path=MODEL_PATH, source='local',
                           device='0', force_reload=False)
    model.conf = CONF_THRESH
    model.classes = [TARGET_CLASS]

    # FP16 fallback：Windows 训练的模型未必兼容
    try:
        model.half()
        print("模型加载成功 (FP16)")
    except Exception:
        model.float()
        print("模型加载成功 (FP32) — FP16 不可用，已回退")

except Exception as e:
    print(f"模型加载失败: {e}"); sys.exit(1)

# [v2] 预热（dtype 与模型实际 dtype 保持一致）
dtype = next(model.parameters()).dtype
dummy = torch.randn(1, 3, IMG_SIZE, IMG_SIZE).to(device='cuda', dtype=dtype)
model(dummy)
print("预热完成")

if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)
    print(f"创建管道 {PIPE_PATH}")

print("等待 C++ 程序连接管道...")
pipe_w = open(PIPE_PATH, 'wb')
fd = pipe_w.fileno()
flags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
print("C++ 已连接")

def classify_bucket_id(pixel_width):
    with ALT_LOCK:
        alt = CURRENT_ALTITUDE
    if alt <= 0.1:
        alt = 1.5
    real_diameter_m = (pixel_width * alt) / FX
    if 0.125 <= real_diameter_m <= 0.175:
        return 1
    elif 0.175 < real_diameter_m <= 0.225:
        return 2
    elif 0.225 < real_diameter_m <= 0.325:
        return 3
    else:
        if pixel_width < THRESH_15_20:
            return 1
        elif pixel_width < THRESH_20_25:
            return 2
        else:
            return 3

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
    cv2.circle(img, (uL_s, vL_s), r, (0, 0, 255), 2)
    cv2.circle(img, (uL_s, vL_s), 3, (0, 0, 255), -1)
    cv2.putText(img, "L", (uL_s - 20, vL_s - r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.circle(img, (uR_s, vR_s), r, (0, 255, 0), 2)
    cv2.circle(img, (uR_s, vR_s), 3, (0, 255, 0), -1)
    cv2.putText(img, "R", (uR_s - 20, vR_s - r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(img, f"H={alt:.2f}m r={r}px",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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
        try:
            raw_queue.put_nowait(frame)
        except queue.Full:
            pass
    cap.release()
    print("采集线程退出")

# ---------- 推理线程（v2: 跳帧 + last_result 缓存） ----------
def inference_worker():
    global running
    print("推理线程已启动")
    frame_count = 0
    last_result = None

    while running:
        if raw_queue.empty():
            time.sleep(0.001)
            continue
        frame = raw_queue.get()
        frame_count += 1

        # [v2] 跳帧：跳过时转发上一帧检测结果
        if frame_count % (SKIP_FRAMES + 1) != 1:
            if last_result is not None:
                try:
                    disp_queue.put_nowait(last_result)
                except queue.Full:
                    pass
            continue

        results = model(frame, size=IMG_SIZE)
        detections = results.xyxy[0].cpu().numpy()
        target_dets = detections[detections[:, 5] == TARGET_CLASS] if len(detections) > 0 else []

        bucket_list = []
        # [v2] render() 返回只读数组 → 必须 .copy()
        result_frame = results.render()[0].copy()

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
        last_result = result_frame  # [v2] 缓存

        if len(bucket_list) == 0:
            print("None")
        else:
            for bid, cx, cy in bucket_list:
                print(f"bucket{bid}: ({cx:.1f}, {cy:.1f})")

        try:
            pipe_queue.put_nowait((len(bucket_list), bucket_list))
        except queue.Full:
            pass

        if USE_DISPLAY:
            try:
                disp_queue.put_nowait(result_frame)
            except queue.Full:
                pass
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
    for target in [capture_worker, inference_worker, pipe_sender_worker,
                   display_worker if USE_DISPLAY else None, altitude_reader]:
        if target:
            t = threading.Thread(target=target, daemon=True)
            t.start()
            threads.append(t)

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
