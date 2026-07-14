#!/usr/bin/env python3
"""
Jetson Nano 视觉检测 — 真机版 (detector_real)
- 从 TCP 视频流拉取图像 (CSI→GStreamer→tcpserversink)
- 多线程：采集 / 推理 / 管道发送 / 显示
- 检测 class=0 (circle), 通过 /tmp/vision_pipe 发送圆心坐标
- 发送格式: struct.pack('fff', cx, cy, conf) — 12 字节二进制
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

# ================== 配置 ==================
MODEL_PATH = "/home/hy/yolo_test/best.pt"
YOLOV5_REPO = "/home/hy/yolov5"
PIPE_PATH = "/tmp/vision_pipe"
IMG_SIZE = 416
CONF_THRESH = 0.6
TARGET_CLASS = 0

STREAM_URL = "tcp://127.0.0.1:5000"
USE_DISPLAY = True
DISPLAY_FPS = 15
# ==========================================

# ---------- 全局队列 ----------
# 容量为 1，保证永远只保留最新帧
raw_queue = queue.Queue(maxsize=1)        # 采集 -> 推理
disp_queue = queue.Queue(maxsize=1)       # 推理 -> 显示
pipe_queue = queue.Queue(maxsize=1)       # 推理 -> 管道发送（可选，避免阻塞）

# 全局标志
running = True
pipe_w = None

# ---------- 检查 CUDA ----------
if not torch.cuda.is_available():
    print("⚠️ CUDA 不可用，使用 CPU")
    device = 'cpu'
else:
    print(f"✅ CUDA 可用，设备: {torch.cuda.get_device_name(0)}")

# ---------- 加载模型 ----------
if not os.path.exists(YOLOV5_REPO):
    print(f"❌ YOLOv5 仓库不存在: {YOLOV5_REPO}")
    sys.exit(1)
if not os.path.exists(MODEL_PATH):
    print(f"❌ 模型文件不存在: {MODEL_PATH}")
    sys.exit(1)

print("正在加载模型...")
try:
    model = torch.hub.load(YOLOV5_REPO, 'custom',
                           path=MODEL_PATH, source='local',
                           device='0', force_reload=True)
    model.conf = CONF_THRESH
    model.classes = [TARGET_CLASS]
    print("✅ 模型加载成功")
except Exception as e:
    print(f"❌ 模型加载失败: {e}")
    sys.exit(1)

# ---------- 创建命名管道（与 C++ 通信） ----------
if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)
    print(f"📁 创建管道 {PIPE_PATH}")

print("等待 C++ 程序连接管道...")
pipe_w = open(PIPE_PATH, 'wb')
print("C++ 已连接")

# ---------- 采集线程 ----------
def capture_worker():
    """从 TCP 流拉取最新帧，放入 raw_queue"""
    global running
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print("❌ 无法连接 TCP 视频流")
        running = False
        return

    # 关键：减少内部缓冲
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print("✅ 采集线程已启动")

    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.005)
            continue

        # 只保留最新帧（丢弃旧帧）
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
    """从 raw_queue 取最新帧，执行推理，结果放入 disp_queue 和 pipe_queue"""
    global running
    print("推理线程已启动")

    while running:
        # 没有新帧时等待
        if raw_queue.empty():
            time.sleep(0.001)
            continue

        frame = raw_queue.get()

        # ---------- 推理 ----------
        results = model(frame, size=IMG_SIZE)
        detections = results.xyxy[0].cpu().numpy()

        cx = cy = conf = 0.0
        if len(detections) > 0:
            target_dets = detections[detections[:, 5] == TARGET_CLASS]
            if len(target_dets) > 0:
                best = target_dets[target_dets[:, 4].argmax()]
                x1, y1, x2, y2, conf, cls = best
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

        # ---------- 绘制结果（在拷贝上操作，不影响原始帧） ----------
        result_frame = frame.copy()
        if cx != 0 or cy != 0:
            cv2.rectangle(result_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(result_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(result_frame, f"({cx:.1f},{cy:.1f}) {conf:.2f}",
                        (int(cx)+10, int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)

        # ---------- 发送坐标到管道（非阻塞方式） ----------
        # 使用队列避免管道阻塞影响推理循环
        while not pipe_queue.empty():
            try:
                pipe_queue.get_nowait()
            except queue.Empty:
                break
        pipe_queue.put((cx, cy, conf))

        # ---------- 结果送入显示队列 ----------
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
    """独立线程发送坐标到管道，避免阻塞推理"""
    global running, pipe_w
    print("管道发送线程已启动")

    while running:
        if pipe_queue.empty():
            time.sleep(0.002)
            continue

        cx, cy, conf = pipe_queue.get()
        try:
            data = struct.pack('fff', cx, cy, conf)
            pipe_w.write(data)
            pipe_w.flush()
        except Exception as e:
            print(f"管道发送错误: {e}")
            running = False
            break

    print("管道发送线程退出")

# ---------- 显示线程 ----------
def display_worker():
    """固定帧率显示，不影响推理速度"""
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
            if cv2.waitKey(1) == 27:   # ESC 退出
                running = False
                break
        else:
            # 没有新帧时按固定节奏等待
            time.sleep(delay)

    cv2.destroyAllWindows()
    print("显示线程退出")

# ---------- 主程序 ----------
def main():
    global running, pipe_w

    # 启动所有线程
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

    print("所有线程已启动，按 Ctrl+C 退出")

    try:
        # 保持主线程存活
        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n用户中断")
        running = False
    finally:
        # 等待所有线程结束
        for t in threads:
            t.join(timeout=1)
        if pipe_w:
            pipe_w.close()
        print("程序已退出")

if __name__ == "__main__":
    main()