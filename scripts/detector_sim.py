#!/usr/bin/env python3
"""
仿真视觉检测 (detector_sim)
- 自动启动 gz_gst_bridge (Gazebo 相机 → TCP :5000), 退出时自动关闭
- 从 TCP 流拉取图像进行 YOLO 检测
- 多线程: 采集 → 推理 → 管道发送 → OpenCV 显示窗口
- 发送格式: struct.pack('fff', cx, cy, conf) — 12 字节
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

# ================== 配置 ==================
MODEL_PATH = os.path.expanduser("~/yolo_test/best.pt")
YOLOV5_REPO = os.path.expanduser("~/yolov5")
PIPE_PATH = "/tmp/vision_pipe"
IMG_SIZE = 416
CONF_THRESH = 0.6
TARGET_CLASS = 0

STREAM_URL = "tcp://127.0.0.1:5000"
USE_DISPLAY = True
DISPLAY_FPS = 15

# gz_gst_bridge 脚本路径 (用系统 python3, 不依赖 conda)
GST_BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gz_gst_bridge.py")
# ==========================================

raw_queue  = queue.Queue(maxsize=1)
disp_queue = queue.Queue(maxsize=1)
pipe_queue = queue.Queue(maxsize=1)

running = True
pipe_w = None
bridge_proc = None

# ── CUDA ──
if not torch.cuda.is_available():
    print("⚠️ CUDA 不可用, 使用 CPU")
else:
    print(f"✅ CUDA 可用, 设备: {torch.cuda.get_device_name(0)}")

# ── 加载模型 ──
if not os.path.exists(YOLOV5_REPO):
    print(f"❌ YOLOv5 仓库不存在: {YOLOV5_REPO}"); sys.exit(1)
if not os.path.exists(MODEL_PATH):
    print(f"❌ 模型文件不存在: {MODEL_PATH}"); sys.exit(1)

print("正在加载模型...")
try:
    model = torch.hub.load(YOLOV5_REPO, 'custom',
                           path=MODEL_PATH, source='local',
                           device='0', force_reload=True)
    model.conf = CONF_THRESH
    model.classes = [TARGET_CLASS]
    print("✅ 模型加载成功")
except Exception as e:
    print(f"❌ 模型加载失败: {e}"); sys.exit(1)

# ── 管道 ──
if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)
    print(f"📁 创建管道 {PIPE_PATH}")

print("等待 C++ 程序连接管道...")
pipe_w = open(PIPE_PATH, 'wb')
print("C++ 已连接")

# ── 启动相机桥接 ──
def start_camera_bridge():
    global bridge_proc
    cmd = ["/usr/bin/python3", GST_BRIDGE, "--tcp", "5000", "--no-display"]
    print(f"[bridge] 启动: {' '.join(cmd)}")
    bridge_proc = subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr)
    print(f"[bridge] PID={bridge_proc.pid}")
    # 等待 TCP 服务器就绪
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

# ── 采集线程 ──
def capture_worker():
    global running
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print(f"❌ 无法连接 {STREAM_URL}")
        running = False
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print("✅ 采集线程已启动")
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

# ── 推理线程 ──
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

        cx = cy = conf = 0.0
        x1 = y1 = x2 = y2 = 0
        if len(detections) > 0:
            target_dets = detections[detections[:, 5] == TARGET_CLASS]
            if len(target_dets) > 0:
                best = target_dets[target_dets[:, 4].argmax()]
                x1, y1, x2, y2, conf, cls = best
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                # 每 30 帧打印一次识别结果
                if int(time.time() * 2) % 60 == 0:
                    print(f"  [YOLO] circle: ({cx:.0f},{cy:.0f}) conf={conf:.2f}  → pipe",
                          flush=True)

        result_frame = frame.copy()
        if cx != 0 or cy != 0:
            cv2.rectangle(result_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
            cv2.circle(result_frame, (int(cx), int(cy)), 5, (0,0,255), -1)
            cv2.putText(result_frame, f"({cx:.1f},{cy:.1f}) {conf:.2f}",
                        (int(cx)+10, int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)

        while not pipe_queue.empty():
            try: pipe_queue.get_nowait()
            except queue.Empty: break
        pipe_queue.put((cx, cy, conf))

        if USE_DISPLAY:
            while not disp_queue.empty():
                try: disp_queue.get_nowait()
                except queue.Empty: break
            disp_queue.put(result_frame)
    print("推理线程退出")

# ── 管道发送线程 ──
def pipe_sender_worker():
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

# ── 显示线程 ──
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

# ── 主程序 ──
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
