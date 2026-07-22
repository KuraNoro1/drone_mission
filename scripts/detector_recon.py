#!/usr/bin/env python3
"""
侦察区视觉检测 - 桶内色块比例分析
- YOLO 检测白色圆桶，HSV 色块分析桶内颜色标识
- 无头模式: 结果通过 /tmp/recon_pipe 文本管道发送
- 有显示屏: 每个桶独立小窗显示 ROI + 颜色占比
"""

import cv2
import torch
import os
import sys
import time
import queue
import threading
import numpy as np
import fcntl
from collections import defaultdict

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../yolo/best.pt")
YOLOV5_REPO = os.path.expanduser(f"~{os.environ.get('SUDO_USER', '')}/yolov5")
PIPE_PATH = "/tmp/recon_pipe"
IMG_SIZE = 416
CONF_THRESH = 0.4
TARGET_CLASS = 0
STREAM_URL = "tcp://127.0.0.1:5000"

MAX_BUCKETS = 5
MARKER_MIN_AREA = 120
MARGIN = 15
EXPAND_RATIO = 2.5

COLOR_RANGES = {
    'red':     ((0, 20, 20),   (10, 255, 255)),
    'red2':    ((160, 20, 20), (180, 255, 255)),
    'orange':  ((10, 20, 20),  (20, 255, 255)),
    'yellow':  ((25, 20, 20),  (35, 255, 255)),
    'green':   ((40, 20, 20),  (80, 255, 255)),
    'blue':    ((100, 20, 20), (130, 255, 255)),
    'purple':  ((135, 20, 20), (160, 255, 255)),
    'black':   ((0, 0, 0),     (180, 255, 30)),
    'gray':    ((0, 0, 30),    (180, 30, 70)),
    'white':   ((0, 0, 70),    (180, 30, 255)),
}

IGNORE_COLORS = {'white', 'gray', 'black'}

COLOR_NAMES_CN = {
    'red': '红色', 'orange': '橙色', 'yellow': '黄色',
    'green': '绿色', 'blue': '蓝色', 'purple': '紫色',
    'black': '黑色', 'gray': '灰色', 'white': '白色',
}

USE_DISPLAY = "--display" in sys.argv
DISPLAY_FPS = 15

def find_color_buckets(frame, min_area=80):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    buckets = []
    for color_name, (lower, upper) in COLOR_RANGES.items():
        if color_name in IGNORE_COLORS:
            continue
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            center_x = x + w // 2
            center_y = y + h // 2
            bucket_size = int(max(w, h) * EXPAND_RATIO)
            bx = max(0, center_x - bucket_size // 2)
            by = max(0, center_y - bucket_size // 2)
            bw = min(frame.shape[1] - bx, bucket_size)
            bh = min(frame.shape[0] - by, bucket_size)
            if bw > 30 and bh > 30 and bw < frame.shape[1] * 0.8:
                buckets.append((bx, by, bx + bw, by + bh))
    return buckets

def merge_buckets(buckets, iou_thresh=0.3):
    if not buckets:
        return []
    buckets = sorted(buckets, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]), reverse=True)
    merged = []
    for b in buckets:
        overlap = False
        for m in merged:
            x1 = max(b[0], m[0]); y1 = max(b[1], m[1])
            x2 = min(b[2], m[2]); y2 = min(b[3], m[3])
            if x2 > x1 and y2 > y1:
                inter = (x2 - x1) * (y2 - y1)
                area_b = (b[2] - b[0]) * (b[3] - b[1])
                area_m = (m[2] - m[0]) * (m[3] - m[1])
                if inter / min(area_b, area_m) > iou_thresh:
                    overlap = True
                    break
        if not overlap:
            merged.append(b)
    return merged

def analyze_color_proportions(roi, min_area=60):
    if roi is None or roi.size == 0:
        return {}
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    total_pixels = roi.shape[0] * roi.shape[1]
    color_area_map = defaultdict(int)
    for color_name, (lower, upper) in COLOR_RANGES.items():
        if color_name in IGNORE_COLORS:
            continue
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area_sum = 0
        for cnt in contours:
            if cv2.contourArea(cnt) >= min_area:
                area_sum += cv2.contourArea(cnt)
        if area_sum > 0:
            color_area_map[color_name] += area_sum

    if 'red' in color_area_map and 'red2' in color_area_map:
        color_area_map['red'] += color_area_map['red2']
        del color_area_map['red2']
    elif 'red2' in color_area_map:
        color_area_map['red'] = color_area_map.pop('red2')

    final = {}
    for color, area in color_area_map.items():
        pct = (area / total_pixels) * 100.0
        if pct > 0.5:
            final[color] = round(pct, 2)
    return dict(sorted(final.items(), key=lambda item: item[1], reverse=True))

def format_proportions(proportions):
    if not proportions:
        return "empty"
    return ",".join(f"{c}:{p:.1f}" for c, p in proportions.items())

raw_queue = queue.Queue(maxsize=1)
disp_queue = queue.Queue(maxsize=1)
popup_queue = queue.Queue(maxsize=1)
pipe_queue = queue.Queue(maxsize=1)
running = True
pipe_w = None

if not torch.cuda.is_available():
    print("CUDA 不可用，使用 CPU")
else:
    print(f"CUDA 可用，设备: {torch.cuda.get_device_name(0)}")

if not os.path.exists(YOLOV5_REPO) or not os.path.exists(MODEL_PATH):
    print(f"模型或仓库路径错误: {YOLOV5_REPO}, {MODEL_PATH}")
    sys.exit(1)

print("正在加载侦察模型...")
model = torch.hub.load(YOLOV5_REPO, 'custom', path=MODEL_PATH, source='local',
                       device='0', force_reload=True)
model.conf = CONF_THRESH
model.classes = [TARGET_CLASS]
print("侦察模型加载成功")

if not os.path.exists(PIPE_PATH):
    os.mkfifo(PIPE_PATH)

print("等待 C++ 连接侦察管道...")
pipe_w = open(PIPE_PATH, 'wb')
fd = pipe_w.fileno()
flags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
print("C++ 已连接侦察管道")

def capture_worker():
    global running
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        print("无法连接 TCP 视频流")
        running = False
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print("侦察采集线程已启动")
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
    print("侦察采集线程退出")

def inference_worker():
    global running
    print("侦察推理线程已启动")

    while running:
        if raw_queue.empty():
            time.sleep(0.001)
            continue

        frame = raw_queue.get()
        result_frame = frame.copy()

        results = model(frame, size=IMG_SIZE)
        detections = results.xyxy[0].cpu().numpy()
        yolo_boxes = []
        if len(detections) > 0:
            target_dets = detections[detections[:, 5] == TARGET_CLASS]
            for box in target_dets:
                x1, y1, x2, y2, conf, cls = box
                yolo_boxes.append((x1, y1, x2, y2, conf))
        yolo_boxes.sort(key=lambda b: b[0])

        if len(yolo_boxes) < MAX_BUCKETS:
            color_buckets_raw = find_color_buckets(frame, min_area=80)
            color_buckets = merge_buckets(color_buckets_raw, iou_thresh=0.3)
            color_boxes = [(b[0], b[1], b[2], b[3], 0.0) for b in color_buckets]
            all_boxes = yolo_boxes + color_boxes
            all_boxes.sort(key=lambda b: b[0])
            merged = []
            for box in all_boxes:
                x1, y1, x2, y2, conf = box
                overlap = False
                for m in merged:
                    mx1, my1, mx2, my2, _ = m
                    ix1 = max(x1, mx1); iy1 = max(y1, my1)
                    ix2 = min(x2, mx2); iy2 = min(y2, my2)
                    if ix2 > ix1 and iy2 > iy1:
                        inter = (ix2 - ix1) * (iy2 - iy1)
                        area1 = (x2 - x1) * (y2 - y1)
                        area2 = (mx2 - mx1) * (my2 - my1)
                        if inter / min(area1, area2) > 0.3:
                            overlap = True
                            break
                if not overlap:
                    merged.append((x1, y1, x2, y2, conf))
            final_boxes = merged
        else:
            final_boxes = yolo_boxes

        bucket_results = {}
        pipe_parts = []
        popups = []

        for idx, box in enumerate(final_boxes):
            bucket_id = idx + 1
            if bucket_id > MAX_BUCKETS:
                break

            x1, y1, x2, y2, conf = box
            x1i = max(0, int(x1)); y1i = max(0, int(y1))
            x2i = min(frame.shape[1], int(x2)); y2i = min(frame.shape[0], int(y2))

            roi = frame[y1i + MARGIN:y2i - MARGIN, x1i + MARGIN:x2i - MARGIN]
            if roi.size == 0:
                pipe_parts.append(f"{bucket_id}:empty")
                continue

            proportions = analyze_color_proportions(roi, min_area=MARKER_MIN_AREA)
            bucket_results[bucket_id] = {'proportions': proportions, 'roi': roi, 'bbox': (x1i, y1i, x2i, y2i)}

            if proportions:
                props_str = format_proportions(proportions)
                cn_str = ",".join(f"{COLOR_NAMES_CN.get(c,c)}:{p:.1f}" for c, p in proportions.items())
                pipe_parts.append(f"{bucket_id}:{props_str}")
                print(f"B{bucket_id}: {cn_str}")

                y_offset = y1i - 10
                for color, pct in list(proportions.items())[:3]:
                    cn = COLOR_NAMES_CN.get(color, color)
                    cv2.putText(result_frame, f"{cn}:{pct:.1f}%", (x1i, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    y_offset -= 18
                cv2.rectangle(result_frame, (x1i, y1i), (x2i, y2i), (0, 255, 0), 3)
                cv2.putText(result_frame, f"B{bucket_id} [DETECTED]", (x1i, y1i - 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                pipe_parts.append(f"{bucket_id}:empty")
                cv2.rectangle(result_frame, (x1i, y1i), (x2i, y2i), (0, 255, 0), 1)
                cv2.putText(result_frame, f"B{bucket_id} [EMPTY]", (x1i, y1i - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)

            if USE_DISPLAY:
                if roi.size > 0:
                    popups.append(cv2.resize(roi, (160, 160)))

        if len(bucket_results) == MAX_BUCKETS:
            print(f"[RECON SUMMARY] {' | '.join(pipe_parts)}")

        pipe_msg = ";".join(pipe_parts) if pipe_parts else "None"
        while not pipe_queue.empty():
            try:
                pipe_queue.get_nowait()
            except queue.Empty:
                break
        pipe_queue.put(pipe_msg)

        if USE_DISPLAY:
            while not popup_queue.empty():
                try: popup_queue.get_nowait()
                except queue.Empty: break
            popup_queue.put(popups)
            while not disp_queue.empty():
                try:
                    disp_queue.get_nowait()
                except queue.Empty:
                    break
            disp_queue.put(result_frame)

    print("侦察推理线程退出")

def pipe_sender_worker():
    global running, pipe_w
    print("侦察管道发送线程已启动")
    while running:
        if pipe_queue.empty():
            time.sleep(0.002)
            continue
        msg = pipe_queue.get()
        try:
            pipe_w.write((msg + "\n").encode('utf-8'))
            pipe_w.flush()
        except (BlockingIOError, BrokenPipeError, OSError):
            pass
        except Exception as e:
            print(f"侦察管道发送错误: {e}")
            running = False
            break
    print("侦察管道发送线程退出")

def display_worker():
    global running
    if not USE_DISPLAY:
        return
    print("侦察显示线程已启动")
    cv2.namedWindow("Recon Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Recon Detection", 1280, 720)
    delay = 1.0 / DISPLAY_FPS
    popup_windows = []
    while running:
        if not disp_queue.empty():
            frame = disp_queue.get()
            cv2.imshow("Recon Detection", frame)
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
    print("侦察显示线程退出")

def main():
    global running, pipe_w
    threads = []
    t_cap = threading.Thread(target=capture_worker, daemon=True); t_cap.start(); threads.append(t_cap)
    t_infer = threading.Thread(target=inference_worker, daemon=True); t_infer.start(); threads.append(t_infer)
    t_pipe = threading.Thread(target=pipe_sender_worker, daemon=True); t_pipe.start(); threads.append(t_pipe)
    if USE_DISPLAY:
        t_disp = threading.Thread(target=display_worker, daemon=True); t_disp.start(); threads.append(t_disp)
    print("侦察检测已启动，按 Ctrl+C 退出")
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
        print("侦察检测程序退出")

if __name__ == "__main__":
    main()
