#!/usr/bin/env python3
"""
统一视觉检测 v5 — TensorRT 投放区加速版 (基于 v4)
================================================================
v5 核心改动:
  - 投放区桶检测: PyTorch best.pt → TensorRT best.engine (10-30x 加载加速)
  - H 降落检测: PyTorch best_H.pt → TensorRT best_H.engine (与投放区相同)
  - pycuda 必须先于 TensorRT 导入 (建立 CUDA 上下文)
  - 其他功能完全继承 v4: 侦察区视频流、模型切换、WebSocket、管道协议

v4 架构说明:
  - 侦察区完全停止推理，只覆盖式推送 WebSocket 视频帧
  - 投放区与 H 区继续实时输出像素中心坐标
  - 延迟优先: 投放隔帧推理、所有队列只保留最新数据
  - H 模型常驻策略: 首次进入侦察区后 H 模型保持加载至程序退出

任务模式:
  - 投放区: TensorRT 桶检测 → /tmp/vision_pipe (二进制)
  - 侦察区: 不做识别，仅通过 WebSocket 发送原始视频帧
  - 返航降落: TensorRT H 检测 → /tmp/h_pipe (文本)
  - 读取 /tmp/mission_cmd 自动切换 (C++ 状态通知)

用法:
  python3.6 detector_unified_v5.py [--sim] [--display]
  注意: Jetson Nano 必须用 python3.6 (自带 tensorrt 8.2.1.9 + pycuda)
================================================================
"""

import os
import sys
import time

# ═══════════════════════════════════════════════════════════════
#  ⚠️ CRITICAL: pycuda 必须在任何 TensorRT 操作之前导入
#  (建立 CUDA 上下文, 否则报 "invalid resource handle" / Cuda Driver Error)
# ═══════════════════════════════════════════════════════════════
import pycuda.driver as cuda
import pycuda.autoinit          # noqa: F401  建立并锁定当前线程 CUDA 上下文
import tensorrt as trt

import cv2
import struct
import queue
import threading
import numpy as np
from datetime import datetime
import asyncio
import websockets

# ─────────────────────────── 配置 ───────────────────────────
_USER = os.environ.get('SUDO_USER', 'hy')
# v5: 投放区和 H 区都使用 TensorRT engine
_USER = os.environ.get('SUDO_USER', 'hy')
BUCKET_ENGINE_PATH = os.path.expanduser(f"~{_USER}/exportengine/best.engine")
H_ENGINE_PATH      = os.path.expanduser(f"~{_USER}/exportengine/best_H.engine")

VISION_PIPE = "/tmp/vision_pipe"
RECON_PIPE  = "/tmp/recon_pipe"
H_PIPE      = "/tmp/h_pipe"
CMD_PIPE    = "/tmp/mission_cmd"

# ── WebSocket 推送配置 ──
WS_HOST = "0.0.0.0"
WS_PORT = 8765
RECON_WS_JPEG_QUALITY = 70  # 侦察区高质量 (平衡帧率与画质)
WS_JPEG_QUALITY = 60         # 投放/降落标准质量

STREAM_URL = "tcp://127.0.0.1:5000"
IMG_SIZE = 416
CONF_THRESH = 0.6
IOU_THRESH = 0.45
H_CONF_THRESH = 0.35
NC = 2  # YOLOv5 类别数: 0=bucket, 1=h

SIM_MODE = False
GST_BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "gz_gst_bridge.py")

# ── 相机内参 (真机 IMX219) ──
CALIB_NPZ_PATH = os.path.expanduser(f"~{os.environ.get('SUDO_USER', 'hy')}/rgb_camera_calib_1.npz")
if os.path.exists(CALIB_NPZ_PATH):
    try:
        _calib_data = np.load(CALIB_NPZ_PATH)
        _K = _calib_data['camera_matrix']
        CAM_FX_REAL = float(_K[0, 0])
        CAM_FY_REAL = float(_K[1, 1])
        CAM_CX_REAL = float(_K[0, 2])
        CAM_CY_REAL = float(_K[1, 2])
        print(f"✅ 已加载相机标定: FX={CAM_FX_REAL:.1f}, FY={CAM_FY_REAL:.1f}, CX={CAM_CX_REAL:.1f}, CY={CAM_CY_REAL:.1f}")
    except Exception as e:
        print(f"⚠️ 标定文件加载失败: {e}，使用默认值")
        CAM_FX_REAL, CAM_FY_REAL = 1357.0, 1357.0
        CAM_CX_REAL, CAM_CY_REAL = 640.0, 360.0
else:
    CAM_FX_REAL, CAM_FY_REAL = 1357.0, 1357.0
    CAM_CX_REAL, CAM_CY_REAL = 640.0, 360.0

# ── 仿真向下相机 ──
CAM_FX_SIM   = 554.26; CAM_FY_SIM   = 554.26
CAM_CX_SIM   = 320.0;  CAM_CY_SIM   = 320.0

# 运行时自动切换
CAM_FX = CAM_FX_REAL; CAM_FY = CAM_FY_REAL
CAM_CX = CAM_CX_REAL; CAM_CY = CAM_CY_REAL
ALT_PIPE = "/tmp/altitude_pipe"
CURRENT_ALTITUDE = 1.5
ALT_TIMESTAMP = 0.0
ALT_LOCK = threading.Lock()
ALT_MAX_AGE = 1.0

MAX_BUCKETS = 5

DROP_LOG_EVERY = 15
H_LOG_EVERY = 15
DROP_WS_EVERY = 2
H_WS_EVERY = 2

THRESH_15_20 = 30
THRESH_20_25 = 55

# v4: 相机支架固定偏移 (世界坐标系, m)
MNT_LX = -0.23; MNT_LY = 0.087   # 左点 (m)
MNT_RX = -0.23; MNT_RY = -0.087  # 右点
CAM_DX = 0.00;  CAM_DY = 0.00    # 云台中心=相机中心 (无偏移)
WORLD_R = 0.038                  # 真实半径 3.8cm

USE_DISPLAY = False

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


# ═══════════════════════════════════════════════════════════════
#  任务模式枚举
# ═══════════════════════════════════════════════════════════════

class MissionMode:
    DROP = "DROP"
    RECON = "RECON"
    RTL = "RTL"
    H_LAND = "H_LAND"
    OTHER = "OTHER"

def parse_mission_state(state):
    state_u = state.upper()
    if "DROP" in state_u: return MissionMode.DROP
    if "RECON" in state_u or "SCAN" in state_u: return MissionMode.RECON
    if "RTL" in state_u: return MissionMode.RTL
    if "H_LAND" in state_u or "LAND" in state_u: return MissionMode.H_LAND
    return MissionMode.OTHER


# ═══════════════════════════════════════════════════════════════
#  TensorRT 投放区检测器 (移植自 test_drop_trt.py)
# ═══════════════════════════════════════════════════════════════

class TRTBucketDetector(object):
    """加载 best.engine, 提供单帧推理。输出解码为 xyxy 像素框(原图坐标)。"""

    def __init__(self, engine_path):
        if not os.path.exists(engine_path):
            raise FileNotFoundError("Engine 不存在: %s" % engine_path)

        # ⚠️ TRT/pycuda 跨线程: autoinit 上下文只在主线程, 推理线程拿不到.
        #     显式建上下文放在最前 —— 引擎/执行上下文/缓冲/stream 全部在其内创建,
        #     推理时 push 复用 (否则 execute_async_v2 报 invalid resource handle)
        #     真机已验证 (drop_zone_jetson_nano_4gb.py / test_bucket_trt.py 同款)
        self.cfx = cuda.Device(0).make_context()

        t0 = time.time()
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(f.read())
        load_time = (time.time() - t0) * 1000

        if self.engine is None:
            raise RuntimeError("Engine 反序列化失败")

        t1 = time.time()
        self.context = self.engine.create_execution_context()
        context_time = (time.time() - t1) * 1000

        # 绑定信息
        self.input_shape = None
        self.output_shape = None
        for i in range(self.engine.num_bindings):
            shape = tuple(self.engine.get_binding_shape(i))
            if self.engine.binding_is_input(i):
                self.input_shape = shape
            else:
                self.output_shape = shape
        if self.input_shape is None or self.output_shape is None:
            raise RuntimeError("未找到输入/输出绑定")

        # 主机/设备内存 (固定 batch=1)
        t2 = time.time()
        self.in_size = int(np.prod(self.input_shape))
        self.out_size = int(np.prod(self.output_shape))
        self.h_input = cuda.pagelocked_empty(self.in_size, dtype=np.float32)
        self.h_output = cuda.pagelocked_empty(self.out_size, dtype=np.float32)
        self.d_input = cuda.mem_alloc(self.h_input.nbytes)
        self.d_output = cuda.mem_alloc(self.h_output.nbytes)
        self.stream = cuda.Stream()
        mem_time = (time.time() - t2) * 1000

        # 归还上下文: 推理时才 push (推理线程 ≠ 加载线程)
        self.cfx.pop()

        print("[TRT] Engine 加载成功")
        print("[TRT]   输入 %s  输出 %s" % (self.input_shape, self.output_shape))
        print("[TRT] ┌─ 加载耗时详情 ─┐")
        print("[TRT] │ 反序列化: %7.1f ms" % load_time)
        print("[TRT] │ 创建上下文: %5.1f ms" % context_time)
        print("[TRT] │ 分配内存: %7.1f ms" % mem_time)
        print("[TRT] └─ 总计:     %7.1f ms" % (load_time + context_time + mem_time))

    # ---- letterbox: 保持宽高比缩放 + 灰边填充到 IMG_SIZE ----
    @staticmethod
    def _letterbox(frame, size=IMG_SIZE):
        h, w = frame.shape[:2]
        r = min(size / float(h), size / float(w))
        nw, nh = int(round(w * r)), int(round(h * r))
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        dx, dy = (size - nw) // 2, (size - nh) // 2
        canvas[dy:dy + nh, dx:dx + nw] = resized
        return canvas, r, dx, dy

    def preprocess(self, frame):
        lb, r, dx, dy = self._letterbox(frame, IMG_SIZE)
        img = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))          # HWC → CHW
        np.copyto(self.h_input, np.ascontiguousarray(img).ravel())
        return r, dx, dy

    def infer_raw(self):
        # ⚠️ 跨线程: 让当前线程的 CUDA 上下文 = 加载时的上下文, 再跑 TRT
        self.cfx.push()
        try:
            cuda.memcpy_htod_async(self.d_input, self.h_input, self.stream)
            ok = self.context.execute_async_v2(
                bindings=[int(self.d_input), int(self.d_output)],
                stream_handle=self.stream.handle)
            if not ok:
                raise RuntimeError("execute_async_v2 返回 False")
            cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
            self.stream.synchronize()
        finally:
            self.cfx.pop()
        return self.h_output.reshape(self.output_shape)

    def release(self):
        """跨线程安全释放本检测器占用的 GPU 资源 (修复 unload 泄漏 / 双 CUDA context)。

        背景: pycuda 的显存释放依赖调用线程的 *当前* CUDA 上下文。直接 `del detector`
        时 (推理线程/预加载线程通常无当前上下文), DeviceAllocation/Stream 的 __del__
        释放会失败 → 显存泄漏 + 旧 context 残留 → 与下一个模型的 CUDA context 并存。
        此方法先把本检测器的上下文 push 到本线程, 再断开 GPU 对象引用, 让它们的
        __del__ 在正确上下文下执行; 之后调用方 del 检测器对象即可。幂等, 重复调用安全。
        """
        self.cfx.push()
        try:
            for name in ('h_input', 'h_output', 'd_input', 'd_output',
                         'stream', 'context', 'engine'):
                try:
                    setattr(self, name, None)
                except Exception:
                    pass
        finally:
            self.cfx.pop()

    def detect(self, frame, conf_thresh=CONF_THRESH, iou_thresh=IOU_THRESH):
        """返回 [(x1,y1,x2,y2,conf,cls), ...] 原图像素坐标。"""
        r, dx, dy = self.preprocess(frame)
        pred = self.infer_raw()[0]                  # (10647, 7)

        # YOLOv5 导出输出已 sigmoid: [cx,cy,w,h] 为 416 输入像素, [4]=obj, [5:]=cls
        obj = pred[:, 4]
        cls_scores = pred[:, 5:5 + NC]
        cls_id = np.argmax(cls_scores, axis=1)
        cls_conf = cls_scores[np.arange(cls_scores.shape[0]), cls_id]
        conf = obj * cls_conf

        keep = conf >= conf_thresh
        if not np.any(keep):
            return []
        pred = pred[keep]; conf = conf[keep]; cls_id = cls_id[keep]

        # xywh(中心) → xyxy, 并 de-letterbox 回原图
        cx, cy, bw, bh = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
        x1 = (cx - bw / 2.0 - dx) / r
        y1 = (cy - bh / 2.0 - dy) / r
        x2 = (cx + bw / 2.0 - dx) / r
        y2 = (cy + bh / 2.0 - dy) / r
        boxes = np.stack([x1, y1, x2, y2], axis=1)

        keep_idx = self._nms(boxes, conf, cls_id, iou_thresh)
        return [(float(boxes[i, 0]), float(boxes[i, 1]),
                 float(boxes[i, 2]), float(boxes[i, 3]),
                 float(conf[i]), int(cls_id[i])) for i in keep_idx]

    @staticmethod
    def _nms(boxes, scores, cls_id, iou_thresh):
        """类内 NMS (纯 numpy)。"""
        final = []
        for c in np.unique(cls_id):
            idx = np.where(cls_id == c)[0]
            b = boxes[idx]; s = scores[idx]
            x1, y1, x2, y2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
            area = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
            order = s.argsort()[::-1]
            while order.size > 0:
                i = order[0]
                final.append(idx[i])
                if order.size == 1:
                    break
                xx1 = np.maximum(x1[i], x1[order[1:]])
                yy1 = np.maximum(y1[i], y1[order[1:]])
                xx2 = np.minimum(x2[i], x2[order[1:]])
                yy2 = np.minimum(y2[i], y2[order[1:]])
                w = np.maximum(0.0, xx2 - xx1)
                h = np.maximum(0.0, yy2 - yy1)
                inter = w * h
                iou = inter / (area[i] + area[order[1:]] - inter + 1e-9)
                order = order[1:][iou <= iou_thresh]
        return final


# ═══════════════════════════════════════════════════════════════
#  桶直径分类 (继承 v4)
# ═══════════════════════════════════════════════════════════════

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


# ═══════════════════════════════════════════════════════════════
#  全局状态 & 管道 & 模型
# ═══════════════════════════════════════════════════════════════

current_mode = MissionMode.OTHER
mode_lock = threading.Lock()
running = True

raw_queue = queue.Queue(maxsize=1)
disp_queue = queue.Queue(maxsize=1)
popup_queue = queue.Queue(maxsize=1)
save_queue = queue.Queue(maxsize=10)
vision_pipe_queue = queue.Queue(maxsize=1)
recon_pipe_queue  = queue.Queue(maxsize=1)
h_pipe_queue      = queue.Queue(maxsize=1)
ws_queue = queue.Queue(maxsize=2)

vision_pipe_fd = None
recon_pipe_fd  = None
h_pipe_fd      = None

save_dir = ""
save_frame_interval = 5
save_frame_count = 0

# v5: 投放区使用 TensorRT 检测器
bucket_detector = None
# v5: H 区也使用 TensorRT 检测器
h_detector = None
bridge_proc = None

# 模型切换互斥 + 异步预加载
model_switch_lock = threading.Lock()
h_detector_loading = False
bucket_detector_loading = False

# H 标识历史缓存
h_last_cx = None
h_last_cy = None
h_last_conf = 0.0
h_last_box = None
h_loss_frames = 0
H_HISTORY_MAX = 90
H_LOST_CONFIRM = 5
H_CONF_LOW = 0.15
H_SEARCH_RADIUS = 200


# ══════════════════════════════════════════════════════════════
#  相机桥 & 模型加载
# ══════════════════════════════════════════════════════════════

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

# ── v5: 投放区 TensorRT 检测器加载 ──
def load_bucket_detector():
    global bucket_detector, bucket_detector_loading
    if bucket_detector is not None: return
    bucket_detector_loading = True
    try:
        with model_switch_lock:
            if bucket_detector is not None: return
            print("加载 TensorRT 桶检测模型...")
            bucket_detector = TRTBucketDetector(BUCKET_ENGINE_PATH)
            # 预热一次 (首帧包含 CUDA kernel 编译)
            dummy = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
            bucket_detector.preprocess(dummy)
            bucket_detector.infer_raw()
            print("TensorRT 桶模型预热完成")
    finally:
        bucket_detector_loading = False


def preload_bucket_detector_async():
    """后台线程预加载 TensorRT 桶检测器。"""
    def _worker():
        try:
            load_bucket_detector()
            with mode_lock:
                m = current_mode
            if m != MissionMode.DROP:
                maybe_unload_bucket_detector()
        except Exception as e:
            print(f"[MODEL] TensorRT 桶检测器预加载失败: {e}")
    threading.Thread(target=_worker, daemon=True).start()


# ── v5: H 区 TensorRT 检测器加载 ──
def load_h_detector():
    global h_detector, h_detector_loading
    if h_detector is not None: return
    h_detector_loading = True
    try:
        with model_switch_lock:
            if h_detector is not None: return
            print("加载 TensorRT H 检测模型...")
            h_detector = TRTBucketDetector(H_ENGINE_PATH)
            # 预热一次
            dummy = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
            h_detector.preprocess(dummy)
            h_detector.infer_raw()
            print("TensorRT H 模型预热完成")
    finally:
        h_detector_loading = False


def maybe_unload_bucket_detector():
    global bucket_detector
    with model_switch_lock:   # ⚠️ 卸载竞态修复: 与推理线程的 detect 互斥, 不在推理进行中释放
        if bucket_detector is not None:
            det = bucket_detector
            bucket_detector = None   # 先摘全局引用, 防并发读到半释放对象
            det.release()            # 在正确 CUDA 上下文下释放 GPU 资源
            del det
            print("已释放 TensorRT 桶检测器显存")

def maybe_unload_h_detector():
    global h_detector
    with model_switch_lock:
        if h_detector is not None:
            det = h_detector
            h_detector = None
            det.release()
            del det
            print("已释放 TensorRT H 检测器显存")


def preload_h_detector_async():
    """后台线程预加载 TensorRT H 检测器。"""
    def _worker():
        try:
            load_h_detector()
        except Exception as e:
            print(f"[MODEL] TensorRT H 检测器预加载失败: {e}")
    threading.Thread(target=_worker, daemon=True).start()


# ══════════════════════════════════════════════════════════════
#  管道初始化 & 线程
# ══════════════════════════════════════════════════════════════

def try_open_write(path):
    try:
        fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
        return os.fdopen(fd, 'wb')
    except OSError:
        return None

def init_pipes():
    global vision_pipe_fd, recon_pipe_fd, h_pipe_fd
    for p in [VISION_PIPE, RECON_PIPE, H_PIPE, CMD_PIPE, ALT_PIPE]:
        if not os.path.exists(p):
            os.mkfifo(p)
    vision_pipe_fd = None
    recon_pipe_fd  = None
    h_pipe_fd      = None
    print("管道文件已创建（等待 C++ 连接）")

def mission_cmd_reader():
    global current_mode
    fd = None
    buf = b""
    while running:
        if fd is None:
            try:
                fd = os.open(CMD_PIPE, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                time.sleep(0.5)
                continue
        try:
            data = os.read(fd, 256)
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
        except OSError:
            try:
                os.close(fd)
            except OSError:
                pass
            fd = None
            time.sleep(0.5)

def altitude_reader():
    global CURRENT_ALTITUDE, ALT_TIMESTAMP
    while running:
        if not os.path.exists(ALT_PIPE):
            time.sleep(0.2)
            continue
        try:
            alt_fd = os.open(ALT_PIPE, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            time.sleep(0.2)
            continue
        buf = b""
        try:
            while running:
                try:
                    data = os.read(alt_fd, 256)
                except BlockingIOError:
                    time.sleep(0.05)
                    continue
                except OSError:
                    break
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
                    break
        finally:
            try:
                os.close(alt_fd)
            except OSError:
                pass
        time.sleep(0.1)

def capture_worker():
    global running
    cap = None
    retry_count = 0
    while running:
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(STREAM_URL)
        if not cap.isOpened():
            retry_count += 1
            if retry_count <= 3 or retry_count % 30 == 0:
                print(f"无法连接 TCP 视频流，等待重试 ({retry_count} 次)...", flush=True)
            time.sleep(2)
            continue
        retry_count = 0
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


# ══════════════════════════════════════════════════════════════
#  推理主循环 & 三个任务处理函数
# ══════════════════════════════════════════════════════════════

def inference_worker():
    global running, current_mode
    global h_loss_frames, h_last_cx, h_last_cy, h_last_conf, h_last_box
    print("推理线程已启动")
    frame_count = 0
    prev_mode = None

    while running:
        if raw_queue.empty():
            time.sleep(0.001); continue
        try:
            frame = raw_queue.get()
            frame_count += 1
            with mode_lock: mode = current_mode

            if mode != prev_mode:
                if mode in (MissionMode.RTL, MissionMode.H_LAND):
                    if bucket_detector is not None: maybe_unload_bucket_detector()
                    if h_detector is None: preload_h_detector_async()
                    if prev_mode not in (MissionMode.RTL, MissionMode.H_LAND):
                        h_loss_frames = 0
                        h_last_cx = h_last_cy = None
                        h_last_conf = 0.0
                        h_last_box = None
                    print("[MODEL] 切到返航/降落: 释放 TensorRT 桶 → 异步加载 TensorRT H (单 CUDA context)")
                elif mode == MissionMode.DROP:
                    if h_detector is not None: maybe_unload_h_detector()
                    if bucket_detector is None:
                        preload_bucket_detector_async()
                    print("[MODEL] 切到投放: 释放 TensorRT H (若有) + 预加载 TensorRT 桶")
                elif mode == MissionMode.RECON:
                    print("[MODEL] 切到侦察: 仅视频流, 桶模型保持加载 (单 CUDA context)")
                prev_mode = mode

            if mode == MissionMode.RECON:
                process_recon(frame, frame_count)
            elif mode == MissionMode.DROP:
                process_drop(frame, frame_count)
            elif mode in (MissionMode.RTL, MissionMode.H_LAND):
                process_h(frame, frame_count)
            else:
                push_display(frame)
                push_ws(frame, None)
        except Exception as e:
            print(f"[推理异常] {e}", flush=True)
    print("推理线程退出")


# ── v5: 投放区 (TensorRT) ──

def process_drop(frame, frame_count):
    global bucket_detector
    det = bucket_detector
    if det is None:
        maybe_unload_h_detector()   # 单 CUDA context: H 仅返航期间加载, 正常流程此处为 None (幂等)
        load_bucket_detector()
        det = bucket_detector
    if det is None:
        # 模型未就绪/加载失败: 本帧只转发画面, 不推理不写管道
        result_frame = frame.copy()
        push_save(result_frame, "DROP")
        push_display(result_frame)
        if frame_count % DROP_WS_EVERY == 0:
            push_ws(result_frame, MissionMode.DROP)
        return

    # v5: 使用 TensorRT 检测器推理
    # ⚠️ 卸载竞态修复: 一次取引用 + 锁内推理, 后台预加载线程无法在推理中 release
    with model_switch_lock:
        detections = det.detect(frame)
    target_dets = [d for d in detections if d[5] == 0]  # class 0 = bucket

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
        if frame_count % DROP_LOG_EVERY == 0:
            print("[DROP] " + ", ".join(
                f"bucket{bid}: ({cx:.1f}, {cy:.1f})"
                for bid, cx, cy in bucket_list))

    while not vision_pipe_queue.empty():
        try: vision_pipe_queue.get_nowait()
        except queue.Empty: break
    vision_pipe_queue.put((len(bucket_list), bucket_list))

    push_save(result_frame, "DROP")
    push_display(result_frame)
    if frame_count % DROP_WS_EVERY == 0:
        push_ws(result_frame, MissionMode.DROP)


# ── 侦察区 (继承 v4: 仅视频流) ──

def process_recon(frame, frame_count):
    """侦察区只转发最新视频帧；不加载模型、不运行推理、不写识别坐标。"""
    if frame_count % 2 == 0:
        push_ws(frame, MissionMode.RECON)
    push_display(frame)


# ── v5: H 降落 (TensorRT) ──

def process_h(frame, frame_count):
    global h_detector, h_last_cx, h_last_cy, h_last_conf, h_last_box, h_loss_frames
    det = h_detector
    if det is None:
        if h_detector_loading:
            h_loss_frames += 1
            h_img, w_img = frame.shape[:2]
            result_frame = frame.copy()
            if h_last_cx is not None and h_loss_frames < H_LOST_CONFIRM:
                msg = f"{h_last_cx:.2f},{h_last_cy:.2f}"
                cv2.circle(result_frame, (int(h_last_cx), int(h_last_cy)),
                           15, (0, 165, 255), 2)
                cv2.putText(result_frame, "H:LOADING (hold)",
                            (int(h_last_cx) + 20, int(h_last_cy) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
            else:
                msg = "None"
                cv2.putText(result_frame, "H:LOADING", (8, h_img - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
            print(f"[H] {msg} (model loading)")
            while not h_pipe_queue.empty():
                try: h_pipe_queue.get_nowait()
                except queue.Empty: break
            h_pipe_queue.put(msg)
            push_save(result_frame, "H")
            push_display(result_frame)
            if frame_count % H_WS_EVERY == 0:
                push_ws(result_frame)
            return
        maybe_unload_bucket_detector()
        load_h_detector()
        det = h_detector
    if det is None:
        # H 模型未就绪/加载失败: 按丢失处理, 保持转发画面
        h_loss_frames += 1
        h_img, w_img = frame.shape[:2]
        result_frame = frame.copy()
        msg = "None"
        cv2.putText(result_frame, "H:NONE", (8, h_img - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
        while not h_pipe_queue.empty():
            try: h_pipe_queue.get_nowait()
            except queue.Empty: break
        h_pipe_queue.put(msg)
        push_save(result_frame, "H")
        push_display(result_frame)
        if frame_count % H_WS_EVERY == 0:
            push_ws(result_frame)
        return

    # v5: 使用 TensorRT 检测器推理
    # ⚠️ 卸载竞态修复: 一次取引用 + 锁内推理, 后台预加载线程无法在推理中 release
    with model_switch_lock:
        detections = det.detect(frame, conf_thresh=H_CONF_THRESH)
    h_dets = [d for d in detections if d[5] == 0]  # class 0 = H

    msg = "None"
    result_frame = frame.copy()
    h_img, w_img = frame.shape[:2]
    best_box = None

    if len(h_dets) > 0:
        # 选置信度最高的
        top = max(h_dets, key=lambda d: d[4])
        x1, y1, x2, y2, conf, cls = top
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        best_box = (x1, y1, x2, y2, conf, cx, cy)
        h_last_box = (x2 - x1, y2 - y1)
        h_last_cx, h_last_cy = cx, cy
        h_last_conf = conf
        h_loss_frames = 0
        msg = f"{cx:.2f},{cy:.2f}"

    else:
        h_loss_frames += 1
        # v4 扩大搜索逻辑: 丢失窗口内用低阈值重检
        if h_loss_frames <= H_LOST_CONFIRM and h_last_cx is not None:
            with model_switch_lock:
                dets2 = det.detect(frame, conf_thresh=H_CONF_LOW)
            h2 = [d for d in dets2 if d[5] == 0]
            if len(h2) > 0:
                top2 = max(h2, key=lambda d: d[4])
                x1, y1, x2, y2, conf2, cls2 = top2
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                # 扩大搜索: 位置在上次检测附近才接受
                radius = H_SEARCH_RADIUS
                if h_last_box is not None:
                    radius = max(radius, h_last_box[0], h_last_box[1])
                if (abs(cx - h_last_cx) <= radius and
                        abs(cy - h_last_cy) <= radius):
                    best_box = (x1, y1, x2, y2, conf2, cx, cy)
                    h_last_box = (x2 - x1, y2 - y1)
                    h_last_cx, h_last_cy = cx, cy
                    h_last_conf = conf2
                    h_loss_frames = 0
                    msg = f"{cx:.2f},{cy:.2f}"

        # 三帧滤波: 连续 H_LOST_CONFIRM 帧未检出才判真丢失
        if msg == "None" and h_last_cx is not None:
            if h_loss_frames < H_LOST_CONFIRM:
                msg = f"{h_last_cx:.2f},{h_last_cy:.2f}"
            else:
                msg = "None"
                if h_loss_frames > H_HISTORY_MAX:
                    h_last_cx = h_last_cy = None
                    h_last_conf = 0.0
                    h_last_box = None
                    h_loss_frames = 0
        else:
            h_loss_frames = 0

    if best_box is not None:
        _, _, _, _, conf, cx_draw, cy_draw = best_box
        cv2.rectangle(result_frame,
                      (int(best_box[0]), int(best_box[1])),
                      (int(best_box[2]), int(best_box[3])),
                      (0, 0, 255), 2)
        cv2.circle(result_frame, (int(cx_draw), int(cy_draw)), 5,
                   (0, 255, 255), -1)
        cv2.putText(result_frame, f"H ({cx_draw:.0f},{cy_draw:.0f})",
                    (int(best_box[0]), int(best_box[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    elif h_last_cx is not None and h_loss_frames <= H_HISTORY_MAX:
        cx_i, cy_i = int(h_last_cx), int(h_last_cy)
        cv2.circle(result_frame, (cx_i, cy_i), 15, (0, 165, 255), 2)
        cv2.circle(result_frame, (cx_i, cy_i), 3, (0, 165, 255), -1)
        cv2.putText(result_frame, f"H hist {h_loss_frames}/{H_HISTORY_MAX}",
                    (cx_i + 20, cy_i - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

    cv2.line(result_frame, (w_img // 2, 0), (w_img // 2, h_img),
             (100, 100, 100), 1)
    cv2.line(result_frame, (0, h_img // 2), (w_img, h_img // 2),
             (100, 100, 100), 1)

    hud_y = h_img - 8
    if best_box is not None:
        cv2.putText(result_frame, "H:LIVE", (8, hud_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    elif h_loss_frames > 0:
        cv2.putText(result_frame, f"H:HOLD {h_loss_frames}/{H_HISTORY_MAX}",
                    (8, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 165, 255), 1)
    else:
        cv2.putText(result_frame, "H:LOST", (8, hud_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    tag = ""
    if best_box is not None:
        tag = f" conf={best_box[4]:.2f}"
    elif msg != "None" and h_loss_frames > 0:
        tag = " (hist)"
    if frame_count % H_LOG_EVERY == 0 or msg == "None":
        print(f"[H] {msg}{tag}")

    while not h_pipe_queue.empty():
        try: h_pipe_queue.get_nowait()
        except queue.Empty: break
    h_pipe_queue.put(msg)

    push_save(result_frame, "H")
    push_display(result_frame)
    if frame_count % H_WS_EVERY == 0:
        push_ws(result_frame, MissionMode.H_LAND)


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
    save_queue.put((frame, mode_name))

def push_ws(frame, mode=None):
    """v4: 覆盖式推送到 WebSocket 队列，带模式信息用于选择压缩质量"""
    while not ws_queue.empty():
        try:
            ws_queue.get_nowait()
        except queue.Empty:
            break
    ws_queue.put((frame, mode))


# ══════════════════════════════════════════════════════════════
#  管道发送、显示、保存、WebSocket 线程
# ══════════════════════════════════════════════════════════════

def ws_server_thread():
    """v3.8: WebSocket 服务线程 (独立事件循环运行异步服务器)"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(ws_main())
    except Exception as e:
        print(f"[WS] WebSocket 服务异常: {e}")
    finally:
        loop.close()


async def ws_handler(websocket, path=None):
    """v4: 每个客户端独立 handler —— 慢/假死客户端只拖自己, 不拖死整条推流。
    JPEG 编码放线程池执行, 避免阻塞事件循环 (回传视频卡死的根因修复, 与 v4 同款)。

    ⚠️ 无 send 超时 (与 v4 一致): websockets 库的 send 本身是异步非阻塞,
    慢客户端会自动进 TCP 发送缓冲队列, 不会卡死事件循环; 如果客户端真的假死,
    底层 TCP keep-alive 会在几十秒后断开连接 (操作系统级); ws_queue maxsize=2
    保证推流端不积压 (老帧被新帧覆盖), 所以不需要应用层超时强制断连。"""
    print(f"[WS] 客户端已连接: {websocket.remote_address}")
    loop = asyncio.get_event_loop()
    try:
        while running:
            try:
                data = ws_queue.get_nowait()
                if isinstance(data, tuple):
                    frame, mode = data
                else:
                    frame, mode = data, None
            except queue.Empty:
                await asyncio.sleep(0.01)
                continue

            # 侦察区高质量 (需识别标签), 其他区标准质量
            quality = RECON_WS_JPEG_QUALITY if mode == MissionMode.RECON else WS_JPEG_QUALITY
            # JPEG 编码放到线程池, 避免卡住事件循环 (v4 同款)
            _, jpeg = await loop.run_in_executor(
                None, cv2.imencode, '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
            )
            # v4 同款: 直接 send, 无应用层超时 (依赖 TCP 层 + ws_queue maxsize=2)
            await websocket.send(jpeg.tobytes())
    except websockets.exceptions.ConnectionClosed:
        print(f"[WS] 客户端已断开: {websocket.remote_address}")
    except Exception as e:
        print(f"[WS] 连接异常: {e}")


async def ws_main():
    """v4: WebSocket 服务主循环 (每客户端独立 handler, 永久运行)"""
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        print(f"[WS] WebSocket 服务器已启动 ws://{WS_HOST}:{WS_PORT}")
        await asyncio.Future()  # 永久运行


def pipe_sender_worker():
    """管道发送线程 (投放/侦察/H 三个管道)"""
    global vision_pipe_fd, recon_pipe_fd, h_pipe_fd
    print("管道发送线程已启动")
    vision_pipe_drops = 0

    while running:
        # ── 投放区管道 ──
        if not vision_pipe_queue.empty():
            try:
                count, bucket_list = vision_pipe_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                if vision_pipe_fd is None:
                    vision_pipe_fd = try_open_write(VISION_PIPE)
                if vision_pipe_fd:
                    try:
                        vision_pipe_fd.write(struct.pack('B', count))
                        for bid, cx, cy in bucket_list:
                            vision_pipe_fd.write(struct.pack('B', bid))
                            vision_pipe_fd.write(struct.pack('ff', float(cx), float(cy)))
                        vision_pipe_fd.flush()
                    except (OSError, BrokenPipeError):
                        vision_pipe_drops += 1
                        if vision_pipe_drops == 1 or vision_pipe_drops % 50 == 0:
                            print(f"[PIPE] 投放管道写入失败 {vision_pipe_drops} 次", flush=True)
                        try: vision_pipe_fd.close()
                        except: pass
                        vision_pipe_fd = None

        # ── 侦察区管道 (v4: 不再写入识别结果, 保留管道确保协议兼容) ──
        if not recon_pipe_queue.empty():
            try:
                msg = recon_pipe_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                if recon_pipe_fd is None:
                    recon_pipe_fd = try_open_write(RECON_PIPE)
                if recon_pipe_fd:
                    try:
                        recon_pipe_fd.write((msg + "\n").encode('utf-8'))
                        recon_pipe_fd.flush()
                    except (OSError, BrokenPipeError):
                        try: recon_pipe_fd.close()
                        except: pass
                        recon_pipe_fd = None

        # ── H 降落管道 ──
        if not h_pipe_queue.empty():
            try:
                msg = h_pipe_queue.get_nowait()
            except queue.Empty:
                pass
            else:
                if h_pipe_fd is None:
                    h_pipe_fd = try_open_write(H_PIPE)
                if h_pipe_fd:
                    try:
                        h_pipe_fd.write((msg + "\n").encode('utf-8'))
                        h_pipe_fd.flush()
                    except (OSError, BrokenPipeError):
                        try: h_pipe_fd.close()
                        except: pass
                        h_pipe_fd = None

        time.sleep(0.005)
    print("管道发送线程退出")


def display_worker():
    """显示线程"""
    print("显示线程已启动")
    cv2.namedWindow("Vision", cv2.WINDOW_NORMAL)

    while running:
        if disp_queue.empty():
            time.sleep(0.01)
            continue
        try:
            frame = disp_queue.get_nowait()
            cv2.imshow("Vision", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except queue.Empty:
            pass

    cv2.destroyAllWindows()
    print("显示线程退出")


def save_worker():
    """图片保存线程"""
    global save_dir
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = f"/tmp/vision_frames_{timestamp}"
    os.makedirs(save_dir, exist_ok=True)
    print(f"图片保存目录: {save_dir}")

    while running:
        if save_queue.empty():
            time.sleep(0.1)
            continue
        try:
            frame, mode_name = save_queue.get(timeout=1)
            ts = datetime.now().strftime("%H%M%S_%f")[:-3]
            filename = f"{mode_name}_{ts}.jpg"
            path = os.path.join(save_dir, filename)
            cv2.imwrite(path, frame)
        except queue.Empty:
            pass
        except Exception as e:
            print(f"[SAVE] 保存失败: {e}")
    print("图片保存线程退出")


# ══════════════════════════════════════════════════════════════
#  主程序
# ══════════════════════════════════════════════════════════════

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

    print("预加载 TensorRT 桶检测模型...")
    load_bucket_detector()

    init_pipes()

    threads = []
    t_cmd = threading.Thread(target=mission_cmd_reader, daemon=True); t_cmd.start(); threads.append(t_cmd)
    t_alt = threading.Thread(target=altitude_reader, daemon=True); t_alt.start(); threads.append(t_alt)
    t_cap = threading.Thread(target=capture_worker, daemon=True); t_cap.start(); threads.append(t_cap)
    t_inf = threading.Thread(target=inference_worker, daemon=True); t_inf.start(); threads.append(t_inf)
    t_pipe = threading.Thread(target=pipe_sender_worker, daemon=True); t_pipe.start(); threads.append(t_pipe)
    t_ws = threading.Thread(target=ws_server_thread, daemon=True); t_ws.start(); threads.append(t_ws)
    if USE_DISPLAY:
        t_disp = threading.Thread(target=display_worker, daemon=True); t_disp.start(); threads.append(t_disp)

    t_save = threading.Thread(target=save_worker, daemon=True); t_save.start(); threads.append(t_save)

    print("=" * 50)
    print("  统一视觉检测系统 v5 (TensorRT 投放区)")
    print(f"  模式: {'仿真' if SIM_MODE else '真机'}")
    print("  投放区 (TensorRT) → 侦察区(视频流) → H降落 (TensorRT)")
    print("  读取 /tmp/mission_cmd 自动切换")
    print("  Ctrl+C 退出")
    print("=" * 50)

    try:
        while running:
            with mode_lock: m = current_mode
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
        maybe_unload_bucket_detector()
        maybe_unload_h_detector()
        print("程序已退出")


if __name__ == "__main__":
    main()