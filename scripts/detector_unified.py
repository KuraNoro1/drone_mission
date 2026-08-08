#!/usr/bin/env python3
"""
统一视觉检测 v4 — 延迟优先：侦察区仅 WebSocket 视频，投放/H 实时坐标
================================================================
v4 核心改动:
  - 侦察区完全停止 YOLO/HSV/跟踪推理，只覆盖式推送最新 WebSocket 视频帧
  - 投放区与 H 区继续实时输出像素中心坐标，所有队列只保留最新数据
  - 延迟优先：投放隔帧推理、禁用梯度、降低同步日志频率、减少图像复制
  - 保留 v3.9 H 丢失窗口逐帧低阈值重检与自适应搜索半径
  - C++ 管道路径和数据协议保持不变；recon_pipe 保留但侦察区不再产生识别结果

历史版本说明:
v2 → v3 改动 (侦察区):
  - ✅ 定点突发跟踪器 BucketTrack: 悬停帧按中心距离关联成稳定ID,
        颜色/空桶连续确认 RECON_CONFIRM_FRAMES 帧才切换输出 (滞回/防抖)
  - ✅ unknown vs empty 协议区分: 检测失败→unknown, 椭圆内确认无标签→empty
  - ✅ K-means 改在 Lab 空间聚类 (发灰后红/橙不再糊在一起)
  - ✅ 白色参考 = 椭圆内白色掩码像素均值 (不够40px再回退到最白簇)
  - ✅ 簇内高饱和像素色相"圆均值" (避免红色0/180环绕平均变绿)
  - ✅ 白环双阈值: 绝对阈值找不到环时用更低V下限重试 (阴影环境减少NO OPENING)
  - ✅ 高度带门控: 粗锁 3.5m 工况 (RECON_ALT_MIN/MAX), 超出直接跳过
  v3.1:
  - ✅ 定点判定=帧差悬停门控: 移动→跳过检测, 移动→悬停=到达新定点→清空轨道
  - ✅ RECON_FOLD_UNKNOWN_TO_EMPTY: unknown折叠为empty, 与v2协议完全一致
  v3.2 (Jetson 4GB 省算力 + 协议还原):
  - ✅ 隔帧重检测: 悬停门控每帧跑 (~2ms), YOLO+轮廓+K-means 每
        RECON_INFER_EVERY 帧才跑一次; 中间帧复用上次稳定结果照常输出
        (定点悬停场景桶几乎不动, 信息量无损失)
  - ✅ recon_pipe 多色协议还原: 轨道状态改为完整颜色 dict
        (BucketTrack.stable_colors), 输出与原 unified 一致
        "1:red:70.0,blue:20.0;2:empty", 单色/多色都兼容
  - 投放区 / H降落 / 线程 / 管道 / 模型管理: 不变
  v3.3 (H 降落区丢失问题修复):
  - ✅ 模型切换反序: process_h 先卸载桶模型释放显存, 再加载 H 模型
        (原顺序: 加载H时桶模型仍占显存 → 慢+抖动 → 模型就绪时飞机已飞过H,
         造成"只检测到一次就丢失")
  - ✅ H 丢失滤波: 连续 H_LOST_CONFIRM(5, v3.6 由3调大) 帧未检出才判真丢失,
        单帧抖动/偶发误检不误发 None → 控制端不误返航
        (v3.6 修复: 真丢失后管道发 None, 不再补位旧坐标 — C++ 端才能进 lost-tracking)
  - ✅ 丢失后扩大搜索: 首帧未检出用低阈值 H_CONF_LOW(0.15) 重检,
        位置在上次检测 H_SEARCH_RADIUS(200px) 邻域内才接受 (防远距离误检)
  - ✅ H 阈值 0.5→0.35: 降落视角 H 小/畸变/运动模糊 conf 偏低, 多捞回边缘帧
  - ✅ RTL 提前切换模型: 模式一切到 RTL/H_LAND → 立即释放桶 + 后台预加载 H
        (返航飞行途中 H 模型就绪, 到降落区零等待; 不再等 H_LAND 第一帧才加载)
        对称: 切 DROP/RECON → 释放 H + 后台预加载桶
        用 model_switch_lock 互斥, 防后台加载与首帧同步加载竞态
  - 投放区 / 侦察区 / 线程 / 管道协议: 不变
  v3.4 (侦察区检测策略重做 — 修复正上方全 None):
  - ✅ 删除"YOLO/白环 为主"的检测链: 正上方俯视 YOLO 认不出桶, 白环兜底又
        要求"白色圆形大块", 桶顶偏色/阴影 → 整帧 None (v2/v3 均如此)
  - ✅ 恢复原 scripts/detector_recon.py 的全帧 HSV 颜色搜索为主检测:
        find_color_buckets 直接找彩色标签块 (不依赖桶形状), 正上方也能读到标签;
        + YOLO 候选 IoU 合并 (重叠保 YOLO 框, HSV 补漏);
        每个候选 → analyze_color_proportions HSV 占比判色 (无标签→empty)
  - ✅ 删除 v3.1 运动门控 + 高度带门控 (全 None 的另一个来源):
        process_recon 每帧直接 检测→跟踪→输出, 不再被帧差/高度硬阻断
  - ✅ 保留 v3 定点跟踪 (BucketTrack 滞回/丢帧保留/稳定ID) + v3.2 隔帧调度
  - ✅ 已删除: find_bucket_opening / classify_label_color (Lab K-means) /
        find_white_circles_fullframe / _frame_motion 及配套参数
  v3.4 (草地测试临时):
  - ✅ RECON_HSV_SKIP_COLORS={'green'}: 草地测试禁用绿色标签 (草地=绿色会误判)
  - ⚠️ 比赛场地是水泥地, 会放绿色桶标识 → 赛前必须把 RECON_HSV_SKIP_COLORS 恢复为 set()
  - ✅ 低饱和度回退 (3.5m 标签发灰): COLOR_RANGES S 20→12 / V 20→18;
        正常轮(全帧/ROI)判不出时, 以 S≥8 / V≥40 (全帧面积≥150) 重扫重判
  - ✅ 红白标识/白壁污染修复 (v3.5 吸收 v2 中心掩码思路):
        判色只统计 ROI 中心圆 (RECON_HSV_CENTER_RATIO=0.8), 桶壁/外圈不进统计区
        → 不再稀释 pct、不再反光染成标签色; 分母=中心圆面积不被灌水;
        白色去主导: 白色最高但有其他显著色时, 提第一个非白非黑显著色为主色;
        RECON_HSV_COUNT_WHITE=True: 白色参与判色 (红白标识 → red:xx,white:xx);
        RECON_HSV_COUNT_BLACK=True: 黑色参与判色 (腐蚀品/刺激性需要;
        红底易燃的黑字也计入黑色, 但红+黑分支仍判易燃, 不影响);
        RECON_HSV_NEVER_COUNT={'gray'}: 灰永不参与; 纯白中心(无其他色)=空桶
        (白色绝不参与全帧搜索, 防白桶壁/地面误报)
  v3.7 (侦察区实测修复 — 放2桶报13桶 / 自燃漏检 / 误报遇湿易燃):
  - ✅ 标签碎片聚类: find_color_buckets 每色色块先按空间邻近聚类再外扩,
        反光/抗锯齿拆开的同标签碎片不再各生成一个候选框 (原合并 IoU 救不回)
  - ✅ 逐色低饱和回退: 某颜色正常S扫不到时单独 S≥8/V≥40 重扫,
        不再要求"整帧全空"才回退 → 黄标够亮时红标也能被扫到 (自燃物品不再漏检)
  - ✅ 轨道上限 MAX_BUCKETS: 跨帧碎片框累积不再让输出膨胀到十几个
  - ✅ 附带修复: 自燃物品红底没检到时, 其白半边反射天空蓝会被误判为蓝色
        (遇湿易燃物品) — 红底检到后该桶正确锚定在红上, 蓝误报消失
  v3.7.1 (正上方悬停自燃 → 生物危害回归修复):
  - ✅ 白色双计防护收窄: 只排除"红"(红底抗锯齿污染白色), 不再排除黄/蓝 —
        v3.6 排除黄/蓝把自燃/放射性的白半边(暖光反射淡黄/蓝, S∈[12,30])
        剔除出白色 → 白半边只算成黄 → 纯黄 → 生物危害 (自燃上方实测)
  - ✅ identify_label_name: 红在而白/黑均不在 → 返回易燃,
        堵住"红+反射黄 → 单色黄 → 生物危害"的回落漏洞
  - ✅ [RECON SUMMARY] 附带原始颜色占比 (诊断: 判断"生物危害"是纯黄还是红+白)
  v3.7.2 (红色整帧识别不到修复):
  - ✅ 低饱和回退 V 死区修复: 回退轮 V 下限 40→18 — 原设计只放宽 S 却抬高 V,
        制造死区 (S∈[8,12] 且 V∈[18,40] 正常轮与回退轮都漏); 红颜料发灰又偏暗,
        正好掉进死区 → 红色整帧识别不到 (自燃物品上方实测返回生物危害)
  - ✅ 回退 S 8→6 (现场实测红色"更灰一点点", S∈[6,8] 纳入回退)
  - ✅ 红色探针 _probe_red: 每 20 帧打印全帧红像素 S≥12/S≥6 两档计数,
        直接定位红色是发灰(S)、发暗(V) 还是色相偏

架构前提 (与飞控约定):
  - C++ 只在到达侦察定点悬停稳定后才发 RECON_SCAN → 收到即信任"已在定点"
  - 侦察高度固定 3.5m, 参数按该工况调优, 只留高度带粗保护
  - 飞机在定点悬停, 帧间桶中心几乎不动 → 中心距离关联足够

任务模式:
  - 投放区: YOLO 桶检测 → /tmp/vision_pipe (二进制)
  - 侦察区: 不做识别，仅通过 WebSocket 覆盖式发送最新原始视频帧
  - 返航降落: YOLO H 检测 → /tmp/h_pipe (文本)
  - 读取 /tmp/mission_cmd 自动切换 (C++ 状态通知)
  - 仿真模式自动启动 gz_gst_bridge 相机桥

用法:
  python3 detector_unified_v4.py [--sim] [--display]
================================================================
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
from datetime import datetime
import asyncio
import websockets

# ─────────────────────────── 配置 ───────────────────────────
_USER = os.environ.get('SUDO_USER', 'hy')
BUCKET_MODEL_PATH = os.path.expanduser(f"~{_USER}/drone_mission/yolo/best.pt")
H_MODEL_PATH      = os.path.expanduser(f"~{_USER}/drone_mission/yolo/best_H.pt")
YOLOV5_REPO       = os.path.expanduser(f"~{_USER}/yolov5")

VISION_PIPE = "/tmp/vision_pipe"
RECON_PIPE  = "/tmp/recon_pipe"
H_PIPE      = "/tmp/h_pipe"
CMD_PIPE    = "/tmp/mission_cmd"

# ── WebSocket 推送配置 (v3.8: 侦察区实时标注帧推送) ──
WS_HOST = "0.0.0.0"
WS_PORT = 8765
# v4: 侦察区高质量（需识别标签颜色细节），投放/降落区标准质量（只需看到目标位置）
RECON_WS_JPEG_QUALITY = 85  # 侦察区高质量 (单帧 ~85KB)
WS_JPEG_QUALITY = 60         # 投放/降落/启动区标准质量 (单帧 ~40KB)

STREAM_URL = "tcp://127.0.0.1:5000"
IMG_SIZE = 416
CONF_THRESH = 0.6
# v3.3: H 阈值从 0.5 降到 0.35 — 降落视角 H 小/畸变/运动模糊 conf 偏低,
#       降阈值多捞回边缘帧; 误检由"连续3帧确认"兜底抑制
H_CONF_THRESH = 0.35

SIM_MODE = False   # 由启动脚本传入 --sim
GST_BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "gz_gst_bridge.py")

# ── 真机 IMX219 — 优先加载棋盘格标定 ──
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

# v4: 投放区和 H 区每个最新帧都推理；只限制 WebSocket 编码频率。
DROP_LOG_EVERY = 15
H_LOG_EVERY = 15
# 侦察区仍按每个最新相机帧推送；检测区约每2帧推送一帧画面。
DROP_WS_EVERY = 2
H_WS_EVERY = 2


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


# ═══════════════════════════════════════════════════════════════
#  侦察区 v6: 全帧 HSV 颜色搜索为主 + 定点突发跟踪 (原 scripts 方案)
# ═══════════════════════════════════════════════════════════════

# v4: 侦察区不再调用以下历史实现；保留文本仅供现场参数追溯。
'''
def _cluster_rects(rects, ratio=RECON_HSV_EXPAND_RATIO,
                   extra=RECON_HSV_CLUSTER_RATIO):
    """同一颜色、空间相邻的色块合并成一个候选 (同标签碎片去重).

    v3.7 修复: 标签碎片(反光/抗锯齿拆开)间距常超过 2.5×色块,
    原逻辑每个碎片独立外扩 → 同一标签被拆成多个框 (侦察区"放2个桶报13个")。
    判据: 两色块的候选框按 extra 倍率外扩后重叠 → 视为同一标签, 先合并原 bbox。
    """
    boxes = [list(r) for r in rects]  # [x, y, w, h]
    changed = True
    while changed:
        changed = False
        i = 0
        while i < len(boxes):
            j = i + 1
            while j < len(boxes):
                a, b = boxes[i], boxes[j]
                pa = max(a[2], a[3]) * ratio * extra / 2
                pb = max(b[2], b[3]) * ratio * extra / 2
                if (a[0] - pa < b[0] + b[2] + pb and
                        b[0] - pb < a[0] + a[2] + pa and
                        a[1] - pa < b[1] + b[3] + pb and
                        b[1] - pb < a[1] + a[3] + pa):
                    x1 = min(a[0], b[0]); y1 = min(a[1], b[1])
                    x2 = max(a[0] + a[2], b[0] + b[2])
                    y2 = max(a[1] + a[3], b[1] + b[3])
                    boxes[i] = [x1, y1, x2 - x1, y2 - y1]
                    boxes.pop(j)
                    changed = True
                else:
                    j += 1
            i += 1
    return [(x, y, w, h) for x, y, w, h in boxes]


def _scan_color(hsv, color_name, min_area, s_min, v_min):
    """对单色做 inRange + 形态学, 返回通过面积阈值的连通域 bbox 列表."""
    lo, hi = _color_range_lo(color_name, s_min, v_min)
    mask = cv2.inRange(hsv, lo, hi)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)  # v3.7.3 草地测试: 1→2 打碎草地细纹理
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        blobs.append((x, y, w, h))
    return blobs


def find_color_buckets(frame, min_area=RECON_HSV_MIN_AREA,
                       s_min=None, v_min=None,
                       per_color_fallback=False):
    """
    全帧 HSV 颜色搜索 — 侦察区正上方主检测。

    直接对 红/橙/黄/绿/蓝/紫 六色做 HSV inRange 找彩色标签连通域;
    不依赖"桶的形状", 正上方俯视 YOLO 认不出桶时也能直接读到标签色
    (原 scripts/detector_recon.py 方案)。

    每色独立搜索 → 碎片按空间邻近聚类 (_cluster_rects) → 外扩成候选框。
    (修复: 反光/抗锯齿把同一标签拆成多个框 → 放2个桶却报13个)

    per_color_fallback=True: 某颜色正常 S 扫不到时, 用低饱和 (S≥8/V≥40)
    对该颜色单独重扫 (3.5m 标签发灰)。旧逻辑只在"整帧全空"才回退,
    会漏掉"黄色够亮、红色发灰"的混合场景 → 自燃物品(红)整帧漏检。

    返回: [(x1,y1,x2,y2), ...]
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    boxes = []
    for color_name in COLOR_RANGES:
        if color_name in RECON_HSV_IGNORE_COLORS:
            continue
        if color_name in RECON_HSV_SKIP_COLORS:
            continue
        blobs = _scan_color(hsv, color_name, min_area, s_min, v_min)
        if not blobs and per_color_fallback:
            blobs = _scan_color(hsv, color_name, RECON_HSV_FALLBACK_MIN_AREA,
                                RECON_HSV_FALLBACK_S, RECON_HSV_FALLBACK_V_MIN)
        for (x, y, w, h) in _cluster_rects(blobs):
            center_x = x + w // 2
            center_y = y + h // 2
            bucket_size = int(max(w, h) * RECON_HSV_EXPAND_RATIO)
            bx = max(0, center_x - bucket_size // 2)
            by = max(0, center_y - bucket_size // 2)
            bw = min(frame.shape[1] - bx, bucket_size)
            bh = min(frame.shape[0] - by, bucket_size)
            if bw > 30 and bh > 30 and bw < frame.shape[1] * 0.8:
                boxes.append((bx, by, bx + bw, by + bh))
    return boxes


def _overlap_ratio(box, merged, iou_thresh):
    """box 与 merged 中任一框的 IoU-min 重叠是否超过阈值"""
    x1, y1, x2, y2 = box
    for m in merged:
        mx1, my1, mx2, my2, _ = m
        ix1 = max(x1, mx1); iy1 = max(y1, my1)
        ix2 = min(x2, mx2); iy2 = min(y2, my2)
        if ix2 > ix1 and iy2 > iy1:
            inter = (ix2 - ix1) * (iy2 - iy1)
            area_a = (x2 - x1) * (y2 - y1)
            area_b = (mx2 - mx1) * (my2 - my1)
            if inter / min(area_a, area_b) > iou_thresh:
                return True
    return False


def merge_yolo_hsv(yolo_boxes, color_boxes, iou_thresh=RECON_HSV_IOU_THRESH):
    """
    合并 YOLO 桶框 + HSV 色块框 (IoU-min 去重)。

    重叠时保留 YOLO 框 (完整桶顶, ROI 更准); 非重叠的 HSV 色块框补漏
    (YOLO 认不出桶时也能找到)。最终按 x 排序, 与旧协议 ID 一致。
    yolo_boxes: [(x1,y1,x2,y2,conf), ...]
    color_boxes: [(x1,y1,x2,y2), ...]
    返回: [(x1,y1,x2,y2,conf), ...]
    """
    merged = []
    # YOLO 优先: 先加 YOLO 框, 再加不重叠的 HSV 框
    for box in yolo_boxes:
        x1, y1, x2, y2, conf = box
        if _overlap_ratio((x1, y1, x2, y2), merged, iou_thresh):
            continue
        merged.append((x1, y1, x2, y2, conf))
    for (bx, by, bx2, by2) in color_boxes:
        if _overlap_ratio((bx, by, bx2, by2), merged, iou_thresh):
            continue
        merged.append((bx, by, bx2, by2, 0.0))
    merged.sort(key=lambda b: b[0])
    return merged


def _color_range_lo(color_name, s_min=None, v_min=None):
    """
    COLOR_RANGES 某颜色的 inRange 下限 (LOWER), 并返回上下界。

    低饱和回退: 传 s_min/v_min 时覆盖 S/V 下限, 色相带不变。
    黑色特殊: 靠低 V 定义 (V≤30), 不受回退 v_min 约束 (回退只放宽不排除真黑)。
    """
    lower, upper = COLOR_RANGES[color_name]
    if color_name == 'black' or (s_min is None and v_min is None):
        return np.array(lower), np.array(upper)
    return (np.array((lower[0],
                      s_min if s_min is not None else lower[1],
                      v_min if v_min is not None else lower[2])),
            np.array(upper))


def _should_count_color(color_name):
    """
    该颜色是否参与判色 (analyze_color_proportions)。

    全帧搜索定位另用 RECON_HSV_IGNORE_COLORS 排除白/灰/黑 —— 不冲突。
    判色参与由 COUNT_WHITE / COUNT_BLACK / NEVER_COUNT / SKIP 控制。
    """
    if color_name in RECON_HSV_SKIP_COLORS:
        return False
    if color_name in RECON_HSV_NEVER_COUNT:
        return False
    if color_name == 'black' and not RECON_HSV_COUNT_BLACK:
        return False
    if color_name == 'white' and not RECON_HSV_COUNT_WHITE:
        return False
    return True


def get_center_mask(h, w, ratio=RECON_HSV_CENTER_RATIO):
    """
    生成中心圆形掩码 (吸收 v2 detector_recon_v2.py)。

    ratio 控制保留区域比例: 0.8 = 保留中心 80%。
    判色只在圆内统计, 排除桶壁/外圈干扰。
    """
    cx, cy = w // 2, h // 2
    radius = int(min(w, h) * ratio / 2)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, (cx, cy), radius, 255, -1)
    return mask


def analyze_color_proportions(roi, min_area=RECON_HSV_BLOB_MIN_AREA,
                              s_min=None, v_min=None):
    """
    HSV 占比分析 — 侦察区判色 (v3.5 吸收 v2 中心掩码思路)。

      1. 只在 ROI 中心圆 (RECON_HSV_CENTER_RATIO) 内统计 → 桶壁/外圈不入
      2. 分母 = 中心圆面积 → 不被整ROI/桶壁灌水
      3. 白/黑/灰参与由 _should_count_color 控制 (红白配色: 白参与, 黑/灰不参与)
      4. v3.6 双计防护: 白色掩码排除已在饱和色(红橙黄绿蓝紫)内命中的像素 —
         red(S≥12) 与 white(S≤30) 在 S∈[12,30] 重叠, 反光/抗锯齿的"浅红"像素
         会同时计入红和白, 抬升 white% → 易燃(红底)可能被误判成自燃物品
      5. 保留低饱和回退: 传 s_min/v_min 覆盖 S/V 下限重判

    返回: {color_name: pct, ...} 或 {} (无标签 → 空桶)
    """
    if roi is None or roi.size == 0:
        return {}
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, w = roi.shape[:2]
    center_mask = get_center_mask(h, w)
    total_pixels = cv2.countNonZero(center_mask)
    if total_pixels == 0:
        return {}

    # 先算各色掩码 (不开孔, 供白色双计防护合并饱和色)
    color_masks = {}
    for color_name in COLOR_RANGES:
        if not _should_count_color(color_name):
            continue
        lo, hi = _color_range_lo(color_name, s_min, v_min)
        mask = cv2.inRange(hsv, lo, hi)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        color_masks[color_name] = mask

    # v3.7.1 双计防护(收窄): 白色只排除"红" — 红底(易燃)抗锯齿/反光的
    #   "浅红"像素(S∈[12,30])会同时命中红和白, 抬升 white% → 易燃误判成自燃物品。
    #   不能排除黄/蓝等: 自燃/放射性的白半边在暖光下反射黄/蓝(S∈[12,30]),
    #   被排除后白半边只算成"黄/蓝" → 自燃(红+白)被误判成生物危害(纯黄)。
    #   实测: 正上方悬停自燃物品 → 返回生物危害, 即此回归所致。
    if 'white' in color_masks:
        redish = None
        for cn in ('red', 'red2'):
            if cn in color_masks:
                redish = (color_masks[cn] if redish is None
                          else cv2.bitwise_or(redish, color_masks[cn]))
        if redish is not None:
            color_masks['white'] = cv2.bitwise_and(
                color_masks['white'], cv2.bitwise_not(redish))

    color_area_map = defaultdict(int)
    for color_name, mask in color_masks.items():
        mask = cv2.bitwise_and(mask, mask, mask=center_mask)
        area = cv2.countNonZero(mask)
        if area >= min_area:
            color_area_map[color_name] += area

    # 红: 合并 H 两个红区 (0-10 和 170-180)
    if 'red' in color_area_map and 'red2' in color_area_map:
        color_area_map['red'] += color_area_map['red2']
        del color_area_map['red2']
    elif 'red2' in color_area_map:
        color_area_map['red'] = color_area_map.pop('red2')

    proportions = {}
    for color, area in color_area_map.items():
        pct = (area / total_pixels) * 100.0
        if pct > RECON_HSV_PCT_MIN:
            proportions[color] = round(pct, 2)
    if not proportions:
        return {}
    proportions = dict(sorted(proportions.items(),
                              key=lambda item: item[1], reverse=True))

    return proportions


# ── v3: 定点突发跟踪器 ──

class BucketTrack:
    """
    单个桶在定点悬停期间的跟踪状态。

    frame_state 每帧喂入:
      ('color', {name: pct, ...})  |  ('empty',)  |  None(本帧无法判断)

    滞回逻辑 (防抖):
      - 连续 RECON_CONFIRM_FRAMES 帧出现同一新状态才切换 stable
      - 判同只比较 (key, 颜色名集合), pct 波动不触发切换, 只刷新占比
      - 单帧闪变/偶发误判不会翻转输出
      - lost>RECON_BUCKET_LOST_MAX 的轨道被清理

    v3.2: 颜色改为完整 dict (多色), 管道协议与原 unified 一致
    """
    __slots__ = ('tid', 'cx', 'cy', 'box', 'ellipse',
                 'stable_key', 'stable_colors',
                 'pending_key', 'pending_colors', 'pending_count', 'lost')

    def __init__(self, tid, cx, cy, box):
        self.tid = tid
        self.cx = cx
        self.cy = cy
        self.box = box            # (x1,y1,x2,y2) 全帧像素
        self.ellipse = None       # 最近一帧的椭圆(全帧坐标), 用于绘制
        self.stable_key = None    # 'color' | 'empty' | None
        self.stable_colors = None # color 时的 {颜色名: 占比} (dict 保序)
        self.pending_key = None   # 待确认状态
        self.pending_colors = None
        self.pending_count = 0
        self.lost = 0             # 连续未匹配帧数

    @staticmethod
    def _same_colors(a, b):
        """判同只比较颜色名集合, 忽略占比波动"""
        if a is None and b is None:
            return True
        if a is None or b is None:
            return False
        return set(a.keys()) == set(b.keys())

    def feed(self, frame_state):
        if frame_state is None:
            return  # unknown 不参与投票
        key = frame_state[0]
        colors = frame_state[1] if key == 'color' else None

        # 与 stable 相同 (key+颜色名集合) → 刷新占比, 重置 pending
        if key == self.stable_key and self._same_colors(colors, self.stable_colors):
            self.stable_colors = colors
            self.pending_key = None
            self.pending_colors = None
            self.pending_count = 0
            return

        if key == self.pending_key and self._same_colors(colors, self.pending_colors):
            self.pending_count += 1
        else:
            self.pending_key = key
            self.pending_colors = colors
            self.pending_count = 1

        if self.pending_count >= RECON_CONFIRM_FRAMES:
            self.stable_key = self.pending_key
            self.stable_colors = colors   # 用当前帧占比 (最新), 而非首次观察
            self.pending_key = None
            self.pending_colors = None
            self.pending_count = 0

    def output(self):
        """管道用状态字符串 (多色: "red:70.0,blue:20.0", 主色在前)"""
        if self.stable_key is None:
            return "unknown"
        if self.stable_key == 'empty':
            return "empty"
        return ",".join(
            f"{c}:{self.stable_colors[c]:.1f}"
            for c in sorted(self.stable_colors,
                            key=lambda c: self.stable_colors[c], reverse=True))


_recon_tid = itertools.count(1)   # 全局单调轨道ID


def update_tracks(tracks, dets):
    """
    v3: 帧间桶关联 (悬停时中心几乎不动, 按最近距离贪心匹配)。

    dets: [ {cx, cy, box, ellipse, state}, ... ]
    返回: 更新后的轨道列表 (含丢失保留 + 新增)
    """
    used = [False] * len(dets)
    out = []

    # ── 现有轨道优先匹配最近的检测 ──
    for t in tracks:
        best_d2, best_i = float('inf'), -1
        for i, d in enumerate(dets):
            if used[i]:
                continue
            dd = (t.cx - d['cx']) ** 2 + (t.cy - d['cy']) ** 2
            if dd < best_d2 and dd <= RECON_MATCH_DIST ** 2:
                best_d2, best_i = dd, i
        if best_i >= 0:
            used[best_i] = True
            t.cx, t.cy = dets[best_i]['cx'], dets[best_i]['cy']
            t.box = dets[best_i]['box']
            t.ellipse = dets[best_i]['ellipse']
            t.lost = 0
            t.feed(dets[best_i]['state'])
            out.append(t)
        else:
            t.lost += 1
            t.feed(None)   # 本帧未看到, 不参与投票, 短暂遮挡保持 stable
            if t.lost <= RECON_BUCKET_LOST_MAX:
                out.append(t)

    # ── 新检测 → 新轨道 ──
    for i, d in enumerate(dets):
        if not used[i]:
            t = BucketTrack(next(_recon_tid), d['cx'], d['cy'], d['box'])
            t.ellipse = d['ellipse']
            t.feed(d['state'])
            out.append(t)

    # v3.7: 轨道数上限 — 跨帧碎片框持续累积会让输出爆炸(放2桶报13),
    #   按 x 保留最左 MAX_BUCKETS 个 (位置序号即按 x, 与输出一致)
    out.sort(key=lambda t: t.cx)
    return out[:MAX_BUCKETS]


'''

# ═══════════════════════════════════════════════════════════════
#  投放区: 桶直径分类
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
ws_queue = queue.Queue(maxsize=2)  # v3.8: WebSocket 推送队列 (只保留最新1-2帧)

vision_pipe_fd = None
recon_pipe_fd  = None
h_pipe_fd      = None

save_dir = ""
save_frame_interval = 5
save_frame_count = 0

bucket_model = None
h_model = None
bridge_proc = None

# v3.3: 模型切换互斥 + 异步预加载
#   RTL 返航期间后台线程预加载 H 模型, 到降落区零等待;
#   加锁避免后台加载与 process_h 同步加载竞态
model_switch_lock = threading.Lock()
h_model_loading = False     # v3.3: 后台预加载 H 进行中标志 (process_h 据此跳过推理不阻塞)
bucket_model_loading = False  # v3.3: 后台预加载桶进行中标志
h_model_pinned = False       # v4: 一旦进入侦察区，H 模型常驻到程序退出/断电

# v4: 侦察区无推理状态。管道对象继续保留，确保 C++ 接口不变。


# ── H 标识历史缓存 ──
h_last_cx = None          # 上一次检测到的 H 中心 X
h_last_cy = None          # 上一次检测到的 H 中心 Y
h_last_conf = 0.0         # 上一次检测的置信度
h_last_box = None         # v3.9: 上一次检出 H 框的宽高 (重检自适应半径用)
h_loss_frames = 0         # 连续未检出帧数 (含滤波过渡)
H_HISTORY_MAX = 90        # 真丢失后最多保持历史的帧数 (~3s @30fps)
H_LOST_CONFIRM = 5        # v3.3: 连续 N 帧未检出才判真丢失 (单帧抖动/偶发误检不算)
                          # v3.6: 3→5, 防抖窗口放宽 — 降落时 H 频繁短暂出画不误停伺服
H_CONF_LOW = 0.15         # v3.3: 丢失后扩大搜索用低阈值 (捞回边缘/小/畸变 H)
H_SEARCH_RADIUS = 200     # v3.3: 扩大搜索最大中心偏移 (px, 416输入), 防远距离误检

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
    global bucket_model, bucket_model_loading
    if bucket_model is not None: return
    bucket_model_loading = True   # 标记加载中, process_drop/recon 据此跳过推理不阻塞
    try:
        with model_switch_lock:
            if bucket_model is not None: return
            print("加载桶检测模型...")
            for cache_ext in ['.cache', '.cache.lock']:
                f = BUCKET_MODEL_PATH + cache_ext
                if os.path.exists(f):
                    os.remove(f)
            bucket_model = torch.hub.load(YOLOV5_REPO, 'custom', path=BUCKET_MODEL_PATH,
                                           source='local', device='0', force_reload=False)
            bucket_model.conf = CONF_THRESH
            bucket_model.classes = [0]
            try:
                bucket_model.half()
                print("桶模型加载成功 (FP16)")
            except Exception:
                bucket_model.float()
                print("桶模型加载成功 (FP32，FP16 不可用已回退)")
            dummy = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
            for _ in range(2):
                bucket_model(dummy, size=IMG_SIZE)
            print("桶模型预热完成")
    finally:
        bucket_model_loading = False


def preload_bucket_model_async():
    """后台线程预加载桶模型，仅在投放模式保留。"""
    def _worker():
        try:
            load_bucket_model()
            with mode_lock:
                m = current_mode
            # 若加载期间已经进入侦察/返航，加载完成后立即释放。
            if m != MissionMode.DROP:
                maybe_unload_bucket_model()
        except Exception as e:
            print(f"[MODEL] 桶模型预加载失败: {e}")
    threading.Thread(target=_worker, daemon=True).start()


def load_h_model():
    global h_model, h_model_loading
    if h_model is not None: return
    h_model_loading = True   # 标记加载中, process_h 据此跳过推理不阻塞
    try:
        with model_switch_lock:
            if h_model is not None: return
            print("加载 H 检测模型...")
            for cache_ext in ['.cache', '.cache.lock']:
                f = H_MODEL_PATH + cache_ext
                if os.path.exists(f):
                    os.remove(f)
            h_model = torch.hub.load(YOLOV5_REPO, 'custom', path=H_MODEL_PATH,
                                      source='local', device='0', force_reload=False)
            h_model.conf = H_CONF_THRESH
            h_model.classes = [0]
            try:
                h_model.half()
                print("H 模型加载成功 (FP16)")
            except Exception:
                h_model.float()
                print("H 模型加载成功 (FP32，FP16 不可用已回退)")
            dummy = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
            for _ in range(2):
                h_model(dummy, size=IMG_SIZE)
            print("H 模型预热完成")
    finally:
        h_model_loading = False


def preload_h_model_async():
    """后台加载并预热 H 模型。

    v4: 首次进入侦察区后 h_model_pinned=True，H 模型不再因后续模式变化释放，
    一直驻留到程序退出或设备断电。
    """
    def _worker():
        try:
            load_h_model()
            with mode_lock:
                m = current_mode
            if (not h_model_pinned and
                    m not in (MissionMode.RTL, MissionMode.H_LAND)):
                maybe_unload_h_model()
        except Exception as e:
            print(f"[MODEL] H 模型预加载失败: {e}")
    threading.Thread(target=_worker, daemon=True).start()

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

# ── 任务状态读取线程 ──
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

# ── 高度读取线程 ──
def altitude_reader():
    """读取 C++ 推送的高度 (m)。

    v3 修复: C++ 晚于本线程启动, 且可能 unlink+mkfifo 重建管道 inode。
    旧实现管道不存在直接 return、EOF 后不重开 → 竞态下高度永远钉死默认值。
    现在: 缺失→重试等待; EOF/旧inode 失效→关闭重开, 自动接上重建后的管道。
    """
    global CURRENT_ALTITUDE, ALT_TIMESTAMP
    while running:
        # 阶段1: 等待管道文件出现 (C++ 可能晚于本线程启动)
        if not os.path.exists(ALT_PIPE):
            time.sleep(0.2)
            continue
        # 阶段2: 打开 (O_NONBLOCK 读端无需等写端)
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
                    break   # 旧 fd 失效 → 重开
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
                    # EOF: 写端已关闭 / C++ unlink 重建管道 → 关闭重开接新 inode
                    break
        finally:
            try:
                os.close(alt_fd)
            except OSError:
                pass
        time.sleep(0.1)

# ── 采集线程 ──
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


# ═══════════════════════════════════════════════════════════════
#  推理主循环 & 三个任务处理函数
# ═══════════════════════════════════════════════════════════════

def inference_worker():
    global running, current_mode, h_model_pinned
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
                    if bucket_model is not None: maybe_unload_bucket_model()
                    if h_model is None: preload_h_model_async()
                    if prev_mode not in (MissionMode.RTL, MissionMode.H_LAND):
                        h_loss_frames = 0
                        h_last_cx = h_last_cy = None
                        h_last_conf = 0.0
                        h_last_box = None
                    print("[MODEL] 切到返航/降落, 释放桶 + 后台预加载 H")
                elif mode == MissionMode.DROP:
                    if h_model is not None and not h_model_pinned:
                        maybe_unload_h_model()
                    if bucket_model is None:
                        preload_bucket_model_async()
                    if h_model_pinned:
                        print("[MODEL] 切到投放, H 模型按常驻策略保留 + 后台预加载桶")
                    else:
                        print("[MODEL] 切到投放, 释放 H + 后台预加载桶")
                elif mode == MissionMode.RECON:
                    # v4: 到达侦察区即进入后半程模型策略：释放桶模型，
                    # 后台加载并预热 H，随后保持 H 常驻直到程序退出/断电。
                    h_model_pinned = True
                    if bucket_model is not None:
                        maybe_unload_bucket_model()
                    if h_model is None:
                        preload_h_model_async()
                    print("[MODEL] 切到侦察, 释放桶 + 后台预加载 H；H 将常驻至断电")
                prev_mode = mode

            if mode == MissionMode.RECON:
                process_recon(frame, frame_count)
            elif mode == MissionMode.DROP:
                process_drop(frame, frame_count)
            elif mode in (MissionMode.RTL, MissionMode.H_LAND):
                process_h(frame, frame_count)
            else:
                # v4: 启动时无模式也推送原始视频流到 WebSocket
                push_display(frame)
                push_ws(frame, None)  # 无模式状态使用标准质量
        except Exception as e:
            print(f"[推理异常] {e}", flush=True)
    print("推理线程退出")


# ── 投放区 ──

def process_drop(frame, frame_count):
    global bucket_model
    if bucket_model is None:
        if not h_model_pinned:
            maybe_unload_h_model()
        load_bucket_model()

    with torch.no_grad():
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


# ── 侦察区 (v6: HSV 颜色搜索为主 + 定点突发跟踪) ──

def push_recon(msg):
    """覆盖式写入 recon_pipe 队列 (只保留最新)"""
    while not recon_pipe_queue.empty():
        try: recon_pipe_queue.get_nowait()
        except queue.Empty: break
    recon_pipe_queue.put(msg)


def _emit_recon_result(recon_tracks, frame, frame_count, used_backup=False):
    """
    v3.2: 发送 + 绘制侦察结果 (检测帧与非检测帧共用)。

    非检测帧调用时轨道状态是上次检测帧的 → 悬停场景桶几乎不动, 复用安全。
    管道格式: "1:易燃;2:empty" (状态为中文标识名/empty; unknown 按 RECON_FOLD_UNKNOWN_TO_EMPTY 折叠)。
    注: 状态是中文标识名, 不再是 v2 的颜色比例串 — C++ 解析端需同步改。
    """
    active = sorted(recon_tracks, key=lambda t: t.cx)
    pipe_parts = []
    result_frame = frame.copy()

    for i, t in enumerate(active):
        bid = i + 1   # 位置序号 (按 x), 与旧协议一致
        x1i, y1i, x2i, y2i = t.box

        if t.stable_key is None:
            color_bgr = (200, 180, 0)      # 青: 尚未确认
            label = f"B{bid} [UNKNOWN]"
            # 协议兼容: 折叠 unknown → empty (与 v2 一致), 视觉仍显示 [UNKNOWN]
            state_str = "empty" if RECON_FOLD_UNKNOWN_TO_EMPTY else "unknown"
        elif t.stable_key == 'empty':
            color_bgr = (0, 200, 200)      # 黄: 确认空桶
            label = f"B{bid} [EMPTY]"
            state_str = "empty"
        else:
            color_bgr = (0, 255, 0)        # 绿: 确认颜色
            # 映射到标识名称 (调试用: 视觉上仍显示原始比例)
            label_name = identify_label_name(t.stable_colors)
            if label_name == 'unknown' and RECON_FOLD_UNKNOWN_TO_EMPTY:
                # 协议: unknown 折叠为 empty (与 v2 一致); 视觉仍标 [UNKNOWN]
                state_str = "empty"
                label = f"B{bid} [UNKNOWN]"
            else:
                state_str = label_name   # 管道仍发中文标识名, 协议不变
                sc = sorted(t.stable_colors.items(),
                            key=lambda kv: kv[1], reverse=True)
                # 显示用英文短码 (OpenCV Hershey 无中文字形, 写中文乱码)
                en_parts = [f"{COLOR_NAMES_EN.get(c, c)}:{p:.0f}%" for c, p in sc]
                label = (f"B{bid} {LABEL_NAMES_EN.get(label_name, label_name)} "
                         f"({' '.join(en_parts)})")

        pipe_parts.append(f"{bid}:{state_str}")

        cv2.rectangle(result_frame, (x1i, y1i), (x2i, y2i), color_bgr, 2)
        if t.ellipse is not None:
            cv2.ellipse(result_frame, t.ellipse, color_bgr, 2)
        cv2.putText(result_frame, label, (x1i, max(y1i - 5, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)

    # ── 汇总 ──
    msg = ";".join(pipe_parts) if pipe_parts else "None"
    if frame_count % 10 == 0 and pipe_parts:
        # v3.7.1 诊断: 附上每个 color 轨道的原始颜色占比, 便于判断
        # "生物危害" 是纯黄(真黄桶) 还是 红+白(自燃被误读) — 看 COLORS 即知
        dbg = []
        for t in active:
            if t.stable_key == 'color':
                sc = sorted(t.stable_colors.items(),
                            key=lambda kv: kv[1], reverse=True)
                dbg.append("(" + " ".join(f"{c}:{p:.1f}" for c, p in sc) + ")")
        print(f"[RECON SUMMARY] {' | '.join(pipe_parts)}" +
              (f"  COLORS: {dbg}" if dbg else ""))

    # ── 管道发送 ──
    push_recon(msg)

    # 备用模式 HUD 标记
    if used_backup:
        cv2.putText(result_frame, "RECON:CV-BACKUP", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 2)

    # ── 显示弹窗 ──
    if USE_DISPLAY:
        popups = []
        for t in active:
            bx1, by1, bx2, by2 = t.box
            roi_pop = frame[by1:by2, bx1:bx2]
            if roi_pop.size > 0:
                popups.append(cv2.resize(roi_pop, (160, 160)))
        while not popup_queue.empty():
            try: popup_queue.get_nowait()
            except queue.Empty: break
        popup_queue.put(popups)

    push_save(result_frame, "RECON")
    push_display(result_frame)
    push_ws(result_frame)  # v3.8: 推送标注帧到 WebSocket


def _probe_red(frame):
    """调试 (v3.7.2): 全帧红像素计数 — 定位红标为何识别不到.

    分 S≥12 / S≥6 两档 (V 都≥18, 对应正常轮 S12 / 回退轮 S6)。判断:
      - S12≈0 且 S6>0  → 红色只是发灰 (S 掉到 6-12), 已由回退(S6)覆盖
      - S12≈0 且 S6≈0  → 红色更灰 (S<6) 或发暗 (V<18) 或色相偏 (不在0-10/160-180)
      - S12>0  但没检出红桶 → 别的环节 (合并/判色/中心圆) 有问题
    每 20 帧一次, 只在有红像素时打印 (隔帧一次 inRange, 开销可忽略)。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    for tag, s in (('S12', 12), ('S6', 6)):
        m = cv2.inRange(hsv, (0, s, 18), (10, 255, 255))
        m2 = cv2.inRange(hsv, (160, s, 18), (180, 255, 255))
        m = cv2.bitwise_or(m, m2)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=1)
        n = cv2.countNonZero(m)
        if n > 30:
            print(f"[RED-PROBE] {tag}: {n}px")


def process_recon(frame, frame_count):
    """侦察区只转发最新视频帧；不加载模型、不运行 YOLO/HSV、不写识别坐标。"""
    push_ws(frame, MissionMode.RECON)
    push_display(frame)



# ── H 降落 (不变) ──

def process_h(frame, frame_count):
    """
    H 标识降落检测 — 摄像头能看到 H 就实时发送中心坐标。

    逻辑 (v3.3 + v3.6):
      1. YOLO 检测 H → 选置信度最高的 → 立即发 cx,cy
      2. 没检测到 → 首帧低阈值扩大搜索 (H_CONF_LOW + 位置邻域) 捞回边缘 H
      3. 连续 H_LOST_CONFIRM(5) 帧未检出才判真丢失 → 真丢失后发 None (v3.6)
         (1~4 帧防抖窗口内补位历史坐标, 防单帧抖动误停伺服)
      4. 真丢失且超过 H_HISTORY_MAX(90) 帧 → 清空历史

    v3.6 修复: 旧代码丢失后 90 帧持续补位旧坐标, C++ 端无法区分真/补位,
    拿旧坐标继续视觉伺服; 现在真丢失即发 None, C++ 端进 lost-tracking 分支。
    模型切换: 先卸载桶模型释放显存, 再加载 H (加载更快, 避免"飞过H才就绪")。
    """
    global h_model, h_last_cx, h_last_cy, h_last_conf, h_last_box, h_loss_frames
    if h_model is None:
        if h_model_loading:
            # v3.3: 后台预加载进行中 → 不阻塞推理线程。
            #   v3.6: 加载期同样走丢失滤波 — 有历史且未超防抖窗才补位,
            #         否则发 None (旧代码加载期无限补位旧坐标, 绕过丢失滤波)
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
        # 兜底: 无后台加载 → 同步加载 (先释放桶显存)
        maybe_unload_bucket_model()
        load_h_model()

    with torch.no_grad():
        results = h_model(frame, size=IMG_SIZE)
    detections = results.xyxy[0].cpu().numpy()
    h_dets = detections[detections[:, 5] == 0] if len(detections) > 0 else []

    msg = "None"
    result_frame = frame.copy()
    h_img, w_img = frame.shape[:2]
    best_box = None   # (x1,y1,x2,y2,conf,cx,cy)

    # ── 1. 检测到了 → 选置信度最高的, 直接发 ──
    if len(h_dets) > 0:
        top = h_dets[h_dets[:, 4].argmax()]
        x1, y1, x2, y2, conf, cls = top
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        best_box = (x1, y1, x2, y2, conf, cx, cy)
        h_last_box = (x2 - x1, y2 - y1)   # v3.9: 记录框大小 (重检自适应半径)

        h_last_cx, h_last_cy = cx, cy
        h_last_conf = conf
        h_loss_frames = 0
        msg = f"{cx:.2f},{cy:.2f}"

    # ── 2. 没检测到 → v3.3 三帧滤波 + 扩大搜索 ──
    else:
        h_loss_frames += 1
        # 扩大搜索: 真丢失前, 丢失窗口内每帧用更低阈值重检 (捞回边缘/小/畸变 H)
        #   v3.9: 旧逻辑只在丢失第1帧重检一次 (单次侥幸), 第2~4帧纯补位旧坐标,
        #   第5帧判死 → 降落时 H 因运动模糊/俯仰瞬间出画就丢。
        #   现在 1..H_LOST_CONFIRM 帧每一帧都重检, 只要 H 在某帧可见立刻捞回;
        #   真丢失确认 (≥H_LOST_CONFIRM) 后不再重检直接发 None, 成本可控。
        if h_loss_frames <= H_LOST_CONFIRM and h_last_cx is not None:
            _h_model_backup_conf = h_model.conf
            h_model.conf = H_CONF_LOW
            try:
                with torch.no_grad():
                    r2 = h_model(frame, size=IMG_SIZE)
            finally:
                h_model.conf = _h_model_backup_conf   # 异常也恢复阈值
            dets2 = r2.xyxy[0].cpu().numpy()
            h2 = dets2[dets2[:, 5] == 0] if len(dets2) > 0 else []
            if len(h2) > 0:
                top2 = h2[h2[:, 4].argmax()]
                x1, y1, x2, y2, conf2, cls2 = top2
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                # 扩大搜索: 位置在上次检测附近才接受, 防远距离误检。
                #   v3.9: 半径随上次 H 框大小自适应 — H 越大越近、中心移动越快,
                #   固定 200px 可能滑出邻域被拒; 小 H (远) 仍用 H_SEARCH_RADIUS。
                radius = H_SEARCH_RADIUS
                if h_last_box is not None:
                    radius = max(radius, h_last_box[0], h_last_box[1])
                if (abs(cx - h_last_cx) <= radius and
                        abs(cy - h_last_cy) <= radius):
                    best_box = (x1, y1, x2, y2, conf2, cx, cy)
                    h_last_box = (x2 - x1, y2 - y1)   # v3.9: 重检捞回也刷新框大小
                    h_last_cx, h_last_cy = cx, cy
                    h_last_conf = conf2
                    h_loss_frames = 0
                    msg = f"{cx:.2f},{cy:.2f}"

        # 三帧滤波: 连续 H_LOST_CONFIRM 帧未检出才判真丢失
        #   (单帧抖动/偶发误检: 不发 None, 继续补上次坐标, 控制端不会误返航)
        #   v3.6 修复: 真丢失 (≥H_LOST_CONFIRM) 后管道必须发 None 停伺服 —
        #   旧代码两分支都补位历史坐标, 导致"没有标识却一直发旧坐标"最长 90 帧(~3s),
        #   C++ 端无法区分真检测/补位, 拿旧坐标继续视觉伺服。
        #   修复后 C++ 端走 lost-tracking 分支 (用上次世界误差 + 继续下降), 无需改 C++。
        if msg == "None" and h_last_cx is not None:
            if h_loss_frames < H_LOST_CONFIRM:
                msg = f"{h_last_cx:.2f},{h_last_cy:.2f}"
            else:
                msg = "None"
                if h_loss_frames > H_HISTORY_MAX:
                    h_last_cx = h_last_cy = None
                    h_last_conf = 0.0
                    h_last_box = None   # v3.9: 历史清空时一并清空框大小
                    h_loss_frames = 0
        else:
            # 无历史 (已清空/从未检出) → 计数无意义, 复位,
            #   防 HUD 显示 "H:HOLD N/90" 无限增长 (应走 LOST)
            h_loss_frames = 0

    # ── 3. 绘制 ──
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

    # 十字参考线
    cv2.line(result_frame, (w_img // 2, 0), (w_img // 2, h_img),
             (100, 100, 100), 1)
    cv2.line(result_frame, (0, h_img // 2), (w_img, h_img // 2),
             (100, 100, 100), 1)

    # HUD
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

    # ── 4. 管道发送 ──
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


# ═══════════════════════════════════════════════════════════════
#  管道发送、显示、保存、WebSocket 线程
# ═══════════════════════════════════════════════════════════════

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
    """v4: 每个客户端连接后，持续从 ws_queue 取帧并发送，根据模式选择 JPEG 质量
    (Python 3.8 兼容: 用 get_nowait + run_in_executor 替代 asyncio.to_thread)
    """
    print(f"[WS] 客户端已连接: {websocket.remote_address}")
    loop = asyncio.get_event_loop()
    try:
        while running:
            try:
                # 非阻塞取帧 (Python 3.8 兼容, 不用 asyncio.to_thread)
                data = ws_queue.get_nowait()
                # v4: 兼容旧格式 (frame) 和新格式 (frame, mode)
                if isinstance(data, tuple):
                    frame, mode = data
                else:
                    frame, mode = data, None
            except queue.Empty:
                await asyncio.sleep(0.01)
                continue

            # v4: 根据模式选择 JPEG 质量（侦察区高质量，其他区标准质量）
            quality = RECON_WS_JPEG_QUALITY if mode == MissionMode.RECON else WS_JPEG_QUALITY

            # JPEG 编码放到线程池执行, 避免卡住事件循环 (Python 3.8 兼容)
            _, jpeg = await loop.run_in_executor(
                None, cv2.imencode, '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality]
            )
            await websocket.send(jpeg.tobytes())
    except websockets.exceptions.ConnectionClosed:
        print(f"[WS] 客户端已断开: {websocket.remote_address}")
    except Exception as e:
        print(f"[WS] 连接异常: {e}")


async def ws_main():
    """v3.8: WebSocket 服务主循环"""
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        print(f"[WS] WebSocket 服务器已启动 ws://{WS_HOST}:{WS_PORT}")
        await asyncio.Future()  # 永久运行


def pipe_sender_worker():
    global running, vision_pipe_fd, recon_pipe_fd, h_pipe_fd
    print("管道发送线程已启动")
    while running:
        sent = False
        # vision_pipe (二进制)
        if not vision_pipe_queue.empty():
            count, blist = vision_pipe_queue.get()
            if vision_pipe_fd is None:
                vision_pipe_fd = try_open_write(VISION_PIPE)
            if vision_pipe_fd is not None:
                try:
                    vision_pipe_fd.write(struct.pack('B', count))
                    for bid, cx, cy in blist:
                        vision_pipe_fd.write(struct.pack('B', bid))
                        vision_pipe_fd.write(struct.pack('ff', cx, cy))
                    vision_pipe_fd.flush()
                    sent = True
                except (BrokenPipeError, OSError):
                    try: vision_pipe_fd.close()
                    except: pass
                    vision_pipe_fd = None

        # recon_pipe (文本)
        if not recon_pipe_queue.empty():
            msg = recon_pipe_queue.get()
            if recon_pipe_fd is None:
                recon_pipe_fd = try_open_write(RECON_PIPE)
            if recon_pipe_fd is not None:
                try:
                    recon_pipe_fd.write((msg + "\n").encode())
                    recon_pipe_fd.flush()
                    sent = True
                except (BrokenPipeError, OSError):
                    try: recon_pipe_fd.close()
                    except: pass
                    recon_pipe_fd = None

        # h_pipe (文本)
        if not h_pipe_queue.empty():
            msg = h_pipe_queue.get()
            if h_pipe_fd is None:
                h_pipe_fd = try_open_write(H_PIPE)
            if h_pipe_fd is not None:
                try:
                    h_pipe_fd.write((msg + "\n").encode())
                    h_pipe_fd.flush()
                    sent = True
                except (BrokenPipeError, OSError):
                    try: h_pipe_fd.close()
                    except: pass
                    h_pipe_fd = None

        if not sent:
            time.sleep(0.002)


def display_worker():
    global running
    if not USE_DISPLAY: return
    print("显示线程已启动")
    cv2.namedWindow("Unified Detection v3", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Unified Detection v3", 960, 540)
    delay = 1.0 / 15
    popup_windows = []
    while running:
        if not disp_queue.empty():
            frame = disp_queue.get()
            cv2.imshow("Unified Detection v3", frame)
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


# ═══════════════════════════════════════════════════════════════
#  主程序
# ═══════════════════════════════════════════════════════════════

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
    t_ws = threading.Thread(target=ws_server_thread, daemon=True); t_ws.start(); threads.append(t_ws)  # v3.8: WebSocket 服务
    if USE_DISPLAY:
        t_disp = threading.Thread(target=display_worker, daemon=True); t_disp.start(); threads.append(t_disp)

    t_save = threading.Thread(target=save_worker, daemon=True); t_save.start(); threads.append(t_save)

    print("=" * 50)
    print("  统一视觉检测系统 v3")
    print(f"  模式: {'仿真' if SIM_MODE else '真机'}")
    print("  投放区 → 侦察区(定点突发融合+Lab) → H降落")
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
        maybe_unload_bucket_model()
        maybe_unload_h_model()
        print("程序已退出")


if __name__ == "__main__":
    main()