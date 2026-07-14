#!/usr/bin/env python3
"""
Gazebo 相机 → GStreamer appsrc → TCP + 本地显示 + 挂载点映射圆圈

挂载点像素投影公式（来自 dxy_apm_ws/CameraGimbal.h::worldToPixel 简化）:
  pixel_u = cx + fx * (mnt_x - cam_x) / altitude
  pixel_v = cy + fy * (mnt_y - cam_y) / altitude
  pixel_r = world_radius * fx / altitude

参数来源:
  dxy_apm_ws/can_config.yaml  → 挂载点物理偏移 (shot_target_x_l/r)
  dxy_apm_ws/OffboardControl.yaml → 相机安装偏移 (drone_to_camera_x/y/z)
  dxy_apm_ws/camera.yaml → 相机内参 (fx/fy/cx/cy)
  仿真相机 (640x640 FOV=60°) → fx=554.26 cx=cy=320
"""

import argparse
import numpy as np
import cv2
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
from gz.transport13 import Node
from gz.msgs10 import image_pb2

WIDTH, HEIGHT, FPS = 640, 640, 30

# 仿真向下相机内参 (FOV_h=60°, 640×640 方像元)
FX = WIDTH / 2.0 / np.tan(np.deg2rad(60) / 2)   # ≈ 554.26
FY = FX
CX = WIDTH / 2.0
CY = HEIGHT / 2.0

# ── 物理参数 (来自 dxy_apm_ws 配置) ──
# 相机在无人机机体坐标系中的偏移 (x=前, y=右)
CAM_DX =  0.15    # 无人机质心→相机 前向偏移 (m)
CAM_DY =  0.0     # 横向偏移

# 左/右挂载点在机体坐标系中的偏移
MNT_LEFT_DX  = -0.07
MNT_LEFT_DY  =  0.001
MNT_RIGHT_DX =  0.07
MNT_RIGHT_DY = -0.001

# 物理桶半径 (用于像素圆半径换算)
WORLD_RADIUS_M = 0.10


def computeMountPixels(altitudeM):
    """根据当前高度计算两个挂载点的像素坐标和半径"""
    if altitudeM < 0.1:
        return None
    # 挂载点相对相机的水平偏移
    d_left_x  = MNT_LEFT_DX  - CAM_DX
    d_left_y  = MNT_LEFT_DY  - CAM_DY
    d_right_x = MNT_RIGHT_DX - CAM_DX
    d_right_y = MNT_RIGHT_DY - CAM_DY

    u_left  = int(CX + FX * d_left_x  / altitudeM)
    v_left  = int(CY + FY * d_left_y  / altitudeM)
    u_right = int(CX + FX * d_right_x / altitudeM)
    v_right = int(CY + FY * d_right_y / altitudeM)
    r_px    = int(WORLD_RADIUS_M * FX / altitudeM)
    return (u_left, v_left), (u_right, v_right), max(r_px, 3)


class GstAppSrcBridge:
    def __init__(self, world_name, tcp_port=None, display=True, altitude=1.2):
        self.topic = (
            f"/world/{world_name}/model/iris_with_ardupilot"
            f"/model/iris_with_standoffs/link/base_link"
            f"/sensor/downward_camera/image"
        )
        self.tcpPort = tcp_port
        self.display = display
        self.altitude = altitude
        self.running = True
        self.frameSeq = 0
        self.appsrc = None
        self.pipeline = None
        self.loop = None

    def buildPipeline(self):
        Gst.init(None)
        pipeDesc = (
            f"appsrc name=src is-live=true block=true format=time"
            f" caps=video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS}/1"
            f" ! videoconvert ! jpegenc ! tee name=t"
        )
        if self.tcpPort:
            pipeDesc += f" t. ! queue ! tcpserversink host=127.0.0.1 port={self.tcpPort}"
        if self.display:
            pipeDesc += f" t. ! queue ! jpegdec ! videoconvert ! autovideosink"

        self.pipeline = Gst.parse_launch(pipeDesc)
        self.appsrc = self.pipeline.get_by_name("src")
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._onBusMessage)
        self.pipeline.set_state(Gst.State.PLAYING)

    def pushFrame(self, rgbData: bytes):
        if self.appsrc is None:
            return
        buf = Gst.Buffer.new_allocate(None, len(rgbData), None)
        buf.fill(0, rgbData)
        buf.duration = Gst.SECOND // FPS
        buf.pts = buf.dts = self.frameSeq * Gst.SECOND // FPS
        ret = self.appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            print(f"[gst] push error: {ret}", flush=True)

    def _onBusMessage(self, bus, msg):
        t = msg.type
        if t == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[gst] ERROR: {err}  {dbg}", flush=True)
            self.running = False
        elif t == Gst.MessageType.EOS:
            print("[gst] EOS", flush=True)
            self.running = False

    def _drawMountCircles(self, frame):
        """在 OpenCV BGR 图像上绘制挂载点映射圆圈"""
        pts = computeMountPixels(self.altitude)
        if pts is None:
            return
        (ul, vl), (ur, vr), radius = pts

        # 左挂载点 — 红色圆圈 + 中心点
        cv2.circle(frame, (ul, vl), radius, (0, 0, 255), 2)
        cv2.circle(frame, (ul, vl), 3, (0, 0, 255), -1)
        cv2.putText(frame, "L", (ul - 20, vl - radius - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 右挂载点 — 绿色圆圈 + 中心点
        cv2.circle(frame, (ur, vr), radius, (0, 255, 0), 2)
        cv2.circle(frame, (ur, vr), 3, (0, 255, 0), -1)
        cv2.putText(frame, "R", (ur - 20, vr - radius - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 高度信息
        cv2.putText(frame, f"H={self.altitude:.1f}m r={radius}px",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    def _onImage(self, msg):
        if not self.running:
            return
        self.frameSeq += 1
        data = np.frombuffer(msg.data, dtype=np.uint8)
        expected = WIDTH * HEIGHT * 3
        if data.size < expected:
            return

        frame = data[:expected].reshape((HEIGHT, WIDTH, 3))
        # Gazebo 输出 RGB → OpenCV BGR
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self._drawMountCircles(frame_bgr)
        # BGR → RGB 推入 GStreamer
        out = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        self.pushFrame(out.tobytes())

        if self.frameSeq % 30 == 0:
            pts = computeMountPixels(self.altitude)
            if pts:
                (ul, vl), (ur, vr), r = pts
                print(f"  [bridge] #{self.frameSeq}  mount L=({ul},{vl}) R=({ur},{vr}) r={r}px",
                      flush=True)
        if self.frameSeq == 1:
            print("[bridge] Streaming started", flush=True)

    def start(self):
        print(f"[bridge] Topic: {self.topic}", flush=True)
        print(f"[bridge] Altitude: {self.altitude}m, "
              f"cam=({CAM_DX:.2f},{CAM_DY:.2f}) "
              f"mounts L=({MNT_LEFT_DX:.2f},{MNT_LEFT_DY:.2f}) "
              f"R=({MNT_RIGHT_DX:.2f},{MNT_RIGHT_DY:.2f})", flush=True)

        pts = computeMountPixels(self.altitude)
        if pts:
            (ul, vl), (ur, vr), r = pts
            print(f"[bridge]  → pixels: L=({ul},{vl}) R=({ur},{vr}) r={r}px", flush=True)

        self.buildPipeline()
        if self.display:
            print("[bridge] Display window: autovideosink", flush=True)
        if self.tcpPort:
            print(f"[bridge] TCP server: 127.0.0.1:{self.tcpPort}", flush=True)

        self.node = Node()
        self.node.subscribe(image_pb2.Image, self.topic, self._onImage)
        print("[bridge] Connected to Gazebo camera", flush=True)

        self.loop = GLib.MainLoop()
        try:
            self.loop.run()
        except KeyboardInterrupt:
            pass

        self.running = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        print("[bridge] Stopped", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Gazebo → GStreamer + mount point overlay")
    parser.add_argument("--world", default="competition_task_random")
    parser.add_argument("--tcp", type=int, default=5000)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--altitude", type=float, default=1.2,
                        help="当前飞行高度 (m), 用于挂载点像素投影")
    args = parser.parse_args()

    bridge = GstAppSrcBridge(
        world_name=args.world,
        tcp_port=args.tcp,
        display=not args.no_display,
        altitude=args.altitude,
    )
    bridge.start()


if __name__ == "__main__":
    main()
