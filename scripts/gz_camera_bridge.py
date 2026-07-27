#!/usr/bin/env python3
"""
Gazebo 仿真相机桥接脚本

从 Gazebo 向下相机订阅图像，转为 OpenCV 格式，支持:
  1. TCP 推流 (模拟 CSI GStreamer，兼容现有视觉代码)
  2. FIFO 管道写入
  3. 屏幕实时显示 (--display)

用法:
  python3 gz_camera_bridge.py [--world WORLD] [--tcp 5000] [--display]

需要:
  pip install opencv-python numpy
"""

import argparse
import os
import struct
import socket
import time
import threading
import numpy as np
import cv2
from gz.transport13 import Node
import gz.msgs10.image_pb2 as image_pb2
from gz.msgs10.image_pb2 import Image


class GazeboCameraBridge:
    """订阅 Gazebo 相机话题，输出到 TCP / FIFO / OpenCV 窗口"""

    def __init__(self, world_name="competition_task_random",
                 tcp_port=None, fifo_path=None, display=False):
        self.world = world_name
        self.tcpPort = tcp_port
        self.fifoPath = fifo_path
        self.display = display
        self.fifoFd = None
        self.tcpServer = None
        self.tcpClient = None
        self.tcpThread = None
        self.running = True
        self.latestFrame = None
        self.frameLock = threading.Lock()

        self.topic = (
            f"/world/{world_name}/model/iris_with_ardupilot"
            f"/model/iris_with_standoffs/link/base_link"
            f"/sensor/downward_camera/image"
        )
        self.node = Node()
        self.frameSeq = 0

    def start(self):
        if self.fifoPath:
            self._openFifo()
        if self.tcpPort:
            self._startTcpServer()
        print(f"[INFO] Subscribing to: {self.topic}")
        self.node.subscribe(Image, self.topic, self._onImage)

    def _openFifo(self):
        if os.path.exists(self.fifoPath) and not os.access(self.fifoPath, os.W_OK):
            os.unlink(self.fifoPath)
        if not os.path.exists(self.fifoPath):
            os.mkfifo(self.fifoPath, 0o666)
        try:
            self.fifoFd = os.open(self.fifoPath, os.O_WRONLY | os.O_NONBLOCK)
            print(f"[INFO] FIFO opened: {self.fifoPath}")
        except OSError as e:
            print(f"[WARN] FIFO open failed: {e}")
            self.fifoFd = None

    def _startTcpServer(self):
        self.tcpServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcpServer.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcpServer.bind(("127.0.0.1", self.tcpPort))
        self.tcpServer.listen(1)
        self.tcpServer.settimeout(1.0)
        print(f"[INFO] TCP server listening on 127.0.0.1:{self.tcpPort}")
        self.tcpThread = threading.Thread(target=self._acceptTcp, daemon=True)
        self.tcpThread.start()

    def _acceptTcp(self):
        while self.running:
            try:
                self.tcpClient, addr = self.tcpServer.accept()
                print(f"[INFO] TCP client connected: {addr}")
            except socket.timeout:
                continue
            except Exception:
                break

    def _onImage(self, msg: Image):
        if msg.pixel_format_type != image_pb2.RGB_INT8:
            return

        self.frameSeq += 1
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size != msg.width * msg.height * 3:
            return
        frame = data.reshape((msg.height, msg.width, 3))

        if self.fifoPath is not None and self.fifoFd is not None:
            self._writeFifo(frame)

        if self.tcpPort and self.tcpClient:
            self._writeTcp(frame)

        if self.display:
            with self.frameLock:
                self.latestFrame = frame.copy()

        if self.frameSeq % 30 == 0:
            print(f"  [cam] frame #{self.frameSeq} {msg.width}x{msg.height}")

    def _writeFifo(self, frame):
        _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        try:
            os.write(self.fifoFd, jpg.tobytes() + b"\n")
        except (BrokenPipeError, OSError):
            pass

    def _writeTcp(self, frame):
        _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        data = jpg.tobytes()
        header = struct.pack(">I", len(data))
        try:
            self.tcpClient.sendall(header + data)
        except Exception:
            self.tcpClient = None

    def _displayLoop(self):
        """在主线程中刷新 OpenCV 显示窗口"""
        cv2.namedWindow("Gazebo Camera (downward)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Gazebo Camera (downward)", 480, 480)
        while self.running:
            with self.frameLock:
                frame = self.latestFrame.copy() if self.latestFrame is not None else None
            if frame is not None:
                cv2.imshow("Gazebo Camera (downward)", frame)
            key = cv2.waitKey(33) & 0xFF  # ~30 fps
            if key == 27:  # ESC
                self.running = False
                break
        cv2.destroyAllWindows()

    def spin(self):
        print("[INFO] Bridge running. Press Ctrl+C to stop, ESC to close window.")
        if self.display:
            self._displayLoop()
        else:
            try:
                while self.running:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
        self.running = False
        self._cleanup()

    def _cleanup(self):
        if self.tcpClient:
            self.tcpClient.close()
        if self.tcpServer:
            self.tcpServer.close()
        if self.fifoFd:
            os.close(self.fifoFd)
        print("[INFO] Bridge stopped.")


def main():
    parser = argparse.ArgumentParser(description="Gazebo camera → OpenCV bridge")
    parser.add_argument("--world", default="competition_task_random")
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--fifo", default=None)
    parser.add_argument("--tcp", type=int, default=5000)
    parser.add_argument("--display", action="store_true",
                        help="Show live camera window on screen")
    parser.add_argument("--no-tcp", action="store_true",
                        help="Disable TCP output")
    args = parser.parse_args()

    port = args.port or args.tcp
    if args.no_tcp:
        port = None

    bridge = GazeboCameraBridge(
        world_name=args.world,
        tcp_port=port,
        fifo_path=args.fifo,
        display=args.display
    )
    bridge.start()
    bridge.spin()


if __name__ == "__main__":
    main()
