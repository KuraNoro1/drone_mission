#!/usr/bin/env python3
"""
侦察区实时标注帧查看器 — 本地机器端 (笔记本)
======================================================
连接到 Jetson Nano 的 WebSocket 服务 (ws://<IP>:8765),
接收 JPEG 编码的侦察区标注帧 (带检测框、颜色标签、空桶标记),
解码后用 cv2.imshow() 实时显示。

用法:
  1. 修改 JETSON_IP 为 Jetson Nano 的 WiFi IP 地址
  2. 确保 Jetson 上 detector_unified.py 已启动 (自动开启 WebSocket 服务)
  3. 运行本脚本: python3 recon_viewer.py
  4. 按 ESC 键退出
  5. 断线后自动重连 (不用手动重启)

依赖:
  pip install websockets opencv-python numpy
======================================================
"""

import asyncio
import websockets
import cv2
import numpy as np

# ── 配置 ──
JETSON_IP = "192.168.144.106"  # Jetson Nano 的固定 IP（图传网络）
WS_PORT = 8765
RECONNECT_INTERVAL = 2  # 断线后每隔几秒自动重连


async def main():
    """主循环: 连接 WebSocket, 接收 JPEG 帧并显示, 断线自动重连"""
    uri = f"ws://{JETSON_IP}:{WS_PORT}"
    print(f"目标地址: {uri}")
    print("按 ESC 退出, 断线后自动重连\n")

    while True:
        try:
            print(f"正在连接 {uri} ...")
            async with websockets.connect(uri) as ws:
                print("✅ 已连接到 Jetson Nano")
                print("显示侦察区标注帧 (按 ESC 退出)\n")

                while True:
                    # 接收 JPEG 字节流
                    data = await ws.recv()

                    # 解码为图像
                    arr = np.frombuffer(data, np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                    if frame is None:
                        print("⚠️ 解码失败, 跳过此帧")
                        continue

                    # 显示
                    cv2.imshow("Recon View - Jetson Nano", frame)

                    # ESC 退出
                    if cv2.waitKey(1) & 0xFF == 27:
                        print("用户中断")
                        cv2.destroyAllWindows()
                        return

        except (ConnectionRefusedError, OSError) as e:
            print(f"❌ 连接失败: {uri} ({e})")
            print("   请检查: IP 是否正确 / Jetson 程序是否启动 / 防火墙是否放行 8765")
        except websockets.exceptions.ConnectionClosed as e:
            print(f"⚠️ 连接已断开 (关闭码 {e.code}: {e.reason})")
        except KeyboardInterrupt:
            print("\n用户中断")
            return
        except Exception as e:
            print(f"❌ 异常: {e}")

        # 到这里说明连接断了, 自动重连
        print(f"  {RECONNECT_INTERVAL} 秒后重试 (按 Ctrl+C 退出)...\n")
        try:
            await asyncio.sleep(RECONNECT_INTERVAL)
        except KeyboardInterrupt:
            print("\n用户中断")
            return
    cv2.destroyAllWindows()


if __name__ == "__main__":
    asyncio.run(main())
