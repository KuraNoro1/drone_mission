# 真机部署指南

> Jetson Nano + ArduPilot FC + CSI 相机 实飞部署

---

## 一、硬件架构

```
┌──────────────────────────────────────────────────────────────────┐
│                          真机 (Drone)                              │
│                                                                   │
│  FC (ArduPilot) ←─ TELEM2 UART ─→ Jetson Nano                    │
│       │                              │                            │
│       ├── Servo CH1 (左投放)         ├── IMX219 CSI 相机          │
│       ├── Servo CH2 (右投放)         │   ↓ nvarguscamerasrc       │
│       └── GPS / Baro / IMU           │   ↓ GStreamer TCP :5000     │
│                                      └── detector_unified.py      │
│                                           ↓                       │
│                                      /tmp/vision_pipe (FIFO)       │
│                                           ↓                       │
│                                      droneMission (C++ MAVSDK)    │
│                                           └──→ FC via serial      │
├──────────────────────────────────────────────────────────────────┤
│  GCS: QGroundControl ←─ 数传/UDP ─→ FC                           │
│  SSH: 笔记本 ←─ WiFi ─→ Jetson Nano                               │
└──────────────────────────────────────────────────────────────────┘
```

---

## 二、硬件接线

### 2.1 Jetson Nano ↔ FC TELEM2

| Jetson Nano | FC TELEM2 | 说明 |
|-------------|-----------|------|
| UART1 TX (pin 8) | RX | Jetson → FC |
| UART1 RX (pin 10) | TX | FC → Jetson |
| GND (pin 6) | GND | 共地 |

UART1 设备路径: `/dev/ttyTHS1`

### 2.2 FC Servo 输出

| FC 通道 | 功能 | PWM 范围 |
|---------|------|----------|
| SERVO_CH1 | 左投放舵机 | 1000-2000 |
| SERVO_CH2 | 右投放舵机 | 1000-2000 |

PWM 参数配置见 `config/servo.yaml`。

### 2.3 CSI 相机

- 型号: IMX219 (800万像素)
- 接口: Jetson Nano CSI-2 (CAM0)
- 分辨率: 1280×720 @ 30fps
- FOV: 约 60° (与 D 版镜头一致)
- 安装: 机体正下方，镜头朝下 (nadir)
- 安装偏移: (0.15m 前, 0m 右, -0.08m 下) 相对 base_link

### 2.4 供电

| 设备 | 供电方式 | 电压 |
|------|----------|------|
| FC + GPS | BEC 5V | 5V |
| Jetson Nano | DC Jack / 电池直供 | 5V 4A |
| CSI 相机 | Jetson CSI 排线供电 | — |
| Servo | FC BEC / 外接 BEC | 5-6V |

> **注意**: Jetson Nano 峰值功耗约 10-15W，确保电池容量充足（建议 5000mAh 以上 3S/4S）。

---

## 三、系统环境

### 3.1 Jetson Nano 软件栈

```bash
# JetPack 版本 (推荐 4.6+)
cat /etc/nv_tegra_release

# Python 环境
conda create -n yolov8_10 python=3.8
conda activate yolov8_10
pip install torch torchvision numpy opencv-python pandas matplotlib

# YOLOv5 仓库
git clone https://github.com/ultralytics/yolov5 /home/hy/yolov5

# MAVSDK (从源码编译或 deb 安装)
# 参考: https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html

# yaml-cpp
sudo apt install libyaml-cpp-dev cmake g++

# tmux (推荐, 用于一键启动多窗口)
sudo apt install tmux
```

### 3.2 UART 权限

```bash
# 将用户加入 dialout 组
sudo usermod -a -G dialout $USER
# 重启生效
sudo reboot

# 验证
ls -l /dev/ttyTHS1
```

---

## 四、飞行前配置

### 4.1 连接配置

`config/connection.yaml`:
```yaml
connection:
  url: "serial:///dev/ttyTHS1:57600"
  heartbeatTimeout: 10.0
```

### 4.2 飞行参数

`config/flight.yaml` — 根据实际场地调整:
- `takeoffAlt`: 起飞高度 (m)
- `cruiseAlt`: 巡航高度 (m)
- `dropAlt`: 投放高度 (m)

### 4.3 任务坐标

`config/mission_zones.yaml` — 填入实际投放区和侦察区 GPS 坐标。

### 4.4 模型文件

- 桶检测模型: `yolo/best.pt`
- H 标识模型: `yolo/best_H.pt` (若有侦察任务)

---

## 五、真机启动流程

### 5.1 一键启动 (推荐)

```bash
cd ~/drone_mission
bash launch_vision_real.sh              # 无头模式
bash launch_vision_real.sh --display    # 带本地显示器
```

该脚本自动完成:
1. 编译 C++ 控制器 (cmake + make)
2. 启动 tmux 会话, 包含两个窗口:
   - `control`: 等待 5s 后启动 `droneMission`
   - `vision`: 启动 `detector_unified.py`

### 5.2 手动启动

```bash
# 终端 1: GStreamer CSI 推流 (如果 detector 不自动启动)
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
    nvvidconv ! jpegenc ! tcpserversink host=127.0.0.1 port=5000 &

# 终端 2: 视觉检测
conda activate yolov8_10
cd ~/drone_mission
python3 scripts/detector_unified.py

# 终端 3: 编译 + 控制
cd ~/drone_mission
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
./droneMission ../config
```

### 5.3 tmux 操作

```bash
tmux attach-session -t drone    # 连接到后台会话
Ctrl+B 1                        # 切换到 vision 窗口
Ctrl+B 0                        # 切换到 control 窗口
tmux kill-session -t drone      # 停止所有进程
```

---

## 六、GStreamer CSI 推流

### 6.1 手动启动

```bash
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
    nvvidconv ! jpegenc ! tcpserversink host=127.0.0.1 port=5000 &
```

| 参数 | 值 | 说明 |
|------|-----|------|
| `width/height` | 1280×720 | CSI 采集分辨率 |
| `framerate` | 30/1 | 帧率 |
| `port` | 5000 | TCP 端口, 与视觉脚本一致 |

### 6.2 CSI 相机调试

```bash
# 检查相机设备
ls /dev/video*

# 测试抓图
nvgstcapture-1.0 --mode=2 --capture-auto

# 测试推流
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink
```

---

## 七、飞行前检查清单

- [ ] FC 固件已烧录, 参数已校准 (加速度计, 罗盘, 水平)
- [ ] GPS 锁定 (3D Fix, HDOP < 2.0)
- [ ] 电池电量充足 (Jetson ≥ 11V, 动力电池 ≥ 标称电压)
- [ ] UART 接线正确, `/dev/ttyTHS1` 可读写
- [ ] CSI 相机安装牢固, 镜头朝下
- [ ] 舵机已连接并测试 (手动 PWM 测试)
- [ ] 模型文件 `yolo/best.pt` 存在且最新
- [ ] `config/mission_zones.yaml` 坐标正确
- [ ] `config/connection.yaml` 设为 `serial:///dev/ttyTHS1:57600`
- [ ] Jetson 风扇正常运转 (防止 thermal throttling)
- [ ] 安全开关 (Safety Switch) 可正常 arm/disarm
- [ ] 飞行区域无障碍物, 无人员
- [ ] 遥控器切到 Stabilize 可随时接管
- [ ] QGroundControl 已连接, 可监控状态
- [ ] SSH 连接正常 (备用紧急控制)
- [ ] 降落伞 / 应急装置已就位 (如有)

---

## 八、安全注意事项

1. **随时可接管**: 遥控器保持 Stabilize/AltHold 模式, 出问题立即切回手动。
2. **低空测试优先**: 首次实测先在 1-2m 低空验证视觉检测和控制逻辑。
3. **视距内飞行**: 始终保持无人机在视距内 (VLOS)。
4. **一个开关断所有**: 提前设定遥控器紧急熄火通道。
5. **逐步递进**: 先测悬停检测 → 再测定点投放 → 再测完整任务。

---

## 九、故障排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 无法连接 FC | UART 接线/权限/波特率 | 检查 `dmesg \| grep tty`, 验证 `serial://` URL |
| 无视频流 | CSI 排线松动/GStreamer 未启动 | `nvgstcapture-1.0` 测试, 检查 `/dev/video0` |
| 检测无输出 | 模型路径错误/YOLO 加载失败 | 检查 `yolo/best.pt` 存在, 检查 `detector_unified.py` 路径 |
| 管道通信失败 | FIFO 权限/已存在残留 | `rm /tmp/vision_pipe /tmp/altitude_pipe /tmp/mission_cmd`, 重启 |
| 舵机不动作 | FC 通道映射/PWM 范围 | QGC 中执行 `MAV_CMD_DO_SET_SERVO` 测试 |
| 温度过高降频 | Jetson Nano 过热 | 开启风扇 `sudo jetson_clocks`, 加散热片 |
| offboard 超时 | 数传延迟/心跳丢失 | 检查 `heartbeatTimeout`, 降低控制频率 |
