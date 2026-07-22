# DroneMission — 真机版 (Real Flight)

> C++ MAVSDK 飞行控制 + Python YOLOv5 视觉检测 + Jetson Nano 机载计算

**本分支 (`real`) 为真机飞行代码。** 仿真代码请切换到 `main` 分支。

---

## 目录结构

```
drone_mission/
├── CMakeLists.txt                      # CMake 构建 (C++17, MAVSDK + yaml-cpp)
├── launch_vision_real.sh               # 真机一键启动 (tmux 双窗口)
├── config/                             # 多模块 YAML 配置
│   ├── connection.yaml                 # serial:///dev/ttyTHS1:57600 (真机默认)
│   ├── flight.yaml                     # 飞行高度
│   ├── mission_zones.yaml              # 投放区 + 侦察区航点
│   ├── pid.yaml                        # PID 参数
│   ├── pid_test.yaml                   # 测试参数 + 着陆超时
│   ├── servo.yaml                      # 舵机 PWM
│   └── vision.yaml                     # 视觉管道路径
├── include/                            # C++ 头文件
│   ├── comm/                           # 飞控通信 (droneLink, offboardControl, flightOps, servoControl)
│   ├── control/                        # PID 控制器
│   ├── vision/                         # 视觉管道 (visionInterface)
│   ├── mission/                        # 任务层 (状态机, 配置)
│   └── bomb/                           # 投放系统 (坐标映射, Kalman, 跟踪, 投放)
├── src/                                # C++ 源文件 (镜像 include/ 结构)
├── scripts/                            # 全部脚本
│   ├── build.sh                        # 编译
│   ├── detector_unified.py             # 真机统一视觉检测 (桶+H标识)
│   ├── detector_real.py                # 真机桶检测 (备用)
│   ├── gz_gst_bridge.py                # Gazebo → GStreamer (仅仿真)
│   ├── detector_sim.py                 # 仿真视觉检测 (仅仿真)
│   ├── gz_camera_bridge.py             # Gazebo → OpenCV (仿真备用)
│   ├── OpenSim.sh                      # 一键启动仿真 (仅仿真)
│   ├── analyze.sh                      # PID 数据可视化
│   └── plot_pid_test.py                # CSV → 图表
├── yolo/                               # YOLO 模型文件
│   ├── best.pt                         # 桶检测模型
│   └── best_H.pt                       # H 标识检测模型 (侦察用)
└── docs/                               # 文档
    ├── real_hardware.md                # 真机部署指南 (接线/CSI/环境)
    ├── architecture.md                 # 系统架构
    ├── state_machine.md                # 状态机详解
    ├── mapping_algorithm.md            # 建图算法与调试
    ├── code_review_guide.md            # Code Review 要点
    └── reading_guide.md                # 代码阅读引导
```

---

## 真机快速开始

### 硬件

| 组件 | 型号 | 用途 |
|------|------|------|
| 飞控 | ArduPilot (Cube Orange / Pixhawk) | 飞行控制 |
| 机载计算 | Jetson Nano | 视觉推理 + MAVSDK 任务控制 |
| 相机 | IMX219 CSI-2 | 下视桶检测 |
| 数传 | 433/915MHz 数传 | QGC 地面站监控 |

### 前置依赖

```bash
# Jetson Nano 系统
sudo apt install cmake g++ libyaml-cpp-dev python3-pip tmux
pip install torch torchvision numpy opencv-python pandas matplotlib

# MAVSDK (从源码或 deb 安装)
# https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html

# YOLOv5
git clone https://github.com/ultralytics/yolov5 /home/hy/yolov5

# UART 权限
sudo usermod -a -G dialout $USER && sudo reboot
```

### 一键启动

```bash
cd ~/drone_mission
bash launch_vision_real.sh              # 无头模式 (无显示器)
bash launch_vision_real.sh --display    # 带本地显示器
```

该脚本自动完成编译 + tmux 双窗口启动 (`vision` 窗口 + `control` 窗口)。

### 手动启动

```bash
# 终端 1: GStreamer CSI 推流 (如需手动)
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
    nvvidconv ! jpegenc ! tcpserversink host=127.0.0.1 port=5000 &

# 终端 2: 视觉检测
conda activate yolov8_10
python3 scripts/detector_unified.py

# 终端 3: 控制
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
./droneMission ../config
```

### tmux 操作

```bash
tmux attach-session -t drone    # 连接会话
Ctrl+B 1                        # 切换到 vision 窗口
Ctrl+B 0                        # 切换到 control 窗口
tmux kill-session -t drone      # 停止
```

### 飞行前检查清单

- [ ] 硬件接线正确 (UART, CSI, 舵机)
- [ ] GPS 锁定, 电池充足
- [ ] 模型文件 `yolo/best.pt` 存在
- [ ] `config/connection.yaml` 为 `serial:///dev/ttyTHS1:57600`
- [ ] 遥控器切换到 Stabilize 模式可随时接管
- [ ] 低空 (1-2m) 先测试视觉检测
- [ ] **完整硬件部署指南见 `docs/real_hardware.md`**

---

## 配置说明

| 文件 | 关键参数 | 真机注意事项 |
|------|----------|-------------|
| `connection.yaml` | `url: "serial:///dev/ttyTHS1:57600"` | **真机默认**, 仿真改为 `udp://:14550` |
| `flight.yaml` | `takeoffAlt`, `cruiseAlt`, `dropAlt` | 根据场地调整 |
| `mission_zones.yaml` | 投放区中心/桶数, 侦察区航点 | 填入实际 GPS 坐标 |
| `pid.yaml` | `pidXY` (PI), `pidZ` (完整), `pidVisual` | 真机与仿真 PID 可能不同, 需调参 |
| `pid_test.yaml` | 测试高度, 搜索偏移, 收敛阈值, 超时 | 低空测试后微调 |
| `servo.yaml` | 舵机通道, PWM 值, 释放时长 | 确认 FC 通道映射正确 |
| `vision.yaml` | 管道路径, 图像尺寸, 目标类别 | — |

---

## 管道通信协议

| 方向 | 路径 | 格式 | 用途 |
|------|------|------|------|
| Python → C++ | `/tmp/vision_pipe` | `[count:u8] [id:u8] [cx:f32] [cy:f32]...` | 桶检测结果 |
| Python → C++ | `/tmp/recon_pipe` | 文本: `color_type x_ratio` | 侦察区色块 |
| Python → C++ | `/tmp/h_pipe` | 文本: `hx hy` | H 标识位置 |
| C++ → Python | `/tmp/altitude_pipe` | 文本: 高度值 (m) | 实时高度 |
| C++ → Python | `/tmp/mission_cmd` | 文本: 状态名 | 任务状态通知 |

### vision_pipe 二进制格式

```
[count: uint8] [id: uint8] [cx: float32] [cy: float32]  (每桶重复)
总长度 = 1 + count × 9 字节
count=0 时只有 1 字节
```

---

## 端口与连接约定

| 端口/路径 | 真机 | 用途 |
|-----------|------|------|
| `serial:///dev/ttyTHS1:57600` | ✓ | MAVSDK 连接 FC TELEM2 |
| `tcp://127.0.0.1:5000` | ✓ | 视觉代码拉取 CSI 流 |
| `/tmp/vision_pipe` | ✓ | 视觉→控制 管道通信 |
| `/tmp/mission_cmd` | ✓ | 控制→视觉 状态通知 |
| `/tmp/altitude_pipe` | ✓ | 控制→视觉 高度数据 |

---

## 任务状态机

```
ARMING → TAKEOFF → TRANSIT_TO_DROP → DROP_SEARCH → TRANSIT_TO_RECON → RECON_SCAN → RTL → LANDED
                       (3m)              (90s max)      (12s)          (5航点)    (25s,4m)
```

投放子系统 (BombDropSystem):

```
SCAN → SELECT → GOTO → TRACKING → PREDICT → DROP → CLIMB
(8s)              (1.4m)   (1.0m)            ↓       (3.5m)
                                    (五条件: err<pix, vel<tol, alt<tol, stable>0.5s)
```

---

## 安全注意事项

1. **遥控器随时可接管**: 保持 Stabilize/AltHold 模式待命
2. **低空测试优先**: 首次 1-2m 验证检测 + 控制
3. **视距内飞行 (VLOS)**: 始终在可见范围内
4. **逐步递进**: 悬停检测 → 定点投放 → 完整任务
5. **飞行前完整检查**: 见 `docs/real_hardware.md` 检查清单

---

## 真机 GStreamer CSI 推流

```bash
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
    nvvidconv ! jpegenc ! tcpserversink host=127.0.0.1 port=5000 &
```

`detector_unified.py` 从 `tcp://127.0.0.1:5000` 拉流进行 YOLO 推理。

---

## 视觉检测脚本

| 脚本 | 环境 | 功能 |
|------|------|------|
| `detector_unified.py` | Jetson 真机 (推荐) | 全模式: 桶检测 + 侦察 + H 识别, 根据 mission_cmd 自动切换 |
| `detector_real.py` | Jetson 真机 (备用) | 仅桶检测 (投放区专用) |
| `detector_sim.py` | 仿真 | 仅桶检测 + 自动启动 gz_gst_bridge |

**detector_unified.py** 是真机推荐脚本, 内部参数:

| 参数 | 值 | 说明 |
|------|-----|------|
| `BUCKET_MODEL_PATH` | `../yolo/best.pt` | 桶检测模型 |
| `H_MODEL_PATH` | `../yolo/best_H.pt` | H 标识检测模型 |
| `YOLOV5_REPO` | `/home/hy/yolov5` | YOLOv5 仓库路径 |
| `STREAM_URL` | `tcp://127.0.0.1:5000` | GStreamer CSI 推流地址 |
| `IMG_SIZE` | 416 | YOLO 输入尺寸 |
| `CONF_THRESH` | 0.6 | 桶检测置信度 |
| `H_CONF_THRESH` | 0.5 | H 标识检测置信度 |

---

## 相机参数

### 真机 IMX219 (1280×720 binned, pixel=2.24µm)

| 参数 | 值 |
|------|-----|
| 焦距 f | 3.04mm |
| fx/fy | 1357.0 |
| cx | 640.0 |
| cy | 360.0 |
| 安装偏移 (base_link) | (0.15, 0, -0.08) m |

修改后需同步:
- `scripts/detector_unified.py` 中的 `FX, FY, CX, CY`
- `scripts/detector_real.py` 中的 `FX, FY, CX, CY`
- `src/bomb/coordinateMapper.cpp` 中的 `fx, cx, cy`
- `src/mission/missionStateMachine.cpp` 中的 `computeMountPixels()`

---

## 物理参数 (挂载点与相机偏移)

| 参数 | 值 (m) | 来源 |
|------|--------|------|
| 相机 X 偏移 | 0.15 | 安装位置 |
| 左挂载点 | (-0.07, 0.001) | 舵机安装 |
| 右挂载点 | (0.07, -0.001) | 舵机安装 |
| 桶物理半径 | 0.10 | 标准桶 |

---

## 文档索引

| 文档 | 内容 |
|------|------|
| `docs/real_hardware.md` | **真机部署指南**: 接线、CSI、GStreamer、飞行前检查清单、故障排查 |
| `docs/architecture.md` | 系统架构、分层、数据流 |
| `docs/state_machine.md` | 全部状态机 (顶层 + 投放子系统 + TargetTracker) |
| `docs/mapping_algorithm.md` | 建图算法详解、调试方法、误差分析 |
| `docs/code_review_guide.md` | 逐文件审查要点、超时检查表、日志速查 |
| `docs/reading_guide.md` | 自顶向下代码阅读路线 |

---

## 快速修改参考

| 修改什么 | 改哪里 |
|----------|--------|
| 飞行高度 | `config/flight.yaml` |
| 投放区/侦察区坐标 | `config/mission_zones.yaml` |
| PID 参数 | `config/pid.yaml` |
| 舵机 PWM | `config/servo.yaml` |
| 状态机流控 | `src/mission/missionStateMachine.cpp` |
| 投放策略 | `src/bomb/bombDropSystem.cpp` |
| 目标容错参数 | `src/bomb/targetTracker.cpp` |
| 管道协议 | `src/vision/visionInterface.cpp` |
| 相机参数 | `src/bomb/coordinateMapper.cpp` + `scripts/detector_unified.py` |
| FC 连接 | `config/connection.yaml` |

---

## 仿真说明

> 本分支为真机代码。如需仿真, 请切换到 `main` 分支。

仿真 `main` 分支使用:
- `config/connection.yaml` → `udp://:14550`
- `scripts/OpenSim.sh` 启动 Gazebo + SITL
- `scripts/detector_sim.py` + `gz_gst_bridge.py`
