# DroneMission — 低空检测投放系统

> C++ MAVSDK 飞行控制 + Python YOLOv5 视觉检测 + Gazebo SITL 仿真

## 目录结构

```
drone_mission/
├── CMakeLists.txt                 # CMake 构建 (C++17, MAVSDK + yaml-cpp)
├── config/                        # 多模块 YAML 配置
│   ├── connection.yaml            # 飞控连接 (serial/UDP)
│   ├── flight.yaml                # 飞行高度
│   ├── mission_zones.yaml         # 投放区 + 侦察区航点
│   ├── pid.yaml                   # PID 参数
│   ├── pid_test.yaml              # 测试参数 + 着陆超时
│   ├── servo.yaml                 # 舵机 PWM
│   └── vision.yaml                # 视觉管道路径
├── include/                       # C++ 头文件
├── src/                           # C++ 源文件
├── scripts/                       # 全部脚本
│   ├── build.sh                   # 编译
│   ├── OpenSim.sh                 # 一键启动仿真 (Gazebo + SITL)
│   ├── detector_sim.py            # 仿真视觉检测
│   ├── detector_real.py           # 真机视觉检测 (Jetson)
│   ├── gz_gst_bridge.py           # Gazebo 相机 → GStreamer (TCP + 挂载点叠加)
│   ├── gz_camera_bridge.py        # Gazebo 相机 → OpenCV (备用)
│   ├── analyze.sh                 # 一键 PID 数据可视化
│   └── plot_pid_test.py           # CSV → 图表
├── docs/                          # 架构文档
│   ├── architecture.md            # 通信与数据处理详解
│   └── readingGuide.md            # 自顶向下阅读引导
├── StaticAnalysis/                # 飞行数据 + 可视化输出
└── build/                         # 编译产物
    └── droneMission
```

---

## 快速开始 (仿真)

### 前置依赖

```bash
# 系统包
sudo apt install cmake g++ libyaml-cpp-dev python3-pip

# MAVSDK (需从源码或 deb 安装)
# https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html

# Python 依赖
pip install torch numpy opencv-python pandas matplotlib

# YOLOv5 仓库 + 模型
git clone https://github.com/ultralytics/yolov5 ~/yolov5
# 将 best.pt 放入 ~/yolo_test/ 或修改 detector_sim.py 的 MODEL_PATH

# Gazebo + ArduPilot SITL 仿真环境 (参考 ardupilot_gazebo 仓库)
```

### 完整启动流程

```bash
# 终端 1: 启动仿真 (Gazebo + SITL)
cd ~/drone_mission
bash scripts/OpenSim.sh

# 终端 2: 视觉检测 (会自动启动相机桥接)
conda activate yolov8_10
python3 scripts/detector_sim.py

# 终端 3: 编译 + 任务控制
cd ~/drone_mission
bash scripts/build.sh          # 编译 (首次或修改代码后)
./build/droneMission ../config
```

`detector_sim.py` 启动时会自动 spawn `gz_gst_bridge.py` 子进程, 将 Gazebo 相机转为 TCP :5000 流, 退出时自动关闭。

### PID 数据分析

测试飞行后 CSV 自动写入 `StaticAnalysis/`, 运行:

```bash
bash scripts/analyze.sh
```

生成 `pid_test_left.png` / `pid_test_right.png` 可视化图表。

---

## 启动参数说明

### 控制程序 (`droneMission`)

```bash
./build/droneMission <config_dir>
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `<config_dir>` | `../config` | YAML 配置目录, 从 `build/` 运行时用 `../config`, 从项目根运行时用 `config` |

运行前修改 `config/connection.yaml` 切换仿真/真机:

```yaml
# 仿真 (SITL)
url: "udp://:14550"

# 真机 (Jetson UART → FC TELEM2)
url: "serial:///dev/ttyTHS1:57600"
```

### 视觉检测 (`detector_sim.py` / `detector_real.py`)

| 脚本 | 环境 | 图像源 | 自动启动相机桥接 |
|------|------|--------|:---:|
| `detector_sim.py` | 任意, conda `yolov8_10` | TCP `127.0.0.1:5000` | 是 (`gz_gst_bridge.py` 子进程) |
| `detector_real.py` | Jetson, conda `yolov8_10` | TCP `127.0.0.1:5000` | 否 (需手动启动 GStreamer CSI 推流) |

两个脚本内部参数:

| 参数 | 仿真值 | 真机值 | 说明 |
|------|--------|--------|------|
| `MODEL_PATH` | `~/yolo_test/best.pt` | `/home/hy/yolo_test/best.pt` | YOLO 模型路径 |
| `YOLOV5_REPO` | `~/yolov5` | `/home/hy/yolov5` | YOLOv5 仓库路径 |
| `PIPE_PATH` | `/tmp/vision_pipe` | `/tmp/vision_pipe` | 管道路径 |
| `IMG_SIZE` | 416 | 416 | YOLO 输入尺寸 |
| `CONF_THRESH` | 0.6 | 0.6 | 置信度阈值 |

### 相机桥接 (`gz_gst_bridge.py`, 仅仿真)

```bash
python3 scripts/gz_gst_bridge.py --world competition_task_random --tcp 5000 [--no-display] [--altitude 1.2]
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--world` | `competition_task_random` | Gazebo 世界名称, 与 topic 路径绑定 |
| `--tcp` | `5000` | TCP 推流端口 (视觉代码连接此端口) |
| `--no-display` | 否 | 关闭本地 autovideosink 窗口 |
| `--altitude` | `1.2` | 飞行高度 (m), 用于挂载点像素叠加 |

### 仿真启动 (`OpenSim.sh`)

`scripts/OpenSim.sh` 内部变量 (直接修改脚本或通过环境变量覆盖):

```bash
WORLD_FILE="$HOME/ardupilot_gazebo/worlds/competition_task_random.sdf"
WORLD_NAME="competition_task_random"
```

### 真机 GStreamer CSI 推流 (仅真机)

```bash
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
    nvvidconv ! jpegenc ! tcpserversink host=127.0.0.1 port=5000 &
```

| 参数 | 值 | 说明 |
|------|------|------|
| `width/height` | 1280×720 | CSI 采集分辨率 |
| `port` | 5000 | TCP 端口, 与视觉脚本一致 |

### 各端口的统一约定

| 端口/路径 | 仿真 | 真机 | 用途 |
|-----------|------|------|------|
| `udp://:14550` | ✓ | ✗ | MAVSDK 连接 SITL |
| `serial:///dev/ttyTHS1:57600` | ✗ | ✓ | MAVSDK 连接 FC |
| `tcp://127.0.0.1:5000` | ✓ | ✓ | 视觉代码拉取相机流 |
| `/tmp/vision_pipe` | ✓ | ✓ | 视觉→控制 管道通信 |
| `/tmp/mission_cmd` | ✓ | ✓ | 控制→视觉 状态通知 |

---

## Gazebo 模型修改 (添加相机)

本项目已为 `iris_with_standoffs` 添加了向下相机, 位于其 `base_link` 上。

### 文件位置

| 改什么 | 文件 |
|--------|------|
| **添加 sensor** | `ardupilot_gazebo/models/iris_with_standoffs/model.sdf` |
| **验证 sensor** | `ardupilot_gazebo/worlds/competition_task_random.sdf` |

### 当前相机的定义 (已存在于 iris_with_standoffs/model.sdf 的 base_link 内)

在 `<link name='base_link'>` 中, 所有 `<visual>` 之后、`</link>` 之前的这段代码即相机:

```xml
<sensor name="downward_camera" type="camera">
  <pose degrees="true">0.15 0 -0.08 0 90 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>640</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>200</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### 迁移到其他模型的步骤

**第 1 步** — 打开目标模型的 `.sdf` 文件 (如 `ardupilot_gazebo/models/你的模型/model.sdf`)

**第 2 步** — 找到 `<link name='base_link'>` 块, 在它的 `</link>` 结束标签**前一行**粘贴上面的 `<sensor>` 块。确保 sensor 在 link 内部, 不在 link 之间。

**第 3 步** — 如果世界文件还没有 Sensors 渲染插件, 在 `<world>` 下的 `<plugin>` 区添加 (本项目已有):

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

**第 4 步** — 重新启动 Gazebo:
```bash
gz sim -v4 -r ardupilot_gazebo/worlds/competition_task_random.sdf
```

**第 5 步** — 验证相机是否正常工作:
```bash
# 列出所有话题, 搜索 camera
gz topic -l | grep camera

# 应看到类似输出:
# /world/competition_task_random/model/iris_with_ardupilot/.../sensor/downward_camera/image
```

### 参数含义

| SDF 参数 | 作用 | 改了这里 → 同步改代码 |
|----------|------|----------------------|
| `<pose>` 前三个数 `x y z` | 相机在机体上的安装位置 (前/右/下, 米) | `computeMountPixels()` 中的 `camDx, camDy` |
| `<pose>` 后三个数 `r p y` | 相机姿态角 (度). `0 90 0` = 镜头朝下 | 影响投影公式中的坐标轴方向 |
| `<horizontal_fov>` | 水平视场角 (弧度). `1.047` = 60° | `FX = WIDTH/2 / tan(FOV/2)` |
| `<width>`, `<height>` | 分辨率 (像素) | `gz_gst_bridge.py`: `WIDTH`, `HEIGHT`<br>`computeMountPixels()`: `fx`, `cx`, `cy` |
| `<update_rate>` | 帧率 (Hz) | `gz_gst_bridge.py`: `FPS` |
| `<near>` / `<far>` | 裁剪面 (米) | `far` 需 > 最大飞行高度 |
| `<format>` | 像素格式. 必须 `R8G8B8` | 不可改, 代码依赖此格式 |

### 修改相机后需同步的代码位置

修改了分辨率 / FOV 后, 需要在以下两处同步更新:

**1. `scripts/gz_gst_bridge.py` (第 27-31 行)**
```python
WIDTH, HEIGHT, FPS = 640, 640, 30
FX = WIDTH / 2.0 / np.tan(np.deg2rad(FOV_DEG) / 2)
CX, CY = WIDTH / 2.0, HEIGHT / 2.0
```

**2. `src/missionStateMachine.cpp` — `computeMountPixels()` 函数**
```cpp
const double fx = 640.0 / 2.0 / tan(60.0 * M_PI / 180.0 / 2.0);  // ≈554
const double cx = 640.0 / 2.0, cy = 640.0 / 2.0;
```

**3. `scripts/detector_sim.py` 的 `IMG_SIZE`** — 如果分辨率变了, YOLO 输入尺寸可能需要调整。

### 挂载点和相机偏移的来源

这些物理参数来自 `/home/limile/dxy_apm_ws/` 的原始配置:

| 参数 | 值 (m) | 来源文件 |
|------|--------|----------|
| 相机 X 偏移 | `0.15` | Gazebo 模型 `<pose>` 的 x |
| 左挂载点 | `(-0.07, 0.001)` | `can_config.yaml` → `shot_target_x_l/y_l` |
| 右挂载点 | `(0.07, -0.001)` | `can_config.yaml` → `shot_target_x_r/y_r` |
| 桶物理半径 | `0.10` | 标准桶半径 |

如果需要匹配真实硬件, 修改 `computeMountPixels()` 和 `gz_gst_bridge.py` 中对应的常量即可。

---

## 配置说明

所有参数通过 `config/` 下 7 个 YAML 文件管理, 按功能拆分:

| 文件 | 关键参数 |
|------|----------|
| `connection.yaml` | `url` (串口/UDP), `heartbeatTimeout` |
| `flight.yaml` | `takeoffAlt` (3m), `cruiseAlt` (5m), `dropAlt` (1.5m) |
| `mission_zones.yaml` | 投放区中心/桶数, 侦察区航点 |
| `pid.yaml` | `pidXY` (PI), `pidZ` (完整), `pidVisual` (视觉伺服) |
| `pid_test.yaml` | `testAlt` (1.2m), 搜索偏移, 收敛阈值, 超时 |
| `servo.yaml` | 舵机通道, PWM 值, 释放时长 |
| `vision.yaml` | 管道路径, 图像尺寸, 目标类别 |

---

## 状态机流程图

```
arming → takeoff → transitToDrop(3m)
                        ↓
                   dropSearch ────(90s超时/3航点无桶)──→ transitToRecon(5m)
                   │      ↓                                        ↓
                   │ 检测到桶                                  reconScan(5航点)
                   │      ↓                                        ↓
                   └→ dropVisualServo(40s) ────────────→ rtl(3.5m)
                                                              ↓
                                                           landed
```

---

## 挂载点映射

物理参数 (来自 `dxy_apm_ws`):

| 参数 | 值 |
|------|-----|
| 相机在机体偏移 | (0.15, 0, -0.08) m |
| 左挂载点 | (-0.07, 0.001) m |
| 右挂载点 | (0.07, -0.001) m |
| 桶物理半径 | 0.10 m |

像素投影 (仿真相机 fx=554.26, 640×640):
```
u = 320 + 554 * (mount_x - 0.15) / altitude
r = 0.10 * 554 / altitude
```

---

## 管道协议

| 方向 | 路径 | 格式 | 字节 |
|------|------|------|------|
| Python → C++ | `/tmp/vision_pipe` | `struct.pack('fff', cx, cy, confidence)` | 12 |

C++ 端通过 `binaryVisionPipe` 类阻塞打开 + `fcntl` 非阻塞排空读取。
