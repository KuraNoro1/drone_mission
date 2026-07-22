# DroneMission — 低空检测投放系统

> C++ MAVSDK 飞行控制 + Python YOLOv5 视觉检测 + Gazebo SITL 仿真

## 目录结构

```
drone_mission/
├── CMakeLists.txt                 # CMake 构建 (C++17, MAVSDK + yaml-cpp)
├── config/                        # 多模块 YAML 配置
│   ├── connection.yaml            # 飞控连接 (serial/UDP)
│   ├── flight.yaml                # 飞行高度
│   ├── mission_zones.yaml         # 投放区 + 侦察区航点 + 优先级
│   ├── pid.yaml                   # PID 参数
│   ├── servo.yaml                 # 舵机 PWM
│   ├── vision.yaml                # 视觉管道路径
│   └── camera.yaml                # 相机内参与外参
├── include/                       # C++ 头文件
│   ├── comm/                      # 飞控通信层
│   ├── control/                   # 控制器 (PID)
│   ├── vision/                    # 视觉管道
│   ├── mission/                   # 任务层
│   └── bomb/                      # 投放子系统
├── src/                           # C++ 源文件 (镜像 include/ 结构)
├── scripts/                       # 全部脚本
│   ├── build.sh                   # 编译
│   ├── run_mission.sh             # 真机一键启动 (tmux: control + vision)
│   ├── OpenSim.sh                 # 一键启动仿真 (Gazebo + SITL)
│   ├── detector_unified.py        # 统一视觉检测 (仿真/真机, 多模式切换)
│   ├── detector_sim.py            # 仿真视觉检测 (独立版)
│   ├── detector_real.py           # 真机视觉检测 (独立版)
│   ├── detector_h.py              # H 标识检测 (着舰引导)
│   ├── detector_recon.py          # 侦察颜色分析 (HSV 桶内颜色识别)
│   ├── gz_gst_bridge.py           # Gazebo 相机 → GStreamer (TCP + 挂载点叠加)
│   ├── analyze.sh                 # 一键 PID 数据可视化
│   └── plot_pid_test.py           # CSV → 图表
├── launch_vision_sim.sh           # 仿真视觉一键启动
├── launch_vision_real.sh          # 真机一键启动 (自动检测连接模式)
├── docs/                          # 架构文档
│   ├── architecture.md            # 通信与数据处理详解
│   ├── state_machine.md           # 全部状态机详解
│   ├── mapping_algorithm.md       # 建图算法详解与调试指南
│   ├── code_review_guide.md       # Code Review 逐文件审查要点
│   └── reading_guide.md           # 自顶向下阅读引导
├── TestImgs/                      # 检测帧截图存档 (按日期/任务组织)
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

# 终端 2: 统一视觉检测 (自动启动相机桥接, 根据任务阶段切换检测模式)
conda activate yolov8_10
python3 scripts/detector_unified.py --sim

# 终端 3: 编译 + 任务控制
cd ~/drone_mission
bash scripts/build.sh          # 编译 (首次或修改代码后)
./build/droneMission ../config
```

`detector_unified.py --sim` 是一个**统一的视觉检测进程**，启动时自动 spawn `gz_gst_bridge.py` 子进程，根据 C++ 端通过 `/tmp/mission_cmd` 发送的任务状态，在投放检测 / 侦察颜色分析 / H着舰检测三种模式间自动切换，无需手动切换脚本。

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

### 视觉检测

| 脚本 | 环境 | 图像源 | 说明 |
|------|------|------|------|
| `detector_unified.py` | 仿真/真机 | TCP :5000 | **推荐**。统一进程, 根据 mission_cmd 在投放/侦察/H着舰三模式间自动切换, 动态加载/卸载模型节省 GPU 显存 |
| `detector_sim.py` | 仿真, conda `yolov8_10` | TCP :5000 | 独立版投放检测, 会自动启动 `gz_gst_bridge.py` 子进程 |
| `detector_real.py` | Jetson, conda `yolov8_10` | TCP :5000 | 独立版投放检测, 需外部 GStreamer CSI 推流 |
| `detector_h.py` | 任意 | TCP :5000 | H 标识检测, 加载 `best_H.pt` 模型, 输出到 `/tmp/h_pipe` |
| `detector_recon.py` | 任意 | TCP :5000 | 侦察颜色分析, YOLO + HSV 桶内颜色识别, 输出到 `/tmp/recon_pipe` |


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

### 统一检测器 (`detector_unified.py`) 参数

```bash
python3 scripts/detector_unified.py [--sim] [--display]
```

| 参数 | 说明 |
|------|------|
| `--sim` | 仿真模式: 自动启动 `gz_gst_bridge.py`, 使用 640×640 相机内参 |
| `--display` | 开启检测画面显示 (Jetson 默认关闭以节省资源) |

内部参数 (脚本顶部常量):

| 参数 | 仿真值 | 真机值 | 说明 |
|------|--------|--------|------|
| `MODEL_PATH` | `~/yolo_test/best.pt` | `/home/hy/yolo_test/best.pt` | 桶检测 YOLO 模型 |
| `H_MODEL_PATH` | `~/yolo_test/best_H.pt` | `/home/hy/yolo_test/best_H.pt` | H 标识检测 YOLO 模型 |
| `YOLOV5_REPO` | `~/yolov5` | `/home/hy/yolov5` | YOLOv5 仓库路径 |
| `IMG_SIZE` | 640 | 1280 | 图像采集分辨率 |
| `CONF_THRESH` | 0.6 | 0.6 | 置信度阈值 |
| `FX / FY` | 554.26 | 1357.0 | 相机焦距 (像素) |

### 各端口的统一约定

| 端口/路径 | 仿真 | 真机 | 用途 |
|-----------|------|------|------|
| `udp://:14550` | ✓ | ✗ | MAVSDK 连接 SITL |
| `serial:///dev/ttyTHS1:57600` | ✗ | ✓ | MAVSDK 连接 FC |
| `tcp://127.0.0.1:5000` | ✓ | ✓ | 视觉代码拉取相机流 |
| `/tmp/vision_pipe` | ✓ | ✓ | 投放多桶检测结果 (二进制) |
| `/tmp/h_pipe` | ✓ | ✓ | H 标识着舰检测结果 |
| `/tmp/recon_pipe` | ✓ | ✓ | 侦察颜色分析结果 |
| `/tmp/mission_cmd` | ✓ | ✓ | 控制→视觉 状态通知 |
| `/tmp/altitude_pipe` | ✓ | ✓ | C++ 高度 → Python 桶尺寸分类 |

### 一键启动脚本

| 脚本 | 环境 | 说明 |
|------|------|------|
| `launch_vision_sim.sh` | 仿真 | 启动 `detector_unified.py --sim --display` |
| `launch_vision_real.sh` | Jetson 真机 | 自动检测连接模式, 启动 tmux (Mission + Vision 双窗口) |
| `scripts/run_mission.sh` | Jetson 真机 | 编译 + tmux 双窗口 (C++ 控制 + 统一视觉) |

### 真机 GStreamer CSI 推流 (仅真机)

```bash
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
    nvvidconv ! jpegenc ! tcpserversink host=127.0.0.1 port=5000 &
```

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

修改了分辨率 / FOV / 安装位置后, 需要同步更新以下位置:

| 修改内容 | 同步位置 |
|----------|----------|
| 分辨率 / FOV | `config/camera.yaml` (内参 fx/fy/cx/cy) |
| 相机安装位置 | `config/camera.yaml` (外参 offsetForward/offsetRight/offsetDown) |
| 图像分辨率 | `scripts/gz_gst_bridge.py` — `WIDTH, HEIGHT` |
| YOLO 输入 | `scripts/detector_*.py` — `IMG_SIZE` |

`config/camera.yaml` 是统一的相机参数来源, C++ 端通过 `parseCamera()` 加载。Python 端各自维护内参常量。

---

## 配置说明

所有参数通过 `config/` 下 7 个 YAML 文件管理, 按功能拆分:

| 文件 | 关键参数 |
|------|----------|
| `connection.yaml` | `url` (串口/UDP), `heartbeatTimeout` |
| `flight.yaml` | `takeoffAlt` (3m), `cruiseAlt` (5m), `dropAlt` (1.8m), `rtlTimeout` |
| `mission_zones.yaml` | 投放区 forwardDistance + 优先级, 侦察区 forwardDistance + 航点 + hoverTime |
| `pid.yaml` | 视觉伺服 PID (kp/ki/kd), 速度限制, 丢失检测阈值, 收敛容差, 降落参数 |
| `servo.yaml` | 舵机通道, PWM 值, 释放时长 |
| `vision.yaml` | 管道路径, 图像尺寸, 目标类别 |
| `camera.yaml` | 相机内参 (fx/fy/cx/cy), 外参偏移 (forward/right/down), 仿真/真机双配置 |

---

## 状态机流程图

```
arm → takeoff → transitToDrop(3m)
                       ↓
                  dropSearch(90s)
                       │
                  BombDropSystem:
                  SCAN(8s) → SELECT → GOTO(1.2m) → TRACKING → PREDICT → DROP → CLIMB
                       │                         (REACQUIRE 恢复)
                       ↓
                  稳定爬升 3.5m → transitToRecon(5m)
                                            ↓
                                       reconScan(5航点, 每点3s)
                                            ↓
                                       rtl(25s,4m)
                                            │
                                    H-guided 着舰 ──→ landed
```

**关键变化**: 投放阶段完全委派给 `BombDropSystem`, 其内部包含完整的 7 阶段状态机 (SCAN → SELECT → GOTO → TRACKING → PREDICT → DROP → CLIMB)。丢失时有 REACQUIRE 三阶段恢复策略 (Hover → Spiral → Climb)。RTL 阶段支持 H 标识视觉引导着舰。

---

## 管道协议

系统使用 4 个命名管道 (FIFO) 进行 C++ ↔ Python 进程间通信:

| 方向 | 路径 | 格式 | 用途 |
|------|------|------|------|
| Python → C++ | `/tmp/vision_pipe` | 二进制: `[count:1B] [id:1B cx:4B cy:4B] × count` | 投放阶段多桶检测结果 |
| Python → C++ | `/tmp/h_pipe` | 文本: `"cx,cy"` 或 `"None"` | H 标识着舰检测 |
| Python → C++ | `/tmp/recon_pipe` | 文本: `"B1:red:45.2,green:30.1;..."` 或 `"None"` | 侦察颜色分析 |
| C++ → Python | `/tmp/mission_cmd` | 文本: 状态名 (如 `"DROP_SEARCH"`) | 通知视觉脚本当前任务阶段, 切换检测模式 |
| C++ → Python | `/tmp/altitude_pipe` | 文本: 高度值 (m) | 当前飞行高度, 用于桶尺寸分类 |

`detector_unified.py` 读取 `/tmp/mission_cmd` 自动在三种检测模式间切换, 并动态加载/卸载 YOLO 模型以节省 GPU 显存。
