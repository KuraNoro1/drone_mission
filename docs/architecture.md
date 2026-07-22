# DroneMission 系统架构

> 2026-07-19 | 当前实现版本 v3

---

## 一、总体架构

```
┌────────────────────────────────────────────────────────────────────────────────────┐
│                           仿真环境 (SITL)                                            │
│                                                                                    │
│  Gazebo 世界 ←→ gz_gst_bridge → TCP :5000 → detector_unified.py (YOLO, 多模式)    │
│       │                                                    │   │   │   │          │
│  iris SITL (ArduPilot)                              /tmp/vision_pipe │   │   │      │
│       │                                              /tmp/h_pipe ←──┘   │   │      │
│       │                                              /tmp/recon_pipe ←──┘   │      │
│       │                                                    │               │      │
│       ├── UDP :14550 → MAVSDK → droneMission (C++) ←───────┘               │      │
│       │                       → /tmp/mission_cmd → detector_unified.py ────┘      │
│       │                       → /tmp/altitude_pipe → detector_unified.py          │
│       └── TCP :5760 → QGroundControl                                               │
└────────────────────────────────────────────────────────────────────────────────────┘
```

**真机**：CSI 相机 → nvarguscamerasrc → TCP :5000 → detector_unified.py，FC TELEM2 → UART → MAVSDK。

**统一检测器 (`detector_unified.py`)** 是推荐的单进程视觉方案：根据 `/tmp/mission_cmd` 中的任务状态，在投放检测 / 侦察颜色分析 / H 着舰检测三种模式间自动切换，并动态加载/卸载模型以节省 GPU 显存。

---

## 二、软件分层

```
┌──── main.cpp ────────────────────────────────────┐
│  配置加载 → droneLink 连接 → 状态机运行            │
└─────────────┬────────────────────────────────────┘
              │
┌──── missionStateMachine ─────────────────────────┐
│  顶层状态机: ARM → TAKEOFF → TRANSIT → DROP →    │
│  RECON → RTL → LANDED                            │
│                                                   │
│  handleDropSearch() → BombDropSystem              │
└─────────────┬────────────────────────────────────┘
              │ (投放阶段委派)
┌──── BombDropSystem ──────────────────────────────┐
│  SCAN → SELECT → GOTO → TRACKING → PREDICT →     │
│  DROP → CLIMB                                     │
│                                                   │
│  使用: TargetTracker / CoordinateMapper / PID     │
└──────┬──────────┬──────────◇──────────────────────┘
       │          │
  TargetTracker  CoordinateMapper
   (Kalman +       (像素→世界)
    状态机)
```

---

## 三、核心模块

### 3.1 通信层

| 模块 | 文件 | 职责 |
|------|------|------|
| `droneLink` | `droneLink.h/cpp` | MAVSDK 封装：遥测、Action、Offboard、伺服 PWM |
| `offboardControl` | `offboardControl.h/cpp` | 位置模式/速度模式管理 |
| `flightOps` | `flightOps.h/cpp` | arm/takeoff/land/RTL |
| `servoControl` | `servoControl.h/cpp` | MAV_CMD_DO_SET_SERVO 释放载荷 |
| `visionInterface` | `visionInterface.h/cpp` | 多 FIFO 管道: `multiBucketPipe` (二进制多桶), `hDetectionPipe` (H着舰, 线程), `reconPipe` (侦察颜色, 线程), `missionCmdPipe` (状态写) |

### 3.2 投放系统 (新架构 v3)

| 模块 | 文件 | 职责 |
|------|------|------|
| `BombDropSystem` | `bombDropSystem.h/cpp` | 投放阶段状态机：扫描→选目标→导航→跟踪→投弹 |
| `TargetTracker` | `targetTracker.h/cpp` | 目标容错跟踪：VISIBLE/LOST_SHORT/LOST_LONG/LOST_CRITICAL，Kalman 滤波，commit 机制 |
| `CoordinateMapper` | `coordinateMapper.h/cpp` | 像素坐标→世界坐标映射，地平面求交 |
| `KalmanFilter2D` | `kalmanFilter.h/cpp` | 4状态 (x,y,vx,vy) 常速度卡尔曼滤波 |

### 3.3 控制层

| 模块 | 文件 | 职责 |
|------|------|------|
| `pidController` | `pidController.h/cpp` | 通用 PI/PID，抗积分饱和 |
| `dualLoopPid` | `dualLoopPid.h/cpp` | 双环位置 P 控制器 |

---

## 四、数据流

```
detector_unified.py (根据 mission_cmd 切换模式)
     │
     ├─ DROP 模式: YOLO 多桶检测 → /tmp/vision_pipe (二进制协议)
     │      multiBucketPipe::readLatest()
     │         │
     │     ┌───┘
     │     │  SCAN阶段: 像素聚类 → pixelToWorld() → 目标地图 (按优先级排序)
     │     │  GOTO/TRACKING阶段: TargetTracker.update() → Kalman 预测/更新
     │     │     ├── 未commit: Pixel PID (errPx → vx,vy)
     │     │     └── 已commit: World PID (errN,errE → vx,vy)
     │     │
     │     └── offboardControl::setVelocityNed() → MAVSDK → ArduPilot
     │
     ├─ RECON 模式: YOLO + HSV → /tmp/recon_pipe (文本, 颜色比例)
     │
     └─ RTL 模式: YOLO H 检测 → /tmp/h_pipe → H-guided 着舰

/tmp/altitude_pipe: C++ altitude → Python (桶尺寸分类)
/tmp/mission_cmd: C++ state → Python (切换检测模式)
```

---

## 五、管道协议

### `/tmp/vision_pipe` (二进制, 投放阶段)

```
[count: uint8] [id: uint8] [cx: float32] [cy: float32]  (每桶重复, 最多 count=10)
总长度 = 1 + count × 9 字节
count=0 时只有 1 字节 (表示无检测)
```

C++ 端通过 `multiBucketPipe` 类非阻塞读取, 跳过 count=0 帧。

### `/tmp/h_pipe` (文本, 着舰阶段)

```
"cx,cy"  或  "None"
每行一个结果, 线程后台读取, mutex 保护最新数据
```

### `/tmp/recon_pipe` (文本, 侦察阶段)

```
"B1:red:45.2,green:30.1;B2:blue:60.0;..."  或  "None"
格式: B<编号>:<颜色>:<占比>;..., 按占比降序
```

### `/tmp/mission_cmd` (文本, C++ → Python)

```
状态名 (换行分隔), 如 "DROP_SEARCH", "RECON_SCAN", "RTL"
```

### `/tmp/altitude_pipe` (文本, C++ → Python)

```
高度值 (m, 换行分隔), 2Hz
```

---

## 六、关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 搜索高度 | 3.5m | SCAN 阶段悬停高度 |
| 靠近高度 | 1.2m | GOTO 飞到目标上方高度 |
| 投弹高度 | 1.8m | 触发放弹的高度 |
| 投弹区超时 | 90s | 进入投放区总时限 |
| 侦察高度 | 5.0m | 侦察航点巡航高度 |
| 侦察悬停 | 3.0s | 每航点悬停时间 |
| 返航超时 | 120s | RTL 总超时 |
| 28m 距离滤波 | 28m | 忽略起飞点 28m 内的检测 (滤除起飞点误检) |
| Kalman 过程噪声 | 0.01 m²/s | Q |
| Kalman 观测噪声 | 0.05 m² | R |
| 像素聚类半径 | 50px | SCAN 阶段聚类阈值 |
| 连续丢失确认帧 | 4 | 丢失才切换 LOST_SHORT 状态 |
| 严重丢失帧 | 40 | LOST_CRITICAL / REACQUIRE |
| commit 高度 | < 3.0m | 水平误差 < 0.25m + 连续 8 帧 |
| REACQUIRE 冷却 | 1.0s | 每次退出后冷却 |
| REACQUIRE 最多重试 | 5 次 | 每目标次数上限 |

---

## 七、坐标系统

```
世界: NED (North, East, Down)
  - 相对起飞点 (origin)
  - altitude() 返回相对高度 (正=上)

相机: OpenCV标准 (X右, Y下, Z前)
  - 安装: 朝下 (nadir)
  - 内参: fx=fy=554.26, cx=cy=320 (仿真)

机体: 前(North), 右(East), 下(Down)

像素→世界映射流程:
  像素(u,v) → 归一化(xn,yn) → 相机系射线 → 机体系(旋转) → NED系 → 地平面求交
```

---

## 八、源码目录结构

```
include/
├── comm/           # 飞控通信层
│   ├── droneLink.h
│   ├── offboardControl.h
│   ├── flightOps.h
│   └── servoControl.h
├── control/        # 控制器
│   ├── pidController.h
│   └── dualLoopPid.h
├── vision/         # 视觉管道
│   └── visionInterface.h
├── mission/        # 任务层
│   ├── types.h
│   ├── missionConfig.h
│   └── missionStateMachine.h
└── bomb/           # 投放系统
    ├── coordinateMapper.h
    ├── kalmanFilter.h
    ├── targetTracker.h
    └── bombDropSystem.h

src/             (镜像 include/ 结构)
├── comm/           → droneLink, offboardControl, flightOps, servoControl
├── control/        → pidController, dualLoopPid
├── vision/         → visionInterface
├── mission/        → main, missionConfig, missionStateMachine
└── bomb/           → coordinateMapper, kalmanFilter, targetTracker, bombDropSystem

scripts/             # Python 视觉检测
├── detector_unified.py    # 统一检测器 (推荐, 投放+侦察+H着舰)
├── detector_sim.py        # 独立投放检测 (仿真)
├── detector_real.py       # 独立投放检测 (真机)
├── detector_recon.py      # 独立侦察颜色分析
├── detector_h.py          # 独立 H 标识检测
├── gz_gst_bridge.py        # 仿真相机桥接
├── build.sh               # 编译
└── run_mission.sh         # 真机一键启动
```
