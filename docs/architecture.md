# DroneMission 系统架构

> 2026-07-19 | 当前实现版本 v3

---

## 一、总体架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                          仿真环境 (SITL)                             │
│                                                                     │
│  Gazebo 世界 ←→ gst_bridge → TCP :5000 → detector_sim.py (YOLO)    │
│       │                                                    │        │
│  iris SITL (ArduPilot)                            /tmp/vision_pipe  │
│       │                                                    │        │
│       ├── UDP :14550 → MAVSDK → droneMission (C++) ←───────┘        │
│       │                       → /tmp/altitude_pipe                   │
│       │                       → /tmp/mission_cmd → detector_sim.py   │
│       └── TCP :5760 → QGroundControl                                │
└─────────────────────────────────────────────────────────────────────┘
```

**真机**：CSI相机 → nvarguscamerasrc → TCP :5000 → detector_real.py，FC TELEM2 → UART → MAVSDK。

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
| `visionInterface` | `visionInterface.h/cpp` | FIFO 命名管道：多桶检测、H标识、状态同步 |

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
YOLO检测 → /tmp/vision_pipe → multiBucketPipe::readLatest()
                                      │
    ┌─────────────────────────────────┘
    │  SCAN阶段: 像素聚类 → pixelToWorld() → 目标地图
    │  TRACKING阶段: TargetTracker.update() → Kalman预测/更新
    │
    ├── Pixel PID (未commit): errPx → pidX/pidY → vx,vy
    └── World PID (已commit): errN,errE → pidX/pidY → vx,vy
                                       │
                              offboardControl::setVelocityNed()
                                       │
                                  MAVSDK → ArduPilot
```

---

## 五、管道协议

### `/tmp/vision_pipe` (二进制)

```
[count: uint8] [id: uint8] [cx: float32] [cy: float32]  (每桶重复)
总长度 = 1 + count × 9 字节
count=0 时只有 1 字节
```

### `/tmp/altitude_pipe` (文本)
```
高度值 (m, 换行分隔), 2Hz, C++ → Python
```

### `/tmp/mission_cmd` (文本)
```
状态名 (换行分隔), C++ → Python
```

---

## 六、关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 搜索高度 | 3.5m | SCAN 阶段悬停高度 |
| 靠近高度 | 1.4m | GOTO 位置模式飞到目标上方高度 |
| 投弹高度 | 1.0m | PID 最终下降目标高度 |
| 投弹区超时 | 90s | 进入投放区总时限 |
| Kalman 过程噪声 | 0.01 m²/s | Q |
| Kalman 观测噪声 | 0.05 m² | R |
| 像素聚类半径 | 50px | SCAN 阶段聚类阈值 |
| 连续丢失确认帧 | 4 | 丢失才切换状态 |
| 严重丢失帧 | 40 | LOST_CRITICAL |
| commit 高度 | < 3.0m | 水平误差 < 0.25m + 连续 8 帧 |
| 投弹像素容差 | 20px | PREDICT 阶段 |
| 稳定持续时间 | 0.5s | 投弹前稳定检查 |
| commit 阈值 | alt<3.0m, err<0.25m | 连续8帧稳定 |
| REACQUIRE 冷却 | 3.0s | 每次退出后冷却 |

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
```
