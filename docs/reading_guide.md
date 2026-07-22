# DroneMission 阅读指南

> 最后更新: 2026-07-22 | 真机分支 (real)

---

## 代码导航（按阅读顺序）

### 第一步：数据结构 `include/mission/types.h`
所有消息/配置的 C++ struct。**先读这个，后面所有模块都会引用**。
- `bucketDetection`, `multiBucketData` — YOLO 输出 → C++ 数据结构
- `visualServoConfig` — 视觉伺服全部参数
- `missionConfigData` — 根配置, 聚合所有子结构
- `missionState` — 11 个顶层状态枚举

### 第二步：坐标映射 `include/bomb/coordinateMapper.h` → `src/bomb/coordinateMapper.cpp`
**独立数学模块**。像素 → 世界坐标的完整实现。
- `pixelToWorld()` — 地平面求交 + 欧拉角旋转
- `eulerToRotation()` — Rz*Ry*Rx 旋转矩阵
- `cameraToBodyRotation()` — 相机安装姿态矩阵

### 第三步：飞控通信 `include/comm/droneLink.h` → `src/comm/droneLink.cpp`
MAVSDK 唯一所有者。所有飞控操作封装在此。
- `altitude()`, `nedPosition()`, `headingDeg()` — 同步遥测

### 第四步：Offboard 控制 `include/comm/offboardControl.h` → `src/comm/offboardControl.cpp`
两套控制模式：
- `startPositionModeAt()` → `setPositionNed()` — 固定航点, 飞控内部 PID
- `startVelocityMode()` → `setVelocityNed()` — 伴飞脑 PID, 每 50ms

### 第五步：视觉管道 `include/vision/visionInterface.h` → `src/vision/visionInterface.cpp`
`multiBucketPipe`: 非阻塞读取二进制管道, 跳过 count=0 帧。

### 第六步：目标跟踪 `include/bomb/targetTracker.h` → `src/bomb/targetTracker.cpp`
**核心容错模块**。
- VISIBLE → LOST_SHORT → LOST_LONG → LOST_CRITICAL 状态机
- 连续丢失帧计数 (非时间) 防止单帧闪烁触发
- Kalman 滤波平滑 + commit 机制

### 第七步：投放系统 `include/bomb/bombDropSystem.h` → `src/bomb/bombDropSystem.cpp`
**投放阶段完整逻辑**。
- `scanForTargets()` — 像素聚类建图 (50px 半径)
- `gotoWorldTarget()` — 位置模式飞到目标 1.4m
- `trackAndDescend()` — velocity 模式 PID + REACQUIRE 恢复
- `predictAndDrop()` — 五条件投弹 + 落点预测

### 第八步：顶层状态机 `include/mission/missionStateMachine.h` → `src/mission/missionStateMachine.cpp`
任务编排层。
- `run()` — 11 状态轮询, `handleDropSearch()` 委派给 BombDropSystem
- `handleReconScan()` — 5 航点 H 标识检测
- `handleRtl()` — 25s 返航降落

### 第九步：入口 `src/mission/main.cpp`
信号处理 → 配置加载 → droneLink → 状态机 run()。

---

## 文档索引

| 文档 | 内容 |
|------|------|
| `docs/real_hardware.md` | **真机部署指南**: 硬件接线、CSI 相机、GStreamer 推流、飞行前检查清单、故障排查 |
| `docs/architecture.md` | 系统架构、分层、数据流、参数表 |
| `docs/state_machine.md` | 全部状态机（顶层 + 投放子系统 + TargetTracker） |
| `docs/mapping_algorithm.md` | 建图算法详解、调试方法、误差分析 |
| `docs/code_review_guide.md` | 逐文件审查要点、超时检查表、日志速查 |

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
| 像素聚类参数 | `src/bomb/bombDropSystem.cpp` → `scanForTargets()` |
| 管道协议 | `src/vision/visionInterface.cpp` |
| 相机参数 | `src/bomb/coordinateMapper.cpp` + `scripts/detector_*.py` |
| 连接方式 | `config/connection.yaml` |

---

## 新增特性 (v3)

- **BombDropSystem**: 分层投放架构，SCAN→SELECT→GOTO→TRACKING→PREDICT→CLIMB
- **TargetTracker**: Kalman 滤波 + 4 状态容错 + commit 机制
- **CoordinateMapper**: 像素→世界坐标独立模块
- **像素聚类建图**: 50px 半径, bucketId 投票, 不依赖 YOLO 标签
- **REACQUIRE 三阶段恢复**: Hover→Spiral→Climb
- **连续 N 帧丢失确认**: 消除单帧闪烁误触发
- **GOTO 低空靠近**: 位置模式飞到 1.4m, PID 仅做最后下降
- **投弹后稳定爬升**: 5s 升到 3.5m 再过渡到侦察
