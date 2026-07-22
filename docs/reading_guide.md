# DroneMission 阅读指南

> 最后更新: 2026-07-19

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
多 FIFO 管道管理层:
- `multiBucketPipe`: 非阻塞读取二进制多桶检测 (投放阶段)
- `hDetectionPipe`: 线程后台读取 H 着舰检测 (文本)
- `reconPipe`: 线程后台读取侦察颜色分析 (文本)
- `missionCmdPipe`: 写状态通知到 Python (投放/侦察/RTL)

### 第六步：目标跟踪 `include/bomb/targetTracker.h` → `src/bomb/targetTracker.cpp`
**核心容错模块**。
- VISIBLE → LOST_SHORT → LOST_LONG → LOST_CRITICAL 状态机
- 连续丢失帧计数 (非时间) 防止单帧闪烁触发
- Kalman 滤波平滑 + commit 机制

### 第七步：投放系统 `include/bomb/bombDropSystem.h` → `src/bomb/bombDropSystem.cpp`
**投放阶段完整逻辑**。
- `scanForTargets()` — 像素聚类建图 (50px 半径, 3帧稳定, bucketId 投票)
- `gotoWorldTarget()` — 位置模式飞到目标 1.2m (水平/垂直容忍度 0.5m)
- `trackAndDescend()` — velocity 模式 PID + REACQUIRE 三阶段恢复 + 安全高度约束
- `predictAndDrop()` — 收敛触发放弹 (15s 超时保护)
- `climbToSearchAlt()` — 爬升回 3.5m, 平台无关

### 第八步：顶层状态机 `include/mission/missionStateMachine.h` → `src/mission/missionStateMachine.cpp`
任务编排层。
- `run()` — 11 状态轮询, `handleDropSearch()` 委派给 BombDropSystem
- `handleReconScan()` — 5 航点 H 标识检测 + `/tmp/recon_pipe` 颜色分析
- `handleRtl()` — H-guided 着舰: 返航 25s → 两阶段下降 → 视觉伺服 → FALLTHROUGH → MAVSDK land
- `flyToWithPipeCheck()` — 飞行途中同时检测桶的辅助原语

### 第九步：入口 `src/mission/main.cpp`
信号处理 → 配置加载 → droneLink → 状态机 run()。

---

## 文档索引

| 文档 | 内容 |
|------|------|
| `docs/architecture.md` | 系统架构、分层、数据流、参数表 |
| `docs/state_machine.md` | 全部状态机（顶层 + 投放子系统 + TargetTracker） |
| `docs/mapping_algorithm.md` | 建图算法详解、调试方法、误差分析 |
| `docs/code_review_guide.md` | 逐文件审查要点、超时检查表、日志速查 |

---

## 快速修改参考

| 修改什么 | 改哪里 |
|----------|--------|
| 飞行高度 | `config/flight.yaml` |
| 投放区/侦察区坐标 + 优先级 | `config/mission_zones.yaml` |
| PID 参数 + 着舰参数 | `config/pid.yaml` |
| 舵机 PWM | `config/servo.yaml` |
| 相机内参/外参 | `config/camera.yaml` |
| 状态机流控 | `src/mission/missionStateMachine.cpp` |
| 投放策略 (搜索/跟踪/投弹) | `src/bomb/bombDropSystem.cpp` |
| 目标容错参数 | `src/bomb/targetTracker.cpp` |
| 像素聚类参数 | `src/bomb/bombDropSystem.cpp` → `scanForTargets()` |
| 管道协议 | `src/vision/visionInterface.cpp` |
| 坐标映射 | `src/bomb/coordinateMapper.cpp` |
| 视觉检测逻辑 (多模式切换) | `scripts/detector_unified.py` |
| 桶尺寸分类 | `scripts/detector_unified.py` / `detector_real.py` |
| H 检测 | `scripts/detector_h.py` |
| 侦察颜色分析 | `scripts/detector_recon.py` |
| 连接方式 | `config/connection.yaml` |

---

## 新增特性 (v3)

### BombDropSystem (投放子系统)
- **分层投放架构**: SCAN → SELECT → GOTO → TRACKING → PREDICT → DROP → CLIMB 七阶段状态机
- **像素聚类建图**: 50px 半径, bucketId 投票, 不依赖 YOLO 标签
- **空间最近邻匹配**: TargetTracker 通过像素距离 (100px) 匹配连续帧目标，非标签依赖
- **REACQUIRE 三阶段恢复**: Hover (1s) → Spiral (6s, 扩展半径螺旋) → Climb (4s, 升视野)
- **安全高度约束**: 最低 0.6m, 减速区线性降速, 防止过冲
- **优先级选择**: priority=0 (优先大桶25cm) / priority=1 (优先小桶15cm)

### 多模式视觉检测 (`detector_unified.py`)
- **统一进程**: 根据 `/tmp/mission_cmd` 在投放/侦察/H着舰三模式间自动切换
- **动态模型**: 自动 load/unload YOLO 模型节省 GPU 显存
- **高度感知**: 读取 `/tmp/altitude_pipe` 用于透视补偿桶尺寸分类
- **图像存档**: 后台线程保存检测帧到 `TestImgs/` (按日期分目录)
- **仿真集成**: `--sim` 自动 spawn `gz_gst_bridge.py`

### 管道系统
- **5 管道**: vision_pipe (二进制多桶), h_pipe (H着舰), recon_pipe (侦察颜色), mission_cmd (状态通知), altitude_pipe (高度共享)
- **C++ 端**: 4 个管道类 (multiBucketPipe, hDetectionPipe, reconPipe, missionCmdPipe)

### 着舰系统
- **H 标识检测**: `detector_h.py` 或统一检测器的 RTL 模式, 加载 `best_H.pt`
- **H-guided 着舰**: 两阶段下降 (above 0.4m / below 0.4m), commit 世界坐标, fallback MAVSDK land (30s 超时)
- **新管道**: `/tmp/h_pipe` (文本), `/tmp/recon_pipe` (文本)

### 改进项
- **28m 距离滤波**: 滤除起飞点 NED(0,0) 周边 28m 误检测
- **GOTO 独立超时**: 10s 默认, 防止卡死
- **投弹后稳定**: 5s 稳爬升 + forceDropAll 保底
