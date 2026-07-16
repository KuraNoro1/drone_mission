# DroneMission 阅读引导：自顶向下学习路径

## 阅读顺序

```
1. README.md                     ← 功能概览 + 快速开始
2. include/types.h               ← 数据结构 (词汇表)
3. src/missionStateMachine.cpp   ← 核心状态机 (业务逻辑)
4. include/droneLink.h/.cpp      ← MAVSDK 连接 + 遥测
5. include/offboardControl.h/.cpp← 位置/速度 offboard 控制
6. include/visionInterface.h/.cpp← FIFO 二进制管道
7. include/pidController.h/.cpp  ← PID 算法
8. src/main.cpp                  ← 程序入口
9. config/*.yaml                 ← 配置参数
```

---

## 第 1 层：README.md

建立对系统的整体认知：目录结构、启动流程、Gazebo 模型修改方法、配置说明、状态机流程图。

---

## 第 2 层：类型定义 (`include/types.h`)

关键结构体:

| 结构体 | 作用 |
|--------|------|
| `nedCoord` | 北-东-下位置 |
| `visionBinaryData` | 管道接收的桶检测数据 (cx, cy, conf) |
| `missionConfigData` | 所有 YAML 配置汇总 |
| `missionState` | 状态机枚举, 11 个状态 |

---

## 第 3 层：核心状态机 (`missionStateMachine.cpp`)

### init() — 初始化
创建所有子模块 → 锁机头朝向 → 打开管道 → 状态设为 `arming`

### run() — 主循环
```cpp
while (running_) {
    switch (state_) {
        case arming:        handleArming();        break;
        case takeoff:       handleTakeoff();       break;
        case transitToDrop: handleTransitToDrop(); break;
        case dropSearch:    handleDropSearch();    break;
        case dropVisualServo: handleDropVisualServo(); break;
        // ...
    }
}
```

### 重点: handleDropSearch()

1. 检查投放区总超时 (90s)
2. 飞到搜索航点 (左/中/右) — `flyToWithPipeCheck()` 途中查管道
3. 悬停 10s — `waitForDetection()` 查管道
4. 任一步检测到桶 → `gotoBucketFound()` 下降 1.2m → `dropVisualServo`
5. 3 点全无 → `transitToRecon`

### 重点: runVisualServoLoop()

1. 启动 velocity offboard
2. 循环 (最大 40s):
   - 读管道 → 挂载点像素 → 选择更近挂载点 → 像素误差 → PID → 速度指令
   - 无检测时用 `lastDetection_` 缓存
3. 超时 → 上升至巡航高度 → 侦察区

---

## 第 4~7 层：各模块

### droneLink
唯一拥有 MAVSDK 插件的类。连接、遥测读取、暴露底层 Action/Offboard/Telemetry/Passthrough 引用。

### offboardControl
位置模式 (`flyToPosition`) 和速度模式 (`startVelocityMode` + `setVelocityNed`)。`startPositionMode` 启动时自动发两帧当前位姿建立 offboard。

### binaryVisionPipe (visionInterface)
阻塞 `open(O_RDONLY)`, `readLatest()` 用 `fcntl(O_NONBLOCK)` 排空管道取最新 12B 数据后恢复阻塞标志。

### pidController
首次调用只做 P 控制, 积分项 anti-windup 钳位, 输出限幅。

---

## 第 8 层：main.cpp

- `signalHandler` 直接调 `mission->stop()`
- Config → droneLink → missionStateMachine → run()
- 无业务逻辑, 纯接线

---

## 第 9 层：config/*.yaml

7 个文件按功能拆分, `missionConfig::load()` 依次加载到 `missionConfigData`。

---

## 快速定位

| 想改什么 | 文件 |
|----------|------|
| 飞行高度 | `config/flight.yaml` |
| PID 参数 | `config/pid.yaml` |
| 投放区/侦察区 | `config/mission_zones.yaml` |
| 状态机流程 | `src/missionStateMachine.cpp` — `handleXxx()` |
| 管道通信 | `src/visionInterface.cpp` — `binaryVisionPipe` |
| 挂载点像素 | `missionStateMachine.cpp::computeMountPixels()` |
| 视觉检测 | `scripts/detector_sim.py` |
| 相机桥接 | `scripts/gz_gst_bridge.py` |
