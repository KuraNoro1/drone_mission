# DroneMission 架构文档：通信与关键数据处理

## 一、系统总览

```
┌──────────────────────────────────────────────────────────────┐
│                      PC / Jetson                              │
│                                                               │
│  ┌──────────────────┐        ┌─────────────────────────────┐ │
│  │ detector_sim.py   │ 12B    │ droneMission (C++)           │ │
│  │ (YOLOv5 检测)     │──FIFO──►│                              │ │
│  │                   │       │ missionStateMachine (FSM)     │ │
│  │  ┌─────────────┐  │       │  ├─ droneLink    (MAVSDK)    │ │
│  │  │gz_gst_bridge│  │       │  ├─ flightOps    (arm/land)  │ │
│  │  │ (子进程)     │  │       │  ├─ offboardControl          │ │
│  │  └──────┬──────┘  │       │  ├─ servoControl  (舵机)     │ │
│  │         │gz-topic │       │  ├─ binaryVisionPipe (FIFO)  │ │
│  └─────────┼─────────┘       │  └─ pidController (x3)       │ │
│            │                 └──────────────┬──────────────┘ │
│            │                                │ MAVLink/MAVSDK │
│  ┌─────────┴─────────┐           ┌──────────┴──────────┐    │
│  │ Gazebo Sim        │           │ ArduPilot SITL       │    │
│  │  downward_camera  │           │  fdm_addr: 127.0.0.1 │    │
│  │  640x640 30Hz     │           │  port_in: 9002       │    │
│  └───────────────────┘           └──────────────────────┘    │
└──────────────────────────────────────────────────────────────┘
```

## 二、三条通信通道

### 2.1 飞控通道：MAVSDK over UDP/Serial

| 模式 | URL | 说明 |
|------|-----|------|
| 仿真 | `udp://:14550` | SITL 默认端口 |
| 真机 | `serial:///dev/ttyTHS1:57600` | Jetson UART → FC TELEM2 |

三类 MAVSDK 插件通过 `droneLink` 统一管理：

| 插件 | 用途 | 关键方法 |
|------|------|----------|
| `Action` | 基础动作 | `arm()`, `takeoff()`, `land()`, `return_to_launch()` |
| `Offboard` | 外部控制 | `set_position_ned()`, `set_velocity_ned()`, `start()`, `stop()` |
| `Telemetry` | 遥测 | `position()`, `position_velocity_ned()`, `heading()`, `altitude()` |

舵机通过 `MavlinkPassthrough` 发送原始 `MAV_CMD_DO_SET_SERVO` (cmd=183), 不使用 `set_actuator()`。

### 2.2 视觉通道：FIFO 二进制管道

```
detector_sim.py ──struct.pack('fff')──► /tmp/vision_pipe ──binaryVisionPipe──► missionStateMachine
```

| 字段 | 类型 | 字节 |
|------|------|------|
| cx | float32 | 0-3 |
| cy | float32 | 4-7 |
| confidence | float32 | 8-11 |

`binaryVisionPipe` 实现:
- `open()`: 阻塞 `O_RDONLY`, 等 Python 连接
- `readLatest()`: `fcntl(O_NONBLOCK)` 排空管道取最新, 恢复阻塞

### 2.3 相机通道：Gazebo → GStreamer → TCP

```
Gazebo camera sensor (gz-topic)
    │  gz-transport subscribe
    ▼
gz_gst_bridge.py (appsrc → jpegenc → tcpserversink :5000)
    │  TCP JPEG stream
    ▼
detector_sim.py (cv2.VideoCapture)
    │  YOLOv5 inference
    ▼
/tmp/vision_pipe (12B binary)
```

## 三、关键数据处理

### 3.1 挂载点像素投影

```
输入: 当前高度 altitude(m)
输出: 左挂载点像素 (uL, vL), 右挂载点像素 (uR, vR), 桶半径像素(r)
```

公式 (pinhole 模型, 相机向下):
```
u = cx + fx * (mount_x - cam_x) / altitude
v = cy + fy * (mount_y - cam_y) / altitude
r = world_radius * fx / altitude
```

### 3.2 视觉伺服 PID 控制链

```
管道读取 (cx, cy, conf)
    │
    ▼
computeMountPixels(alt) → (uL, vL, uR, vR)
    │
    ▼
选择更近挂载点: min(distance((cx,cy), (uL,vL)), distance((cx,cy), (uR,vR)))
    │
    ▼
像素误差: errU = mount_u - bucket_u, errV = mount_v - bucket_v
    │
    ▼
归一化: errNorm = err / 320 (图像半宽)
    │
    ▼
PID(归一化误差, dt) → velocity (vx, vy)
    │  kp=0.8, ki=0.02, kd=0.0 (纯 PI)
    ▼
offboard::setVelocityNed(vx, vy, 0, yaw)
    │  Z 轴 vd=0, 高度由飞控维持
    ▼
飞控执行速度指令 → 无人机移动 → 更新像素位置 → 循环
```

### 3.3 检测结果缓存

当管道暂时无新数据时 (`readLatest` 返回 false), 用 `lastDetection_` 中缓存的上次坐标继续驱动 PID, 避免丢帧导致速度归零。

## 四、状态机

```
arming → takeoff → transitToDrop → dropSearch → dropVisualServo
                                    ↓ (90s超时)    ↓ (40s超时)
                               transitToRecon ←────────┘
                                    ↓
                                reconScan → rtl → landed
```

**dropSearch** 内部子阶段:
- 3 个搜索航点 (左 3m, 中心, 右 3m), 高度 3m
- `flyToWithPipeCheck()` 在飞行途中持续检查管道, 有桶即中断飞往对准
- 到达后 `waitForDetection(10s)` 悬停等待
- 全部无桶或投放区总时间超 90s → 侦察区

## 五、配置加载

`missionConfig` 从 `config/` 下 7 个 YAML 加载并解析为 `missionConfigData` 结构体, 通过只读引用传入各模块。同时自动从 `configDir` 推导项目根路径, 创建 `StaticAnalysis/` 目录。
