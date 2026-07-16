# 首飞方案：仿真→真机移植指南

> 对比文件: `/home/limile/test_mavsdk/` (Jetson验证过) vs `/home/limile/drone_mission/` (仿真控制代码)
> 目标平台: Jetson Nano B01, Ubuntu 18.04, ArduPilot

---

## 一、test_mavsdk 与 drone_mission 接口对比

### 1.1 MAVSDK 连接方式

| 项目 | test_servo.cpp | touch.cpp | drone_mission |
|------|---------------|-----------|---------------|
| ComponentType | `CompanionComputer` | `CompanionComputer` | `CompanionComputer` ✅ 一致 |
| 连接URL | `serial:///dev/ttyTHS1:57600` | `tcpout://127.0.0.1:5760` | `connection.yaml` 已预留真实串口 |
| 等待机制 | while 循环 + 10s 超时 | while 循环 + 10s 超时 | `first_autopilot(timeout)` ✅ 更规范 |

**结论**: `drone_mission` 已通过 `connection.yaml` 预留真实硬件连接配置(`config/connection.yaml:3` 注释行)，只需取消注释并注释仿真行。

### 1.2 舵机控制 — **完全相同**

| 项目 | test_servo.cpp | drone_mission (servoControl.cpp) |
|------|---------------|----------------------------------|
| 发送方式 | `MAV_CMD_DO_SET_SERVO` + `queue_message` | `MAV_CMD_DO_SET_SERVO` + `queue_message` |
| 通道编号 | 11(左), 12(右) | 11(左), 12(右) |
| 调用方式 | `set_servo_pwm(ch, pwm)` | `releasePayload(config)` → `setPwm(ch, pwm)` |
| 释放 PWM | 1000(测试值) | 1900(释放) / 1100(保持) / 800ms |
| 系统/组件 ID | sys=255, comp=0 → target_sys=1, target_comp=1 | sys=255, comp=0 → target_sys=1, target_comp=1 ✅ 一致 |

> `drone_mission` 的舵机接口**已从 test_servo.cpp 完整移植**，可直接用于真机。PWM 值(1900/1100)需根据实物舵机实际行程校准。

### 1.3 高度获取 — **关键差异，需关注**

| 项目 | touch.cpp | drone_mission |
|------|-----------|---------------|
| 订阅方式 | `telemetry.subscribe_position` → `relative_altitude_m` | `telemetry_->position().relative_altitude_m` |
| 用户描述 | "读取到的数据实际为激光雷达数据（用于测量高度）且数据准确" | 使用 EKF 估算的相对高度(以起飞点为基准) |
| 距离传感器 | 注释掉了 `distance_sensor` 订阅 | 已订阅 `distance_sensor` 但**未用于控制逻辑** |

**分析**:
- `relative_altitude_m` 在 ArduPilot 中是 EKF 估算的相对高度（以GPS/home点或起飞点为基准）。如果已配置 `EK3_RNG_USE_HGT=1`，则 EKF 会融合激光雷达数据来修正高度估计——此时 `relative_altitude_m` 就是经过激光雷达校正的高度。
- 如果未配置此参数，`relative_altitude_m` 仅依赖气压计+IMU，低空精度有限。
- `droneLink` 中已订阅 `distance_sensor`(存储在 `latestDistanceM_`)，提供 `distanceSensorM()` 方法，但 `missionStateMachine` 的 `computeMountPixels(altitude, ...)` 和视觉伺服循环始终使用 `link_.altitude()`(即 `relative_altitude_m`)。
- **建议**: 在地面测试中先对比两种数据源，若激光雷达更准确，修改 `missionStateMachine.cpp` 在低空(<3m)时使用 `distanceSensorM()` 替代 `altitude()` 参与挂载点像素计算。

### 1.4 相机来源 — 唯一硬件差异

| 仿真 | 真机 |
|------|------|
| Gazebo 虚拟相机 → `gz_gst_bridge.py` → TCP :5000 | CSI 相机 → GStreamer `nvarguscamerasrc` → TCP :5000 |
| `detector_sim.py` 从 TCP 拉流 | `detector_real.py` 从 TCP 拉流 |

**结论**: `detector_real.py` 已写好，与 `detector_sim.py` 的 YOLOv5 推理逻辑和管道发送逻辑**完全一致**。只需在真机上正确启动 CSI→GStreamer→TCP 管道即可。两个 detector 都输出相同格式的二进制约到 `/tmp/vision_pipe`。

### 1.5 视觉管道通信 — **完全一致**

两个 detector 都使用相同的多桶二进制协议写入 `/tmp/vision_pipe`:
```
[count:1B] [id:1B cx:4B cy:4B] * count
```
C++ 端的 `multiBucketPipe` 读取逻辑与仿真无异。

### 1.6 Jetson Nano B01 + Ubuntu 18.04 兼容性

| 依赖 | 状态 | 说明 |
|------|------|------|
| MAVSDK | ✅ 已验证 | `test_mavsdk/` 已在 Jetson 上编译运行 |
| C++ 标准 | ⚠️ 需确认 | CMakeLists.txt 使用 C++17，Ubuntu 18.04 的 gcc 7.5 支持，但可能需要添加 `-latomic` 链接 |
| yaml-cpp | ❓ 需安装 | `sudo apt install libyaml-cpp-dev` |
| YOLOv5 / PyTorch | ❓ 需安装 | Jetson Nano 需 PyTorch for JetPack 4.x (NVIDIA 官方提供) |
| OpenCV (Python) | ❓ 需确认 | `detector_real.py` 使用 `cv2` |

---

## 二、仿真与真机数据处理对比

| 数据流 | 仿真 | 真机 | 相似度 |
|--------|------|------|--------|
| MAVSDK 飞控通信 | `udp://:14550` (本地SITL) | `serial:///dev/ttyTHS1:57600` (UART) | ✅ 协议相同，仅传输层不同 |
| 遥测数据(位置/姿态/速度) | SITL EKF 模拟 | 真机 EKF | ✅ 相同格式 |
| 高度 | 仿真 EKF `relative_altitude_m` | 真机 EKF `relative_altitude_m` (可能已融合激光雷达) | ⚠️ 取决于 `EK3_RNG_USE_HGT` 参数 |
| 视觉管道 | FIFO 二进制 | FIFO 二进制 | ✅ 完全相同 |
| 舵机 | `MAV_CMD_DO_SET_SERVO` | `MAV_CMD_DO_SET_SERVO` | ✅ 完全相同 |
| 相机图像 | Gazebo → GStreamer → TCP | CSI → GStreamer → TCP | ⚠️ 来源不同，但TCP:5000接口相同 |
| Offboard 控制 | `set_position_ned` / `set_velocity_ned` | 相同 | ✅ 完全相同 |

**结论**: 仿真和真机的数据处理方式**几乎相同**，唯一实质差异是**(1)相机数据来源**和**(2)高度数据的精度来源**。这两个差异可通过配置和标定消除。

---

## 三、首飞前测试步骤

### 阶段 1: 地面验证（不接电机电池/不装桨）

#### 1.1 编译验证
在 Jetson 上:
```bash
# 安装依赖
sudo apt install libyaml-cpp-dev cmake g++

# 编译
cd ~/drone_mission
mkdir -p build && cd build
cmake .. && make -j4
```
确认 `droneMission` 二进制生成成功。

#### 1.2 舵机地面测试
1. 连接飞控(仅 USB 或 BEC 供电，**不接电机动力电池**)
2. 修改 `connection.yaml` 为 `serial:///dev/ttyTHS1:57600`
3. 运行舵机单项测试: `./droneMission ../config` (在 arming 状态前手动停止，或单独写一个只测试舵机的短程序)
4. 验证:
   - 通道 11/12 PWM 输出正常(用示波器或直接观察舵机动作)
   - 1900 PWM 释放行程、1100 PWM 保持行程与实物舵机匹配
   - 800ms 释放持续时间足够舵机完成动作

#### 1.3 视觉管道独立测试
```bash
# 终端 1: 启动相机 GStreamer 管道
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
    nvvidconv ! 'video/x-raw,width=640,height=640' ! \
    jpegenc ! tcpserversink host=127.0.0.1 port=5000

# 终端 2: 运行检测器
conda activate <jetson_env>
python3 ~/drone_mission/scripts/detector_real.py

# 终端 3: 验证管道数据格式
cat /tmp/vision_pipe | xxd | head
```
- 手持桶在相机下移动，确认 `detector_real.py` 终端输出正确的桶ID和坐标
- 确认管道中二进制数据格式正确(每帧 count 字节 + N × 9 字节)

#### 1.4 MAVSDK 连接与遥测验证
1. 修改 `connection.yaml` → 真实串口
2. 修改 `mission_zones.yaml` 使投放区/侦察区坐标为 `(0, 0)`(原地测试)
3. 设置 `config/flight.yaml` 中 `takeoffAlt` 为 0(不真飞)，临时在代码中跳过 takeoff
4. 运行 `./droneMission`，验证所有遥测数据可正常读取:
   - `relative_altitude_m` 是否合理（应为 ~0）
   - `distance_sensor` 是否与激光雷达读数一致
   - GPS 是否锁定、位置数据是否抖动

#### 1.5 高度数据源对比测试 (**关键**)
在 Jetson 上写一个简短测试程序，同时打印两种高度:
```cpp
// 同时订阅 relative_altitude_m 和 distance_sensor
double baroAlt = telemetry_->position().relative_altitude_m;
double lidarAlt = latestDistanceM_;  // from distance_sensor
```
- 将无人机放置在不同高度(桌面、地面、举起)，对比两种读数
- 检查 ArduPilot 参数: `EK3_RNG_USE_HGT` 是否为 1
  - 如果为 1: `relative_altitude_m` 已经融合激光雷达，可直接使用
  - 如果为 0: 低空(<3m)视觉伺服时**强烈建议**改用 `distanceSensorM()`

**代码修改位置**(如需切换高度源):
`missionStateMachine.cpp` 中所有 `link_.altitude()` 调用 → 在低空时使用 `link_.distanceSensorM()`
最关键的是 `computeMountPixels()` 的 `altitude` 参数——低空时像素投影对高度极度敏感:
- 高度 1.2m 时，±0.1m 误差 ≈ ±7 像素偏移
- 高度 0.5m 时，±0.1m 误差 ≈ ±22 像素偏移

#### 1.6 相机内参标定
- 仿真相机: FOV=60°, 640×640 → fx=fy=554.26, cx=cy=320
- 真机相机: **必须重新标定**，CSI 相机内参可能与仿真不同
- 使用 OpenCV 棋盘格标定获取真实 `fx, fy, cx, cy`
- 更新 `missionStateMachine.cpp::computeMountPixels()` 中的参数

#### 1.7 相机安装偏移实测
- 测量真机上相机光心与无人机质心的相对位置 → 更新 `camDx, camDy`
- 测量左/右挂载点与无人机质心的相对位置 → 更新 `mntLx, mntLy, mntRx, mntRy`
- 这些参数直接影响 `computeMountPixels()` 的挂载点投影精度

### 阶段 2: 系留飞行测试

#### 2.1 系留低空悬停
- 用绳子系住无人机，限制最大飞行高度 1.5m
- 只测试 offboard 模式切换:
  - Arm → Takeoff → 启动 offboard position mode
  - 发送小幅度位置指令 (±1m)，确认运动方向正确(NED 坐标系)
  - 通过遥控器切回 Stabilize 确认应急接管正常

#### 2.2 视觉伺服方向验证
- 系留下启动视觉伺服，手持桶在相机视野内移动
- **关键验证**: 桶在画面右边→无人机向右飞(NED East 为正?)
  - 如果方向反了，在 PID 输出处取反号
- 验证 PID 输出速度在合理范围内(不超过 `maxVelXY`)
- 验证丢目标后的搜索行为正确

#### 2.3 投放动作测试
- 系留下执行投放流程，**不放真实载荷**(空投或放轻海绵)
- 验证:
  - 视觉伺服 → CONVERGED → READY_DROP 状态转换正常
  - 舵机在 READY_DROP 后正确触发
  - 释放持续时间 800ms 足够

### 阶段 3: 自由飞行(低空渐进)

#### 3.1 首次自由飞行配置
- 限制飞行高度 ≤ 3m
- 不使用完整自动任务流程
- 分阶段手动触发每个状态(而非自动状态机推进)
- 配置 FENCE 地理围栏限制飞行范围
- 准备随时切 Stabilize/AltHold 的应急方案

#### 3.2 PID 参数调优
- 仿真中的 PID 增益(kp=0.8, ki=0.02, kd=0.2)在真机上可能需调整
- 先保守使用较低增益(kp=0.4, ki=0)，逐步增大
- 观察 `StaticAnalysis/pid_visual.csv` 中的误差收敛曲线
- 调优顺序: XY 平面 PI → Z 轴 PID → 视觉伺服 PI

---

## 四、首飞注意事项

### 4.1 坐标系与参数
- **NED 原点**: 真机的 NED 原点以解锁点(Home)为基准，仿真以 Gazebo 世界原点为基准。确保 `mission_zones.yaml` 的坐标是**相对于起飞点**的
- **初始航向**: `initYaw_` 锁定起飞时的航向。真机起飞方向必须与预设航点方向一致，否则 NED 坐标会发生旋转
- **GPS 需求**: NED 位置控制依赖 GPS。确保起飞前 GPS HDOP < 1.0

### 4.2 安全配置
- 遥控器**模式切换开关**(Stabilize/AltHold)必须有最高优先级
- 设置 ArduPilot FENCE 参数:
  ```
  FENCE_ENABLE = 1
  FENCE_TYPE = 3  (高度+圆)
  FENCE_ALT_MAX = 10  (米)
  FENCE_RADIUS = 50 (米)
  ```
- 设置 `FS_GCS_ENABLE=1`: MAVSDK 连接断开时自动 RTL
- 设置 `FS_THR_ENABLE=0`: 关闭油门失控保护(由 MAVSDK 控制油门)
- 准备物理急停方案(遥控器上的 Kill Switch 或模式切换)

### 4.3 桶分类阈值标定
`detector_real.py` 中的分类阈值是基于 416×416 输入在特定高度的估计:
```python
THRESH_15_20 = 30   # 15cm桶 vs 20cm桶的像素宽度阈值
THRESH_20_25 = 55   # 20cm桶 vs 25cm桶的像素宽度阈值
```
- 在实际飞行高度(1.2-1.6m)用真实桶标定这些阈值
- 或者改用基于已知高度的物理直径反算: `pixel_width = real_width * fx / altitude`

### 4.4 投放区搜索策略
- `dropSearch` 使用 3 个航点(左 -3m, 中心, 右 +3m) 搜索
- 真实环境中投放区可能不是规则排列，需根据实际场地调整 `mission_zones.yaml`
- 搜索高度 3m 可能有风的影响，考虑增大悬停容忍度

### 4.5 日志与调试
- 训练日志会自动保存到 `~/yolo_exercise/log/`
- 飞行数据 CSV 保存到 `StaticAnalysis/pid_visual.csv`
- 首飞后立即用 `scripts/analyze.sh` 分析数据
- 保留完整的飞控日志(.bin 文件)用于事后分析

### 4.6 回退方案
- 如果视觉伺服不稳定: 改用纯 GPS 定点投放(精度降低但可靠性高)
- 如果管道通信不稳定: 增加管道缓冲区大小，或改用 UDP 通信
- 如果 MAVSDK 连接不稳定: 检查串口线材质量，降低波特率到 57600(已设)

---

## 五、关键代码修改清单(如需)

| 文件 | 修改内容 | 优先级 |
|------|----------|--------|
| `config/connection.yaml` | 切换为 `serial:///dev/ttyTHS1:57600` | 🔴 必须 |
| `missionStateMachine.cpp::computeMountPixels()` | 更新相机内参 `fx,fy,cx,cy` 和偏移量 `camDx,camDy,mntLx,mntLy,mntRx,mntRy` | 🔴 必须 |
| `missionStateMachine.cpp` 视觉伺服循环 | 低空(<3m)用 `link_.distanceSensorM()` 替代 `link_.altitude()` | 🟡 建议 |
| `detector_real.py` | 校准 `THRESH_15_20`, `THRESH_20_25` | 🟡 建议 |
| `detector_real.py` | 确认摄像头分辨率/帧率 | 🟢 可选 |
| `config/servo.yaml` | 校准 `releasePwm`/`holdPwm`/`releaseDurationMs` | 🟡 建议 |
| `CMakeLists.txt` | 如编译错误，添加 `-latomic` 链接 | 🟡 按需 |

---

## 六、参考资源

| 资源 | 路径 |
|------|------|
| 测试通过的 MAVSDK 代码 | `/home/limile/test_mavsdk/` |
| 仿真开发过程记录 | `/home/limile/OpenCodeSessions/DXY_Competition_Combination/session_summary.md` |
| 项目架构文档 | `/home/limile/drone_mission/docs/architecture.md` |
| 训练脚本 | `/home/limile/yolo_exercise/drop_zone/train.sh` |
| YOLO 模型 | `/home/limile/yolo_test/best.pt` |
| 训练数据集 | `/home/limile/yolo_exercise/drop_zone/` |
