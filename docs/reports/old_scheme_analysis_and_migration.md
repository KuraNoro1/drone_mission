# 老方案投弹逻辑分析及新方案迁移说明

> 生成时间: 2026-07-16

---

## 一、老方案架构概览

老方案基于 **ROS2 + MAVROS** 通信架构，核心代码分布在两个仓库:

| 仓库 | 路径 | 功能 |
|------|------|------|
| dxy_apm_ws | `src/px4_ros_com/src/offboard_control/` | 飞控任务状态机与控制逻辑 (C++) |
| ros2_yolov8 | `src/ros-yolov8/ros_yolo/` | YOLOv8视觉检测节点 (Python) |

主要通信方式:
- 视觉节点 → 控制节点: ROS2 Topic `detection2d_array` (Detection2DArray 消息)
- 控制节点 → 视觉节点: ROS2 Topic `current_state` (Int32, 状态同步)
- 控制节点 → 飞控: MAVROS 命令 (位置/速度/舵机指令)

---

## 二、投弹区投弹逻辑分析

### 2.1 老方案状态机结构

老方案的投弹区状态在 `StateMachine.cpp:68-300`，使用内部子状态机:

```
Goto_shotpoint → Doshot → Goto_scoutpoint → Surround_see → Doland
                    │
            doshot_init → doshot_shot → doshot_wait → doshot_end
```

关键子状态 (`OffboardControl.h:498-505`):
```cpp
enum class DoshotState {
    doshot_init,   // 初始化投弹, 重置计数器
    doshot_shot,   // 主投弹逻辑
    doshot_wait,   // 两次投弹之间的等待
    doshot_end     // 投弹完成, 清理并转移
};
```

### 2.2 超时处理

老方案实现了**多层超时机制**:

| 超时类型 | 阈值 | 行为 |
|----------|------|------|
| 整体投弹超时 | 70s | `state_timer_.elapsed() > 70` → 跳转到 `doshot_end` |
| 单航点稳定时间 | 6s(粗对准) / 10s(精对准) | 超时后进入PID视觉伺服精对准 |
| 单次投弹等待 | `shot_duration + shot_wait` | 投弹后等待舵机完成, 然后转下一桶 |
| 投弹区总超时 | N/A(依赖70s) | doshot_end后跳转Goto_scoutpoint |

**关键代码** (`StateMachine.cpp:89-94`):
```cpp
if (owner_->state_timer_.elapsed() > 70 && 
    owner_->doshot_state_ != owner_->DoshotState::doshot_end) {
    doshot_halt_end_time = owner_->get_cur_time();
    RCLCPP_INFO(owner_->get_logger(), "超时");
    owner_->doshot_state_ = owner_->DoshotState::doshot_end;
}
```

### 2.3 丢失目标的处理

老方案使用 **连续未检测计数器** (`circle_counter`) 处理目标暂时丢失:

```cpp
// StateMachine.cpp:234-243
if (owner_->_yolo->is_get_target(YOLO::TARGET_TYPE::CIRCLE)) {
    circle_counter = 0;  // 检测到则重置
} else {
    circle_counter++;     // 未检测到则递增
}
// ...
} else if (!shot_flag && (!owner_->_yolo->is_get_target(YOLO::TARGET_TYPE::CIRCLE) 
                          ? circle_counter >= 12 : false)) {
    // 连续12帧(≈0.6s)未检测 → 前往下一个航点搜索
    owner_->waypoint_goto_next(/*... 航点遍历 ...*/);
}
```

**设计要点**:
- 短暂丢失 (< 12帧) → 使用最近一次采集的位置数据继续
- 持续丢失 (≥ 12帧) → 切换为航点模式, 遍历搜索区域
- 每帧50ms, 12帧 = 0.6s容错窗口

### 2.4 航点遍历搜索

投弹区使用 `waypoint_goto_next` 在13个预设航点间遍历:
```cpp
vector<Vector2f> surround_shot_points{
    {0.33333, 0.9}, {0.0, 0.9}, {0.0, 0.5},
    {0.16667, 0.66667}, {-0.33333, 0.1}, {0.16667, 0.33333},
    {-0.16667, 0.66667}, {-0.16667, 0.33333}, {0.0, 0.0},
    {0.0, 1.0}, {-0.33333, 0.9}, {0.33333, 0.1}
};
```
这些航点被归一化到投弹区长宽范围内 (8m×5m)，按顺序逐个访问。

---

## 三、Vision 多桶识别与通信时序

### 3.1 数据流向

```
相机帧 → AIDetector (Python/YOLOv8)
           │
           ├─ 模型1 (best_circle-s.pt): 检测 circle, stuffed
           ├─ 模型2 (best_H-s.pt): 检测 H型降落标识
           │
           ├─ 按类别归类: circle_boxes / stuffed_boxes / h_boxes
           ├─ 发布 Detection2DArray → ROS2 Topic "detection2d_array"
           │
           └─ 控制端(YOLO.cpp)订阅 → 卡尔曼滤波 → StateMachine 使用
```

### 3.2 多帧时序保证

老方案通过以下多个机制保证时序不出错:

**① 帧队列限长 (防止堆积)**:
```python
# detect.py:55 - 队列最大长度=1, 只保留最新帧
self.frames_queue = queue.Queue(maxsize=max_queue_size)  # max_queue_size=1
```

**② 状态感知模型切换 (减少计算量)**:
```python
# detect.py:301-314
if self.current_state == 0:   # Doshot 状态
    results1 = self.model1.predict(...)  # 只跑circle模型
    results2 = []
elif self.current_state == 4:  # Doland 状态
    results2 = self.model2.predict(...)  # 只跑H模型
    results1 = []
else:
    results1 = self.model1.predict(...)
    results2 = self.model2.predict(...)  # 其他状态都跑两模型
```

**③ 卡尔曼滤波器平滑**:
```cpp
// Yolo.h:25-93 - 2D卡尔曼滤波
class KalmanFilter2D {
    // 状态: [x, y, vx, vy], 恒定速度模型
    // 过程噪声0.01, 测量噪声0.5
};
```
控制节点每收到一帧检测结果, 就用卡尔曼滤波平滑位置和速度, 避免单帧抖动。

**④ 最近距离优先排序**:
```cpp
// Yolo.h:342-345 - 按距画面中心曼哈顿距离排序
sort(circle_raw.begin(), circle_raw.end(), [this](...) {
    return abs(a.center.position.x - cap_frame_width/2) + 
           abs(a.center.position.y - cap_frame_height/2) < ...;
});
```

**⑤ 视觉端自含前后帧缓存 (Doland H检测)**:
```python
# detect.py:519-521 - H检测时缓存最近一次检测结果
elif self.last_h_detected_in_doland:
    det_list.append(self.last_h_detected_in_doland)  # 使用缓存
```

### 3.3 通信时序总结

老方案通过 ROS2 消息机制天然保证了消息不会丢失或重复:
- ROS2 DDS 协议保证订阅者不会收到乱序消息
- 控制端 50ms 定时器周期 (20Hz) 远低于视觉端 30Hz 发布频率
- 队列长度为1的帧缓冲确保控制端始终使用最新视觉数据

---

## 四、多桶选择逻辑

### 4.1 老方案选桶流程

1. **K-means 聚类** (`clustering.cpp`): 将所有检测到的目标按位置和直径分为 K=3 类
2. **异常坐标过滤**: 过滤掉超出投弹区范围的异常聚类中心
3. **按直径排序**:
```cpp
// OffboardControl.cpp:155-157
sort(cal_center.begin(), cal_center.end(), [this](const Circles& a, const Circles& b) {
    return this->shot_big_target ? a.diameters > b.diameters   // 大桶优先 (保守模式)
                                 : a.diameters < b.diameters;  // 小桶优先 (激进模式)
});
```
4. **顺序投弹**: `shot_counter` 从1开始, 按排序后的顺序依次投弹

### 4.2 直径映射关系

老方案的直径区间由视觉端的 `rangefinder_height` + 相机内参实时估算:
- 15cm 桶: 直径 0.125~0.175m
- 20cm 桶: 直径 0.175~0.225m  
- 25cm 桶: 直径 0.225~0.325m

---

## 五、投弹时机判定

### 5.1 老方案的精对准逻辑

老方案的 `Doshot()` 函数 (`OffboardControl.cpp:367-680`) 实现了复杂的视觉伺服投弹判定:

```
状态流程:
CatchState::init → CatchState::fly_to_target → CatchState::end
                       │
           ┌───────────┼───────────┐
           │           │           │
      未识别到桶    已投弹+无目标   接近目标
      → 等待       → 原地悬停    → catch_target(PID)
                                       │
                               find_duration >= shot_duration?
                                       │
                                   YES → 投弹!
```

**关键判定参数**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `shot_duration` | 2s | 需要持续对准的时间 |
| `shot_wait` | 0.5s | 投弹后稳定等待时间 |
| `accuracy` | 0.1 | 目标半径精度百分比 |
| `radius` | 0.1m | 物理半径 |

**投弹触发条件** (`OffboardControl.cpp:626-631`):
```cpp
if (!shot_flag && find_duration >= shot_duration) {
    shot_flag = true;
    _servo_controller->set_servo(11 + shot_index, servo_open_position);
}
```

**投弹后重复确认** (`OffboardControl.cpp:633-641`):
```cpp
if (shot_flag) {
    if (find_duration <= shot_duration + get_wait_time()) {
        _servo_controller->set_servo(11 + shot_index, servo_open_position); // 再投一次
    }
    find_duration += get_wait_time();
    if (find_duration >= shot_duration + shot_wait) {
        catch_state_ = CatchState::end;  // 投弹完成
    }
}
```

---

## 六、新方案的迁移修改

### 6.1 投弹区超时逻辑

**修改点**: `missionStateMachine.cpp:runVisualServoLoop`

老方案的 70s 整体超时被纳入新方案:
```cpp
// 老方案: state_timer_.elapsed() > 70 → doshot_end
// 新方案: runVisualServoLoop(dropAlt, 70.0, targetBucket)
int result = runVisualServoLoop(dropAlt, 70.0, targetBucket);
```

超时后行为:
- `result == 0 && dropCount_ > 0`: 有部分投弹, 进入侦察
- `result == 0 && dropCount_ == 0`: 无投弹, 返回航点搜索

### 6.2 丢桶处理

**新增变量** (`missionStateMachine.cpp:runVisualServoLoop`):
```cpp
int lostBriefFrames = 0;         // 老方案 circle_counter 等价
bool targetLostBrief = false;    // 短暂丢失标志 (< 12帧)
const int LOST_BEFORE_SEARCH = 12;  // 匹配老方案 circle_counter >= 12
```

**状态机改进**: 短暂丢失 (< 12帧) 时, `TRACKING` 和 `CONVERGED` 状态不退回到 `SEARCHING`, 仅当完全丢失 (≥ 15帧) 时才退回到 `SEARCHING`:
```cpp
// 短暂丢失(<=12帧)不重置收敛计数
} else if (!freshDetection && targetLostBrief) {
    // 匹配老方案 circle_counter 模式: 继续维持当前状态
}
```

### 6.3 选桶逻辑

**bucketId 到直径映射** (`bucketIdToLabel`):
| bucketId | 物理直径 | 精度半径 (maxAccuratePx) |
|----------|----------|--------------------------|
| 1 | 15cm | 45px (大容差) |
| 2 | 20cm | 35px (中容差) |
| 3 | 25cm | 25px (小容差) |

**优先级完全匹配老方案**:
- `missionPriority == 1` (=老方案 `shot_big_target=false`): 优先小桶
- `missionPriority == 0` (=老方案 `shot_big_target=true`): 优先大桶

### 6.4 投弹时机判定

新方案的投弹条件 (`missionStateMachine.cpp:runVisualServoLoop`):

```
READY_DROP + freshDetection + absErr < CONVERGE_TOL_PX(30px)
  + dropCount_ < 2
  + altOk (高度误差 < 0.2m)
  + velOk (速度 < 0.15m/s)
  + detectOk (检测率 >= 60%)
= → 投弹!
```

**与老方案对应**:
| 老方案参数 | 新方案对应 | 
|-----------|-----------|
| `shot_duration=2s` | `HOLD_FRAMES=30` × 50ms = 1.5s 稳定 |
| `find_duration` 累加 | `holdCounter` 连续计数 |
| `shot_wait=0.5s` | 投弹后重置 `vsState=SEARCHING` 自动等待 |
| `shot_flag` 机制 | `dropCount_` 计数 + `droppedSides_` 挂载点记录 |

### 6.5 投弹仅打印

新方案中投弹**不使用舵机接口**, 仅在终端打印清晰的投弹日志:
```cpp
log("========================================");
log(">>>>> DROP Left (第1/2弹) <<<<<");
log("     bucket=桶3 err=12px vel=0.05m/s alt=1.2m detectRate=80%");
log("========================================");
```

这是有意为之: 先行通过仿真和日志输出验证逻辑正确性, 后续再接入硬件舵机。

---

## 七、视觉通信差异

| 方面 | 老方案 | 新方案 |
|------|--------|--------|
| 通信媒介 | ROS2 DDS | 命名管道 (FIFO) |
| 消息格式 | Detection2DArray (含 bbox + class_id + score) | 二进制协议: [count:1B] [id:1B cx:4B cy:4B]*n |
| 状态同步 | ROS2 Topic `current_state` | 命名管道 `cmd_pipe` |
| 延迟 | DDS发现 + 序列化 ≈ 1-5ms | FIFO读写 ≈ 0.1-0.5ms |
| 可靠性 | QoS可配置 | 简单非阻塞读, 丢帧策略内置 |

新方案不使用 ROS2, 改用 Unix 命名管道实现进程间通信, 更加轻量, 适合 Jetson 边缘设备。
