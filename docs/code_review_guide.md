# Code Review 指南

> 适合人工审查的代码逻辑导引。按文件逐一标注关键逻辑、潜在问题和审查要点。

---

## 一、文件清单与审查顺序

**推荐审查顺序** (自底向上):

```
第一步: include/mission/types.h            数据结构定义
第二步: include/bomb/coordinateMapper.h  坐标映射接口
第三步: src/bomb/coordinateMapper.cpp    数学实现
第四步: include/bomb/kalmanFilter.h      Kalman 接口
第五步: include/bomb/targetTracker.h    目标跟踪接口
第六步: src/bomb/targetTracker.cpp      状态机实现
第七步: include/bomb/bombDropSystem.h   投放系统接口
第八步: src/bomb/bombDropSystem.cpp     核心投放逻辑
第九步: include/mission/missionStateMachine.h  顶层接口
第十步: src/mission/missionStateMachine.cpp    任务流控
```

---

## 二、逐文件审查要点

### `include/mission/types.h`

| 结构体 | 用途 | 审查点 |
|--------|------|--------|
| `bucketDetection` | YOLO输出 | bucketId 映射 1/2/3 ↔ 15/20/25cm |
| `multiBucketData` | 管道数据结构 | `empty()` 同时检查 count 和 vector |
| `missionConfigData` | 全部配置 | 与 `config/*.yaml` 一致性 |
| `visualServoConfig` | 视觉伺服参数 | convergeTol15cm/20cm/25cm 是否正确 |

### `src/bomb/coordinateMapper.cpp`

**关键函数**: `pixelToWorld()`

```cpp
// 审查要点:
// 1. cameraToBodyRotation() 矩阵 — Z轴是否朝下 (当前: Zc→Zb, 无取反)
// 2. eulerToRotation() — 是否使用正确的旋转顺序 Rz*Ry*Rx
// 3. 地平面求交 — groundZ=0 假设地面在起飞点平面
// 4. t>0 检查 — 防止射线朝上 (镜头朝天时)
// 5. altitude 下限 — alt<0.1 强制为 0.1 防除零
```

**已知问题 (已修复)**: roll/pitch 现已从 `droneLink.attitudeRollDeg/PitchDeg()` 传入所有 `pixelToWorld/worldToPixel` 调用。

### `src/bomb/targetTracker.cpp`

**关键函数**: `update()`

```cpp
// 审查要点:
// 1. lockedBucketId 匹配 — 只追踪已锁定的 bucketId
// 2. Kalman::predict() 必在 update() 前调用
// 3. 丢失帧计数 consecutiveLostFrames — 正确+1和重置
// 4. commit 条件: alt<3m && err<0.25m && stableFrames>=8
// 5. VISIBLE 保持到连续4帧丢失才切换
```

**状态转换边界**:
```
VISIBLE: consecutiveLostFrames == 0
VISIBLE: consecutiveLostFrames < 4  (单帧闪烁不触发)
LOST_SHORT: 4-12 帧丢失
LOST_LONG: 13-39 帧丢失
LOST_CRITICAL/REACQUIRE: 40+ 帧 (依 committed 区分)
```

### `src/bomb/bombDropSystem.cpp`

**关键函数**: `scanForTargets()`

```cpp
// 聚类核心逻辑:
// 1. 每个检测分配最近簇 (距离<50px)
// 2. 无匹配 → 新建簇 (最多10个)
// 3. 簇中心 = 均值
// 4. 稳定帧计数器: 中心变化<50px → +1, 否则=1
// 5. bucketId 投票取众数
// 6. 稳定≥3帧 → 映射世界坐标
// 7. 世界坐标去重: 0.5m内视为同一目标
// 8. 边界检查: 相对于扫描位置 ±20m

// 潜在问题:
// - 50px 阈值在 3.5m 高度约对应 0.3m 世界距离 — 合理
// - 聚类未考虑桶尺寸差异 — 15cm和25cm桶在3.5m高像素差异约 19px
```

**关键函数**: `gotoWorldTarget()`, `centerAboveTarget()`

```cpp
// GOTO: 位置模式飞到目标上方 cfg_.approachAlt (3.0m)
// CENTER: velocity 模式像素伺服, cfg_.approachAlt 保持高度
// DESCEND: 从 cfg_.approachAlt 降至 cfg_.dropAlt (1.8m), 继承 CENTER PID

// 目标匹配: 取画面中心最近的检测 (距中心 <600px)
// 不再用 worldToPixel 投影世界坐标, 消除 SCAN 坐标偏差导致的检测丢弃
// 无像素 >0.5s: 世界坐标兜底 (导航到 SCAN 地图坐标)
// 视觉丢失 >5s (DESCEND) / >5s (CENTER): 放弃本目标

// PID 保护: hasPix 为 false 时不调用 pidX_->update(), 防止垃圾数据污染积分

// 审查要点:
// 1. GOTO 是否足够靠近目标 (dist<0.3m + |alt-3.0|<0.2m)
// 2. CENTER 初始 hover 阶段用高度 P 控制防止切换跌落
// 3. DESCEND 下降速度 0.3 m/s 是否安全
```

**关键函数**: `descendAndDrop()`

```cpp
// 投弹条件 (滑动窗口, 在到达 1.8m 后检查):
// DROP_WIN=10, DROP_MIN=3  → 10帧中≥3帧满足全部条件
// cond1: pixelErr < 40px
// cond2: 水平速度 < velZeroTol (0.15m/s)
// cond3: |alt - 1.8m| < altTolerance (0.1m)

// 落点预测:
tFall = sqrt(2 * alt / 9.81)
impact = vel * tFall  // 仅日志输出, 未用于决策

// 审查要点:
// 1. 稳定时间检查: convergeStart 是否正确重置
// 2. side 选择: 第1弹Left, 第2弹自动另一侧
// 3. releasedDropAlt=true 后不会重置，关注高度漂移
```

### `src/mission/missionStateMachine.cpp`

**关键函数**: `handleDropSearch()`, `handleTransitToDrop()`, `handleTransitToRecon()`

```cpp
// handleTransitToDrop():
// 位置模式飞到投放区中心, 距离<1m + 高度<0.5m 后进入 DROP_SEARCH
// 超时 25s

// handleTransitToRecon():
// 位置模式飞到侦察区中心, 距离<3m + 高度<1m 后进入 RECON_SCAN
// 超时 25s

// 审查要点:
// 1. bombSystem 超时后是否正确处理
// 2. 稳定爬升 5s 是否足够
// 3. 强制投弹顺序是否正确 (Left then Right)
```

**关键函数**: `handleTransitToRecon()`

```cpp
// 位置模式飞到侦察区中心, 到达条件: 水平<3m + 高度<1m, 超时30s
// 不再做 stop/restart, 直接切换目标坐标
// 审查: isActive() 检查防止重复 start
```

---

## 三、常见 Bug 模式与检查点

### 3.1 状态机死锁

**症状**: 无人机悬停不动
**检查**: 每个 while(true) 循环是否有超时出口

| 循环位置 | 超时机制 | 超时值 |
|----------|----------|--------|
| `centerAboveTarget()` | `CENTER_TIMEOUT` | 30s |
| `descendAndDrop()` | `DESCEND_TIMEOUT` | 60s |
| `scanForTargets()` | `timeoutSec` 参数 | 8s |
| `gotoWorldTarget()` | `GOTO_TIMEOUT` | 15s |
| CENTER converged | N/M 滑动窗口 | 20帧中≥4 (1.0s) |
| CENTER lost | `LOST_TIMEOUT` | 5s |
| DESCEND pre-align | N/M 滑动窗口 | 16帧中≥3 (0.8s) |
| DESCEND release | N/M 滑动窗口 | 10帧中≥3 (0.5s) |
| DESCEND lost | `LOST_TIMEOUT` | 5s |
| 全局超时 | `totalTimeout` | 90s |

### 3.2 Offboard 模式切换真空期

**问题**: `stop() → sleep → start()` 中间失去控制, 导致高度跌落
**已修复**:
- `startPositionModeAt` 和 `startVelocityMode` 内部自动处理模式切换, 外部不再调用 stop/sleep
- 切换真空期从 ~400ms 缩减到 ~100ms
- CENTER 初始 hover 阶段用高度 P 控制替代 vz=0
- DESCEND 下降阶段视觉丢失 2s 内保持下降, 避免间歇检测卡住

### 3.3 YOLO 分类与建图不一致

**问题**: YOLO 误分类导致同一物理桶被标为不同ID
**缓解**: 像素聚类 + bucketId 投票。建图准确性由像素位置确定。

### 3.4 PID 漂移

**问题**: 长时间无检测时 PID 积分饱和
**检查**: `pidController` 的 `reset()` 和 `setMaxOutput()`

---

## 四、日志关键信息速查

```
[BOMB] Phase: SCAN          → 投放流程开始
[BOMB] Map: 桶XX @(N,E)     → 建图结果
[BOMB] GOTO: 桶XX @world(N,E) at Xm → 粗逼近
[BOMB] CENTER: centering...  → 像素伺服开始
[BOMB] DESCEND: 桶XX from Xm to Xm → 下降
[BOMB] >>>>> DROP ...       → 投弹 (检查 err/vel/alt)
[BOMB] CLIMB: to Xm         → 爬升
[DROP] BombDropSystem finished: drops=X/2 → 投放结束
STATE: DROP_SEARCH -> TRANSIT_TO_RECON → 侦察开始
```

**异常模式识别**:
- `REACQUIRE #1..#5` 频繁出现 → commit 太晚或视觉不稳定
- `commit=no` in DROP → commit 条件未满足 (正常，视觉满足即可)
- `err=0.00px` in DROP → 可能 YOLO 无检测时误判, 检查 `hasVis` 逻辑
