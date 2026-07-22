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
第六步: src/bomb/targetTracker.cpp      状态机实现 (空间匹配 + commit)
第七步: include/bomb/bombDropSystem.h   投放系统接口
第八步: src/bomb/bombDropSystem.cpp     核心投放逻辑 (7 阶段状态机)
第九步: include/mission/missionStateMachine.h  顶层接口
第十步: src/mission/missionStateMachine.cpp    任务流控 (H 着舰)
第十一步: include/vision/visionInterface.h     管道接口
第十二步: src/vision/visionInterface.cpp       管道实现 (5 管道)
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

**当前已知问题**: 未使用真实的 roll/pitch 值 (传 0,0)，导致 ~0.5-1m 映射误差。

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
// 聚类核心逻辑 (当前实现):
// 1. 每个检测分配最近簇 (距离<50px)
// 2. 无匹配 → 新建簇 (最多10个)
// 3. 簇中心 = 本帧分配检测的均值
// 4. 稳定帧计数器: 中心变化<50px → +1, 否则=1
// 5. bucketId 投票取众数
// 6. 稳定≥3帧 → 映射世界坐标
// 7. 世界坐标去重: 0.5m 内视为同一目标
// 8. 边界检查: 相对于扫描原点 ±20m
// 9. 扫描结束按评分 (尺寸 × 优先级权重) 排序

// 潜在问题:
// - 50px 阈值在 3.5m 高度约对应 0.3m 世界距离 — 合理
// - 聚类未考虑桶尺寸差异 — 15cm 和 25cm 桶在 3.5m 高像素差异约 19px
```

**关键函数**: `trackAndDescend()`

```cpp
// 5 控制分支:
if (VISIBLE || LOST_SHORT) {
    if (committed) → World PID
    else → Pixel PID
}
else if (LOST_LONG || REACQUIRE early) {
    if (!committed) → vx=vy=0 (悬停), vz=0 (暂停下降)
}
else if (committed) {
    → 弱世界坐标修正 (k=0.15), 继续下降
}
else {
    → 进入 REACQUIRE
}

// 安全高度:
if (alt < minSafeAlt (0.6m)) → vz = MAX_DESCENT_RATE (0.3 m/s) 强制上升
// 减速区:
if (alt < dropAlt + decelZone (0.6m)) → 线性降速至 dropAlt 处速度=0

// REACQUIRE 子状态:
HOVER (1s) → SPIRAL (6s, 0.2→1.0m 扩展半径) → CLIMB (4s, 升 upTo1.5m)
退出后 1s 冷却, 最多 5 次重试 per target
```

**关键函数**: `predictAndDrop()`

```cpp
// 简化的投弹判断 (vs 早期五条件):
// committed → 世界坐标弱修正, 等待 pixelErr < convergeTolPx → DROP
// 未commit → 视觉 PID, 等待 pixelErr < convergeTolPx → DROP
// 超时: 15s

// 落点预测 (仅日志):
tFall = sqrt(2 * alt / 9.81)
impact = vel * tFall
```

### `scripts/detector_unified.py`

```cpp
// 状态机:
DROP_SEARCH/DROP_VISUAL_SERVO/TRANSIT_TO_DROP → DROP 模式 (YOLO 桶检测)
RECON_SCAN → RECON 模式 (YOLO + HSV 颜色分析)
RTL → H_LAND 模式 (YOLO H 检测)
OTHER → IDLE (仅过帧)

// GPU 内存:
maybe_unload_bucket_model() / maybe_unload_h_model() 动态切换

// 图像存档:
后台线程, 每 5 帧存 1 帧 → TestImgs/data_YYYYMMDD_N/
文件名格式: {MODE}_{HHMMSS}_{microseconds}.jpg
```

### `src/mission/missionStateMachine.cpp`

**关键函数**: `handleDropSearch()`

```cpp
// 流控:
1. 位置模式悬停 searchAlt (3.5m)
2. 28m 距离滤波 (起飞点周围 28m 内忽略检测)
3. 90s 超时检查
4. bombSystem_->execute(remaining, initYaw_)   // 委派给投放子系统
5. 结果处理: 5s 稳定爬升 + forceDropAll 补齐 → transitToRecon

// 审查要点:
// 1. bombSystem 超时后是否正确返回 timedOut
// 2. 稳定爬升 5s 是否足够恢复姿态
// 3. forceDropAll 顺序: Left → Right (串行)
```

**关键函数**: `handleRtl()`

```cpp
// H-guided 着舰:
1. 返航巡航 25s 保持 4m
2. 切换到 position 模式
3. 预读 H 检测 (可能有巡航阶段积累的结果)
4. 两阶段下降:
   - alt > 4m: vz = 0.3 m/s (快速)
   - alt < 4m: above 0.4m → 0.3 m/s, below 0.4m → 0.15 m/s (慢速)
5. H 丢失处理:
   - committed (世界坐标可用) → 继续用上次世界坐标下降
   - 未commit → 暂停, 等待 (30s 超时 → fallback MAVSDK land)
6. alt < 0.2m → FALLTHROUGH → MAVSDK land()

// 审查要点:
// 1. committed 条件 (连续稳定帧 + 高度阈值)
// 2. 两阶段速度切换边界是否正确
// 3. FALLTHROUGH 时机和 fallback 超时
```

**关键函数**: `handleTransitToRecon()`

```cpp
// 不再做 stop/restart, 直接切换目标坐标
// 前提: handleDropSearch 已稳定在 searchAlt (3.5m)
// 审查: isActive() 检查防止重复 start
```

---

## 三、常见 Bug 模式与检查点

### 3.1 状态机死锁

**症状**: 无人机悬停不动
**检查**: 每个 while(true) 循环是否有超时出口

| 循环位置 | 超时机制 | 超时值 |
|----------|----------|--------|
| `trackAndDescend()` | `TRACKING_TIMEOUT` | 60s |
| `predictAndDrop()` | PREDICT 阶段循环 | 15s |
| `scanForTargets()` | `timeoutSec` 参数 | 8s / 5s |
| `gotoWorldTarget()` | `gotoTimeout` | 10s |
| `climbToSearchAlt()` | CLIMB 阶段循环 | 10s |
| REACQUIRE HOVER | `raElapsed > 1.0` | 1s |
| REACQUIRE SPIRAL | `raElapsed > 6.0` | 6s |
| REACQUIRE CLIMB | `raElapsed > 4.0` | 4s |
| H 着舰等待 | `hLandTimeout` | 30s |
| DROP_SEARCH 全局 | 外层 90s | 90s |

### 3.2 Offboard 模式切换真空期

**问题**: `stop() → sleep → start()` 中间失去控制
**缓解**: 
- 减少 stop/start 次数 (如 handleTransitToRecon 不再 stop)
- 在低高度 (<2m) 时不切换模式
- 投弹后先爬升再切换

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
[BOMB] GOTO: (N,E) at Hm    → 导航目标
[BOMB] TRACKING: ...        → 跟踪开始
[BOMB] REACQUIRE #N: ...    → 进入恢复 (检查次数和触发原因)
[BOMB] TARGET COMMITTED     → commit 成功 (检查高度和误差)
[BOMB] >>>>> DROP ...       → 投弹 (检查 err/vel/alt/commit)
[BOMB] CLIMB: to Hm         → 爬升
[DROP] BombDropSystem finished: drops=X/2 → 投放结束
[DROP] Stabilizing...       → 稳定爬升
STATE: DROP_SEARCH -> TRANSIT_TO_RECON → 侦察开始
[RTL] H detected: (cx,cy)   → H 着舰检测
STATE: RTL -> LANDED        → 着舰完成
```

**异常模式识别**:
- `REACQUIRE #1..#5` 频繁出现 → commit 太晚或视觉不稳定
- `commit=no` in DROP → commit 条件未满足 (正常，视觉满足即可)
- `err=0.00px` in DROP → 可能检测丢失时误判, 检查 `hasVis` 逻辑
- H 着舰超时 fallback → H 模型未加载或 H 标识不在视野
