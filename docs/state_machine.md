# 状态机详解

> 对应代码: `missionStateMachine.cpp`, `bombDropSystem.cpp`, `targetTracker.cpp`

---

## 一、顶层状态机 (`missionStateMachine::run()`)

```
INIT → ARMING → TAKEOFF → TRANSIT_TO_DROP → DROP_SEARCH → TRANSIT_TO_RECON → RECON_SCAN → RTL → LANDED
                              (15s,3m)        (90s max)       (12s)          (5点×3s)   (25s,4m)
```

### 关键状态处理

| 状态 | 处理函数 | 关键逻辑 |
|------|----------|----------|
| `arming` | `handleArming()` | `flight_->arm()` → TAKEOFF |
| `takeoff` | `handleTakeoff()` | `flight_->takeoff(3m)` → TRANSIT_TO_DROP |
| `transitToDrop` | `handleTransitToDrop()` | 位置模式飞往投放区中心 15s，高度 3m。重置投放区相关状态，包括 `bombSystem_->reset()` → DROP_SEARCH |
| `dropSearch` | `handleDropSearch()` | **见第二节** |
| `transitToRecon` | `handleTransitToRecon()` | 位置模式飞往侦察区 12s |
| `reconScan` | `handleReconScan()` | 5航点，每点悬停 3s 检查 H pipe |
| `rtl` | `handleRtl()` | 返航 4m 25s, 降落 |
| `error` | (run()内) | stop offboard, RTL |

### `handleDropSearch()` 详细逻辑

```cpp
void handleDropSearch() {
    // 1. 维持位置模式悬停在投放区中心 3.5m
    // 2. 28m 距离滤波 (前28m忽略检测)
    // 3. 检查 90s 超时
    // 4. 委派给 BombDropSystem:
    //    result = bombSystem_->execute(remaining, initYaw_)
    // 5. 投弹完成后稳定爬升到 3.5m
    // 6. 不足 2 枚弹则强制释放 → transitToRecon
}
```

---

## 二、投放子系统 (`BombDropSystem::execute()`)

```
SCAN → SELECT → GOTO → TRACKING → PREDICT → DROP → CLIMB
 (8s)                                ↓         ↓
                            (失败: 下一目标)     (完成: 下一目标)
```

### SCAN (8s, 3.5m)
```
悬停扫描 → 像素聚类 → 建世界坐标地图 → 按桶尺寸评分排序
```
**详见**: `docs/mapping_algorithm.md`

### SELECT
```
选最高分未使用的目标
├─ 有未使用 → GOTO
└─ 全部用完 → 重新扫描(5s) → 按桶ID去重(新桶)
              ├─ 有结果 → SELECT
              └─ 无结果 → timeout
```

### GOTO (position 模式, 1.4m)
```
offboard_->startPositionModeAt(target_NED, -1.4m)
while dist > 0.5m: setPositionNed()  ← 位置模式粗定位
```
**为什么用 position 模式**: 飞控内部位置控制器比伴飞脑 PID 更稳定，适合较长距离导航。

### TRACKING (velocity 模式, 1.4m → 1.0m)
```
offboard_->startVelocityMode()
5个控制分支:

┌─ VISIBLE/LOST_SHORT ──────────────────────────────┐
│ 未commit: 视觉 PID (errPx → vx,vy)                │
│ 已commit: 世界坐标 PID (errN,errE → vx,vy)        │
│ vz: 继续下降至 1.0m                                 │
├─ LOST_LONG/REACQUIRE (未commit) ──────────────────┤
│ vx,vy=0 (悬停), vz=0 (暂停下降)                     │
├─ LOST_CRITICAL (已commit) ────────────────────────┤
│ 世界坐标弱修正 (k=0.15), vz: 继续下降                │
└─ LOST_CRITICAL (未commit) ────────────────────────┘
  → 失败, 返回 SELECT
```

### REACQUIRE 子状态机

当未commit且目标丢失触发：

```
HOVER (1s) → 悬停检测
    ↓ 未找到
SPIRAL (6s) → 围绕最后已知世界坐标螺旋搜索
    ↓ 半径 0.2m → 0.4m → 0.6m → 1.0m
    ↓ 未找到
CLIMB (4s) → 上升扩宽视野
    ↓ 未找到
ABORT → 失败, 下一目标
```

**防抖**: 退出 REACQUIRE 后 3s 冷却期, 每轮最多 5 次

### PREDICT (velocity 模式, 1.0m)

五条件投弹判断:
```
cond1: pixelErr < 20px || committed
cond2: 水平速度 < velZeroTol
cond3: |alt - 1.0m| < altTolerance
cond4: 持续稳定 > stableDuration (0.5s)
cond5: 检测率 (当前始终为 true)
```

落点预测:
```
tFall = sqrt(2h / g)
impact_N = vx * tFall
impact_E = vy * tFall
```

### CLIMB (position 模式)
```
爬升回搜索高度 3.5m
dropCount ≥ 2 → DONE
否则 → SELECT (下一目标)
```

---

## 三、TargetTracker 容错状态机

```cpp
enum TargetState { VISIBLE, LOST_SHORT, LOST_LONG, REACQUIRE, LOST_CRITICAL };
```

### 状态转换 (基于连续丢失帧数, 非时间)

| 连续丢失帧 | 状态 | 行为 |
|------------|------|------|
| 0 | `VISIBLE` | Kalman 更新 + PID |
| 1-3 | `VISIBLE` | 仍保持 VISIBLE, 单帧闪烁不触发 |
| 4-12 | `LOST_SHORT` | Kalman 预测继续, 控制不变 |
| 13-39 | `LOST_LONG` | Kalman 预测, 悬停等待 |
| 40+ (未commit) | `LOST_CRITICAL` | 放弃 |
| 40+ (已commit) | `REACQUIRE` | 进入恢复策略 |

**关键参数**: `lostConfirmFrames=4`, `criticalLostFrames=40`

### Commit 机制

```
触发条件 (全部满足):
  1. alt ≤ 3.0m         (commitAltThreshold)
  2. err < 0.25m        (commitErrThreshold)
  3. 连续稳定 8 帧       (commitStableFrames)

commit 后:
  - 水平: 世界坐标 PID 替代视觉 PID
  - 高度: 即使丢失也继续下降
  - 投弹: pixelErr 条件放宽 (cond1 自动通过)
```

### Kalman 滤波

```
状态: [x, y, vx, vy]  (常速度模型)
预测: x += vx*dt, y += vy*dt
更新: 融合观测 (mx, my)
丢失: predictOnly() 纯预测

过程噪声 Q=0.01 m²/s
观测噪声 R=0.05 m²
速度限幅 max=1.0 m/s
```

---

## 四、关键时序

```
t=0      进入 DROP_SEARCH
t=0-8s   SCAN: 悬停扫描 + 建图
t=8s     选目标 → GOTO position模式 1.4m
t=11s    GOTO到达 → TRACKING velocity模式
t=12s    下降到 1.0m
t=13s    五条件满足 → 投弹
t=14s    CLIMB 爬升 3.5m
t=18s    SELEC T下一目标
...
t=90s    全局超时 → 强制投弹 → RECON
```
