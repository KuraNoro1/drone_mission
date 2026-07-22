# 状态机详解

> 对应代码: `missionStateMachine.cpp`, `bombDropSystem.cpp`, `targetTracker.cpp`

---

## 一、顶层状态机 (`missionStateMachine::run()`)

```
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP → DROP_SEARCH → TRANSIT_TO_RECON → RECON_SCAN → RTL → LANDED
                          (15s,3m)       (90s max)       (12s)          (5点×3s)   (120s, H-guided)
```

### 关键状态处理

| 状态 | 处理函数 | 关键逻辑 |
|------|----------|----------|
| `arming` | `handleArming()` | `flight_->arm()` → TAKEOFF |
| `takeoff` | `handleTakeoff()` | `flight_->takeoff(3m)` → TRANSIT_TO_DROP |
| `transitToDrop` | `handleTransitToDrop()` | 位置模式飞往投放区中心 15s，高度 3m。重置投放区相关状态，包括 `bombSystem_->reset()` → DROP_SEARCH |
| `dropSearch` | `handleDropSearch()` | **委派给 BombDropSystem** (见第二节)。投放完成后稳定爬升至 3.5m, 不足 2 弹则强制释放 → TRANSIT_TO_RECON |
| `transitToRecon` | `handleTransitToRecon()` | 位置模式飞往侦察区 12s, 不重新 stop/start offboard |
| `reconScan` | `handleReconScan()` | 5航点 (菱形), 每点悬停 3s 检查 `/tmp/recon_pipe`, 日志记录颜色分析 |
| `rtl` | `handleRtl()` | **H-guided 着舰**: 返航 25s→两阶段下降→H 标识视觉伺服→FALLTHROUGH→MAVSDK land (30s超时回退) |
| `error` | (run()内) | stop offboard, RTL |

### `handleDropSearch()` 详细逻辑

```cpp
void handleDropSearch() {
    // 1. 维持位置模式悬停在投放区中心 3.5m
    // 2. 28m 距离滤波 (前28m 忽略检测，滤除起飞点误检)
    // 3. 检查 90s 超时
    // 4. 委派给 BombDropSystem:
    //    result = bombSystem_->execute(remaining, initYaw_)
    //    内部: SCAN → SELECT → GOTO(1.2m) → TRACKING → PREDICT → DROP → CLIMB
    // 5. 投弹完成后 5s 稳定爬升到 3.5m
    // 6. 不足 2 枚弹则 forceDropAll → transitToRecon
}
```

### RTL H 着舰子状态机

```
→ 先读 hDetectionPipe (可能已有检测)
   ↓ 有检测
   切换到 position 模式 → 读取 H 目标世界坐标
   ↓
   下降阶段:
     alt > 4m → 继续下降 (0.3 m/s)
     alt ≤ 4m → 两阶段: above 0.4m 快速下降 / below 0.4m 慢速 (0.15 m/s)
   ↓
   H 检测丢失:
     ├─ 已 commit → 用上次世界坐标继续
     └─ 未 commit → 暂停下降, 等待恢复 (30s 超时 → fallback MAVSDK land)
   ↓
   高度 < 0.2m → FALLTHROUGH → MAVSDK land()
```

---

## 二、投放子系统 (`BombDropSystem::execute()`)

```
SCAN → SELECT → GOTO → TRACKING → PREDICT → DROP → CLIMB
 (8s)                                ↓
                           (失败: SELECT下一目标 / timeout)
```

### SCAN (8s, 3.5m)
```
悬停扫描 → 像素聚类 (50px) → 3帧稳定 → 世界坐标映射 → 按 bucketId 投票+尺寸评分排序
```
**详见**: `docs/mapping_algorithm.md`

### SELECT
```
优先选择未使用的最高评分目标
├─ 有未使用 → GOTO
└─ 全部用完 → 返回扫描原点重新扫描(5s) → 旧目标去重(0.5m)检查新目标
              ├─ 有新目标 → SELECT
              └─ 无新目标 → timeout
```

### GOTO (position 模式, 1.2m)
```
offboard_->startPositionModeAt(target_NED, -1.2m)
while dist > 0.5m 水平 || |alt - 1.2m| > 0.5m 垂直: setPositionNed()
超时: configurable (默认 10s)
```

### TRACKING (velocity 模式, 1.2m → 1.8m)
```
offboard_->startVelocityMode()
控制分流:
┌─ VISIBLE / LOST_SHORT ──────────────────────────┐
│ 未commit: 视觉 PID (errPx → vx,vy)                │
│ 已commit: 世界坐标 PID (errN,errE → vx,vy)        │
│ vz: 继续下降                      │
│ 减速区: alt < dropAlt+0.6m 时线性降速            │
│ 安全高度: alt < 0.6m 时强制上升至 0.3m/s         │
├─ LOST_LONG / REACQUIRE (未commit) ───────────────┤
│ vx,vy=0 (悬停), vz=0 (暂停下降)                   │
├─ LOST_CRITICAL (已commit) ───────────────────────┤
│ 世界坐标弱修正 (k=0.15), vz: 继续下降             │
└─ LOST_CRITICAL (未commit) ───────────────────────┘
  → 失败, 返回 SELECT
```

### REACQUIRE 子状态机

```
HOVER (1s) → 悬停检测
    ↓ 未找到
SPIRAL (6s) → 围绕最后已知世界坐标螺旋搜索
    ↓ 半径 0.2m → 0.4m → 0.6m → 1.0m
    ↓ 未找到
CLIMB (4s) → 上升扩宽视野 (最多至 approachAlt + 1.5m)
    ↓ 未找到
ABORT → 失败, 返回 SELECT
```

**防抖**: 退出 REACQUIRE 后 1s 冷却期, 每轮最多 5 次重试

### PREDICT (velocity 模式, ~1.8m)
```
退化跟踪回路:
  已 commit → 世界坐标弱修正
  未 commit → 视觉 PID
  pixelErr < convergeTolPx → DROP
  超时: 15s
```

### DROP
```
释放 payload → dropCount++
落点预测: impact = vel × sqrt(2·alt/g) (日志)
```

### CLIMB (position 模式)
```
爬升回 searchAlt (3.5m), 超时 10s
dropCount ≥ 2 → DONE
否则 → SELECT (下一目标)
```

### 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| searchAlt | 3.5m | SCAN + CLIMB 高度 |
| approachAlt | 1.2m | GOTO 目标高度 |
| dropAlt | 1.8m | 触发放弹高度 |
| 减速区 | 0.6m | dropAlt 以上线性降速区间 |
| minSafeAlt | 0.6m | 绝对最低安全高度 |
| gotoTimeout | 10s | GOTO 超时 |
| predictTimeout | 15s | PREDICT 超时 |
| REACQUIRE 冷却 | 1.0s | 退出后冷却 |
| REACQUIRE 重试 | 5 次 | 最大重试次数 |

---

## 三、TargetTracker 容错状态机

```cpp
enum TargetState { VISIBLE, LOST_SHORT, LOST_LONG, REACQUIRE, LOST_CRITICAL };
```

### 空间匹配 (非标签匹配)

Tracker 通过**像素空间最近邻匹配** (最大距离 100px) 关联连续帧中的同一目标，而非依赖 YOLO bucketId。匹配候选: (a) 上一帧已匹配像素, (b) 首帧时投影的世界坐标。

### 状态转换 (基于连续丢失帧计数, 非时间)

| 连续丢失帧 | 状态 | 行为 |
|------------|------|------|
| 0 | `VISIBLE` | Kalman 更新 + PID |
| 1-3 | `VISIBLE` | 仍保持 VISIBLE, 单帧闪烁不触发 |
| 4-11 | `LOST_SHORT` | Kalman 预测继续, 控制不变 (0.5s lostShortThreshold) |
| 12-39 | `LOST_LONG` | Kalman 预测, 向上层报告悬停等待 (2.0s lostLongThreshold) |
| 40+ (未commit) | `LOST_CRITICAL` | 放弃当前目标 |
| 40+ (已commit) | `REACQUIRE` | 进入恢复策略 |

**关键参数**: `lostConfirmFrames=4`, `criticalLostFrames=40`

### Commit 机制

```
触发条件 (全部满足):
  1. alt ≤ 3.0m           (commitAltThreshold)
  2. err < 0.25m          (commitErrThreshold)
  3. 连续稳定 8 帧         (commitStableFrames)

commit 后:
  - 水平: 世界坐标 PID 替代视觉 PID
  - 高度: 即使丢失也继续下降 (弱修正 k=0.15)
  - 丢失 40+ 帧 → REACQUIRE 而非 LOST_CRITICAL
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
t=0       进入 DROP_SEARCH
t=0-8s    SCAN: 悬停扫描 + 像素聚类建图
t=8s      选目标 → GOTO position 模式 1.2m
t=12s     GOTO 到达 → TRACKING velocity 模式
t=15s     下降到 1.8m, 满足条件 → DROP
t=16s     CLIMB 爬升 3.5m
t=18s     SELECT 下一目标
...
t=90s     全局超时 → forceDropAll → RECON

RTL 阶段:
t=0s      返航巡航 25s @ 4m
t=25s     切换到 position 模式
t=26s     读 H 检测 → 两阶段下降
t=35s     高度 < 0.2m → FALLTHROUGH → MAVSDK land
```
