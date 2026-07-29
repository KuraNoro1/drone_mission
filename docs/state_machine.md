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
| `transitToDrop` | `handleTransitToDrop()` | 位置模式飞往投放区中心，到达条件: 水平 <1m + 高度 <0.5m，超时 25s。重置投放区相关状态，包括 `bombSystem_->reset()` → DROP_SEARCH |
| `dropSearch` | `handleDropSearch()` | **见第二节** |
| `transitToRecon` | `handleTransitToRecon()` | 位置模式飞往侦察区，到达条件: 水平 <3m + 高度 <1m，超时 25s |
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
SCAN → SELECT → GOTO → CENTER → DESCEND → DROP → CLIMB
 (8s)  (pick  (pos模式    (vel模式   (下降)     ↓      ↓
        target) 2.5m)      2.5m)           (失败: 下一目标) (完成: 下一目标)
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
└─ 全部用完 → 重新扫描(8s, +0.5m高度)
              ├─ 有结果 → SELECT
              └─ 无结果 → timeout
```

### GOTO (position 模式, 2.5m)
```
offboard_->startPositionModeAt(target_NED, -2.5m)
while dist > 0.5m || alt not reached: setPositionNed()
timeout 15s
```
**为什么用 position 模式**: 飞控内部位置控制器比伴飞脑 PID 更稳定，适合较长距离导航。粗逼近让目标进入视野，后续 CENTER 阶段做精对准。

### CENTER (velocity 模式, 2.5m)
```
offboard_->startVelocityMode()
像素伺服: errPx → bodyFrame velocity → NED velocity
世界坐标兜底: 视觉丢失 >0.5s → 导航到 SCAN 地图坐标
收敛条件: pixelErr < 15px 持续 0.6s
失败: 丢失 >5s 或超时 30s → 返回 SELECT
```

### DESCEND (velocity 模式, 2.5m → 1.8m)
```
继承 CENTER PID 积分 (不 reset)
下降速率: 0.3 m/s, 视觉有效时下降
到达 1.8m: 5条件投弹检查
  cond1: pixelErr < 15px
  cond2: 水平速度 < 0.15m/s
  cond3: |alt - 1.8m| < 0.1m
  cond4: 稳定 > 0.3s
视觉丢失 >3s: 放弃本目标
```

### DROP
```
落点预测 (仅日志):
  tFall = sqrt(2h/g)
  impact_N = vx * tFall
释放逻辑: 第一弹 Left, 第二弹 Right
PWM: releasePwm=1900 (800ms), 然后 holdPwm=1100
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
t=8s     选目标 → GOTO position模式 2.5m (到达条件: 水平<0.5m)
t≈12s    GOTO到达 → CENTER velocity模式 2.5m (像素伺服)
t=20s    CENTER 收敛 → DESCEND 开始下降
t=23s    降至 1.8m, 五条件满足 → 投弹
t=24s    CLIMB 爬升至 3.5m
t=28s    SELECT 下一目标
...
t=90s    全局超时 → 强制投弹 → RECON
```
