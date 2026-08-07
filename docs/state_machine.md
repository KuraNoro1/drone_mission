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

### GOTO (position 模式, 3.0m)
```
startPositionModeAt 内部自动处理 velocity→position 切换 (无外部 stop/sleep)
while dist > 0.3m || |alt-3.0| > 0.2m: setPositionNed()
timeout 15s
```

### CENTER (velocity 模式, 3.0m)
```
startVelocityMode 内部处理 position→velocity 切换, 无外部 stop/sleep
初始稳定阶段用高度 P 控制而非 vz=0, 防止模式切换时跌落
目标匹配: 取画面中心最近检测 (距中心 <600px), 无世界坐标投影
像素伺服: PID(kp=0.98, ki=0.15) → bodyFrame velocity → NED velocity
收敛条件: 滑动窗口 20帧中≥4帧 pixelErr<40px (1.0s, 容忍80%丢帧)
失败: 丢失 >5s 或超时 30s → 返回 SELECT
```

### DESCEND (velocity 模式, 3.0m → 1.8m)
```
继承 CENTER PID 积分 (不 reset)
下降前对齐: 滑动窗口 16帧中≥3帧 pixelErr<40px (0.8s, 容忍81%丢帧)
下降速率: 0.3 m/s, 视觉丢失 2s 内保持下降 (避免间歇检测卡住)
丢检测>2s: 悬停等待; 丢检测>5s: 放弃本目标
到达 1.8m: 滑动窗口 10帧中≥3帧满足 pix+vel+alt (0.5s, 容忍70%丢帧)
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
