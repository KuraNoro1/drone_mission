# 视觉伺服对准 + 投弹决策策略

> 基于仿真日志分析 | 2026-07-15

---

## 一、日志分析：当前问题诊断

### 1.1 检测质量特征

| 现象 | 表现 | 频率 |
|------|------|------|
| 连续检测 | `bucket3: (540, 625)` 持续 10-20 帧 | 高频区段 |
| 检测丢失 | `None` 连续 5-30 帧 | 间歇性 |
| ID 跳变 | 同一物理桶在 桶1/2/3 之间切换 | 每帧都可能变化 |
| 多桶共存 | 同时输出 `bucket1` + `bucket2` | 少数帧 |
| 位置跳跃 | 桶坐标在相邻帧内偏移 5-30px | 常态 |

### 1.2 晃动根因

```
帧 N:   检测到桶 @(540, 625) → PID 计算 vx, vy → 无人机移动
帧 N+1: 丢失目标 ("None")    → 复用缓存 (540, 625) → 继续移动
帧 N+5: 新检测 @(510, 608)   → 新误差，PID 反向修正 → 晃动
帧 N+6: 丢失目标              → 再次缓存 → 惯性过冲
```

**结论：缓存 + 检测跳跃 = 正反馈振荡。** 丢失目标时的策略是晃动主因。

---

## 二、视觉伺服状态机设计

在 `runVisualServoLoop()` 内部引入**子状态机**，替代当前简单的"有检测→跟踪 / 无检测→缓存"二元逻辑。

```
                  ┌──────────────────────────────────┐
                  │                                  │
                  ▼                                  │
           ┌──────────┐    检测到桶     ┌──────────────┐
  ────────►│ SEARCHING │──────────────►│  TRACKING    │
           │ (悬停等待) │               │ (PID 跟随)    │
           └──────────┘               └──────┬───────┘
                  ▲                          │
                  │         误差 < 阈值       │
                  │        + 连续 N 帧        │
                  │                          ▼
                  │                   ┌──────────────┐
                  │       丢目标      │  CONVERGED   │
                  │     ◄──────────── │ (已收敛对齐)  │
                  │                   └──────┬───────┘
                  │                          │
                  │         连续保持 M 帧      │
                  │         + 无丢帧          │
                  │                          ▼
                  │                   ┌──────────────┐
                  │                   │ READY_DROP   │
                  │                   │ (可以投弹)    │
                  │                   └──────────────┘
                  │
                  │ 注意: CONVERGED 和 READY_DROP 状态丢目标时
                  │ 直接回到 SEARCHING，不走缓存复用
                  └─────────────────────────────────┘
```

### 2.1 各状态行为

| 状态 | 行为 | 进入条件 | 退出条件 |
|------|------|----------|----------|
| **SEARCHING** | 悬停保持当前位置，vx=0 vy=0，等待检测 | 连续丢目标超过 `coastFrames` | 检测到桶且置信度有效 |
| **TRACKING** | 正常 PID 跟随目标桶，全速修正 | 从 SEARCHING 检测到桶 | 误差 < `convergeTolPx` 持续 `convergeFrames` 帧 |
| **CONVERGED** | 降速 PID 精细微调，检测丢失立即悬停 | 从 TRACKING 收敛到位 | 连续保持 `holdFrames` 帧 或 丢目标 |
| **READY_DROP** | 悬停保持，触发投弹 | 从 CONVERGED 连稳 `holdFrames` 帧 | 投弹完成 或 丢目标 → SEARCHING |

### 2.2 关键参数

```yaml
visualServo:
  # 误差阈值
  convergeTolPx: 30        # 挂载点与桶中心像素误差 < 此值视为"对准" (原 40)
  
  # 帧数计数器 (以 50ms 循环周期计, 1s = 20帧)
  convergeFrames: 10       # 持续对准 10 帧 (0.5s) 进入 CONVERGED
  holdFrames: 30           # CONVERGED 持续 30 帧 (1.5s) 进入 READY_DROP
  
  # 丢目标处理
  coastFrames: 8           # 丢目标后惯性滑行 8 帧 (0.4s), 然后进入 SEARCHING
  coastDecay: 0.7          # 滑行期间每帧速度衰减系数
  
  # 速度限制
  fineVelocityMax: 0.3     # CONVERGED 状态最大速度 (m/s), 精细微调
  coarseVelocityMax: 1.0   # TRACKING 状态最大速度 (m/s)

  # 桶位置平滑
  smoothingAlpha: 0.4      # 指数移动平均 alpha (0=只用旧值, 1=不用平滑)
  maxJumpPx: 80            # 相邻帧间桶位置跳变 > 此值视为检测噪声, 丢弃
```

---

## 三、临时丢目标处理策略（核心）

### 3.1 三级响应

```
丢目标时刻
    │
    ├─ 阶段1: COAST (0~coastFrames 帧, ~0.4s)
    │   ├── 保持最后 PID 输出的速度方向
    │   ├── 每帧乘以 coastDecay 衰减
    │   └── 若此期间重新检测到桶 → 回到 TRACKING
    │
    ├─ 阶段2: HOLD (8~inf 帧)
    │   ├── vx=0, vy=0, 完全悬停
    │   ├── 仍然 poll 管道, 等待检测恢复
    │   ├── 若重新检测到桶 → 判断是"同一个桶"还是"新桶"
    │   │   ├── 新检测位置与上次位置差 < reconnectPx → 回到 TRACKING
    │   │   └── 新检测位置与上次位置差 > reconnectPx → 视为新桶, 重新搜索
    │   └── 进入 SEARCHING 子状态
    │
    └─ 阶段3: RE-DETECT
        ├── 重新检测到桶后, 根据优先级选择目标桶
        └── 平滑过渡到新目标 (避免阶跃速度跳变)
```

### 3.2 关键点：CONVERGED/READY_DROP 状态丢目标

```
CONVERGED 或 READY_DROP 状态下丢失目标:
  → 立即进入 HOLD (跳过 COAST 阶段)
  → 状态回退到 SEARCHING
  → 不继续依赖缓存坐标驱动 PID
```

**理由**：已对准状态下丢目标通常是因为桶移出视野或遮挡，此时如果按缓存坐标继续移动，几乎 100% 导致过冲。

### 3.3 关键点：桶位置平滑

连续检测时，桶坐标在相邻帧间通常有 5-30px 的随机跳动。使用 EMA 滤波：

```
smoothed_cx = alpha * raw_cx + (1-alpha) * smoothed_cx_prev
smoothed_cy = alpha * raw_cy + (1-alpha) * smoothed_cy_prev
```

取 `alpha = 0.4`，这意味着平滑后的位置是最近约 5 帧的加权平均。同时，如果新检测与平滑位置的差值超过 `maxJumpPx`（80px），丢弃该帧（视为误检）。

---

## 四、投弹判定逻辑

### 4.1 投弹条件（全部满足）

1. **子状态 = READY_DROP**
2. **高度验证**：当前高度与目标投弹高度差 < 0.2m
3. **速度验证**：`sqrt(vx² + vy²)` < 0.15 m/s（确认静止）
4. **连续检测验证**：最近 N 帧内有 ≥ 80% 帧检测到桶（排除偶然检测）

### 4.2 投弹执行

```
条件满足
    │
    ▼
记录当前 (cx, cy) 和目标挂载点 (mount_u, mount_v)
    │
    ▼
触发舵机: servo.release(左/右挂载点)
    │
    ▼
等待舵机执行完成 (releaseDurationMs)
    │
    ▼
记录投弹完成 → 上升至巡航高度 → 进入侦察区
```

### 4.3 多桶投弹策略

```
在 SEARCHING 状态重新检测到桶后:
    │
    ├─ 检查新桶是否与已投过的桶位置接近 (< proximityPx)
    │   ├─ 是 → 跳过 (已投过, 换另一个)
    │   └─ 否 → 选择为目标, 开始新一轮对准
    │
    └─ 若所有桶都已投过 → 上升进入侦察区
```

增加 `droppedBuckets` 列表记录已投的桶坐标，避免对同一目标重复投弹。

---

## 五、桶 ID 稳定性处理

### 5.1 问题

同一物理桶在不同帧被分类为不同的 ID（因为 YOLO 检测框宽度受远近和角度影响）。

### 5.2 策略：用位置跟踪替代 ID 跟踪

```
不依赖 bucketId 区分桶，改用空间位置:

1. 维护 trackedBuckets[] 列表, 每个元素 = {avg_cx, avg_cy, bid_history[], lastSeen}
2. 新检测到的桶, 找到 trackedBuckets[] 中最近的(空间距离 < trackDistancePx):
   ├─ 找到 → 更新该跟踪目标的 avg_cx/cy, 追加 bid 到 history
   └─ 未找到 → 创建新的跟踪目标
3. 选择目标桶时:
   ├─ priority=0 (大桶): 选 bid_history 中出现桶3频率最高的
   └─ priority=1 (小桶): 选 bid_history 中出现桶1频率最高的
4. 超过 maxLostFrames 未更新的跟踪目标 → 删除
```

参数建议：
- `trackDistancePx`: 50px（同一桶在不同帧的合理漂移范围）
- `maxLostFrames`: 60（3秒不出现则放弃该跟踪目标）

---

## 六、PID 参数调优建议

当前 PID 参数导致超调和振荡，建议两套参数对应两个阶段：

| 参数 | TRACKING 阶段 | CONVERGED 阶段 | 说明 |
|------|-------------|---------------|------|
| kp | 0.8 | 0.3 | 收敛后降低比例增益 |
| ki | 0.02 | 0.01 | 减小积分防止过冲 |
| kd | 0.0 | 0.1 | 收敛后加入微分抑制振荡 |
| maxVel | 1.0 | 0.3 | 收敛后限速 |
| centerTolPx | — | 30 | 收敛判定阈值 |

---

## 七、实现优先级建议

| 优先级 | 改动 | 影响 |
|--------|------|------|
| P0 | 丢目标后的 COAST→HOLD 策略 | 消除晃动主因 |
| P0 | CONVERGED 状态丢目标立即悬停 | 防止对准后过冲 |
| P1 | 桶坐标 EMA 平滑 + 跳变过滤 | 提升跟踪稳定性 |
| P1 | 投弹多条件判定（状态+高度+速度+检测率） | 防止误投 |
| P2 | 桶空间跟踪（位置替代 ID） | 解决 ID 跳变 |
| P2 | 两阶段 PID 参数 | 减少超调 |
| P3 | 已投桶去重 | 多桶场景 |

---

## 八、调试日志增强

建议在现有 `[VSERVO]` 日志基础上增加：

```
[VSERVO] 桶3[L]: pos=(540,625) mount=(218,320) err=(-322,-305)px
         vel=(0.8,-0.7)m/s alt=1.2m state=TRACKING smooth=(535,620)

[VSERVO] state: TRACKING→CONVERGED  err=22px<30px  convergeCount=10

[VSERVO] state: CONVERGED→SEARCHING  target lost! hold at current pos

[VSERVO] state: SEARCHING→TRACKING  re-detected bucket @(532,618) reconnected

[VSERVO] DROP READY: state=READY_DROP  err=15px  vel=0.05m/s  alt=1.18m  detectRate=92%
```
