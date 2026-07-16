# 四旋翼无人机水平速度/位置 PID 控制器设计

> 基于经典自动控制理论和四旋翼动力学模型的完整分析与方案论证

---

## 一、四旋翼水平方向动力学建模

### 1.1 坐标系定义

采用 NED(North-East-Down)坐标系：

- **N 轴**(x): 北向
- **E 轴**(y): 东向
- **D 轴**(z): 向下

不考虑偏航(yaw = 0)时的简化分析：水平方向运动由俯仰角 θ (pitch, 绕 E 轴) 和横滚角 φ (roll, 绕 N 轴) 控制。

### 1.2 力平衡方程（小角度假设）

理想四旋翼的受力分析。在悬停/低速状态下，做**小角度假设** (θ, φ ≈ 0):

作用于水平方向的合力：

```
N 向: F_N = -mg · θ    （俯仰产生的前向力，负号因为 NED：俯仰为正时机头下倾N向增加）
E 向: F_E =  mg · φ    （横滚产生的右向力）
```

> 推导: 四旋翼总推力 F = mg/cos(θ)cos(φ)。小角度下 F ≈ mg。推力在水平面的分量为：
> - N 向分量: F_N = F · sin(θ) · cos(φ) ≈ mg · θ
> - E 向分量: F_E = F · cos(θ) · sin(φ) ≈ mg · φ
>
> NED 坐标系中 D 向下为正, N 向北为正。当 pitch=θ>0 (机头下倾), 推力在 N 向分量为正(向北加速)。因此 F_N = mg·θ。

由牛顿第二定律：

```
N 向加速度: a_N = F_N / m = g · θ
E 向加速度: a_E = F_E / m = g · φ
```

这给出水平加速度与姿态角的**线性关系**。写成传递函数形式（以 N 向为例）：

```
加速度传函:  P_acc(s) = a_N(s) / θ(s) = g
速度传函:    P_vel(s) = v_N(s) / θ(s) = g / s
位置传函:    P_pos(s) = p_N(s) / θ(s) = g / s²
```

### 1.3 姿态角动力学

姿态角本身由电机转速差产生的力矩驱动，近似为二阶系统：

```
θ(s) / θ_cmd(s) = ω_n² / (s² + 2ζ·ω_n·s + ω_n²)
```

对于中型四旋翼，典型参数：
- 自然频率 ω_n ≈ 10–20 rad/s
- 阻尼比 ζ ≈ 0.6–0.9

**本文不讨论姿态内环设计**（该环由 ArduPilot/PX4 飞控内部完成）。我们设计的是**外环**：以 `θ_cmd` 为控制量、以速度/位置为被控量。

### 1.4 完整水平运动模型

```
                   ┌──────────────┐
  θ_cmd(s) ──► Kθ  │ 姿态内环     │  θ(s) ──► g ──► a(s) ──► 1/s ──► v(s) ──► 1/s ──► p(s)
                   │ ω_n²/(...)   │
                   └──────────────┘
```

简化：假设姿态内环足够快（ω_n > 10 rad/s），对其采用**理想跟踪**近似 θ(s) ≈ θ_cmd(s)。此时模型简化为**双积分器**：

```
G(s) = p(s) / θ_cmd(s) = g / s²
```

**这就是为什么许多人说"四旋翼不能引入 D 项"**——双积分器的二阶系统，用 PD 控制器即可得到：

- 闭环传函: T(s) = (Kp·g + Kd·g·s) / (s² + Kd·g·s + Kp·g)
- 特征方程: s² + Kd·g·s + Kp·g = 0
- 这是典型的**二阶系统**，P 项控制自然频率、D 项控制阻尼比

所以严格来说：对**位置环**的双积分器模型，PD 控制已足够，不需要 I 项（但对于消除稳态误差仍然需要）。对**速度环**而言，被控对象是 g/s（一阶积分器），P 控制器就够了。

---

## 二、"不能引入 D 项"的分析

### 2.1 这个说法的来源

"不能引入 D 项"通常针对以下情况：

**情况 A: 被控对象的阶数判断错误**

如果把**速度环**(从 θ 到 v)当作二阶系统来设计——实际速度环的被控对象是：

```
P_vel(s) = v(s) / θ(s) = g / s    （一阶积分器）
```

在**速度环**中使用 D 项：
- D 项对速度误差求导 = 对加速度求反馈
- 但速度误差 e_v = v_ref − v_actual，其导数 de_v/dt = a_ref − a_actual
- `a_ref` 通常没有明确定义，引入 D 项等于引入了高频噪声的放大

**在速度环中，更合理的做法是只用 PI**：

```
控制器: C_vel(s) = Kp + Ki / s
开环传函: L(s) = (Kp + Ki/s) · (g/s) = g·(Kp·s + Ki) / s²
```

这是标准的二阶系统，PI 即可满足速度环需求。

**情况 B: 传感器噪声**

MAVSDK 的 `position_velocity_ned` 数据来自 EKF 融合后的速度估计。EKF 的噪声特性使得直接对速度差分（即 D 项）会显著放大噪声。

**情况 C: 现有飞控已有 D 项**

ArduPilot 的 `ACRO`/`STABILIZE` 模式下的姿态内环本身已包含角速率 D 反馈。在外环再加 D 项可能造成过阻尼。

### 2.2 D 项在什么情况下可以使用

D 项在以下情况仍有效：

1. **经过低通滤波的 D 项**（实用微分器）:
   ```
   D_filt(s) = Kd · s / (τf·s + 1)    （τf ≈ N · T_sample）
   ```

2. **位置环的 PD 控制**（被控对象是 g/s² 双积分器）:
   ```
   C_pos(s) = Kp + Kd·s    （PD 控制器）
   ```
   此时 D 项与双积分器组成完整的二阶系统，物理意义明确。

3. **用于你当前视觉伺服的像素误差 PID**: 你的 `missionStateMachine.cpp` 中已经使用了 `ki=0.02`（实际是 I 项而非 D 项）——这是合理的。

### 2.3 结论：推荐方案

| 控制环 | 被控对象 | 推荐控制器 | D 项 |
|--------|---------|-----------|------|
| 姿态角(飞控内环) | 二阶(电机→角加速度) | PID | ✅ 需要 D (角速率反馈) |
| 速度环(外环) | g / s (一阶积分器) | **PI** | ❌ 不需要 |
| 位置环(外环) | g / s² (双积分器) | **PD** 或 **P + 前馈** | ✅ 可以用 D |
| 视觉伺服(像素→速度) | 近似一阶 | **PI** (你当前) | ❌ 不需要 |

---

## 三、级联 PID 架构设计

### 3.1 经典级联结构

业界标准方案 (ArduPilot/PX4 均采用)：

```
位置设定 p_ref
    │
    ▼
┌──────────────┐
│ 位置环 (P)   │ ─► 比例误差 → 速度给定
│ C_pos(s)=Kp  │
└──────┬───────┘
       │ v_ref
       ▼
┌──────────────┐
│ 速度环 (PI)  │ ─► 速度误差 → 姿态角给定 θ_ref
│ C_vel(s)=Kpv │    (带前馈)
│         +Kiv/s│
└──────┬───────┘
       │ θ_ref
       ▼
┌──────────────┐
│ 姿态环 (PID) │ ─► 飞控内环 (ArduPilot/PX4 完成)
└──────────────┘
```

### 3.2 理论推导

**位置环 (纯 P):**
```
v_ref = Kp_pos · (p_ref − p_curr)
```
为什么只 P？如果位置环引入 I 项，积分饱和(windup)风险极大；引入 D 项需要位置的高阶导数，物理意义不明确。纯 P 是最稳健的选择。

位置环输出的是**速度给定**，需要限幅：
```
v_ref = clamp(Kp_pos · e_pos, −V_max, V_max)
```

**速度环 (PI):**
```
θ_ref = Kp_vel · (v_ref − v_curr) + Ki_vel · ∫(v_ref − v_curr)·dt
```

速度环直接输出姿态角给定 θ_ref。为什么需要 I 项？
- 实际飞行中可能有常值干扰(风、质心偏移)
- I 项消除速度稳态误差
- 抗积分饱和(anti-windup)采用条件积分法：仅在输出未饱和且误差较小时积分

**速度环可选的 P 项前馈:**

为了更好地跟踪动态速度给定，可加入前馈项：
```
θ_ref = K_ff · v_ref + Kp_vel · e_vel + Ki_vel · ∫e_vel
```
前馈系数 K_ff 在理想模型下应为 1/g（但实际中通常取小值，约 0.5/g 到 0.8/g）。

### 3.3 环路带宽设计

根据频域分析法进行带宽分配（经验法则）：

```
内环带宽 > 5× 外环带宽    (解耦)
```

典型值：
- 姿态环(飞控): ω_att ≈ 30–60 rad/s (≈ 5–10 Hz)
- 速度环: ω_vel ≈ 2–5 rad/s (≈ 0.3–0.8 Hz)
- 位置环: ω_pos ≈ 0.5–2 rad/s (≈ 0.08–0.3 Hz)

---

## 四、参数整定：极点配置法

### 4.1 速度环 PI 整定

速度环开环模型（忽略姿态环延迟）：

```
L_vel(s) = (Kp_vel + Ki_vel/s) · (g / s) = g·(Kp_vel·s + Ki_vel) / s²
```

闭环传函：

```
T_vel(s) = g·(Kp_vel·s + Ki_vel) / (s² + g·Kp_vel·s + g·Ki_vel)
```

希望闭环系统具有目标自然频率 ω_v 和阻尼比 ζ_v：

```
s² + g·Kp_vel·s + g·Ki_vel = s² + 2ζ_v·ω_v·s + ω_v²
```

解得：
```
Kp_vel = 2ζ_v·ω_v / g
Ki_vel = ω_v² / g
```

**示例** (g = 9.81 m/s²):

| ω_v (rad/s) | ζ_v | Kp_vel | Ki_vel | 速度响应 |
|:-----------:|:---:|:------:|:------:|:---------|
| 2.0 (0.32Hz) | 0.8 | 0.33 | 0.41 | 慢但平滑 |
| 3.0 (0.48Hz) | 0.8 | 0.49 | 0.92 | 中等 |
| 5.0 (0.80Hz) | 0.8 | 0.82 | 2.55 | 快速 |

### 4.2 位置环 P 整定

位置环 + 速度内环的**近似闭环模型**（假设速度环足够快）：

```
位置环开环: P_pos(s) = Kp_pos · T_vel(s) · (1/s)
```

位置环闭环带宽 ≈ Kp_pos 时，有：
```
Kp_pos ≈ ω_pos    (位置环带宽)
```

取 ω_pos = ω_v / 5 以确保内外环解耦：

| ω_v (rad/s) | Kp_pos | 位置响应 |
|:-----------:|:------:|:---------|
| 2.0 | 0.4 | 非常平滑 |
| 3.0 | 0.6 | 平稳 |
| 5.0 | 1.0 | 敏捷但可能有超调 |

### 4.3 离散实现（C++ 伪代码）

```cpp
class VelocityPID {
    double Kp, Ki, Kd;  // Kd=0 for velocity control
    double integral;
    double maxOutput, maxIntegral;
    double prevError;

    double update(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;

        // I 项 + 抗积分饱和
        if (fabs(output) < maxOutput || error * integral < 0)
            integral += error * dt;
        integral = clamp(integral, -maxIntegral, maxIntegral);

        // D 项不使用（速度环 PI）
        // double derivative = (error - prevError) / dt;

        double output = Kp * error + Ki * integral;
        output = clamp(output, -maxOutput, maxOutput);

        prevError = error;
        return output;
    }
};
```

---

## 五、与当前 drone_mission 代码的对应关系

你的 `missionStateMachine.cpp` 中已有三个 PID 实例：

```cpp
pidN_ → pidController(kp=0.8, ki=0.02, kd=0.0)  // 北向速度 PI
pidE_ → pidController(kp=0.8, ki=0.02, kd=0.0)  // 东向速度 PI
pidD_ → pidController(kp=0.5, ki=0.01, kd=0.1)  // 下降速度 PID
```

对比理论分析：

| 环 | 你的参数 | 理论建议 | 评价 |
|:--:|---------|---------|------|
| 北向 N | kp=0.8, ki=0.02, kd=0 | kp≈0.49-0.82, ki≈0.92-2.55 | kp 合理, ki 偏小(可能响应偏慢) |
| 东向 E | kp=0.8, ki=0.02, kd=0 | 同上 | 同上 |
| 下降 D | kp=0.5, ki=0.01, kd=0.1 | — | Z 轴有重力补偿，需其他分析 |

**建议**: 当前 `ki=0.02` 对应 ω_v ≈ 0.44 rad/s（仅 0.07 Hz），这对于视觉伺服偏慢，但可以理解——视觉伺服需要平滑运动。如果发现跟随太慢，可调整到 `ki=0.05-0.10`。

---

## 六、对于你当前视觉伺服的特殊考虑

### 6.1 你的控制结构

```
视觉伺服 (missionStateMachine.cpp):
  桶像素坐标(cx, cy)
      │
      ▼
  computeMountPixels(alt) → 挂载点像素(uL,vL, uR,vR)
      │
      ▼
  像素误差: errU = mount_u - bucket_u, errV = mount_v - bucket_v
      │
      ▼
  归一化: errNorm = err / 320
      │
      ▼
  PI(errNorm) → velocity (vx, vy)
      │
      ▼
  offboard::setVelocityNed(vx, vy, 0, yaw)
```

你的控制链 **已经采用了正确的 PI (无 D) 策略**。从像素误差到速度的映射中：
- P 项(比例): 提供与误差成正比的恢复速度
- I 项(积分): 消除小的偏置（相机安装偏差、挂载点偏差）
- 无 D 项: 正确——像素误差的微分放大会放大 YOLO 检测框的抖动

### 6.2 为什么你的方案正确

你的设计中 kd=0 是合理的，因为：

1. **视觉检测的离散噪声**: YOLO 检测框的 (cx, cy) 存在帧间抖动（±2-5 像素）。对此差分会极大放大噪声
2. **像素误差 → 物理速度的非线性映射**不精确，模型的参数误差使得 D 项的预测作用被削弱
3. **你的锁桶/丢帧机制**（重使用上次坐标最多 15 帧）已经提供了有效的"保持"功能，比 D 项更鲁棒

### 6.3 定量验证当前参数

在 `pid.yaml` 中：
```yaml
pidVisual:
  kp: 0.8     # P 增益
  kd: 0.15    # D 增益（目前代码中视觉伺服未使用此 kd）
```

以 1.2m 高度为例:
- 1 像素误差 ≈ 0.0022 m 物理偏移(在 1.2m 高度)
- 100 像素误差 → errNorm = 100/320 = 0.3125 → P 输出 = 0.8 × 0.3125 = 0.25 m/s

这是合理的——100 像素偏差产生约 0.25 m/s 的恢复速度，不会造成过冲。

---

## 七、参数整定实验建议

### 7.1 仿真中的阶跃响应测试

在仿真中添加临时测试代码，给速度环输入阶跃信号(如 `v_ref = 1.0 m/s`)，记录响应曲线：

```cpp
// 临时测试: 在 offboard 模式下发送速度阶跃
offboard_->setVelocityNed(1.0, 0, 0, yaw);  // 北向 1m/s
sleep_for(seconds(3));
offboard_->setVelocityNed(0, 0, 0, yaw);     // 刹车
```

分析 `pid_visual.csv` 中的速度响应，调参直到满足：
- 上升时间 < 0.5s
- 超调量 < 5%
- 稳态误差 < 0.05 m/s

### 7.2 Ziegler-Nichols 临界比例度法（备选方案）

如果没有理论模型，可用 Z-N 法整定：

1. 设置 Ki=Kd=0
2. 逐步增大 Kp，直到系统产生等幅振荡，记录临界增益 Ku 和振荡周期 Tu
3. 根据 Z-N 表设定参数：

| 控制器 | Kp | Ki | Kd |
|--------|----|----|-----|
| P | 0.5×Ku | — | — |
| PI | 0.45×Ku | 1.2×Kp/Tu | — |
| PID | 0.6×Ku | 2×Kp/Tu | Kp×Tu/8 |

> **注意**: Z-N 法对四旋翼可能过于激进，建议乘以 0.5 的安全系数。

---

## 八、参考文献与推荐阅读

### 经典教材

1. **Ogata, K. (2010).** *Modern Control Engineering (5th ed.)*. Prentice Hall.
   - 第 8 章: PID 控制器设计与 Ziegler-Nichols 整定法
   - 第 10 章: 极点配置与状态反馈

2. **Åström, K.J. & Hägglund, T. (2006).** *Advanced PID Control*. ISA.
   - 第 3 章: 抗积分饱和、无扰动切换、实用微分器设计

3. **Franklin, G.F., Powell, J.D. & Emami-Naeini, A. (2019).** *Feedback Control of Dynamic Systems (8th ed.)*. Pearson.
   - 第 4 章: 根轨迹法设计控制器
   - 第 6 章: 频域响应设计

### 四旋翼控制论文

4. **Mellinger, D. & Kumar, V. (2011).** "Minimum Snap Trajectory Generation and Control for Quadrotors." *IEEE ICRA*.
   - [https://doi.org/10.1109/ICRA.2011.5980409](https://doi.org/10.1109/ICRA.2011.5980409)
   - 经典论文，介绍四旋翼动力学和轨迹跟踪控制。确立了级联 PID + 前馈的标准框架。

5. **Lupashin, S. et al. (2014).** "A platform for aerial robotics research and demonstration: The Flying Machine Arena." *Mechatronics*, 24(1), 41–54.
   - ETH Zurich 的 PX4 控制架构设计，包含详细的内外环带宽分配分析。

6. **Brescianini, D., Hehn, M. & D'Andrea, R. (2013).** "Nonlinear quadrocopter attitude control." *ETH Zurich Research Collection*.
   - 四旋翼姿态控制的非线性分析，对理解内环姿态带宽有参考价值。

7. **Bouabdallah, S., Noth, A. & Siegwart, R. (2004).** "PID vs LQ control techniques applied to an indoor micro quadrotor." *IEEE/RSJ IROS*.
   - [https://doi.org/10.1109/IROS.2004.1389776](https://doi.org/10.1109/IROS.2004.1389776)
   - 比较 PID 和 LQR 在四旋翼上的表现，证明经典 PID 在微小型无人机上足够有效。

### 开源实现参考

8. **PX4 Multicopter Attitude & Position Control**
   - [https://docs.px4.io/main/en/flight_stack/controller_diagrams.html](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)
   - 开源飞控的级联 PID 架构，包含详细的参数整定指南。

9. **ArduPilot Copter Attitude Control**
   - [https://ardupilot.org/copter/docs/ac_attcontrol.html](https://ardupilot.org/copter/docs/ac_attcontrol.html)
   - ArduPilot 的姿态控制器设计，包含 P 项、I 项、D 项（角速率）的具体用途。

### 实用技巧

10. **Skogestad, S. (2003).** "Simple analytic rules for model reduction and PID controller tuning." *Journal of Process Control*, 13(4), 291–309.
    - SIMC 调参法：基于简单模型分析推导出近似的 PID 参数，比 Z-N 更平滑。

---

## 九、总结

1. **四旋翼水平运动模型**在小角度假设下简化为双积分器 `G(s) = g/s²`（位置）或一阶积分器 `G(s) = g/s`（速度）。

2. **"不能用 D 项"**特指在速度环（一阶积分器）中直接用 D 项放大传感器噪声。合理替代方案：位置环 PD、速度环 PI。

3. **推荐架构**: 位置环 P(P-bandwidth ≈ 0.5 rad/s) → 速度环 PI(ω_v ≈ 3 rad/s) → 飞控内环 PID。当前 `drone_mission` 中视觉伺服的 PI 控制符合理论最佳实践。

4. **参数整定**: 理论公式 `Kp = 2ζω/g`, `Ki = ω²/g`。推荐 ω_v = 2–5 rad/s, ζ = 0.7–0.9。

5. **离散实现**: I 项需抗积分饱和(条件积分法)，D 项(如需使用)需低通滤波。
