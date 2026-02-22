# 实验五：固定翼飞行器控制与仿真

**课程**: ETH Zurich — Robot Dynamics  
**实验编号**: Exercise 5  
**主题**: Fixed-Wing Aircraft Control and Simulation  
**工具**: MATLAB / Simulink  
**Simulink 模型文件**: `fw_sim.slx`

---

## 1. 实验概述 (Lab Overview)

### 1.1 实验目标

本实验围绕固定翼飞行器 (Fixed-Wing Aircraft) 展开，核心目标包括：

- 建立固定翼飞行器的 **六自由度动力学模型** (6-DOF Dynamic Model)
- 理解并计算飞行器的 **配平状态** (Trim Condition)
- 设计 **纵向控制器** (Longitudinal Controller) 和 **横向控制器** (Lateral Controller)
- 在 Simulink 环境中完成闭环仿真，验证控制效果

### 1.2 实验流程

```
建模 (Modeling) → 配平 (Trim) → 线性化 (Linearization) → 控制设计 (Control Design) → 仿真验证 (Simulation)
```

### 1.3 Simulink 模型说明

打开 `fw_sim.slx`，模型主要包含以下子系统：

| 子系统 | 功能 |
|--------|------|
| `Plant` | 飞行器六自由度动力学 |
| `Aerodynamics` | 气动力与气动力矩计算 |
| `Controller` | 飞行控制律 |
| `Environment` | 大气模型与重力 |
| `Visualization` | 三维飞行轨迹可视化 |

---

## 2. 固定翼建模 (Fixed-Wing Modeling)

### 2.1 坐标系定义 (Reference Frames)

固定翼飞行器分析中涉及三个关键坐标系：

#### a) 惯性坐标系 {I} (Inertial Frame)

- 原点固定于地面某参考点
- $x_I$ 指向北，$y_I$ 指向东，$z_I$ 指向地心（NED 约定）
- 用于描述飞行器的绝对位置和姿态

#### b) 机体坐标系 {B} (Body Frame)

- 原点位于飞行器质心 (Center of Gravity, CG)
- $x_B$ 沿机身纵轴指向机头
- $y_B$ 指向右翼
- $z_B$ 向下，与 $x_B$, $y_B$ 构成右手系

#### c) 风轴坐标系 {W} (Wind Frame)

- 原点同样位于质心
- $x_W$ 沿气流速度方向 (Airspeed Vector)
- 用于定义气动力的方向

#### d) 气流角定义

**攻角** (Angle of Attack) $\alpha$ 和 **侧滑角** (Sideslip Angle) $\beta$ 定义了机体系与风轴系之间的关系：

$$
\alpha = \arctan\left(\frac{w}{u}\right)
$$

$$
\beta = \arcsin\left(\frac{v}{V}\right)
$$

其中 $u, v, w$ 为机体系下的速度分量，$V = \sqrt{u^2 + v^2 + w^2}$ 为总空速 (True Airspeed)。

从风轴系到机体系的旋转矩阵为：

$$
R_{WB} = R_y(\alpha) \cdot R_z(-\beta)
$$


### 2.2 六自由度运动方程 (6-DOF Equations of Motion)

固定翼飞行器是一个刚体，具有六个自由度：三个平动自由度和三个转动自由度。

#### a) 状态向量

完整状态向量为 12 维：

$$
\mathbf{x} = \begin{bmatrix} x & y & z & u & v & w & \phi & \theta & \psi & p & q & r \end{bmatrix}^T
$$

| 符号 | 含义 | 坐标系 |
|------|------|--------|
| $x, y, z$ | 位置 (Position) | 惯性系 {I} |
| $u, v, w$ | 线速度 (Linear Velocity) | 机体系 {B} |
| $\phi, \theta, \psi$ | 欧拉角：滚转、俯仰、偏航 (Roll, Pitch, Yaw) | — |
| $p, q, r$ | 角速度 (Angular Velocity) | 机体系 {B} |

#### b) 平动方程 (Translational Dynamics)

在机体系 {B} 中，牛顿第二定律写为：

$$
m \dot{\mathbf{V}}_B + \boldsymbol{\omega} \times (m \mathbf{V}_B) = \mathbf{F}_{aero} + \mathbf{F}_{gravity} + \mathbf{F}_{thrust}
$$

展开为分量形式：

$$
m(\dot{u} + qw - rv) = F_x
$$

$$
m(\dot{v} + ru - pw) = F_y
$$

$$
m(\dot{w} + pv - qu) = F_z
$$

其中各力的分量包括：

**重力** (Gravity)：在机体系中的投影为

$$
\mathbf{F}_{gravity}^B = m g \begin{bmatrix} -\sin\theta \ \sin\phi \cos\theta \ \cos\phi \cos\theta \end{bmatrix}
$$

**推力** (Thrust)：假设推力沿 $x_B$ 轴方向

$$
\mathbf{F}_{thrust}^B = \begin{bmatrix} T \ 0 \ 0 \end{bmatrix}
$$

#### c) 转动方程 (Rotational Dynamics)

欧拉方程 (Euler's Equation) 描述刚体的转动动力学：

$$
\mathbf{I} \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega}) = \mathbf{M}_{aero} + \mathbf{M}_{thrust}
$$

其中 $\mathbf{I}$ 为惯性张量 (Inertia Tensor)。假设飞行器关于 $x_B$-$z_B$ 平面对称，则：

$$
\mathbf{I} = \begin{bmatrix} I_{xx} & 0 & -I_{xz} \ 0 & I_{yy} & 0 \ -I_{xz} & 0 & I_{zz} \end{bmatrix}
$$

展开为分量形式：

$$
I_{xx}\dot{p} - I_{xz}\dot{r} + (I_{zz} - I_{yy})qr - I_{xz}pq = L_{aero}
$$

$$
I_{yy}\dot{q} + (I_{xx} - I_{zz})pr + I_{xz}(p^2 - r^2) = M_{aero}
$$

$$
I_{zz}\dot{r} - I_{xz}\dot{p} + (I_{yy} - I_{xx})pq + I_{xz}qr = N_{aero}
$$

其中 $L_{aero}$, $M_{aero}$, $N_{aero}$ 分别为滚转力矩 (Rolling Moment)、俯仰力矩 (Pitching Moment) 和偏航力矩 (Yawing Moment)。

#### d) 运动学方程 (Kinematics)

**位置运动学**：惯性系下的位置变化率

$$
\begin{bmatrix} \dot{x} \ \dot{y} \ \dot{z} \end{bmatrix} = R_{BI}^T \begin{bmatrix} u \ v \ w \end{bmatrix}
$$

其中 $R_{BI}$ 为从惯性系到机体系的旋转矩阵（由欧拉角 $\phi, \theta, \psi$ 构造）。

**姿态运动学**：欧拉角变化率与角速度的关系

$$
\begin{bmatrix} \dot{\phi} \ \dot{\theta} \ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 1 & \sin\phi\tan\theta & \cos\phi\tan\theta \ 0 & \cos\phi & -\sin\phi \ 0 & \sin\phi\sec\theta & \cos\phi\sec\theta \end{bmatrix} \begin{bmatrix} p \ q \ r \end{bmatrix}
$$

> **注意**：当 $\theta = \pm 90°$ 时存在万向锁 (Gimbal Lock) 奇异性。对于一般飞行状态，俯仰角远离 $\pm 90°$，此公式适用。


### 2.3 气动力模型 (Aerodynamic Model)

气动力和气动力矩是飞行器动力学的核心。所有气动力均可用无量纲系数表示。

#### a) 动压与参考量

**动压** (Dynamic Pressure)：

$$
\bar{q} = \frac{1}{2} \rho V^2
$$

其中 $\rho$ 为空气密度 (Air Density)，$V$ 为真空速 (True Airspeed)。

关键参考量：

| 符号 | 含义 | 典型单位 |
|------|------|----------|
| $S$ | 机翼参考面积 (Wing Reference Area) | $\text{m}^2$ |
| $b$ | 翼展 (Wingspan) | $\text{m}$ |
| $\bar{c}$ | 平均气动弦长 (Mean Aerodynamic Chord, MAC) | $\text{m}$ |

#### b) 气动力 (Aerodynamic Forces)

在风轴系 {W} 中，气动力分解为升力、阻力和侧力：

**升力** (Lift)：

$$
L = \bar{q} S \cdot C_L
$$

**阻力** (Drag)：

$$
D = \bar{q} S \cdot C_D
$$

**侧力** (Side Force)：

$$
Y = \bar{q} S \cdot C_Y
$$

#### c) 升力系数 (Lift Coefficient)

$$
C_L = C_{L_0} + C_{L_\alpha} \alpha + C_{L_q} \frac{q \bar{c}}{2V} + C_{L_{\delta_e}} \delta_e
$$

| 导数 | 含义 |
|------|------|
| $C_{L_0}$ | 零攻角升力系数 |
| $C_{L_\alpha}$ | 升力线斜率 (Lift Curve Slope) |
| $C_{L_q}$ | 俯仰角速率对升力的影响 |
| $C_{L_{\delta_e}}$ | 升降舵偏转对升力的影响 |

#### d) 阻力系数 (Drag Coefficient)

采用抛物线阻力极线 (Parabolic Drag Polar)：

$$
C_D = C_{D_0} + K C_L^2
$$

其中 $C_{D_0}$ 为零升阻力系数 (Parasite Drag Coefficient)，$K$ 为诱导阻力因子：

$$
K = \frac{1}{\pi e AR}
$$

$e$ 为 Oswald 效率因子 (Oswald Efficiency Factor)，$AR = b^2 / S$ 为展弦比 (Aspect Ratio)。

#### e) 气动力矩系数 (Aerodynamic Moment Coefficients)

**俯仰力矩** (Pitching Moment)：

$$
C_m = C_{m_0} + C_{m_\alpha} \alpha + C_{m_q} \frac{q \bar{c}}{2V} + C_{m_{\delta_e}} \delta_e
$$

**滚转力矩** (Rolling Moment)：

$$
C_l = C_{l_\beta} \beta + C_{l_p} \frac{p b}{2V} + C_{l_r} \frac{r b}{2V} + C_{l_{\delta_a}} \delta_a + C_{l_{\delta_r}} \delta_r
$$

**偏航力矩** (Yawing Moment)：

$$
C_n = C_{n_\beta} \beta + C_{n_p} \frac{p b}{2V} + C_{n_r} \frac{r b}{2V} + C_{n_{\delta_a}} \delta_a + C_{n_{\delta_r}} \delta_r
$$

**侧力系数** (Side Force Coefficient)：

$$
C_Y = C_{Y_\beta} \beta + C_{Y_{\delta_r}} \delta_r
$$

#### f) 控制输入

飞行器有四个控制输入：

| 符号 | 名称 | 作用 |
|------|------|------|
| $\delta_e$ | 升降舵 (Elevator) | 控制俯仰力矩 |
| $\delta_a$ | 副翼 (Aileron) | 控制滚转力矩 |
| $\delta_r$ | 方向舵 (Rudder) | 控制偏航力矩 |
| $\delta_t$ | 油门 (Throttle) | 控制推力大小 |

#### g) 气动力从风轴系到机体系的转换

$$
\begin{bmatrix} F_x^{aero} \ F_y^{aero} \ F_z^{aero} \end{bmatrix} = \begin{bmatrix} \cos\alpha \cos\beta & -\cos\alpha \sin\beta & -\sin\alpha \ \sin\beta & \cos\beta & 0 \ \sin\alpha \cos\beta & -\sin\alpha \sin\beta & \cos\alpha \end{bmatrix} \begin{bmatrix} -D \ Y \ -L \end{bmatrix}
$$

对于小侧滑角 ($\beta \approx 0$) 的简化形式：

$$
F_x^{aero} \approx -D\cos\alpha + L\sin\alpha
$$

$$
F_z^{aero} \approx -D\sin\alpha - L\cos\alpha
$$


---

## 3. 配平 (Trim)

### 3.1 配平的概念

**配平** (Trim) 是指飞行器处于稳态飞行 (Steady-State Flight) 的条件，即所有加速度为零：

$$
\dot{u} = \dot{v} = \dot{w} = 0, \quad \dot{p} = \dot{q} = \dot{r} = 0
$$

在配平状态下，作用在飞行器上的所有力和力矩达到平衡。

### 3.2 直线水平飞行配平 (Straight and Level Flight Trim)

对于最简单的直线水平飞行 ($\phi = 0$, $\beta = 0$, $p = q = r = 0$)，配平条件简化为：

**力平衡**：

$$
T \cos\alpha - D - mg\sin\theta = 0 \quad \text{(沿 } x_B \text{)}
$$

$$
L + T\sin\alpha - mg\cos\theta = 0 \quad \text{(沿 } z_B \text{)}
$$

**力矩平衡**：

$$
M_{aero} = \bar{q} S \bar{c} \cdot C_m(\alpha, \delta_e) = 0
$$

### 3.3 配平求解

需要求解的未知量为：

- 攻角 $\alpha_{trim}$
- 升降舵偏转 $\delta_{e,trim}$
- 推力 $T_{trim}$（或油门 $\delta_{t,trim}$）

这是一个非线性方程组，可以用以下方法求解。

#### 方法一：MATLAB `fsolve`

```matlab
function residual = trim_equations(x, params)
    alpha = x(1);
    delta_e = x(2);
    delta_t = x(3);
    
    V = params.V_trim;
    m = params.m;
    g = params.g;
    rho = params.rho;
    S = params.S;
    c_bar = params.c_bar;
    
    % 动压
    q_bar = 0.5 * rho * V^2;
    
    % 气动系数
    C_L = params.C_L0 + params.C_La * alpha + params.C_Lde * delta_e;
    C_D = params.C_D0 + params.K * C_L^2;
    C_m = params.C_m0 + params.C_ma * alpha + params.C_mde * delta_e;
    
    % 气动力
    L = q_bar * S * C_L;
    D = q_bar * S * C_D;
    T = delta_t * params.T_max;
    
    % 配平方程 (残差)
    theta = alpha;  % 水平飞行时 gamma = 0, 故 theta = alpha
    residual(1) = T * cos(alpha) - D - m * g * sin(theta);
    residual(2) = L + T * sin(alpha) - m * g * cos(theta);
    residual(3) = C_m;
end

% 调用 fsolve
x0 = [0.05; -0.01; 0.5];  % 初始猜测
options = optimoptions('fsolve', 'Display', 'iter');
x_trim = fsolve(@(x) trim_equations(x, params), x0, options);

alpha_trim = x_trim(1);
delta_e_trim = x_trim(2);
delta_t_trim = x_trim(3);
```

#### 方法二：手动迭代法

对于水平飞行，可以按以下步骤手动迭代：

1. **初始猜测** $\alpha_0$
2. 由升力平衡求 $C_L$：$C_L = \frac{mg}{\bar{q}S\cos\alpha}$
3. 由 $C_L$ 反算 $\alpha$：$\alpha = \frac{C_L - C_{L_0}}{C_{L_\alpha}}$
4. 由力矩平衡求 $\delta_e$：$\delta_e = -\frac{C_{m_0} + C_{m_\alpha}\alpha}{C_{m_{\delta_e}}}$
5. 由阻力和 $x$ 方向力平衡求推力 $T$
6. 检查收敛，若未收敛则返回步骤 2

### 3.4 配平状态验证

配平求解完成后，应验证以下条件：

- $|\alpha_{trim}|$ 在合理范围内（通常 $0° \sim 15°$）
- $|\delta_{e,trim}|$ 未超过舵面限幅（通常 $\pm 25°$）
- $0 \leq \delta_{t,trim} \leq 1$（油门在有效范围内）
- 将配平状态代入完整动力学方程，确认残差接近零


---

## 4. 控制设计 (Control Design)

固定翼飞行器的控制通常分为 **纵向控制** (Longitudinal Control) 和 **横向控制** (Lateral Control) 两个独立通道。这种解耦在小扰动假设下是合理的。

### 4.1 控制架构总览

```
                    ┌─────────────────────────────────────────────┐
                    │            Guidance (制导层)                  │
                    │   航点跟踪 / 航迹规划                         │
                    └──────────┬──────────────────┬───────────────┘
                               │                  │
                    ┌──────────▼──────┐  ┌────────▼────────┐
                    │  纵向控制通道    │  │  横向控制通道    │
                    │  (Longitudinal) │  │  (Lateral)      │
                    │                 │  │                 │
                    │  h_des → θ_des  │  │  ψ_des → φ_des │
                    │  θ_des → δ_e   │  │  φ_des → δ_a   │
                    │  V_des → δ_t   │  │  r → δ_r       │
                    └────────┬────────┘  └────────┬────────┘
                             │                    │
                    ┌────────▼────────────────────▼────────┐
                    │         飞行器动力学 (Plant)           │
                    └──────────────────────────────────────┘
```

### 4.2 纵向控制 (Longitudinal Control)

纵向通道控制飞行器的俯仰姿态、高度和速度，使用升降舵 $\delta_e$ 和油门 $\delta_t$ 作为执行器。

#### a) 俯仰角控制 (Pitch Attitude Control) — 内环

俯仰角控制是纵向控制的内环，采用 PD 控制器：

$$
\delta_e = \delta_{e,trim} + K_\theta (\theta_{des} - \theta) + K_q \cdot q
$$

| 参数 | 含义 | 说明 |
|------|------|------|
| $K_\theta$ | 俯仰角比例增益 | 负值（拉杆抬头为负 $\delta_e$） |
| $K_q$ | 俯仰角速率阻尼增益 | 提供阻尼，抑制振荡 |
| $\delta_{e,trim}$ | 配平升降舵偏转 | 前馈项 |

**设计要点**：

- $K_q$ 增大可提高阻尼比 (Damping Ratio)，但过大会导致响应迟缓
- $K_\theta$ 决定自然频率 (Natural Frequency)，增大可加快响应
- 典型闭环特性：$\omega_n \approx 2\text{-}5 \text{ rad/s}$，$\zeta \approx 0.7\text{-}1.0$

**MATLAB 实现**：

```matlab
function delta_e = pitch_controller(theta_des, theta, q, params)
    K_theta = params.K_theta;   % 例如 -2.0
    K_q     = params.K_q;       % 例如 -0.5
    
    delta_e = params.delta_e_trim ...
            + K_theta * (theta_des - theta) ...
            + K_q * q;
    
    % 舵面限幅 (Saturation)
    delta_e = max(min(delta_e, params.delta_e_max), params.delta_e_min);
end
```

#### b) 高度控制 (Altitude Control) — 外环

高度控制是纵向控制的外环，输出期望俯仰角 $\theta_{des}$：

$$
\theta_{des} = \theta_{trim} + K_h (h_{des} - h) + K_{\dot{h}} \cdot \dot{h}
$$

其中 $h = -z$（NED 坐标系中高度为 $z$ 的负值），$\dot{h} = -\dot{z}$。

| 参数 | 含义 |
|------|------|
| $K_h$ | 高度误差比例增益 |
| $K_{\dot{h}}$ | 爬升率阻尼增益 |

**设计要点**：

- 外环带宽必须低于内环带宽（通常为内环的 $1/3 \sim 1/5$）
- 应对 $\theta_{des}$ 施加限幅，防止过大的俯仰指令：

$$
\theta_{des} \in [\theta_{min}, \theta_{max}], \quad \text{例如 } [-15°, +20°]
$$

**MATLAB 实现**：

```matlab
function theta_des = altitude_controller(h_des, h, h_dot, params)
    K_h    = params.K_h;       % 例如 0.05
    K_hdot = params.K_hdot;    % 例如 -0.1
    
    theta_des = params.theta_trim ...
              + K_h * (h_des - h) ...
              + K_hdot * h_dot;
    
    % 俯仰角指令限幅
    theta_des = max(min(theta_des, deg2rad(20)), deg2rad(-15));
end
```

#### c) 速度控制 (Speed Control)

速度控制通过油门 $\delta_t$ 实现，采用 PI 控制器：

$$
\delta_t = \delta_{t,trim} + K_V (V_{des} - V) + K_{V_i} \int (V_{des} - V) \, dt
$$

| 参数 | 含义 |
|------|------|
| $K_V$ | 速度误差比例增益 |
| $K_{V_i}$ | 速度误差积分增益 |

**MATLAB 实现**：

```matlab
function [delta_t, V_error_int] = speed_controller(V_des, V, V_error_int, dt, params)
    K_V  = params.K_V;     % 例如 0.1
    K_Vi = params.K_Vi;    % 例如 0.01
    
    V_error = V_des - V;
    V_error_int = V_error_int + V_error * dt;
    
    % 积分抗饱和 (Anti-Windup)
    V_error_int = max(min(V_error_int, params.int_max), params.int_min);
    
    delta_t = params.delta_t_trim + K_V * V_error + K_Vi * V_error_int;
    
    % 油门限幅
    delta_t = max(min(delta_t, 1.0), 0.0);
end
```

> **提示**：积分项需要抗饱和 (Anti-Windup) 处理，防止积分器在执行器饱和时持续累积误差。


### 4.3 横向控制 (Lateral Control)

横向通道控制飞行器的滚转姿态和航向，使用副翼 $\delta_a$ 和方向舵 $\delta_r$ 作为执行器。

#### a) 滚转角控制 (Roll Attitude Control) — 内环

滚转角控制采用 PD 控制器，通过副翼实现：

$$
\delta_a = K_\phi (\phi_{des} - \phi) + K_p \cdot p
$$

| 参数 | 含义 | 说明 |
|------|------|------|
| $K_\phi$ | 滚转角比例增益 | 正值（正副翼偏转产生正滚转力矩） |
| $K_p$ | 滚转角速率阻尼增益 | 负值，提供滚转阻尼 |

**设计要点**：

- 滚转通道的动态响应通常比俯仰通道快
- 典型闭环特性：$\omega_n \approx 5\text{-}10 \text{ rad/s}$，$\zeta \approx 0.7$
- 滚转角指令应限幅，防止过大的倾斜角：

$$
\phi_{des} \in [-\phi_{max}, +\phi_{max}], \quad \text{例如 } \pm 45°
$$

**MATLAB 实现**：

```matlab
function delta_a = roll_controller(phi_des, phi, p, params)
    K_phi = params.K_phi;   % 例如 1.5
    K_p   = params.K_p;     % 例如 -0.3
    
    delta_a = K_phi * (phi_des - phi) + K_p * p;
    
    % 副翼限幅
    delta_a = max(min(delta_a, params.delta_a_max), params.delta_a_min);
end
```

#### b) 航向控制 (Heading Control) — 外环

航向控制是横向控制的外环，通过协调转弯 (Coordinated Turn) 实现。期望滚转角由航向误差生成：

$$
\phi_{des} = K_\psi (\psi_{des} - \psi)
$$

| 参数 | 含义 |
|------|------|
| $K_\psi$ | 航向误差比例增益 |

**航向角误差处理**：

航向角 $\psi$ 存在 $0°/360°$ 的不连续性，需要对误差进行归一化：

```matlab
function psi_error = wrap_heading_error(psi_des, psi)
    psi_error = psi_des - psi;
    % 将误差归一化到 [-pi, pi]
    psi_error = mod(psi_error + pi, 2*pi) - pi;
end
```

**协调转弯关系**：

在稳态协调转弯中，滚转角与转弯半径的关系为：

$$
\tan\phi = \frac{V^2}{gR} = \frac{V \dot{\psi}}{g}
$$

其中 $R$ 为转弯半径，$\dot{\psi}$ 为航向变化率。

**MATLAB 实现**：

```matlab
function phi_des = heading_controller(psi_des, psi, params)
    K_psi = params.K_psi;   % 例如 2.0
    
    psi_error = wrap_heading_error(psi_des, psi);
    phi_des = K_psi * psi_error;
    
    % 滚转角指令限幅
    phi_des = max(min(phi_des, params.phi_max), -params.phi_max);
end
```

#### c) 偏航阻尼器 (Yaw Damper)

方向舵用于抑制荷兰滚模态 (Dutch Roll Mode) 的振荡：

$$
\delta_r = K_r \cdot r
$$

| 参数 | 含义 |
|------|------|
| $K_r$ | 偏航角速率反馈增益（负值） |

荷兰滚是固定翼飞行器横向运动中的一种耦合振荡模态，表现为滚转和偏航的交替振荡。偏航阻尼器通过反馈偏航角速率 $r$ 来增加该模态的阻尼。

**MATLAB 实现**：

```matlab
function delta_r = yaw_damper(r, params)
    K_r = params.K_r;   % 例如 -1.0
    
    delta_r = K_r * r;
    
    % 方向舵限幅
    delta_r = max(min(delta_r, params.delta_r_max), params.delta_r_min);
end
```

### 4.4 增益调参指南 (Gain Tuning Guidelines)

#### 调参顺序

按照从内环到外环的顺序调参：

1. **偏航阻尼器** $K_r$ → 抑制荷兰滚振荡
2. **滚转内环** $K_p, K_\phi$ → 快速稳定的滚转响应
3. **俯仰内环** $K_q, K_\theta$ → 快速稳定的俯仰响应
4. **速度控制** $K_V, K_{V_i}$ → 稳定的速度保持
5. **高度外环** $K_h, K_{\dot{h}}$ → 平稳的高度跟踪
6. **航向外环** $K_\psi$ → 平稳的航向跟踪

#### 带宽分离原则 (Bandwidth Separation)

各控制环路的带宽应满足：

$$
\omega_{heading} < \omega_{altitude} < \omega_{roll} \approx \omega_{pitch} < \omega_{actuator}
$$

典型带宽比例为 $1 : 2 : 5 : 5 : 20$。

#### 常见问题与解决方案

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| 俯仰振荡 | $K_q$ 过小或 $K_\theta$ 过大 | 增大 $K_q$，减小 $K_\theta$ |
| 高度超调 | 外环带宽过高 | 减小 $K_h$ |
| 速度漂移 | 缺少积分项 | 增加 $K_{V_i}$ |
| 航向振荡 | $K_\psi$ 过大 | 减小 $K_\psi$，检查滚转内环 |
| 荷兰滚 | $K_r$ 不足 | 增大 $|K_r|$ |


---

## 5. Simulink 仿真 (Simulation)

### 5.1 模型结构说明

`fw_sim.slx` 的顶层结构如下：

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Reference   │────▶│  Controller  │────▶│    Plant      │
│  Generator   │     │              │     │  (6-DOF EOM)  │
│  (指令生成)   │     │  (控制器)     │     │  (动力学模型)  │
└──────────────┘     └──────┬───────┘     └──────┬────────┘
                            │                     │
                            │    ┌────────────┐   │
                            │◀───│  Sensors    │◀──│
                            │    │  (传感器)    │   │
                            │    └────────────┘   │
                            │                     │
                     ┌──────▼─────────────────────▼────────┐
                     │          Visualization               │
                     │          (可视化与数据记录)             │
                     └─────────────────────────────────────┘
```

### 5.2 Plant 子系统详解

Plant 子系统实现完整的六自由度动力学，内部包含：

```
输入: [delta_e, delta_a, delta_r, delta_t]
  │
  ▼
┌─────────────────┐
│  Aerodynamics   │──▶ 气动力 F_aero, 气动力矩 M_aero
│  (气动力计算)    │
└─────────────────┘
  │
  ▼
┌─────────────────┐
│  Force Summation│──▶ 总力 F_total, 总力矩 M_total
│  (力/力矩合成)   │
└─────────────────┘
  │
  ▼
┌─────────────────┐
│  EOM Integration│──▶ 状态向量 x = [pos, vel, euler, omega]
│  (运动方程积分)   │
└─────────────────┘
  │
  ▼
输出: 完整状态向量 (12维)
```

### 5.3 Controller 子系统详解

Controller 子系统实现第 4 节中设计的控制律：

```matlab
%% Controller 子系统的 MATLAB Function Block 示例
function [delta_e, delta_a, delta_r, delta_t] = controller( ...
    h_des, V_des, psi_des, ...
    x, y, z, u, v, w, phi, theta, psi, p, q, r, ...
    params)

    % 计算空速
    V = sqrt(u^2 + v^2 + w^2);
    h = -z;  % NED 坐标系
    h_dot = -(-u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta));
    
    % === 纵向通道 ===
    % 高度控制 (外环)
    theta_des = params.theta_trim ...
              + params.K_h * (h_des - h) ...
              + params.K_hdot * h_dot;
    theta_des = max(min(theta_des, deg2rad(20)), deg2rad(-15));
    
    % 俯仰角控制 (内环)
    delta_e = params.delta_e_trim ...
            + params.K_theta * (theta_des - theta) ...
            + params.K_q * q;
    delta_e = max(min(delta_e, deg2rad(25)), deg2rad(-25));
    
    % 速度控制
    delta_t = params.delta_t_trim ...
            + params.K_V * (V_des - V);
    delta_t = max(min(delta_t, 1.0), 0.0);
    
    % === 横向通道 ===
    % 航向控制 (外环)
    psi_error = mod(psi_des - psi + pi, 2*pi) - pi;
    phi_des = params.K_psi * psi_error;
    phi_des = max(min(phi_des, deg2rad(45)), deg2rad(-45));
    
    % 滚转角控制 (内环)
    delta_a = params.K_phi * (phi_des - phi) + params.K_p * p;
    delta_a = max(min(delta_a, deg2rad(25)), deg2rad(-25));
    
    % 偏航阻尼器
    delta_r = params.K_r * r;
    delta_r = max(min(delta_r, deg2rad(25)), deg2rad(-25));
end
```

### 5.4 参数设置

在运行仿真前，需要在 MATLAB Workspace 中设置飞行器参数：

```matlab
%% 飞行器物理参数
params.m     = 13.5;       % 质量 [kg]
params.g     = 9.81;       % 重力加速度 [m/s^2]
params.S     = 0.55;       % 机翼面积 [m^2]
params.b     = 2.90;       % 翼展 [m]
params.c_bar = 0.19;       % 平均气动弦长 [m]
params.rho   = 1.225;      % 海平面空气密度 [kg/m^3]

%% 惯性参数
params.Ixx = 0.8244;       % [kg·m^2]
params.Iyy = 1.135;
params.Izz = 1.759;
params.Ixz = 0.1204;

%% 气动导数 (纵向)
params.C_L0   =  0.28;
params.C_La   =  3.45;     % [1/rad]
params.C_Lq   =  0.0;
params.C_Lde  =  0.36;     % [1/rad]
params.C_D0   =  0.03;
params.K      =  0.0430;
params.C_m0   = -0.02;
params.C_ma   = -0.38;     % [1/rad] (静稳定性要求为负)
params.C_mq   = -3.6;
params.C_mde  = -0.50;     % [1/rad]

%% 气动导数 (横向)
params.C_lb   = -0.12;     % [1/rad]
params.C_lp   = -0.26;
params.C_lr   =  0.14;
params.C_lda  =  0.08;     % [1/rad]
params.C_ldr  =  0.105;    % [1/rad]
params.C_nb   =  0.25;     % [1/rad] (风标稳定性要求为正)
params.C_np   =  0.022;
params.C_nr   = -0.35;
params.C_nda  =  0.06;
params.C_ndr  = -0.032;    % [1/rad]
params.C_Yb   = -0.98;     % [1/rad]
params.C_Ydr  =  0.17;     % [1/rad]

%% 推力参数
params.T_max = 100;         % 最大推力 [N]
```


### 5.5 仿真场景 (Simulation Scenarios)

#### 场景一：直线水平飞行 (Straight and Level Flight)

验证配平状态和控制器的基本稳定性。

```matlab
%% 仿真参数
sim_params.t_end   = 60;       % 仿真时长 [s]
sim_params.dt      = 0.01;     % 时间步长 [s]

%% 初始条件 (配平状态)
IC.V     = 18;                 % 空速 [m/s]
IC.h     = 100;                % 高度 [m]
IC.alpha = alpha_trim;
IC.theta = alpha_trim;         % 水平飞行 gamma = 0
IC.phi   = 0;
IC.psi   = 0;                 % 初始航向：北

%% 指令
cmd.h_des   = 100;             % 保持高度 [m]
cmd.V_des   = 18;              % 保持速度 [m/s]
cmd.psi_des = 0;               % 保持航向 [rad]

%% 运行仿真
sim('fw_sim');
```

**预期结果**：
- 所有状态量保持在配平值附近
- 高度、速度、航向无明显漂移
- 控制输入保持在配平值附近

#### 场景二：高度阶跃响应 (Altitude Step Response)

测试高度控制器的跟踪性能。

```matlab
%% 在 t = 10s 时给出高度阶跃指令
cmd.h_des_initial = 100;       % 初始高度 [m]
cmd.h_des_final   = 120;       % 目标高度 [m]
cmd.h_step_time   = 10;        % 阶跃时刻 [s]
```

**观察指标**：
- 上升时间 (Rise Time)：到达目标高度 90% 所需时间
- 超调量 (Overshoot)：高度超过目标值的百分比
- 调节时间 (Settling Time)：高度进入 $\pm 2\%$ 误差带的时间
- 俯仰角变化是否平滑，升降舵是否饱和

#### 场景三：协调转弯 (Coordinated Turn)

测试横向控制器的航向跟踪能力。

```matlab
%% 在 t = 10s 时给出航向阶跃指令 (转弯 90°)
cmd.psi_des_initial = 0;               % 初始航向 [rad]
cmd.psi_des_final   = deg2rad(90);     % 目标航向 [rad]
cmd.psi_step_time   = 10;              % 阶跃时刻 [s]
```

**观察指标**：
- 转弯过程中高度是否保持（纵横耦合效应）
- 滚转角是否在限幅范围内
- 侧滑角 $\beta$ 是否接近零（协调转弯条件）
- 速度是否保持稳定

#### 场景四：综合任务 — 爬升转弯 (Climbing Turn)

同时改变高度和航向，测试多通道协调能力。

```matlab
%% 综合指令
cmd.h_des   = 150;             % 爬升至 150m
cmd.psi_des = deg2rad(180);    % 转向南
cmd.V_des   = 20;              % 加速至 20 m/s
```

**观察指标**：
- 三个通道是否能同时跟踪各自的指令
- 是否存在严重的通道间耦合
- 执行器是否出现饱和

### 5.6 仿真结果分析与绘图

```matlab
%% 绘制仿真结果
figure('Name', 'Longitudinal Response');

subplot(4,1,1);
plot(t, h, 'b', t, h_des*ones(size(t)), 'r--');
xlabel('Time [s]'); ylabel('Altitude [m]');
legend('h', 'h_{des}'); grid on;
title('高度响应 (Altitude Response)');

subplot(4,1,2);
plot(t, rad2deg(theta), 'b');
xlabel('Time [s]'); ylabel('\theta [deg]');
title('俯仰角 (Pitch Angle)'); grid on;

subplot(4,1,3);
plot(t, V, 'b', t, V_des*ones(size(t)), 'r--');
xlabel('Time [s]'); ylabel('Airspeed [m/s]');
legend('V', 'V_{des}'); grid on;
title('空速 (Airspeed)');

subplot(4,1,4);
plot(t, rad2deg(delta_e), 'b', t, delta_t*100, 'r');
xlabel('Time [s]'); ylabel('Control Input');
legend('\delta_e [deg]', '\delta_t [%]'); grid on;
title('纵向控制输入 (Longitudinal Control Inputs)');

figure('Name', 'Lateral Response');

subplot(4,1,1);
plot(t, rad2deg(psi), 'b', t, rad2deg(psi_des)*ones(size(t)), 'r--');
xlabel('Time [s]'); ylabel('\psi [deg]');
legend('\psi', '\psi_{des}'); grid on;
title('航向角 (Heading)');

subplot(4,1,2);
plot(t, rad2deg(phi), 'b');
xlabel('Time [s]'); ylabel('\phi [deg]');
title('滚转角 (Roll Angle)'); grid on;

subplot(4,1,3);
plot(t, rad2deg(beta), 'b');
xlabel('Time [s]'); ylabel('\beta [deg]');
title('侧滑角 (Sideslip Angle)'); grid on;

subplot(4,1,4);
plot(t, rad2deg(delta_a), 'b', t, rad2deg(delta_r), 'r');
xlabel('Time [s]'); ylabel('Control Input [deg]');
legend('\delta_a', '\delta_r'); grid on;
title('横向控制输入 (Lateral Control Inputs)');

%% 三维飞行轨迹
figure('Name', '3D Trajectory');
plot3(y, x, h, 'b', 'LineWidth', 1.5);
xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude [m]');
title('三维飞行轨迹 (3D Flight Trajectory)');
grid on; axis equal;
```


---

## 6. 关键公式总结 (Key Formulas Summary)

### 6.1 坐标系与气流角

| 公式 | 说明 |
|------|------|
| $\alpha = \arctan\left(\dfrac{w}{u}\right)$ | 攻角 (Angle of Attack) |
| $\beta = \arcsin\left(\dfrac{v}{V}\right)$ | 侧滑角 (Sideslip Angle) |
| $V = \sqrt{u^2 + v^2 + w^2}$ | 真空速 (True Airspeed) |

### 6.2 气动力与气动力矩

| 公式 | 说明 |
|------|------|
| $\bar{q} = \frac{1}{2}\rho V^2$ | 动压 (Dynamic Pressure) |
| $L = \bar{q} S \cdot C_L$ | 升力 (Lift) |
| $D = \bar{q} S \cdot C_D$ | 阻力 (Drag) |
| $C_L = C_{L_0} + C_{L_\alpha}\alpha + C_{L_{\delta_e}}\delta_e$ | 升力系数 |
| $C_D = C_{D_0} + K C_L^2$ | 阻力极线 (Drag Polar) |
| $C_m = C_{m_0} + C_{m_\alpha}\alpha + C_{m_q}\dfrac{q\bar{c}}{2V} + C_{m_{\delta_e}}\delta_e$ | 俯仰力矩系数 |
| $C_l = C_{l_\beta}\beta + C_{l_p}\dfrac{pb}{2V} + C_{l_{\delta_a}}\delta_a + C_{l_{\delta_r}}\delta_r$ | 滚转力矩系数 |
| $C_n = C_{n_\beta}\beta + C_{n_r}\dfrac{rb}{2V} + C_{n_{\delta_a}}\delta_a + C_{n_{\delta_r}}\delta_r$ | 偏航力矩系数 |

### 6.3 运动方程

**平动方程** (机体系)：

$$
m(\dot{u} + qw - rv) = F_x, \quad m(\dot{v} + ru - pw) = F_y, \quad m(\dot{w} + pv - qu) = F_z
$$

**转动方程** (机体系)：

$$
\mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega}) = \mathbf{M}
$$

**姿态运动学**：

$$
\begin{bmatrix} \dot{\phi} \ \dot{\theta} \ \dot{\psi} \end{bmatrix} = \begin{bmatrix} 1 & \sin\phi\tan\theta & \cos\phi\tan\theta \ 0 & \cos\phi & -\sin\phi \ 0 & \sin\phi\sec\theta & \cos\phi\sec\theta \end{bmatrix} \begin{bmatrix} p \ q \ r \end{bmatrix}
$$

### 6.4 配平条件 (水平直线飞行)

$$
T\cos\alpha = D + mg\sin\alpha, \quad L + T\sin\alpha = mg\cos\alpha, \quad C_m(\alpha, \delta_e) = 0
$$

### 6.5 控制律

**纵向通道**：

| 控制环 | 控制律 |
|--------|--------|
| 俯仰角控制 | $\delta_e = \delta_{e,trim} + K_\theta(\theta_{des} - \theta) + K_q \cdot q$ |
| 高度控制 | $\theta_{des} = \theta_{trim} + K_h(h_{des} - h) + K_{\dot{h}} \cdot \dot{h}$ |
| 速度控制 | $\delta_t = \delta_{t,trim} + K_V(V_{des} - V) + K_{V_i}\displaystyle\int(V_{des} - V)\,dt$ |

**横向通道**：

| 控制环 | 控制律 |
|--------|--------|
| 滚转角控制 | $\delta_a = K_\phi(\phi_{des} - \phi) + K_p \cdot p$ |
| 航向控制 | $\phi_{des} = K_\psi(\psi_{des} - \psi)$ |
| 偏航阻尼 | $\delta_r = K_r \cdot r$ |

### 6.6 协调转弯

$$
\tan\phi = \frac{V \dot{\psi}}{g}, \quad n = \frac{1}{\cos\phi}
$$

其中 $n$ 为载荷因子 (Load Factor)。

---

## 附录 A：符号表 (Symbol Table)

| 符号 | 英文名称 | 中文名称 | 单位 |
|------|----------|----------|------|
| $m$ | Mass | 质量 | kg |
| $g$ | Gravitational acceleration | 重力加速度 | m/s$^2$ |
| $\rho$ | Air density | 空气密度 | kg/m$^3$ |
| $V$ | True airspeed | 真空速 | m/s |
| $S$ | Wing reference area | 机翼参考面积 | m$^2$ |
| $b$ | Wingspan | 翼展 | m |
| $\bar{c}$ | Mean aerodynamic chord | 平均气动弦长 | m |
| $\alpha$ | Angle of attack | 攻角 | rad |
| $\beta$ | Sideslip angle | 侧滑角 | rad |
| $\phi$ | Roll angle | 滚转角 | rad |
| $\theta$ | Pitch angle | 俯仰角 | rad |
| $\psi$ | Yaw angle / Heading | 偏航角 / 航向角 | rad |
| $p, q, r$ | Angular velocities | 角速度分量 | rad/s |
| $u, v, w$ | Body-frame velocities | 机体系速度分量 | m/s |
| $\delta_e$ | Elevator deflection | 升降舵偏转 | rad |
| $\delta_a$ | Aileron deflection | 副翼偏转 | rad |
| $\delta_r$ | Rudder deflection | 方向舵偏转 | rad |
| $\delta_t$ | Throttle setting | 油门设定 | [0, 1] |
| $L$ | Lift | 升力 | N |
| $D$ | Drag | 阻力 | N |
| $T$ | Thrust | 推力 | N |
| $C_L$ | Lift coefficient | 升力系数 | — |
| $C_D$ | Drag coefficient | 阻力系数 | — |
| $C_m$ | Pitching moment coefficient | 俯仰力矩系数 | — |
| $C_l$ | Rolling moment coefficient | 滚转力矩系数 | — |
| $C_n$ | Yawing moment coefficient | 偏航力矩系数 | — |

---

## 附录 B：常见调试问题 (Troubleshooting)

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 仿真发散 (Divergence) | 增益符号错误或过大 | 检查增益符号，减小增益值 |
| 飞行器翻转 | 配平状态不正确 | 重新计算配平，检查初始条件 |
| 高度持续下降 | 推力不足或升力系数错误 | 检查配平推力，验证气动参数 |
| Simulink 代数环警告 | 控制器中存在直接馈通 | 在反馈路径中加入单位延迟 (Unit Delay) |
| 仿真速度极慢 | 步长过小或模型刚性 | 使用变步长求解器 (ode45)，检查是否有数值奇异 |
| 舵面持续饱和 | 指令变化过大或增益过高 | 对指令加入速率限制 (Rate Limiter)，减小增益 |

---

## 附录 C：参考文献 (References)

1. Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). *Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems*. Wiley.
2. Beard, R. W., & McLain, T. W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press.
3. Etkin, B., & Reid, L. D. (1996). *Dynamics of Flight: Stability and Control*. Wiley.
4. ETH Zurich, Robot Dynamics Course Materials.

---

> **实验提示**：完成本实验后，你应该能够理解固定翼飞行器从建模到控制的完整流程，并具备在 Simulink 中搭建和调试飞行控制系统的能力。建议在调参过程中多观察 Scope 输出，逐步理解每个增益对系统响应的影响。
