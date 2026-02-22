# Lecture 10: 固定翼飞行器 — 动力学建模与控制

> **ETH Zurich — Robot Dynamics**
> Fixed-Wing Aircraft: Dynamic Modeling and Control

---

## 目录

1. [固定翼飞行器概述](#1-固定翼飞行器概述)
2. [空气动力学基础](#2-空气动力学基础)
3. [坐标系](#3-坐标系)
4. [固定翼运动方程](#4-固定翼运动方程)
5. [气动力和力矩模型](#5-气动力和力矩模型)
6. [配平和线性化](#6-配平和线性化)
7. [飞行控制](#7-飞行控制)
8. [案例研究](#8-案例研究)

---

## 1. 固定翼飞行器概述

### 1.1 什么是固定翼飞行器

固定翼飞行器 (Fixed-Wing Aircraft) 是指机翼相对于机身固定不动的飞行器。与旋翼飞行器 (Rotary-Wing Aircraft) 不同，固定翼飞行器通过机翼与气流的相对运动产生升力，而非依赖旋转的桨叶。

### 1.2 与旋翼飞行器的区别

| 特性 | 固定翼 (Fixed-Wing) | 旋翼 (Rotary-Wing) |
|------|---------------------|---------------------|
| 升力来源 | 机翼前进产生升力 | 旋转桨叶产生升力 |
| 悬停能力 | 不能悬停（需持续前飞） | 可以悬停 |
| 起降方式 | 需要跑道或弹射 | 垂直起降 (VTOL) |
| 续航时间 | 长（数小时至数天） | 短（通常 20–40 分钟） |
| 巡航速度 | 快 | 慢 |
| 能效 | 高 | 低 |
| 机械复杂度 | 低（无旋转部件） | 高 |

**核心区别**: 固定翼飞行器必须保持一定的最低前飞速度 (Minimum Airspeed) 才能维持飞行，即失速速度 $V_{\text{stall}}$。

### 1.3 固定翼飞行器的优势

- **续航长**: 由于升阻比 (Lift-to-Drag Ratio, $L/D$) 高，固定翼飞行器的能效远优于旋翼飞行器。典型的固定翼无人机 $L/D \approx 10$–$20$，而多旋翼仅约 $3$–$5$。
- **速度快**: 巡航速度通常为 $15$–$30 \, \text{m/s}$（小型无人机），可快速覆盖大面积区域。
- **能效高**: 在巡航状态下，固定翼飞行器只需克服阻力，而旋翼飞行器需持续做功以维持悬停。

### 1.4 应用领域

- **测绘 (Mapping)**: 大面积地形测绘、正射影像生成
- **监测 (Surveillance)**: 边境巡逻、环境监测、农业遥感
- **太阳能飞机 (Solar-Powered Aircraft)**: 利用太阳能实现超长航时飞行，如 ETH 的 AtlantikSolar 项目
- **货物运输 (Cargo Delivery)**: 偏远地区物资投送
- **气象观测**: 大气数据采集

---

## 2. 空气动力学基础

### 2.1 升力 (Lift)

升力是垂直于来流方向的气动力分量，由机翼上下表面的压差产生。根据伯努利原理和环量理论：

$$
L = \frac{1}{2} \rho V^2 S C_L
$$

其中：
- $\rho$ — 空气密度 (Air Density)，海平面标准值 $\rho_0 = 1.225 \, \text{kg/m}^3$
- $V$ — 空速 (Airspeed)，即飞行器相对于空气的速度
- $S$ — 机翼参考面积 (Wing Reference Area)
- $C_L$ — 升力系数 (Lift Coefficient)，无量纲

**升力系数与攻角的关系**（线性区域）：

$$
C_L = C_{L_0} + C_{L_\alpha} \cdot \alpha
$$

其中 $C_{L_0}$ 是零攻角升力系数，$C_{L_\alpha}$ 是升力线斜率 (Lift Curve Slope)，$\alpha$ 是攻角。

> **注意**: 当攻角超过临界攻角 $\alpha_{\text{stall}}$（通常 $12°$–$18°$）时，气流分离导致升力骤降，即**失速 (Stall)**。

### 2.2 阻力 (Drag)

阻力是沿来流方向的气动力分量，阻碍飞行器前进：

$$
D = \frac{1}{2} \rho V^2 S C_D
$$

阻力系数 $C_D$ 通常用**阻力极线 (Drag Polar)** 表示：

$$
C_D = C_{D_0} + K C_L^2
$$

其中：
- $C_{D_0}$ — 零升阻力系数 (Parasite Drag Coefficient)，包括摩擦阻力和压差阻力
- $K = \frac{1}{\pi e AR}$ — 诱导阻力因子 (Induced Drag Factor)
- $e$ — Oswald 效率因子，通常 $0.7$–$0.9$
- $AR = \frac{b^2}{S}$ — 展弦比 (Aspect Ratio)，$b$ 为翼展 (Wingspan)

**升阻比**：

$$
\frac{L}{D} = \frac{C_L}{C_D} = \frac{C_L}{C_{D_0} + K C_L^2}
$$

最大升阻比对应的升力系数：

$$
C_{L}^* = \sqrt{\frac{C_{D_0}}{K}}, \quad \left(\frac{L}{D}\right)_{\max} = \frac{1}{2\sqrt{K C_{D_0}}}
$$

### 2.3 侧力 (Side Force)

当飞行器存在侧滑时，会产生侧向气动力：

$$
Y = \frac{1}{2} \rho V^2 S C_Y
$$

侧力系数主要与侧滑角 $\beta$ 相关：

$$
C_Y = C_{Y_\beta} \cdot \beta + C_{Y_{\delta_r}} \cdot \delta_r
$$

其中 $\delta_r$ 为方向舵偏转角。

### 2.4 攻角 (Angle of Attack, $\alpha$)

攻角定义为机体 $x$ 轴（机头方向）与来流速度矢量在机体对称面内投影之间的夹角：

$$
\alpha = \arctan\left(\frac{w}{u}\right)
$$

其中 $u$ 和 $w$ 分别是机体系下速度矢量的 $x$ 和 $z$ 分量。

### 2.5 侧滑角 (Sideslip Angle, $\beta$)

侧滑角定义为来流速度矢量与机体对称面之间的夹角：

$$
\beta = \arcsin\left(\frac{v}{V}\right)
$$

其中 $v$ 是机体系下速度矢量的 $y$ 分量，$V = \sqrt{u^2 + v^2 + w^2}$ 是总空速。

> **物理意义**: $\alpha$ 描述飞行器"抬头/低头"的程度，$\beta$ 描述飞行器"偏航/侧飞"的程度。理想巡航状态下 $\beta = 0$。

---

## 3. 坐标系

固定翼飞行器分析中常用四种坐标系。坐标系之间的转换关系是建立运动方程的基础。

### 3.1 惯性系 (Inertial Frame) $\{I\}$

- 也称为 NED (North-East-Down) 坐标系
- 原点固定于地面某参考点
- $x_I$ 指向北，$y_I$ 指向东，$z_I$ 指向地心（向下）
- 假设地球平坦且不旋转（对小型无人机足够精确）

### 3.2 机体系 (Body Frame) $\{B\}$

- 原点位于飞行器质心 (Center of Gravity, CG)
- $x_B$ 沿机身纵轴指向机头
- $y_B$ 指向右翼
- $z_B$ 向下（满足右手定则）
- 机体系随飞行器一起运动和旋转

**机体系下的速度和角速度**：

$$
\mathbf{V}_B = \begin{pmatrix} u \ v \ w \end{pmatrix}, \quad
\boldsymbol{\omega}_B = \begin{pmatrix} p \ q \ r \end{pmatrix}
$$

其中 $p$, $q$, $r$ 分别是滚转角速度 (Roll Rate)、俯仰角速度 (Pitch Rate)、偏航角速度 (Yaw Rate)。

### 3.3 风轴系 (Wind Frame) $\{W\}$

- 原点位于质心
- $x_W$ 沿来流速度矢量方向（即飞行方向）
- 升力沿 $-z_W$ 方向，阻力沿 $-x_W$ 方向
- 风轴系是定义气动力最自然的坐标系

**从机体系到风轴系的转换**：

$$
\mathbf{R}_{BW} = \mathbf{R}_z(-\beta) \cdot \mathbf{R}_y(\alpha)
$$

展开为：

$$
\mathbf{R}_{BW} = \begin{pmatrix}
\cos\alpha \cos\beta & -\cos\alpha \sin\beta & -\sin\alpha \
\sin\beta & \cos\beta & 0 \
\sin\alpha \cos\beta & -\sin\alpha \sin\beta & \cos\alpha
\end{pmatrix}
$$

### 3.4 稳定轴系 (Stability Frame) $\{S\}$

- 风轴系的简化版本，仅考虑攻角 $\alpha$ 的旋转
- 假设 $\beta = 0$
- 常用于纵向动力学分析

$$
\mathbf{R}_{BS} = \mathbf{R}_y(\alpha) = \begin{pmatrix}
\cos\alpha & 0 & -\sin\alpha \
0 & 1 & 0 \
\sin\alpha & 0 & \cos\alpha
\end{pmatrix}
$$

### 3.5 坐标系关系总结

```
惯性系 {I}  ──(欧拉角 φ,θ,ψ)──>  机体系 {B}  ──(α,β)──>  风轴系 {W}
                                       │
                                       └──(α)──>  稳定轴系 {S}
```

---

## 4. 固定翼运动方程

### 4.1 六自由度模型 (6-DOF Model)

固定翼飞行器是刚体，具有六个自由度：三个平动自由度（沿 $x$, $y$, $z$）和三个转动自由度（绕 $x$, $y$, $z$）。

**状态向量**：

$$
\mathbf{x} = \begin{pmatrix} x \ y \ z \ \phi \ \theta \ \psi \ u \ v \ w \ p \ q \ r \end{pmatrix}
\quad \text{共 12 个状态变量}
$$

其中：
- $(x, y, z)$ — 惯性系下的位置
- $(\phi, \theta, \psi)$ — 欧拉角：滚转角 (Roll)、俯仰角 (Pitch)、偏航角 (Yaw)
- $(u, v, w)$ — 机体系下的线速度
- $(p, q, r)$ — 机体系下的角速度

### 4.2 平动方程 (Translational Dynamics)

在机体系下，牛顿第二定律为：

$$
m \dot{\mathbf{V}}_B + \boldsymbol{\omega}_B \times (m \mathbf{V}_B) = \mathbf{F}_{\text{aero}} + \mathbf{F}_{\text{gravity}} + \mathbf{F}_{\text{thrust}}
$$

展开为标量形式：

$$
\begin{cases}
m(\dot{u} + qw - rv) = F_{x}^{\text{aero}} + F_{x}^{\text{thrust}} - mg\sin\theta \
m(\dot{v} + ru - pw) = F_{y}^{\text{aero}} + F_{y}^{\text{thrust}} + mg\cos\theta\sin\phi \
m(\dot{w} + pv - qu) = F_{z}^{\text{aero}} + F_{z}^{\text{thrust}} + mg\cos\theta\cos\phi
\end{cases}
$$

**各项含义**：
- $m\dot{\mathbf{V}}_B$: 机体系下的加速度
- $\boldsymbol{\omega}_B \times (m\mathbf{V}_B)$: 科里奥利力项（由于在旋转坐标系中描述运动）
- $\mathbf{F}_{\text{aero}}$: 气动力（升力、阻力、侧力）
- $\mathbf{F}_{\text{gravity}}$: 重力在机体系下的投影
- $\mathbf{F}_{\text{thrust}}$: 推力

**重力在机体系下的分量**：

$$
\mathbf{F}_{\text{gravity}}^B = m \mathbf{R}_{BI} \begin{pmatrix} 0 \ 0 \ g \end{pmatrix} = m \begin{pmatrix} -g\sin\theta \ g\cos\theta\sin\phi \ g\cos\theta\cos\phi \end{pmatrix}
$$

### 4.3 转动方程 (Rotational Dynamics)

欧拉方程描述刚体的转动动力学：

$$
\mathbf{I} \dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (\mathbf{I} \boldsymbol{\omega}_B) = \mathbf{M}_{\text{aero}} + \mathbf{M}_{\text{thrust}}
$$

其中 $\mathbf{I}$ 是惯性张量 (Inertia Tensor)。对于对称飞行器（$I_{xy} = I_{yz} = 0$）：

$$
\mathbf{I} = \begin{pmatrix}
I_{xx} & 0 & -I_{xz} \
0 & I_{yy} & 0 \
-I_{xz} & 0 & I_{zz}
\end{pmatrix}
$$

展开为标量形式：

$$
\begin{cases}
I_{xx}\dot{p} - I_{xz}\dot{r} + (I_{zz} - I_{yy})qr - I_{xz}pq = \mathcal{L} \
I_{yy}\dot{q} + (I_{xx} - I_{zz})pr + I_{xz}(p^2 - r^2) = \mathcal{M} \
I_{zz}\dot{r} - I_{xz}\dot{p} + (I_{yy} - I_{xx})pq + I_{xz}qr = \mathcal{N}
\end{cases}
$$

其中 $\mathcal{L}$, $\mathcal{M}$, $\mathcal{N}$ 分别是绕 $x_B$, $y_B$, $z_B$ 轴的总力矩（滚转力矩、俯仰力矩、偏航力矩）。

### 4.4 运动学方程 (Kinematics)

**位置运动学**（惯性系下的速度）：

$$
\begin{pmatrix} \dot{x} \ \dot{y} \ \dot{z} \end{pmatrix} = \mathbf{R}_{IB} \begin{pmatrix} u \ v \ w \end{pmatrix}
$$

其中 $\mathbf{R}_{IB}$ 是从机体系到惯性系的旋转矩阵（ZYX 欧拉角）：

$$
\mathbf{R}_{IB} = \mathbf{R}_z(\psi) \mathbf{R}_y(\theta) \mathbf{R}_x(\phi)
$$

**姿态运动学**（欧拉角速率与机体角速度的关系）：

$$
\begin{pmatrix} \dot{\phi} \ \dot{\theta} \ \dot{\psi} \end{pmatrix} = \begin{pmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \
0 & \cos\phi & -\sin\phi \
0 & \sin\phi\sec\theta & \cos\phi\sec\theta
\end{pmatrix} \begin{pmatrix} p \ q \ r \end{pmatrix}
$$

> **注意**: 当 $\theta = \pm 90°$ 时，上述表达式存在奇异性 (Gimbal Lock)。实际工程中可使用四元数 (Quaternion) 表示来避免此问题。

### 4.5 完整运动方程总结

将以上方程组合，得到 12 个一阶常微分方程：

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})
$$

其中控制输入为：

$$
\mathbf{u} = \begin{pmatrix} \delta_a \ \delta_e \ \delta_r \ \delta_t \end{pmatrix}
$$

分别对应副翼偏转角 (Aileron)、升降舵偏转角 (Elevator)、方向舵偏转角 (Rudder)、油门 (Throttle)。

---

## 5. 气动力和力矩模型

### 5.1 气动系数的一般形式

气动力和力矩通过无量纲系数表示。这些系数是飞行状态（$\alpha$, $\beta$, $V$）、角速度（$p$, $q$, $r$）和控制面偏转角（$\delta_a$, $\delta_e$, $\delta_r$）的函数。

**无量纲化约定**：
- 力系数: $C = \frac{F}{\frac{1}{2}\rho V^2 S}$
- 力矩系数: $C = \frac{M}{\frac{1}{2}\rho V^2 S \bar{c}}$（纵向）或 $C = \frac{M}{\frac{1}{2}\rho V^2 S b}$（横向）

其中 $\bar{c}$ 是平均气动弦长 (Mean Aerodynamic Chord)，$b$ 是翼展。

### 5.2 纵向气动模型 (Longitudinal Aerodynamics)

纵向运动发生在飞行器对称面内，涉及 $u$, $w$, $q$, $\theta$ 四个状态。

**升力系数**：

$$
C_L = C_{L_0} + C_{L_\alpha}\alpha + C_{L_q}\frac{q\bar{c}}{2V} + C_{L_{\delta_e}}\delta_e
$$

**阻力系数**：

$$
C_D = C_{D_0} + K C_L^2
$$

或更精确地：

$$
C_D = C_{D_0} + C_{D_\alpha}\alpha + C_{D_{\alpha^2}}\alpha^2 + C_{D_{\delta_e}}\delta_e^2
$$

**俯仰力矩系数**：

$$
C_m = C_{m_0} + C_{m_\alpha}\alpha + C_{m_q}\frac{q\bar{c}}{2V} + C_{m_{\delta_e}}\delta_e
$$

> **纵向静稳定性条件**: $C_{m_\alpha} < 0$，即攻角增大时产生低头力矩，使飞行器自动恢复。

### 5.3 横向气动模型 (Lateral-Directional Aerodynamics)

横向运动涉及 $v$, $p$, $r$, $\phi$, $\psi$ 五个状态。

**侧力系数**：

$$
C_Y = C_{Y_\beta}\beta + C_{Y_p}\frac{pb}{2V} + C_{Y_r}\frac{rb}{2V} + C_{Y_{\delta_a}}\delta_a + C_{Y_{\delta_r}}\delta_r
$$

**滚转力矩系数**：

$$
C_l = C_{l_\beta}\beta + C_{l_p}\frac{pb}{2V} + C_{l_r}\frac{rb}{2V} + C_{l_{\delta_a}}\delta_a + C_{l_{\delta_r}}\delta_r
$$

**偏航力矩系数**：

$$
C_n = C_{n_\beta}\beta + C_{n_p}\frac{pb}{2V} + C_{n_r}\frac{rb}{2V} + C_{n_{\delta_a}}\delta_a + C_{n_{\delta_r}}\delta_r
$$

> **横向静稳定性条件**:
> - $C_{l_\beta} < 0$（横滚稳定性，即上反效应 Dihedral Effect）
> - $C_{n_\beta} > 0$（航向稳定性，即风标效应 Weathercock Stability）

### 5.4 控制面效应

| 控制面 | 符号 | 主要作用 | 产生的力矩 |
|--------|------|----------|------------|
| 副翼 (Aileron) | $\delta_a$ | 差动升力 → 滚转 | 滚转力矩 $\mathcal{L}$ |
| 升降舵 (Elevator) | $\delta_e$ | 改变尾翼升力 → 俯仰 | 俯仰力矩 $\mathcal{M}$ |
| 方向舵 (Rudder) | $\delta_r$ | 垂尾侧力 → 偏航 | 偏航力矩 $\mathcal{N}$ |

**副翼**位于主翼后缘外侧，左右差动偏转产生不对称升力，从而产生滚转力矩。

**升降舵**位于水平尾翼后缘，偏转改变尾翼升力，产生绕质心的俯仰力矩。

**方向舵**位于垂直尾翼后缘，偏转产生侧向力，从而产生偏航力矩。

### 5.5 稳定性导数 (Stability Derivatives)

稳定性导数是气动系数对各状态变量的偏导数，描述飞行器对扰动的响应特性。

**纵向稳定性导数**：

| 导数 | 含义 | 典型符号要求 |
|------|------|-------------|
| $C_{L_\alpha}$ | 升力线斜率 | $> 0$ |
| $C_{m_\alpha}$ | 俯仰力矩对攻角的导数 | $< 0$（静稳定） |
| $C_{m_q}$ | 俯仰阻尼导数 | $< 0$（阻尼） |

**横向稳定性导数**：

| 导数 | 含义 | 典型符号要求 |
|------|------|-------------|
| $C_{l_\beta}$ | 滚转力矩对侧滑角的导数 | $< 0$ |
| $C_{n_\beta}$ | 偏航力矩对侧滑角的导数 | $> 0$ |
| $C_{l_p}$ | 滚转阻尼导数 | $< 0$ |
| $C_{n_r}$ | 偏航阻尼导数 | $< 0$ |

---

## 6. 配平和线性化

### 6.1 配平条件 (Trim Condition)

配平是指飞行器处于稳态飞行 (Steady-State Flight) 的状态，即所有加速度为零：

$$
\dot{\mathbf{x}} = f(\mathbf{x}_0, \mathbf{u}_0) = \mathbf{0}
$$

**典型配平状态 — 稳定平飞 (Steady Level Flight)**：

$$
\dot{u} = \dot{v} = \dot{w} = 0, \quad \dot{p} = \dot{q} = \dot{r} = 0
$$

$$
\phi_0 = 0, \quad v_0 = 0, \quad p_0 = q_0 = r_0 = 0
$$

此时力平衡条件为：

$$
\begin{cases}
\text{升力} = \text{重力}: & L_0 = mg\cos\theta_0 \approx mg \
\text{推力} = \text{阻力}: & T_0 = D_0 \
\text{俯仰力矩} = 0: & \mathcal{M}_0 = 0
\end{cases}
$$

从升力平衡可求得配平攻角 $\alpha_0$：

$$
\frac{1}{2}\rho V_0^2 S C_L(\alpha_0) = mg \quad \Rightarrow \quad C_{L_0} + C_{L_\alpha}\alpha_0 = \frac{2mg}{\rho V_0^2 S}
$$

从俯仰力矩平衡可求得配平升降舵偏角 $\delta_{e_0}$：

$$
C_{m_0} + C_{m_\alpha}\alpha_0 + C_{m_{\delta_e}}\delta_{e_0} = 0 \quad \Rightarrow \quad \delta_{e_0} = -\frac{C_{m_0} + C_{m_\alpha}\alpha_0}{C_{m_{\delta_e}}}
$$

### 6.2 小扰动线性化 (Small Perturbation Linearization)

在配平点附近，将状态和输入分解为配平值加小扰动：

$$
\mathbf{x} = \mathbf{x}_0 + \Delta\mathbf{x}, \quad \mathbf{u} = \mathbf{u}_0 + \Delta\mathbf{u}
$$

对非线性方程进行泰勒展开，保留一阶项：

$$
\Delta\dot{\mathbf{x}} = \underbrace{\frac{\partial f}{\partial \mathbf{x}}\bigg|_{\mathbf{x}_0, \mathbf{u}_0}}_{\mathbf{A}} \Delta\mathbf{x} + \underbrace{\frac{\partial f}{\partial \mathbf{u}}\bigg|_{\mathbf{x}_0, \mathbf{u}_0}}_{\mathbf{B}} \Delta\mathbf{u}
$$

得到线性时不变 (LTI) 系统：

$$
\Delta\dot{\mathbf{x}} = \mathbf{A}\Delta\mathbf{x} + \mathbf{B}\Delta\mathbf{u}
$$

其中 $\mathbf{A}$ 是系统矩阵 (State Matrix)，$\mathbf{B}$ 是输入矩阵 (Input Matrix)。

### 6.3 纵向和横向解耦 (Longitudinal-Lateral Decoupling)

对于对称飞行器在对称配平状态下（$\beta_0 = 0$, $\phi_0 = 0$），12 阶系统可以解耦为两个独立的子系统：

**纵向子系统** (Longitudinal)：状态 $\Delta\mathbf{x}_{\text{lon}} = (\Delta u, \Delta w, \Delta q, \Delta\theta)^T$

$$
\Delta\dot{\mathbf{x}}_{\text{lon}} = \mathbf{A}_{\text{lon}} \Delta\mathbf{x}_{\text{lon}} + \mathbf{B}_{\text{lon}} \begin{pmatrix} \Delta\delta_e \ \Delta\delta_t \end{pmatrix}
$$

纵向系统矩阵：

$$
\mathbf{A}_{\text{lon}} = \begin{pmatrix}
X_u & X_w & 0 & -g\cos\theta_0 \
Z_u & Z_w & u_0 + Z_q & -g\sin\theta_0 \
M_u & M_w & M_q & 0 \
0 & 0 & 1 & 0
\end{pmatrix}
$$

其中 $X_u = \frac{1}{m}\frac{\partial F_x}{\partial u}$ 等为量纲稳定性导数 (Dimensional Stability Derivatives)。

**横向子系统** (Lateral-Directional)：状态 $\Delta\mathbf{x}_{\text{lat}} = (\Delta v, \Delta p, \Delta r, \Delta\phi)^T$

$$
\Delta\dot{\mathbf{x}}_{\text{lat}} = \mathbf{A}_{\text{lat}} \Delta\mathbf{x}_{\text{lat}} + \mathbf{B}_{\text{lat}} \begin{pmatrix} \Delta\delta_a \ \Delta\delta_r \end{pmatrix}
$$

横向系统矩阵：

$$
\mathbf{A}_{\text{lat}} = \begin{pmatrix}
Y_v & Y_p & -(u_0 - Y_r) & g\cos\theta_0 \
L_v & L_p & L_r & 0 \
N_v & N_p & N_r & 0 \
0 & 1 & \tan\theta_0 & 0
\end{pmatrix}
$$

### 6.4 特征模态 (Characteristic Modes)

通过分析系统矩阵 $\mathbf{A}$ 的特征值，可以识别飞行器的固有运动模态。

**纵向模态**：

| 模态 | 英文名 | 特征 | 典型参数 |
|------|--------|------|----------|
| 短周期模态 | Short Period | 快速、高阻尼的俯仰振荡 | $\omega_n \approx 2$–$10 \, \text{rad/s}$, $\zeta \approx 0.3$–$0.7$ |
| 长周期模态 | Phugoid | 缓慢的速度-高度交换振荡 | $T \approx 30$–$100 \, \text{s}$, $\zeta \approx 0.01$–$0.1$ |

**横向模态**：

| 模态 | 英文名 | 特征 | 典型参数 |
|------|--------|------|----------|
| 滚转收敛模态 | Roll Subsidence | 快速的纯滚转衰减 | $\tau \approx 0.1$–$1 \, \text{s}$ |
| 螺旋模态 | Spiral | 缓慢的滚转-偏航耦合 | $\tau \approx 10$–$100 \, \text{s}$（可能不稳定） |
| 荷兰滚模态 | Dutch Roll | 滚转-偏航耦合振荡 | $\omega_n \approx 1$–$5 \, \text{rad/s}$, $\zeta \approx 0.05$–$0.3$ |

> **Phugoid 模态**的物理本质是动能和势能之间的周期性交换：飞行器先俯冲加速、再爬升减速，如此往复。该模态频率低、阻尼小，但通常不影响飞行安全。

> **Dutch Roll 模态**是一种"摇摆"运动，飞行器同时发生滚转和偏航振荡。阻尼不足时会导致乘坐不适，通常需要偏航阻尼器 (Yaw Damper) 来增强阻尼。

---

## 7. 飞行控制

### 7.1 控制架构概述

固定翼飞行器的自动驾驶系统 (Autopilot) 通常采用**级联控制 (Cascaded Control)** 架构，从内环到外环依次为：

```
┌─────────────────────────────────────────────────────────┐
│                    任务规划层 (Mission Planner)            │
│                    航点导航 (Waypoint Navigation)          │
├─────────────────────────────────────────────────────────┤
│              制导层 (Guidance)                             │
│         航迹跟踪 → 期望姿态角                               │
├─────────────────────────────────────────────────────────┤
│              姿态控制层 (Attitude Control)                  │
│         姿态角 → 控制面偏转                                  │
├─────────────────────────────────────────────────────────┤
│              执行器 (Actuators)                            │
│         舵机驱动控制面                                      │
└─────────────────────────────────────────────────────────┘
```

### 7.2 纵向控制 (Longitudinal Control)

纵向控制的目标是控制飞行器的俯仰角、高度和空速。

**俯仰角控制 (Pitch Control)**：

使用升降舵 $\delta_e$ 控制俯仰角 $\theta$。最简单的方法是 PID 控制器：

$$
\delta_e = K_{p_\theta}(\theta_{\text{cmd}} - \theta) + K_{d_\theta}(0 - q) + K_{i_\theta}\int(\theta_{\text{cmd}} - \theta) \, dt
$$

其中：
- $K_{p_\theta}$: 比例增益，提供基本的俯仰角跟踪
- $K_{d_\theta}$: 微分增益（使用俯仰角速度 $q$ 反馈），提供阻尼
- $K_{i_\theta}$: 积分增益，消除稳态误差

**高度控制 (Altitude Control)**：

外环控制器根据高度误差生成期望俯仰角：

$$
\theta_{\text{cmd}} = K_{p_h}(h_{\text{cmd}} - h) + K_{i_h}\int(h_{\text{cmd}} - h) \, dt
$$

**空速控制 (Airspeed Control)**：

通过油门 $\delta_t$ 控制空速 $V$：

$$
\delta_t = K_{p_V}(V_{\text{cmd}} - V) + K_{i_V}\int(V_{\text{cmd}} - V) \, dt
$$

> **TECS (Total Energy Control System)**: 一种更先进的纵向控制方法，同时管理飞行器的总能量（动能 + 势能）和能量分配，通过协调油门和升降舵来同时控制空速和高度。

### 7.3 横向控制 (Lateral Control)

横向控制的目标是控制飞行器的滚转角和航向。

**滚转角控制 (Roll Control)**：

使用副翼 $\delta_a$ 控制滚转角 $\phi$：

$$
\delta_a = K_{p_\phi}(\phi_{\text{cmd}} - \phi) + K_{d_\phi}(0 - p)
$$

**航向控制 (Heading Control)**：

外环控制器根据航向误差生成期望滚转角（协调转弯）：

$$
\phi_{\text{cmd}} = K_{p_\psi}(\psi_{\text{cmd}} - \psi)
$$

协调转弯 (Coordinated Turn) 的关系：

$$
\phi = \arctan\left(\frac{V \dot{\psi}}{g}\right)
$$

**航迹跟踪 (Path Following)**：

常用方法包括：
- **L1 导航算法**: 基于前视距离的非线性制导律，适用于直线和圆弧航迹
- **Pure Pursuit**: 追踪前方参考点
- **Stanley 方法**: 基于横向偏差和航向偏差的组合

L1 导航律的侧向加速度指令：

$$
a_{\text{cmd}} = 2\frac{V^2}{L_1}\sin\eta
$$

其中 $L_1$ 是前视距离，$\eta$ 是当前速度方向与前视点方向之间的夹角。

### 7.4 偏航阻尼器 (Yaw Damper)

为了增强 Dutch Roll 模态的阻尼，通常使用方向舵进行偏航速率反馈：

$$
\delta_r = -K_{r} \cdot r
$$

这是一个简单但有效的阻尼增强控制器，通常作为内环独立运行。

### 7.5 自动驾驶架构总结

```
                    ┌──────────┐
  h_cmd ──────────> │ 高度控制  │──> θ_cmd ──┐
                    └──────────┘             │
                    ┌──────────┐             v
  V_cmd ──────────> │ 空速控制  │──> δ_t    ┌──────────┐
                    └──────────┘            │ 俯仰控制  │──> δ_e
                                            └──────────┘
                    ┌──────────┐
  ψ_cmd ──────────> │ 航向控制  │──> φ_cmd ──┐
                    └──────────┘             │
                                             v
                    ┌──────────┐            ┌──────────┐
              r ──> │ 偏航阻尼  │──> δ_r    │ 滚转控制  │──> δ_a
                    └──────────┘            └──────────┘
```

---

## 8. 案例研究

### 8.1 AtlantikSolar — 太阳能无人机

**项目背景**：

AtlantikSolar 是 ETH Zurich 自主系统实验室 (Autonomous Systems Lab, ASL) 开发的太阳能驱动固定翼无人机，目标是实现**连续多日自主飞行** (Multi-Day Endurance Flight)。

**技术参数**：

| 参数 | 数值 |
|------|------|
| 翼展 (Wingspan) | $5.6 \, \text{m}$ |
| 总质量 (Total Mass) | $6.9 \, \text{kg}$ |
| 太阳能电池面积 | $\approx 1.4 \, \text{m}^2$ |
| 巡航速度 | $8$–$11 \, \text{m/s}$ |
| 最大续航 | $> 81 \, \text{h}$（连续飞行世界纪录） |
| 升阻比 $L/D$ | $\approx 18$ |
| 展弦比 $AR$ | $\approx 18.5$ |

**关键技术挑战**：

1. **能量管理 (Energy Management)**：
   - 白天：太阳能电池为电机供电并为电池充电
   - 夜间：依靠电池储能维持飞行
   - 能量平衡条件：白天采集的太阳能 $\geq$ 24 小时飞行总能耗

$$
E_{\text{solar}} = \eta_{\text{panel}} \cdot I_{\text{solar}} \cdot A_{\text{panel}} \cdot t_{\text{day}} \geq P_{\text{flight}} \cdot 24 \, \text{h}
$$

2. **气动效率优化**：
   - 高展弦比机翼设计，最大化 $L/D$
   - 低雷诺数 ($Re \approx 10^5$) 下的翼型优化
   - 最小化结构重量以降低所需升力

3. **鲁棒自主飞行**：
   - 在湍流和阵风条件下保持稳定飞行
   - 自主热气流利用 (Autonomous Thermal Soaring) 以节省能量
   - 基于模型预测控制 (MPC) 的航迹规划

**动力学建模要点**：

AtlantikSolar 的飞行速度低（$V \approx 9 \, \text{m/s}$），工作在低雷诺数区域，气动特性与常规飞行器有显著差异：

- 升力系数对雷诺数敏感
- 层流分离泡 (Laminar Separation Bubble) 的影响显著
- 需要使用 XFOIL 或 CFD 进行精确的气动系数预测

**控制系统设计**：

$$
\text{航迹规划} \xrightarrow{\text{期望航迹}} \text{L1 导航} \xrightarrow{\phi_{\text{cmd}}, \theta_{\text{cmd}}} \text{PID 姿态控制} \xrightarrow{\delta_a, \delta_e, \delta_r} \text{舵机}
$$

AtlantikSolar 使用 Pixhawk 飞控硬件，运行基于 PX4 的自定义固件，实现了从起飞到降落的全自主飞行。

### 8.2 Wingtra — 垂直起降固定翼

**项目背景**：

Wingtra 是 ETH Zurich 孵化的初创公司，开发了一种**尾座式垂直起降固定翼无人机** (Tail-Sitter VTOL Fixed-Wing UAV)，结合了固定翼的高效巡航和旋翼的垂直起降能力。

**技术参数**：

| 参数 | 数值 |
|------|------|
| 翼展 | $1.2 \, \text{m}$ |
| 总质量 | $\approx 4.5 \, \text{kg}$ |
| 巡航速度 | $16 \, \text{m/s}$ |
| 续航时间 | $\approx 55 \, \text{min}$ |
| 覆盖面积 | 单次飞行可达 $200 \, \text{ha}$ |
| 测绘精度 | 厘米级 (cm-level) |

**飞行模式转换 (Transition)**：

Wingtra 的核心技术挑战是在悬停模式和巡航模式之间的平滑过渡：

```
垂直起飞 (Hover)  →  过渡 (Transition)  →  水平巡航 (Cruise)  →  过渡  →  垂直降落
     ↑                                                                    ↓
  尾部朝下              逐渐倾斜，加速                逐渐减速，抬头        尾部朝下
  双旋翼悬停            气动力逐渐接管                旋翼逐渐接管          精确着陆
```

**过渡阶段的动力学**：

过渡阶段是最具挑战性的飞行阶段，因为：

1. **大攻角飞行**: 攻角从 $90°$（悬停）变化到 $\approx 5°$（巡航），经历失速区域
2. **混合控制**: 气动控制面和旋翼差动同时参与姿态控制
3. **非线性强**: 无法使用简单的线性化模型，需要增益调度 (Gain Scheduling) 或非线性控制

过渡阶段的控制分配 (Control Allocation)：

$$
\boldsymbol{\tau}_{\text{total}} = \alpha_{\text{blend}}(V) \cdot \boldsymbol{\tau}_{\text{aero}} + (1 - \alpha_{\text{blend}}(V)) \cdot \boldsymbol{\tau}_{\text{rotor}}
$$

其中 $\alpha_{\text{blend}}(V) \in [0, 1]$ 是基于空速的混合因子：
- $V \approx 0$: $\alpha_{\text{blend}} = 0$，完全依赖旋翼控制
- $V \geq V_{\text{cruise}}$: $\alpha_{\text{blend}} = 1$，完全依赖气动控制面

**应用领域**：

- 高精度航空测绘 (Aerial Surveying)
- 矿山体积测量
- 基础设施巡检
- 农业精准管理

### 8.3 两种方案的对比

| 特性 | AtlantikSolar | Wingtra |
|------|---------------|---------|
| 类型 | 常规固定翼 | 尾座式 VTOL |
| 核心优势 | 超长续航 | 无需跑道 + 高效巡航 |
| 起降方式 | 手抛/弹射 + 腹部着陆 | 垂直起降 |
| 续航 | $> 81 \, \text{h}$ | $\approx 55 \, \text{min}$ |
| 控制难度 | 中等 | 高（过渡阶段） |
| 典型应用 | 长航时监测 | 精准测绘 |

---

## 总结与要点

### 核心公式回顾

**气动力**：

$$
L = \frac{1}{2}\rho V^2 S C_L, \quad D = \frac{1}{2}\rho V^2 S C_D, \quad Y = \frac{1}{2}\rho V^2 S C_Y
$$

**六自由度运动方程**：

$$
m\dot{\mathbf{V}}_B + \boldsymbol{\omega}_B \times (m\mathbf{V}_B) = \mathbf{F}_{\text{aero}} + \mathbf{F}_{\text{gravity}} + \mathbf{F}_{\text{thrust}}
$$

$$
\mathbf{I}\dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (\mathbf{I}\boldsymbol{\omega}_B) = \mathbf{M}_{\text{aero}} + \mathbf{M}_{\text{thrust}}
$$

**线性化模型**：

$$
\Delta\dot{\mathbf{x}} = \mathbf{A}\Delta\mathbf{x} + \mathbf{B}\Delta\mathbf{u}
$$

### 关键概念

1. 固定翼飞行器通过前飞产生升力，必须维持最低空速以避免失速
2. 气动力和力矩通过无量纲系数建模，系数是 $\alpha$, $\beta$, 角速度和控制面偏角的函数
3. 在配平点附近线性化后，纵向和横向运动可以解耦分析
4. 纵向有短周期和长周期 (Phugoid) 两种模态；横向有滚转收敛、螺旋和 Dutch Roll 三种模态
5. 级联 PID 控制架构是固定翼自动驾驶的基础，高级方法包括 TECS 和 MPC

### 延伸阅读

- R. W. Beard & T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press
- R. C. Nelson, *Flight Stability and Automatic Control*, McGraw-Hill
- B. L. Stevens, F. L. Lewis & E. N. Johnson, *Aircraft Control and Simulation*, Wiley
- PX4 Autopilot 开源项目: [https://px4.io](https://px4.io)

---

> **ETH Zurich — Robot Dynamics, Lecture 10**
> 固定翼飞行器动力学建模与控制
