# Lecture 7: 浮动基座动力学与动态控制 (Floating Base Dynamics and Dynamic Control)

> ETH Zurich - Robot Dynamics 课程笔记
> 基于 7floatingbasedynamics_ex.pdf

---

## 目录

1. [浮动基座动力学概述](#1-浮动基座动力学概述)
2. [浮动基座运动方程](#2-浮动基座运动方程)
3. [接触力和约束](#3-接触力和约束)
4. [浮动基座系统的动力学分解](#4-浮动基座系统的动力学分解)
5. [足式机器人的动力学建模](#5-足式机器人的动力学建模)
6. [浮动基座控制方法](#6-浮动基座控制方法)
7. [关键公式总结](#7-关键公式总结)

---

## 1. 浮动基座动力学概述

### 1.1 从固定基座到浮动基座的扩展

在前几讲中，我们讨论的机器人系统都假设基座 (base) 固定在世界坐标系中，例如工业机械臂固定在地面上。这类系统的特点是：

- 基座与世界坐标系之间存在刚性约束
- 广义坐标仅包含关节角度 $q_j \in \mathbb{R}^{n_j}$
- 所有自由度都是被驱动的 (fully actuated)

然而，对于移动机器人（如足式机器人、人形机器人、飞行机器人等），基座本身可以在空间中自由移动，不再固定。这就引出了**浮动基座系统 (Floating Base System)** 的概念：

- 基座可以在三维空间中自由平移和旋转，具有 6 个自由度 (6 DoF)
- 基座的运动不受直接驱动，而是通过关节力矩和接触力间接产生
- 系统是**欠驱动的 (underactuated)**：驱动器数量少于广义坐标维度

### 1.2 广义坐标 (Generalized Coordinates)

浮动基座系统的广义坐标由两部分组成：

$$
q = \begin{bmatrix} q_b \ q_j \end{bmatrix} \in \mathbb{R}^{n_q}
$$

其中：

- $q_b \in \mathbb{R}^7$：基座坐标，包含位置和姿态
  - 位置：$r_b = [x, y, z]^T \in \mathbb{R}^3$（基座在世界坐标系中的位置）
  - 姿态：$\phi_b \in S^3$（通常用单位四元数 (unit quaternion) 表示，$\|\phi_b\| = 1$）
- $q_j \in \mathbb{R}^{n_j}$：关节坐标（各关节角度）
- $n_q = 7 + n_j$：广义坐标总维度

> **注意**：使用四元数表示姿态时，$q_b$ 有 7 个分量，但由于单位四元数约束 $\|\phi_b\| = 1$，实际自由度仍为 6。

### 1.3 广义速度 (Generalized Velocities)

广义速度的定义与广义坐标略有不同：

$$
u = \begin{bmatrix} u_b \ \dot{q}_j \end{bmatrix} \in \mathbb{R}^{n_u}
$$

其中：

- $u_b \in \mathbb{R}^6$：基座的广义速度
  - 线速度：$v_b = [\dot{x}, \dot{y}, \dot{z}]^T \in \mathbb{R}^3$
  - 角速度：$\omega_b = [\omega_x, \omega_y, \omega_z]^T \in \mathbb{R}^3$
- $\dot{q}_j \in \mathbb{R}^{n_j}$：关节速度
- $n_u = 6 + n_j$：广义速度总维度

> **关键区别**：$n_q \neq n_u$（当使用四元数时，$n_q = 7 + n_j$，而 $n_u = 6 + n_j$）。广义坐标和广义速度之间的关系为：
>
> $$\dot{q} = E(q) \, u$$
>
> 其中 $E(q)$ 是一个映射矩阵，将广义速度映射到广义坐标的时间导数。

### 1.4 坐标系与参考系

在浮动基座系统中，常用的坐标系包括：

| 符号 | 名称 | 说明 |
|------|------|------|
| $\mathcal{I}$ | 惯性系 (Inertial Frame) | 固定在世界中的参考系 |
| $\mathcal{B}$ | 基座系 (Body Frame) | 固定在基座上，随基座运动 |
| $\mathcal{C}$ | 接触系 (Contact Frame) | 定义在接触点处 |

基座的位姿可以用齐次变换矩阵 (Homogeneous Transformation Matrix) 表示：

$$
T_{IB} = \begin{bmatrix} C_{IB} & {}_I r_{IB} \ 0 & 1 \end{bmatrix} \in SE(3)
$$

其中 $C_{IB} \in SO(3)$ 是旋转矩阵，${}_I r_{IB} \in \mathbb{R}^3$ 是平移向量。

---

## 2. 浮动基座运动方程

### 2.1 标准形式 (Standard Form)

浮动基座系统的运动方程 (Equations of Motion, EoM) 为：

$$
\boxed{M(q)\dot{u} + b(q, u) + g(q) = S^T \tau + \sum_{i=1}^{n_c} J_{c,i}^T F_{c,i}}
$$

简写为：

$$
M(q)\dot{u} + h(q, u) = S^T \tau + J_c^T F_c
$$

其中 $h(q,u) = b(q,u) + g(q)$ 包含了科氏力/离心力项和重力项。

### 2.2 各项含义

**质量矩阵 (Mass/Inertia Matrix)** $M(q) \in \mathbb{R}^{n_u \times n_u}$：

$$
M(q) = \begin{bmatrix} M_{bb} & M_{bj} \ M_{jb} & M_{jj} \end{bmatrix}
$$

- $M_{bb} \in \mathbb{R}^{6 \times 6}$：基座的惯性矩阵，反映基座平移和旋转的惯性特性
- $M_{bj} = M_{jb}^T \in \mathbb{R}^{6 \times n_j}$：基座与关节之间的耦合惯性
- $M_{jj} \in \mathbb{R}^{n_j \times n_j}$：关节空间的惯性矩阵
- 性质：$M(q)$ 是对称正定矩阵 (symmetric positive definite, SPD)

**非线性项 (Nonlinear Terms)**：

- $b(q, u) \in \mathbb{R}^{n_u}$：科氏力和离心力项 (Coriolis and centrifugal terms)
- $g(q) \in \mathbb{R}^{n_u}$：重力项 (gravity terms)

**选择矩阵 (Selection Matrix)** $S \in \mathbb{R}^{n_j \times n_u}$：

$$
S = \begin{bmatrix} \mathbf{0}_{n_j \times 6} & I_{n_j \times n_j} \end{bmatrix}
$$

选择矩阵的作用是将关节力矩 $\tau \in \mathbb{R}^{n_j}$ 映射到广义力空间。由于基座是非驱动的 (unactuated)，$S$ 的前 6 列为零，表示没有直接作用在基座上的驱动力/力矩。

因此：

$$
S^T \tau = \begin{bmatrix} \mathbf{0}_{6 \times 1} \ \tau \end{bmatrix} \in \mathbb{R}^{n_u}
$$

**接触力项 (Contact Force Terms)**：

- $J_c \in \mathbb{R}^{3n_c \times n_u}$：接触雅可比矩阵 (Contact Jacobian)
- $F_c \in \mathbb{R}^{3n_c}$：接触力向量
- $n_c$：接触点数量
- $J_c^T F_c$：接触力在广义坐标空间中的等效力

### 2.3 分块形式 (Block Form)

将运动方程按基座和关节分块展开：

$$
\begin{bmatrix} M_{bb} & M_{bj} \ M_{jb} & M_{jj} \end{bmatrix}
\begin{bmatrix} \dot{u}_b \ \ddot{q}_j \end{bmatrix}
+
\begin{bmatrix} h_b \ h_j \end{bmatrix}
=
\begin{bmatrix} \mathbf{0} \ \tau \end{bmatrix}
+
\begin{bmatrix} J_{c,b}^T \ J_{c,j}^T \end{bmatrix} F_c
$$

这给出两组方程：

**基座动力学 (Base Dynamics)**（前 6 行）：

$$
M_{bb} \dot{u}_b + M_{bj} \ddot{q}_j + h_b = J_{c,b}^T F_c
$$

> 注意：基座方程中没有 $\tau$ 项，因为基座是非驱动的。基座的运动完全由关节运动的耦合效应和接触力决定。

**关节动力学 (Joint Dynamics)**（后 $n_j$ 行）：

$$
M_{jb} \dot{u}_b + M_{jj} \ddot{q}_j + h_j = \tau + J_{c,j}^T F_c
$$

### 2.4 与固定基座的对比

| 特性 | 固定基座 | 浮动基座 |
|------|----------|----------|
| 广义坐标 | $q = q_j$ | $q = [q_b; q_j]$ |
| 自由度 | $n_j$ | $6 + n_j$ |
| 驱动性 | 全驱动 (fully actuated) | 欠驱动 (underactuated) |
| 基座约束 | 刚性固定 | 自由浮动 |
| 外力 | 基座反力已知 | 接触力需要求解 |
| EoM | $M\ddot{q} + h = \tau$ | $M\dot{u} + h = S^T\tau + J_c^T F_c$ |

---

## 3. 接触力和约束

### 3.1 接触雅可比矩阵 (Contact Jacobian)

接触雅可比矩阵 $J_c$ 描述了接触点速度与广义速度之间的映射关系：

$$
v_c = J_c(q) \, u
$$

其中 $v_c \in \mathbb{R}^{3n_c}$ 是所有接触点的线速度。

对于第 $i$ 个接触点，其雅可比矩阵为：

$$
J_{c,i}(q) = \frac{\partial r_{c,i}}{\partial q} \in \mathbb{R}^{3 \times n_u}
$$

总的接触雅可比矩阵为所有接触点雅可比的堆叠：

$$
J_c = \begin{bmatrix} J_{c,1} \ J_{c,2} \ \vdots \ J_{c,n_c} \end{bmatrix} \in \mathbb{R}^{3n_c \times n_u}
$$

同样可以按基座和关节分块：

$$
J_{c,i} = \begin{bmatrix} J_{c,i}^{(b)} & J_{c,i}^{(j)} \end{bmatrix}
$$

### 3.2 接触约束 (Contact Constraints)

当足端与地面保持刚性接触 (rigid contact) 时，接触点的速度和加速度必须为零：

**速度级约束 (Velocity-level constraint)**：

$$
\boxed{J_c \, u = 0}
$$

**加速度级约束 (Acceleration-level constraint)**：

对速度约束求时间导数：

$$
J_c \, \dot{u} + \dot{J}_c \, u = 0
$$

因此：

$$
\boxed{J_c \, \dot{u} = -\dot{J}_c \, u}
$$

这个约束在动力学控制中非常重要，它确保接触点在运动过程中始终保持静止。

### 3.3 接触力模型 (Contact Force Model)

接触力 $F_c$ 必须满足以下物理约束：

**单边约束 (Unilateral Constraint)**：接触力只能"推"不能"拉"：

$$
F_{c,i}^{(n)} \geq 0
$$

其中 $F_{c,i}^{(n)}$ 是第 $i$ 个接触点的法向力分量。

**摩擦锥约束 (Friction Cone Constraint)**：切向力受摩擦系数 $\mu$ 限制：

$$
\sqrt{(F_{c,i}^{(t_1)})^2 + (F_{c,i}^{(t_2)})^2} \leq \mu \, F_{c,i}^{(n)}
$$

在实际计算中，常用**摩擦锥的线性近似 (Linearized Friction Cone)** 来简化：

$$
D_i F_{c,i} \leq 0
$$

其中 $D_i$ 是近似摩擦锥的约束矩阵。

### 3.4 接触力作为约束力 (Contact Forces as Constraint Forces)

从运动方程和接触约束出发，可以将接触力理解为维持接触约束所需的约束力 (constraint force)。

将运动方程代入加速度级约束：

$$
J_c M^{-1}(S^T \tau + J_c^T F_c - h) = -\dot{J}_c u
$$

整理得到接触力的表达式：

$$
\boxed{F_c = \Lambda_c \left( J_c M^{-1} h - J_c M^{-1} S^T \tau - \dot{J}_c u \right)}
$$

其中 $\Lambda_c$ 是**接触空间惯性矩阵 (Contact Space Inertia Matrix)**：

$$
\Lambda_c = (J_c M^{-1} J_c^T)^{-1} \in \mathbb{R}^{3n_c \times 3n_c}
$$

> **物理意义**：$\Lambda_c$ 描述了系统在接触点处的等效惯性。它类似于操作空间动力学 (Operational Space Dynamics) 中的操作空间惯性矩阵。

---

## 4. 浮动基座系统的动力学分解

### 4.1 基座动力学与关节动力学的关系

从分块运动方程的基座部分：

$$
M_{bb} \dot{u}_b + M_{bj} \ddot{q}_j + h_b = J_{c,b}^T F_c
$$

可以解出基座加速度：

$$
\dot{u}_b = M_{bb}^{-1} \left( J_{c,b}^T F_c - M_{bj} \ddot{q}_j - h_b \right)
$$

这表明基座的加速度完全由以下因素决定：
- 接触力 $F_c$ 通过接触雅可比传递到基座
- 关节加速度 $\ddot{q}_j$ 通过耦合惯性 $M_{bj}$ 影响基座
- 非线性力 $h_b$（重力、科氏力等）

### 4.2 投影动力学 (Projected Dynamics)

投影动力学的核心思想是：通过将运动方程投影到接触约束的零空间 (null space) 中，消除未知的接触力 $F_c$，得到一个不含接触力的动力学方程。

定义**支撑零空间投影矩阵 (Support Null-space Projector)**：

$$
P = I - J_c^T (J_c M^{-1} J_c^T)^{-1} J_c M^{-1} = I - J_c^T \Lambda_c J_c M^{-1}
$$

将运动方程左乘 $P$：

$$
P \left( M \dot{u} + h \right) = P S^T \tau + \underbrace{P J_c^T}_{= \, 0} F_c
$$

由于 $P$ 的构造使得 $P J_c^T = 0$，接触力项被消除：

$$
\boxed{P \left( M \dot{u} + h \right) = P S^T \tau}
$$

这就是**投影动力学方程 (Projected Dynamics Equation)**。

> **优势**：投影动力学方程不包含接触力 $F_c$，因此可以直接用于控制器设计，无需显式估计或测量接触力。

### 4.3 约束一致的动力学 (Constraint-Consistent Dynamics)

在接触约束下，系统的运动被限制在约束一致的子空间中。定义约束一致的加速度：

$$
\dot{u} = \dot{u}_{\text{free}} + \dot{u}_{\text{constraint}}
$$

其中：

- $\dot{u}_{\text{free}} = M^{-1}(S^T \tau - h)$：无约束时的自由加速度
- $\dot{u}_{\text{constraint}} = M^{-1} J_c^T F_c$：由接触约束引起的加速度修正

将加速度级约束 $J_c \dot{u} = -\dot{J}_c u$ 代入，可以得到约束一致的加速度：

$$
\dot{u} = (I - M^{-1} J_c^T \Lambda_c J_c) \dot{u}_{\text{free}} - M^{-1} J_c^T \Lambda_c \dot{J}_c u
$$

定义**动力学一致的零空间投影矩阵 (Dynamically Consistent Null-space Projector)**：

$$
N_c = I - M^{-1} J_c^T \Lambda_c J_c = I - J_c^{\#} J_c
$$

其中 $J_c^{\#} = M^{-1} J_c^T \Lambda_c$ 是接触雅可比的**动力学一致伪逆 (Dynamically Consistent Pseudo-inverse)**。

则约束一致的加速度为：

$$
\boxed{\dot{u} = N_c M^{-1}(S^T \tau - h) - M^{-1} J_c^T \Lambda_c \dot{J}_c u}
$$

### 4.4 投影方法的几何解释

投影动力学可以从几何角度理解：

```
广义力空间 R^{n_u}
├── 接触力子空间: span(J_c^T)     ← 接触力作用的方向
└── 运动子空间: null(J_c)          ← 系统实际可运动的方向

投影矩阵 P 将动力学方程投影到运动子空间中，
消除了接触力子空间中的分量。
```

---

## 5. 足式机器人的动力学建模

### 5.1 四足机器人示例

以四足机器人 (quadruped robot) 为例，如 ANYmal：

**系统参数**：
- 基座：6 DoF（3 平移 + 3 旋转）
- 每条腿：3 个关节（HAA: 髋关节外展/内收, HFE: 髋关节屈/伸, KFE: 膝关节屈/伸）
- 总关节数：$n_j = 12$
- 广义坐标维度：$n_q = 7 + 12 = 19$（使用四元数）
- 广义速度维度：$n_u = 6 + 12 = 18$

**接触配置**：
- 四足站立：$n_c = 4$，接触约束维度 $3 \times 4 = 12$
- 三足支撑：$n_c = 3$，接触约束维度 $3 \times 3 = 9$
- 两足支撑：$n_c = 2$，接触约束维度 $3 \times 2 = 6$

### 5.2 混合动力学 (Hybrid Dynamics)

足式机器人在运动过程中会经历不同的接触模式 (contact modes)，例如：

- **支撑相 (Stance Phase)**：足端与地面接触，$J_c u = 0$
- **摆动相 (Swing Phase)**：足端离开地面，自由运动
- **过渡 (Transition)**：接触建立或断开的瞬间

这种接触/非接触的切换使得系统具有**混合动力学 (Hybrid Dynamics)** 特性：

$$
\text{系统动力学} = \begin{cases}
M\dot{u} + h = S^T\tau + J_c^T F_c, & \text{支撑相 (stance)} \
M\dot{u} + h = S^T\tau, & \text{摆动相 (swing)}
\end{cases}
$$

**接触事件 (Contact Events)**：

1. **触地 (Touch-down)**：摆动腿接触地面
   - 接触约束被激活
   - 可能产生冲击 (impact)，导致速度不连续
   - 冲击动力学：$M(u^+ - u^-) = J_c^T \hat{F}_c$，其中 $\hat{F}_c$ 是冲击力

2. **离地 (Lift-off)**：支撑腿离开地面
   - 接触约束被释放
   - 接触力变为零：$F_c = 0$

**冲击映射 (Impact Map)**：

在触地瞬间，假设位形不变 ($q^+ = q^-$)，但速度发生跳变。结合冲击方程和接触约束 $J_c u^+ = 0$：

$$
u^+ = (I - M^{-1} J_c^T \Lambda_c J_c) \, u^-
$$

$$
\boxed{u^+ = N_c \, u^-}
$$

这表明冲击后的速度是冲击前速度在约束一致零空间中的投影。

### 5.3 步态与接触序列 (Gait and Contact Sequence)

不同的步态 (gait) 对应不同的接触序列：

| 步态 | 接触模式 | 特点 |
|------|----------|------|
| 静步态 (Static Walk) | 始终 $\geq 3$ 足着地 | 重心始终在支撑多边形内 |
| 对角小跑 (Trot) | 对角两足交替 | 动态稳定 |
| 跳跃 (Bound) | 前后两足交替 | 高速运动 |
| 飞奔 (Gallop) | 复杂四足序列 | 最高速度 |
| 跳跃 (Pronk) | 四足同时离地/着地 | 全身跳跃 |

每种步态对应不同的接触雅可比 $J_c$ 和约束维度，控制器需要根据当前接触模式切换动力学模型。

### 5.4 内力 (Internal Forces)

当系统有多个接触点时，存在**内力 (internal forces)** 的概念。内力是指不对系统整体运动产生影响，但在接触点之间传递的力。

对于 $n_c$ 个接触点，接触力可以分解为：

$$
F_c = F_c^{\text{motion}} + F_c^{\text{internal}}
$$

- $F_c^{\text{motion}}$：产生系统运动的接触力分量
- $F_c^{\text{internal}}$：内力分量，满足 $J_c^T F_c^{\text{internal}} = 0$

内力存在于接触雅可比转置的零空间中：

$$
F_c^{\text{internal}} \in \text{null}(J_c^T)
$$

> **物理意义**：内力类似于桌子四条腿上的力分配——即使总的支撑力确定，各腿的力分配仍有无穷多种可能。内力的存在意味着接触力的分配不唯一，这为力优化提供了自由度。

**内力的计算**：

定义内力投影矩阵：

$$
P_{\text{int}} = I - (J_c^T)^{\#} J_c^T
$$

其中 $(J_c^T)^{\#}$ 是 $J_c^T$ 的伪逆。则内力为：

$$
F_c^{\text{internal}} = P_{\text{int}} F_c
$$

### 5.5 质心动力学 (Centroidal Dynamics)

对于足式机器人，质心动力学 (Centroidal Dynamics) 提供了一个简洁的全局视角：

**线动量 (Linear Momentum)**：

$$
\dot{p} = m\ddot{r}_{\text{CoM}} = \sum_{i=1}^{n_c} F_{c,i} + mg
$$

**角动量 (Angular Momentum)**：

$$
\dot{L}_{\text{CoM}} = \sum_{i=1}^{n_c} (r_{c,i} - r_{\text{CoM}}) \times F_{c,i}
$$

合并写成矩阵形式：

$$
\begin{bmatrix} m\ddot{r}_{\text{CoM}} \ \dot{L}_{\text{CoM}} \end{bmatrix}
=
\begin{bmatrix} I_3 & \cdots & I_3 \ [r_{c,1} - r_{\text{CoM}}]_\times & \cdots & [r_{c,n_c} - r_{\text{CoM}}]_\times \end{bmatrix}
\begin{bmatrix} F_{c,1} \ \vdots \ F_{c,n_c} \end{bmatrix}
+
\begin{bmatrix} mg \ 0 \end{bmatrix}
$$

其中 $[\cdot]_\times$ 表示叉积的反对称矩阵 (skew-symmetric matrix)。

---

## 6. 浮动基座控制方法

### 6.1 控制问题的特殊性

浮动基座系统的控制面临以下挑战：

1. **欠驱动性 (Underactuation)**：基座没有直接驱动，只能通过关节力矩和接触力间接控制
2. **接触约束**：控制器必须确保接触力满足物理约束（单边性、摩擦锥）
3. **混合特性**：接触模式的切换导致动力学模型不连续
4. **高维度**：系统自由度多，计算复杂度高

### 6.2 全身控制 (Whole-body Control, WBC)

全身控制是一种同时考虑基座运动和所有关节运动的控制框架。其核心思想是在满足各种约束的前提下，跟踪期望的运动任务。

**任务空间逆动力学 (Task-space Inverse Dynamics)**：

给定期望的任务空间加速度 $\ddot{x}_{\text{des}}$，求解关节力矩 $\tau$ 和接触力 $F_c$：

$$
\ddot{x} = J_{\text{task}} \dot{u} + \dot{J}_{\text{task}} u = \ddot{x}_{\text{des}}
$$

结合运动方程和接触约束，形成一个约束优化问题。

**层次化任务分解 (Hierarchical Task Decomposition)**：

当存在多个任务时，按优先级排列：

1. **最高优先级**：接触约束（$J_c \dot{u} = -\dot{J}_c u$）
2. **高优先级**：基座姿态控制
3. **中优先级**：摆动腿轨迹跟踪
4. **低优先级**：姿态优化、能量最小化等

每个低优先级任务在高优先级任务的零空间中执行：

$$
\tau = \tau_1 + N_1^T \tau_2 + N_1^T N_2^T \tau_3 + \cdots
$$

其中 $N_i$ 是第 $i$ 个任务的零空间投影矩阵。

### 6.3 基于优化的控制 (QP-based Control)

现代足式机器人控制中，最常用的方法是将控制问题表述为**二次规划 (Quadratic Programming, QP)** 问题：

$$
\min_{\dot{u}, \tau, F_c} \quad \sum_{k} w_k \| J_k \dot{u} + \dot{J}_k u - \ddot{x}_{k,\text{des}} \|^2 + w_\tau \|\tau\|^2
$$

$$
\text{s.t.} \quad \begin{cases}
M(q)\dot{u} + h(q,u) = S^T \tau + J_c^T F_c & \text{(动力学约束)} \
J_c \dot{u} = -\dot{J}_c u & \text{(接触约束)} \
F_{c,i}^{(n)} \geq 0 & \text{(单边约束)} \
D_i F_{c,i} \leq 0 & \text{(摩擦锥约束)} \
\tau_{\min} \leq \tau \leq \tau_{\max} & \text{(力矩限制)} \
\dot{u}_{\min} \leq \dot{u} \leq \dot{u}_{\max} & \text{(加速度限制)}
\end{cases}
$$

**决策变量 (Decision Variables)**：$[\dot{u}, \tau, F_c]^T$

**目标函数各项含义**：
- $\| J_k \dot{u} + \dot{J}_k u - \ddot{x}_{k,\text{des}} \|^2$：第 $k$ 个任务的跟踪误差
- $\|\tau\|^2$：力矩正则化项，最小化能量消耗
- $w_k, w_\tau$：权重系数，反映任务优先级

> **优势**：QP 方法可以自然地处理不等式约束（摩擦锥、力矩限制等），并且可以在毫秒级时间内求解，适合实时控制。

### 6.4 力矩分配 (Torque Distribution)

在某些简化的控制框架中，先通过质心动力学规划期望的接触力，再将接触力映射为关节力矩。

**步骤 1：接触力优化**

给定期望的质心加速度 $\ddot{r}_{\text{CoM,des}}$ 和角动量变化率 $\dot{L}_{\text{des}}$：

$$
\min_{F_c} \quad \|F_c\|^2
$$

$$
\text{s.t.} \quad \begin{cases}
A F_c = b_{\text{des}} & \text{(质心动力学)} \
F_{c,i}^{(n)} \geq f_{\min} & \text{(最小法向力)} \
D_i F_{c,i} \leq 0 & \text{(摩擦锥)}
\end{cases}
$$

其中 $A$ 是质心动力学中的接触力映射矩阵，$b_{\text{des}}$ 是期望的质心力/力矩。

**步骤 2：关节力矩计算**

已知期望的接触力 $F_c^*$ 和期望的关节加速度 $\ddot{q}_{j,\text{des}}$，从关节动力学方程：

$$
\tau = M_{jb} \dot{u}_b + M_{jj} \ddot{q}_{j,\text{des}} + h_j - J_{c,j}^T F_c^*
$$

### 6.5 操作空间控制 (Operational Space Control)

操作空间控制 (Operational Space Control, OSC) 是另一种经典的全身控制方法，源自 Khatib 的操作空间框架：

对于任务空间坐标 $x = f(q)$，任务空间动力学为：

$$
\Lambda_x \ddot{x} + \mu_x + p_x = F_x
$$

其中：
- $\Lambda_x = (J_x M^{-1} J_x^T)^{-1}$：操作空间惯性矩阵
- $\mu_x$：操作空间科氏力/离心力
- $p_x$：操作空间重力
- $F_x$：操作空间力

控制律为：

$$
F_x = \Lambda_x \ddot{x}_{\text{des}} + \mu_x + p_x
$$

对应的关节力矩：

$$
\tau = J_x^T F_x + N_x^T \tau_0
$$

其中 $N_x = I - J_x^{\#} J_x$ 是任务零空间投影矩阵，$\tau_0$ 是零空间中的辅助力矩。

---

## 7. 关键公式总结

### 7.1 运动方程的矩阵形式

**完整运动方程**：

$$
\boxed{M(q)\dot{u} + b(q,u) + g(q) = S^T \tau + J_c^T F_c}
$$

**分块形式**：

$$
\begin{bmatrix} M_{bb} & M_{bj} \ M_{jb} & M_{jj} \end{bmatrix}
\begin{bmatrix} \dot{u}_b \ \ddot{q}_j \end{bmatrix}
+
\begin{bmatrix} h_b \ h_j \end{bmatrix}
=
\begin{bmatrix} 0 \ \tau \end{bmatrix}
+
\begin{bmatrix} J_{c,b}^T \ J_{c,j}^T \end{bmatrix} F_c
$$

**选择矩阵**：

$$
S = \begin{bmatrix} 0_{n_j \times 6} & I_{n_j} \end{bmatrix}, \quad
S^T \tau = \begin{bmatrix} 0 \ \tau \end{bmatrix}
$$

### 7.2 接触约束的处理

**速度级约束**：

$$
J_c \, u = 0
$$

**加速度级约束**：

$$
J_c \, \dot{u} = -\dot{J}_c \, u
$$

**接触空间惯性矩阵**：

$$
\Lambda_c = (J_c M^{-1} J_c^T)^{-1}
$$

**接触力求解**：

$$
F_c = \Lambda_c \left( J_c M^{-1} h - J_c M^{-1} S^T \tau - \dot{J}_c u \right)
$$

**摩擦锥约束**：

$$
\sqrt{(F_{c,i}^{(t_1)})^2 + (F_{c,i}^{(t_2)})^2} \leq \mu \, F_{c,i}^{(n)}, \quad F_{c,i}^{(n)} \geq 0
$$

### 7.3 投影与零空间

**支撑零空间投影矩阵**：

$$
P = I - J_c^T \Lambda_c J_c M^{-1}
$$

**投影动力学**：

$$
P(M\dot{u} + h) = P S^T \tau
$$

**动力学一致伪逆**：

$$
J_c^{\#} = M^{-1} J_c^T \Lambda_c
$$

**动力学一致零空间投影**：

$$
N_c = I - J_c^{\#} J_c
$$

**约束一致加速度**：

$$
\dot{u} = N_c M^{-1}(S^T \tau - h) - M^{-1} J_c^T \Lambda_c \dot{J}_c u
$$

### 7.4 从关节力矩到接触力的映射

给定关节力矩 $\tau$，接触力由以下关系确定：

$$
F_c = \Lambda_c \left( J_c M^{-1}(h - S^T \tau) + \dot{J}_c u \right)
$$

反过来，给定期望接触力 $F_c^*$，所需的关节力矩为：

$$
\tau = (S M^{-1} J_c^T)^{\#} \left( S M^{-1}(h - J_c^T F_c^*) + S \dot{u}_{\text{des}} \right)
$$

**质心动力学映射**：

$$
\begin{bmatrix} m\ddot{r}_{\text{CoM}} - mg \ \dot{L}_{\text{CoM}} \end{bmatrix}
= A_c \, F_c
$$

其中：

$$
A_c = \begin{bmatrix} I_3 & I_3 & \cdots & I_3 \ [r_{c,1} - r_{\text{CoM}}]_\times & [r_{c,2} - r_{\text{CoM}}]_\times & \cdots & [r_{c,n_c} - r_{\text{CoM}}]_\times \end{bmatrix} \in \mathbb{R}^{6 \times 3n_c}
$$

### 7.5 冲击动力学

**冲击方程**：

$$
M(u^+ - u^-) = J_c^T \hat{F}_c
$$

**冲击后速度**（完全非弹性碰撞）：

$$
u^+ = N_c \, u^- = (I - M^{-1} J_c^T \Lambda_c J_c) \, u^-
$$

**能量损失**：

$$
\Delta T = \frac{1}{2} (u^+)^T M u^+ - \frac{1}{2} (u^-)^T M u^- \leq 0
$$

### 7.6 QP 控制器标准形式

$$
\min_{x} \quad \frac{1}{2} x^T H x + c^T x
$$

$$
\text{s.t.} \quad A_{\text{eq}} x = b_{\text{eq}}, \quad A_{\text{ineq}} x \leq b_{\text{ineq}}
$$

其中决策变量 $x = [\dot{u}^T, \tau^T, F_c^T]^T$，各矩阵由运动方程、接触约束、摩擦锥约束和力矩限制构成。

---

## 附录 A：符号表 (Notation Reference)

| 符号 | 含义 | 维度 |
|------|------|------|
| $q$ | 广义坐标 | $n_q = 7 + n_j$ |
| $u$ | 广义速度 | $n_u = 6 + n_j$ |
| $q_b$ | 基座坐标（位置 + 四元数） | $7$ |
| $q_j$ | 关节坐标 | $n_j$ |
| $u_b$ | 基座速度（线速度 + 角速度） | $6$ |
| $M(q)$ | 质量/惯性矩阵 | $n_u \times n_u$ |
| $b(q,u)$ | 科氏力和离心力 | $n_u$ |
| $g(q)$ | 重力项 | $n_u$ |
| $h(q,u)$ | 非线性项 $b + g$ | $n_u$ |
| $S$ | 选择矩阵 | $n_j \times n_u$ |
| $\tau$ | 关节力矩 | $n_j$ |
| $J_c$ | 接触雅可比矩阵 | $3n_c \times n_u$ |
| $F_c$ | 接触力 | $3n_c$ |
| $\Lambda_c$ | 接触空间惯性矩阵 | $3n_c \times 3n_c$ |
| $P$ | 支撑零空间投影矩阵 | $n_u \times n_u$ |
| $N_c$ | 动力学一致零空间投影 | $n_u \times n_u$ |
| $J_c^{\#}$ | 动力学一致伪逆 | $n_u \times 3n_c$ |
| $n_c$ | 接触点数量 | 标量 |
| $\mu$ | 摩擦系数 | 标量 |

---

## 附录 B：常见问题与直觉理解

### Q1：为什么基座是非驱动的？

足式机器人的基座（躯干）没有安装推进器或轮子，它不能直接产生力或力矩来驱动自身运动。基座的运动完全依赖于：
- 关节力矩通过腿部结构传递到地面，产生接触力
- 接触力的反作用力通过腿部结构传递回基座

这就是为什么选择矩阵 $S$ 的前 6 列为零。

### Q2：投影动力学的直觉是什么？

想象你站在地面上。地面对你施加的接触力（法向支撑力和摩擦力）是被动的——它们的大小恰好使你的脚不穿透地面、不滑动。投影动力学通过数学投影消除了这些被动力，只保留了你主动控制的部分（肌肉力/关节力矩）。

### Q3：为什么需要摩擦锥约束？

如果不考虑摩擦锥约束，控制器可能会计算出需要很大的切向接触力。但实际上，地面能提供的切向力受摩擦系数限制。如果要求的切向力超过摩擦锥范围，脚就会打滑，接触约束失效。

### Q4：内力有什么实际意义？

当四足机器人站立时，四条腿的接触力之和必须等于重力（牛顿第二定律）。但每条腿分担多少力有无穷多种可能。例如，对角两条腿可以互相"推"或"拉"，只要总效果不变。这些不影响整体运动的力分量就是内力。在控制中，可以利用内力来优化力分配，例如避免某条腿过载。

---

## 附录 C：与其他讲次的联系

| 讲次 | 主题 | 与本讲的关系 |
|------|------|-------------|
| Lecture 1-2 | 运动学 | 接触雅可比 $J_c$ 的计算基础 |
| Lecture 3-4 | 动力学基础 | 固定基座 EoM 是浮动基座的特例 |
| Lecture 5 | 逆动力学 | 全身控制中的逆动力学求解 |
| Lecture 6 | 正向动力学 | 仿真中的浮动基座正向动力学 |
| Lecture 8 | 轨迹优化 | 基于浮动基座模型的运动规划 |

---

> **参考文献**：
> - M. Hutter et al., "ANYmal - A Highly Mobile and Dynamic Quadrupedal Robot," IEEE/RSJ IROS, 2016.
> - O. Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation," IEEE J. Robotics and Automation, 1987.
> - S. Kajita et al., "Resolved Momentum Control: Humanoid Motion Planning based on the Linear and Angular Momentum," IEEE/RSJ IROS, 2003.
> - C. D. Bellicoso et al., "Dynamic Locomotion Through Online Nonlinear Motion Optimization for Quadrupedal Robots," IEEE Robotics and Automation Letters, 2018.
> - R. Grandia et al., "Multi-Layered Safety for Legged Robots via Control Barrier Functions and Model Predictive Control," IEEE ICRA, 2021.
