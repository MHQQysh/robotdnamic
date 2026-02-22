# Lecture 6: Dynamics 2 — 动态控制方法 (Dynamic Control Methods)

> ETH Zurich · Robot Dynamics · Lecture 06
>
> 本讲义涵盖基于动力学模型的机器人控制方法，从关节阻抗控制到操作空间控制，再到基于优化的控制框架。

---

## 1. 回顾 (Recapitulation)

### 1.1 刚体多体系统运动方程

一般形式的运动方程 (Equations of Motion)：

$$
M(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + J_c^T \mathbf{F}_c
$$

其中各项含义：

| 符号 | 含义 | 维度 |
|------|------|------|
| $M(\mathbf{q})$ | 广义质量/惯性矩阵 (Mass/Inertia Matrix) | $n \times n$ |
| $\mathbf{b}(\mathbf{q}, \dot{\mathbf{q}})$ | 科氏力与离心力项 (Coriolis & Centrifugal) | $n \times 1$ |
| $\mathbf{g}(\mathbf{q})$ | 重力项 (Gravity) | $n \times 1$ |
| $\boldsymbol{\tau}$ | 广义力/关节力矩 (Generalized Forces) | $n \times 1$ |
| $J_c^T \mathbf{F}_c$ | 接触力映射到关节空间 (Contact Forces) | $n \times 1$ |

### 1.2 三种推导方法

1. **Newton-Euler 方法**：对每个刚体分别列写牛顿（平动）和欧拉（转动）方程，包含约束力，再消去内力。
2. **Projected Newton-Euler 方法**：利用投影矩阵将约束力消除，直接得到最小坐标形式的运动方程。
3. **Lagrange 方法**：基于能量（动能 $T$、势能 $U$），通过 Lagrangian $\mathcal{L} = T - U$ 和 Euler-Lagrange 方程推导：

$$
\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i
$$

三种方法最终得到相同的运动方程，但计算效率和适用场景不同。Newton-Euler 递推算法适合实时计算，Lagrange 方法适合理论分析。

---

## 2. 位置控制 vs 力矩控制 (Position vs Torque Controlled Robot Arms)

### 2.1 经典位置控制 (Position Control)

传统工业机器人采用**高增益 PID 位置控制**：

$$
\tau = K_P(\mathbf{q}_d - \mathbf{q}) + K_I \int(\mathbf{q}_d - \mathbf{q})\,dt + K_D(\dot{\mathbf{q}}_d - \dot{\mathbf{q}})
$$

特点：
- 每个关节独立控制，视为 SISO 系统
- 使用非常高的增益 → 关节非常"刚硬" (stiff)
- **不关心动力学模型**：高增益使得系统对模型误差不敏感
- 适合重复性高、环境确定的工业场景（如焊接、喷涂）

缺点：
- 与环境接触时，高刚度可能导致巨大的接触力
- 不适合人机协作 (Human-Robot Interaction)
- 无法实现柔顺行为 (Compliant Behavior)

### 2.2 关节力矩控制 (Torque Control)

现代协作机器人（如 KUKA iiwa, Franka Emika）采用**力矩控制**：

- 电机直接输出期望力矩（而非期望位置）
- 可以实现力反馈 (Force Feedback)
- 主动调节系统动力学特性
- 允许低增益控制 → 柔顺交互

力矩控制是后续所有高级控制方法的基础。

---

## 3. 关节阻抗控制 (Joint Impedance Control)

### 3.1 基本思想

阻抗控制的目标：让机器人关节表现为一个**虚拟弹簧-阻尼器系统**。

控制律：

$$
\boldsymbol{\tau} = K_p(\mathbf{q}^* - \mathbf{q}) + K_d(\dot{\mathbf{q}}^* - \dot{\mathbf{q}}) + \hat{\mathbf{g}}(\mathbf{q})
$$

其中：
- $K_p \in \mathbb{R}^{n \times n}$：刚度矩阵 (Stiffness Matrix)，对角正定
- $K_d \in \mathbb{R}^{n \times n}$：阻尼矩阵 (Damping Matrix)，对角正定
- $\mathbf{q}^*$：期望关节位置 (Desired Joint Position)
- $\hat{\mathbf{g}}(\mathbf{q})$：重力补偿项 (Gravity Compensation)

### 3.2 重力补偿 (Gravity Compensation)

如果没有重力补偿项 $\hat{\mathbf{g}}(\mathbf{q})$，机器人在静止时会因为重力而偏离期望位置。

将控制律代入运动方程（无接触力情况）：

$$
M(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q}) = K_p(\mathbf{q}^* - \mathbf{q}) + K_d(\dot{\mathbf{q}}^* - \dot{\mathbf{q}}) + \hat{\mathbf{g}}(\mathbf{q})
$$

在平衡点 $\ddot{\mathbf{q}} = 0, \dot{\mathbf{q}} = 0$：

$$
\mathbf{g}(\mathbf{q}) = K_p(\mathbf{q}^* - \mathbf{q}) + \hat{\mathbf{g}}(\mathbf{q})
$$

### 3.3 静态偏移问题 (Steady-State Offset)

**情况 1：完美重力补偿** $\hat{\mathbf{g}}(\mathbf{q}) = \mathbf{g}(\mathbf{q})$

$$
K_p(\mathbf{q}^* - \mathbf{q}) = 0 \implies \mathbf{q} = \mathbf{q}^*
$$

机器人精确到达期望位置，无静态偏移。

**情况 2：无重力补偿** $\hat{\mathbf{g}}(\mathbf{q}) = 0$

$$
K_p(\mathbf{q}^* - \mathbf{q}) = \mathbf{g}(\mathbf{q}) \implies \mathbf{q} = \mathbf{q}^* - K_p^{-1}\mathbf{g}(\mathbf{q})
$$

存在静态偏移 $\Delta \mathbf{q} = K_p^{-1}\mathbf{g}(\mathbf{q})$。增大 $K_p$ 可以减小偏移，但会降低柔顺性。

**情况 3：不完美重力补偿** $\hat{\mathbf{g}}(\mathbf{q}) \approx \mathbf{g}(\mathbf{q})$

$$
\Delta \mathbf{q} = K_p^{-1}[\mathbf{g}(\mathbf{q}) - \hat{\mathbf{g}}(\mathbf{q})]
$$

偏移取决于重力模型的精度。实际中，由于负载变化、参数不确定性等，总会存在一定偏移。

> **Quiz 思考题**：如果一个 2-DOF 机械臂末端负载增加了 0.5 kg，而重力补偿模型未更新，关节 2 的静态偏移方向是什么？
>
> **答**：关节 2 会向重力方向偏移（下垂），偏移量为 $\Delta q_2 = K_{p,2}^{-1} \Delta g_2$，其中 $\Delta g_2$ 是未补偿的额外重力矩。

---

## 4. 逆动力学控制 (Inverse Dynamics Control)

### 4.1 核心思想

阻抗控制的问题：闭环动力学仍然是**耦合的、非线性的**。不同关节之间通过惯性矩阵 $M(\mathbf{q})$ 和科氏力项 $\mathbf{b}(\mathbf{q}, \dot{\mathbf{q}})$ 相互影响。

逆动力学控制（也称 Computed Torque Control）的思路：**利用完整的动力学模型来补偿所有非线性项**，使闭环系统变为线性解耦系统。

### 4.2 控制律

$$
\boldsymbol{\tau} = M(\mathbf{q})\ddot{\mathbf{q}}^* + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})
$$

其中 $\ddot{\mathbf{q}}^*$ 是"虚拟输入"（Commanded Acceleration）。

将此控制律代入运动方程：

$$
M(\mathbf{q})\ddot{\mathbf{q}} + \cancel{\mathbf{b}(\mathbf{q}, \dot{\mathbf{q}})} + \cancel{\mathbf{g}(\mathbf{q})} = M(\mathbf{q})\ddot{\mathbf{q}}^* + \cancel{\mathbf{b}(\mathbf{q}, \dot{\mathbf{q}})} + \cancel{\mathbf{g}(\mathbf{q})}
$$

由于 $M(\mathbf{q})$ 正定可逆：

$$
\ddot{\mathbf{q}} = \ddot{\mathbf{q}}^*
$$

系统被**完全线性化**。每个关节变成一个简单的双积分器 (Double Integrator)。

### 4.3 PD 控制律设计

选择虚拟输入为 PD 反馈形式：

$$
\ddot{\mathbf{q}}^* = \ddot{\mathbf{q}}_d + K_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + K_p(\mathbf{q}_d - \mathbf{q})
$$

定义跟踪误差 $\mathbf{e} = \mathbf{q}_d - \mathbf{q}$，闭环误差动力学为：

$$
\ddot{\mathbf{e}} + K_d \dot{\mathbf{e}} + K_p \mathbf{e} = 0
$$

这是一个**线性、解耦的二阶系统**。每个关节 $i$ 独立满足：

$$
\ddot{e}_i + k_{d,i} \dot{e}_i + k_{p,i} e_i = 0
$$

等价于一个**质量-弹簧-阻尼器** (Mass-Spring-Damper) 系统，质量为 1。

### 4.4 增益选择

对于每个关节，特征方程为：

$$
s^2 + k_{d,i} s + k_{p,i} = 0
$$

为实现临界阻尼 (Critical Damping)，选择：

$$
k_{d,i} = 2\sqrt{k_{p,i}}, \quad \text{即阻尼比 } \zeta = 1
$$

自然频率 $\omega_n = \sqrt{k_{p,i}}$ 决定了响应速度。

> **Quiz 示例**：设 $k_{p} = 100$，$k_{d} = 20$，初始误差 $e(0) = 0.1$ rad，$\dot{e}(0) = 0$。
>
> 特征方程：$s^2 + 20s + 100 = (s+10)^2 = 0$（临界阻尼）
>
> 解：$e(t) = (0.1 + t) e^{-10t}$
>
> 误差单调衰减至零，无超调，无静态偏移。

### 4.5 逆动力学控制的优缺点

**优点**：
- 闭环系统完全线性化、解耦
- 无静态偏移（无需积分项）
- 增益调节简单直观

**缺点**：
- 需要精确的动力学模型 $M(\mathbf{q})$, $\mathbf{b}(\mathbf{q}, \dot{\mathbf{q}})$, $\mathbf{g}(\mathbf{q})$
- 计算量大（需要实时计算完整动力学）
- 模型误差会导致不完全的线性化和解耦

---

## 5. 多任务逆动力学控制 (Inverse Dynamics with Multiple Tasks)

### 5.1 从关节空间到任务空间

很多情况下，我们关心的不是关节角度 $\mathbf{q}$，而是**任务空间**中的变量，例如末端执行器的位置和姿态。

任务空间变量 $\mathbf{x} \in \mathbb{R}^m$ 与关节变量的关系：

$$
\mathbf{x} = f(\mathbf{q})
$$

微分关系（Jacobian）：

$$
\dot{\mathbf{x}} = J(\mathbf{q}) \dot{\mathbf{q}}, \quad \ddot{\mathbf{x}} = J(\mathbf{q}) \ddot{\mathbf{q}} + \dot{J}(\mathbf{q}) \dot{\mathbf{q}}
$$

如果我们希望实现任务空间加速度 $\ddot{\mathbf{x}}^*$，需要求解：

$$
\ddot{\mathbf{q}}^* = J^{\dagger}(\ddot{\mathbf{x}}^* - \dot{J}\dot{\mathbf{q}})
$$

其中 $J^{\dagger}$ 是 Jacobian 的伪逆 (Pseudoinverse)。

### 5.2 等优先级多任务 (Equal Priority Multi-Task)

当有多个任务 $\mathbf{x}_1, \mathbf{x}_2, \ldots, \mathbf{x}_k$ 时，可以将它们堆叠：

$$
\underbrace{\begin{bmatrix} \ddot{\mathbf{x}}_1^* - \dot{J}_1 \dot{\mathbf{q}} \ \ddot{\mathbf{x}}_2^* - \dot{J}_2 \dot{\mathbf{q}} \ \vdots \end{bmatrix}}_{\mathbf{v}} = \underbrace{\begin{bmatrix} J_1 \ J_2 \ \vdots \end{bmatrix}}_{\bar{J}} \ddot{\mathbf{q}}^*
$$

最小二乘解：

$$
\ddot{\mathbf{q}}^* = \bar{J}^{\dagger} \mathbf{v}
$$

所有任务被**等权重**对待。当任务之间冲突时，误差在所有任务之间均匀分配。

### 5.3 层次化优先级 (Hierarchical Priority / Null-Space Projection)

在很多场景中，某些任务比其他任务更重要。例如：避障 > 末端执行器位置 > 姿态优化。

**两级优先级**的解法：

$$
\ddot{\mathbf{q}}^* = J_1^{\dagger}(\ddot{\mathbf{x}}_1^* - \dot{J}_1 \dot{\mathbf{q}}) + N_1 J_{2|1}^{\dagger}(\ddot{\mathbf{x}}_2^* - \dot{J}_2 \dot{\mathbf{q}} - J_2 J_1^{\dagger}(\ddot{\mathbf{x}}_1^* - \dot{J}_1 \dot{\mathbf{q}}))
$$

其中：
- $N_1 = I - J_1^{\dagger} J_1$：任务 1 的零空间投影矩阵 (Null-Space Projector)
- $J_{2|1} = J_2 N_1$：任务 2 在任务 1 零空间中的有效 Jacobian

**关键性质**：
- 任务 1 被精确满足（在可行范围内）
- 任务 2 仅在不影响任务 1 的前提下被尽可能满足
- 可以递归扩展到任意多级优先级

> **Quiz 思考题**：一个 7-DOF 机械臂，任务 1 是 3D 末端位置控制（3 维），任务 2 是末端姿态控制（3 维）。零空间 $N_1$ 的秩是多少？任务 2 能被完全满足吗？
>
> **答**：$N_1$ 的秩为 $7 - 3 = 4$。任务 2 是 3 维的，$4 \geq 3$，所以在一般构型下任务 2 可以被完全满足。剩余 1 个自由度可用于第三级任务（如关节极限避免）。

---

## 6. 任务空间动力学 (Task Space Dynamics)

### 6.1 从关节空间到任务空间的映射

我们已知关节空间运动方程：

$$
M(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}
$$

目标：将其转换为以任务空间变量 $\mathbf{x}$ 表示的动力学方程。

利用运动学关系：

$$
\dot{\mathbf{x}} = J \dot{\mathbf{q}} \implies \dot{\mathbf{q}} = J^{\dagger} \dot{\mathbf{x}} \quad (\text{假设方阵或使用伪逆})
$$

$$
\ddot{\mathbf{x}} = J \ddot{\mathbf{q}} + \dot{J} \dot{\mathbf{q}} \implies \ddot{\mathbf{q}} = J^{-1}(\ddot{\mathbf{x}} - \dot{J}\dot{\mathbf{q}})
$$

将 $\ddot{\mathbf{q}}$ 代入关节空间运动方程，并左乘 $J^{-T}$：

$$
\underbrace{(J M^{-1} J^T)^{-1}}_{\Lambda} \ddot{\mathbf{x}} + \underbrace{\Lambda J M^{-1} \mathbf{b} - \Lambda \dot{J} \dot{\mathbf{q}}}_{\boldsymbol{\mu}} + \underbrace{\Lambda J M^{-1} \mathbf{g}}_{\mathbf{p}} = \mathbf{F}
$$

### 6.2 任务空间动力学方程

$$
\boxed{\Lambda(\mathbf{q}) \ddot{\mathbf{x}} + \boldsymbol{\mu}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{p}(\mathbf{q}) = \mathbf{F}}
$$

各项含义：

| 符号 | 名称 | 表达式 |
|------|------|--------|
| $\Lambda$ | 任务空间惯性矩阵 (Task Space Inertia Matrix) | $\Lambda = (J M^{-1} J^T)^{-1}$ |
| $\boldsymbol{\mu}$ | 任务空间科氏力与离心力 | $\boldsymbol{\mu} = \Lambda J M^{-1} \mathbf{b} - \Lambda \dot{J} \dot{\mathbf{q}}$ |
| $\mathbf{p}$ | 任务空间重力 | $\mathbf{p} = \Lambda J M^{-1} \mathbf{g}$ |
| $\mathbf{F}$ | 任务空间广义力 (Task Space Force) | $\mathbf{F} = J^{-T} \boldsymbol{\tau}$ |

### 6.3 关节力矩与任务空间力的关系

任务空间力 $\mathbf{F}$ 与关节力矩 $\boldsymbol{\tau}$ 的映射关系：

$$
\boldsymbol{\tau} = J^T \mathbf{F}
$$

这是虚功原理 (Principle of Virtual Work) 的直接结果：

$$
\delta W = \boldsymbol{\tau}^T \delta \mathbf{q} = \mathbf{F}^T \delta \mathbf{x} = \mathbf{F}^T J \delta \mathbf{q}
$$

因此 $\boldsymbol{\tau} = J^T \mathbf{F}$。

### 6.4 任务空间惯性矩阵的性质

$\Lambda(\mathbf{q})$ 具有以下重要性质：

1. **对称正定**：$\Lambda = \Lambda^T > 0$（因为 $M$ 对称正定，$J$ 满秩时成立）
2. **构型相关**：$\Lambda$ 随机器人构型变化，反映了末端执行器在不同方向上的"有效质量"
3. **各向异性**：一般情况下，末端执行器在不同方向上的惯性不同

> **Quiz 示例**：考虑一个平面 2-DOF 机械臂，在某构型下 $\Lambda$ 的特征值为 $\lambda_1 = 2$ kg 和 $\lambda_2 = 8$ kg。这意味着什么？
>
> **答**：末端执行器在 $\lambda_1$ 对应的特征向量方向上的有效质量为 2 kg（容易加速），在 $\lambda_2$ 方向上的有效质量为 8 kg（难以加速）。惯性椭球 (Inertia Ellipsoid) 的长轴对应小特征值方向。

---

## 7. 末端执行器运动控制 (End-Effector Motion Control)

### 7.1 问题描述

给定末端执行器的期望轨迹 $\mathbf{x}_d(t)$，设计关节力矩 $\boldsymbol{\tau}$ 使末端执行器跟踪该轨迹。

### 7.2 方法一：关节空间逆动力学 + 逆运动学

步骤：
1. 通过逆运动学将 $\mathbf{x}_d(t)$ 转换为 $\mathbf{q}_d(t)$
2. 应用关节空间逆动力学控制

$$
\boldsymbol{\tau} = M(\mathbf{q})\ddot{\mathbf{q}}^* + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})
$$

$$
\ddot{\mathbf{q}}^* = \ddot{\mathbf{q}}_d + K_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + K_p(\mathbf{q}_d - \mathbf{q})
$$

缺点：需要求解逆运动学，可能存在奇异性和多解问题。

### 7.3 方法二：任务空间直接控制

直接在任务空间设计控制律，避免逆运动学。

**步骤 1**：确定期望末端执行器加速度

$$
\ddot{\mathbf{x}}^* = \ddot{\mathbf{x}}_d + K_d(\dot{\mathbf{x}}_d - \dot{\mathbf{x}}) + K_p(\mathbf{x}_d - \mathbf{x})
$$

**步骤 2**：将期望加速度转换为关节加速度

$$
\ddot{\mathbf{q}}^* = J^{-1}(\ddot{\mathbf{x}}^* - \dot{J}\dot{\mathbf{q}})
$$

**步骤 3**：计算关节力矩

$$
\boldsymbol{\tau} = M(\mathbf{q})\ddot{\mathbf{q}}^* + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})
$$

合并后：

$$
\boldsymbol{\tau} = M J^{-1}(\ddot{\mathbf{x}}^* - \dot{J}\dot{\mathbf{q}}) + \mathbf{b} + \mathbf{g}
$$

### 7.4 轨迹控制中的注意事项

- **姿态误差**：对于 SO(3) 上的姿态，不能简单做减法。需要使用旋转误差的对数映射或四元数误差。
- **奇异性处理**：当 Jacobian 接近奇异时，$J^{-1}$ 或 $J^{\dagger}$ 的数值不稳定。可使用阻尼最小二乘 (Damped Least Squares)：

$$
J^{\dagger}_{\text{DLS}} = J^T(JJ^T + \lambda^2 I)^{-1}
$$

- **冗余自由度**：当 $n > m$ 时，利用零空间实现次级目标。

---

## 8. 操作空间控制 (Operational Space Control, OSC)

### 8.1 基本框架

操作空间控制 (Operational Space Control) 由 Oussama Khatib 于 1987 年提出，是一个在任务空间中**统一运动控制和力控制**的框架。

核心思想：直接在任务空间中设计力/力矩，然后通过 Jacobian 转置映射到关节空间。

基本控制律：

$$
\boldsymbol{\tau}^* = J^T \left( \Lambda \ddot{\mathbf{w}}_{\text{des}} + \boldsymbol{\mu} + \mathbf{p} + \mathbf{F}_{\text{contact}} \right)
$$

其中：
- $\ddot{\mathbf{w}}_{\text{des}}$：期望任务空间加速度（由运动控制器产生）
- $\Lambda \ddot{\mathbf{w}}_{\text{des}}$：实现期望加速度所需的任务空间惯性力
- $\boldsymbol{\mu}$：任务空间科氏力/离心力补偿
- $\mathbf{p}$：任务空间重力补偿
- $\mathbf{F}_{\text{contact}}$：期望接触力

将此代入任务空间动力学方程，可验证闭环系统实现 $\ddot{\mathbf{x}} = \ddot{\mathbf{w}}_{\text{des}}$。

### 8.2 冗余机器人的 OSC

对于冗余机器人（$n > m$），任务空间控制只约束了 $m$ 个自由度，剩余 $n - m$ 个自由度可用于次级目标：

$$
\boldsymbol{\tau} = J^T \mathbf{F}_{\text{task}} + N^T \boldsymbol{\tau}_0
$$

其中：
- $N = I - J^{\dagger} J$：零空间投影矩阵
- $\boldsymbol{\tau}_0$：零空间力矩（用于次级目标，如关节极限避免、奇异性避免等）
- $N^T \boldsymbol{\tau}_0$：保证零空间力矩不影响主任务

### 8.3 运动与力的统一控制 (Unified Motion and Force Control)

在与环境接触的场景中，末端执行器的某些方向需要做**运动控制**，另一些方向需要做**力控制**。

引入选择矩阵 (Selection Matrices)：

$$
\Sigma_M + \Sigma_F = I
$$

- $\Sigma_M$：运动控制方向的选择矩阵 (Motion Selection Matrix)
- $\Sigma_F$：力控制方向的选择矩阵 (Force Selection Matrix)

$\Sigma_M$ 和 $\Sigma_F$ 是对角矩阵，对角元素为 0 或 1，互补。

统一控制律：

$$
\mathbf{F} = \Sigma_M \mathbf{F}_{\text{motion}} + \Sigma_F \mathbf{F}_{\text{force}}
$$

其中：
- $\mathbf{F}_{\text{motion}} = \Lambda \ddot{\mathbf{w}}_{\text{des}} + \boldsymbol{\mu} + \mathbf{p}$（运动控制部分）
- $\mathbf{F}_{\text{force}} = \mathbf{F}_d$（期望接触力）

最终关节力矩：

$$
\boldsymbol{\tau} = J^T \left[ \Sigma_M (\Lambda \ddot{\mathbf{w}}_{\text{des}} + \boldsymbol{\mu} + \mathbf{p}) + \Sigma_F \mathbf{F}_d \right]
$$

### 8.4 示例：圆柱销插孔 (Peg-in-Hole Insertion)

经典的装配任务：将一个圆柱销插入一个孔中。

**任务分析**（假设孔沿 $z$ 轴方向）：

| 方向 | 控制类型 | 说明 |
|------|----------|------|
| $x$ | 运动控制 | 对准孔的 $x$ 位置 |
| $y$ | 运动控制 | 对准孔的 $y$ 位置 |
| $z$ | 力控制 | 沿插入方向施加恒定力 |
| $\theta_x$ | 运动控制 | 保持姿态对准 |
| $\theta_y$ | 运动控制 | 保持姿态对准 |
| $\theta_z$ | 力控制 | 绕轴自由旋转（零力矩） |

选择矩阵：

$$
\Sigma_M = \text{diag}(1, 1, 0, 1, 1, 0), \quad \Sigma_F = \text{diag}(0, 0, 1, 0, 0, 1)
$$

控制律：
- $x, y$ 方向：位置伺服，使销对准孔中心
- $z$ 方向：施加向下的恒定力 $F_{d,z}$（如 10 N）
- $\theta_x, \theta_y$：姿态伺服，保持销竖直
- $\theta_z$：零力矩，允许销绕轴自由旋转

> **Quiz 思考题**：如果插入过程中销与孔壁发生接触，$x$ 方向的运动控制器会产生什么效果？
>
> **答**：运动控制器会试图将销保持在孔中心位置。接触力会被运动控制器"抵抗"，产生恢复力使销回到中心。这正是混合运动/力控制的优势——运动方向保持精确定位，力方向保持柔顺。

### 8.5 示例：沿表面滑动棱柱体 (Sliding a Prism Along a Surface)

任务：让一个棱柱体沿着一个平面表面滑动。

**任务分析**（假设表面法线沿 $z$ 轴）：

| 方向 | 控制类型 | 说明 |
|------|----------|------|
| $x$ | 运动控制 | 沿表面滑动方向 1 |
| $y$ | 运动控制 | 沿表面滑动方向 2 |
| $z$ | 力控制 | 法线方向施加恒定接触力 |
| $\theta_x$ | 力控制 | 绕 $x$ 轴零力矩（适应表面） |
| $\theta_y$ | 力控制 | 绕 $y$ 轴零力矩（适应表面） |
| $\theta_z$ | 运动控制 | 控制绕法线的旋转 |

选择矩阵：

$$
\Sigma_M = \text{diag}(1, 1, 0, 0, 0, 1), \quad \Sigma_F = \text{diag}(0, 0, 1, 1, 1, 0)
$$

控制策略：
- $x, y$ 方向：按期望轨迹滑动
- $z$ 方向：施加恒定法向力 $F_{d,z}$，保持接触
- $\theta_x, \theta_y$：零力矩，让棱柱体自然适应表面倾斜
- $\theta_z$：控制棱柱体绕法线的朝向

> **Quiz 思考题**：如果表面不是完全平坦的（有轻微起伏），力控制方向会如何响应？
>
> **答**：$z$ 方向的力控制会自动调整末端执行器的高度以维持恒定的法向接触力。$\theta_x, \theta_y$ 方向的零力矩控制会让棱柱体自然倾斜以适应局部表面法线方向。这种柔顺行为正是力控制的核心优势。

---

## 9. 三种控制方法比较 (Comparison of Three Approaches)

### 9.1 经典逆动力学 (Classic Inverse Dynamics)

**控制律**：

$$
\boldsymbol{\tau} = M(\mathbf{q})\ddot{\mathbf{q}}^* + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})
$$

**虚拟输入**：

$$
\ddot{\mathbf{q}}^* = \ddot{\mathbf{q}}_d + K_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + K_p(\mathbf{q}_d - \mathbf{q})
$$

**特点**：
- 在关节空间中工作
- 完全补偿非线性动力学
- 闭环系统为线性解耦的双积分器
- 需要精确的动力学模型
- 不直接处理约束（关节极限、力矩限制等）

**适用场景**：关节空间轨迹跟踪，无接触任务。

### 9.2 操作空间控制 (Operational Space Control, OSC)

**控制律**：

$$
\boldsymbol{\tau}^* = J^T(\Lambda \ddot{\mathbf{w}}_{\text{des}} + \boldsymbol{\mu} + \mathbf{p}) + N^T \boldsymbol{\tau}_0
$$

**期望加速度**：

$$
\ddot{\mathbf{w}}_{\text{des}} = \ddot{\mathbf{x}}_d + K_d(\dot{\mathbf{x}}_d - \dot{\mathbf{x}}) + K_p(\mathbf{x}_d - \mathbf{x})
$$

**特点**：
- 直接在任务空间中工作
- 自然处理冗余自由度（零空间投影）
- 支持运动/力混合控制
- 需要计算任务空间动力学量 $\Lambda, \boldsymbol{\mu}, \mathbf{p}$
- 不直接处理不等式约束

**适用场景**：末端执行器轨迹跟踪，接触任务，冗余机器人。

### 9.3 基于 QP 优化的控制 (QP-Based Optimization)

**基本形式**：

$$
\min_{\ddot{\mathbf{q}}, \boldsymbol{\tau}} \quad \| \ddot{\mathbf{q}} - \ddot{\mathbf{q}}^* \|^2
$$

$$
\text{s.t.} \quad M(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + J_c^T \mathbf{F}_c
$$

$$
\boldsymbol{\tau}_{\min} \leq \boldsymbol{\tau} \leq \boldsymbol{\tau}_{\max}
$$

$$
\mathbf{F}_c \in \mathcal{F} \quad (\text{摩擦锥约束, Friction Cone Constraint})
$$

**扩展形式（多任务加权）**：

$$
\min_{\ddot{\mathbf{q}}, \boldsymbol{\tau}, \mathbf{F}_c} \quad \sum_{i} w_i \| J_i \ddot{\mathbf{q}} + \dot{J}_i \dot{\mathbf{q}} - \ddot{\mathbf{x}}_i^* \|^2 + w_\tau \| \boldsymbol{\tau} \|^2
$$

$$
\text{s.t.} \quad M\ddot{\mathbf{q}} + \mathbf{b} + \mathbf{g} = \boldsymbol{\tau} + J_c^T \mathbf{F}_c
$$

$$
\boldsymbol{\tau}_{\min} \leq \boldsymbol{\tau} \leq \boldsymbol{\tau}_{\max}
$$

$$
\ddot{q}_{\min} \leq \ddot{\mathbf{q}} \leq \ddot{q}_{\max}
$$

$$
\mathbf{F}_c \geq 0 \quad (\text{单边接触, Unilateral Contact})
$$

$$
|F_{c,t}| \leq \mu F_{c,n} \quad (\text{摩擦锥, Friction Cone})
$$

**特点**：
- 最灵活的框架
- 自然处理不等式约束（力矩限制、关节极限、摩擦锥等）
- 多任务优先级可通过权重或层次化 QP 实现
- 接触力作为优化变量，自动满足物理约束
- 计算量最大（需要实时求解 QP）
- 现代高效 QP 求解器（如 qpOASES, OSQP）使实时应用成为可能

**适用场景**：腿足机器人、全身运动控制 (Whole-Body Control)、复杂接触场景。

### 9.4 三种方法对比总结

| 特性 | Classic ID | OSC | QP 优化 |
|------|-----------|-----|---------|
| 工作空间 | 关节空间 | 任务空间 | 关节/任务空间 |
| 动力学补偿 | 完全 | 完全 | 完全 |
| 冗余处理 | 伪逆 + 零空间 | 零空间投影 | 优化目标/约束 |
| 不等式约束 | 不支持 | 不支持 | 原生支持 |
| 接触力处理 | 不直接支持 | 混合运动/力 | 优化变量 |
| 计算复杂度 | 低 | 中 | 高 |
| 实现难度 | 低 | 中 | 高 |
| 理论优雅性 | 高 | 高 | 中 |

> **Quiz 综合题**：一个四足机器人在不平坦地形上行走，需要同时满足：(a) 躯干高度和姿态控制，(b) 足端轨迹跟踪，(c) 关节力矩限制，(d) 地面接触力在摩擦锥内。应该选择哪种控制方法？为什么？
>
> **答**：应选择 QP 优化方法。原因：
> 1. 需要处理不等式约束（力矩限制、摩擦锥）→ Classic ID 和 OSC 无法直接处理
> 2. 多个任务需要同时优化 → QP 可以通过加权或层次化处理
> 3. 接触力需要满足物理约束（单边接触 $F_n \geq 0$，摩擦锥 $|F_t| \leq \mu F_n$）→ QP 将接触力作为优化变量
> 4. 现代四足机器人控制器（如 MIT Cheetah, ANYmal）均采用 QP 优化框架

---

## 10. 总结与关键要点

### 10.1 从简单到复杂的控制方法演进

```
位置控制 (PID)
  │  不关心动力学，高增益
  ▼
关节阻抗控制
  │  重力补偿，虚拟弹簧-阻尼器
  ▼
逆动力学控制
  │  完全补偿动力学，线性化解耦
  ▼
任务空间逆动力学
  │  直接在任务空间控制
  ▼
操作空间控制 (OSC)
  │  运动/力统一框架，零空间利用
  ▼
QP 优化控制
  │  约束处理，多任务优化
  ▼
全身运动控制 (Whole-Body Control)
     复杂系统的统一框架
```

### 10.2 核心公式速查

| 控制方法 | 核心公式 |
|----------|----------|
| 阻抗控制 | $\tau = K_p(q^*-q) + K_d(\dot{q}^*-\dot{q}) + \hat{g}(q)$ |
| 逆动力学 | $\tau = M\ddot{q}^* + b + g$ |
| 任务空间惯性 | $\Lambda = (JM^{-1}J^T)^{-1}$ |
| OSC | $\tau = J^T(\Lambda \ddot{w}_{\text{des}} + \mu + p) + N^T\tau_0$ |
| QP 优化 | $\min \|\ddot{q}-\ddot{q}^*\|^2$ s.t. $M\ddot{q}+b+g=\tau+J_c^TF_c$ |

### 10.3 设计选择指南

- **简单轨迹跟踪，无接触**：逆动力学控制即可
- **需要柔顺交互**：阻抗控制或 OSC
- **接触任务（装配、打磨等）**：OSC 混合运动/力控制
- **复杂约束（腿足、全身）**：QP 优化控制
- **模型不确定性大**：阻抗控制（鲁棒性好）+ 自适应项

---

> **参考文献**
>
> 1. Khatib, O. (1987). "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation." IEEE Journal of Robotics and Automation.
> 2. Siciliano, B., et al. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapters 8-9.
> 3. Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*. Springer.
> 4. Hutter, M., et al. (2014). "Quadrupedal Locomotion using Hierarchical Operational Space Control." International Journal of Robotics Research.
