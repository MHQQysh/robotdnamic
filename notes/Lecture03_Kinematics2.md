# Lecture 3: Kinematics 2 — 多体运动学与雅可比矩阵

> **课程**: ETH Zurich — Robot Dynamics  
> **主题**: Kinematics of Systems of Bodies; Jacobians  
> **关键词**: 多体运动学, 正微分运动学, 解析雅可比, 几何雅可比, 浮动基座

---

## 目录

1. [多体运动学简介](#1-多体运动学简介)
2. [固定基座 vs 浮动基座](#2-固定基座-vs-浮动基座)
3. [末端执行器配置参数](#3-末端执行器配置参数)
4. [正微分运动学](#4-正微分运动学)
5. [运动体中的速度](#5-运动体中的速度)
6. [几何雅可比矩阵推导](#6-几何雅可比矩阵推导)
7. [雅可比矩阵的重要性](#7-雅可比矩阵的重要性)
8. [浮动基座运动学](#8-浮动基座运动学)
9. [小测验与练习](#9-小测验与练习)

---

## 1. 多体运动学简介

### 1.1 为什么研究多体运动学？

在 Lecture 2 中，我们研究了**单个刚体**的运动学。然而，真实的机器人系统由**多个刚体**通过关节连接而成。多体运动学（Multi-body Kinematics）研究的核心问题是：

> **给定所有关节角度 $q$，如何确定机器人末端执行器（end-effector）的位置和姿态？**

### 1.2 工业机器人的精度要求

现代工业机器人的重复定位精度可达：

$$\text{Repeatability} \approx \pm 0.02 \, \text{mm}$$

实现如此高精度的关键因素：

| 因素 | 说明 |
|------|------|
| **刚性结构** (Rigid Structure) | 连杆变形极小，可忽略弹性效应 |
| **无间隙齿轮** (Backlash-free Gears) | 谐波减速器（Harmonic Drive）等消除齿轮间隙 |
| **高精度关节传感器** (High-precision Joint Sensors) | 编码器分辨率可达百万级 counts/rev |
| **精确运动学模型** | DH 参数标定后的运动学模型误差极小 |

### 1.3 基本假设

在本课程中，我们做以下假设：

- 所有连杆为**完美刚体**（perfectly rigid bodies）
- 关节为**理想关节**（ideal joints），无间隙、无摩擦
- 运动学模型**精确已知**（perfectly known kinematics）

> **注意**: 在实际应用中，需要通过运动学标定（kinematic calibration）来补偿制造误差。

---

## 2. 固定基座 vs 浮动基座

### 2.1 固定基座系统 (Fixed-base System)

固定基座机器人的基座固定在世界坐标系中，不发生运动。

**特征**：
- 基座坐标系 $\text{CS}_0$ 与世界坐标系（World Frame）重合或固定
- 所有自由度均为**驱动自由度**（actuated DOF）
- 广义坐标 $q \in \mathbb{R}^{n}$，其中 $n$ 为关节数

$$q = \begin{bmatrix} q_1 \ q_2 \ \vdots \ q_n \end{bmatrix} \in \mathbb{R}^n$$

**典型示例**：工业机械臂（如 ABB, KUKA, FANUC）

### 2.2 浮动基座系统 (Floating-base System)

浮动基座机器人的基座本身可以在空间中自由运动。

**特征**：
- 基座坐标系 $\text{CS}_B$ 相对于世界坐标系有 **6 个非驱动自由度**（unactuated DOF）
  - 3 个平移自由度：$_I r_B = \begin{bmatrix} x & y & z \end{bmatrix}^T$
  - 3 个旋转自由度：由旋转参数化表示（如欧拉角、四元数）
- 广义坐标包含基座位姿和关节角度

$$q = \begin{bmatrix} q_B \ q_j \end{bmatrix}, \quad q_B \in \mathbb{R}^{6+}, \quad q_j \in \mathbb{R}^{n_j}$$

**典型示例**：四足机器人、人形机器人、无人机

### 2.3 对比总结

```
固定基座 (Fixed Base)              浮动基座 (Floating Base)
┌─────────────────┐               ┌─────────────────┐
│   World Frame   │               │   World Frame   │
│   CS_0 (固定)   │               │   CS_I (惯性系) │
│       │         │               │       │         │
│       ▼         │               │   6 DOF (free)  │
│   Joint 1       │               │       ▼         │
│       │         │               │   CS_B (浮动)   │
│       ▼         │               │       │         │
│   Joint 2       │               │   Joint 1       │
│       │         │               │       │         │
│      ...        │               │      ...        │
│       ▼         │               │       ▼         │
│  End-effector   │               │  End-effector(s) │
└─────────────────┘               └─────────────────┘
  n 个驱动 DOF                     6 非驱动 + n_j 驱动 DOF
```

---

## 3. 末端执行器配置参数

### 3.1 操作空间描述

末端执行器的位姿（pose）由 $m_e$ 个参数完整描述，这些参数构成**操作空间坐标**（operational space coordinates）：

$$\chi_e = \begin{bmatrix} x_e \ \theta_e \end{bmatrix} \in \mathbb{R}^{m_e}$$

其中：
- $x_e$：末端执行器的**位置参数**（position parameters）
- $\theta_e$：末端执行器的**姿态参数**（orientation parameters）

### 3.2 正运动学映射

正运动学（Forward Kinematics）建立从关节空间到操作空间的映射：

$$\chi_e = f(q)$$

$$\begin{bmatrix} x_e \ \theta_e \end{bmatrix} = \begin{bmatrix} f_p(q) \ f_o(q) \end{bmatrix}$$

### 3.3 典型机器人示例

#### 示例 1: 一般 6-DOF 机器人臂

$$m_e = 6: \quad \chi_e = \begin{bmatrix} x \ y \ z \ \alpha \ \beta \ \gamma \end{bmatrix} \in \mathbb{R}^6$$

- 3 个位置参数 + 3 个姿态参数（如 ZYX 欧拉角）
- $n = 6$ 个关节 $\Rightarrow$ 方阵雅可比矩阵

#### 示例 2: SCARA 机器人

SCARA（Selective Compliance Assembly Robot Arm）：

$$m_e = 4: \quad \chi_e = \begin{bmatrix} x \ y \ z \ \theta_z \end{bmatrix} \in \mathbb{R}^4$$

- 平面内 2 个位置 + 垂直方向 1 个位置 + 绕 $z$ 轴 1 个旋转
- $n = 4$ 个关节（2 旋转 + 1 平移 + 1 旋转）

#### 示例 3: ANYpulator (4-DOF)

$$m_e = 4: \quad \chi_e = \begin{bmatrix} x \ y \ z \ \theta \end{bmatrix} \in \mathbb{R}^4$$

- 3 个位置参数 + 1 个姿态参数
- 适用于不需要完整姿态控制的任务

### 3.4 自由度与参数关系

| 关系 | 含义 |
|------|------|
| $n = m_e$ | **恰好约束**（fully determined），雅可比为方阵 |
| $n > m_e$ | **冗余**（redundant），无穷多逆运动学解 |
| $n < m_e$ | **欠约束**（under-determined），无法到达所有位姿 |

---

## 4. 正微分运动学

### 4.1 从位置到速度

正运动学给出位置映射 $\chi_e = f(q)$，对其求时间导数得到**正微分运动学**（Forward Differential Kinematics）：

$$\dot{\chi}_e = \frac{\partial f(q)}{\partial q} \dot{q}$$

这引出了**雅可比矩阵**（Jacobian Matrix）的概念——机器人学中最重要的数学工具之一。

### 4.2 解析雅可比矩阵 (Analytical Jacobian)

**定义**：解析雅可比矩阵 $J_A(q)$ 是操作空间坐标对关节坐标的偏导数矩阵：

$$\boxed{\dot{\chi}_e = J_A(q) \cdot \dot{q}}$$

$$J_A(q) = \frac{\partial f(q)}{\partial q} = \begin{bmatrix} \frac{\partial f_1}{\partial q_1} & \frac{\partial f_1}{\partial q_2} & \cdots & \frac{\partial f_1}{\partial q_n} \ \frac{\partial f_2}{\partial q_1} & \frac{\partial f_2}{\partial q_2} & \cdots & \frac{\partial f_2}{\partial q_n} \ \vdots & \vdots & \ddots & \vdots \ \frac{\partial f_{m_e}}{\partial q_1} & \frac{\partial f_{m_e}}{\partial q_2} & \cdots & \frac{\partial f_{m_e}}{\partial q_n} \end{bmatrix} \in \mathbb{R}^{m_e \times n}$$

**特点**：
- 依赖于姿态的**参数化方式**（如欧拉角、四元数等）
- 不同的参数化会导致不同的解析雅可比
- 可能存在**表示奇异性**（representation singularity），如欧拉角的万向锁（gimbal lock）

### 4.3 几何雅可比矩阵 (Geometric Jacobian)

**定义**：几何雅可比矩阵 $J(q)$ 直接将关节速度映射到末端执行器的**线速度**和**角速度**：

$$\boxed{\begin{bmatrix} _I v_E \ _I \omega_E \end{bmatrix} = J(q) \cdot \dot{q}}$$

$$J(q) = \begin{bmatrix} J_P(q) \ J_R(q) \end{bmatrix} \in \mathbb{R}^{6 \times n}$$

其中：
- $J_P(q) \in \mathbb{R}^{3 \times n}$：**位置雅可比**（Positional/Translational Jacobian）
- $J_R(q) \in \mathbb{R}^{3 \times n}$：**旋转雅可比**（Rotational Jacobian）
- $_I v_E \in \mathbb{R}^3$：末端执行器在惯性系中的线速度
- $_I \omega_E \in \mathbb{R}^3$：末端执行器在惯性系中的角速度

**特点**：
- **与参数化无关**（parameterization-independent）
- 具有明确的物理意义
- 在动力学中更常用

### 4.4 两者的区别与联系

解析雅可比和几何雅可比之间的关系：

$$\dot{\chi}_e = \begin{bmatrix} \dot{x}_e \ \dot{\theta}_e \end{bmatrix} = \begin{bmatrix} I & 0 \ 0 & E(\theta_e) \end{bmatrix}^{-1} \begin{bmatrix} _I v_E \ _I \omega_E \end{bmatrix}$$

其中 $E(\theta_e)$ 是将角速度 $\omega$ 映射到姿态参数导数 $\dot{\theta}_e$ 的矩阵：

$$_I \omega_E = E(\theta_e) \cdot \dot{\theta}_e$$

因此：

$$J_A(q) = \begin{bmatrix} I & 0 \ 0 & E^{-1}(\theta_e) \end{bmatrix} J(q)$$

> **重要**: 位置部分两者相同（$J_{A,P} = J_P$），区别仅在旋转部分。

**对比表**：

| 特性 | 解析雅可比 $J_A$ | 几何雅可比 $J$ |
|------|------------------|----------------|
| 定义方式 | $\partial f / \partial q$ | 速度传播 |
| 依赖参数化 | 是 | 否 |
| 物理意义 | 参数导数 | 线速度 + 角速度 |
| 奇异性 | 运动学奇异 + 表示奇异 | 仅运动学奇异 |
| 动力学中使用 | 较少 | 常用 |

---

## 5. 运动体中的速度

### 5.1 欧拉微分法则 (Euler Differentiation Rule)

对于在运动坐标系 $B$ 中观察的向量 $_B r$，其在惯性系 $I$ 中的时间导数为：

$$\boxed{\frac{^I d}{dt}(_{B} r) = \frac{^B d}{dt}(_{B} r) + {_I \omega_B} \times {_B r}}$$

用旋转矩阵表示：

$$\frac{d}{dt}(_I r) = \frac{d}{dt}(C_{IB} \cdot {_B r}) = \dot{C}_{IB} \cdot {_B r} + C_{IB} \cdot {_B \dot{r}}$$

其中 $\dot{C}_{IB} = \tilde{\omega}_{IB} \cdot C_{IB}$，$\tilde{\omega}$ 为角速度的反对称矩阵（skew-symmetric matrix）：

$$\tilde{\omega} = \begin{bmatrix} 0 & -\omega_z & \omega_y \ \omega_z & 0 & -\omega_x \ -\omega_y & \omega_x & 0 \end{bmatrix}$$

### 5.2 刚体上任意点的速度

考虑刚体 $B$，其上一点 $P$ 在惯性系中的位置为：

$$_I r_P = {_I r_B} + C_{IB} \cdot {_B r_{BP}}$$

对时间求导（注意 $_B r_{BP}$ 在体坐标系中为常数）：

$$_I v_P = {_I v_B} + {_I \omega_B} \times (C_{IB} \cdot {_B r_{BP}})$$

$$\boxed{_I v_P = {_I v_B} + {_I \omega_B} \times {_I r_{BP}}}$$

这就是经典的**刚体速度公式**（rigid body velocity formula）。

### 5.3 角速度的传播 (Angular Velocity Propagation)

对于串联机构中相邻连杆 $i-1$ 和 $i$，通过关节 $i$ 连接：

**旋转关节** (Revolute Joint)：

$$_I \omega_i = {_I \omega_{i-1}} + \dot{q}_i \cdot {_I n_i}$$

其中 $_I n_i$ 是关节 $i$ 的旋转轴在惯性系中的单位方向向量。

**平移关节** (Prismatic Joint)：

$$_I \omega_i = {_I \omega_{i-1}}$$

平移关节不改变角速度。

### 5.4 线速度的传播

**旋转关节**：

$$_I v_{O_i} = {_I v_{O_{i-1}}} + {_I \omega_i} \times {_I r_{O_{i-1} O_i}}$$

**平移关节**：

$$_I v_{O_i} = {_I v_{O_{i-1}}} + {_I \omega_i} \times {_I r_{O_{i-1} O_i}} + \dot{q}_i \cdot {_I n_i}$$

---

## 6. 几何雅可比矩阵推导

### 6.1 推导思路

几何雅可比矩阵的推导基于速度传播。对于 $n$ 个关节的串联机构，末端执行器的速度可以表示为各关节速度贡献的**线性叠加**：

$$\begin{bmatrix} _I v_E \ _I \omega_E \end{bmatrix} = \sum_{i=1}^{n} \begin{bmatrix} J_{P,i} \ J_{R,i} \end{bmatrix} \dot{q}_i = J(q) \cdot \dot{q}$$

### 6.2 位置雅可比矩阵 $J_P$

末端执行器的位置：

$$_I r_E = {_I r_{O_0}} + \sum_{k=1}^{n} {_I r_{O_{k-1} O_k}} + {_I r_{O_n E}}$$

对时间求导，利用速度传播公式，可以得到每个关节对末端线速度的贡献。

**旋转关节 $i$ 的贡献**：

$$J_{P,i} = {_I n_i} \times {_I r_{O_i E}}$$

其中：
- $_I n_i$：关节 $i$ 旋转轴的单位方向向量（在惯性系中）
- $_I r_{O_i E}$：从关节 $i$ 原点到末端执行器的位置向量

**平移关节 $i$ 的贡献**：

$$J_{P,i} = {_I n_i}$$

### 6.3 旋转雅可比矩阵 $J_R$

**旋转关节 $i$ 的贡献**：

$$J_{R,i} = {_I n_i}$$

**平移关节 $i$ 的贡献**：

$$J_{R,i} = 0$$

### 6.4 汇总公式

$$\boxed{J(q) = \begin{bmatrix} J_P \ J_R \end{bmatrix}, \quad J_i = \begin{cases} \begin{bmatrix} _I n_i \times {_I r_{O_i E}} \ _I n_i \end{bmatrix} & \text{旋转关节 (revolute)} \ \begin{bmatrix} _I n_i \ 0 \end{bmatrix} & \text{平移关节 (prismatic)} \end{cases}}$$

### 6.5 平面机器人臂示例 (Planar Robot Arm)

考虑一个平面 2-DOF 旋转机器人臂（RR manipulator）：

```
        q₂
    ┌───●───→ E (末端)
    │   l₂
    │
q₁  ●
    │   l₁
    │
   ═══ (基座)
```

**参数**：
- 连杆长度：$l_1, l_2$
- 关节角度：$q_1, q_2$
- 所有旋转轴沿 $z$ 方向：$n_1 = n_2 = e_z = \begin{bmatrix} 0 \ 0 \ 1 \end{bmatrix}$

**正运动学**：

$$x_E = l_1 \cos q_1 + l_2 \cos(q_1 + q_2)$$

$$y_E = l_1 \sin q_1 + l_2 \sin(q_1 + q_2)$$

**解析雅可比矩阵**（取 $m_e = 2$，仅考虑平面位置）：

$$J_A = \begin{bmatrix} \frac{\partial x_E}{\partial q_1} & \frac{\partial x_E}{\partial q_2} \ \frac{\partial y_E}{\partial q_1} & \frac{\partial y_E}{\partial q_2} \end{bmatrix}$$

$$\boxed{J_A = \begin{bmatrix} -l_1 \sin q_1 - l_2 \sin(q_1+q_2) & -l_2 \sin(q_1+q_2) \ l_1 \cos q_1 + l_2 \cos(q_1+q_2) & l_2 \cos(q_1+q_2) \end{bmatrix}}$$

**几何雅可比矩阵**（位置部分，使用叉积公式验证）：

关节 1（$O_1$ 在原点）：

$$J_{P,1} = n_1 \times r_{O_1 E} = \begin{bmatrix} 0 \ 0 \ 1 \end{bmatrix} \times \begin{bmatrix} l_1 c_1 + l_2 c_{12} \ l_1 s_1 + l_2 s_{12} \ 0 \end{bmatrix} = \begin{bmatrix} -(l_1 s_1 + l_2 s_{12}) \ l_1 c_1 + l_2 c_{12} \ 0 \end{bmatrix}$$

关节 2（$O_2$ 在 $(l_1 c_1, l_1 s_1, 0)$）：

$$J_{P,2} = n_2 \times r_{O_2 E} = \begin{bmatrix} 0 \ 0 \ 1 \end{bmatrix} \times \begin{bmatrix} l_2 c_{12} \ l_2 s_{12} \ 0 \end{bmatrix} = \begin{bmatrix} -l_2 s_{12} \ l_2 c_{12} \ 0 \end{bmatrix}$$

其中 $c_1 = \cos q_1$，$s_1 = \sin q_1$，$c_{12} = \cos(q_1+q_2)$，$s_{12} = \sin(q_1+q_2)$。

> **验证**: 几何雅可比的 $x, y$ 分量与解析雅可比完全一致。

**奇异性分析**：

$$\det(J_A) = l_1 l_2 \sin q_2$$

当 $q_2 = 0$ 或 $q_2 = \pi$ 时，$\det(J_A) = 0$，机器人处于**奇异构型**（singular configuration）：
- $q_2 = 0$：手臂完全伸展（fully extended）
- $q_2 = \pi$：手臂完全折叠（fully folded）

---

## 7. 雅可比矩阵的重要性

雅可比矩阵是机器人学中**最核心的数学工具**之一，贯穿运动学、动力学和控制的各个方面。

### 7.1 运动学映射 (Kinematic Mapping)

雅可比矩阵建立了关节空间与操作空间之间的**微分映射**：

$$\delta \chi_e = J(q) \cdot \delta q$$

这意味着：
- **正向**：已知关节速度 $\dot{q}$，可计算末端速度
- **逆向**：已知期望末端速度，可求解所需关节速度

### 7.2 逆运动学控制 (Inverse Kinematics Control)

给定期望的末端执行器速度 $\dot{\chi}_e^{des}$，求解关节速度：

**非冗余情况** ($n = m_e$，$J$ 为方阵)：

$$\dot{q} = J^{-1}(q) \cdot \dot{\chi}_e^{des}$$

前提是 $J$ 非奇异（$\det(J) \neq 0$）。

**冗余情况** ($n > m_e$)：

使用**伪逆**（pseudoinverse）：

$$\dot{q} = J^{\dagger}(q) \cdot \dot{\chi}_e^{des}$$

其中 Moore-Penrose 伪逆为：

$$J^{\dagger} = J^T (J J^T)^{-1}$$

这给出**最小范数解**（minimum norm solution），即所有满足约束的 $\dot{q}$ 中，$\|\dot{q}\|$ 最小的那个。

### 7.3 冗余问题 (Redundancy)

当 $n > m_e$ 时，系统具有**运动学冗余**（kinematic redundancy）。零空间（null space）维度为 $n - m_e$。

一般解为：

$$\boxed{\dot{q} = J^{\dagger} \dot{\chi}_e + (I - J^{\dagger} J) \dot{q}_0}$$

其中：
- $J^{\dagger} \dot{\chi}_e$：完成主任务的最小范数解
- $(I - J^{\dagger} J)$：零空间投影矩阵（null-space projector）
- $\dot{q}_0$：任意关节速度向量，投影到零空间后不影响末端运动

**零空间运动的应用**：
- 避障（obstacle avoidance）
- 关节极限回避（joint limit avoidance）
- 操作性优化（manipulability optimization）
- 能量最小化

### 7.4 接触约束 (Contact Constraints)

对于与环境接触的机器人（如行走机器人），接触点的速度为零：

$$_I v_{contact} = 0 \quad \Rightarrow \quad J_c(q) \cdot \dot{q} = 0$$

这构成了运动学约束，限制了系统的可行运动。

### 7.5 静力学：虚功原理 (Statics: Principle of Virtual Work)

雅可比矩阵的转置建立了**操作空间力**与**关节力矩**之间的映射：

$$\boxed{\tau = J^T(q) \cdot F}$$

其中：
- $\tau \in \mathbb{R}^n$：关节力矩/力向量
- $F \in \mathbb{R}^6$：末端执行器处的广义力（力 + 力矩）

**推导**（基于虚功原理）：

虚功在关节空间和操作空间中必须相等：

$$\delta W = \tau^T \delta q = F^T \delta \chi_e = F^T J \delta q$$

由于 $\delta q$ 任意，因此：

$$\tau^T = F^T J \quad \Rightarrow \quad \tau = J^T F$$

> **物理意义**: 末端执行器施加 1N 的力，通过 $J^T$ 可以计算每个关节需要提供多大的力矩来维持静力平衡。

### 7.6 雅可比矩阵的其他应用

| 应用领域 | 使用方式 |
|----------|----------|
| **操作性椭球** (Manipulability Ellipsoid) | $w = \sqrt{\det(J J^T)}$，衡量机器人在某构型下的灵活程度 |
| **力椭球** (Force Ellipsoid) | $(J J^T)^{-1}$，衡量末端力的传递能力 |
| **奇异性分析** | $\det(J) = 0$ 时为奇异构型 |
| **阻抗控制** (Impedance Control) | 通过 $J$ 在操作空间实现柔顺控制 |
| **动力学方程** | 操作空间动力学：$\Lambda \ddot{x} + \mu + p = F$ |

---

## 8. 浮动基座运动学

### 8.1 广义坐标和速度

对于浮动基座系统，广义坐标和广义速度的定义需要特别注意。

**广义坐标** (Generalized Coordinates)：

$$q = \begin{bmatrix} _I r_B \ \phi_B \ q_j \end{bmatrix} \in \mathbb{R}^{n_q}$$

其中：
- $_I r_B \in \mathbb{R}^3$：基座在惯性系中的位置
- $\phi_B$：基座的姿态参数（如欧拉角 $\in \mathbb{R}^3$ 或四元数 $\in \mathbb{R}^4$）
- $q_j \in \mathbb{R}^{n_j}$：关节角度

**广义速度** (Generalized Velocities)：

$$u = \begin{bmatrix} _I v_B \ _I \omega_B \ \dot{q}_j \end{bmatrix} \in \mathbb{R}^{n_u}$$

> **注意**: 当使用四元数时，$n_q = 7 + n_j$ 但 $n_u = 6 + n_j$，即 $n_q \neq n_u$。这是因为四元数有 4 个参数但只有 3 个旋转自由度（受单位约束 $\|q\| = 1$）。

### 8.2 任意点的位置和速度

机器人上任意一点 $P$（位于连杆 $k$ 上）的位置：

$$_I r_P = {_I r_B} + C_{IB} \cdot {_B r_{BP}(q_j)}$$

其速度：

$$_I v_P = {_I v_B} + {_I \omega_B} \times {_I r_{BP}} + J_{P,j}(q) \cdot \dot{q}_j$$

写成矩阵形式：

$$_I v_P = \underbrace{\begin{bmatrix} I & -\widetilde{_I r_{BP}} & J_{P,j} \end{bmatrix}}_{J_P(q)} \cdot \underbrace{\begin{bmatrix} _I v_B \ _I \omega_B \ \dot{q}_j \end{bmatrix}}_{u}$$

$$\boxed{_I v_P = J_P(q) \cdot u}$$

其中 $\widetilde{_I r_{BP}}$ 是 $_I r_{BP}$ 的反对称矩阵。

### 8.3 接触约束

#### 基本约束方程

当机器人的某个点（如足端）与地面保持接触时，该点的速度为零：

$$\boxed{J_c(q) \cdot u = 0}$$

其中 $J_c$ 是**接触雅可比矩阵**（Contact Jacobian）。

#### 接触雅可比矩阵的性质

对于 $k$ 个接触点，每个接触点提供 3 个约束（假设无滑动）：

$$J_c = \begin{bmatrix} J_{c,1} \ J_{c,2} \ \vdots \ J_{c,k} \end{bmatrix} \in \mathbb{R}^{3k \times n_u}$$

约束方程 $J_c u = 0$ 意味着广义速度 $u$ 必须位于 $J_c$ 的**零空间**中：

$$u \in \mathcal{N}(J_c)$$

系统的有效自由度为：

$$\text{DOF}_{eff} = n_u - \text{rank}(J_c)$$

### 8.4 四足机器人示例 (Quadruped Robot)

以 ANYmal 四足机器人为例：

```
          前左 (LF)          前右 (RF)
            ●──HAA──HFE──KFE──●
           /                    \
    ┌─────┤      基座 (Body)     ├─────┐
    │      \                    /      │
            ●──HAA──HFE──KFE──●
          后左 (LH)          后右 (RH)
```

**自由度分析**：

| 组成部分 | 自由度 |
|----------|--------|
| 基座（浮动）| 6 DOF（3 平移 + 3 旋转），非驱动 |
| 每条腿 | 3 DOF（HAA + HFE + KFE），驱动 |
| 4 条腿合计 | $4 \times 3 = 12$ DOF，驱动 |
| **总计** | **18 DOF**（$n_u = 18$）|

其中：
- HAA: Hip Abduction/Adduction（髋关节外展/内收）
- HFE: Hip Flexion/Extension（髋关节屈/伸）
- KFE: Knee Flexion/Extension（膝关节屈/伸）

**广义速度**：

$$u = \begin{bmatrix} _I v_B \ _I \omega_B \ \dot{q}_{LF} \ \dot{q}_{RF} \ \dot{q}_{LH} \ \dot{q}_{RH} \end{bmatrix} \in \mathbb{R}^{18}$$

**四足站立时的接触约束**：

4 个足端接触地面，每个提供 3 个约束：

$$J_c \in \mathbb{R}^{12 \times 18}, \quad J_c \cdot u = 0$$

有效自由度：

$$\text{DOF}_{eff} = 18 - 12 = 6$$

> **物理意义**: 四足机器人站立时，虽然有 18 个广义速度，但接触约束消除了 12 个自由度，剩余 6 个自由度恰好对应基座的 6 DOF 运动（可以通过调整腿的姿态来移动身体）。

**行走时（一条腿抬起）**：

$$J_c \in \mathbb{R}^{9 \times 18}, \quad \text{DOF}_{eff} = 18 - 9 = 9$$

多出的 3 个自由度对应摆动腿（swing leg）的运动。

---

## 9. 小测验与练习

### Quiz 1: 基本概念

**Q1.1**: 一个 7-DOF 机器人臂在三维空间中控制末端执行器的完整位姿（位置 + 姿态），该系统是冗余的还是恰好约束的？

<details>
<summary>点击查看答案</summary>

**冗余系统**。末端执行器完整位姿需要 $m_e = 6$ 个参数（3 位置 + 3 姿态），而关节数 $n = 7 > m_e = 6$，冗余度为 $7 - 6 = 1$。零空间维度为 1，意味着存在一个"自运动"（self-motion）自由度，可以在不改变末端位姿的情况下改变机器人构型。

</details>

---

**Q1.2**: 浮动基座四足机器人有 12 个驱动关节。当四条腿都接触地面时，系统的有效自由度是多少？

<details>
<summary>点击查看答案</summary>

广义速度维度：$n_u = 6 + 12 = 18$（6 个基座自由度 + 12 个关节自由度）。

四足接触约束：$4 \times 3 = 12$ 个约束（假设每个足端提供 3 个约束，无滑动）。

有效自由度：$\text{DOF}_{eff} = 18 - 12 = 6$。

这 6 个有效自由度对应基座的 6 DOF 运动。机器人可以通过协调四条腿的关节运动来实现身体的平移和旋转。

</details>

---

**Q1.3**: 固定基座和浮动基座系统的根本区别是什么？

<details>
<summary>点击查看答案</summary>

根本区别在于基座是否有**非驱动自由度**：

- **固定基座**：基座坐标系 $\text{CS}_0$ 固定，所有自由度均为驱动自由度，$q \in \mathbb{R}^n$
- **浮动基座**：基座坐标系 $\text{CS}_B$ 自由运动，有 6 个非驱动自由度（3 平移 + 3 旋转），广义坐标包含基座位姿参数

浮动基座系统的运动学和动力学分析更为复杂，因为需要额外处理基座的运动以及接触约束。

</details>

---

### Quiz 2: 雅可比矩阵计算

**Q2.1**: 对于平面 2R 机器人臂（$l_1 = 1\text{m}, l_2 = 0.5\text{m}$），当 $q_1 = \pi/4, q_2 = \pi/2$ 时：

(a) 计算末端执行器位置 $(x_E, y_E)$

(b) 计算解析雅可比矩阵 $J_A$

(c) 判断该构型是否为奇异构型

<details>
<summary>点击查看答案</summary>

**(a) 末端位置**：

$$x_E = l_1 \cos q_1 + l_2 \cos(q_1 + q_2) = 1 \cdot \cos\frac{\pi}{4} + 0.5 \cdot \cos\frac{3\pi}{4}$$

$$= \frac{\sqrt{2}}{2} - \frac{\sqrt{2}}{4} = \frac{\sqrt{2}}{4} \approx 0.354 \text{m}$$

$$y_E = l_1 \sin q_1 + l_2 \sin(q_1 + q_2) = 1 \cdot \sin\frac{\pi}{4} + 0.5 \cdot \sin\frac{3\pi}{4}$$

$$= \frac{\sqrt{2}}{2} + \frac{\sqrt{2}}{4} = \frac{3\sqrt{2}}{4} \approx 1.061 \text{m}$$

**(b) 解析雅可比矩阵**：

$$J_A = \begin{bmatrix} -l_1 s_1 - l_2 s_{12} & -l_2 s_{12} \ l_1 c_1 + l_2 c_{12} & l_2 c_{12} \end{bmatrix}$$

代入数值（$s_1 = \frac{\sqrt{2}}{2}$，$c_1 = \frac{\sqrt{2}}{2}$，$s_{12} = \sin\frac{3\pi}{4} = \frac{\sqrt{2}}{2}$，$c_{12} = \cos\frac{3\pi}{4} = -\frac{\sqrt{2}}{2}$）：

$$J_A = \begin{bmatrix} -\frac{\sqrt{2}}{2} - \frac{\sqrt{2}}{4} & -\frac{\sqrt{2}}{4} \ \frac{\sqrt{2}}{2} - \frac{\sqrt{2}}{4} & -\frac{\sqrt{2}}{4} \end{bmatrix} = \begin{bmatrix} -\frac{3\sqrt{2}}{4} & -\frac{\sqrt{2}}{4} \ \frac{\sqrt{2}}{4} & -\frac{\sqrt{2}}{4} \end{bmatrix}$$

$$\approx \begin{bmatrix} -1.061 & -0.354 \ 0.354 & -0.354 \end{bmatrix}$$

**(c) 奇异性判断**：

$$\det(J_A) = l_1 l_2 \sin q_2 = 1 \times 0.5 \times \sin\frac{\pi}{2} = 0.5 \neq 0$$

**不是奇异构型**。$q_2 = \pi/2$ 时手臂呈直角弯曲，具有良好的操作性。

</details>

---

**Q2.2**: 解释为什么几何雅可比矩阵比解析雅可比矩阵在动力学中更常用。

<details>
<summary>点击查看答案</summary>

几何雅可比矩阵在动力学中更常用的原因：

1. **与参数化无关**：几何雅可比直接映射到物理量（线速度和角速度），不依赖于姿态的参数化方式。而解析雅可比依赖于所选的姿态表示（欧拉角、四元数等）。

2. **无表示奇异性**：解析雅可比在某些姿态参数化下会出现额外的奇异性（如欧拉角的万向锁），而几何雅可比只有运动学奇异性。

3. **动力学方程的自然形式**：牛顿-欧拉方程直接使用线速度和角速度，与几何雅可比的输出一致。

4. **虚功原理**：$\tau = J^T F$ 中的 $J$ 是几何雅可比，$F$ 是物理力和力矩。

5. **能量一致性**：动能表达式 $T = \frac{1}{2} v^T M v$ 中的速度是物理速度，与几何雅可比对应。

</details>

---

### Quiz 3: 虚功原理与静力学

**Q3.1**: 一个 3-DOF 平面机器人臂的末端执行器需要在 $x$ 方向施加 $F_x = 10\text{N}$ 的力。已知当前构型下的位置雅可比矩阵为：

$$J_P = \begin{bmatrix} -0.5 & -0.3 & -0.1 \ 0.8 & 0.4 & 0.2 \end{bmatrix}$$

求所需的关节力矩 $\tau$。

<details>
<summary>点击查看答案</summary>

根据虚功原理 $\tau = J_P^T F$：

$$\tau = J_P^T \begin{bmatrix} 10 \ 0 \end{bmatrix} = \begin{bmatrix} -0.5 & 0.8 \ -0.3 & 0.4 \ -0.1 & 0.2 \end{bmatrix} \begin{bmatrix} 10 \ 0 \end{bmatrix} = \begin{bmatrix} -5.0 \ -3.0 \ -1.0 \end{bmatrix} \text{Nm}$$

**物理解释**：为了在末端 $x$ 方向施加 10N 的正向力，三个关节都需要提供负方向的力矩。这是因为雅可比矩阵第一行（对应 $x$ 方向）的所有元素都是负的，说明正方向的关节运动会导致末端在 $x$ 负方向运动。

</details>

---

### Quiz 4: 冗余与零空间

**Q4.1**: 一个 7-DOF 机器人臂控制末端的 3D 位置（$m_e = 3$）。零空间的维度是多少？这意味着什么？

<details>
<summary>点击查看答案</summary>

零空间维度 = $n - m_e = 7 - 3 = 4$（假设雅可比满秩）。

这意味着存在一个 4 维的关节速度子空间，在该子空间中的任何运动都**不会改变末端执行器的位置**。机器人可以利用这 4 个冗余自由度来：

- 避开障碍物
- 远离关节极限
- 优化某个性能指标（如操作性）
- 同时完成次要任务

一般解的形式为：

$$\dot{q} = J^{\dagger} \dot{x}_{des} + (I - J^{\dagger} J) \dot{q}_0$$

其中 $(I - J^{\dagger} J) \dot{q}_0$ 是零空间中的运动分量。

</details>

---

### Quiz 5: 综合应用

**Q5.1**: 考虑一个浮动基座的双臂机器人，每条手臂有 7 个关节，双脚固定在地面上（每只脚提供 6 个约束，即完全固定）。

(a) 广义速度的维度 $n_u$ 是多少？

(b) 接触约束的数量是多少？

(c) 有效自由度是多少？

(d) 如果要用双手各控制一个 6-DOF 的末端位姿，系统是冗余的吗？

<details>
<summary>点击查看答案</summary>

**(a)** $n_u = 6 \text{(基座)} + 7 \times 2 \text{(双臂)} + n_{legs} \text{(腿部关节)}$

假设每条腿有 6 个关节：$n_u = 6 + 14 + 12 = 32$

**(b)** 双脚固定，每只脚 6 个约束（3 平移 + 3 旋转）：$n_c = 12$

**(c)** $\text{DOF}_{eff} = 32 - 12 = 20$

但腿部的 12 个关节自由度被 12 个约束完全消耗，所以有效的"可用"自由度为双臂的 14 个。

**(d)** 双手各控制 6-DOF 位姿，共需 $m_e = 12$ 个约束。可用自由度为 14，因此冗余度为 $14 - 12 = 2$。**系统是冗余的**，有 2 个冗余自由度可用于次要任务优化。

</details>

---

## 附录：关键公式速查表

| 公式 | 表达式 | 说明 |
|------|--------|------|
| 正运动学 | $\chi_e = f(q)$ | 关节角到末端位姿 |
| 解析雅可比 | $\dot{\chi}_e = J_A(q) \dot{q}$ | 依赖参数化 |
| 几何雅可比 | $\begin{bmatrix} v \ \omega \end{bmatrix} = J(q) \dot{q}$ | 与参数化无关 |
| 旋转关节列 | $J_i = \begin{bmatrix} n_i \times r_{O_i E} \ n_i \end{bmatrix}$ | 叉积公式 |
| 平移关节列 | $J_i = \begin{bmatrix} n_i \ 0 \end{bmatrix}$ | 仅影响线速度 |
| 虚功原理 | $\tau = J^T F$ | 力/力矩映射 |
| 冗余逆运动学 | $\dot{q} = J^{\dagger} \dot{x} + (I - J^{\dagger}J)\dot{q}_0$ | 零空间优化 |
| 接触约束 | $J_c u = 0$ | 接触点零速度 |
| 刚体速度 | $v_P = v_B + \omega \times r_{BP}$ | 速度传播 |
| 角速度传播 | $\omega_i = \omega_{i-1} + \dot{q}_i n_i$ | 旋转关节 |

---

> **下一讲预告**: Lecture 4 将介绍**逆运动学**（Inverse Kinematics）的数值方法，包括牛顿-拉夫森法、阻尼最小二乘法（Damped Least Squares），以及任务优先级框架（Task Priority Framework）。
