# Lecture 5: Dynamics 1 — 多体动力学 (Multi-body Dynamics)

> ETH Zurich · Robot Dynamics 课程笔记

---

## 目录

1. [动力学概述 (Dynamics Overview)](#1-动力学概述-dynamics-overview)
2. [虚功原理 (Principle of Virtual Work)](#2-虚功原理-principle-of-virtual-work)
3. [单刚体的虚位移 (Virtual Displacements of Single Rigid Bodies)](#3-单刚体的虚位移-virtual-displacements-of-single-rigid-bodies)
4. [冲量和角动量 (Impulse and Angular Momentum)](#4-冲量和角动量-impulse-and-angular-momentum)
5. [牛顿-欧拉法 (Newton-Euler Method)](#5-第一种方法-牛顿-欧拉法-newton-euler-for-single-bodies)
6. [投影牛顿-欧拉法 (Projected Newton-Euler)](#6-第二种方法-投影牛顿-欧拉法-projected-newton-euler)
7. [拉格朗日方程 (Lagrange Equations)](#7-第三种方法-拉格朗日方程-lagrange-equations)
8. [外力 (External Forces)](#8-外力-external-forces)

---

## 1. 动力学概述 (Dynamics Overview)

### 1.1 什么是动力学?

运动学 (Kinematics) 描述的是"运动是什么样的"，而动力学 (Dynamics) 描述的是"运动的原因"。

- **输入**: 力 (Forces) / 扭矩 (Torques)
- **输出**: 系统的运动 (Motion of the system)

动力学的核心任务是建立力与运动之间的数学关系，即**运动方程 (Equations of Motion, EoM)**。

### 1.2 运动方程的一般形式

对于一个具有广义坐标 $q \in \mathbb{R}^n$ 的多体系统，运动方程为:

$$
M(q)\ddot{q} + b(q, \dot{q}) + g(q) = \tau + J_c^T F_c
$$

各项含义如下:

| 符号 | 名称 | 说明 |
|------|------|------|
| $M(q) \in \mathbb{R}^{n \times n}$ | 质量矩阵 (Mass/Inertia Matrix) | 对称正定矩阵，描述系统的惯性特性 |
| $b(q, \dot{q}) \in \mathbb{R}^n$ | 离心力和科里奥利力 (Centrifugal & Coriolis) | 与速度相关的非线性力项 |
| $g(q) \in \mathbb{R}^n$ | 重力项 (Gravity) | 由重力引起的广义力 |
| $\tau \in \mathbb{R}^n$ | 广义力 (Generalized Forces) | 执行器施加的力/扭矩 |
| $F_c$ | 外部接触力 (External Contact Forces) | 环境施加的力 |
| $J_c$ | 接触雅可比 (Contact Jacobian) | 将接触力映射到广义坐标空间 |

### 1.3 运动方程的性质

- $M(q)$ 是**对称正定**的: $M = M^T$, $M > 0$
- 等式左边完全由系统本身的物理特性决定
- 等式右边是外部输入（执行器力和接触力）
- 该方程是关于 $\ddot{q}$ 的二阶常微分方程 (ODE)

### 1.4 正向动力学与逆向动力学

- **正向动力学 (Forward Dynamics)**: 已知 $\tau$，求 $\ddot{q} = M^{-1}(\tau - b - g)$
  - 用于仿真 (Simulation)
- **逆向动力学 (Inverse Dynamics)**: 已知期望的 $\ddot{q}$，求所需的 $\tau$
  - 用于控制 (Control)

---

## 2. 虚功原理 (Principle of Virtual Work)

### 2.1 达朗贝尔原理 (D'Alembert's Principle)

虚功原理是分析力学的基础。其核心思想是:

> 在动态平衡状态下，所有作用力（包括惯性力）在任意虚位移上所做的虚功之和为零。

数学表达:

$$
\delta W = \sum_i (F_i - m_i \ddot{r}_i) \cdot \delta r_i = 0
$$

其中 $\delta r_i$ 是与约束相容的**虚位移 (virtual displacement)**。

关键概念:
- **虚位移**: 在时间冻结的瞬间，系统在约束允许范围内的无穷小位移
- 虚位移不是实际发生的位移，而是"假想的"、"可能的"位移
- 虚位移必须与系统的约束条件相容

### 2.2 牛顿定律回顾

**牛顿第二定律 (Newton's Second Law)** — 线性动量 (Linear Momentum):

$$
\frac{d\mathbf{p}}{dt} = \mathbf{F}
$$

其中线性动量 (impulse/linear momentum) 定义为:

$$
\mathbf{p} = m \mathbf{v}_S
$$

- $m$: 刚体质量
- $\mathbf{v}_S$: 质心 (Center of Mass, CoM) 的速度

**欧拉方程 (Euler's Equation)** — 角动量 (Angular Momentum):

$$
\frac{d\mathbf{N}_S}{dt} = \boldsymbol{\Gamma}_S
$$

其中角动量定义为:

$$
\mathbf{N}_S = \boldsymbol{\Theta}_S \boldsymbol{\omega}
$$

- $\boldsymbol{\Theta}_S \in \mathbb{R}^{3 \times 3}$: 关于质心的惯性张量 (Inertia Tensor)
- $\boldsymbol{\omega}$: 角速度 (Angular Velocity)
- $\boldsymbol{\Gamma}_S$: 关于质心的外力矩之和

### 2.3 惯性张量 (Inertia Tensor)

惯性张量是一个 $3 \times 3$ 对称矩阵:

$$
\boldsymbol{\Theta}_S = \begin{pmatrix} I_{xx} & -I_{xy} & -I_{xz} \ -I_{xy} & I_{yy} & -I_{yz} \ -I_{xz} & -I_{yz} & I_{zz} \end{pmatrix}
$$

其中:
- 对角元素 $I_{xx}, I_{yy}, I_{zz}$ 为**转动惯量 (Moments of Inertia)**
- 非对角元素 $I_{xy}, I_{xz}, I_{yz}$ 为**惯性积 (Products of Inertia)**

惯性张量依赖于参考坐标系的选取。通过**平行轴定理 (Parallel Axis Theorem / Steiner's Theorem)** 可以在不同参考点之间转换:

$$
\boldsymbol{\Theta}_A = \boldsymbol{\Theta}_S + m \left( \mathbf{r}^T \mathbf{r} \cdot \mathbf{I}_{3 \times 3} - \mathbf{r} \mathbf{r}^T \right)
$$

其中 $\mathbf{r}$ 是从质心 $S$ 到新参考点 $A$ 的向量。

---

## 3. 单刚体的虚位移 (Virtual Displacements of Single Rigid Bodies)

### 3.1 刚体的运动学描述

一个刚体在三维空间中有 6 个自由度 (Degrees of Freedom, DoF):
- 3 个平移自由度 (Translation)
- 3 个旋转自由度 (Rotation)

刚体上任意一点 $P$ 的位置可以表示为:

$$
\mathbf{r}_P = \mathbf{r}_S + C \cdot \mathbf{r}_{SP,\text{body}}
$$

其中:
- $\mathbf{r}_S$: 质心在世界坐标系中的位置
- $C$: 从体坐标系到世界坐标系的旋转矩阵
- $\mathbf{r}_{SP,\text{body}}$: 点 $P$ 在体坐标系中相对于质心的位置（常量）

### 3.2 速度关系

对位置求时间导数，得到速度:

$$
\dot{\mathbf{r}}_P = \dot{\mathbf{r}}_S + \boldsymbol{\omega} \times (C \cdot \mathbf{r}_{SP,\text{body}})
$$

即:

$$
\dot{\mathbf{r}}_P = \dot{\mathbf{r}}_S + \boldsymbol{\omega} \times \mathbf{r}_{SP}
$$

### 3.3 虚位移的定义

虚位移是速度关系在"冻结时间"下的无穷小版本:

$$
\delta \mathbf{r}_P = \delta \mathbf{r}_S + \delta \boldsymbol{\phi} \times \mathbf{r}_{SP}
$$

其中:
- $\delta \mathbf{r}_S$: 质心的虚平移
- $\delta \boldsymbol{\phi}$: 虚旋转（无穷小旋转向量）

### 3.4 用广义坐标表示虚位移

如果系统的广义坐标为 $q$，则:

$$
\delta \mathbf{r}_P = \frac{\partial \mathbf{r}_P}{\partial q} \delta q = J_P \, \delta q
$$

$$
\delta \boldsymbol{\phi} = \frac{\partial \boldsymbol{\phi}}{\partial q} \delta q = J_R \, \delta q
$$

其中 $J_P$ 和 $J_R$ 分别是平移雅可比和旋转雅可比。

---

## 4. 冲量和角动量 (Impulse and Angular Momentum)

### 4.1 外力和力矩

作用在刚体上的外力和力矩包括:
- **重力**: $\mathbf{F}_g = m\mathbf{g}$，作用于质心
- **接触力**: $\mathbf{F}_c$，作用于接触点
- **执行器力/扭矩**: $\tau$，作用于关节

### 4.2 线性动量 (Linear Momentum / Impulse)

线性动量的时间变化率等于合外力:

$$
\dot{\mathbf{p}} = \frac{d}{dt}(m \mathbf{v}_S) = m \dot{\mathbf{v}}_S = \sum \mathbf{F}_{\text{ext}}
$$

对于质量不变的刚体:

$$
m \ddot{\mathbf{r}}_S = \sum \mathbf{F}_{\text{ext}}
$$

### 4.3 角动量 (Angular Momentum)

关于质心的角动量变化率等于合外力矩:

$$
\dot{\mathbf{N}}_S = \frac{d}{dt}(\boldsymbol{\Theta}_S \boldsymbol{\omega})
$$

在体坐标系 (Body Frame) 中表达时，需要注意惯性张量的时间导数:

$$
\dot{\mathbf{N}}_S = \boldsymbol{\Theta}_S \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\boldsymbol{\Theta}_S \boldsymbol{\omega}) = \sum \boldsymbol{\Gamma}_{\text{ext}}
$$

其中 $\boldsymbol{\omega} \times (\boldsymbol{\Theta}_S \boldsymbol{\omega})$ 项就是**陀螺效应 (Gyroscopic Effect)**，这正是科里奥利力和离心力的来源。

### 4.4 守恒定律

- 若合外力为零: $\sum \mathbf{F}_{\text{ext}} = 0 \Rightarrow \mathbf{p} = \text{const}$（线性动量守恒）
- 若合外力矩为零: $\sum \boldsymbol{\Gamma}_{\text{ext}} = 0 \Rightarrow \mathbf{N}_S = \text{const}$（角动量守恒）

---

## 5. 第一种方法: 牛顿-欧拉法 (Newton-Euler for Single Bodies)

### 5.1 方法概述

牛顿-欧拉法是最直观的多体动力学建模方法。其基本思路:

1. **切断所有刚体** (Free-body diagram): 将多体系统拆解为独立的刚体
2. **引入约束力** (Constraint forces): 在每个切断的关节处引入未知的约束力和约束力矩
3. **对每个刚体应用牛顿-欧拉方程**: 分别写出线性动量守恒 ($\dot{\mathbf{p}} = \mathbf{F}$) 和角动量守恒 ($\dot{\mathbf{N}} = \boldsymbol{\Gamma}$)
4. **消除约束力**: 联立方程，消去内部约束力，得到最终的运动方程

### 5.2 方程数量分析

对于一个由 $n$ 个刚体组成的系统:

- 每个刚体有 6 个方程（3 个平移 + 3 个旋转）
- 总共 $6n$ 个方程
- 需要消除的约束力数量: 取决于关节类型
  - 旋转关节 (Revolute Joint): 引入 5 个约束力（只允许 1 个旋转自由度）
  - 棱柱关节 (Prismatic Joint): 引入 5 个约束力（只允许 1 个平移自由度）
- 对于 $n$ 个刚体通过旋转关节串联: 需要消除 $5n$ 个约束力（假设有 $n$ 个关节）

### 5.3 优缺点

**优点:**
- 物理直觉清晰，每一步都有明确的物理意义
- 可以直接获得**约束力** (Constraint Forces)，这在结构设计和安全分析中非常有用
- 适合简单系统的手动推导

**缺点:**
- 对于大型多体系统，方程数量爆炸式增长
- 消除约束力的代数运算极其繁琐
- 组合问题 (Combinatorial problem) 巨大: $6n$ 个方程中消除 $5n$ 个未知数

### 5.4 小车摆示例 (Cart-Pole Example)

考虑经典的小车摆 (Cart-Pole) 系统:

```
        O ← 铰链 (pivot)
       /
      / L (摆杆长度)
     /
    ● ← 摆锤 (mass m_p)
    
 [████████] ← 小车 (mass m_c)
 ──────────────── ← 轨道
```

**广义坐标**: $q = \begin{pmatrix} x \ \theta \end{pmatrix}$

- $x$: 小车沿轨道的位移
- $\theta$: 摆杆与竖直方向的夹角

**Step 1: 切断系统，画自由体图**

对小车 (Cart):
- 外力: 驱动力 $F$，重力 $m_c g$，法向力 $N$
- 约束力: 铰链处的力 $F_{cx}, F_{cy}$（来自摆杆）

对摆杆 (Pendulum):
- 外力: 重力 $m_p g$
- 约束力: 铰链处的力 $-F_{cx}, -F_{cy}$（牛顿第三定律）

**Step 2: 摆锤的位置**

$$
x_p = x + L \sin\theta
$$
$$
y_p = -L \cos\theta
$$

加速度:

$$
\ddot{x}_p = \ddot{x} + L\ddot{\theta}\cos\theta - L\dot{\theta}^2\sin\theta
$$
$$
\ddot{y}_p = L\ddot{\theta}\sin\theta + L\dot{\theta}^2\cos\theta
$$

**Step 3: 对每个刚体写牛顿方程**

小车 ($x$ 方向):

$$
m_c \ddot{x} = F - F_{cx}
$$

摆锤 ($x$ 方向):

$$
m_p \ddot{x}_p = F_{cx}
$$

摆锤 ($y$ 方向):

$$
m_p \ddot{y}_p = F_{cy} - m_p g
$$

摆锤绕铰链的角动量:

$$
m_p L^2 \ddot{\theta} = -m_p g L \sin\theta - F_{cx} L \cos\theta \cdot 0 + \text{...}
$$

**Step 4: 消除约束力 $F_{cx}, F_{cy}$**

将摆锤的 $x$ 方程代入小车方程:

$$
m_c \ddot{x} = F - m_p(\ddot{x} + L\ddot{\theta}\cos\theta - L\dot{\theta}^2\sin\theta)
$$

整理得:

$$
(m_c + m_p)\ddot{x} + m_p L \ddot{\theta}\cos\theta - m_p L \dot{\theta}^2 \sin\theta = F
$$

对摆锤绕铰链取矩:

$$
m_p L \ddot{x}\cos\theta + m_p L^2 \ddot{\theta} - m_p g L \sin\theta = 0
$$

**最终运动方程 (矩阵形式):**

$$
\underbrace{\begin{pmatrix} m_c + m_p & m_p L \cos\theta \ m_p L \cos\theta & m_p L^2 \end{pmatrix}}_{M(q)} \begin{pmatrix} \ddot{x} \ \ddot{\theta} \end{pmatrix} + \underbrace{\begin{pmatrix} -m_p L \dot{\theta}^2 \sin\theta \ 0 \end{pmatrix}}_{b(q,\dot{q})} + \underbrace{\begin{pmatrix} 0 \ -m_p g L \sin\theta \end{pmatrix}}_{g(q)} = \begin{pmatrix} F \ 0 \end{pmatrix}
$$

> **Quiz 思考**: 验证质量矩阵 $M(q)$ 是否对称正定? 是的，$M = M^T$，且行列式 $= m_c m_p L^2 + m_p^2 L^2 - m_p^2 L^2 \cos^2\theta = m_p L^2(m_c + m_p \sin^2\theta) > 0$。

---

## 6. 第二种方法: 投影牛顿-欧拉法 (Projected Newton-Euler)

### 6.1 方法概述

投影牛顿-欧拉法是对经典牛顿-欧拉法的改进。其核心思想是:

> 利用虚功原理，将牛顿-欧拉方程**投影**到广义坐标空间，从而自动消除约束力。

这种方法结合了牛顿-欧拉法的物理直觉和拉格朗日方法的简洁性。

### 6.2 推导过程

**Step 1: 用广义坐标表示冲量和角动量**

对于系统中的第 $i$ 个刚体，其质心速度和角速度可以通过雅可比矩阵与广义速度联系:

$$
\mathbf{v}_{S_i} = J_{P_i}(q) \, \dot{q}
$$

$$
\boldsymbol{\omega}_i = J_{R_i}(q) \, \dot{q}
$$

其中:
- $J_{P_i} \in \mathbb{R}^{3 \times n}$: 第 $i$ 个刚体质心的平移雅可比
- $J_{R_i} \in \mathbb{R}^{3 \times n}$: 第 $i$ 个刚体的旋转雅可比

**Step 2: 线性动量和角动量**

第 $i$ 个刚体的线性动量:

$$
\mathbf{p}_i = m_i \mathbf{v}_{S_i} = m_i J_{P_i} \dot{q}
$$

第 $i$ 个刚体的角动量:

$$
\mathbf{N}_{S_i} = \boldsymbol{\Theta}_{S_i} \boldsymbol{\omega}_i = \boldsymbol{\Theta}_{S_i} J_{R_i} \dot{q}
$$

**Step 3: 虚位移用广义坐标表示**

$$
\delta \mathbf{r}_{S_i} = J_{P_i} \, \delta q
$$

$$
\delta \boldsymbol{\phi}_i = J_{R_i} \, \delta q
$$

**Step 4: 应用虚功原理**

虚功原理要求:

$$
\sum_{i=1}^{N} \left[ \delta \mathbf{r}_{S_i}^T (\dot{\mathbf{p}}_i - \mathbf{F}_i) + \delta \boldsymbol{\phi}_i^T (\dot{\mathbf{N}}_{S_i} - \boldsymbol{\Gamma}_i) \right] = 0
$$

将虚位移用广义坐标表示:

$$
\delta q^T \sum_{i=1}^{N} \left[ J_{P_i}^T (\dot{\mathbf{p}}_i - \mathbf{F}_i) + J_{R_i}^T (\dot{\mathbf{N}}_{S_i} - \boldsymbol{\Gamma}_i) \right] = 0
$$

由于 $\delta q$ 是任意的，括号内必须为零:

$$
\sum_{i=1}^{N} \left[ J_{P_i}^T \dot{\mathbf{p}}_i + J_{R_i}^T \dot{\mathbf{N}}_{S_i} \right] = \sum_{i=1}^{N} \left[ J_{P_i}^T \mathbf{F}_i + J_{R_i}^T \boldsymbol{\Gamma}_i \right]
$$

### 6.3 质量矩阵的构造

展开左边的惯性项，可以得到:

$$
\dot{\mathbf{p}}_i = m_i (J_{P_i} \ddot{q} + \dot{J}_{P_i} \dot{q})
$$

$$
\dot{\mathbf{N}}_{S_i} = \boldsymbol{\Theta}_{S_i} (J_{R_i} \ddot{q} + \dot{J}_{R_i} \dot{q}) + \boldsymbol{\omega}_i \times (\boldsymbol{\Theta}_{S_i} \boldsymbol{\omega}_i)
$$

代入虚功方程，整理后得到:

$$
M(q) \ddot{q} + b(q, \dot{q}) + g(q) = \tau
$$

其中**质量矩阵**为所有刚体贡献之和:

$$
\boxed{M(q) = \sum_{i=1}^{N} \left[ m_i J_{P_i}^T J_{P_i} + J_{R_i}^T \boldsymbol{\Theta}_{S_i} J_{R_i} \right]}
$$

这是一个非常重要的结果: 质量矩阵由每个刚体的**平移惯性**和**旋转惯性**两部分组成。

**非线性项** $b(q, \dot{q})$ 包含:

$$
b(q, \dot{q}) = \sum_{i=1}^{N} \left[ m_i J_{P_i}^T \dot{J}_{P_i} \dot{q} + J_{R_i}^T \left( \boldsymbol{\Theta}_{S_i} \dot{J}_{R_i} \dot{q} + \boldsymbol{\omega}_i \times (\boldsymbol{\Theta}_{S_i} \boldsymbol{\omega}_i) \right) \right]
$$

### 6.4 方法优点

- **自动消除约束力**: 通过投影到广义坐标空间，内部约束力不出现在最终方程中
- **直接得到标准形式**: 结果直接就是 $M(q)\ddot{q} + b + g = \tau$ 的形式
- **系统化**: 适合计算机实现，可以自动化处理任意多体系统
- **物理意义清晰**: 质量矩阵的每一项都有明确的物理来源

### 6.5 小车摆示例 (Cart-Pole — Projected Newton-Euler)

继续使用小车摆系统: $q = (x, \theta)^T$

**Step 1: 计算各刚体的雅可比**

小车质心位置: $\mathbf{r}_{S_c} = (x, 0)^T$

$$
J_{P_c} = \frac{\partial \mathbf{r}_{S_c}}{\partial q} = \begin{pmatrix} 1 & 0 \ 0 & 0 \end{pmatrix}
$$

小车无旋转: $J_{R_c} = \begin{pmatrix} 0 & 0 \end{pmatrix}$

摆锤质心位置: $\mathbf{r}_{S_p} = (x + L\sin\theta, \; -L\cos\theta)^T$

$$
J_{P_p} = \frac{\partial \mathbf{r}_{S_p}}{\partial q} = \begin{pmatrix} 1 & L\cos\theta \ 0 & L\sin\theta \end{pmatrix}
$$

摆杆旋转（2D 情况下角速度就是 $\dot{\theta}$）: $J_{R_p} = \begin{pmatrix} 0 & 1 \end{pmatrix}$

**Step 2: 构造质量矩阵**

$$
M(q) = m_c J_{P_c}^T J_{P_c} + m_p J_{P_p}^T J_{P_p} + J_{R_p}^T I_p J_{R_p}
$$

计算各项:

$$
m_c J_{P_c}^T J_{P_c} = m_c \begin{pmatrix} 1 & 0 \ 0 & 0 \end{pmatrix}
$$

$$
m_p J_{P_p}^T J_{P_p} = m_p \begin{pmatrix} 1 & L\cos\theta \ L\cos\theta & L^2 \end{pmatrix}
$$

对于质点摆锤 $I_p = 0$（若考虑摆杆转动惯量则 $I_p \neq 0$），因此:

$$
M(q) = \begin{pmatrix} m_c + m_p & m_p L\cos\theta \ m_p L\cos\theta & m_p L^2 \end{pmatrix}
$$

这与牛顿-欧拉法得到的结果完全一致，但推导过程更加系统化，无需处理约束力。

> **Quiz 思考**: 如果摆杆有非零的转动惯量 $I_p$，质量矩阵如何变化? 答: $M_{22}$ 变为 $m_p L^2 + I_p$，其余不变。

---

## 7. 第三种方法: 拉格朗日方程 (Lagrange Equations)

### 7.1 方法概述

拉格朗日方法是分析力学中最优雅的方法之一。它完全基于**能量** (Energy) 而非力，通过标量函数（拉格朗日量）来推导运动方程。

核心思想:

> 系统的动力学完全由动能和势能决定。

### 7.2 拉格朗日量 (Lagrangian)

拉格朗日量定义为动能与势能之差:

$$
\boxed{L(q, \dot{q}) = T(q, \dot{q}) - U(q)}
$$

其中:
- $T(q, \dot{q})$: 系统的总动能 (Kinetic Energy)
- $U(q)$: 系统的总势能 (Potential Energy)

### 7.3 动能 (Kinetic Energy)

系统的总动能可以写成广义坐标的二次型:

$$
T = \frac{1}{2} \dot{q}^T M(q) \dot{q}
$$

其中 $M(q)$ 就是质量矩阵。展开来看，每个刚体的动能贡献为:

$$
T_i = \underbrace{\frac{1}{2} m_i \mathbf{v}_{S_i}^T \mathbf{v}_{S_i}}_{\text{平移动能}} + \underbrace{\frac{1}{2} \boldsymbol{\omega}_i^T \boldsymbol{\Theta}_{S_i} \boldsymbol{\omega}_i}_{\text{旋转动能}}
$$

总动能:

$$
T = \sum_{i=1}^{N} T_i = \frac{1}{2} \dot{q}^T \underbrace{\left[\sum_{i=1}^{N} (m_i J_{P_i}^T J_{P_i} + J_{R_i}^T \boldsymbol{\Theta}_{S_i} J_{R_i})\right]}_{M(q)} \dot{q}
$$

注意: 这与投影牛顿-欧拉法中得到的质量矩阵完全一致。

### 7.4 势能 (Potential Energy)

势能通常包含两部分:

**重力势能 (Gravitational Potential Energy):**

$$
U_{\text{grav}} = \sum_{i=1}^{N} m_i g \, h_{S_i}(q)
$$

其中 $h_{S_i}(q)$ 是第 $i$ 个刚体质心的高度。

**弹性势能 (Elastic/Spring Potential Energy):**

$$
U_{\text{spring}} = \frac{1}{2} k \, \Delta x^2
$$

其中 $k$ 是弹簧刚度，$\Delta x$ 是弹簧的变形量。

### 7.5 拉格朗日方程的推导

从虚功原理出发，经过变分运算，可以得到**拉格朗日方程 (Euler-Lagrange Equations)**:

$$
\boxed{\frac{d}{dt}\frac{\partial T}{\partial \dot{q}_j} - \frac{\partial T}{\partial q_j} + \frac{\partial U}{\partial q_j} = \tau_j, \quad j = 1, \ldots, n}
$$

或者等价地用拉格朗日量表示:

$$
\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_j} - \frac{\partial L}{\partial q_j} = \tau_j
$$

其中 $\tau_j$ 是对应于广义坐标 $q_j$ 的**广义力 (Generalized Force)**。

### 7.6 从拉格朗日方程到标准运动方程

将 $T = \frac{1}{2}\dot{q}^T M(q) \dot{q}$ 代入拉格朗日方程，展开计算:

**第一项**: $\frac{\partial T}{\partial \dot{q}_j} = \sum_k M_{jk} \dot{q}_k$

$$
\frac{d}{dt}\frac{\partial T}{\partial \dot{q}_j} = \sum_k M_{jk} \ddot{q}_k + \sum_k \dot{M}_{jk} \dot{q}_k
$$

**第二项**: 

$$
\frac{\partial T}{\partial q_j} = \frac{1}{2} \dot{q}^T \frac{\partial M}{\partial q_j} \dot{q}
$$

合并后可以得到:

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau
$$

其中 $C(q, \dot{q})$ 是**科里奥利矩阵 (Coriolis Matrix)**，其元素由 **Christoffel 符号**给出:

$$
C_{jk} = \sum_{l} c_{jkl} \, \dot{q}_l, \quad c_{jkl} = \frac{1}{2}\left(\frac{\partial M_{jk}}{\partial q_l} + \frac{\partial M_{jl}}{\partial q_k} - \frac{\partial M_{kl}}{\partial q_j}\right)
$$

注意: $b(q, \dot{q}) = C(q, \dot{q})\dot{q}$，即非线性力项可以分解为科里奥利矩阵乘以广义速度。

### 7.7 重要性质

1. **$\dot{M} - 2C$ 是反对称矩阵**: $\dot{q}^T(\dot{M} - 2C)\dot{q} = 0$
   - 这个性质在控制器设计中非常重要（用于证明无源性 / Passivity）

2. **质量矩阵对称正定**: $M(q) = M(q)^T > 0$

3. **势能只依赖于位置**: $U = U(q)$，不依赖于速度

### 7.8 小车摆示例 (Cart-Pole — Lagrange)

**Step 1: 计算动能**

小车动能:

$$
T_c = \frac{1}{2} m_c \dot{x}^2
$$

摆锤速度分量:

$$
\dot{x}_p = \dot{x} + L\dot{\theta}\cos\theta, \quad \dot{y}_p = L\dot{\theta}\sin\theta
$$

摆锤动能:

$$
T_p = \frac{1}{2} m_p (\dot{x}_p^2 + \dot{y}_p^2) = \frac{1}{2} m_p \left[(\dot{x} + L\dot{\theta}\cos\theta)^2 + (L\dot{\theta}\sin\theta)^2\right]
$$

展开:

$$
T_p = \frac{1}{2} m_p \left[\dot{x}^2 + 2L\dot{x}\dot{\theta}\cos\theta + L^2\dot{\theta}^2\right]
$$

总动能:

$$
T = T_c + T_p = \frac{1}{2}(m_c + m_p)\dot{x}^2 + m_p L \dot{x}\dot{\theta}\cos\theta + \frac{1}{2}m_p L^2 \dot{\theta}^2
$$

写成矩阵形式:

$$
T = \frac{1}{2} \begin{pmatrix} \dot{x} \ \dot{\theta} \end{pmatrix}^T \begin{pmatrix} m_c + m_p & m_p L\cos\theta \ m_p L\cos\theta & m_p L^2 \end{pmatrix} \begin{pmatrix} \dot{x} \ \dot{\theta} \end{pmatrix}
$$

**Step 2: 计算势能**

以铰链为零势能参考点:

$$
U = -m_p g L \cos\theta
$$

（小车在水平面上运动，重力势能不变，可忽略）

**Step 3: 应用拉格朗日方程**

对 $q_1 = x$:

$$
\frac{d}{dt}\frac{\partial T}{\partial \dot{x}} - \frac{\partial T}{\partial x} + \frac{\partial U}{\partial x} = \tau_x
$$

$$
\frac{\partial T}{\partial \dot{x}} = (m_c + m_p)\dot{x} + m_p L \dot{\theta}\cos\theta
$$

$$
\frac{d}{dt}\frac{\partial T}{\partial \dot{x}} = (m_c + m_p)\ddot{x} + m_p L \ddot{\theta}\cos\theta - m_p L \dot{\theta}^2 \sin\theta
$$

$$
\frac{\partial T}{\partial x} = 0, \quad \frac{\partial U}{\partial x} = 0
$$

因此:

$$
(m_c + m_p)\ddot{x} + m_p L \ddot{\theta}\cos\theta - m_p L \dot{\theta}^2 \sin\theta = F
$$

对 $q_2 = \theta$:

$$
\frac{d}{dt}\frac{\partial T}{\partial \dot{\theta}} - \frac{\partial T}{\partial \theta} + \frac{\partial U}{\partial \theta} = \tau_\theta
$$

$$
\frac{\partial T}{\partial \dot{\theta}} = m_p L \dot{x}\cos\theta + m_p L^2 \dot{\theta}
$$

$$
\frac{d}{dt}\frac{\partial T}{\partial \dot{\theta}} = m_p L \ddot{x}\cos\theta - m_p L \dot{x}\dot{\theta}\sin\theta + m_p L^2 \ddot{\theta}
$$

$$
\frac{\partial T}{\partial \theta} = -m_p L \dot{x}\dot{\theta}\sin\theta
$$

$$
\frac{\partial U}{\partial \theta} = m_p g L \sin\theta
$$

因此:

$$
m_p L \ddot{x}\cos\theta + m_p L^2 \ddot{\theta} + m_p g L \sin\theta = 0
$$

（假设摆杆关节无外加扭矩: $\tau_\theta = 0$）

**最终运动方程:**

$$
\begin{pmatrix} m_c + m_p & m_p L\cos\theta \ m_p L\cos\theta & m_p L^2 \end{pmatrix} \begin{pmatrix} \ddot{x} \ \ddot{\theta} \end{pmatrix} + \begin{pmatrix} -m_p L \dot{\theta}^2 \sin\theta \ 0 \end{pmatrix} + \begin{pmatrix} 0 \ m_p g L \sin\theta \end{pmatrix} = \begin{pmatrix} F \ 0 \end{pmatrix}
$$

> **Quiz 验证**: 三种方法（牛顿-欧拉、投影牛顿-欧拉、拉格朗日）得到的运动方程完全一致! 这验证了方法的等价性。

---

## 8. 外力 (External Forces)

### 8.1 广义力的计算

在拉格朗日框架中，外力通过**广义力 (Generalized Forces)** 进入运动方程。广义力的计算基于虚功原理:

> 外力在虚位移上所做的虚功等于广义力在广义坐标虚位移上所做的虚功。

设一个外力 $\mathbf{F}$ 作用在点 $P$，该点的虚位移为 $\delta \mathbf{r}_P$，则虚功为:

$$
\delta W = \mathbf{F}^T \delta \mathbf{r}_P = \mathbf{F}^T J_P \, \delta q = \tau^T \delta q
$$

因此广义力为:

$$
\boxed{\tau = J_P^T \mathbf{F}}
$$

其中 $J_P = \frac{\partial \mathbf{r}_P}{\partial q}$ 是力作用点的雅可比矩阵。

类似地，如果有一个外力矩 $\boldsymbol{\Gamma}$ 作用在某个刚体上:

$$
\tau = J_R^T \boldsymbol{\Gamma}
$$

### 8.2 多个外力的叠加

如果系统受到多个外力作用，广义力是各个力贡献的叠加:

$$
\tau = \sum_k J_{P_k}^T \mathbf{F}_k + \sum_k J_{R_k}^T \boldsymbol{\Gamma}_k
$$

### 8.3 常见外力类型

#### 8.3.1 执行器扭矩 (Actuator Torques)

对于关节执行器，扭矩直接作用在广义坐标上。如果执行器扭矩 $\tau_{\text{act}}$ 直接驱动关节 $q_j$，则:

$$
\tau_j = \tau_{\text{act}}
$$

这是最简单的情况，因为执行器力/扭矩与广义坐标直接对应。

#### 8.3.2 弹簧力 (Spring Forces)

线性弹簧产生的力与变形量成正比:

$$
\mathbf{F}_{\text{spring}} = -k \, \Delta \mathbf{x}
$$

其中:
- $k$: 弹簧刚度 (Spring Stiffness)
- $\Delta \mathbf{x}$: 弹簧变形量（当前长度 - 自然长度）

弹簧力对应的广义力:

$$
\tau_{\text{spring}} = J_{\text{spring}}^T \mathbf{F}_{\text{spring}}
$$

或者，如果弹簧势能已知，可以直接通过势能的梯度计算:

$$
\tau_{\text{spring}} = -\frac{\partial U_{\text{spring}}}{\partial q}
$$

其中 $U_{\text{spring}} = \frac{1}{2} k \, \|\Delta \mathbf{x}\|^2$。

#### 8.3.3 阻尼力 (Damping Forces)

粘性阻尼力与速度成正比:

$$
\mathbf{F}_{\text{damp}} = -d \, \dot{\mathbf{x}}
$$

对应的广义力:

$$
\tau_{\text{damp}} = -J^T d \, J \, \dot{q} = -D(q) \dot{q}
$$

其中 $D(q) = J^T d \, J$ 是广义阻尼矩阵。

注意: 阻尼力是**非保守力 (Non-conservative Force)**，不能通过势能来描述，必须作为广义力直接加入运动方程。

### 8.4 小车摆示例: 电机和弹簧 (Cart-Pole with Motor and Spring)

考虑扩展的小车摆系统，增加以下外力:

```
        O ← 铰链 (pivot)
       /|
      / | 扭簧 k_θ
     /  |
    ●   
    
 [████████] ←→ F_motor (电机驱动力)
 ===弹簧k_x===|墙
 ──────────────── 
```

**外力清单:**
1. 电机驱动力 $F_{\text{motor}}$: 作用在小车上，沿 $x$ 方向
2. 弹簧力 (线性弹簧): 连接小车与墙壁，刚度 $k_x$，自然长度 $x_0$
3. 扭簧 (Torsional Spring): 作用在铰链处，刚度 $k_\theta$，自然角度 $\theta_0$

**Step 1: 电机驱动力的广义力**

电机力 $F_{\text{motor}}$ 作用在小车上，作用点的雅可比:

$$
J_{\text{motor}} = \frac{\partial x}{\partial q} = \begin{pmatrix} 1 & 0 \end{pmatrix}
$$

广义力:

$$
\tau_{\text{motor}} = J_{\text{motor}}^T F_{\text{motor}} = \begin{pmatrix} F_{\text{motor}} \ 0 \end{pmatrix}
$$

**Step 2: 线性弹簧的广义力**

弹簧势能:

$$
U_{\text{spring}} = \frac{1}{2} k_x (x - x_0)^2
$$

广义力（通过势能梯度）:

$$
\tau_{\text{spring}} = -\frac{\partial U_{\text{spring}}}{\partial q} = \begin{pmatrix} -k_x(x - x_0) \ 0 \end{pmatrix}
$$

**Step 3: 扭簧的广义力**

扭簧势能:

$$
U_{\text{torsion}} = \frac{1}{2} k_\theta (\theta - \theta_0)^2
$$

广义力:

$$
\tau_{\text{torsion}} = -\frac{\partial U_{\text{torsion}}}{\partial q} = \begin{pmatrix} 0 \ -k_\theta(\theta - \theta_0) \end{pmatrix}
$$

**Step 4: 完整运动方程**

将所有外力合并:

$$
M(q)\ddot{q} + b(q, \dot{q}) + g(q) = \tau_{\text{motor}} + \tau_{\text{spring}} + \tau_{\text{torsion}}
$$

展开:

$$
\begin{pmatrix} m_c + m_p & m_p L\cos\theta \ m_p L\cos\theta & m_p L^2 \end{pmatrix} \begin{pmatrix} \ddot{x} \ \ddot{\theta} \end{pmatrix} + \begin{pmatrix} -m_p L \dot{\theta}^2 \sin\theta \ 0 \end{pmatrix} + \begin{pmatrix} 0 \ m_p g L \sin\theta \end{pmatrix}
$$

$$
= \begin{pmatrix} F_{\text{motor}} - k_x(x - x_0) \ -k_\theta(\theta - \theta_0) \end{pmatrix}
$$

或者，将弹簧力归入势能项，定义总势能:

$$
U_{\text{total}} = -m_p g L \cos\theta + \frac{1}{2}k_x(x - x_0)^2 + \frac{1}{2}k_\theta(\theta - \theta_0)^2
$$

则运动方程变为:

$$
M(q)\ddot{q} + b(q, \dot{q}) + \frac{\partial U_{\text{total}}}{\partial q} = \begin{pmatrix} F_{\text{motor}} \ 0 \end{pmatrix}
$$

其中:

$$
\frac{\partial U_{\text{total}}}{\partial q} = \begin{pmatrix} k_x(x - x_0) \ m_p g L \sin\theta + k_\theta(\theta - \theta_0) \end{pmatrix}
$$

> **Quiz 思考**: 如果弹簧的自然长度 $x_0 = 0$ 且 $\theta_0 = 0$，系统的平衡点在哪里? 
> 
> 令 $\ddot{q} = 0, \dot{q} = 0, F_{\text{motor}} = 0$，得到:
> $$k_x \cdot x = 0 \Rightarrow x = 0$$
> $$m_p g L \sin\theta + k_\theta \cdot \theta = 0$$
> 
> 对于 $\theta = 0$（竖直向上）是一个平衡点，但它是不稳定的（倒立摆）。$\theta = \pi$（竖直向下）附近也有平衡点，且是稳定的。

---

## 总结: 三种方法的比较

| 特性 | 牛顿-欧拉法 | 投影牛顿-欧拉法 | 拉格朗日法 |
|------|-------------|-----------------|-----------|
| **基本思想** | 力平衡 | 虚功原理 + 力平衡 | 能量方法 |
| **约束力** | 显式出现，需消除 | 自动消除 | 自动消除 |
| **方程数量** | $6n$ 个，需化简 | 直接 $n$ 个 | 直接 $n$ 个 |
| **物理直觉** | 最强 | 较强 | 基于能量 |
| **计算复杂度** | 大系统极高 | 中等 | 中等 |
| **获取约束力** | 直接可得 | 需额外计算 | 需额外计算 |
| **适用场景** | 简单系统、需要约束力 | 通用 | 通用、控制设计 |
| **计算机实现** | 递推算法高效 | 系统化 | 系统化 |

### 关键公式速查

**运动方程:**

$$
M(q)\ddot{q} + b(q, \dot{q}) + g(q) = \tau + J_c^T F_c
$$

**质量矩阵:**

$$
M(q) = \sum_{i=1}^{N} \left[ m_i J_{P_i}^T J_{P_i} + J_{R_i}^T \boldsymbol{\Theta}_{S_i} J_{R_i} \right]
$$

**拉格朗日方程:**

$$
\frac{d}{dt}\frac{\partial T}{\partial \dot{q}} - \frac{\partial T}{\partial q} + \frac{\partial U}{\partial q} = \tau
$$

**广义力:**

$$
\tau = J^T \mathbf{F}
$$

---

> **下一讲预告**: Lecture 6 将继续讨论动力学，重点介绍递推牛顿-欧拉算法 (Recursive Newton-Euler Algorithm, RNEA) 和动力学的数值实现。
