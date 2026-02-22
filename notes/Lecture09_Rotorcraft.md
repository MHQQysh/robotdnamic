# Lecture 9: Rotorcraft — 旋翼飞行器动力学建模与控制

> **ETH Zurich — Robot Dynamics**
> 讲义整理 | Lecture Notes

---

## 目录

1. [旋翼飞行器概述 (Rotorcraft Overview)](#1-旋翼飞行器概述-rotorcraft-overview)
2. [旋翼飞行器建模 (Rotorcraft Modeling)](#2-旋翼飞行器建模-rotorcraft-modeling)
3. [旋翼推力模型 (Rotor Thrust Model)](#3-旋翼推力模型-rotor-thrust-model)
4. [力和力矩的合成 (Force and Moment Aggregation)](#4-力和力矩的合成-force-and-moment-aggregation)
5. [运动方程 (Equations of Motion)](#5-运动方程-equations-of-motion)
6. [控制架构 (Control Architecture)](#6-控制架构-control-architecture)
7. [六旋翼案例研究 (Hexacopter Case Study)](#7-六旋翼案例研究-hexacopter-case-study)
8. [关键公式总结](#8-关键公式总结)

---

## 1. 旋翼飞行器概述 (Rotorcraft Overview)

### 1.1 什么是旋翼飞行器

旋翼飞行器 (Rotorcraft) 是一类通过旋转翼面 (rotor) 产生升力和控制力矩的飞行器。与固定翼飞行器不同，旋翼飞行器能够实现**垂直起降 (VTOL, Vertical Take-Off and Landing)** 和**悬停 (Hover)**，这使其在机器人领域具有极高的应用价值。

### 1.2 多旋翼飞行器类型

根据旋翼数量的不同，常见的多旋翼飞行器 (Multirotor) 可分为以下几类：

| 类型 | 旋翼数量 | 特点 |
|------|----------|------|
| **Tricopter** (三旋翼) | 3 | 结构简单，需要伺服机构调整偏航 |
| **Quadrotor** (四旋翼) | 4 | 最常见，结构对称，控制简洁 |
| **Hexacopter** (六旋翼) | 6 | 具有冗余性，可容忍单旋翼失效 |
| **Octocopter** (八旋翼) | 8 | 高冗余、高载荷能力 |

**四旋翼 (Quadrotor)** 是最经典的构型。四个旋翼分布在十字形或 X 形机架的末端，其中**相邻旋翼旋转方向相反**，以抵消反扭矩 (reaction torque)。

- 旋翼 1, 3: 顺时针 (CW, Clockwise)
- 旋翼 2, 4: 逆时针 (CCW, Counter-Clockwise)

**六旋翼 (Hexacopter)** 在四旋翼基础上增加了两个旋翼，提供了**执行器冗余 (actuator redundancy)**。即使一个旋翼失效，飞行器仍可维持可控飞行。

**八旋翼 (Octocopter)** 进一步增加冗余性和最大推力，适用于重载任务。

### 1.3 应用领域

旋翼飞行器在以下领域有广泛应用：

- **航拍与测绘 (Aerial Photography & Mapping)**: 搭载相机或 LiDAR 进行地形测绘、建筑检测
- **工业检测 (Industrial Inspection)**: 桥梁、风力发电机、输电线路的无损检测
- **物流配送 (Logistics & Delivery)**: 最后一公里配送、医疗物资运输
- **搜索与救援 (Search & Rescue)**: 灾区搜索、热成像探测
- **农业 (Agriculture)**: 精准喷洒、作物监测
- **科研 (Research)**: 作为移动机器人平台，研究控制、规划、感知算法

---

## 2. 旋翼飞行器建模 (Rotorcraft Modeling)

### 2.1 坐标系定义

建模的第一步是定义参考坐标系。我们使用两个主要坐标系：

**惯性坐标系 (Inertial Frame) $\{I\}$**:
- 固定于地面，满足 Newton 定律的惯性参考系
- 通常采用 NED (North-East-Down) 或 ENU (East-North-Up) 约定
- 本课程采用 $z$ 轴向上的右手坐标系: $\{I\} = \{e_x^I, e_y^I, e_z^I\}$

**机体坐标系 (Body Frame) $\{B\}$**:
- 固连于飞行器质心 (Center of Mass, CoM)
- $x_B$ 轴指向飞行器前方
- $z_B$ 轴指向飞行器上方（与推力方向一致）
- $\{B\} = \{e_x^B, e_y^B, e_z^B\}$

### 2.2 姿态表示 (Attitude Representation)

机体坐标系相对于惯性坐标系的姿态可以用多种方式表示：

**Euler 角 (ZYX 约定)**:

$$\boldsymbol{\phi} = \begin{bmatrix} \phi \ \theta \ \psi \end{bmatrix}$$

其中：
- $\phi$ — 滚转角 (Roll)，绕 $x$ 轴旋转
- $\theta$ — 俯仰角 (Pitch)，绕 $y$ 轴旋转
- $\psi$ — 偏航角 (Yaw)，绕 $z$ 轴旋转

对应的旋转矩阵 (Rotation Matrix) $\mathbf{C}_{IB} \in SO(3)$ 将机体系向量变换到惯性系：

$$\mathbf{C}_{IB} = \mathbf{R}_z(\psi) \, \mathbf{R}_y(\theta) \, \mathbf{R}_x(\phi)$$

展开为：

$$\mathbf{C}_{IB} = \begin{bmatrix} c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \ s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \ -s\theta & c\theta s\phi & c\theta c\phi \end{bmatrix}$$

其中 $c(\cdot) = \cos(\cdot)$，$s(\cdot) = \sin(\cdot)$。

> **注意**: Euler 角在 $\theta = \pm 90°$ 时存在**万向节锁 (Gimbal Lock)** 问题。对于大角度机动，建议使用**四元数 (Quaternion)** 或**旋转矩阵**直接参数化。

### 2.3 广义坐标与广义速度

**广义坐标 (Generalized Coordinates)**:

$$\mathbf{q} = \begin{bmatrix} \mathbf{r}_{IB} \ \boldsymbol{\phi} \end{bmatrix} \in \mathbb{R}^6$$

其中 $\mathbf{r}_{IB} = [x, y, z]^T$ 是机体质心在惯性系中的位置。

**广义速度 (Generalized Velocities)**:

$$\mathbf{u} = \begin{bmatrix} \mathbf{v}_B \ \boldsymbol{\omega}_B \end{bmatrix} \in \mathbb{R}^6$$

其中：
- $\mathbf{v}_B$ — 质心速度在**机体系**中的表示
- $\boldsymbol{\omega}_B$ — 角速度在**机体系**中的表示

广义坐标与广义速度之间的运动学关系为：

$$\dot{\mathbf{r}}_{IB} = \mathbf{C}_{IB} \, \mathbf{v}_B$$

$$\dot{\boldsymbol{\phi}} = \mathbf{E}(\boldsymbol{\phi}) \, \boldsymbol{\omega}_B$$

其中 $\mathbf{E}(\boldsymbol{\phi})$ 是将机体角速度映射到 Euler 角速率的矩阵：

$$\mathbf{E}(\boldsymbol{\phi}) = \begin{bmatrix} 1 & \sin\phi \tan\theta & \cos\phi \tan\theta \ 0 & \cos\phi & -\sin\phi \ 0 & \sin\phi / \cos\theta & \cos\phi / \cos\theta \end{bmatrix}$$

### 2.4 刚体假设

在建模中，我们做以下假设：

1. 飞行器为**刚体 (Rigid Body)**，忽略结构弹性变形
2. 质量 $m$ 和惯性张量 $\mathbf{I}_B$ 恒定
3. 机体关于 $xz$ 平面和 $yz$ 平面对称，因此惯性张量为对角矩阵：

$$\mathbf{I}_B = \begin{bmatrix} I_{xx} & 0 & 0 \ 0 & I_{yy} & 0 \ 0 & 0 & I_{zz} \end{bmatrix}$$

---

## 3. 旋翼推力模型 (Rotor Thrust Model)

### 3.1 单旋翼推力

每个旋翼通过高速旋转产生气动推力。根据叶素理论 (Blade Element Theory) 的简化模型，单个旋翼 $i$ 产生的推力与其转速的平方成正比：

$$F_i = k_F \cdot \omega_i^2$$

其中：
- $F_i$ — 第 $i$ 个旋翼产生的推力 (单位: N)
- $k_F$ — 推力系数 (Thrust Coefficient)，取决于桨叶几何形状、空气密度等 (单位: $\text{N} \cdot \text{s}^2 / \text{rad}^2$)
- $\omega_i$ — 第 $i$ 个旋翼的角速度 (单位: rad/s)

推力方向沿旋翼轴，即机体系的 $z_B$ 方向（向上）：

$$\mathbf{F}_i^B = \begin{bmatrix} 0 \ 0 \ k_F \omega_i^2 \end{bmatrix}$$

### 3.2 单旋翼反扭矩

旋翼旋转时，空气对桨叶的阻力会产生一个与旋转方向相反的反扭矩 (Reaction Torque / Drag Moment)：

$$M_i = k_M \cdot \omega_i^2$$

其中：
- $M_i$ — 第 $i$ 个旋翼产生的反扭矩大小 (单位: N·m)
- $k_M$ — 扭矩系数 (Torque Coefficient) (单位: $\text{N} \cdot \text{m} \cdot \text{s}^2 / \text{rad}^2$)

反扭矩的方向取决于旋翼的旋转方向：
- 顺时针 (CW) 旋翼: 反扭矩沿 $-z_B$ 方向
- 逆时针 (CCW) 旋翼: 反扭矩沿 $+z_B$ 方向

定义旋转方向因子 $\sigma_i$：

$$\sigma_i = \begin{cases} +1 & \text{CCW 旋翼} \ -1 & \text{CW 旋翼} \end{cases}$$

则反扭矩向量为：

$$\mathbf{M}_i^B = \begin{bmatrix} 0 \ 0 \ \sigma_i \, k_M \, \omega_i^2 \end{bmatrix}$$

### 3.3 推力系数与扭矩系数的关系

推力系数 $k_F$ 和扭矩系数 $k_M$ 之间的比值定义为：

$$\kappa = \frac{k_M}{k_F}$$

$\kappa$ 具有长度的量纲 (单位: m)，其物理意义是将推力"转换"为扭矩的等效力臂长度。典型值范围为 $\kappa \approx 0.005 \sim 0.02 \, \text{m}$。

### 3.4 电机动力学

实际系统中，电机转速不能瞬间改变。电机动力学可近似为一阶系统：

$$\dot{\omega}_i = \frac{1}{\tau_m} (\omega_i^{\text{cmd}} - \omega_i)$$

其中 $\tau_m$ 是电机时间常数（通常为 $10 \sim 50 \, \text{ms}$），$\omega_i^{\text{cmd}}$ 是指令转速。在控制器设计中，若控制带宽远低于电机带宽，可忽略此动力学。

---

## 4. 力和力矩的合成 (Force and Moment Aggregation)

### 4.1 总推力

对于具有 $n$ 个旋翼的飞行器，所有旋翼产生的总推力在机体系中为：

$$\mathbf{F}_{\text{total}}^B = \sum_{i=1}^{n} \mathbf{F}_i^B = \begin{bmatrix} 0 \ 0 \ \sum_{i=1}^{n} k_F \omega_i^2 \end{bmatrix} = \begin{bmatrix} 0 \ 0 \ T \end{bmatrix}$$

其中总推力标量 $T = \sum_{i=1}^{n} k_F \omega_i^2$。

注意：总推力始终沿机体系 $z_B$ 轴方向，这是旋翼飞行器**欠驱动 (underactuated)** 特性的根本原因。

### 4.2 总力矩

作用在机体上的总力矩由两部分组成：

**a) 推力产生的力矩**

每个旋翼位于机体系中的位置为 $\mathbf{r}_i^B = [x_i, y_i, 0]^T$（假设所有旋翼在同一平面上）。推力 $\mathbf{F}_i^B$ 对质心产生的力矩为：

$$\boldsymbol{\tau}_i^{\text{thrust}} = \mathbf{r}_i^B \times \mathbf{F}_i^B = \begin{bmatrix} x_i \ y_i \ 0 \end{bmatrix} \times \begin{bmatrix} 0 \ 0 \ k_F \omega_i^2 \end{bmatrix} = \begin{bmatrix} y_i \, k_F \omega_i^2 \ -x_i \, k_F \omega_i^2 \ 0 \end{bmatrix}$$

**b) 反扭矩**

$$\boldsymbol{\tau}_i^{\text{drag}} = \begin{bmatrix} 0 \ 0 \ \sigma_i \, k_M \omega_i^2 \end{bmatrix}$$

**总力矩**:

$$\mathbf{M}_{\text{total}}^B = \sum_{i=1}^{n} \left( \boldsymbol{\tau}_i^{\text{thrust}} + \boldsymbol{\tau}_i^{\text{drag}} \right) = \begin{bmatrix} \sum_{i=1}^{n} y_i \, k_F \omega_i^2 \ \sum_{i=1}^{n} (-x_i) \, k_F \omega_i^2 \ \sum_{i=1}^{n} \sigma_i \, k_M \omega_i^2 \end{bmatrix}$$

即：
- **Roll 力矩** $L$: 由旋翼沿 $y$ 方向的分布产生
- **Pitch 力矩** $M$: 由旋翼沿 $x$ 方向的分布产生
- **Yaw 力矩** $N$: 由旋翼反扭矩的差异产生

### 4.3 分配矩阵 (Allocation Matrix)

将控制输入定义为各旋翼转速的平方：

$$\boldsymbol{\gamma} = \begin{bmatrix} \omega_1^2 \ \omega_2^2 \ \vdots \ \omega_n^2 \end{bmatrix} \in \mathbb{R}^n$$

将期望的总推力和力矩组合为控制向量：

$$\mathbf{w} = \begin{bmatrix} T \ L \ M \ N \end{bmatrix} \in \mathbb{R}^4$$

则两者之间的关系可以用**分配矩阵 (Allocation Matrix)** $\mathbf{A} \in \mathbb{R}^{4 \times n}$ 表示：

$$\mathbf{w} = \mathbf{A} \, \boldsymbol{\gamma}$$

其中：

$$\mathbf{A} = \begin{bmatrix} k_F & k_F & \cdots & k_F \ y_1 k_F & y_2 k_F & \cdots & y_n k_F \ -x_1 k_F & -x_2 k_F & \cdots & -x_n k_F \ \sigma_1 k_M & \sigma_2 k_M & \cdots & \sigma_n k_M \end{bmatrix}$$

### 4.4 四旋翼的分配矩阵

对于标准 "+" 构型四旋翼，旋翼位置和旋转方向为：

| 旋翼 | 位置 $(x_i, y_i)$ | 旋转方向 | $\sigma_i$ |
|------|-------------------|----------|------------|
| 1 (前) | $(d, 0)$ | CCW | $+1$ |
| 2 (左) | $(0, d)$ | CW | $-1$ |
| 3 (后) | $(-d, 0)$ | CCW | $+1$ |
| 4 (右) | $(0, -d)$ | CW | $-1$ |

其中 $d$ 是旋翼到质心的距离 (arm length)。分配矩阵为：

$$\mathbf{A}_{\text{quad}} = \begin{bmatrix} k_F & k_F & k_F & k_F \ 0 & d \, k_F & 0 & -d \, k_F \ -d \, k_F & 0 & d \, k_F & 0 \ k_M & -k_M & k_M & -k_M \end{bmatrix}$$

对于四旋翼，$\mathbf{A}$ 是 $4 \times 4$ 的方阵。若 $\mathbf{A}$ 满秩（通常成立），则可以直接求逆：

$$\boldsymbol{\gamma} = \mathbf{A}^{-1} \, \mathbf{w}$$

### 4.5 冗余系统的分配

当旋翼数量 $n > 4$ 时（如六旋翼、八旋翼），分配矩阵 $\mathbf{A}$ 不是方阵，方程 $\mathbf{w} = \mathbf{A} \boldsymbol{\gamma}$ 是**欠定的 (underdetermined)**。此时可使用**伪逆 (Pseudo-inverse)** 求最小范数解：

$$\boldsymbol{\gamma} = \mathbf{A}^T (\mathbf{A} \mathbf{A}^T)^{-1} \, \mathbf{w} = \mathbf{A}^{\dagger} \, \mathbf{w}$$

这给出了满足力矩需求的**最小转速平方和**解，有利于降低能耗。也可以加入约束（如转速上下限）使用二次规划 (QP) 求解。

---

## 5. 运动方程 (Equations of Motion)

### 5.1 Newton-Euler 方程

旋翼飞行器作为六自由度刚体，其运动由 Newton-Euler 方程描述。

**平动方程 (Translational Dynamics)**:

在惯性系中，Newton 第二定律给出：

$$m \, \ddot{\mathbf{r}}_{IB} = \mathbf{C}_{IB} \, \mathbf{F}_{\text{total}}^B + m \mathbf{g}$$

其中：
- $m$ — 飞行器总质量
- $\ddot{\mathbf{r}}_{IB}$ — 质心在惯性系中的加速度
- $\mathbf{C}_{IB}$ — 机体系到惯性系的旋转矩阵
- $\mathbf{g} = [0, 0, -g]^T$ — 重力加速度向量（$z$ 轴向上时）

展开为：

$$m \begin{bmatrix} \ddot{x} \ \ddot{y} \ \ddot{z} \end{bmatrix} = \mathbf{C}_{IB} \begin{bmatrix} 0 \ 0 \ T \end{bmatrix} + \begin{bmatrix} 0 \ 0 \ -mg \end{bmatrix}$$

即：

$$m\ddot{x} = T(\cos\phi \sin\theta \cos\psi + \sin\phi \sin\psi)$$

$$m\ddot{y} = T(\cos\phi \sin\theta \sin\psi - \sin\phi \cos\psi)$$

$$m\ddot{z} = T\cos\phi \cos\theta - mg$$

等价地，在机体系中表示平动方程：

$$m(\dot{\mathbf{v}}_B + \boldsymbol{\omega}_B \times \mathbf{v}_B) = \mathbf{F}_{\text{total}}^B + \mathbf{C}_{IB}^T \, m\mathbf{g}$$

### 5.2 转动方程 (Rotational Dynamics)

**Euler 方程**在机体系中表示为：

$$\mathbf{I}_B \, \dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (\mathbf{I}_B \, \boldsymbol{\omega}_B) = \mathbf{M}_{\text{total}}^B$$

展开为三个分量方程：

$$I_{xx} \dot{p} + (I_{zz} - I_{yy}) q r = L$$

$$I_{yy} \dot{q} + (I_{xx} - I_{zz}) p r = M$$

$$I_{zz} \dot{r} + (I_{yy} - I_{xx}) p q = N$$

其中 $\boldsymbol{\omega}_B = [p, q, r]^T$ 分别是绕机体系 $x$, $y$, $z$ 轴的角速度分量。

> **物理解释**: 项 $\boldsymbol{\omega}_B \times (\mathbf{I}_B \boldsymbol{\omega}_B)$ 是**陀螺效应 (Gyroscopic Effect)**，它描述了由于角动量方向变化而产生的力矩。当飞行器同时绕多个轴旋转时，陀螺效应会导致轴间耦合。

### 5.3 陀螺力矩补充项

在精确建模中，还需考虑旋翼自身旋转产生的**陀螺力矩 (Gyroscopic Torque)**：

$$\mathbf{M}_{\text{gyro}} = \boldsymbol{\omega}_B \times \left( \sum_{i=1}^{n} J_r \, \sigma_i \, \omega_i \, \mathbf{e}_z^B \right)$$

其中 $J_r$ 是单个旋翼绕其旋转轴的转动惯量。此项在高速机动时不可忽略。

### 5.4 完整状态空间模型

定义状态向量：

$$\mathbf{x} = \begin{bmatrix} \mathbf{r}_{IB} \ \dot{\mathbf{r}}_{IB} \ \boldsymbol{\phi} \ \boldsymbol{\omega}_B \end{bmatrix} \in \mathbb{R}^{12}$$

控制输入为：

$$\mathbf{u} = \begin{bmatrix} T \ L \ M \ N \end{bmatrix} \in \mathbb{R}^4$$

状态方程 $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$：

$$\dot{\mathbf{r}}_{IB} = \mathbf{v}_I$$

$$\dot{\mathbf{v}}_I = \frac{1}{m} \mathbf{C}_{IB} \begin{bmatrix} 0 \ 0 \ T \end{bmatrix} + \mathbf{g}$$

$$\dot{\boldsymbol{\phi}} = \mathbf{E}(\boldsymbol{\phi}) \, \boldsymbol{\omega}_B$$

$$\dot{\boldsymbol{\omega}}_B = \mathbf{I}_B^{-1} \left( \begin{bmatrix} L \ M \ N \end{bmatrix} - \boldsymbol{\omega}_B \times (\mathbf{I}_B \, \boldsymbol{\omega}_B) \right)$$

### 5.5 欠驱动特性 (Underactuation)

旋翼飞行器具有 **6 个自由度** (3 平动 + 3 转动)，但只有 **4 个独立控制输入** ($T, L, M, N$)。这意味着：

- 飞行器是**欠驱动系统 (Underactuated System)**
- 不能独立控制所有 6 个自由度
- 水平位置 ($x, y$) 的控制必须通过**倾斜机体**来实现

具体而言：
- 要向前飞行 ($+x$ 方向)，需要先**俯仰 (pitch)** 使推力向量产生水平分量
- 要向左飞行 ($+y$ 方向)，需要先**滚转 (roll)**
- 偏航 ($\psi$) 通过旋翼反扭矩差控制

这种耦合关系是旋翼飞行器控制设计中的核心挑战，也是采用**级联控制 (Cascaded Control)** 架构的根本原因。

### 5.6 悬停平衡点 (Hover Equilibrium)

在悬停状态下，所有速度和加速度为零：

$$\dot{\mathbf{r}}_{IB} = \mathbf{0}, \quad \ddot{\mathbf{r}}_{IB} = \mathbf{0}, \quad \boldsymbol{\omega}_B = \mathbf{0}, \quad \dot{\boldsymbol{\omega}}_B = \mathbf{0}$$

代入平动方程可得悬停推力：

$$T_{\text{hover}} = mg$$

此时姿态为水平 ($\phi = \theta = 0$)，每个旋翼的悬停转速为：

$$\omega_{\text{hover}} = \sqrt{\frac{mg}{n \, k_F}}$$

---

## 6. 控制架构 (Control Architecture)

### 6.1 级联控制结构 (Cascaded Control)

由于旋翼飞行器的欠驱动特性，控制器通常采用**级联 (Cascaded)** 或**分层 (Hierarchical)** 结构，将控制问题分解为多个层次：

```
位置参考 --> [位置控制器] --> 期望姿态 --> [姿态控制器] --> 期望力矩 --> [分配矩阵] --> 电机转速
  (外环)                       (内环)                        (执行器)
```

**设计原则**: 内环（姿态控制）的带宽必须**显著高于**外环（位置控制）的带宽，以实现**时间尺度分离 (Time-Scale Separation)**。典型比例为：

- 姿态内环带宽: $\sim 20\text{-}50 \, \text{rad/s}$
- 位置外环带宽: $\sim 2\text{-}10 \, \text{rad/s}$

### 6.2 位置控制器 (Outer Loop — Position Controller)

位置控制器的目标是跟踪期望位置轨迹 $\mathbf{r}_{\text{des}}(t)$。

定义位置误差：

$$\mathbf{e}_r = \mathbf{r}_{IB} - \mathbf{r}_{\text{des}}$$

$$\dot{\mathbf{e}}_r = \dot{\mathbf{r}}_{IB} - \dot{\mathbf{r}}_{\text{des}}$$

**PID 位置控制器**计算期望的加速度：

$$\ddot{\mathbf{r}}_{\text{cmd}} = \ddot{\mathbf{r}}_{\text{des}} - K_p^{\text{pos}} \, \mathbf{e}_r - K_d^{\text{pos}} \, \dot{\mathbf{e}}_r - K_i^{\text{pos}} \int \mathbf{e}_r \, dt$$

其中 $K_p^{\text{pos}}, K_d^{\text{pos}}, K_i^{\text{pos}}$ 分别是比例、微分、积分增益矩阵。

由平动方程 $m \ddot{\mathbf{r}} = \mathbf{C}_{IB} [0, 0, T]^T + m\mathbf{g}$，可以求出期望的推力向量：

$$\mathbf{F}_{\text{des}} = m(\ddot{\mathbf{r}}_{\text{cmd}} - \mathbf{g})$$

从期望推力向量中提取：

**总推力大小**:

$$T_{\text{cmd}} = \| \mathbf{F}_{\text{des}} \|$$

**期望姿态**: 由 $\mathbf{F}_{\text{des}}$ 的方向确定期望的机体 $z_B$ 轴方向，结合期望偏航角 $\psi_{\text{des}}$，可以构造完整的期望旋转矩阵 $\mathbf{C}_{IB}^{\text{des}}$，进而提取期望 Euler 角：

$$\phi_{\text{des}} = \arcsin\left(\frac{F_{\text{des},x} \sin\psi_{\text{des}} - F_{\text{des},y} \cos\psi_{\text{des}}}{T_{\text{cmd}}}\right)$$

$$\theta_{\text{des}} = \arctan\left(\frac{F_{\text{des},x} \cos\psi_{\text{des}} + F_{\text{des},y} \sin\psi_{\text{des}}}{F_{\text{des},z}}\right)$$

> **关键洞察**: 位置控制器的输出不是直接的力，而是**期望姿态**。这正是欠驱动特性的体现——水平力只能通过倾斜机体来产生。

### 6.3 姿态控制器 (Inner Loop — Attitude Controller)

姿态控制器的目标是快速跟踪位置控制器给出的期望姿态。

**方法一: PD 角速度 + P 角度控制器**

定义姿态误差：

$$\mathbf{e}_\phi = \boldsymbol{\phi} - \boldsymbol{\phi}_{\text{des}}$$

期望角速度：

$$\boldsymbol{\omega}_{\text{des}} = -K_p^{\text{att}} \, \mathbf{e}_\phi$$

角速度误差：

$$\mathbf{e}_\omega = \boldsymbol{\omega}_B - \boldsymbol{\omega}_{\text{des}}$$

控制力矩：

$$\mathbf{M}_{\text{cmd}} = -K_p^{\omega} \, \mathbf{e}_\omega - K_d^{\omega} \, \dot{\mathbf{e}}_\omega + \boldsymbol{\omega}_B \times (\mathbf{I}_B \, \boldsymbol{\omega}_B)$$

最后一项是**前馈补偿 (Feedforward Compensation)**，用于抵消陀螺耦合项。

**方法二: 全 PID 姿态控制器**

$$\begin{bmatrix} L_{\text{cmd}} \ M_{\text{cmd}} \ N_{\text{cmd}} \end{bmatrix} = -K_p^{\text{att}} \, \mathbf{e}_\phi - K_d^{\text{att}} \, \dot{\mathbf{e}}_\phi - K_i^{\text{att}} \int \mathbf{e}_\phi \, dt$$

其中各增益矩阵通常取为对角矩阵，可以独立调节各轴的响应特性。

### 6.4 PID 增益调节指南

对于悬停附近的线性化模型，各轴近似解耦。以 Roll 轴为例：

$$I_{xx} \ddot{\phi} = L$$

这是一个双积分器 (Double Integrator)。PD 控制器 $L = -K_p \phi - K_d \dot{\phi}$ 的闭环特征方程为：

$$s^2 + \frac{K_d}{I_{xx}} s + \frac{K_p}{I_{xx}} = 0$$

与标准二阶系统 $s^2 + 2\zeta\omega_n s + \omega_n^2 = 0$ 对比，可得：

$$K_p = I_{xx} \omega_n^2, \quad K_d = 2 \zeta \omega_n I_{xx}$$

其中 $\omega_n$ 是自然频率，$\zeta$ 是阻尼比（通常取 $\zeta = 0.7 \sim 1.0$）。

### 6.5 推力分配 (Thrust Allocation)

控制器输出期望的总推力和力矩 $\mathbf{w}_{\text{cmd}} = [T_{\text{cmd}}, L_{\text{cmd}}, M_{\text{cmd}}, N_{\text{cmd}}]^T$ 后，需要通过分配矩阵将其转换为各旋翼的转速指令：

**四旋翼 (方阵情况)**:

$$\boldsymbol{\gamma}_{\text{cmd}} = \mathbf{A}^{-1} \, \mathbf{w}_{\text{cmd}}$$

$$\omega_i^{\text{cmd}} = \sqrt{\gamma_i^{\text{cmd}}}$$

**六旋翼/八旋翼 (冗余情况)**:

$$\boldsymbol{\gamma}_{\text{cmd}} = \mathbf{A}^\dagger \, \mathbf{w}_{\text{cmd}}$$

其中 $\mathbf{A}^\dagger = \mathbf{A}^T(\mathbf{A}\mathbf{A}^T)^{-1}$ 是 Moore-Penrose 伪逆。

**转速饱和处理**: 若计算得到的 $\omega_i^{\text{cmd}}$ 超出电机转速范围 $[\omega_{\min}, \omega_{\max}]$，需要进行饱和限幅。常见策略：

1. **简单截断 (Clipping)**: 直接限幅，但会改变力矩方向
2. **等比缩放 (Proportional Scaling)**: 保持力矩方向不变，等比缩小所有转速
3. **优先级分配 (Priority Allocation)**: 优先满足总推力，再分配力矩

### 6.6 完整控制框图

```
                    ┌─────────────────────────────────────────────────┐
                    │              外环 (Outer Loop)                   │
  r_des ──> [+] ──>│  位置 PID  ──> F_des ──> 姿态提取              │──> φ_des
             │-     │                                                 │     T_cmd
             │      └─────────────────────────────────────────────────┘
           r_IB                                                          │
             ^      ┌─────────────────────────────────────────────────┐  │
             │      │              内环 (Inner Loop)                   │  │
  φ_des ──> [+] ──>│  姿态 PID  ──> M_cmd                           │──┤
             │-     │                                                 │  │
             │      └─────────────────────────────────────────────────┘  │
           φ_meas                                                        │
                    ┌─────────────────────────────────────────────────┐  │
                    │           推力分配 (Allocation)                  │  │
  [T,L,M,N] <──────│  w_cmd = [T_cmd; M_cmd]                        │<─┘
                    │  γ = A^(-1) * w_cmd                            │
                    │  ω_i = sqrt(γ_i)                               │──> 电机
                    └─────────────────────────────────────────────────┘
```

---

## 7. 六旋翼案例研究 (Hexacopter Case Study)

### 7.1 旋翼布局

六旋翼飞行器的旋翼均匀分布在以质心为圆心、半径为 $d$ 的圆上，相邻旋翼间隔 $60°$。

```
          旋翼1 (CCW)
            ▲
           / \
          /   \
  旋翼2  /     \  旋翼6
  (CW)  ·       · (CW)
         \     /
          \   /
  旋翼3    \ /    旋翼5
  (CCW)    ▼     (CCW)
          旋翼4 (CW)
```

各旋翼的位置和旋转方向如下表所示：

| 旋翼 $i$ | 角度 $\alpha_i$ | 位置 $x_i = d\cos\alpha_i$ | 位置 $y_i = d\sin\alpha_i$ | 旋转方向 | $\sigma_i$ |
|-----------|-----------------|----------------------------|----------------------------|----------|------------|
| 1 | $90°$ | $0$ | $d$ | CCW | $+1$ |
| 2 | $150°$ | $-\frac{\sqrt{3}}{2}d$ | $\frac{1}{2}d$ | CW | $-1$ |
| 3 | $210°$ | $-\frac{\sqrt{3}}{2}d$ | $-\frac{1}{2}d$ | CCW | $+1$ |
| 4 | $270°$ | $0$ | $-d$ | CW | $-1$ |
| 5 | $330°$ | $\frac{\sqrt{3}}{2}d$ | $-\frac{1}{2}d$ | CCW | $+1$ |
| 6 | $30°$ | $\frac{\sqrt{3}}{2}d$ | $\frac{1}{2}d$ | CW | $-1$ |

> **旋转方向规则**: 相邻旋翼旋转方向交替排列 (CCW-CW-CCW-CW-CCW-CW)，以平衡悬停时的偏航力矩。

### 7.2 分配矩阵的构建

将上表中的参数代入分配矩阵公式，得到六旋翼的分配矩阵 $\mathbf{A} \in \mathbb{R}^{4 \times 6}$：

$$\mathbf{A}_{\text{hex}} = \begin{bmatrix} k_F & k_F & k_F & k_F & k_F & k_F \ d k_F & \frac{1}{2}d k_F & -\frac{1}{2}d k_F & -d k_F & -\frac{1}{2}d k_F & \frac{1}{2}d k_F \ 0 & \frac{\sqrt{3}}{2}d k_F & \frac{\sqrt{3}}{2}d k_F & 0 & -\frac{\sqrt{3}}{2}d k_F & -\frac{\sqrt{3}}{2}d k_F \ k_M & -k_M & k_M & -k_M & k_M & -k_M \end{bmatrix}$$

提取公因子后可简化为：

$$\mathbf{A}_{\text{hex}} = \begin{bmatrix} 1 & 1 & 1 & 1 & 1 & 1 \ 0 & \frac{\sqrt{3}}{2} & \frac{\sqrt{3}}{2} & 0 & -\frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \ 1 & \frac{1}{2} & -\frac{1}{2} & -1 & -\frac{1}{2} & \frac{1}{2} \ \kappa & -\kappa & \kappa & -\kappa & \kappa & -\kappa \end{bmatrix} \cdot \text{diag}(k_F)$$

> **注意**: 上式中第二行和第三行的具体形式取决于旋翼编号起始角度的约定，不同文献可能有所不同，但物理本质一致。

### 7.3 伪逆求解

由于 $\mathbf{A}_{\text{hex}}$ 是 $4 \times 6$ 矩阵（行数 < 列数），方程 $\mathbf{w} = \mathbf{A}_{\text{hex}} \boldsymbol{\gamma}$ 有无穷多解。使用 Moore-Penrose 伪逆：

$$\boldsymbol{\gamma}^* = \mathbf{A}_{\text{hex}}^{\dagger} \, \mathbf{w} = \mathbf{A}_{\text{hex}}^T \left(\mathbf{A}_{\text{hex}} \mathbf{A}_{\text{hex}}^T\right)^{-1} \mathbf{w}$$

此解满足：
1. $\mathbf{A}_{\text{hex}} \boldsymbol{\gamma}^* = \mathbf{w}$（精确满足推力和力矩需求）
2. $\|\boldsymbol{\gamma}^*\|_2$ 最小（最小化转速平方和，即最小化总功耗的近似）

### 7.4 零空间与冗余利用

$\mathbf{A}_{\text{hex}}$ 的零空间 (Null Space) 维度为 $n - \text{rank}(\mathbf{A}) = 6 - 4 = 2$。这意味着存在 2 维的自由度可以在不影响总推力和力矩的前提下调整各旋翼转速。

通用解为：

$$\boldsymbol{\gamma} = \boldsymbol{\gamma}^* + \mathbf{N} \, \boldsymbol{\lambda}$$

其中 $\mathbf{N} \in \mathbb{R}^{6 \times 2}$ 是零空间基矩阵，$\boldsymbol{\lambda} \in \mathbb{R}^2$ 是自由参数。

零空间的利用场景：
- **避免转速饱和**: 调整 $\boldsymbol{\lambda}$ 使所有 $\gamma_i$ 远离上下限
- **最小化最大转速**: 均衡各旋翼负载
- **容错控制**: 当某旋翼失效时，利用零空间重新分配

### 7.5 容错分析 (Fault Tolerance)

六旋翼相比四旋翼的核心优势在于**容错能力**。

**单旋翼失效场景**: 假设旋翼 $j$ 失效（$\omega_j = 0$），则从分配矩阵中删除第 $j$ 列，得到降级分配矩阵 $\mathbf{A}_{\text{red}} \in \mathbb{R}^{4 \times 5}$。

由于 $5 > 4$，$\mathbf{A}_{\text{red}}$ 仍然是超定的（列数 > 行数），系统仍可通过伪逆求解：

$$\boldsymbol{\gamma}_{\text{red}} = \mathbf{A}_{\text{red}}^{\dagger} \, \mathbf{w}$$

此时系统仍然**完全可控**，但：
- 最大可用推力降低约 $1/6$
- 力矩包络 (moment envelope) 不再对称
- 某些方向的机动能力下降

**双旋翼失效场景**: 若两个旋翼失效，降级矩阵为 $4 \times 4$。
- 若失效的两个旋翼**不相邻**（如旋翼 1 和 4），$\mathbf{A}_{\text{red}}$ 通常仍满秩，系统可控
- 若失效的两个旋翼**相邻**，可能导致某个力矩通道不可控

**可控性判据**: 降级后系统可控的充要条件是 $\text{rank}(\mathbf{A}_{\text{red}}) = 4$。

### 7.6 数值示例

考虑一个典型六旋翼的参数：

| 参数 | 符号 | 值 | 单位 |
|------|------|-----|------|
| 质量 | $m$ | $2.0$ | kg |
| 臂长 | $d$ | $0.3$ | m |
| 推力系数 | $k_F$ | $8.55 \times 10^{-6}$ | $\text{N}\cdot\text{s}^2/\text{rad}^2$ |
| 扭矩系数 | $k_M$ | $1.36 \times 10^{-7}$ | $\text{N}\cdot\text{m}\cdot\text{s}^2/\text{rad}^2$ |
| 转动惯量 $I_{xx}$ | $I_{xx}$ | $0.025$ | $\text{kg}\cdot\text{m}^2$ |
| 转动惯量 $I_{yy}$ | $I_{yy}$ | $0.025$ | $\text{kg}\cdot\text{m}^2$ |
| 转动惯量 $I_{zz}$ | $I_{zz}$ | $0.045$ | $\text{kg}\cdot\text{m}^2$ |
| 最大转速 | $\omega_{\max}$ | $8000$ | rpm |

**悬停转速计算**:

$$\omega_{\text{hover}} = \sqrt{\frac{mg}{n \, k_F}} = \sqrt{\frac{2.0 \times 9.81}{6 \times 8.55 \times 10^{-6}}} \approx 618 \, \text{rad/s} \approx 5900 \, \text{rpm}$$

**推力裕度 (Thrust Margin)**:

$$\text{Margin} = \frac{n \, k_F \, \omega_{\max}^2}{mg} - 1$$

其中 $\omega_{\max} = 8000 \times \frac{2\pi}{60} \approx 838 \, \text{rad/s}$：

$$\text{Margin} = \frac{6 \times 8.55 \times 10^{-6} \times 838^2}{2.0 \times 9.81} - 1 \approx 1.84$$

即最大推力约为悬停推力的 $2.84$ 倍，推力裕度为 $184\%$，这为机动飞行和容错提供了充足的余量。

---

## 8. 关键公式总结

本节汇总 Lecture 9 中最重要的公式，便于复习和考试参考。

### 8.1 旋翼力学

| 描述 | 公式 |
|------|------|
| 单旋翼推力 | $F_i = k_F \, \omega_i^2$ |
| 单旋翼反扭矩 | $M_i = k_M \, \omega_i^2$ |
| 推力向量 (机体系) | $\mathbf{F}_i^B = [0, \; 0, \; k_F \omega_i^2]^T$ |
| 反扭矩向量 (机体系) | $\mathbf{M}_i^B = [0, \; 0, \; \sigma_i k_M \omega_i^2]^T$ |
| 推力-扭矩比 | $\kappa = k_M / k_F$ |

### 8.2 力与力矩合成

| 描述 | 公式 |
|------|------|
| 总推力 | $T = \sum_{i=1}^{n} k_F \omega_i^2$ |
| Roll 力矩 | $L = \sum_{i=1}^{n} y_i \, k_F \omega_i^2$ |
| Pitch 力矩 | $M = \sum_{i=1}^{n} (-x_i) \, k_F \omega_i^2$ |
| Yaw 力矩 | $N = \sum_{i=1}^{n} \sigma_i \, k_M \omega_i^2$ |
| 分配关系 | $\mathbf{w} = \mathbf{A} \, \boldsymbol{\gamma}$ |

### 8.3 运动方程

| 描述 | 公式 |
|------|------|
| 平动方程 (惯性系) | $m \ddot{\mathbf{r}}_{IB} = \mathbf{C}_{IB} [0, 0, T]^T + m\mathbf{g}$ |
| 转动方程 (机体系) | $\mathbf{I}_B \dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (\mathbf{I}_B \boldsymbol{\omega}_B) = \mathbf{M}_{\text{total}}^B$ |
| 运动学 — 平动 | $\dot{\mathbf{r}}_{IB} = \mathbf{C}_{IB} \mathbf{v}_B$ |
| 运动学 — 转动 | $\dot{\boldsymbol{\phi}} = \mathbf{E}(\boldsymbol{\phi}) \boldsymbol{\omega}_B$ |
| 悬停推力 | $T_{\text{hover}} = mg$ |
| 悬停转速 | $\omega_{\text{hover}} = \sqrt{mg / (n \, k_F)}$ |

### 8.4 控制

| 描述 | 公式 |
|------|------|
| 位置 PID | $\ddot{\mathbf{r}}_{\text{cmd}} = \ddot{\mathbf{r}}_{\text{des}} - K_p \mathbf{e}_r - K_d \dot{\mathbf{e}}_r - K_i \int \mathbf{e}_r \, dt$ |
| 期望推力向量 | $\mathbf{F}_{\text{des}} = m(\ddot{\mathbf{r}}_{\text{cmd}} - \mathbf{g})$ |
| 姿态 PD + 前馈 | $\mathbf{M}_{\text{cmd}} = -K_p^\omega \mathbf{e}_\omega - K_d^\omega \dot{\mathbf{e}}_\omega + \boldsymbol{\omega}_B \times (\mathbf{I}_B \boldsymbol{\omega}_B)$ |
| PD 增益设计 | $K_p = I \omega_n^2, \quad K_d = 2\zeta\omega_n I$ |
| 四旋翼分配 | $\boldsymbol{\gamma} = \mathbf{A}^{-1} \mathbf{w}$ |
| 冗余系统分配 | $\boldsymbol{\gamma} = \mathbf{A}^\dagger \mathbf{w} = \mathbf{A}^T(\mathbf{A}\mathbf{A}^T)^{-1}\mathbf{w}$ |

### 8.5 六旋翼容错

| 描述 | 公式 / 条件 |
|------|-------------|
| 零空间维度 | $\dim(\text{null}(\mathbf{A})) = n - \text{rank}(\mathbf{A}) = 6 - 4 = 2$ |
| 通用分配解 | $\boldsymbol{\gamma} = \mathbf{A}^\dagger \mathbf{w} + \mathbf{N}\boldsymbol{\lambda}$ |
| 单旋翼失效可控条件 | $\text{rank}(\mathbf{A}_{\text{red}}) = 4$ （始终满足） |
| 双旋翼失效可控条件 | $\text{rank}(\mathbf{A}_{\text{red}}) = 4$ （取决于失效组合） |

---

## 附录 A: 旋转矩阵基本元素

$$\mathbf{R}_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \ 0 & \cos\phi & -\sin\phi \ 0 & \sin\phi & \cos\phi \end{bmatrix}$$

$$\mathbf{R}_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \ 0 & 1 & 0 \ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

$$\mathbf{R}_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \ \sin\psi & \cos\psi & 0 \ 0 & 0 & 1 \end{bmatrix}$$

## 附录 B: 叉积的反对称矩阵表示

对于向量 $\mathbf{a} = [a_1, a_2, a_3]^T$，叉积 $\mathbf{a} \times \mathbf{b}$ 可以写为矩阵乘法：

$$\mathbf{a} \times \mathbf{b} = [\mathbf{a}]_\times \, \mathbf{b}$$

其中反对称矩阵 (Skew-Symmetric Matrix)：

$$[\mathbf{a}]_\times = \begin{bmatrix} 0 & -a_3 & a_2 \ a_3 & 0 & -a_1 \ -a_2 & a_1 & 0 \end{bmatrix}$$

此表示在推导力矩方程和 Jacobian 时非常有用。

## 附录 C: 符号表 (Nomenclature)

| 符号 | 含义 | 单位 |
|------|------|------|
| $\{I\}$ | 惯性坐标系 (Inertial Frame) | — |
| $\{B\}$ | 机体坐标系 (Body Frame) | — |
| $\mathbf{r}_{IB}$ | 机体质心在惯性系中的位置 | m |
| $\mathbf{v}_B$ | 质心速度 (机体系) | m/s |
| $\boldsymbol{\omega}_B$ | 角速度 (机体系) | rad/s |
| $\mathbf{C}_{IB}$ | 机体系到惯性系的旋转矩阵 | — |
| $\phi, \theta, \psi$ | Roll, Pitch, Yaw 角 | rad |
| $m$ | 飞行器质量 | kg |
| $\mathbf{I}_B$ | 惯性张量 (机体系) | $\text{kg}\cdot\text{m}^2$ |
| $k_F$ | 推力系数 | $\text{N}\cdot\text{s}^2/\text{rad}^2$ |
| $k_M$ | 扭矩系数 | $\text{N}\cdot\text{m}\cdot\text{s}^2/\text{rad}^2$ |
| $\omega_i$ | 第 $i$ 个旋翼转速 | rad/s |
| $T$ | 总推力 | N |
| $L, M, N$ | Roll, Pitch, Yaw 力矩 | N·m |
| $\mathbf{A}$ | 分配矩阵 (Allocation Matrix) | — |
| $d$ | 旋翼臂长 | m |
| $\sigma_i$ | 旋翼旋转方向因子 ($\pm 1$) | — |
| $g$ | 重力加速度 ($9.81$) | $\text{m/s}^2$ |

---

> **参考文献**:
> - Mahony, R., Kumar, V., & Corke, P. (2012). *Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor.* IEEE Robotics & Automation Magazine.
> - Mellinger, D., & Kumar, V. (2011). *Minimum Snap Trajectory Generation and Control for Quadrotors.* ICRA.
> - ETH Zurich, Robot Dynamics Lecture Series.

---

*ETH Zurich — Robot Dynamics — Lecture 9 讲义整理完毕*
