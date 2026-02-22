# Lecture 11: 课程总结与考试复习 (Course Summary & Exam Review)

> ETH Zurich — Robot Dynamics  
> 综合复习笔记 | Comprehensive Review Notes

---

## 目录 (Table of Contents)

1. [运动学总结 (Kinematics Summary)](#1-运动学总结-kinematics-summary)
2. [动力学总结 (Dynamics Summary)](#2-动力学总结-dynamics-summary)
3. [控制方法总结 (Control Methods Summary)](#3-控制方法总结-control-methods-summary)
4. [浮动基座系统 (Floating Base Systems)](#4-浮动基座系统-floating-base-systems)
5. [足式机器人 (Legged Robots)](#5-足式机器人-legged-robots)
6. [旋翼飞行器 (Rotorcraft)](#6-旋翼飞行器-rotorcraft)
7. [固定翼飞行器 (Fixed-Wing Aircraft)](#7-固定翼飞行器-fixed-wing-aircraft)
8. [考试要点 (Exam Key Points)](#8-考试要点-exam-key-points)

---

## 1. 运动学总结 (Kinematics Summary)

运动学研究物体的运动几何关系，不涉及力和质量。它是机器人学的基础，为后续动力学和控制提供几何框架。

### 1.1 位置参数化 (Position Parameterization)

三维空间中一个点的位置可以用不同的坐标系来描述：

#### 笛卡尔坐标 (Cartesian Coordinates)

最直观的表示方法，用三个正交轴上的投影来描述位置：

$$\mathbf{r} = \begin{pmatrix} x \ y \ z \end{pmatrix} \in \mathbb{R}^3$$

- 优点：直观、线性、无奇异性
- 缺点：对某些几何结构（如圆柱形工作空间）不够自然

#### 柱坐标 (Cylindrical Coordinates)

适用于具有旋转对称性的系统：

$$\begin{cases} x = \rho \cos\varphi \ y = \rho \sin\varphi \ z = z \end{cases}$$

其中 $\rho \geq 0$ 为径向距离，$\varphi \in [0, 2\pi)$ 为方位角，$z$ 为高度。

#### 球坐标 (Spherical Coordinates)

适用于以某点为中心的径向运动描述：

$$\begin{cases} x = r\sin\theta\cos\varphi \ y = r\sin\theta\sin\varphi \ z = r\cos\theta \end{cases}$$

其中 $r \geq 0$ 为径向距离，$\theta \in [0, \pi]$ 为极角，$\varphi \in [0, 2\pi)$ 为方位角。

> **注意**：柱坐标和球坐标在 $\rho = 0$ 或 $r = 0$ 处存在奇异性（singularity），方位角 $\varphi$ 不唯一。

---

### 1.2 旋转参数化 (Rotation Parameterization)

刚体的姿态（orientation）描述是机器人学中的核心问题。旋转群 $SO(3)$ 是一个三维流形，但不同的参数化方法各有优劣。

#### 1.2.1 旋转矩阵 (Rotation Matrix)

旋转矩阵 $C \in SO(3)$ 满足：

$$SO(3) = \{ C \in \mathbb{R}^{3\times3} \mid C^T C = I, \; \det(C) = +1 \}$$

**基本旋转矩阵**（绕各坐标轴旋转）：

$$C_x(\alpha) = \begin{pmatrix} 1 & 0 & 0 \ 0 & \cos\alpha & -\sin\alpha \ 0 & \sin\alpha & \cos\alpha \end{pmatrix}$$

$$C_y(\beta) = \begin{pmatrix} \cos\beta & 0 & \sin\beta \ 0 & 1 & 0 \ -\sin\beta & 0 & \cos\beta \end{pmatrix}$$

$$C_z(\gamma) = \begin{pmatrix} \cos\gamma & -\sin\gamma & 0 \ \sin\gamma & \cos\gamma & 0 \ 0 & 0 & 1 \end{pmatrix}$$

**性质**：
- 9 个参数，6 个约束 → 3 个自由度
- 旋转的复合：$C_{AC} = C_{AB} \cdot C_{BC}$
- 逆旋转：$C_{BA} = C_{AB}^T = C_{AB}^{-1}$
- 无奇异性，但参数冗余

#### 1.2.2 欧拉角 (Euler Angles)

用三次连续旋转来描述姿态，共有 12 种约定（如 ZYX, ZYZ 等）。以 **ZYX 欧拉角**（也称 yaw-pitch-roll）为例：

$$C = C_z(\gamma) \cdot C_y(\beta) \cdot C_x(\alpha)$$

其中 $\gamma$ 为偏航角（yaw），$\beta$ 为俯仰角（pitch），$\alpha$ 为滚转角（roll）。

**欧拉角速度与角速度的关系**：

$$\boldsymbol{\omega} = E(\boldsymbol{\Phi}) \dot{\boldsymbol{\Phi}}$$

其中 $E(\boldsymbol{\Phi})$ 为映射矩阵，依赖于具体的欧拉角约定。

**万向锁 (Gimbal Lock)**：当 $\beta = \pm\frac{\pi}{2}$ 时，$E(\boldsymbol{\Phi})$ 奇异（不可逆），丢失一个自由度。这是所有三参数表示的固有缺陷（由拓扑学中的"毛球定理" Hairy Ball Theorem 保证）。

#### 1.2.3 轴角表示 (Angle-Axis Representation)

任何旋转都可以用一个旋转轴 $\mathbf{n}$（单位向量）和旋转角 $\varphi$ 来描述：

$$\mathbf{p} = \varphi \mathbf{n}, \quad \|\mathbf{n}\| = 1, \quad \varphi \in [0, \pi]$$

**Rodrigues 旋转公式**：

$$C(\varphi, \mathbf{n}) = I + \sin\varphi \; [\mathbf{n}]_\times + (1 - \cos\varphi) \; [\mathbf{n}]_\times^2$$

其中 $[\mathbf{n}]_\times$ 为反对称矩阵（skew-symmetric matrix）：

$$[\mathbf{n}]_\times = \begin{pmatrix} 0 & -n_z & n_y \ n_z & 0 & -n_x \ -n_y & n_x & 0 \end{pmatrix}$$

**反对称矩阵的重要性质**：
- $[\mathbf{a}]_\times \mathbf{b} = \mathbf{a} \times \mathbf{b}$（叉积）
- $[\mathbf{a}]_\times^T = -[\mathbf{a}]_\times$
- $C[\mathbf{a}]_\times C^T = [C\mathbf{a}]_\times$

#### 1.2.4 单位四元数 (Unit Quaternions)

四元数提供了旋转的最紧凑无奇异表示（4 个参数，1 个约束）：

$$\mathbf{q} = q_0 + q_1 \mathbf{i} + q_2 \mathbf{j} + q_3 \mathbf{k} = \begin{pmatrix} q_0 \ \mathbf{q}_v \end{pmatrix}, \quad \|\mathbf{q}\| = 1$$

其中 $q_0 = \cos\frac{\varphi}{2}$，$\mathbf{q}_v = \sin\frac{\varphi}{2} \; \mathbf{n}$。

**四元数乘法**（Hamilton 积）：

$$\mathbf{p} \otimes \mathbf{q} = \begin{pmatrix} p_0 q_0 - \mathbf{p}_v^T \mathbf{q}_v \ p_0 \mathbf{q}_v + q_0 \mathbf{p}_v + \mathbf{p}_v \times \mathbf{q}_v \end{pmatrix}$$

**四元数到旋转矩阵**：

$$C(\mathbf{q}) = (2q_0^2 - 1)I + 2q_0 [\mathbf{q}_v]_\times + 2\mathbf{q}_v \mathbf{q}_v^T$$

**四元数的优点**：
- 无万向锁（no gimbal lock）
- 插值方便（SLERP 球面线性插值）
- 计算效率高（旋转复合只需四元数乘法）
- 数值稳定（只需归一化约束）

**双覆盖性**：$\mathbf{q}$ 和 $-\mathbf{q}$ 表示同一旋转。

#### 1.2.5 各种表示之间的转换总结

| 从 \ 到 | 旋转矩阵 | 欧拉角 | 轴角 | 四元数 |
|---------|----------|--------|------|--------|
| 旋转矩阵 | — | 反三角函数提取 | $\varphi = \arccos\frac{\text{tr}(C)-1}{2}$ | 特征值分解 |
| 欧拉角 | 矩阵连乘 | — | 先转矩阵再提取 | 先转矩阵再提取 |
| 轴角 | Rodrigues 公式 | 先转矩阵 | — | $q_0=\cos\frac{\varphi}{2}, \mathbf{q}_v=\sin\frac{\varphi}{2}\mathbf{n}$ |
| 四元数 | $C(\mathbf{q})$ 公式 | 先转矩阵 | $\varphi=2\arccos q_0$ | — |

---

### 1.3 正运动学 (Forward Kinematics)

正运动学建立关节空间到任务空间的映射：给定关节角 $\mathbf{q}$，求末端执行器的位姿。

#### 齐次变换矩阵 (Homogeneous Transformation Matrix)

$$T_{AB} = \begin{pmatrix} C_{AB} & {}_A\mathbf{r}_{AB} \ \mathbf{0}^T & 1 \end{pmatrix} \in SE(3)$$

其中 $C_{AB} \in SO(3)$ 为旋转矩阵，${}_A\mathbf{r}_{AB} \in \mathbb{R}^3$ 为平移向量。

**性质**：
- 变换的复合：$T_{AC} = T_{AB} \cdot T_{BC}$
- 逆变换：$T_{BA} = T_{AB}^{-1} = \begin{pmatrix} C_{AB}^T & -C_{AB}^T {}_A\mathbf{r}_{AB} \ \mathbf{0}^T & 1 \end{pmatrix}$

#### 串联机器人的正运动学

对于 $n$ 个关节的串联机器人：

$$T_{0n}(\mathbf{q}) = T_{01}(q_1) \cdot T_{12}(q_2) \cdots T_{(n-1)n}(q_n)$$

每个关节变换 $T_{(i-1)i}(q_i)$ 由 DH 参数或其他约定确定。

---

### 1.4 微分运动学 (Differential Kinematics)

微分运动学研究关节速度与末端执行器速度之间的关系，核心工具是雅可比矩阵。

#### 1.4.1 解析雅可比 (Analytical Jacobian)

从正运动学的直接微分得到。设任务空间坐标为 $\mathbf{x}(\mathbf{q})$（包含位置和欧拉角）：

$$\dot{\mathbf{x}} = J_A(\mathbf{q}) \dot{\mathbf{q}}$$

其中 $J_A = \frac{\partial \mathbf{x}}{\partial \mathbf{q}} \in \mathbb{R}^{6 \times n}$。

**注意**：解析雅可比依赖于姿态参数化的选择（如 ZYX 欧拉角），且在欧拉角奇异点处不可用。

#### 1.4.2 几何雅可比 (Geometric Jacobian)

直接将关节速度映射到末端的线速度和角速度：

$$\begin{pmatrix} \mathbf{v} \ \boldsymbol{\omega} \end{pmatrix} = J_G(\mathbf{q}) \dot{\mathbf{q}} = \begin{pmatrix} J_P \ J_R \end{pmatrix} \dot{\mathbf{q}}$$

其中 $\mathbf{v}$ 为线速度，$\boldsymbol{\omega}$ 为角速度，$J_P \in \mathbb{R}^{3 \times n}$ 为平移部分，$J_R \in \mathbb{R}^{3 \times n}$ 为旋转部分。

**两种雅可比的关系**：

$$J_A = \begin{pmatrix} I & 0 \ 0 & E^{-1} \end{pmatrix} J_G$$

其中 $E$ 为欧拉角速度到角速度的映射矩阵。

#### 对于旋转关节 (Revolute Joint) $i$：

$$J_P^{(i)} = \mathbf{e}_i \times (\mathbf{r}_{0E} - \mathbf{r}_{0i}), \quad J_R^{(i)} = \mathbf{e}_i$$

其中 $\mathbf{e}_i$ 为关节 $i$ 的旋转轴方向，$\mathbf{r}_{0E}$ 为末端位置，$\mathbf{r}_{0i}$ 为关节 $i$ 的位置。

#### 对于移动关节 (Prismatic Joint) $i$：

$$J_P^{(i)} = \mathbf{e}_i, \quad J_R^{(i)} = \mathbf{0}$$

#### 1.4.3 雅可比矩阵的重要性质

- **奇异性 (Singularity)**：当 $\text{rank}(J) < \min(m, n)$ 时，雅可比矩阵奇异，某些方向的运动不可实现
- **可操作度 (Manipulability)**：$w = \sqrt{\det(J J^T)}$，衡量机器人在当前构型下的运动能力
- **力映射**：$\boldsymbol{\tau} = J^T \mathbf{F}$（静力学对偶关系）

---

### 1.5 逆运动学 (Inverse Kinematics)

给定期望的末端位姿，求解关节角。这是正运动学的逆问题。

#### 1.5.1 封闭解 (Closed-Form / Analytical Solutions)

对于特定结构的机器人（如具有球形手腕的 6-DOF 机器人），可以利用几何关系直接求解：

- **位置逆解**：利用几何关系求前三个关节角
- **姿态逆解**：利用球形手腕的解耦特性求后三个关节角
- 通常存在多组解（如肘上/肘下、左手/右手构型）

#### 1.5.2 数值迭代法 (Numerical Methods)

对于一般结构的机器人，使用迭代方法：

**基于雅可比的方法**：

$$\Delta \mathbf{q} = J^{\dagger} \Delta \mathbf{x}$$

其中 $J^{\dagger}$ 为伪逆（pseudoinverse）：

- 当 $n = m$（方阵）：$J^{\dagger} = J^{-1}$
- 当 $n > m$（冗余）：$J^{\dagger} = J^T(JJ^T)^{-1}$（右伪逆，最小范数解）
- 当 $n < m$（欠驱动）：$J^{\dagger} = (J^TJ)^{-1}J^T$（左伪逆，最小二乘解）

**阻尼最小二乘法 (Damped Least Squares / Levenberg-Marquardt)**：

$$\Delta \mathbf{q} = J^T(JJ^T + \lambda^2 I)^{-1} \Delta \mathbf{x}$$

在奇异点附近通过阻尼因子 $\lambda$ 保证数值稳定性，代价是精度略有下降。

**冗余机器人的零空间投影**：

$$\dot{\mathbf{q}} = J^{\dagger} \dot{\mathbf{x}} + (I - J^{\dagger}J) \mathbf{z}$$

其中 $(I - J^{\dagger}J)$ 为零空间投影矩阵，$\mathbf{z}$ 为任意向量，可用于次要任务优化（如避障、关节极限回避等）。

---

## 2. 动力学总结 (Dynamics Summary)

动力学研究力与运动之间的关系。机器人动力学的核心是建立运动方程（Equations of Motion, EoM），将关节力矩与关节加速度联系起来。

### 2.1 运动方程的标准形式

$$M(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + J_c^T \mathbf{F}_c$$

各项含义：
- $M(\mathbf{q}) \in \mathbb{R}^{n \times n}$：广义质量矩阵（generalized mass/inertia matrix）
- $\mathbf{b}(\mathbf{q}, \dot{\mathbf{q}})$：科氏力和离心力项（Coriolis and centrifugal terms）
- $\mathbf{g}(\mathbf{q})$：重力项（gravity terms）
- $\boldsymbol{\tau}$：广义力/关节力矩（generalized forces/joint torques）
- $J_c^T \mathbf{F}_c$：接触力的广义力贡献（contact force contribution）

有时将科氏力和离心力写成矩阵形式：

$$\mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) = C(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}}$$

其中 $C(\mathbf{q}, \dot{\mathbf{q}})$ 为科氏力矩阵。

---

### 2.2 质量矩阵 M(q) 的性质

质量矩阵是动力学方程中最重要的量，具有以下性质：

1. **对称性**：$M(\mathbf{q}) = M(\mathbf{q})^T$
2. **正定性**：$\mathbf{x}^T M(\mathbf{q}) \mathbf{x} > 0, \; \forall \mathbf{x} \neq \mathbf{0}$
3. **构型依赖**：$M$ 随关节角 $\mathbf{q}$ 变化
4. **反对称性质**：$\dot{M} - 2C$ 是反对称矩阵（当 $C$ 用 Christoffel 符号定义时）

$$\mathbf{x}^T (\dot{M} - 2C) \mathbf{x} = 0, \quad \forall \mathbf{x}$$

这个性质在控制器稳定性证明（Lyapunov 分析）中非常重要。

---

### 2.3 能量

#### 动能 (Kinetic Energy)

$$T = \frac{1}{2} \dot{\mathbf{q}}^T M(\mathbf{q}) \dot{\mathbf{q}}$$

对于单个刚体：

$$T_i = \frac{1}{2} m_i \mathbf{v}_{S_i}^T \mathbf{v}_{S_i} + \frac{1}{2} \boldsymbol{\omega}_i^T \Theta_{S_i} \boldsymbol{\omega}_i$$

其中 $m_i$ 为质量，$\mathbf{v}_{S_i}$ 为质心线速度，$\Theta_{S_i}$ 为相对于质心的惯性张量，$\boldsymbol{\omega}_i$ 为角速度。

#### 势能 (Potential Energy)

**重力势能**：

$$U_{\text{grav}} = \sum_i m_i g h_i(\mathbf{q})$$

其中 $h_i$ 为第 $i$ 个连杆质心的高度。

**弹簧势能**（线性弹簧）：

$$U_{\text{spring}} = \frac{1}{2} k (\Delta l)^2$$

其中 $k$ 为弹簧刚度，$\Delta l$ 为弹簧伸长量。

**重力项的推导**：

$$\mathbf{g}(\mathbf{q}) = \frac{\partial U}{\partial \mathbf{q}}$$

---

### 2.4 三种推导方法

#### 2.4.1 Newton-Euler 方法

基于牛顿第二定律和欧拉方程，对每个刚体分别列方程：

**牛顿方程**（平动）：

$$m_i \mathbf{a}_{S_i} = \mathbf{F}_i$$

**欧拉方程**（转动）：

$$\Theta_{S_i} \dot{\boldsymbol{\omega}}_i + \boldsymbol{\omega}_i \times (\Theta_{S_i} \boldsymbol{\omega}_i) = \boldsymbol{\tau}_i$$

**递推算法**（Recursive Newton-Euler Algorithm, RNEA）：

1. **正向递推**（从基座到末端）：计算各连杆的速度和加速度
   - $\boldsymbol{\omega}_i = \boldsymbol{\omega}_{i-1} + \mathbf{e}_i \dot{q}_i$
   - $\dot{\boldsymbol{\omega}}_i = \dot{\boldsymbol{\omega}}_{i-1} + \mathbf{e}_i \ddot{q}_i + \boldsymbol{\omega}_{i-1} \times \mathbf{e}_i \dot{q}_i$
   - $\mathbf{a}_{S_i} = \mathbf{a}_{S_{i-1}} + \dot{\boldsymbol{\omega}}_i \times \mathbf{r}_{(i-1)S_i} + \boldsymbol{\omega}_i \times (\boldsymbol{\omega}_i \times \mathbf{r}_{(i-1)S_i})$

2. **反向递推**（从末端到基座）：计算各关节的力和力矩
   - $\mathbf{F}_i = m_i \mathbf{a}_{S_i} - \mathbf{F}_i^{\text{ext}}$
   - $\boldsymbol{\tau}_i = \Theta_{S_i} \dot{\boldsymbol{\omega}}_i + \boldsymbol{\omega}_i \times (\Theta_{S_i} \boldsymbol{\omega}_i) - \boldsymbol{\tau}_i^{\text{ext}}$
   - $\tau_i = \mathbf{e}_i^T \boldsymbol{\tau}_i^{\text{total}}$

**优点**：计算效率高 $O(n)$，适合实时控制  
**缺点**：不直接给出解析形式的运动方程

#### 2.4.2 Projected Newton-Euler 方法

将 Newton-Euler 方程投影到广义坐标空间，消除内部约束力：

对于每个刚体 $i$，定义其速度与广义速度的关系：

$$\mathbf{v}_{S_i} = J_{P_i}(\mathbf{q}) \dot{\mathbf{q}}, \quad \boldsymbol{\omega}_i = J_{R_i}(\mathbf{q}) \dot{\mathbf{q}}$$

将所有刚体的 Newton-Euler 方程堆叠并投影：

$$\sum_i \left[ J_{P_i}^T (m_i \mathbf{a}_{S_i}) + J_{R_i}^T (\Theta_{S_i} \dot{\boldsymbol{\omega}}_i + \boldsymbol{\omega}_i \times \Theta_{S_i} \boldsymbol{\omega}_i) \right] = \boldsymbol{\tau} + \boldsymbol{\tau}_{\text{ext}}$$

由此可以直接识别出质量矩阵：

$$M(\mathbf{q}) = \sum_i \left[ m_i J_{P_i}^T J_{P_i} + J_{R_i}^T \Theta_{S_i} J_{R_i} \right]$$

**优点**：
- 直接给出 $M(\mathbf{q})$ 的解析表达式
- 自动消除内部约束力
- 物理意义清晰

#### 2.4.3 Lagrange 方法

基于能量方法，利用 Lagrangian 函数：

$$\mathcal{L}(\mathbf{q}, \dot{\mathbf{q}}) = T(\mathbf{q}, \dot{\mathbf{q}}) - U(\mathbf{q})$$

**Euler-Lagrange 方程**：

$$\frac{d}{dt} \frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i, \quad i = 1, \ldots, n$$

展开后得到标准形式：

$$M(\mathbf{q})\ddot{\mathbf{q}} + C(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}$$

其中科氏力矩阵的元素可由 **Christoffel 符号** 计算：

$$C_{ij} = \sum_k c_{ijk} \dot{q}_k, \quad c_{ijk} = \frac{1}{2}\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right)$$

**优点**：
- 系统化的推导过程
- 只需要标量能量函数
- 自动满足 $\dot{M} - 2C$ 的反对称性质

**缺点**：
- 对于复杂系统，偏微分计算量大
- 计算复杂度 $O(n^3)$ 或更高

#### 2.4.4 三种方法的比较

| 特性 | Newton-Euler | Projected N-E | Lagrange |
|------|-------------|---------------|----------|
| 基本原理 | 力平衡 | 投影消约束 | 能量方法 |
| 计算复杂度 | $O(n)$ 递推 | $O(n^2)$ | $O(n^3)$+ |
| 解析表达式 | 不直接 | 直接 | 直接 |
| 约束力 | 需要处理 | 自动消除 | 自动消除 |
| 实时控制 | 最适合 | 适合 | 不太适合 |
| 物理直觉 | 强 | 中等 | 弱（能量角度） |

---

## 3. 控制方法总结 (Control Methods Summary)

### 3.1 运动学控制 (Kinematic Control)

#### 3.1.1 逆微分运动学 (Inverse Differential Kinematics)

假设机器人可以完美跟踪速度指令，直接在速度层面进行控制：

$$\dot{\mathbf{q}}_{\text{cmd}} = J^{\dagger}(\dot{\mathbf{x}}_d + K_p \mathbf{e})$$

其中 $\mathbf{e} = \mathbf{x}_d - \mathbf{x}$ 为任务空间误差，$K_p$ 为比例增益矩阵。

**位置误差的定义**：
- 平移误差：$\mathbf{e}_P = \mathbf{r}_d - \mathbf{r}$
- 旋转误差（多种定义方式）：
  - 欧拉角误差：$\mathbf{e}_R = \boldsymbol{\Phi}_d - \boldsymbol{\Phi}$
  - 基于旋转矩阵：$\mathbf{e}_R = \frac{1}{2}(C_d C^T - C C_d^T)^\vee$（vee 映射提取向量）

#### 3.1.2 多任务控制 (Multi-Task Control)

利用冗余自由度同时执行多个任务，按优先级排列：

**任务优先级框架 (Task Priority Framework)**：

$$\dot{\mathbf{q}} = J_1^{\dagger} \dot{\mathbf{x}}_1 + N_1 (J_2 N_1)^{\dagger} (\dot{\mathbf{x}}_2 - J_2 J_1^{\dagger} \dot{\mathbf{x}}_1) + \cdots$$

其中 $N_1 = I - J_1^{\dagger} J_1$ 为第一个任务的零空间投影矩阵。

**关键思想**：低优先级任务只在高优先级任务的零空间中执行，不影响高优先级任务。

---

### 3.2 关节阻抗控制 (Joint Impedance Control)

阻抗控制的核心思想是让机器人表现出期望的动态行为（如弹簧-阻尼系统），而不是精确跟踪轨迹。

#### 期望的阻抗行为

$$M_d \ddot{\mathbf{e}} + D_d \dot{\mathbf{e}} + K_d \mathbf{e} = \mathbf{F}_{\text{ext}}$$

其中 $M_d, D_d, K_d$ 分别为期望的惯性、阻尼和刚度矩阵，$\mathbf{e} = \mathbf{q}_d - \mathbf{q}$ 为关节误差。

#### 控制律

$$\boldsymbol{\tau} = K_d (\mathbf{q}_d - \mathbf{q}) + D_d (\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})$$

**特点**：
- 不需要精确的动力学模型（只需重力补偿）
- 柔顺行为（compliance）：遇到外力时会让步
- 适合与环境交互的任务（如装配、打磨）
- $K_d$ 和 $D_d$ 的选择决定了机器人的"软硬"程度

**稳定性分析**：选择 Lyapunov 函数

$$V = \frac{1}{2} \dot{\mathbf{q}}^T M(\mathbf{q}) \dot{\mathbf{q}} + \frac{1}{2} \mathbf{e}^T K_d \mathbf{e}$$

利用 $\dot{M} - 2C$ 的反对称性质可以证明系统稳定。

---

### 3.3 逆动力学控制 (Inverse Dynamics Control)

也称为计算力矩法（Computed Torque Method），通过精确补偿非线性动力学实现线性化解耦控制。

#### 控制律

$$\boldsymbol{\tau} = M(\mathbf{q})\mathbf{a}_{\text{cmd}} + \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})$$

其中指令加速度：

$$\mathbf{a}_{\text{cmd}} = \ddot{\mathbf{q}}_d + K_D(\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + K_P(\mathbf{q}_d - \mathbf{q})$$

#### 闭环动力学

代入运动方程后得到线性解耦的误差动力学：

$$\ddot{\mathbf{e}} + K_D \dot{\mathbf{e}} + K_P \mathbf{e} = \mathbf{0}$$

这是一个线性二阶系统，通过选择 $K_P$ 和 $K_D$ 可以任意配置极点。

**临界阻尼选择**：$K_D = 2\sqrt{K_P}$（对角矩阵情况下）

**优点**：
- 精确的轨迹跟踪
- 线性解耦的闭环行为
- 可以独立设计每个关节的响应

**缺点**：
- 需要精确的动力学模型 $M(\mathbf{q}), \mathbf{b}(\mathbf{q}, \dot{\mathbf{q}}), \mathbf{g}(\mathbf{q})$
- 对模型误差敏感
- 计算量大（需要实时计算完整动力学）

---

### 3.4 任务空间动力学控制 (Operational Space Dynamics Control)

将逆动力学控制扩展到任务空间（笛卡尔空间）。

#### 任务空间动力学方程

从关节空间动力学出发，利用 $\dot{\mathbf{x}} = J\dot{\mathbf{q}}$ 变换：

$$\Lambda(\mathbf{q})\ddot{\mathbf{x}} + \boldsymbol{\mu}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{p}(\mathbf{q}) = \mathbf{F}$$

其中：
- $\Lambda = (J M^{-1} J^T)^{-1}$：任务空间惯性矩阵
- $\boldsymbol{\mu} = \Lambda J M^{-1} \mathbf{b} - \Lambda \dot{J}\dot{\mathbf{q}}$：任务空间科氏力/离心力
- $\mathbf{p} = \Lambda J M^{-1} \mathbf{g}$：任务空间重力
- $\mathbf{F} = \Lambda J M^{-1} \boldsymbol{\tau}$：任务空间力（也可写为 $\boldsymbol{\tau} = J^T \mathbf{F}$）

#### 控制律

$$\boldsymbol{\tau} = J^T \left[ \Lambda \mathbf{a}_{\text{cmd}} + \boldsymbol{\mu} + \mathbf{p} \right]$$

其中：

$$\mathbf{a}_{\text{cmd}} = \ddot{\mathbf{x}}_d + K_D(\dot{\mathbf{x}}_d - \dot{\mathbf{x}}) + K_P(\mathbf{x}_d - \mathbf{x})$$

闭环行为：$\ddot{\mathbf{e}}_x + K_D \dot{\mathbf{e}}_x + K_P \mathbf{e}_x = \mathbf{0}$

---

### 3.5 操作空间控制：运动与力的混合控制 (Motion and Force Control)

在某些方向控制运动，在其他方向控制力。

#### 选择矩阵 (Selection Matrix)

定义选择矩阵 $S \in \mathbb{R}^{6 \times 6}$（对角矩阵，元素为 0 或 1）：
- $S$：力控制方向
- $(I - S)$：运动控制方向

#### 混合控制律

$$\mathbf{F} = S \mathbf{F}_d + (I - S)\left[\Lambda \mathbf{a}_{\text{cmd}} + \boldsymbol{\mu} + \mathbf{p}\right]$$

$$\boldsymbol{\tau} = J^T \mathbf{F}$$

**应用场景**：
- 沿表面滑动（法向力控制 + 切向运动控制）
- 打磨、抛光（控制接触力的同时控制运动轨迹）
- 装配任务（插入方向力控制 + 横向位置控制）

---

## 4. 浮动基座系统 (Floating Base Systems)

与固定基座机器人不同，浮动基座系统（如足式机器人、飞行器）的基座本身可以自由运动，没有固定在世界坐标系中。

### 4.1 广义坐标和速度

#### 广义坐标 (Generalized Coordinates)

$$\mathbf{q} = \begin{pmatrix} \mathbf{q}_b \ \mathbf{q}_r \end{pmatrix} \in \mathbb{R}^{n_q}$$

其中：
- $\mathbf{q}_b = \begin{pmatrix} \mathbf{r}_b \ \boldsymbol{\Phi}_b \end{pmatrix}$：基座的位置和姿态（6 个自由度）
- $\mathbf{q}_r \in \mathbb{R}^{n_r}$：关节角（$n_r$ 个关节自由度）
- $n_q = 6 + n_r$：总广义坐标数

> **注意**：由于旋转参数化的问题，广义坐标的维度可能与广义速度的维度不同。例如使用四元数时，$\mathbf{q}_b$ 有 7 个参数（3 平移 + 4 四元数），但广义速度只有 6 维。

#### 广义速度 (Generalized Velocities)

$$\mathbf{u} = \begin{pmatrix} \mathbf{u}_b \ \dot{\mathbf{q}}_r \end{pmatrix} \in \mathbb{R}^{n_u}$$

其中 $\mathbf{u}_b = \begin{pmatrix} \mathbf{v}_b \ \boldsymbol{\omega}_b \end{pmatrix}$ 为基座的线速度和角速度，$n_u = 6 + n_r$。

广义坐标与广义速度的关系：

$$\dot{\mathbf{q}} = H(\mathbf{q}) \mathbf{u}$$

其中 $H$ 为映射矩阵（当使用欧拉角时包含 $E$ 矩阵，使用四元数时包含四元数微分映射）。

### 4.2 浮动基座的运动方程

$$M(\mathbf{q})\dot{\mathbf{u}} + \mathbf{b}(\mathbf{q}, \mathbf{u}) + \mathbf{g}(\mathbf{q}) = S^T \boldsymbol{\tau} + J_c^T \mathbf{F}_c$$

其中选择矩阵 $S$ 反映了基座不被直接驱动的事实：

$$S = \begin{pmatrix} \mathbf{0}_{n_r \times 6} & I_{n_r} \end{pmatrix}$$

即 $S^T \boldsymbol{\tau}$ 只将关节力矩映射到关节自由度，基座的 6 个自由度没有直接驱动力。

将运动方程按基座和关节分块：

$$\begin{pmatrix} M_{bb} & M_{br} \ M_{rb} & M_{rr} \end{pmatrix} \begin{pmatrix} \dot{\mathbf{u}}_b \ \ddot{\mathbf{q}}_r \end{pmatrix} + \begin{pmatrix} \mathbf{b}_b \ \mathbf{b}_r \end{pmatrix} + \begin{pmatrix} \mathbf{g}_b \ \mathbf{g}_r \end{pmatrix} = \begin{pmatrix} \mathbf{0} \ \boldsymbol{\tau} \end{pmatrix} + \begin{pmatrix} J_{c,b}^T \ J_{c,r}^T \end{pmatrix} \mathbf{F}_c$$

**关键观察**：基座方程（第一行）表示系统的整体动量守恒（无外力时），这是浮动基座系统的本质特征。

### 4.3 接触约束 (Contact Constraints)

#### 接触雅可比矩阵 (Contact Jacobian)

接触点的速度为零（假设无滑动）：

$$\mathbf{v}_c = J_c(\mathbf{q}) \mathbf{u} = \mathbf{0}$$

对时间微分得到加速度约束：

$$J_c \dot{\mathbf{u}} + \dot{J}_c \mathbf{u} = \mathbf{0}$$

#### 接触力的求解

将加速度约束代入运动方程，可以求解接触力 $\mathbf{F}_c$：

$$\mathbf{F}_c = (J_c M^{-1} J_c^T)^{-1} (J_c M^{-1}(\mathbf{b} + \mathbf{g} - S^T\boldsymbol{\tau}) + \dot{J}_c \mathbf{u})$$

#### 接触约束的物理意义

- **不穿透约束**：接触点法向速度为零
- **无滑动约束**：接触点切向速度为零（需要足够摩擦力）
- **摩擦锥约束**：$\|\mathbf{F}_t\| \leq \mu \mathbf{F}_n$（库仑摩擦模型）

其中 $\mathbf{F}_t$ 为切向力，$\mathbf{F}_n$ 为法向力，$\mu$ 为摩擦系数。

---

## 5. 足式机器人 (Legged Robots)

足式机器人是浮动基座系统的典型应用。与轮式机器人相比，足式机器人能够在非结构化地形上运动，但控制难度更大。

### 5.1 静态稳定 vs 动态稳定

#### 静态稳定 (Static Stability)

- **定义**：在任意时刻，即使机器人停止运动，也不会倒下
- **条件**：重心（Center of Mass, CoM）的垂直投影落在支撑多边形（Support Polygon）内部
- **支撑多边形**：所有接触点在水平面上投影的凸包
- **特点**：运动速度慢，至少需要三条腿同时着地（对于四足机器人）
- **例子**：缓慢行走的四足机器人

#### 动态稳定 (Dynamic Stability)

- **定义**：利用惯性效应维持平衡，静止时可能不稳定
- **关键概念**：零力矩点（Zero Moment Point, ZMP）
  
$$\mathbf{r}_{\text{ZMP}} = \frac{\sum_i m_i(\ddot{z}_i + g)r_i^{xy} - \sum_i m_i \ddot{r}_i^{xy} z_i}{\sum_i m_i(\ddot{z}_i + g)}$$

- **ZMP 稳定条件**：ZMP 必须落在支撑多边形内部
- **特点**：可以实现快速运动（跑、跳）
- **例子**：奔跑的四足机器人、双足行走

#### 两者的比较

| 特性 | 静态稳定 | 动态稳定 |
|------|---------|---------|
| 速度 | 慢 | 快 |
| 复杂度 | 低 | 高 |
| 鲁棒性 | 高（对扰动） | 需要主动控制 |
| 最少支撑腿 | 3（四足） | 可以少于 3 |
| 分析工具 | CoM 投影 | ZMP / 角动量 |

### 5.2 运动学约束 (Kinematic Constraints)

足式机器人的运动受到多种约束：

#### 完整约束 (Holonomic Constraints)

可以写成位置级别的等式约束：

$$\mathbf{h}(\mathbf{q}) = \mathbf{0}$$

例如：足端接触地面（位置约束）。

#### 非完整约束 (Non-holonomic Constraints)

只能写成速度级别的约束，不可积分为位置约束：

$$A(\mathbf{q})\dot{\mathbf{q}} = \mathbf{0}$$

例如：纯滚动约束（轮子不打滑）。

#### 足式机器人的典型约束

1. **接触约束**：着地足端速度为零
   $$J_{c,i}(\mathbf{q})\mathbf{u} = \mathbf{0}, \quad \text{for each contact foot } i$$

2. **摩擦约束**：接触力在摩擦锥内
   $$\sqrt{F_{c,x}^2 + F_{c,y}^2} \leq \mu F_{c,z}, \quad F_{c,z} \geq 0$$

3. **关节限位**：$\mathbf{q}_{\min} \leq \mathbf{q}_r \leq \mathbf{q}_{\max}$

4. **力矩限制**：$|\tau_i| \leq \tau_{i,\max}$

### 5.3 全身控制 (Whole-Body Control)

全身控制同时考虑基座运动和所有关节运动，是足式机器人控制的核心方法。

#### 基本框架

全身控制通常基于优化的方法，将多个任务和约束统一处理：

$$\min_{\dot{\mathbf{u}}, \boldsymbol{\tau}, \mathbf{F}_c} \sum_k w_k \| J_k \dot{\mathbf{u}} + \dot{J}_k \mathbf{u} - \ddot{\mathbf{x}}_{k,\text{des}} \|^2$$

**约束条件**：
1. 运动方程：$M\dot{\mathbf{u}} + \mathbf{b} + \mathbf{g} = S^T\boldsymbol{\tau} + J_c^T \mathbf{F}_c$
2. 接触约束：$J_c \dot{\mathbf{u}} + \dot{J}_c \mathbf{u} = \mathbf{0}$
3. 摩擦锥约束：$\mathbf{F}_c \in \mathcal{FC}$
4. 力矩限制：$\boldsymbol{\tau}_{\min} \leq \boldsymbol{\tau} \leq \boldsymbol{\tau}_{\max}$

#### 任务层次 (Task Hierarchy)

典型的任务优先级（从高到低）：

1. **接触约束维持**：保持已有的接触不滑动
2. **基座姿态控制**：保持躯干水平
3. **摆动腿轨迹跟踪**：控制摆动腿到达下一个落脚点
4. **基座位置/速度控制**：控制机器人的前进方向和速度
5. **姿态优化**：利用冗余自由度优化关节构型

#### 层次化优化 (Hierarchical Optimization)

使用零空间投影实现严格的任务优先级：

$$\dot{\mathbf{u}}^* = \sum_{k=1}^{N} \left(\prod_{j=1}^{k-1} N_j\right) J_k^{\dagger} (\ddot{\mathbf{x}}_{k,\text{des}} - \dot{J}_k \mathbf{u} - J_k \dot{\mathbf{u}}_{k-1}^*)$$

或者使用加权 QP（Quadratic Programming）求解，更容易处理不等式约束。

---

## 6. 旋翼飞行器 (Rotorcraft)

旋翼飞行器（如四旋翼无人机 quadrotor）是浮动基座系统在空中的典型应用。其特点是欠驱动（underactuated）：只有 4 个输入控制 6 个自由度。

### 6.1 推力模型 (Thrust Model)

#### 单个旋翼的推力和力矩

每个旋翼产生的推力与转速的平方成正比：

$$f_i = k_f \omega_i^2$$

每个旋翼还会产生反扭矩（reaction torque）：

$$\tau_i = k_\tau \omega_i^2$$

其中 $k_f$ 为推力系数，$k_\tau$ 为力矩系数，$\omega_i$ 为第 $i$ 个旋翼的转速。

#### 四旋翼的总推力和力矩

对于标准 "X" 构型的四旋翼：

$$\begin{pmatrix} F_z \ \tau_x \ \tau_y \ \tau_z \end{pmatrix} = \begin{pmatrix} k_f & k_f & k_f & k_f \ 0 & -dk_f & 0 & dk_f \ dk_f & 0 & -dk_f & 0 \ -k_\tau & k_\tau & -k_\tau & k_\tau \end{pmatrix} \begin{pmatrix} \omega_1^2 \ \omega_2^2 \ \omega_3^2 \ \omega_4^2 \end{pmatrix}$$

其中 $d$ 为旋翼到机体中心的距离。

**分配矩阵 (Allocation Matrix)** $A$：

$$\mathbf{u}_{\text{wrench}} = A \begin{pmatrix} \omega_1^2 \ \omega_2^2 \ \omega_3^2 \ \omega_4^2 \end{pmatrix}$$

该矩阵可逆（对于四旋翼），因此可以从期望的力和力矩反算各旋翼转速。

> **注意**：相邻旋翼旋转方向相反，以平衡反扭矩。对角线上的旋翼同向旋转。

### 6.2 运动方程 (Equations of Motion)

#### 平动方程（世界坐标系）

$$m\ddot{\mathbf{r}} = \begin{pmatrix} 0 \ 0 \ -mg \end{pmatrix} + C_{IB} \begin{pmatrix} 0 \ 0 \ F_z \end{pmatrix}$$

其中 $C_{IB}$ 为从机体坐标系到世界坐标系的旋转矩阵，$F_z = \sum_i f_i$ 为总推力（沿机体 $z$ 轴）。

展开：

$$m\ddot{x} = (\cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi) F_z$$
$$m\ddot{y} = (\sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi) F_z$$
$$m\ddot{z} = -mg + \cos\theta\cos\phi \cdot F_z$$

#### 转动方程（机体坐标系）

$$\Theta \dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (\Theta \boldsymbol{\omega}_B) = \boldsymbol{\tau}_B$$

其中 $\Theta$ 为机体惯性张量，$\boldsymbol{\omega}_B$ 为机体坐标系下的角速度，$\boldsymbol{\tau}_B = (\tau_x, \tau_y, \tau_z)^T$。

对于对称四旋翼，$\Theta = \text{diag}(I_{xx}, I_{yy}, I_{zz})$，方程简化为：

$$I_{xx}\dot{\omega}_x = (I_{yy} - I_{zz})\omega_y\omega_z + \tau_x$$
$$I_{yy}\dot{\omega}_y = (I_{zz} - I_{xx})\omega_z\omega_x + \tau_y$$
$$I_{zz}\dot{\omega}_z = (I_{xx} - I_{yy})\omega_x\omega_y + \tau_z$$

#### 欠驱动特性 (Underactuation)

- **输入**：4 个（$\omega_1, \omega_2, \omega_3, \omega_4$ 或等价地 $F_z, \tau_x, \tau_y, \tau_z$）
- **自由度**：6 个（$x, y, z, \phi, \theta, \psi$）
- **可直接控制**：$z$（通过总推力）和 $\phi, \theta, \psi$（通过力矩）
- **间接控制**：$x, y$（通过倾斜机体改变推力方向）

这意味着四旋翼不能在保持水平的同时独立控制水平位置——必须先倾斜才能水平移动。

### 6.3 控制架构 (Control Architecture)

四旋翼的控制通常采用级联（cascade）结构：

#### 外环：位置控制

$$\mathbf{F}_{\text{des}} = m(\ddot{\mathbf{r}}_d + K_D^{\text{pos}}(\dot{\mathbf{r}}_d - \dot{\mathbf{r}}) + K_P^{\text{pos}}(\mathbf{r}_d - \mathbf{r})) + m\mathbf{g}$$

从期望力 $\mathbf{F}_{\text{des}}$ 提取：
- 总推力：$F_z = \|\mathbf{F}_{\text{des}}\|$（或其在机体 $z$ 轴的投影）
- 期望姿态：从 $\mathbf{F}_{\text{des}}$ 的方向和期望偏航角 $\psi_d$ 计算期望的旋转矩阵 $C_d$

#### 内环：姿态控制

$$\boldsymbol{\tau}_{\text{des}} = \Theta(\ddot{\boldsymbol{\Phi}}_d + K_D^{\text{att}}(\dot{\boldsymbol{\Phi}}_d - \dot{\boldsymbol{\Phi}}) + K_P^{\text{att}}(\boldsymbol{\Phi}_d - \boldsymbol{\Phi})) + \boldsymbol{\omega} \times (\Theta \boldsymbol{\omega})$$

或使用更鲁棒的基于旋转矩阵/四元数的误差定义。

#### 电机分配 (Motor Allocation)

$$\begin{pmatrix} \omega_1^2 \ \omega_2^2 \ \omega_3^2 \ \omega_4^2 \end{pmatrix} = A^{-1} \begin{pmatrix} F_z \ \tau_x \ \tau_y \ \tau_z \end{pmatrix}$$

**级联控制的关键假设**：内环（姿态）的响应速度远快于外环（位置），实现时间尺度分离（time-scale separation）。

---

## 7. 固定翼飞行器 (Fixed-Wing Aircraft)

固定翼飞行器通过机翼产生的空气动力学升力来维持飞行，与旋翼飞行器有本质区别：必须保持一定的前飞速度才能产生足够的升力。

### 7.1 空气动力学 (Aerodynamics)

#### 基本概念

- **迎角 (Angle of Attack, $\alpha$)**：机翼弦线与来流方向之间的夹角
- **侧滑角 (Sideslip Angle, $\beta$)**：速度矢量与机体对称面之间的夹角
- **动压 (Dynamic Pressure)**：$\bar{q} = \frac{1}{2}\rho V_a^2$，其中 $\rho$ 为空气密度，$V_a$ 为空速

#### 空气动力和力矩

空气动力通常在风轴系（wind frame）或机体坐标系中表示：

**升力 (Lift)**：

$$L = \bar{q} S C_L(\alpha)$$

**阻力 (Drag)**：

$$D = \bar{q} S C_D(\alpha)$$

**侧力 (Side Force)**：

$$Y = \bar{q} S C_Y(\beta)$$

其中 $S$ 为机翼参考面积，$C_L, C_D, C_Y$ 为无量纲气动系数。

#### 气动系数的线性化模型

在小扰动假设下，气动系数可以线性化：

$$C_L \approx C_{L_0} + C_{L_\alpha} \alpha + C_{L_q} \frac{qc}{2V_a} + C_{L_{\delta_e}} \delta_e$$

$$C_D \approx C_{D_0} + \frac{C_L^2}{\pi e AR}$$

$$C_m \approx C_{m_0} + C_{m_\alpha} \alpha + C_{m_q} \frac{qc}{2V_a} + C_{m_{\delta_e}} \delta_e$$

其中：
- $C_{L_\alpha}$：升力线斜率（lift curve slope）
- $C_{L_{\delta_e}}$：升降舵效率
- $C_{D_0}$：零升阻力系数（parasite drag）
- $e$：Oswald 效率因子
- $AR$：展弦比（aspect ratio）= $b^2/S$
- $c$：平均气动弦长（mean aerodynamic chord）
- $\delta_e$：升降舵偏转角（elevator deflection）

#### 滚转和偏航力矩

$$C_l \approx C_{l_\beta}\beta + C_{l_p}\frac{pb}{2V_a} + C_{l_r}\frac{rb}{2V_a} + C_{l_{\delta_a}}\delta_a + C_{l_{\delta_r}}\delta_r$$

$$C_n \approx C_{n_\beta}\beta + C_{n_p}\frac{pb}{2V_a} + C_{n_r}\frac{rb}{2V_a} + C_{n_{\delta_a}}\delta_a + C_{n_{\delta_r}}\delta_r$$

其中 $b$ 为翼展，$p, q, r$ 为机体坐标系下的角速度分量，$\delta_a$ 为副翼偏转角，$\delta_r$ 为方向舵偏转角。

### 7.2 六自由度模型 (6-DOF Model)

#### 坐标系定义

- **惯性坐标系 (Inertial Frame, $I$)**：固定在地面，NED（北-东-地）约定
- **机体坐标系 (Body Frame, $B$)**：固定在飞行器上，$x_B$ 指向前方，$y_B$ 指向右翼，$z_B$ 指向下方

#### 状态向量

$$\mathbf{x} = \begin{pmatrix} \mathbf{r}_I \ \mathbf{v}_B \ \boldsymbol{\Phi} \ \boldsymbol{\omega}_B \end{pmatrix} = \begin{pmatrix} p_N, p_E, p_D \ u, v, w \ \phi, \theta, \psi \ p, q, r \end{pmatrix}$$

其中 $(u, v, w)$ 为机体坐标系下的速度分量，$(p, q, r)$ 为机体坐标系下的角速度分量。

#### 平动方程（机体坐标系）

$$m\dot{\mathbf{v}}_B = \mathbf{F}_B - \boldsymbol{\omega}_B \times (m\mathbf{v}_B)$$

展开：

$$m\dot{u} = F_x + m(rv - qw)$$
$$m\dot{v} = F_y + m(pw - ru)$$
$$m\dot{w} = F_z + m(qu - pv)$$

其中 $\mathbf{F}_B = \mathbf{F}_{\text{aero}} + \mathbf{F}_{\text{grav}} + \mathbf{F}_{\text{thrust}}$。

#### 转动方程（机体坐标系）

$$\Theta_B \dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (\Theta_B \boldsymbol{\omega}_B) = \mathbf{M}_B$$

展开（假设 $\Theta_B$ 对称，$I_{xy} = I_{yz} = 0$，但 $I_{xz} \neq 0$）：

$$I_{xx}\dot{p} - I_{xz}\dot{r} = M_x + (I_{yy} - I_{zz})qr + I_{xz}pq$$
$$I_{yy}\dot{q} = M_y + (I_{zz} - I_{xx})pr + I_{xz}(r^2 - p^2)$$
$$I_{zz}\dot{r} - I_{xz}\dot{p} = M_z + (I_{xx} - I_{yy})pq - I_{xz}qr$$

#### 运动学方程

位置更新（惯性坐标系）：

$$\dot{\mathbf{r}}_I = C_{IB} \mathbf{v}_B$$

姿态更新（欧拉角）：

$$\begin{pmatrix} \dot{\phi} \ \dot{\theta} \ \dot{\psi} \end{pmatrix} = \begin{pmatrix} 1 & \sin\phi\tan\theta & \cos\phi\tan\theta \ 0 & \cos\phi & -\sin\phi \ 0 & \sin\phi\sec\theta & \cos\phi\sec\theta \end{pmatrix} \begin{pmatrix} p \ q \ r \end{pmatrix}$$

> **注意**：当 $\theta = \pm 90°$ 时出现万向锁，实际飞行控制中通常使用四元数避免此问题。

### 7.3 配平 (Trim)

配平是指飞行器在稳态飞行条件下的平衡状态，即所有加速度为零。

#### 配平条件

$$\dot{\mathbf{v}}_B = \mathbf{0}, \quad \dot{\boldsymbol{\omega}}_B = \mathbf{0}$$

对于直线水平飞行（wings-level, constant altitude）：

$$\dot{u} = \dot{v} = \dot{w} = 0, \quad \dot{p} = \dot{q} = \dot{r} = 0$$
$$\phi = 0, \quad \dot{\theta} = 0, \quad \dot{\psi} = 0$$

#### 配平求解

需要求解以下变量使得力和力矩平衡：
- 迎角 $\alpha^*$（决定升力等于重力）
- 油门 $\delta_t^*$（决定推力等于阻力）
- 升降舵 $\delta_e^*$（决定俯仰力矩为零）

**升力平衡**：$L = mg\cos\gamma$（$\gamma$ 为航迹角）

$$\frac{1}{2}\rho V_a^2 S C_L(\alpha^*) = mg\cos\gamma$$

**阻力平衡**：$T = D + mg\sin\gamma$

$$T^* = \frac{1}{2}\rho V_a^2 S C_D(\alpha^*) + mg\sin\gamma$$

**俯仰力矩平衡**：$M_y = 0$

$$C_{m_0} + C_{m_\alpha}\alpha^* + C_{m_{\delta_e}}\delta_e^* = 0$$

$$\delta_e^* = -\frac{C_{m_0} + C_{m_\alpha}\alpha^*}{C_{m_{\delta_e}}}$$

#### 协调转弯配平 (Coordinated Turn Trim)

在恒定坡度角 $\phi$ 的协调转弯中：

- 升力的垂直分量平衡重力：$L\cos\phi = mg$
- 升力的水平分量提供向心力：$L\sin\phi = \frac{mV_a^2}{R}$
- 转弯半径：$R = \frac{V_a^2}{g\tan\phi}$
- 转弯角速率：$\dot{\psi} = \frac{g\tan\phi}{V_a}$
- 需要方向舵 $\delta_r$ 来消除侧滑（$\beta = 0$）

### 7.4 固定翼飞行器的控制

#### 控制输入

固定翼飞行器通常有 4 个控制输入：

| 控制面 | 符号 | 主要控制效果 |
|--------|------|-------------|
| 副翼 (Aileron) | $\delta_a$ | 滚转力矩 $M_x$ |
| 升降舵 (Elevator) | $\delta_e$ | 俯仰力矩 $M_y$ |
| 方向舵 (Rudder) | $\delta_r$ | 偏航力矩 $M_z$ |
| 油门 (Throttle) | $\delta_t$ | 推力 $T$ |

#### 纵向与横航向解耦

在小扰动假设下，固定翼的运动可以解耦为：

**纵向运动 (Longitudinal)**：$(u, w, q, \theta)$ — 由 $\delta_e, \delta_t$ 控制
- 短周期模态（Short Period）：快速的迎角和俯仰率响应
- 长周期模态（Phugoid）：缓慢的速度和高度振荡

**横航向运动 (Lateral-Directional)**：$(v, p, r, \phi, \psi)$ — 由 $\delta_a, \delta_r$ 控制
- 滚转模态（Roll）：快速的滚转响应
- 荷兰滚模态（Dutch Roll）：耦合的偏航-滚转振荡
- 螺旋模态（Spiral）：缓慢的发散或收敛

#### 控制架构

典型的固定翼控制采用内外环结构：

1. **内环（稳定增强 / SAS）**：角速率反馈，增加阻尼
   - 俯仰阻尼器：$\delta_e = K_q (q_d - q)$
   - 偏航阻尼器：$\delta_r = K_r (r_d - r)$

2. **中环（姿态控制）**：姿态角反馈
   - 俯仰姿态保持：$q_d = K_\theta (\theta_d - \theta)$
   - 滚转姿态保持：$\delta_a = K_\phi (\phi_d - \phi) + K_p(p_d - p)$

3. **外环（制导）**：航迹和速度控制
   - 高度控制：$\theta_d = K_h (h_d - h) + K_{\dot{h}}(\dot{h}_d - \dot{h})$
   - 航向控制：$\phi_d = K_\chi (\chi_d - \chi)$（$\chi$ 为航迹方位角）
   - 速度控制：$\delta_t = K_V (V_d - V)$

---

## 8. 考试要点 (Exam Key Points)

### 8.1 关键公式清单 (Essential Formulas)

以下是考试中最常用的公式，按主题分类整理。建议熟记并理解其物理意义。

#### 运动学核心公式

| 编号 | 公式 | 说明 |
|------|------|------|
| K1 | $T_{AC} = T_{AB} \cdot T_{BC}$ | 齐次变换的复合 |
| K2 | $T^{-1} = \begin{pmatrix} C^T & -C^T\mathbf{r} \ \mathbf{0}^T & 1 \end{pmatrix}$ | 齐次变换的逆 |
| K3 | $C = I + \sin\varphi[\mathbf{n}]_\times + (1-\cos\varphi)[\mathbf{n}]_\times^2$ | Rodrigues 公式 |
| K4 | $\begin{pmatrix}\mathbf{v}\\boldsymbol{\omega}\end{pmatrix} = J_G(\mathbf{q})\dot{\mathbf{q}}$ | 几何雅可比定义 |
| K5 | $\boldsymbol{\tau} = J^T \mathbf{F}$ | 静力学对偶 |
| K6 | $\dot{\mathbf{q}} = J^{\dagger}\dot{\mathbf{x}} + (I - J^{\dagger}J)\mathbf{z}$ | 冗余逆运动学 |
| K7 | $J^{\dagger} = J^T(JJ^T)^{-1}$ | 右伪逆（冗余情况） |
| K8 | $\Delta\mathbf{q} = J^T(JJ^T + \lambda^2 I)^{-1}\Delta\mathbf{x}$ | 阻尼最小二乘 |

#### 动力学核心公式

| 编号 | 公式 | 说明 |
|------|------|------|
| D1 | $M(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{b}(\mathbf{q},\dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + J_c^T\mathbf{F}_c$ | 运动方程标准形式 |
| D2 | $T = \frac{1}{2}\dot{\mathbf{q}}^T M(\mathbf{q})\dot{\mathbf{q}}$ | 动能 |
| D3 | $\mathbf{g}(\mathbf{q}) = \frac{\partial U}{\partial \mathbf{q}}$ | 重力项 |
| D4 | $M = \sum_i (m_i J_{P_i}^T J_{P_i} + J_{R_i}^T \Theta_{S_i} J_{R_i})$ | 质量矩阵（Projected N-E） |
| D5 | $\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i$ | Euler-Lagrange 方程 |
| D6 | $c_{ijk} = \frac{1}{2}(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i})$ | Christoffel 符号 |
| D7 | $\mathbf{x}^T(\dot{M} - 2C)\mathbf{x} = 0$ | 反对称性质 |

#### 控制核心公式

| 编号 | 公式 | 说明 |
|------|------|------|
| C1 | $\boldsymbol{\tau} = K_d(\mathbf{q}_d - \mathbf{q}) + D_d(\dot{\mathbf{q}}_d - \dot{\mathbf{q}}) + \mathbf{g}(\mathbf{q})$ | 关节阻抗控制 |
| C2 | $\boldsymbol{\tau} = M\mathbf{a}_{\text{cmd}} + \mathbf{b} + \mathbf{g}$ | 逆动力学控制 |
| C3 | $\mathbf{a}_{\text{cmd}} = \ddot{\mathbf{q}}_d + K_D\dot{\mathbf{e}} + K_P\mathbf{e}$ | PD 指令加速度 |
| C4 | $\Lambda = (JM^{-1}J^T)^{-1}$ | 任务空间惯性矩阵 |
| C5 | $\boldsymbol{\tau} = J^T[\Lambda\mathbf{a}_{\text{cmd}} + \boldsymbol{\mu} + \mathbf{p}]$ | 任务空间逆动力学 |

#### 浮动基座与足式机器人

| 编号 | 公式 | 说明 |
|------|------|------|
| F1 | $M\dot{\mathbf{u}} + \mathbf{b} + \mathbf{g} = S^T\boldsymbol{\tau} + J_c^T\mathbf{F}_c$ | 浮动基座运动方程 |
| F2 | $J_c\mathbf{u} = \mathbf{0}$ | 接触速度约束 |
| F3 | $\|\mathbf{F}_t\| \leq \mu F_n$ | 摩擦锥约束 |

#### 飞行器

| 编号 | 公式 | 说明 |
|------|------|------|
| A1 | $f_i = k_f\omega_i^2$ | 旋翼推力模型 |
| A2 | $m\ddot{\mathbf{r}} = m\mathbf{g}_I + C_{IB}\mathbf{F}_B$ | 四旋翼平动方程 |
| A3 | $\Theta\dot{\boldsymbol{\omega}} + \boldsymbol{\omega}\times(\Theta\boldsymbol{\omega}) = \boldsymbol{\tau}$ | 刚体转动方程 |
| A4 | $L = \frac{1}{2}\rho V_a^2 S C_L$ | 升力公式 |
| A5 | $D = \frac{1}{2}\rho V_a^2 S C_D$ | 阻力公式 |

---

### 8.2 常见题型 (Common Exam Question Types)

以下是历年考试中常见的题型分类和解题思路。

#### 题型一：正运动学推导

**典型问题**：给定机器人的几何参数，求末端执行器的位姿 $T_{0E}(\mathbf{q})$。

**解题步骤**：
1. 确定每个关节的类型（旋转/移动）和旋转轴方向
2. 写出每个关节的齐次变换矩阵 $T_{(i-1)i}(q_i)$
3. 连乘得到 $T_{0E} = T_{01} \cdot T_{12} \cdots T_{(n-1)n}$
4. 提取位置向量 $\mathbf{r}_{0E}$ 和旋转矩阵 $C_{0E}$

**常见陷阱**：
- 注意坐标系的定义方向
- 旋转轴不一定是标准坐标轴
- 平移向量要在正确的坐标系中表示

#### 题型二：雅可比矩阵计算

**典型问题**：求给定构型下的几何雅可比矩阵，分析奇异性。

**解题步骤**：
1. 对每个关节，确定旋转轴方向 $\mathbf{e}_i$（在世界坐标系中）
2. 确定关节位置 $\mathbf{r}_{0i}$ 和末端位置 $\mathbf{r}_{0E}$
3. 对旋转关节：$J_P^{(i)} = \mathbf{e}_i \times (\mathbf{r}_{0E} - \mathbf{r}_{0i})$，$J_R^{(i)} = \mathbf{e}_i$
4. 对移动关节：$J_P^{(i)} = \mathbf{e}_i$，$J_R^{(i)} = \mathbf{0}$
5. 组装雅可比矩阵 $J_G = \begin{pmatrix} J_P \ J_R \end{pmatrix}$
6. 分析 $\det(J)$ 或 $\det(JJ^T)$ 何时为零 → 奇异构型

**常见陷阱**：
- $\mathbf{e}_i$ 和 $\mathbf{r}_{0i}$ 必须在同一坐标系（通常是世界坐标系）中表示
- 叉积的顺序不能搞反
- 在特定构型下代入数值时要仔细

#### 题型三：动力学方程推导

**典型问题**：用 Lagrange 方法或 Projected Newton-Euler 方法推导简单系统的运动方程。

**Lagrange 方法步骤**：
1. 选择广义坐标 $\mathbf{q}$
2. 用 $\mathbf{q}$ 和 $\dot{\mathbf{q}}$ 表示每个刚体的速度（线速度和角速度）
3. 计算总动能 $T = \sum_i T_i$
4. 计算总势能 $U = \sum_i U_i$
5. 构造 Lagrangian：$\mathcal{L} = T - U$
6. 对每个 $q_i$ 应用 Euler-Lagrange 方程
7. 整理成标准形式 $M\ddot{\mathbf{q}} + C\dot{\mathbf{q}} + \mathbf{g} = \boldsymbol{\tau}$

**Projected Newton-Euler 方法步骤**：
1. 计算每个刚体的雅可比矩阵 $J_{P_i}, J_{R_i}$
2. 计算质量矩阵 $M = \sum_i (m_i J_{P_i}^T J_{P_i} + J_{R_i}^T \Theta_i J_{R_i})$
3. 计算非线性项（通过对 $M\dot{\mathbf{q}}$ 求时间导数）
4. 计算重力项 $\mathbf{g} = \frac{\partial U}{\partial \mathbf{q}}$

**常见陷阱**：
- 动能中不要遗漏旋转动能项
- 惯性张量要在正确的坐标系中表示
- Christoffel 符号的计算容易出错，建议仔细检查对称性

#### 题型四：控制器设计

**典型问题**：为给定系统设计控制器并分析稳定性。

**逆动力学控制**：
1. 写出系统的运动方程
2. 设计控制律 $\boldsymbol{\tau} = M\mathbf{a}_{\text{cmd}} + \mathbf{b} + \mathbf{g}$
3. 选择 $\mathbf{a}_{\text{cmd}} = \ddot{\mathbf{q}}_d + K_D\dot{\mathbf{e}} + K_P\mathbf{e}$
4. 验证闭环方程为 $\ddot{\mathbf{e}} + K_D\dot{\mathbf{e}} + K_P\mathbf{e} = 0$
5. 选择增益使系统临界阻尼

**阻抗控制**：
1. 设计控制律（通常包含重力补偿）
2. 选择 Lyapunov 函数 $V = \frac{1}{2}\dot{\mathbf{q}}^T M \dot{\mathbf{q}} + \frac{1}{2}\mathbf{e}^T K_d \mathbf{e}$
3. 计算 $\dot{V}$，利用 $\dot{M} - 2C$ 的反对称性质
4. 证明 $\dot{V} \leq 0$（稳定性）或 $\dot{V} < 0$（渐近稳定性）

#### 题型五：浮动基座系统分析

**典型问题**：写出浮动基座机器人的运动方程，分析接触约束。

**解题步骤**：
1. 定义广义坐标（基座 + 关节）
2. 写出运动方程（注意选择矩阵 $S$）
3. 写出接触约束 $J_c \mathbf{u} = \mathbf{0}$
4. 求解接触力或消除接触力得到约束运动方程
5. 检查摩擦锥约束是否满足

#### 题型六：飞行器动力学与控制

**典型问题**：
- 推导四旋翼的运动方程
- 计算固定翼飞行器的配平状态
- 设计级联控制器

**四旋翼常见考点**：
- 从旋翼转速到力/力矩的映射（分配矩阵）
- 欠驱动特性的解释
- 位置-姿态级联控制的设计

**固定翼常见考点**：
- 配平条件的建立和求解
- 纵向/横航向运动的解耦
- 各模态的物理意义（短周期、长周期、荷兰滚等）

---

### 8.3 MATLAB 编程要点 (MATLAB Programming Key Points)

考试中可能涉及 MATLAB 编程题，以下是常用的代码模式和函数。

#### 8.3.1 符号计算 (Symbolic Computation)

```matlab
% 定义符号变量
syms q1 q2 q3 dq1 dq2 dq3 real
syms m1 m2 l1 l2 g real

% 定义广义坐标向量
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
```

#### 8.3.2 旋转矩阵构造

```matlab
% 绕 z 轴旋转
Rz = @(t) [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];

% 绕 y 轴旋转
Ry = @(t) [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];

% 绕 x 轴旋转
Rx = @(t) [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];

% 齐次变换矩阵
T = [C, r; 0 0 0 1];
```

#### 8.3.3 雅可比矩阵计算

```matlab
% 方法一：直接微分（解析雅可比）
x_ee = forward_kinematics(q);  % 末端位置（符号表达式）
J = jacobian(x_ee, q);         % 自动求偏导

% 方法二：几何雅可比（逐列构造）
% 对于旋转关节 i:
%   J_P(:,i) = cross(e_i, r_0E - r_0i)
%   J_R(:,i) = e_i
```

#### 8.3.4 动力学方程推导 (Lagrange 方法)

```matlab
% 1. 计算动能
T_total = 0;
for i = 1:n
    v_i = jacobian(r_Si, q) * dq;    % 质心线速度
    omega_i = J_Ri * dq;              % 角速度
    T_i = 0.5 * m(i) * (v_i.' * v_i) + 0.5 * omega_i.' * Theta_i * omega_i;
    T_total = T_total + simplify(T_i);
end

% 2. 计算势能
U_total = 0;
for i = 1:n
    U_i = m(i) * g * h_i(q);  % 重力势能
    U_total = U_total + U_i;
end

% 3. Lagrangian
L = T_total - U_total;

% 4. Euler-Lagrange 方程
EoM = sym(zeros(n, 1));
for i = 1:n
    dL_dqi = diff(L, q(i));
    dL_ddqi = diff(L, dq(i));
    % 对 dL_ddqi 求时间导数
    ddL_ddqi_dt = jacobian(dL_ddqi, q) * dq + jacobian(dL_ddqi, dq) * ddq;
    EoM(i) = simplify(ddL_ddqi_dt - dL_dqi);
end
```

#### 8.3.5 提取质量矩阵和非线性项

```matlab
% 从 EoM 中提取 M(q)
M = sym(zeros(n, n));
for i = 1:n
    for j = 1:n
        M(i,j) = diff(EoM(i), ddq(j));
    end
end
M = simplify(M);

% 非线性项 (科氏力 + 离心力 + 重力)
h = simplify(EoM - M * ddq);

% 重力项
g_vec = subs(h, dq, zeros(n,1));

% 科氏力和离心力项
b_vec = simplify(h - g_vec);
```

#### 8.3.6 数值逆运动学

```matlab
function q_sol = inverse_kinematics(x_des, q0, tol, max_iter)
    q = q0;
    for k = 1:max_iter
        x_curr = forward_kinematics(q);
        e = x_des - x_curr;
        if norm(e) < tol
            break;
        end
        J = compute_jacobian(q);
        % 阻尼最小二乘
        lambda = 0.01;
        dq = J' * ((J * J' + lambda^2 * eye(size(J,1))) \ e);
        q = q + dq;
    end
    q_sol = q;
end
```

#### 8.3.7 逆动力学控制仿真

```matlab
function tau = inverse_dynamics_controller(q, dq, q_d, dq_d, ddq_d, Kp, Kd)
    % 计算动力学参数
    M = mass_matrix(q);
    b = coriolis_centrifugal(q, dq);
    g = gravity_vector(q);
    
    % 误差
    e = q_d - q;
    de = dq_d - dq;
    
    % 指令加速度
    a_cmd = ddq_d + Kd * de + Kp * e;
    
    % 控制力矩
    tau = M * a_cmd + b + g;
end
```

#### 8.3.8 常用 MATLAB 技巧

```matlab
% 符号简化
simplify(expr)
collect(expr, q1)       % 按 q1 合并同类项

% 符号替换
subs(expr, old, new)
subs(M, {q1, q2}, {0, pi/2})  % 代入特定值

% 符号转数值
double(subs(expr, {q1,q2}, {0.1, 0.2}))

% 反对称矩阵
skew = @(v) [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];

% 验证矩阵性质
eig(M)                  % 检查正定性（所有特征值 > 0）
norm(M - M')            % 检查对称性（应为 0）
```

---

### 8.4 概念辨析与易混淆点 (Conceptual Clarifications)

以下是课程中容易混淆的概念对比，考试中经常以判断题或简答题的形式出现。

#### 解析雅可比 vs 几何雅可比

| 特性 | 解析雅可比 $J_A$ | 几何雅可比 $J_G$ |
|------|-----------------|-----------------|
| 定义 | $\dot{\mathbf{x}} = J_A \dot{\mathbf{q}}$ | $(\mathbf{v}, \boldsymbol{\omega})^T = J_G \dot{\mathbf{q}}$ |
| 旋转部分 | 欧拉角速率 $\dot{\boldsymbol{\Phi}}$ | 角速度 $\boldsymbol{\omega}$ |
| 依赖参数化 | 是（取决于欧拉角约定） | 否（物理量） |
| 奇异性 | 雅可比奇异 + 表示奇异 | 仅雅可比奇异 |
| 力映射 | 不直接适用 | $\boldsymbol{\tau} = J_G^T \mathbf{F}$ |
| 动力学中使用 | 较少 | 常用 |

> **关键区别**：$\dot{\boldsymbol{\Phi}}$ 不是角速度！欧拉角的时间导数不具有物理角速度的含义。两者通过映射矩阵 $E$ 联系：$\boldsymbol{\omega} = E(\boldsymbol{\Phi})\dot{\boldsymbol{\Phi}}$。

#### 逆动力学控制 vs 阻抗控制

| 特性 | 逆动力学控制 | 阻抗控制 |
|------|------------|---------|
| 目标 | 精确轨迹跟踪 | 期望的动态行为 |
| 模型需求 | 完整动力学模型 | 仅需重力补偿 |
| 闭环行为 | 线性解耦 | 弹簧-阻尼系统 |
| 对外力的响应 | 抵抗（刚性） | 顺从（柔性） |
| 适用场景 | 自由空间运动 | 接触任务 |
| 鲁棒性 | 对模型误差敏感 | 对模型误差鲁棒 |
| 计算量 | 大（需要 $M, \mathbf{b}, \mathbf{g}$） | 小 |

#### 关节空间 vs 任务空间控制

| 特性 | 关节空间 | 任务空间 |
|------|---------|---------|
| 误差定义 | $\mathbf{e} = \mathbf{q}_d - \mathbf{q}$ | $\mathbf{e}_x = \mathbf{x}_d - \mathbf{x}$ |
| 需要逆运动学 | 是（离线规划） | 否（在线计算） |
| 奇异性处理 | 无需（关节空间无奇异） | 需要（雅可比奇异） |
| 冗余利用 | 需要额外处理 | 自然支持（零空间） |
| 直觉性 | 关节层面 | 末端执行器层面 |

#### 完整约束 vs 非完整约束

| 特性 | 完整约束 | 非完整约束 |
|------|---------|-----------|
| 数学形式 | $\mathbf{h}(\mathbf{q}) = \mathbf{0}$ | $A(\mathbf{q})\dot{\mathbf{q}} = \mathbf{0}$（不可积） |
| 减少自由度 | 是 | 否（减少瞬时可达速度方向） |
| 例子 | 刚性连杆约束、接触点位置 | 纯滚动、刀刃约束 |
| 可达构型空间 | 降维子流形 | 全构型空间（但路径受限） |

---

### 8.5 重要性质与定理总结 (Important Properties & Theorems)

#### 性质 1：$\dot{M} - 2C$ 的反对称性

$$\mathbf{x}^T(\dot{M}(\mathbf{q}) - 2C(\mathbf{q}, \dot{\mathbf{q}}))\mathbf{x} = 0, \quad \forall \mathbf{x} \in \mathbb{R}^n$$

**前提条件**：$C$ 必须用 Christoffel 符号定义。

**应用**：
- Lyapunov 稳定性分析中消除交叉项
- 证明阻抗控制和自适应控制的稳定性
- 被动性（passivity）分析

**证明思路**：
$$\dot{M}_{ij} = \sum_k \frac{\partial M_{ij}}{\partial q_k}\dot{q}_k = \sum_k (c_{ijk} + c_{jik})\dot{q}_k$$

因此 $\dot{M} = C + C^T$，即 $\dot{M} - 2C = C^T - C$，这是一个反对称矩阵。

#### 性质 2：质量矩阵的正定性

$M(\mathbf{q})$ 对所有 $\mathbf{q}$ 正定，因为：

$$\dot{\mathbf{q}}^T M \dot{\mathbf{q}} = 2T \geq 0$$

且等号仅在 $\dot{\mathbf{q}} = \mathbf{0}$ 时成立（假设没有冗余坐标）。

**物理意义**：动能总是非负的，且只有在系统静止时为零。

#### 性质 3：能量守恒（无耗散、无外力时）

$$\frac{d}{dt}(T + U) = \dot{\mathbf{q}}^T \boldsymbol{\tau}$$

当 $\boldsymbol{\tau} = \mathbf{0}$ 时，总机械能守恒。

**证明**：

$$\frac{dT}{dt} = \dot{\mathbf{q}}^T M \ddot{\mathbf{q}} + \frac{1}{2}\dot{\mathbf{q}}^T \dot{M} \dot{\mathbf{q}}$$

利用运动方程和 $\dot{M} - 2C$ 的反对称性：

$$\frac{dT}{dt} = \dot{\mathbf{q}}^T(\boldsymbol{\tau} - \mathbf{g}) + \frac{1}{2}\dot{\mathbf{q}}^T \dot{M} \dot{\mathbf{q}} - \dot{\mathbf{q}}^T C \dot{\mathbf{q}} = \dot{\mathbf{q}}^T \boldsymbol{\tau} - \dot{\mathbf{q}}^T \mathbf{g}$$

而 $\frac{dU}{dt} = \dot{\mathbf{q}}^T \mathbf{g}$，因此 $\frac{d}{dt}(T+U) = \dot{\mathbf{q}}^T \boldsymbol{\tau}$。

#### 定理：毛球定理 (Hairy Ball Theorem) 的推论

> 不存在 $SO(3)$ 上的全局无奇异三参数表示。

**含义**：任何用三个参数描述旋转的方法（欧拉角、轴角等）都必然存在奇异点。要避免奇异性，必须使用冗余参数（如四元数的 4 个参数或旋转矩阵的 9 个参数）。

---

### 8.6 复习策略与考试建议 (Study Strategy & Exam Tips)

#### 复习优先级（按重要性排序）

1. **最高优先级**：运动方程的推导（Lagrange 和 Projected N-E 方法）、雅可比矩阵的计算、逆动力学控制
2. **高优先级**：旋转参数化及其转换、正运动学、阻抗控制、任务空间控制
3. **中等优先级**：浮动基座系统、足式机器人全身控制、四旋翼动力学与控制
4. **了解即可**：固定翼飞行器的详细气动模型、各飞行模态的具体特征

#### 考试中的时间分配建议

- **阅读题目**：5 分钟（通读所有题目，确定难度和分值分布）
- **计算题**：每道 15-25 分钟（运动学/动力学推导通常耗时较长）
- **概念题**：每道 5-10 分钟
- **检查**：预留 10-15 分钟

#### 常见失分点

1. **符号错误**：旋转方向、坐标系定义、叉积顺序
2. **遗漏项**：动能中的旋转部分、科氏力项、重力补偿
3. **维度不匹配**：矩阵乘法的维度检查
4. **奇异性忽略**：未讨论雅可比矩阵的奇异构型
5. **物理意义缺失**：只给公式不解释物理含义
6. **单位不一致**：角度（弧度 vs 度）、力（N vs kN）

#### 解题检查清单

- [ ] 坐标系定义是否明确？
- [ ] 广义坐标的选择是否合理？
- [ ] 质量矩阵是否对称正定？
- [ ] 运动方程的维度是否一致？
- [ ] 控制律是否包含所有必要的补偿项？
- [ ] 稳定性分析是否完整（Lyapunov 函数 + $\dot{V}$ 的符号）？
- [ ] 特殊情况（零位、奇异构型）是否讨论？

---

### 8.7 课程知识图谱 (Course Knowledge Map)

```
Robot Dynamics 课程结构
│
├── 运动学 (Kinematics)
│   ├── 位置描述: 笛卡尔 / 柱坐标 / 球坐标
│   ├── 姿态描述: 旋转矩阵 / 欧拉角 / 轴角 / 四元数
│   ├── 正运动学: 齐次变换 T, DH 参数
│   ├── 微分运动学: 解析雅可比 J_A / 几何雅可比 J_G
│   └── 逆运动学: 封闭解 / 数值迭代 / 零空间利用
│
├── 动力学 (Dynamics)
│   ├── 运动方程: M(q)q̈ + b(q,q̇) + g(q) = τ
│   ├── 推导方法: Newton-Euler / Projected N-E / Lagrange
│   ├── 质量矩阵性质: 对称正定, Ṁ-2C 反对称
│   └── 能量: 动能 T / 势能 U / 能量守恒
│
├── 控制 (Control)
│   ├── 运动学层: 逆微分运动学 / 多任务控制
│   ├── 关节空间: 阻抗控制 / 逆动力学控制
│   ├── 任务空间: 操作空间动力学 / 力-运动混合控制
│   └── 稳定性: Lyapunov 分析 / 被动性
│
├── 浮动基座 (Floating Base)
│   ├── 广义坐标: 基座位姿 + 关节角
│   ├── 运动方程: 选择矩阵 S / 接触力
│   └── 接触约束: 接触雅可比 / 摩擦锥
│
├── 足式机器人 (Legged Robots)
│   ├── 稳定性: 静态 (CoM) / 动态 (ZMP)
│   ├── 约束: 完整 / 非完整
│   └── 全身控制: 层次化优化 / QP
│
└── 飞行器 (Aerial Robots)
    ├── 旋翼: 推力模型 / 欠驱动 / 级联控制
    └── 固定翼: 气动力 / 6-DOF / 配平 / 模态分析
```

---

## 附录：符号表 (Notation Reference)

| 符号 | 含义 | 英文 |
|------|------|------|
| $\mathbf{q}$ | 广义坐标 | Generalized coordinates |
| $\dot{\mathbf{q}}$ | 广义速度 | Generalized velocities |
| $\mathbf{u}$ | 广义速度（浮动基座） | Generalized velocities (floating base) |
| $M(\mathbf{q})$ | 质量/惯性矩阵 | Mass/inertia matrix |
| $C(\mathbf{q}, \dot{\mathbf{q}})$ | 科氏力矩阵 | Coriolis matrix |
| $\mathbf{b}(\mathbf{q}, \dot{\mathbf{q}})$ | 科氏力和离心力向量 | Coriolis and centrifugal vector |
| $\mathbf{g}(\mathbf{q})$ | 重力向量 | Gravity vector |
| $\boldsymbol{\tau}$ | 广义力/关节力矩 | Generalized forces/joint torques |
| $J$ | 雅可比矩阵 | Jacobian matrix |
| $J^{\dagger}$ | 伪逆 | Pseudoinverse |
| $C_{AB}$ | 从 B 系到 A 系的旋转矩阵 | Rotation matrix from B to A |
| $T_{AB}$ | 从 B 系到 A 系的齐次变换 | Homogeneous transformation |
| ${}_A\mathbf{r}_{AB}$ | 在 A 系中表示的从 A 到 B 的位置向量 | Position vector from A to B in frame A |
| $\boldsymbol{\omega}$ | 角速度 | Angular velocity |
| $[\cdot]_\times$ | 反对称矩阵（叉积矩阵） | Skew-symmetric matrix |
| $\Theta$ | 惯性张量 | Inertia tensor |
| $SO(3)$ | 三维旋转群 | Special orthogonal group |
| $SE(3)$ | 三维刚体运动群 | Special Euclidean group |
| $\mathcal{L}$ | Lagrangian 函数 | Lagrangian |
| $T$ | 动能 | Kinetic energy |
| $U$ | 势能 | Potential energy |
| $\Lambda$ | 任务空间惯性矩阵 | Operational space inertia matrix |
| $N$ | 零空间投影矩阵 | Null-space projector |
| $S$ | 选择矩阵 | Selection matrix |
| $J_c$ | 接触雅可比 | Contact Jacobian |
| $\mathbf{F}_c$ | 接触力 | Contact forces |
| $\mu$ | 摩擦系数 | Friction coefficient |

---

> **祝考试顺利！** 理解物理意义比死记公式更重要。每个公式背后都有清晰的物理图像——力的平衡、能量的转换、约束的满足。抓住这些核心思想，具体公式自然水到渠成。
>
> *"The purpose of computing is insight, not numbers."* — Richard Hamming
