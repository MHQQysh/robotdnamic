# Lecture 8: Legged Robotics (足式机器人)

> ETH Zurich - Robot Dynamics 课程讲义
> 基于 legged.pdf 整理

---

## 目录

1. [为什么需要足式机器人](#1-为什么需要足式机器人)
2. [足式机器人历史](#2-足式机器人历史)
3. [静态稳定 vs 动态稳定](#3-静态稳定-vs-动态稳定)
4. [浮动基座系统的运动学](#4-浮动基座系统的运动学)
5. [微分运动学](#5-微分运动学)
6. [逆微分运动学](#6-逆微分运动学)
7. [运动学奇异性](#7-运动学奇异性)
8. [足式机器人的动力学与控制](#8-足式机器人的动力学与控制)

---

## 1. 为什么需要足式机器人

### 1.1 动机 (Motivation)

地球表面大约有 **50%** 的陆地区域是轮式或履带式车辆无法到达的。足式系统 (legged systems) 具有独特的优势:

- **离散接触 (Discrete Contact)**: 足式机器人仅需要离散的立足点 (footholds), 而非连续平坦的路面
- **障碍跨越 (Obstacle Traversal)**: 可以跨越沟壑、攀爬台阶、穿越碎石地形
- **地形适应 (Terrain Adaptation)**: 通过调整步态和足端轨迹适应不同地形
- **全向运动 (Omnidirectional Motion)**: 可以在任意方向上移动, 包括原地转向

### 1.2 核心挑战 (Key Challenges)

| 挑战 | 描述 |
|------|------|
| 多自由度协调控制 | 需要同时协调 12-30+ 个关节 |
| 不确定地形交互 | 地形几何和摩擦特性未知 |
| 动态平衡 | 在运动过程中维持稳定性 |
| 能量效率 | 足式运动的能耗远高于轮式 |
| 接触规划 | 选择合适的立足点序列 |

> **核心思想**: 足式机器人的本质是一个 **欠驱动 (underactuated)** 的多接触系统, 其运动能力取决于与环境的接触状态。

---

## 2. 足式机器人历史

### 2.1 早期机械装置 (Early Mechanical Devices)

足式运动的机械化探索可以追溯到 19 世纪:

- **1878 年 — L.A. Davis 专利**: 最早的机械步行装置专利之一, 使用连杆机构 (linkage mechanism) 模拟腿部运动
- **1893 年 — Rygg 专利**: 一种"机械马" (Mechanical Horse), 通过曲柄连杆将旋转运动转化为步行运动

### 2.2 20 世纪中期发展

- **1957-1960 年 — Shigley 步行机器**: 由 J.E. Shigley 设计的早期步行机器人原型, 使用 Chebyshev 连杆机构生成近似直线的足端轨迹

- **Strandbeest (Theo Jansen)**: 荷兰艺术家 Theo Jansen 创造的风力驱动步行结构, 使用精心设计的多连杆机构 (multi-bar linkage), 展示了纯机械结构实现复杂步行运动的可能性。其腿部机构的关键参数经过进化算法优化

- **GE Walking Truck (1960s)**: 由 General Electric 开发的四足步行卡车, 由人类操作员通过力反馈控制四条液压腿。这是最早的大型足式机器之一, 但操作极其困难, 操作员在几分钟内就会疲惫不堪

### 2.3 早期机器人 (Early Robots)

- **Hardiman (1965)**: GE 开发的外骨骼 (exoskeleton) 项目, 旨在增强人体力量。虽然未完全成功, 但推动了力控制技术的发展

- **Phony Pony (1968)**: 由 McGhee 和 Frank 在南加州大学开发, 是最早的计算机控制步行机器人之一。四足设计, 每条腿 2 个自由度 (DOF), 实现了基本的静态稳定行走

### 2.4 现代发展

- **Honda Asimo (2000-2011)**: 本田公司开发的标志性人形机器人 (humanoid robot), 实现了动态行走、跑步、上下楼梯等功能。采用 ZMP (Zero Moment Point) 控制方法

- **Toyota Humanoid**: 丰田开发的人形机器人, 展示了高速奔跑和复杂操作能力

- **DARPA Robotics Challenge (2012)**: 美国国防高级研究计划局举办的机器人挑战赛, 推动了人形机器人在灾难响应场景中的应用。参赛机器人需要完成驾驶、穿越废墟、开门、爬梯等任务

> **发展趋势**: 从纯机械连杆 → 遥操作 → 预编程步态 → 传感器反馈控制 → 全身动态控制

---

## 3. 静态稳定 vs 动态稳定

### 3.1 静态稳定 (Static Stability)

**定义**: 在任意时刻停止运动, 机器人仍能保持平衡不倒。

**条件**: 重心 (Center of Mass, CoM) 的垂直投影必须落在 **支撑多边形 (Support Polygon)** 内部。

$$\mathbf{p}_{\text{CoM}}^{xy} \in \text{ConvexHull}(\mathbf{p}_{f_1}, \mathbf{p}_{f_2}, \ldots, \mathbf{p}_{f_k})$$

其中 $\mathbf{p}_{f_i}$ 是第 $i$ 个支撑足的位置, $k$ 是支撑足数量。

**特点**:
- 至少需要 **3 条腿** 同时支撑 (形成三角形支撑区域)
- 对于四足机器人, 静态行走时同一时刻最多抬起 **1 条腿**
- **优点**: 安全、可靠、对控制精度要求低
- **缺点**: 速度慢、能量效率低

**稳定裕度 (Stability Margin)**:

$$s = \min_{i} d(\mathbf{p}_{\text{CoM}}^{xy}, \text{edge}_i)$$

即 CoM 投影到支撑多边形各边的最小距离。$s > 0$ 表示静态稳定。

### 3.2 动态稳定 (Dynamic Stability)

**定义**: 机器人必须持续运动才能维持平衡, 一旦停止就会倒下。

**典型例子**: 人类行走本质上是一种 "受控跌倒" (controlled falling)。

**ZMP 准则 (Zero Moment Point Criterion)**:

$$\mathbf{p}_{\text{ZMP}} = \frac{\sum_i m_i (\ddot{z}_i + g) \mathbf{p}_i^{xy} - \sum_i m_i \ddot{\mathbf{p}}_i^{xy} z_i}{\sum_i m_i (\ddot{z}_i + g)}$$

当 ZMP 在支撑多边形内时, 机器人动态稳定。

**特点**:
- 可以在仅 **1-2 条腿** 支撑时运动 (如跑步、跳跃)
- **优点**: 速度快、能量效率高、运动更自然
- **缺点**: 对驱动器 (actuator) 响应速度和控制算法要求极高

### 3.3 对比总结

| 特性 | 静态稳定 | 动态稳定 |
|------|---------|---------|
| 最少支撑腿数 | 3 | 1 (甚至 0, 如跳跃腾空阶段) |
| 速度 | 慢 | 快 |
| 能量效率 | 低 | 高 |
| 控制复杂度 | 低 | 高 |
| 传感器需求 | 低 | 高 (IMU, 力传感器等) |
| 鲁棒性 | 高 (可随时停止) | 需要持续控制 |

### Quiz 3.1: 静态稳定判断

**问题**: 一个四足机器人的四个足端位置为:

$$\mathbf{p}_1 = (0.3, 0.2), \quad \mathbf{p}_2 = (0.3, -0.2), \quad \mathbf{p}_3 = (-0.3, 0.2), \quad \mathbf{p}_4 = (-0.3, -0.2)$$

当抬起第 4 条腿时, CoM 投影在 $(0.05, 0.02)$。机器人是否静态稳定?

**解答**:

抬起第 4 条腿后, 支撑多边形由 $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$ 构成的三角形:

$$\triangle = \{(0.3, 0.2),\ (0.3, -0.2),\ (-0.3, 0.2)\}$$

判断点 $(0.05, 0.02)$ 是否在三角形内部。使用重心坐标法:

$$\mathbf{p} = \alpha \mathbf{p}_1 + \beta \mathbf{p}_2 + \gamma \mathbf{p}_3, \quad \alpha + \beta + \gamma = 1$$

求解得 $\alpha = 0.3$, $\beta = 0.22$, $\gamma = 0.48$, 三个系数均为正。

**结论**: CoM 投影在支撑三角形内部, 机器人**静态稳定**。

---

## 4. 浮动基座系统的运动学

### 4.1 浮动基座概念 (Floating Base Concept)

与固定基座的工业机械臂不同, 足式机器人的基座 (躯干/body) 并不固定在世界坐标系中, 而是 **自由浮动 (free-floating)** 的。基座的位置和姿态由腿部与地面的接触间接决定。

**浮动基座的描述**:

基座位姿用 6 个广义坐标表示:

$$\mathbf{q}_b = \begin{pmatrix} \mathbf{r}_b \ \mathbf{\Theta}_b \end{pmatrix} = \begin{pmatrix} x_b \ y_b \ z_b \ \phi_b \ \theta_b \ \psi_b \end{pmatrix} \in \mathbb{R}^6$$

其中 $\mathbf{r}_b = (x_b, y_b, z_b)^T$ 为基座位置, $\mathbf{\Theta}_b = (\phi_b, \theta_b, \psi_b)^T$ 为基座姿态 (如 Euler 角表示)。

### 4.2 四足机器人静态行走示例

考虑一个四足机器人 (quadruped), 每条腿有 **3 个旋转关节** (HAA: Hip Abduction/Adduction, HFE: Hip Flexion/Extension, KFE: Knee Flexion/Extension)。

**广义坐标 (Generalized Coordinates)**:

$$\mathbf{q} = \begin{pmatrix} \mathbf{q}_b \ \mathbf{q}_j \end{pmatrix} \in \mathbb{R}^{n_q}$$

各部分自由度:

| 组成部分 | 自由度数 | 说明 |
|---------|---------|------|
| 基座坐标 $\mathbf{q}_b$ | 6 | 3 平移 + 3 旋转 |
| 驱动关节坐标 $\mathbf{q}_j$ | 12 | 4 条腿 × 3 关节/腿 |
| **总广义坐标** $n_q$ | **18** | $6 + 12 = 18$ |

$$n_q = n_b + n_j = 6 + 12 = 18$$

### 4.3 接触约束 (Contact Constraints)

当一只脚与地面接触且不滑动时, 该足端的速度为零, 产生 **全约束 (holonomic constraint)**:

$$\mathbf{v}_{f_i} = \mathbf{0} \in \mathbb{R}^3$$

即足端在 $x, y, z$ 三个方向上的速度均为零 (假设点接触, point contact)。

**静态行走时 (3 条腿支撑)**:

$$\text{接触约束数} = 3 \times 3 = 9$$

每个接触足提供 3 个标量约束 (三维空间中的点接触不滑动条件)。

### 4.4 可用自由度分析

系统总自由度减去约束数, 得到可调自由度:

$$n_{\text{free}} = n_q - n_c = 18 - 9 = 9$$

这 9 个自由度的分配:

| 自由度类型 | 数量 | 说明 |
|-----------|------|------|
| 基座运动 | 6 | 基座位置 (3) + 姿态 (3) |
| 摆动腿运动 | 3 | 摆动腿 (swing leg) 的 3 个关节 |
| **合计** | **9** | 与 $n_{\text{free}}$ 一致 |

> **关键洞察**: 在 3 条腿支撑的情况下, 基座的 6 个自由度可以通过支撑腿的 9 个关节 (3 腿 × 3 关节) 在 9 个约束下完全确定。剩余的 3 个自由度恰好对应摆动腿的运动。

### Quiz 4.1: 广义坐标计算

**问题**: 一个六足机器人 (hexapod), 每条腿有 3 个关节。在静态行走时, 采用 tripod gait (三脚步态), 始终有 3 条腿支撑。求:
(a) 总广义坐标数
(b) 接触约束数
(c) 可用自由度数

**解答**:

(a) 总广义坐标:

$$n_q = n_b + n_j = 6 + (6 \times 3) = 6 + 18 = 24$$

(b) 接触约束 (3 条腿支撑, 点接触):

$$n_c = 3 \times 3 = 9$$

(c) 可用自由度:

$$n_{\text{free}} = n_q - n_c = 24 - 9 = 15$$

其中: 6 个基座自由度 + 9 个摆动腿自由度 (3 条摆动腿 × 3 关节/腿) = 15。

---

## 5. 微分运动学

### 5.1 接触约束的微分形式

对于第 $i$ 个接触足, 足端速度为零的约束可以写成:

$$\mathbf{v}_{f_i} = \mathbf{J}_i(\mathbf{q}) \dot{\mathbf{q}} = \mathbf{0}$$

其中 $\mathbf{J}_i \in \mathbb{R}^{3 \times n_q}$ 是第 $i$ 个足端的 Jacobian 矩阵。

将所有接触约束堆叠:

$$\mathbf{J}_c \dot{\mathbf{q}} = \mathbf{0}$$

$$\mathbf{J}_c = \begin{pmatrix} \mathbf{J}_1 \ \mathbf{J}_2 \ \mathbf{J}_3 \end{pmatrix} \in \mathbb{R}^{9 \times 18}$$

### 5.2 Jacobian 的结构

将广义坐标分为基座部分和关节部分:

$$\dot{\mathbf{q}} = \begin{pmatrix} \dot{\mathbf{q}}_b \ \dot{\mathbf{q}}_j \end{pmatrix}$$

对应地, Jacobian 也可以分块:

$$\mathbf{J}_i = \begin{pmatrix} \mathbf{J}_{i,b} & \mathbf{J}_{i,j} \end{pmatrix}$$

其中:
- $\mathbf{J}_{i,b} \in \mathbb{R}^{3 \times 6}$: 足端速度对基座速度的 Jacobian
- $\mathbf{J}_{i,j} \in \mathbb{R}^{3 \times 12}$: 足端速度对关节速度的 Jacobian

**重要特性**: $\mathbf{J}_{i,j}$ 具有稀疏结构, 因为第 $i$ 条腿的足端速度只与该腿的 3 个关节有关:

$$\mathbf{J}_{i,j} = \begin{pmatrix} \mathbf{0} & \cdots & \mathbf{J}_{i,\text{leg}_i} & \cdots & \mathbf{0} \end{pmatrix}$$

其中 $\mathbf{J}_{i,\text{leg}_i} \in \mathbb{R}^{3 \times 3}$ 是第 $i$ 条腿自身的 Jacobian。

### 5.3 约束分析

**9 个独立约束**:

在一般配置下, $\mathbf{J}_c$ 的秩为 9, 即 9 个接触约束是独立的:

$$\text{rank}(\mathbf{J}_c) = 9$$

**6 个独立基座约束**:

从接触约束中可以提取出对基座运动的约束。将约束方程展开:

$$\mathbf{J}_{c,b} \dot{\mathbf{q}}_b + \mathbf{J}_{c,j} \dot{\mathbf{q}}_j = \mathbf{0}$$

其中 $\mathbf{J}_{c,b} \in \mathbb{R}^{9 \times 6}$。在一般配置下:

$$\text{rank}(\mathbf{J}_{c,b}) = 6$$

这意味着 9 个接触约束中有 6 个独立地约束了基座的运动, 基座的所有 6 个自由度都被约束。

### 5.4 自由度分配详解

从约束方程出发, 可以将系统的运动分解为:

**支撑腿关节速度** $\dot{\mathbf{q}}_{\text{st}}$ (stance legs, 9 维):

$$\dot{\mathbf{q}}_{\text{st}} = -\mathbf{J}_{c,\text{st}}^{-1} \mathbf{J}_{c,b} \dot{\mathbf{q}}_b$$

(假设 $\mathbf{J}_{c,\text{st}}$ 可逆, 即支撑腿不在奇异位形)

**摆动腿关节速度** $\dot{\mathbf{q}}_{\text{sw}}$ (swing leg, 3 维): 自由, 不受接触约束限制。

**内力方向 (Internal Force Directions)**: 9 个约束中有 6 个约束基座, 剩余 $9 - 6 = 3$ 个约束对应 **内力方向**。这些方向上的关节运动不会改变基座位姿, 但会改变支撑腿之间的内力分布。

$$n_{\text{internal}} = n_c - n_b = 9 - 6 = 3$$

### Quiz 5.1: 约束自由度分析

**问题**: 一个四足机器人在 trot gait (对角步态) 中, 有 2 条对角腿支撑。求:
(a) 接触约束数
(b) 可用自由度数
(c) 基座是否完全被约束?

**解答**:

(a) 接触约束:

$$n_c = 2 \times 3 = 6$$

(b) 可用自由度:

$$n_{\text{free}} = 18 - 6 = 12$$

其中: 摆动腿自由度 = $2 \times 3 = 6$, 剩余 $12 - 6 = 6$ 个自由度对应基座运动。

(c) 基座约束分析:

$\mathbf{J}_{c,b} \in \mathbb{R}^{6 \times 6}$, 在一般配置下 $\text{rank}(\mathbf{J}_{c,b}) = 6$。

**结论**: 基座的 6 个自由度恰好被 6 个接触约束完全约束。但此时没有冗余约束, 即 $n_{\text{internal}} = 6 - 6 = 0$, 没有内力方向。这意味着接触力是唯一确定的 (statically determinate)。

---

## 6. 逆微分运动学

### 6.1 问题定义

逆微分运动学 (Inverse Differential Kinematics) 的目标: 给定期望的任务空间速度 (如摆动腿足端速度、基座速度), 求解所需的广义速度 $\dot{\mathbf{q}}$。

**已知条件**:
- 接触约束: $\mathbf{J}_c \dot{\mathbf{q}} = \mathbf{0}$
- 期望摆动腿足端速度: $\mathbf{v}_{\text{sw}}^{\text{des}}$

**求解目标**: 广义速度 $\dot{\mathbf{q}}$

### 6.2 公式推导

摆动腿的足端速度可以表示为:

$$\mathbf{v}_{\text{sw}} = \mathbf{J}_{\text{sw}} \dot{\mathbf{q}}$$

其中 $\mathbf{J}_{\text{sw}} \in \mathbb{R}^{3 \times 18}$ 是摆动腿足端的 Jacobian。

将接触约束和摆动腿任务合并为一个方程组:

$$\begin{pmatrix} \mathbf{J}_c \ \mathbf{J}_{\text{sw}} \end{pmatrix} \dot{\mathbf{q}} = \begin{pmatrix} \mathbf{0} \ \mathbf{v}_{\text{sw}}^{\text{des}} \end{pmatrix}$$

即:

$$\mathbf{J}_{\text{task}} \dot{\mathbf{q}} = \mathbf{v}_{\text{task}}$$

其中 $\mathbf{J}_{\text{task}} \in \mathbb{R}^{12 \times 18}$, $\mathbf{v}_{\text{task}} \in \mathbb{R}^{12}$。

### 6.3 解的结构

方程组有 18 个未知数和 12 个方程, 属于 **欠定系统 (underdetermined system)**。

**最小范数解 (Minimum Norm Solution)**:

$$\dot{\mathbf{q}} = \mathbf{J}_{\text{task}}^{\dagger} \mathbf{v}_{\text{task}} + (\mathbf{I} - \mathbf{J}_{\text{task}}^{\dagger} \mathbf{J}_{\text{task}}) \mathbf{z}$$

其中:
- $\mathbf{J}_{\text{task}}^{\dagger} = \mathbf{J}_{\text{task}}^T (\mathbf{J}_{\text{task}} \mathbf{J}_{\text{task}}^T)^{-1}$ 是 Moore-Penrose 伪逆
- $\mathbf{z} \in \mathbb{R}^{18}$ 是任意向量, 对应零空间 (null space) 中的自由运动
- $(\mathbf{I} - \mathbf{J}_{\text{task}}^{\dagger} \mathbf{J}_{\text{task}})$ 是零空间投影矩阵

零空间维度:

$$\dim(\mathcal{N}(\mathbf{J}_{\text{task}})) = 18 - 12 = 6$$

这 6 维零空间对应:
- **3 个内力方向**: 改变支撑腿间的力分布, 不影响运动
- **3 个冗余运动方向**: 可用于次要任务优化

### 6.4 解的唯一性问题

**问题**: 解是否唯一?

**回答**: 不唯一。由于系统存在 6 维零空间, 有无穷多组关节速度可以实现相同的任务空间速度。

**物理解释**: 在满足接触约束和摆动腿速度要求的前提下, 支撑腿的关节速度仍有多种选择。不同的选择对应不同的:
- 基座运动轨迹
- 支撑腿内力分布
- 关节速度分配

### 6.5 不移动摆动腿关节也能跟踪轨迹

**重要观察**: 即使摆动腿的关节速度为零 ($\dot{\mathbf{q}}_{\text{sw}} = \mathbf{0}$), 摆动腿的足端仍然可以有非零速度。

这是因为摆动腿足端速度包含两部分:

$$\mathbf{v}_{\text{sw}} = \mathbf{J}_{\text{sw},b} \dot{\mathbf{q}}_b + \mathbf{J}_{\text{sw,leg}} \dot{\mathbf{q}}_{\text{sw}}$$

当基座运动时 ($\dot{\mathbf{q}}_b \neq \mathbf{0}$), 即使 $\dot{\mathbf{q}}_{\text{sw}} = \mathbf{0}$:

$$\mathbf{v}_{\text{sw}} = \mathbf{J}_{\text{sw},b} \dot{\mathbf{q}}_b \neq \mathbf{0}$$

> **直觉理解**: 想象你站在一条腿上, 另一条腿悬空不动。当你的身体 (基座) 前倾时, 悬空腿的脚尖会相对于地面向后移动, 尽管腿部关节没有任何运动。

### 6.6 加入基座运动任务

为了获得唯一解, 可以添加基座运动任务:

$$\mathbf{v}_b^{\text{des}} = \mathbf{J}_b \dot{\mathbf{q}} = \begin{pmatrix} \mathbf{I}_{6 \times 6} & \mathbf{0}_{6 \times 12} \end{pmatrix} \dot{\mathbf{q}}$$

完整的任务方程:

$$\underbrace{\begin{pmatrix} \mathbf{J}_c \ \mathbf{J}_{\text{sw}} \ \mathbf{J}_b \end{pmatrix}}_{\mathbf{J}_{\text{full}} \in \mathbb{R}^{18 \times 18}} \dot{\mathbf{q}} = \begin{pmatrix} \mathbf{0} \ \mathbf{v}_{\text{sw}}^{\text{des}} \ \mathbf{v}_b^{\text{des}} \end{pmatrix}$$

当 $\mathbf{J}_{\text{full}}$ 满秩时, 解唯一:

$$\dot{\mathbf{q}} = \mathbf{J}_{\text{full}}^{-1} \begin{pmatrix} \mathbf{0} \ \mathbf{v}_{\text{sw}}^{\text{des}} \ \mathbf{v}_b^{\text{des}} \end{pmatrix}$$

### Quiz 6.1: 逆运动学求解

**问题**: 一个四足机器人在三腿支撑下行走。已知:
- 期望基座前进速度: $v_x = 0.5$ m/s
- 期望摆动腿足端速度: $\mathbf{v}_{\text{sw}} = (0.3, 0, 0.1)^T$ m/s
- 系统无奇异

(a) 仅给定摆动腿速度和接触约束, 解是否唯一?
(b) 加入完整基座速度任务后, 解是否唯一?
(c) 零空间维度分别是多少?

**解答**:

(a) 仅接触约束 + 摆动腿速度:
- 方程数: $9 + 3 = 12$
- 未知数: $18$
- 零空间维度: $18 - 12 = 6$
- **解不唯一**, 有 6 维自由度

(b) 加入基座速度任务:
- 方程数: $9 + 3 + 6 = 18$
- 未知数: $18$
- 若 $\mathbf{J}_{\text{full}}$ 满秩, **解唯一**

(c) 零空间维度:
- 情况 (a): $\dim(\mathcal{N}) = 6$
- 情况 (b): $\dim(\mathcal{N}) = 0$

---

## 7. 运动学奇异性

### 7.1 奇异性概述 (Singularity Overview)

运动学奇异性 (kinematic singularity) 发生在 Jacobian 矩阵失去满秩的配置下。在奇异配置中, 某些任务空间方向上的运动能力丧失, 或者需要无穷大的关节速度才能实现有限的任务空间速度。

**奇异性条件**:

$$\text{rank}(\mathbf{J}) < \min(m, n)$$

其中 $m$ 是任务空间维度, $n$ 是关节空间维度。

对于足式机器人, 奇异性可能出现在:
- 支撑腿完全伸直 (膝关节角度为 0 或 $\pi$)
- 多条支撑腿共线排列
- 基座位于特殊几何位置

### 7.2 单任务公式 (Single Task Formulation)

在单任务公式中, 所有约束和任务被合并为一个统一的方程组:

$$\mathbf{J}_{\text{full}} \dot{\mathbf{q}} = \mathbf{v}_{\text{full}}$$

$$\mathbf{J}_{\text{full}} = \begin{pmatrix} \mathbf{J}_c \ \mathbf{J}_{\text{sw}} \ \mathbf{J}_b \end{pmatrix} \in \mathbb{R}^{18 \times 18}$$

**奇异性发生条件**:

$$\det(\mathbf{J}_{\text{full}}) = 0$$

在单任务公式下, 当系统处于奇异配置时:
- 方程组无解或有无穷多解
- **所有任务同时受到影响** — 无法区分哪个任务导致了奇异性
- 伪逆解可能产生极大的关节速度

### 7.3 多任务公式 (Multi-Task Formulation)

多任务公式将不同的任务按优先级分层处理, 使用 **任务优先级框架 (Task Priority Framework)**:

**Task 1 (最高优先级): 接触约束 + 摆动腿足端运动**

$$\mathbf{J}_1 = \begin{pmatrix} \mathbf{J}_c \ \mathbf{J}_{\text{sw}} \end{pmatrix} \in \mathbb{R}^{12 \times 18}, \quad \mathbf{v}_1 = \begin{pmatrix} \mathbf{0} \ \mathbf{v}_{\text{sw}}^{\text{des}} \end{pmatrix}$$

求解:

$$\dot{\mathbf{q}}_1 = \mathbf{J}_1^{\dagger} \mathbf{v}_1$$

**Task 2 (次优先级): 基座位置 + 基座姿态**

$$\mathbf{J}_2 = \mathbf{J}_b \in \mathbb{R}^{6 \times 18}, \quad \mathbf{v}_2 = \mathbf{v}_b^{\text{des}}$$

在 Task 1 的零空间中求解:

$$\dot{\mathbf{q}}_2 = \dot{\mathbf{q}}_1 + \mathbf{N}_1 \mathbf{J}_{2|1}^{\dagger} (\mathbf{v}_2 - \mathbf{J}_2 \dot{\mathbf{q}}_1)$$

其中:
- $\mathbf{N}_1 = \mathbf{I} - \mathbf{J}_1^{\dagger} \mathbf{J}_1$ 是 Task 1 的零空间投影矩阵
- $\mathbf{J}_{2|1} = \mathbf{J}_2 \mathbf{N}_1$ 是 Task 2 在 Task 1 零空间中的投影 Jacobian

### 7.4 奇异配置下的行为差异

这是单任务与多任务公式的关键区别所在。

**场景**: 假设基座运动任务 (Task 2) 出现奇异性, 但接触约束和摆动腿任务 (Task 1) 正常。

#### 单任务公式的行为

$$\mathbf{J}_{\text{full}} \dot{\mathbf{q}} = \mathbf{v}_{\text{full}}$$

- $\det(\mathbf{J}_{\text{full}}) = 0$, 整个系统奇异
- **所有任务都受影响**: 接触约束可能被违反, 摆动腿轨迹跟踪失败
- 伪逆解: $\dot{\mathbf{q}} = \mathbf{J}_{\text{full}}^{\dagger} \mathbf{v}_{\text{full}}$, 可能产生不合理的大关节速度
- **后果严重**: 足端可能滑动, 机器人可能失去平衡

#### 多任务公式的行为

- Task 1 正常求解: 接触约束和摆动腿运动**不受影响**
- Task 2 在零空间中奇异: $\mathbf{J}_{2|1}$ 秩亏
- 基座运动在奇异方向上无法跟踪, 但在非奇异方向上仍然正常
- **后果可控**: 仅基座运动部分降级, 关键的接触安全性得到保障

**对比总结**:

| 特性 | 单任务公式 | 多任务公式 |
|------|-----------|-----------|
| 奇异性影响范围 | 全局, 所有任务 | 局部, 仅低优先级任务 |
| 接触安全性 | 可能被破坏 | 始终保障 |
| 计算复杂度 | 低 | 较高 |
| 实际应用 | 简单场景 | 复杂足式运动 |
| 鲁棒性 | 低 | 高 |

### 7.5 奇异性的物理解释

**支撑腿奇异 (Stance Leg Singularity)**:

当支撑腿完全伸直时, $\mathbf{J}_{i,\text{leg}_i}$ 秩亏:

$$\text{rank}(\mathbf{J}_{i,\text{leg}_i}) < 3$$

此时该腿无法在某个方向上产生足端速度, 对应的接触约束退化。

**运动学意义**: 在伸直配置附近, 微小的任务空间速度需要极大的关节速度:

$$\|\dot{\mathbf{q}}\| \to \infty \quad \text{as} \quad \sigma_{\min}(\mathbf{J}) \to 0$$

其中 $\sigma_{\min}$ 是 Jacobian 的最小奇异值。

**阻尼最小二乘法 (Damped Least Squares, DLS)**:

为了在奇异性附近获得数值稳定的解, 使用阻尼伪逆:

$$\mathbf{J}^{\dagger}_{\lambda} = \mathbf{J}^T (\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I})^{-1}$$

其中 $\lambda > 0$ 是阻尼系数。代价是引入跟踪误差, 但避免了关节速度爆炸。

### Quiz 7.1: 奇异性分析

**问题**: 一个四足机器人在三腿支撑下行走, 使用多任务公式。突然, 基座到达一个配置使得基座绕 $z$ 轴的旋转在 Task 1 的零空间中不可实现 (即 $\mathbf{J}_{2|1}$ 在该方向上奇异)。

(a) 接触约束是否仍然满足?
(b) 摆动腿轨迹跟踪是否受影响?
(c) 基座的平移运动是否受影响?
(d) 如果使用单任务公式, 情况会有何不同?

**解答**:

(a) **是**, 接触约束仍然满足。Task 1 (最高优先级) 包含接触约束, 其求解不受 Task 2 奇异性的影响。

(b) **否**, 摆动腿轨迹跟踪不受影响。摆动腿运动也属于 Task 1, 优先级高于基座运动。

(c) **不一定受影响**。奇异性仅在 $z$ 轴旋转方向, 基座的平移和其他旋转方向可能仍然正常。具体取决于 $\mathbf{J}_{2|1}$ 的秩亏结构。

(d) 如果使用单任务公式:
- $\mathbf{J}_{\text{full}}$ 整体奇异
- **所有任务都受影响**, 包括接触约束和摆动腿运动
- 可能导致足端滑动, 机器人失稳
- 这正是多任务公式优于单任务公式的关键原因

---

## 8. 足式机器人的动力学与控制

### 8.1 全身动力学 (Whole-Body Dynamics)

足式机器人的动力学方程基于浮动基座的 Euler-Lagrange 方程:

$$\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{S}^T \boldsymbol{\tau} + \sum_{i=1}^{n_c} \mathbf{J}_i^T \mathbf{f}_i$$

其中:
- $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{n_q \times n_q}$: 广义质量矩阵 (generalized mass matrix)
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} \in \mathbb{R}^{n_q}$: 科氏力和离心力项 (Coriolis and centrifugal terms)
- $\mathbf{g}(\mathbf{q}) \in \mathbb{R}^{n_q}$: 重力项 (gravity term)
- $\boldsymbol{\tau} \in \mathbb{R}^{n_j}$: 关节力矩 (joint torques)
- $\mathbf{f}_i \in \mathbb{R}^{3}$: 第 $i$ 个接触足的接触力 (contact force)
- $\mathbf{S} = \begin{pmatrix} \mathbf{0}_{n_j \times 6} & \mathbf{I}_{n_j \times n_j} \end{pmatrix}$: 选择矩阵 (selection matrix)

**选择矩阵的作用**: 基座是非驱动的 (unactuated), 关节力矩只作用在关节坐标上。$\mathbf{S}$ 将关节力矩映射到广义力空间。

### 8.2 基座动力学的特殊性

将动力学方程按基座和关节分块:

$$\begin{pmatrix} \mathbf{M}_{bb} & \mathbf{M}_{bj} \ \mathbf{M}_{jb} & \mathbf{M}_{jj} \end{pmatrix} \begin{pmatrix} \ddot{\mathbf{q}}_b \ \ddot{\mathbf{q}}_j \end{pmatrix} + \begin{pmatrix} \mathbf{h}_b \ \mathbf{h}_j \end{pmatrix} = \begin{pmatrix} \mathbf{0} \ \boldsymbol{\tau} \end{pmatrix} + \begin{pmatrix} \mathbf{J}_{c,b}^T \ \mathbf{J}_{c,j}^T \end{pmatrix} \mathbf{f}_c$$

其中 $\mathbf{h} = \mathbf{C} \dot{\mathbf{q}} + \mathbf{g}$ 是非线性项的简写。

**基座方程** (前 6 行):

$$\mathbf{M}_{bb} \ddot{\mathbf{q}}_b + \mathbf{M}_{bj} \ddot{\mathbf{q}}_j + \mathbf{h}_b = \mathbf{J}_{c,b}^T \mathbf{f}_c$$

注意右侧没有 $\boldsymbol{\tau}$ 项 -- 基座运动完全由接触力驱动。这体现了足式机器人的**欠驱动本质**: 我们无法直接控制基座, 只能通过接触力间接影响基座运动。

### 8.3 接触力模型 (Contact Force Model)

#### 单边约束 (Unilateral Constraint)

足式机器人与地面的接触是**单边的** -- 脚只能推地面, 不能拉地面:

$$f_{i,z} \geq 0$$

其中 $f_{i,z}$ 是第 $i$ 个接触足的法向接触力。

#### 摩擦锥约束 (Friction Cone Constraint)

根据 Coulomb 摩擦模型, 接触力必须在摩擦锥内:

$$\sqrt{f_{i,x}^2 + f_{i,y}^2} \leq \mu f_{i,z}$$

其中 $\mu$ 是摩擦系数。

**线性化摩擦锥 (Linearized Friction Cone)**: 实际计算中常将圆锥近似为多面体锥:

$$\mathbf{D}_i \mathbf{f}_i \leq \mathbf{0}$$

其中 $\mathbf{D}_i$ 定义了摩擦锥的线性近似面。常用 4 面或 8 面近似:

4 面近似:

$$|f_{i,x}| \leq \mu f_{i,z}, \quad |f_{i,y}| \leq \mu f_{i,z}$$

### 8.4 接触力分配 (Contact Force Distribution)

当支撑腿数量大于最小所需数量时 (如四足机器人四腿支撑), 接触力分配问题是**超静定的 (statically indeterminate)** -- 有无穷多组接触力满足动力学平衡。

**优化问题**: 选择最优的接触力分配:

$$\min_{\mathbf{f}_c} \quad \|\mathbf{f}_c\|^2_{\mathbf{W}}$$

$$\text{s.t.} \quad \mathbf{J}_{c,b}^T \mathbf{f}_c = \mathbf{h}_b - \mathbf{M}_{bb} \ddot{\mathbf{q}}_b^{\text{des}} - \mathbf{M}_{bj} \ddot{\mathbf{q}}_j^{\text{des}}$$

$$f_{i,z} \geq f_{\min} > 0, \quad \forall i$$

$$\sqrt{f_{i,x}^2 + f_{i,y}^2} \leq \mu f_{i,z}, \quad \forall i$$

其中:
- $\|\mathbf{f}_c\|^2_{\mathbf{W}} = \mathbf{f}_c^T \mathbf{W} \mathbf{f}_c$ 是加权范数, $\mathbf{W}$ 为正定权重矩阵
- $f_{\min}$ 是最小法向力, 确保接触稳定
- 第一个约束保证动力学平衡
- 后两个约束保证接触可行性

### 8.5 从接触力到关节力矩

一旦确定了期望的接触力 $\mathbf{f}_c^*$, 可以通过关节方程求解所需的关节力矩:

$$\boldsymbol{\tau} = \mathbf{M}_{jb} \ddot{\mathbf{q}}_b + \mathbf{M}_{jj} \ddot{\mathbf{q}}_j + \mathbf{h}_j - \mathbf{J}_{c,j}^T \mathbf{f}_c^*$$

这就是**逆动力学 (inverse dynamics)** 的计算过程。

### 8.6 步态规划基础 (Gait Planning Fundamentals)

步态 (gait) 定义了腿部接触状态随时间的变化模式。

#### 常见四足步态

| 步态名称 | 英文 | 支撑腿数 | 占空比 | 速度 | 稳定性 |
|---------|------|---------|--------|------|--------|
| 爬行步态 | Crawl | 3-4 | ~75% | 最慢 | 静态稳定 |
| 对角步态 | Trot | 2 | ~50% | 中等 | 动态稳定 |
| 溜蹄步态 | Pace | 2 | ~50% | 中等 | 动态稳定 |
| 跳跃步态 | Bound | 2->0->2 | ~30% | 快 | 动态稳定 |
| 飞奔步态 | Gallop | 变化 | ~25% | 最快 | 动态稳定 |

**占空比 (Duty Factor)**: 每条腿在一个步态周期中支撑阶段所占的时间比例:

$$\beta = \frac{T_{\text{stance}}}{T_{\text{cycle}}}$$

- $\beta > 0.5$: 慢速步态, 倾向静态稳定
- $\beta = 0.5$: 中速步态 (如 trot)
- $\beta < 0.5$: 快速步态, 存在腾空阶段 (flight phase)

#### 步态时序图 (Gait Timing Diagram)

以 Trot Gait 为例 (LF = 左前, RF = 右前, LH = 左后, RH = 右后):

```
时间 ->  |  Phase 1  |  Phase 2  |  Phase 1  |
LF:     ████████░░░░░░░░████████░░░░░░░░████████
RF:     ░░░░░░░░████████░░░░░░░░████████░░░░░░░░
LH:     ░░░░░░░░████████░░░░░░░░████████░░░░░░░░
RH:     ████████░░░░░░░░████████░░░░░░░░████████

████ = 支撑 (Stance)    ░░░░ = 摆动 (Swing)
```

对角腿对 (LF+RH, RF+LH) 交替支撑和摆动。

#### 足端轨迹规划 (Foot Trajectory Planning)

摆动腿的足端轨迹通常使用多项式或样条插值:

**三次样条 (Cubic Spline)** 足端高度:

$$z_{\text{foot}}(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3$$

边界条件:
- $z(0) = 0$ (起始: 地面)
- $z(T_{\text{sw}}) = 0$ (结束: 地面)
- $\dot{z}(0) = v_{\text{lift}}$ (抬腿速度)
- $\dot{z}(T_{\text{sw}}) = -v_{\text{land}}$ (落地速度)

最大抬腿高度 $h_{\text{step}}$ 通常设为 5-15 cm, 取决于地形粗糙度。

### 8.7 控制架构总览 (Control Architecture Overview)

典型的足式机器人控制架构分为多个层次:

```
┌─────────────────────────────────────────────┐
│         高层规划 (High-Level Planning)         │
│    路径规划, 立足点选择, 步态调度               │
│    频率: 1-10 Hz                              │
├─────────────────────────────────────────────┤
│       运动规划 (Motion Planning)               │
│    基座轨迹, 足端轨迹, 全身运动学              │
│    频率: 50-200 Hz                            │
├─────────────────────────────────────────────┤
│       全身控制 (Whole-Body Control)            │
│    逆动力学, 接触力分配, 任务优先级            │
│    频率: 200-1000 Hz                          │
├─────────────────────────────────────────────┤
│       底层控制 (Low-Level Control)             │
│    关节 PD 控制, 力矩控制, 电机驱动            │
│    频率: 1-10 kHz                             │
└─────────────────────────────────────────────┘
```

每一层的输出作为下一层的输入, 形成级联控制结构。高层规划频率低但处理复杂决策, 底层控制频率高但逻辑简单。

### Quiz 8.1: 动力学与接触力

**问题**: 一个四足机器人质量 $m = 25$ kg, 四腿支撑静止站立在水平地面上。摩擦系数 $\mu = 0.6$。

(a) 总法向接触力是多少?
(b) 如果接触力均匀分配, 每条腿的法向力是多少?
(c) 每条腿允许的最大水平力是多少?
(d) 如果机器人需要以 $a = 2$ m/s$^2$ 向前加速, 所需的总水平力是多少? 接触力分配是否可行?

**解答**:

(a) 静力平衡, 总法向力等于重力:

$$\sum_{i=1}^{4} f_{i,z} = mg = 25 \times 9.81 = 245.25 \text{ N}$$

(b) 均匀分配:

$$f_{i,z} = \frac{mg}{4} = \frac{245.25}{4} = 61.31 \text{ N}$$

(c) 摩擦锥约束下的最大水平力:

$$f_{i,\text{horiz}}^{\max} = \mu f_{i,z} = 0.6 \times 61.31 = 36.79 \text{ N}$$

(d) 所需总水平力:

$$F_x = ma = 25 \times 2 = 50 \text{ N}$$

总可用水平力:

$$F_x^{\max} = 4 \times 36.79 = 147.15 \text{ N}$$

由于 $50 < 147.15$, **接触力分配可行**。每条腿分担 $50/4 = 12.5$ N 的水平力, 远小于摩擦锥限制。

### Quiz 8.2: 步态参数计算

**问题**: 一个四足机器人以 trot gait 行走, 步态周期 $T_{\text{cycle}} = 0.6$ s, 占空比 $\beta = 0.5$, 步长 $L_{\text{step}} = 0.2$ m。

(a) 摆动阶段时长是多少?
(b) 机器人的前进速度是多少?
(c) 如果最大抬腿高度为 $h = 0.08$ m, 摆动腿足端的平均水平速度和平均垂直速度分别是多少?

**解答**:

(a) 摆动阶段时长:

$$T_{\text{sw}} = (1 - \beta) T_{\text{cycle}} = (1 - 0.5) \times 0.6 = 0.3 \text{ s}$$

(b) 前进速度:

$$v = \frac{L_{\text{step}}}{T_{\text{cycle}}} = \frac{0.2}{0.6} = 0.333 \text{ m/s}$$

(c) 足端平均速度:

水平方向 -- 摆动腿需要在 $T_{\text{sw}}$ 内完成步长 $L_{\text{step}}$ 的位移, 同时补偿基座在摆动期间的前进:

$$v_{\text{sw},x} = \frac{L_{\text{step}}}{T_{\text{sw}}} = \frac{0.2}{0.3} = 0.667 \text{ m/s}$$

垂直方向 -- 足端先升高 $h$ 再降低 $h$, 总垂直位移为 $2h$:

$$v_{\text{sw},z}^{\text{avg}} = \frac{2h}{T_{\text{sw}}} = \frac{2 \times 0.08}{0.3} = 0.533 \text{ m/s}$$

---

## 总结与关键公式速查

### 核心概念

| 概念 | 公式/定义 |
|------|----------|
| 广义坐标数 | $n_q = n_b + n_j$ (浮动基座: $n_b = 6$) |
| 接触约束数 | $n_c = k \times 3$ ($k$ 个点接触足) |
| 可用自由度 | $n_{\text{free}} = n_q - n_c$ |
| 内力方向数 | $n_{\text{internal}} = n_c - n_b$ (当 $n_c > n_b$) |
| 接触约束方程 | $\mathbf{J}_i \dot{\mathbf{q}} = \mathbf{0}$ |
| 浮动基座动力学 | $\mathbf{M} \ddot{\mathbf{q}} + \mathbf{h} = \mathbf{S}^T \boldsymbol{\tau} + \mathbf{J}_c^T \mathbf{f}_c$ |
| 摩擦锥约束 | $\sqrt{f_x^2 + f_y^2} \leq \mu f_z$ |
| 占空比 | $\beta = T_{\text{stance}} / T_{\text{cycle}}$ |

### 设计准则

1. **静态稳定行走**: 确保 CoM 投影始终在支撑多边形内, $s > 0$
2. **避免奇异性**: 保持支撑腿关节远离极限位置, 监控 $\sigma_{\min}(\mathbf{J})$
3. **接触力可行性**: 所有接触力必须满足单边约束和摩擦锥约束
4. **任务优先级**: 接触安全 > 足端轨迹 > 基座运动 > 次要任务
5. **控制频率**: 全身控制至少 200 Hz, 底层关节控制至少 1 kHz

---

> **参考文献**:
> - ETH Zurich, Robot Dynamics, Lecture Notes on Legged Robotics
> - M. Hutter et al., "ANYmal - A Highly Mobile and Dynamic Quadrupedal Robot," IEEE/RSJ IROS, 2016
> - S. Kajita et al., "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point," ICRA, 2003
> - O. Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators," IEEE J. Robotics and Automation, 1987
