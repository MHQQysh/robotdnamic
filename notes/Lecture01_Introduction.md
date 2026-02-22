# Lecture 01: 课程导论 (Introduction)

> **课程**: 151-0851-00 V — Robot Dynamics  
> **授课教师**: Prof. Marco Hutter, Prof. Roland Siegwart, Thomas Stastny  
> **学校**: ETH Zurich, 2019  
> **时间安排**: 每周二讲座 (Lecture) + 每周三练习 (Exercise)

---

## 1. 课程概述 (Course Overview)

### 1.1 基本信息

本课程编号为 **151-0851-00 V**，是苏黎世联邦理工学院 (ETH Zurich) 开设的机器人动力学核心课程。课程由三位教师联合授课：

- **Marco Hutter** — 主要负责运动学 (Kinematics)、动力学 (Dynamics) 以及足式机器人 (Legged Robots) 部分
- **Roland Siegwart** — 主要负责旋翼飞行器 (Rotorcraft) 相关内容
- **Thomas Stastny** — 主要负责固定翼飞行器 (Fixed-wing Aircraft) 相关内容

### 1.2 课程安排

| 日期 | 内容 |
|------|------|
| 每周二 | 讲座 (Lecture)：理论知识讲授 |
| 每周三 | 练习 (Exercise)：习题与编程实践 |

课程采用理论与实践相结合的方式，学生需要在练习课中运用 MATLAB 等工具完成编程作业，以加深对理论知识的理解。

---

## 2. 为什么学习机器人动力学？(Why Robot Dynamics?)

机器人动力学是连接机器人**设计**与**控制**的核心桥梁。理解动力学模型对于以下几个关键领域至关重要：

### 2.1 仿真 (Simulation)

> **核心问题**: 给定执行器命令 (actuator commands)，系统如何运动？

在仿真中，我们已知施加在系统上的力和力矩，需要预测系统的运动轨迹。这是一个**正向问题 (Forward Problem)**：

$$\tau \xrightarrow{\text{动力学模型}} \ddot{q} \xrightarrow{\text{积分}} \dot{q}, q$$

其中 $\tau$ 为执行器力矩，$q$ 为广义坐标 (generalized coordinates)，$\ddot{q}$ 为广义加速度。

仿真使我们能够在实际制造机器人之前，在虚拟环境中测试和验证控制算法，大幅降低开发成本和风险。

### 2.2 控制 (Control)

> **核心问题**: 要实现特定运动，需要什么执行器命令？

控制是仿真的**逆问题 (Inverse Problem)**——我们反转因果关系 (invert causality)：

$$q_{\text{desired}} \xrightarrow{\text{逆动力学}} \tau$$

基于模型的控制 (Model-based Control) 利用动力学方程计算前馈力矩 (feedforward torque)，使机器人精确跟踪期望轨迹。这比纯反馈控制 (feedback control) 具有更高的精度和响应速度。

### 2.3 设计 (Design)

> **核心问题**: 结构上的动态载荷 (dynamic loads) 是什么？

在机器人结构设计阶段，工程师需要知道各个连杆 (link) 和关节 (joint) 在运动过程中承受的力和力矩。动力学分析可以提供：

- 关节处的反力 (reaction forces)
- 连杆上的弯矩和扭矩
- 动态冲击载荷 (impact loads)

这些信息直接决定了材料选择、截面尺寸和安全系数。

### 2.4 优化 (Optimization)

> **核心问题**: 最优尺寸 (optimal dimensions) 是什么？

通过动力学模型，我们可以建立优化问题：

- 最小化能量消耗
- 最大化负载能力
- 优化连杆长度和质量分布
- 在满足性能约束的前提下最小化总重量

$$\min_{p} J(p) \quad \text{s.t.} \quad \text{动力学约束}$$

其中 $p$ 为设计参数，$J$ 为目标函数。

### 2.5 驱动选型 (Actuation)

> **核心问题**: 需要什么扭矩 (torque)、速度 (speed)、功率 (power)？

动力学分析帮助工程师确定每个关节所需的：

- **峰值扭矩** $\tau_{\max}$：决定电机的额定扭矩
- **最大角速度** $\dot{q}_{\max}$：决定电机的额定转速
- **峰值功率** $P_{\max} = \tau \cdot \dot{q}$：决定电机和驱动器的功率等级
- **连续功率**：决定散热需求

这些参数直接影响执行器 (actuator)、减速器 (gearbox) 和电源系统的选型。

---

## 3. 课程内容大纲 (Course Outline)

### 3.1 总体结构

课程内容按以下模块组织：

```
┌─────────────────────────────────────────────┐
│           Robot Dynamics 课程结构              │
├─────────────────────────────────────────────┤
│  Part I:   运动学 (Kinematics 1-3)           │
│  Part II:  动力学 (Dynamics 1-3)             │
│  Part III: 足式机器人 (Legged Robots)         │
│  Part IV:  旋翼飞行器 (Rotorcraft)            │
│  Part V:   固定翼飞行器 (Fixed-wing)          │
│  Part VI:  案例研究 (Case Studies)            │
└─────────────────────────────────────────────┘
```

### 3.2 运动学 (Kinematics) — 三讲

运动学研究的是**不考虑力的情况下**物体的运动几何关系。

#### Kinematics 1: 旋转与角速度 (Rotation & Angular Velocity)

- **旋转矩阵 (Rotation Matrix)** $C \in SO(3)$：描述刚体姿态的 $3 \times 3$ 正交矩阵
  
  $$C^T C = I, \quad \det(C) = +1$$

- **角速度 (Angular Velocity)** $\boldsymbol{\omega}$：描述刚体旋转的瞬时速度

  $$\dot{C} = \tilde{\boldsymbol{\omega}} \cdot C$$

  其中 $\tilde{\boldsymbol{\omega}}$ 是角速度的反对称矩阵 (skew-symmetric matrix)。

- **欧拉角 (Euler Angles)**：用三个角度参数化旋转，如 ZYX 欧拉角 $(\phi, \theta, \psi)$
- **旋转参数化方法比较**：欧拉角、旋转向量 (rotation vector)、四元数 (quaternion) 等

#### Kinematics 2: 刚体公式与变换 (Rigid Body Formulas & Transformations)

- **齐次变换矩阵 (Homogeneous Transformation Matrix)**：

  $$T = \begin{bmatrix} C & \mathbf{r} \ \mathbf{0}^T & 1 \end{bmatrix} \in SE(3)$$

  其中 $C$ 为旋转矩阵，$\mathbf{r}$ 为平移向量。

- **刚体速度公式**：

  $$\mathbf{v}_P = \mathbf{v}_O + \boldsymbol{\omega} \times \mathbf{r}_{OP}$$

  点 $P$ 的速度等于参考点 $O$ 的速度加上旋转引起的速度分量。

- **多体运动学 (Multi-body Kinematics)**：通过链式法则将多个变换矩阵相乘

  $$T_{0n} = T_{01} \cdot T_{12} \cdot \ldots \cdot T_{(n-1)n}$$

#### Kinematics 3: 雅可比矩阵与逆运动学 (Jacobian & Inverse Kinematics)

- **雅可比矩阵 (Jacobian Matrix)** $J$：建立关节空间速度与任务空间速度的线性映射

  $$\dot{\mathbf{x}} = J(q) \dot{q}$$

  其中 $\dot{\mathbf{x}}$ 为末端执行器 (end-effector) 在任务空间的速度，$\dot{q}$ 为关节速度。

- **逆运动学 (Inverse Kinematics, IK)**：给定末端执行器的期望位姿，求解关节角度

  $$q = f^{-1}(\mathbf{x}_{\text{desired}})$$

- **逆运动学控制 (IK Control)**：利用雅可比矩阵的伪逆 (pseudo-inverse) 进行迭代求解

  $$\dot{q} = J^{\dagger} \dot{\mathbf{x}}_{\text{desired}}$$

  其中 $J^{\dagger} = J^T(JJ^T)^{-1}$ 为雅可比矩阵的伪逆。

### 3.3 动力学 (Dynamics) — 三讲

动力学研究的是**力与运动之间的关系**。

#### Dynamics 1: 多体动力学 (Multi-body Dynamics)

- **牛顿-欧拉方程 (Newton-Euler Equations)**：

  $$\mathbf{F} = m \mathbf{a}_C, \quad \boldsymbol{\tau} = I_C \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times I_C \boldsymbol{\omega}$$

- **拉格朗日方程 (Lagrangian Mechanics)**：

  $$\frac{d}{dt} \frac{\partial L}{\partial \dot{q}_i} - \frac{\partial L}{\partial q_i} = \tau_i$$

  其中拉格朗日量 $L = T - V$（动能减势能）。

- **运动方程的标准形式 (Equations of Motion)**：

  $$M(q)\ddot{q} + b(q, \dot{q}) + g(q) = \tau$$

  - $M(q)$：质量矩阵 (mass/inertia matrix)，对称正定
  - $b(q, \dot{q})$：科里奥利力和离心力项 (Coriolis and centrifugal terms)
  - $g(q)$：重力项 (gravity terms)
  - $\tau$：广义力 (generalized forces)

#### Dynamics 2: 浮动基座动力学 (Floating Base Dynamics)

- 与固定基座 (fixed-base) 机器人不同，浮动基座机器人（如足式机器人、飞行器）的基座本身也是自由运动的
- 广义坐标扩展为：

  $$q = \begin{bmatrix} q_b \ q_j \end{bmatrix}$$

  其中 $q_b$ 为基座位姿（6自由度），$q_j$ 为关节角度。

- 浮动基座动力学方程：

  $$M(q)\ddot{q} + b(q, \dot{q}) + g(q) = S^T \tau + J_c^T F_c$$

  其中 $S$ 为选择矩阵 (selection matrix)，$J_c$ 为接触雅可比矩阵，$F_c$ 为接触力。

#### Dynamics 3: 基于模型的控制 (Model-based Control)

- **计算力矩控制 (Computed Torque Control)**：

  $$\tau = M(q)\ddot{q}_{\text{des}} + b(q, \dot{q}) + g(q)$$

- **逆动力学 (Inverse Dynamics)**：已知期望运动，计算所需力矩
- **操作空间控制 (Operational Space Control)**：在任务空间中直接进行力控制

### 3.4 足式机器人 (Legged Robots)

- 步态规划 (Gait Planning)
- 接触力优化 (Contact Force Optimization)
- 全身控制 (Whole-body Control)
- 稳定性分析：零力矩点 (Zero Moment Point, ZMP)、支撑多边形 (Support Polygon)

### 3.5 旋翼飞行器 (Rotorcraft)

- 多旋翼 (Multirotor) 动力学建模
- 推力与力矩分配 (Thrust and Torque Allocation)
- 姿态控制 (Attitude Control)
- 位置控制 (Position Control)

### 3.6 固定翼飞行器 (Fixed-wing Aircraft)

- 空气动力学基础 (Aerodynamics Basics)
- 升力、阻力、力矩 (Lift, Drag, Moment)
- 纵向与横向稳定性 (Longitudinal & Lateral Stability)
- 飞行控制 (Flight Control)

### 3.7 案例研究 (Case Studies)

结合实际机器人系统，将理论知识应用于工程实践。

---

## 4. 应用领域 (Application Domains)

机器人动力学的理论广泛应用于多种机器人平台。以下是课程中涉及的主要应用领域：

### 4.1 工业机器人 (Industrial Robots)

工业机器人是动力学理论最成熟的应用领域之一。典型代表包括 ABB、KUKA、FANUC 等公司的产品。

**特点**：
- 固定基座 (fixed-base)，通常为串联机构 (serial manipulator)
- 6 自由度 (6-DOF) 或更多（含冗余自由度）
- 高精度、高重复定位精度 (repeatability)
- 应用场景：焊接 (welding)、喷涂 (painting)、装配 (assembly)、搬运 (pick-and-place)

**动力学需求**：
- 精确的轨迹跟踪 (trajectory tracking) 需要基于模型的前馈控制
- 高速运动时，科里奥利力和离心力不可忽略
- 负载变化时需要自适应控制 (adaptive control)

### 4.2 四足机器人 (Quadruped Robots)

四足机器人是浮动基座动力学的典型应用，代表性平台包括：

- **ANYmal** (ETH Zurich / ANYbotics)：由 Marco Hutter 团队开发的液压/电驱动四足机器人
- **Boston Dynamics** 的 Spot 和早期的 BigDog

**特点**：
- 浮动基座 (floating base)：基座位姿由腿部接触力间接控制
- 混合动力学 (hybrid dynamics)：包含连续动力学和离散接触事件
- 高维系统：通常 12 个驱动关节 + 6 个基座自由度 = 18 维广义坐标

**关键挑战**：
- 步态生成与切换 (gait generation and transition)
- 地形适应 (terrain adaptation)
- 动态平衡 (dynamic balance)
- 接触力分配 (contact force distribution)

### 4.3 飞行器 (Aerial Vehicles)

#### 多旋翼无人机 (Multirotor UAVs)

- 四旋翼 (quadrotor) 是最常见的构型
- 通过调节各旋翼转速实现六自由度控制
- 欠驱动系统 (underactuated system)：4 个输入控制 6 个自由度

动力学模型：

$$m \ddot{\mathbf{r}} = m\mathbf{g} + C \begin{bmatrix} 0 \ 0 \ F_{\text{total}} \end{bmatrix}$$

$$I \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times I \boldsymbol{\omega} = \boldsymbol{\tau}_{\text{rotor}}$$

其中 $F_{\text{total}}$ 为总推力，$\boldsymbol{\tau}_{\text{rotor}}$ 为旋翼产生的合力矩。

#### 太阳能飞机 (Solar-powered Aircraft)

ETH Zurich 的 AtlantikSolar 项目展示了长航时太阳能无人机的可能性，需要精确的气动建模和能量管理。

#### 固定翼无人机 (Fixed-wing UAVs)

- 依靠空气动力学产生升力
- 比旋翼飞行器能效更高，适合长距离飞行
- 需要最低飞行速度 (stall speed)

### 4.4 移动操作平台 (Mobile Manipulation Platforms)

将移动平台与机械臂结合，实现移动操作能力：

- 轮式/履带式底盘 + 机械臂
- 足式机器人 + 机械臂（如 ANYmal 搭载机械臂）
- 飞行器 + 机械臂（空中操作）

**动力学特点**：
- 移动平台与机械臂之间存在动力学耦合 (dynamic coupling)
- 需要协调控制 (coordinated control) 移动平台和机械臂
- 整体系统自由度高，计算复杂度大

---

## 5. 核心方程 (Core Equations)

本节总结课程中最重要的三组方程，它们构成了机器人动力学的理论基础。

### 5.1 正运动学 (Forward Kinematics)

正运动学描述从**关节空间 (joint space)** 到**任务空间 (task space)** 的映射：

$$\mathbf{x} = f(q)$$

- **输入**: 关节位置 $q = [q_1, q_2, \ldots, q_n]^T$
- **输出**: 末端执行器位姿 $\mathbf{x} = [\mathbf{p}^T, \boldsymbol{\phi}^T]^T$（位置 + 姿态）

对于 $n$ 个关节的串联机器人，正运动学通过连续的齐次变换矩阵计算：

$$T_{0n}(q) = \prod_{i=1}^{n} T_{(i-1)i}(q_i) = T_{01}(q_1) \cdot T_{12}(q_2) \cdots T_{(n-1)n}(q_n)$$

每个变换矩阵 $T_{(i-1)i}$ 由 Denavit-Hartenberg (DH) 参数或其他约定确定。

### 5.2 微分运动学 (Differential Kinematics)

微分运动学通过**雅可比矩阵 (Jacobian Matrix)** 建立关节速度与任务空间速度的线性关系：

$$\dot{\mathbf{x}} = J(q) \, \dot{q}$$

其中雅可比矩阵 $J \in \mathbb{R}^{m \times n}$ 定义为：

$$J(q) = \frac{\partial f(q)}{\partial q} = \begin{bmatrix} \frac{\partial f_1}{\partial q_1} & \cdots & \frac{\partial f_1}{\partial q_n} \ \vdots & \ddots & \vdots \ \frac{\partial f_m}{\partial q_1} & \cdots & \frac{\partial f_m}{\partial q_n} \end{bmatrix}$$

**雅可比矩阵的重要性质**：

- 当 $\det(J) = 0$ 时，机器人处于**奇异位形 (singularity)**，某些方向上的运动能力丧失
- 雅可比矩阵的转置 $J^T$ 用于力映射：$\tau = J^T \mathbf{F}$（任务空间力映射到关节力矩）
- 可分解为几何雅可比 (Geometric Jacobian) 和解析雅可比 (Analytical Jacobian)

### 5.3 动力学方程 (Equations of Motion)

机器人系统的完整动力学方程为：

$$\boxed{M(q)\ddot{q} + b(q, \dot{q}) + g(q) = J^T F_{\text{ext}} + S\tau}$$

这是课程中最核心的方程，各项含义如下：

| 符号 | 名称 | 说明 |
|------|------|------|
| $M(q)$ | 质量/惯性矩阵 (Mass/Inertia Matrix) | $n \times n$ 对称正定矩阵，依赖于构型 $q$ |
| $\ddot{q}$ | 广义加速度 (Generalized Accelerations) | $n \times 1$ 向量 |
| $b(q, \dot{q})$ | 非线性项 (Nonlinear Terms) | 包含科里奥利力 (Coriolis) 和离心力 (Centrifugal) |
| $g(q)$ | 重力项 (Gravity Terms) | 由势能对广义坐标求导得到 |
| $J^T F_{\text{ext}}$ | 外力项 (External Forces) | 通过雅可比转置映射到关节空间 |
| $S$ | 选择矩阵 (Selection Matrix) | 将驱动力矩映射到广义坐标（浮动基座时非单位阵） |
| $\tau$ | 驱动力矩 (Actuator Torques) | 电机/液压缸提供的驱动力 |

**质量矩阵的重要性质**：

1. 对称性：$M(q) = M(q)^T$
2. 正定性：$\forall \dot{q} \neq 0, \; \dot{q}^T M(q) \dot{q} > 0$
3. $\dot{M} - 2C$ 是反对称矩阵（其中 $b = C\dot{q}$），这一性质在控制器稳定性证明中非常重要

**动力学方程的两种用途**：

- **正动力学 (Forward Dynamics)**：已知 $\tau$，求 $\ddot{q}$（用于仿真）

  $$\ddot{q} = M^{-1}(q) \left[ J^T F_{\text{ext}} + S\tau - b(q, \dot{q}) - g(q) \right]$$

- **逆动力学 (Inverse Dynamics)**：已知 $\ddot{q}$，求 $\tau$（用于控制）

  $$\tau = S^{-T} \left[ M(q)\ddot{q} + b(q, \dot{q}) + g(q) - J^T F_{\text{ext}} \right]$$

---

## 6. 考试形式 (Examination Format)

### 6.1 评估组成

课程的最终成绩由以下几部分组成：

| 评估项目 | 占比 | 形式 |
|----------|------|------|
| 多选题测验 (Multiple Choice Quizzes) | 15% | 课程期间进行 |
| MATLAB 编程中期考试 (MATLAB Mid-term) | 15% | 编程实践考试 |
| 期末考试 (Final Exam) | 70% | 笔试 |

### 6.2 多选题测验 (Multiple Choice Quizzes)

- 在课程进行过程中穿插进行
- 考察对基本概念的理解
- 涵盖运动学、动力学等核心理论知识点
- 需要对公式推导和物理意义有清晰的理解

### 6.3 MATLAB 编程中期考试

这是本课程的一大特色。学生需要使用 MATLAB 实现：

- 正运动学与逆运动学算法
- 雅可比矩阵的计算
- 动力学方程的数值求解
- 简单的控制器设计与仿真

**备考建议**：
- 熟练掌握 MATLAB 矩阵运算和符号计算工具箱 (Symbolic Math Toolbox)
- 认真完成每周三的编程练习
- 理解代码背后的物理和数学原理，而非死记硬背代码

### 6.4 期末考试

期末考试为笔试形式，占总成绩的 70%，是最重要的评估环节。考试内容覆盖课程全部模块：

- 运动学：旋转矩阵、齐次变换、雅可比矩阵的推导与计算
- 动力学：拉格朗日方程的建立、运动方程的推导
- 足式机器人：步态分析、接触力计算
- 飞行器：旋翼与固定翼的动力学建模

---

## 7. 学习路线图 (Learning Roadmap)

为了帮助理解课程各部分之间的逻辑关系，以下是推荐的学习路线：

```
基础数学工具
    │
    ├── 线性代数 (Linear Algebra): 矩阵运算、特征值、SVD
    ├── 微积分 (Calculus): 偏导数、链式法则
    └── 经典力学 (Classical Mechanics): 牛顿定律、能量方法
         │
         ▼
运动学 (Kinematics)
    │
    ├── Lecture K1: 旋转描述 → 角速度
    ├── Lecture K2: 齐次变换 → 多体运动学
    └── Lecture K3: 雅可比矩阵 → 逆运动学
         │
         ▼
动力学 (Dynamics)
    │
    ├── Lecture D1: 牛顿-欧拉 / 拉格朗日 → 运动方程
    ├── Lecture D2: 浮动基座 → 接触动力学
    └── Lecture D3: 基于模型的控制
         │
         ▼
应用 (Applications)
    │
    ├── 足式机器人 (Legged Robots)
    ├── 旋翼飞行器 (Rotorcraft)
    ├── 固定翼飞行器 (Fixed-wing)
    └── 案例研究 (Case Studies)
```

### 关键依赖关系

1. **运动学是动力学的基础**：动力学方程中的质量矩阵 $M(q)$、科里奥利项 $b(q,\dot{q})$ 和重力项 $g(q)$ 都依赖于运动学关系
2. **雅可比矩阵贯穿始终**：从微分运动学到力映射，再到动力学方程中的外力项 $J^T F_{\text{ext}}$
3. **动力学方程是应用的核心**：无论是足式机器人的全身控制，还是飞行器的姿态控制，都建立在动力学模型之上

---

## 8. 补充说明与参考 (Supplementary Notes)

### 8.1 符号约定 (Notation Conventions)

本课程中常用的符号约定如下：

| 符号 | 含义 |
|------|------|
| $q$ | 广义坐标向量 (generalized coordinates) |
| $\dot{q}$ | 广义速度向量 (generalized velocities) |
| $\ddot{q}$ | 广义加速度向量 (generalized accelerations) |
| $C$ 或 $R$ | 旋转矩阵 (rotation matrix) |
| $T$ | 齐次变换矩阵 (homogeneous transformation) |
| $J$ | 雅可比矩阵 (Jacobian matrix) |
| $M$ | 质量/惯性矩阵 (mass/inertia matrix) |
| $\boldsymbol{\omega}$ | 角速度向量 (angular velocity) |
| $\tau$ | 关节力矩向量 (joint torques) |
| $\mathbf{F}$ | 力向量 (force vector) |
| $I$ 或 $\Theta$ | 惯性张量 (inertia tensor) |
| $_{A}\mathbf{r}_{BP}$ | 从 $B$ 到 $P$ 的位置向量，在坐标系 $A$ 中表示 |
| $C_{AB}$ | 从坐标系 $B$ 到坐标系 $A$ 的旋转矩阵 |

### 8.2 推荐参考资料

- **Robotics: Modelling, Planning and Control** — Siciliano, Sciavicco, Villani, Oriolo
- **Introduction to Robotics: Mechanics and Control** — John J. Craig
- **A Mathematical Introduction to Robotic Manipulation** — Murray, Li, Sastry
- **Springer Handbook of Robotics** — Siciliano, Khatib (Eds.)
- **ETH 课程讲义与练习材料**

### 8.3 软件工具

- **MATLAB / Simulink**：课程主要编程工具，用于运动学和动力学计算
- **Robotics Toolbox (Peter Corke)**：MATLAB 机器人工具箱，提供常用函数
- **ROS (Robot Operating System)**：实际机器人系统中广泛使用的中间件
- **Gazebo / MuJoCo**：物理仿真引擎，用于验证动力学模型

---

## 9. 本讲小结 (Summary)

本讲作为课程导论，主要完成了以下内容：

1. **明确了学习动机**：机器人动力学在仿真、控制、设计、优化和驱动选型中不可或缺
2. **概览了课程结构**：从运动学基础到动力学建模，再到具体机器人平台的应用
3. **介绍了核心方程**：正运动学 $\mathbf{x} = f(q)$、微分运动学 $\dot{\mathbf{x}} = J\dot{q}$、动力学方程 $M\ddot{q} + b + g = J^T F + S\tau$
4. **展示了应用领域**：工业机器人、四足机器人、飞行器、移动操作平台

从下一讲开始，我们将深入运动学的第一个主题——**旋转与角速度 (Rotation & Angular Velocity)**，这是理解所有后续内容的数学基础。

---

> *笔记整理基于 ETH Zurich 151-0851-00 Robot Dynamics 课程 (2019)*  
> *授课教师: Marco Hutter, Roland Siegwart, Thomas Stastny*
