# Lecture 4: Kinematics 3 — 运动学控制方法 (Kinematic Control Methods)

> **课程**: ETH Zurich — Robot Dynamics  
> **主题**: 逆运动学求解、奇异性处理、冗余机器人控制、多任务优先级与轨迹跟踪  
> **关键词**: Inverse Kinematics, Jacobian Pseudo-inverse, Singularity, Redundancy, Null-space Projection

---

## 目录

1. [正运动学与逆运动学](#1-正运动学与逆运动学)
2. [封闭解](#2-封闭解-closed-form-solutions)
3. [逆微分运动学](#3-逆微分运动学-inverse-differential-kinematics)
4. [奇异性](#4-奇异性-singularities)
5. [冗余性](#5-冗余性-redundancy)
6. [多任务控制](#6-多任务控制-multi-task-control)
7. [迭代逆运动学](#7-迭代逆运动学-iterative-inverse-kinematics)
8. [姿态误差](#8-姿态误差-orientation-error)
9. [轨迹控制](#9-轨迹控制-trajectory-control)
10. [Quiz 练习题](#10-quiz-练习题)

---

## 1. 正运动学与逆运动学

### 1.1 问题定义

机器人运动学的核心问题可以分为两类：

| | 正运动学 (Forward Kinematics) | 逆运动学 (Inverse Kinematics) |
|---|---|---|
| **输入** | 关节变量 $q$ | 末端执行器位姿 $x_e$ |
| **输出** | 末端执行器位姿 $x_e$ | 关节变量 $q$ |
| **映射** | $x_e = f(q)$ | $q = f^{-1}(x_e)$ |
| **唯一性** | 总是唯一的 | 可能有多解、无解或无穷多解 |
| **难度** | 相对简单（矩阵连乘） | 通常困难（非线性方程组） |

### 1.2 正运动学回顾

给定关节变量 $q = [q_1, q_2, \dots, q_n]^T$，通过齐次变换矩阵的连乘得到末端执行器的位姿：

$$T_{0n}(q) = T_{01}(q_1) \cdot T_{12}(q_2) \cdots T_{(n-1)n}(q_n) = \begin{bmatrix} C_{0n}(q) & _0 r_{0n}(q) \ 0 & 1 \end{bmatrix}$$

其中 $C_{0n} \in SO(3)$ 为旋转矩阵，${}_0 r_{0n} \in \mathbb{R}^3$ 为位置向量。

### 1.3 逆运动学的挑战

逆运动学问题的本质是求解非线性方程组：

$$x_e = f(q) \quad \Rightarrow \quad q = f^{-1}(x_e)$$

**主要困难**：
- **非线性**: $f(q)$ 包含三角函数的复合
- **多解性**: 同一末端位姿可能对应多组关节角（如"肘上"与"肘下"构型）
- **无解**: 目标位姿可能超出工作空间
- **奇异性**: 在某些构型下自由度退化

> **注意**: 逆运动学的求解方法主要分为三类——封闭解（解析法）、数值迭代法、微分运动学法。

---

## 2. 封闭解 (Closed-form Solutions)

封闭解是指能够用显式数学公式直接计算关节变量的方法，无需迭代。

### 2.1 几何方法 (Geometric Approach)

**核心思想**: 将三维空间的几何关系分解为若干平面几何问题，利用三角关系直接求解。

**典型步骤**：
1. 利用机器人结构的对称性，将问题投影到特定平面
2. 在平面内应用余弦定理、正弦定理等
3. 逐个求解关节角

**示例 — 2-DOF 平面机器人臂**：

对于连杆长度为 $l_1, l_2$ 的平面二连杆机器人，末端位置 $(x, y)$：

$$\cos q_2 = \frac{x^2 + y^2 - l_1^2 - l_2^2}{2 l_1 l_2}$$

$$q_2 = \text{atan2}\!\left(\pm\sqrt{1 - \cos^2 q_2},\; \cos q_2\right)$$

$$q_1 = \text{atan2}(y, x) - \text{atan2}(l_2 \sin q_2,\; l_1 + l_2 \cos q_2)$$

> $\pm$ 号对应"肘上"(elbow-up) 和"肘下"(elbow-down) 两种构型。

### 2.2 代数方法 (Algebraic Approach)

**核心思想**: 直接操作齐次变换矩阵方程，通过左乘逆矩阵逐步隔离未知量。

给定目标位姿 $T_{0n}^{\text{desired}}$，由正运动学有：

$$T_{0n}^{\text{desired}} = T_{01}(q_1) \cdot T_{12}(q_2) \cdots T_{(n-1)n}(q_n)$$

**求解策略**: 左乘 $T_{01}^{-1}(q_1)$：

$$T_{01}^{-1}(q_1) \cdot T_{0n}^{\text{desired}} = T_{12}(q_2) \cdots T_{(n-1)n}(q_n)$$

左侧仅含 $q_1$，右侧不含 $q_1$。比较两侧矩阵元素，可以建立关于 $q_1$ 的方程。依次类推，逐步求解所有关节变量。

### 2.3 Pieper 准则：三轴相交条件

**定理 (Pieper, 1968)**: 如果一个 6-DOF 机器人的**三个相邻关节轴相交于一点**（或平行），则逆运动学存在封闭解。

**工业意义**: 大多数工业机器人（如 PUMA, KUKA, ABB 等）的后三个关节构成球腕 (spherical wrist)，满足此条件。

**求解分解**：
- **位置逆解**: 利用前三个关节求解腕心 (wrist center) 位置
- **姿态逆解**: 利用后三个关节（球腕）求解末端姿态

$$\underbrace{q_1, q_2, q_3}_{\text{位置}} \quad \underbrace{q_4, q_5, q_6}_{\text{姿态}}$$

---

## 3. 逆微分运动学 (Inverse Differential Kinematics)

### 3.1 从速度层面求解

当封闭解不存在时，可以在**速度层面**求解逆运动学。

微分运动学关系：

$$\dot{x}_e = J(q) \cdot \dot{q}$$

其中 $J(q) \in \mathbb{R}^{m \times n}$ 为雅可比矩阵，$m$ 为任务空间维度，$n$ 为关节空间维度。

**逆微分运动学**即求解：

$$\dot{q} = J^{-1}(q) \cdot \dot{x}_e$$

### 3.2 方阵情况 ($m = n$)

当 $J$ 为方阵且满秩时，直接求逆：

$$\dot{q} = J^{-1}(q) \cdot \dot{x}_e$$

但在奇异构型下 $J$ 不可逆，此方法失效。

### 3.3 伪逆方法 (Pseudo-inverse)

当 $J$ 不是方阵或不满秩时，使用 **Moore-Penrose 伪逆**：

$$J^{+} = J^T (J J^T)^{-1} \quad \text{(右伪逆, 当 } m < n \text{)}$$

$$J^{+} = (J^T J)^{-1} J^T \quad \text{(左伪逆, 当 } m > n \text{)}$$

逆微分运动学解：

$$\dot{q} = J^{+}(q) \cdot \dot{x}_e$$

**伪逆的最优性质**: 在所有满足 $\dot{x}_e = J \dot{q}$ 的解中，$\dot{q} = J^{+} \dot{x}_e$ 给出**最小范数解**：

$$\dot{q}^* = \arg\min_{\dot{q}} \|\dot{q}\|^2 \quad \text{s.t.} \quad J\dot{q} = \dot{x}_e$$

---

## 4. 奇异性 (Singularities)

### 4.1 定义

当雅可比矩阵 $J(q)$ **列秩亏缺** (rank-deficient) 时，机器人处于奇异构型：

$$\text{rank}(J(q)) < m$$

其中 $m = \min(\text{行数}, \text{列数})$。

**物理含义**: 在奇异构型下，末端执行器在某些方向上丧失了运动能力——无论关节速度多大，都无法在该方向上产生末端速度。

### 4.2 奇异性的分类

| 类型 | 描述 | 示例 |
|---|---|---|
| **边界奇异性** (Boundary Singularity) | 末端执行器位于工作空间边界 | 机械臂完全伸展或完全折叠 |
| **内部奇异性** (Internal Singularity) | 末端执行器位于工作空间内部 | 两个关节轴对齐（如球腕的万向锁） |

**边界奇异性示例**: 2-DOF 平面臂当 $q_2 = 0$ 或 $q_2 = \pi$ 时：

$$J = \begin{bmatrix} -(l_1 s_1 + l_2 s_{12}) & -l_2 s_{12} \ l_1 c_1 + l_2 c_{12} & l_2 c_{12} \end{bmatrix}$$

当 $q_2 = 0$ 时，$\det(J) = l_1 l_2 \sin(q_2) = 0$，臂完全伸展，无法沿径向运动。

**内部奇异性示例**: 球腕中当 $q_5 = 0$ 时，$q_4$ 和 $q_6$ 的旋转轴重合，产生万向锁 (gimbal lock)。

### 4.3 奇异性的后果

在奇异点附近：
- $J^{-1}$ 或 $J^{+}$ 的元素趋向无穷大
- 关节速度 $\dot{q} = J^{+} \dot{x}_e$ 变得极大
- 实际机器人会出现剧烈抖动或失控

### 4.4 阻尼伪逆 (Damped Least Squares / Levenberg-Marquardt)

为了在奇异点附近保持数值稳定性，引入**阻尼伪逆**：

$$J^{*} = J^T (J J^T + \lambda^2 I)^{-1}$$

其中 $\lambda > 0$ 为阻尼因子 (damping factor)。

**等价优化问题**：

$$\dot{q}^* = \arg\min_{\dot{q}} \left( \|J\dot{q} - \dot{x}_e\|^2 + \lambda^2 \|\dot{q}\|^2 \right)$$

这是一个在**跟踪精度**和**关节速度大小**之间的权衡：

- $\lambda \to 0$: 退化为标准伪逆，精确跟踪但可能速度爆炸
- $\lambda$ 较大: 关节速度受限但跟踪精度下降

**自适应阻尼**: 实践中常根据 $J$ 的最小奇异值 $\sigma_{\min}$ 动态调整 $\lambda$：

$$\lambda^2 = \begin{cases} 0 & \text{if } \sigma_{\min} \geq \epsilon \ \left(1 - \left(\frac{\sigma_{\min}}{\epsilon}\right)^2\right) \lambda_{\max}^2 & \text{if } \sigma_{\min} < \epsilon \end{cases}$$

---

## 5. 冗余性 (Redundancy)

### 5.1 定义

当关节空间维度大于任务空间维度时，机器人具有**运动学冗余性**：

$$n > m \quad \Rightarrow \quad \text{冗余度} = n - m$$

例如：7-DOF 机械臂在 6 维任务空间中具有 1 个冗余自由度。

### 5.2 通解结构

冗余机器人的逆微分运动学通解为：

$$\boxed{\dot{q} = J^{+} \dot{x}_e + (I - J^{+} J) \dot{q}_{\text{arb}}}$$

其中：
- $J^{+} \dot{x}_e$: **特解** — 满足任务要求的最小范数解
- $(I - J^{+} J) \dot{q}_{\text{arb}}$: **齐次解** — 零空间中的任意运动

### 5.3 零空间投影矩阵

$$N = I - J^{+} J$$

**性质**：
- $N$ 是投影矩阵: $N^2 = N$，$N^T = N$
- $J \cdot N = 0$: 零空间运动不影响末端执行器
- $N \dot{q}_{\text{arb}}$ 将任意关节速度投影到雅可比矩阵的零空间

### 5.4 零空间的利用

冗余自由度可以用于优化次要目标，同时不影响主任务：

$$\dot{q} = J^{+} \dot{x}_e + N \dot{q}_0$$

其中 $\dot{q}_0$ 可以选择为某个目标函数的梯度：

| 次要目标 | $\dot{q}_0$ 的选择 |
|---|---|
| 避免关节极限 | $\dot{q}_0 = k \nabla_q w(q)$，$w(q) = \frac{1}{2n}\sum\frac{(q_i^{\max}-q_i^{\min})^2}{(q_i^{\max}-q_i)(q_i-q_i^{\min})}$ |
| 避免奇异性 | $\dot{q}_0 = k \nabla_q \sqrt{\det(JJ^T)}$ |
| 最小化关节力矩 | $\dot{q}_0 = -k \nabla_q \|\tau\|^2$ |
| 避免障碍物 | $\dot{q}_0 = k \nabla_q d_{\min}(q)$ |

---

## 6. 多任务控制 (Multi-task Control)

### 6.1 问题背景

在冗余机器人中，我们通常希望同时完成多个任务。例如：
- **主任务**: 末端执行器跟踪目标轨迹
- **次任务**: 避免关节极限、避免障碍物、保持操作灵巧性

如何合理分配冗余自由度来同时满足多个任务？

### 6.2 等优先级方法 (Equal Priority)

#### 6.2.1 任务堆叠 (Task Augmentation / Stacking)

将多个任务的雅可比矩阵和期望速度堆叠为一个增广系统：

$$\begin{bmatrix} \dot{x}_1 \ \dot{x}_2 \ \vdots \end{bmatrix} = \begin{bmatrix} J_1 \ J_2 \ \vdots \end{bmatrix} \dot{q} \quad \Rightarrow \quad \dot{x}_{\text{aug}} = J_{\text{aug}} \dot{q}$$

然后用伪逆求解：

$$\dot{q} = J_{\text{aug}}^{+} \dot{x}_{\text{aug}}$$

**问题**: 当任务之间存在冲突时，所有任务的精度都会下降，无法保证任何一个任务被精确执行。

#### 6.2.2 加权伪逆 (Weighted Pseudo-inverse)

引入权重矩阵 $W$ 来调整不同关节的优先级：

$$J_W^{+} = W^{-1} J^T (J W^{-1} J^T)^{-1}$$

对应的优化问题：

$$\dot{q}^* = \arg\min_{\dot{q}} \dot{q}^T W \dot{q} \quad \text{s.t.} \quad J\dot{q} = \dot{x}_e$$

权重矩阵 $W$ 越大的关节，其运动越受限制。

### 6.3 优先级排序方法 (Task Priority / Null-space Projection)

**核心思想**: 为任务分配严格的优先级，高优先级任务被精确执行，低优先级任务仅在不影响高优先级任务的前提下执行。

#### 6.3.1 两个任务的情况

给定主任务 $(J_1, \dot{x}_1)$ 和次任务 $(J_2, \dot{x}_2)$：

$$\boxed{\dot{q}^* = J_1^{+} \dot{x}_1 + N_1 (J_2^{+} \dot{x}_2)}$$

其中 $N_1 = I - J_1^{+} J_1$ 是主任务雅可比矩阵的零空间投影矩阵。

**解读**：
- 第一项 $J_1^{+} \dot{x}_1$: 精确执行主任务
- 第二项 $N_1 (J_2^{+} \dot{x}_2)$: 将次任务的解投影到主任务的零空间中，确保不干扰主任务

#### 6.3.2 多个任务的递归公式

对于 $k$ 个按优先级排列的任务：

$$\dot{q}^* = J_1^{+} \dot{x}_1 + N_1 \Big( J_2^{+} \dot{x}_2 + N_2 \big( J_3^{+} \dot{x}_3 + N_3 (\cdots) \big) \Big)$$

更严格的递归形式（考虑前面所有任务的累积零空间）：

$$\dot{q}_k = \dot{q}_{k-1} + (J_k N_{k-1}^{\text{aug}})^{+} (\dot{x}_k - J_k \dot{q}_{k-1})$$

其中 $N_{k-1}^{\text{aug}} = I - J_{\text{aug},k-1}^{+} J_{\text{aug},k-1}$ 是前 $k-1$ 个任务的累积零空间投影。

### 6.4 示例：3-DOF 平面机器人臂

考虑一个 3-DOF 平面机器人臂（3个旋转关节，连杆长度 $l_1 = l_2 = l_3 = 1$）：

**任务空间**: 末端位置 $(x, y)$，维度 $m = 2$

**关节空间**: $q = [q_1, q_2, q_3]^T$，维度 $n = 3$

**冗余度**: $n - m = 1$

**主任务**: 末端执行器到达目标位置 $(x_d, y_d)$

$$J_1 = \begin{bmatrix} -l_1 s_1 - l_2 s_{12} - l_3 s_{123} & -l_2 s_{12} - l_3 s_{123} & -l_3 s_{123} \ l_1 c_1 + l_2 c_{12} + l_3 c_{123} & l_2 c_{12} + l_3 c_{123} & l_3 c_{123} \end{bmatrix}$$

**次任务**: 保持末端执行器的朝向角 $\phi = q_1 + q_2 + q_3$ 为期望值

$$J_2 = \begin{bmatrix} 1 & 1 & 1 \end{bmatrix}$$

**多任务解**：

$$\dot{q} = J_1^{+} \dot{x}_1 + N_1 (J_2^{+} \dot{\phi}_d)$$

其中 $s_1 = \sin q_1$，$c_1 = \cos q_1$，$s_{12} = \sin(q_1+q_2)$，$c_{12} = \cos(q_1+q_2)$，$s_{123} = \sin(q_1+q_2+q_3)$，$c_{123} = \cos(q_1+q_2+q_3)$。

---

## 7. 迭代逆运动学 (Iterative Inverse Kinematics)

### 7.1 基本思想

当无法获得封闭解时，可以通过迭代方法逐步逼近目标位姿。

**基本算法**：

1. 给定初始关节角 $q^{(0)}$ 和目标位姿 $x_d$
2. 计算当前位姿误差: $\Delta x = x_d - f(q^{(k)})$
3. 计算关节角增量: $\Delta q = J^{+}(q^{(k)}) \cdot \Delta x$
4. 更新关节角: $q^{(k+1)} = q^{(k)} + \Delta q$
5. 重复步骤 2-4 直到 $\|\Delta x\| < \epsilon$

### 7.2 伪代码

```
输入: 目标位姿 x_d, 初始关节角 q, 容差 ε, 最大迭代次数 N_max
输出: 关节角 q

for k = 1 to N_max:
    Δx = x_d - f(q)
    if ‖Δx‖ < ε:
        return q  // 收敛
    Δq = J⁺(q) · Δx
    q = q + Δq
return q  // 未收敛，返回当前最优解
```

### 7.3 缩放因子处理大误差

当位姿误差 $\|\Delta x\|$ 较大时，直接使用 $\Delta q = J^{+} \Delta x$ 可能导致：
- 关节角增量过大，超出线性化有效范围
- 算法发散或振荡

**解决方案**: 引入缩放因子 $\alpha \in (0, 1]$：

$$\Delta q = \alpha \cdot J^{+}(q) \cdot \Delta x$$

或者对误差进行截断 (clamping)：

$$\Delta x_{\text{clamped}} = \begin{cases} \Delta x & \text{if } \|\Delta x\| \leq \delta_{\max} \ \delta_{\max} \frac{\Delta x}{\|\Delta x\|} & \text{if } \|\Delta x\| > \delta_{\max} \end{cases}$$

### 7.4 示例：三连杆臂的多解性

考虑平面三连杆臂（$l_1 = l_2 = l_3 = 1$），目标末端位置 $(x_d, y_d) = (1.5, 1.0)$。

从不同初始值出发，迭代逆运动学会收敛到不同的解：

| 初始值 $q^{(0)}$ | 收敛解 | 构型描述 |
|---|---|---|
| $[0, 0, 0]^T$ | $[0.42, 0.73, -0.28]^T$ | 臂向右伸展 |
| $[\pi/2, 0, 0]^T$ | $[1.15, -0.63, 0.35]^T$ | 臂向上弯曲 |
| $[\pi, 0, 0]^T$ | $[2.31, -1.52, 0.88]^T$ | 臂向左绕回 |

> **关键结论**: 迭代逆运动学是**局部方法**，解依赖于初始值。不同初始值可能收敛到不同的解，甚至可能不收敛。

---

## 8. 姿态误差 (Orientation Error)

### 8.1 问题引入

在轨迹跟踪中，位置误差的计算很直观：

$$e_p = x_d - x_{\text{current}} \in \mathbb{R}^3$$

但**姿态误差**的计算并不简单，因为旋转矩阵属于特殊正交群 $SO(3)$，它不是一个线性空间——两个旋转矩阵的差 $C_d - C_{\text{current}}$ 通常不再是旋转矩阵。

### 8.2 SO(3) 中的旋转

旋转矩阵 $C \in SO(3)$ 满足：

$$C^T C = I, \quad \det(C) = +1$$

$SO(3)$ 是一个**李群** (Lie group)，其对应的**李代数** $\mathfrak{so}(3)$ 由反对称矩阵构成。

两个姿态之间的"差"应该用**相对旋转**来表示：

$$C_{\text{rel}} = C_d \cdot C_{\text{current}}^T$$

这表示从当前姿态到目标姿态所需的旋转。

### 8.3 旋转向量参数化 (Rotation Vector / Angle-Axis)

任何旋转都可以用**旋转向量** $\phi = \theta \hat{n}$ 表示，其中：
- $\theta \in [0, \pi]$ 为旋转角度
- $\hat{n} \in \mathbb{R}^3$，$\|\hat{n}\| = 1$ 为旋转轴

旋转向量与旋转矩阵的关系（Rodrigues 公式）：

$$C = I + \sin\theta \; [\hat{n}]_\times + (1 - \cos\theta) \; [\hat{n}]_\times^2$$

其中 $[\hat{n}]_\times$ 是 $\hat{n}$ 的反对称矩阵：

$$[\hat{n}]_\times = \begin{bmatrix} 0 & -n_3 & n_2 \ n_3 & 0 & -n_1 \ -n_2 & n_1 & 0 \end{bmatrix}$$

**旋转向量的优势**: 它参数化了从单位元到目标旋转的**最短路径**（测地线），因此非常适合定义姿态误差。

### 8.4 从相对旋转中提取旋转向量

给定相对旋转矩阵 $C_{\text{rel}}$，提取旋转角度和旋转轴：

**旋转角度**：

$$\theta = \arccos\!\left(\frac{\text{tr}(C_{\text{rel}}) - 1}{2}\right)$$

**旋转轴**（当 $\theta \neq 0$ 且 $\theta \neq \pi$ 时）：

$$\hat{n} = \frac{1}{2\sin\theta} \begin{bmatrix} C_{32} - C_{23} \ C_{13} - C_{31} \ C_{21} - C_{12} \end{bmatrix}$$

**姿态误差向量**：

$$\boxed{e_o = \theta \hat{n} = \frac{\theta}{2\sin\theta} \begin{bmatrix} C_{32} - C_{23} \ C_{13} - C_{31} \ C_{21} - C_{12} \end{bmatrix}}$$

> **注意**: 当 $\theta \to 0$ 时，$\frac{\theta}{2\sin\theta} \to \frac{1}{2}$，数值上是稳定的。当 $\theta \to \pi$ 时需要特殊处理。

### 8.5 完整的位姿误差

将位置误差和姿态误差组合为 6 维位姿误差向量：

$$e = \begin{bmatrix} e_p \ e_o \end{bmatrix} = \begin{bmatrix} x_d - x_{\text{current}} \ \theta \hat{n} \end{bmatrix} \in \mathbb{R}^6$$

---

## 9. 轨迹控制 (Trajectory Control)

### 9.1 位置轨迹控制

给定期望位置轨迹 $x_d(t)$ 及其速度 $\dot{x}_d(t)$，设计控制律使末端执行器跟踪该轨迹。

**控制律 — 反馈 + 前馈**：

$$\dot{q} = J^{+} \left( \dot{x}_d + K_p (x_d - f(q)) \right)$$

其中：
- $\dot{x}_d$: **前馈项** (feedforward) — 提供期望速度，减小跟踪滞后
- $K_p (x_d - f(q))$: **反馈项** (feedback) — 比例控制，修正位置误差
- $K_p = \text{diag}(k_{p1}, k_{p2}, \dots) > 0$: 比例增益矩阵

**误差动力学分析**: 定义位置误差 $e = x_d - f(q)$，则：

$$\dot{e} = \dot{x}_d - J\dot{q} = \dot{x}_d - J \cdot J^{+}(\dot{x}_d + K_p e)$$

当 $J$ 满秩时 $J J^{+} = I$，得到：

$$\dot{e} = -K_p e$$

这是一个指数收敛的线性系统，误差以指数速率衰减：

$$e(t) = e(0) \cdot e^{-K_p t}$$

### 9.2 姿态轨迹控制

对于姿态跟踪，控制律类似，但需要使用姿态误差向量：

$$\dot{q} = J^{+} \begin{bmatrix} \dot{x}_{d,\text{pos}} + K_p \cdot e_p \ \omega_d + K_o \cdot e_o \end{bmatrix}$$

其中：
- $\dot{x}_{d,\text{pos}}$: 期望线速度
- $\omega_d$: 期望角速度
- $e_p = x_d - x_{\text{current}}$: 位置误差
- $e_o = \theta \hat{n}$: 姿态误差（旋转向量）
- $K_p, K_o > 0$: 位置和姿态的比例增益

### 9.3 完整的冗余机器人轨迹控制

结合冗余性利用，完整的控制律为：

$$\boxed{\dot{q} = J^{+} \left( \begin{bmatrix} \dot{x}_{d} \ \omega_d \end{bmatrix} + K \cdot e \right) + N \dot{q}_0}$$

其中：
- $K = \text{diag}(K_p, K_o)$: 增益矩阵
- $e = [e_p^T, e_o^T]^T$: 位姿误差
- $N = I - J^{+}J$: 零空间投影
- $\dot{q}_0$: 次要目标的关节速度

### 9.4 控制框图

```
                    ┌─────────────┐
  x_d, ẋ_d ──────>│             │
                    │  逆微分      │──── q̇_cmd ──── 积分 ──── q_cmd
  q (反馈) ──────>│  运动学控制器  │
                    │             │
  q̇_0 (次要) ───>│             │
                    └─────────────┘
                          │
                    ┌─────┴─────┐
                    │           │
              前馈 ẋ_d    反馈 K·e
              (减小滞后)   (修正误差)
```

### 9.5 增益选择指南

| 参数 | 作用 | 选择建议 |
|---|---|---|
| $K_p$ | 位置误差收敛速率 | 典型值 1-10，过大导致振荡 |
| $K_o$ | 姿态误差收敛速率 | 通常与 $K_p$ 同量级 |
| $\lambda$ (阻尼) | 奇异性附近的稳定性 | 根据最小奇异值自适应调整 |
| $\alpha$ (缩放) | 大误差时的步长控制 | 典型值 0.1-0.5 |

---

## 10. Quiz 练习题

### Quiz 1: 逆运动学基础

**题目**: 一个 2-DOF 平面机器人臂，连杆长度 $l_1 = 1.0\,\text{m}$，$l_2 = 0.8\,\text{m}$。求末端到达点 $(x, y) = (1.2, 0.6)$ 时的关节角 $q_1, q_2$（肘上构型）。

**解答**：

Step 1: 计算 $\cos q_2$

$$\cos q_2 = \frac{x^2 + y^2 - l_1^2 - l_2^2}{2 l_1 l_2} = \frac{1.44 + 0.36 - 1.0 - 0.64}{2 \times 1.0 \times 0.8} = \frac{0.16}{1.6} = 0.1$$

Step 2: 计算 $q_2$（肘上取负值）

$$q_2 = \text{atan2}\!\left(-\sqrt{1 - 0.01},\; 0.1\right) = \text{atan2}(-0.995, 0.1) \approx -1.47\,\text{rad} \approx -84.3°$$

Step 3: 计算 $q_1$

$$q_1 = \text{atan2}(0.6, 1.2) - \text{atan2}(l_2 \sin q_2,\; l_1 + l_2 \cos q_2)$$

$$= \text{atan2}(0.6, 1.2) - \text{atan2}(-0.796, 1.08) \approx 0.4636 - (-0.635) \approx 1.10\,\text{rad} \approx 63.0°$$

---

### Quiz 2: 伪逆与零空间

**题目**: 给定雅可比矩阵

$$J = \begin{bmatrix} 1 & 0 & 1 \ 0 & 1 & 1 \end{bmatrix}$$

(a) 计算右伪逆 $J^{+} = J^T(JJ^T)^{-1}$

(b) 计算零空间投影矩阵 $N = I - J^{+}J$

(c) 验证 $J \cdot N = 0$

**解答**：

(a) 计算 $JJ^T$：

$$JJ^T = \begin{bmatrix} 1 & 0 & 1 \ 0 & 1 & 1 \end{bmatrix} \begin{bmatrix} 1 & 0 \ 0 & 1 \ 1 & 1 \end{bmatrix} = \begin{bmatrix} 2 & 1 \ 1 & 2 \end{bmatrix}$$

$$(JJ^T)^{-1} = \frac{1}{3}\begin{bmatrix} 2 & -1 \ -1 & 2 \end{bmatrix}$$

$$J^{+} = J^T(JJ^T)^{-1} = \begin{bmatrix} 1 & 0 \ 0 & 1 \ 1 & 1 \end{bmatrix} \cdot \frac{1}{3}\begin{bmatrix} 2 & -1 \ -1 & 2 \end{bmatrix} = \frac{1}{3}\begin{bmatrix} 2 & -1 \ -1 & 2 \ 1 & 1 \end{bmatrix}$$

(b) 计算 $J^{+}J$：

$$J^{+}J = \frac{1}{3}\begin{bmatrix} 2 & -1 \ -1 & 2 \ 1 & 1 \end{bmatrix}\begin{bmatrix} 1 & 0 & 1 \ 0 & 1 & 1 \end{bmatrix} = \frac{1}{3}\begin{bmatrix} 2 & -1 & 1 \ -1 & 2 & 1 \ 1 & 1 & 2 \end{bmatrix}$$

$$N = I - J^{+}J = \frac{1}{3}\begin{bmatrix} 1 & 1 & -1 \ 1 & 1 & -1 \ -1 & -1 & 1 \end{bmatrix}$$

(c) 验证：

$$JN = \begin{bmatrix} 1 & 0 & 1 \ 0 & 1 & 1 \end{bmatrix} \cdot \frac{1}{3}\begin{bmatrix} 1 & 1 & -1 \ 1 & 1 & -1 \ -1 & -1 & 1 \end{bmatrix} = \frac{1}{3}\begin{bmatrix} 0 & 0 & 0 \ 0 & 0 & 0 \end{bmatrix} = 0 \quad \checkmark$$

---

### Quiz 3: 阻尼伪逆

**题目**: 雅可比矩阵 $J = \begin{bmatrix} 1 & 0 \end{bmatrix}$，期望末端速度 $\dot{x}_e = 5$。

(a) 使用标准伪逆计算 $\dot{q}$

(b) 如果 $J = \begin{bmatrix} 0.01 & 0 \end{bmatrix}$（接近奇异），用标准伪逆计算 $\dot{q}$

(c) 对 (b) 使用阻尼伪逆（$\lambda = 0.1$）计算 $\dot{q}$

**解答**：

(a) $J^{+} = J^T(JJ^T)^{-1} = \begin{bmatrix} 1 \ 0 \end{bmatrix}$，$\dot{q} = \begin{bmatrix} 5 \ 0 \end{bmatrix}$ — 合理。

(b) $J^{+} = \begin{bmatrix} 100 \ 0 \end{bmatrix}$，$\dot{q} = \begin{bmatrix} 500 \ 0 \end{bmatrix}$ — 关节速度爆炸！

(c) 阻尼伪逆：

$$J^{*} = J^T(JJ^T + \lambda^2 I)^{-1} = \begin{bmatrix} 0.01 \ 0 \end{bmatrix} \cdot (0.0001 + 0.01)^{-1} = \begin{bmatrix} 0.01/0.0101 \ 0 \end{bmatrix} = \begin{bmatrix} 0.99 \ 0 \end{bmatrix}$$

$$\dot{q} = J^{*} \dot{x}_e = \begin{bmatrix} 4.95 \ 0 \end{bmatrix}$$

关节速度被有效限制，代价是末端速度有少量误差：$J\dot{q} = 0.0495 \neq 5$。这正是阻尼伪逆的权衡——牺牲精度换取数值稳定性。

---

### Quiz 4: 多任务优先级

**题目**: 一个 3-DOF 平面机器人，主任务为末端位置控制（$m_1 = 2$），次任务为保持第三关节角 $q_3 = 0$（$m_2 = 1$）。

在某构型下：

$$J_1 = \begin{bmatrix} -1 & -1 & -0.5 \ 1 & 0.5 & 0.5 \end{bmatrix}, \quad J_2 = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}$$

期望速度 $\dot{x}_1 = \begin{bmatrix} 1 \ 0 \end{bmatrix}$，$\dot{x}_2 = -q_3 = 0.5$（将 $q_3$ 拉回零）。

请写出多任务控制律的表达式，并解释为什么次任务可能无法被完全满足。

**解答**：

多任务控制律：

$$\dot{q} = J_1^{+} \dot{x}_1 + (I - J_1^{+}J_1)(J_2^{+} \dot{x}_2)$$

**解释**: 主任务 $J_1$ 的维度为 $2 \times 3$，零空间维度为 $n - \text{rank}(J_1) = 3 - 2 = 1$。次任务 $J_2$ 也需要 1 个自由度。但零空间中的 1 个自由度投影到次任务方向上的分量可能不足以完全满足 $\dot{x}_2$，因此次任务只能被"尽力"满足。

当零空间方向与次任务所需方向正交时，次任务完全无法执行。

---

### Quiz 5: 姿态误差计算

**题目**: 当前姿态为单位矩阵 $C_{\text{current}} = I$，目标姿态为绕 $z$ 轴旋转 $90°$：

$$C_d = \begin{bmatrix} 0 & -1 & 0 \ 1 & 0 & 0 \ 0 & 0 & 1 \end{bmatrix}$$

计算姿态误差向量 $e_o$。

**解答**：

Step 1: 计算相对旋转

$$C_{\text{rel}} = C_d \cdot C_{\text{current}}^T = C_d \cdot I = C_d$$

Step 2: 计算旋转角度

$$\theta = \arccos\!\left(\frac{\text{tr}(C_{\text{rel}}) - 1}{2}\right) = \arccos\!\left(\frac{0 + 0 + 1 - 1}{2}\right) = \arccos(0) = \frac{\pi}{2}$$

Step 3: 提取旋转轴

$$\hat{n} = \frac{1}{2\sin(\pi/2)} \begin{bmatrix} C_{32} - C_{23} \ C_{13} - C_{31} \ C_{21} - C_{12} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} 0 - 0 \ 0 - 0 \ 1 - (-1) \end{bmatrix} = \begin{bmatrix} 0 \ 0 \ 1 \end{bmatrix}$$

Step 4: 姿态误差向量

$$e_o = \theta \hat{n} = \frac{\pi}{2} \begin{bmatrix} 0 \ 0 \ 1 \end{bmatrix} = \begin{bmatrix} 0 \ 0 \ \pi/2 \end{bmatrix} \approx \begin{bmatrix} 0 \ 0 \ 1.571 \end{bmatrix}$$

**物理含义**: 需要绕 $z$ 轴旋转 $90°$ 才能从当前姿态到达目标姿态，这与直觉一致。

---

### Quiz 6: 迭代逆运动学收敛性

**题目**: 对于一个 2-DOF 平面臂（$l_1 = l_2 = 1$），目标位置 $(x_d, y_d) = (2.0, 0)$（工作空间边界）。

(a) 从 $q^{(0)} = [0.1, -0.2]^T$ 出发，迭代逆运动学能否收敛？

(b) 收敛后的构型是什么？会遇到什么问题？

**解答**：

(a) 目标点 $(2, 0)$ 恰好在工作空间边界上（$\sqrt{x^2+y^2} = 2 = l_1 + l_2$），理论上可达。迭代法可以收敛，但收敛速度会非常慢。

(b) 收敛构型为 $q = [0, 0]^T$（臂完全伸展）。此时：

$$\det(J) = l_1 l_2 \sin(q_2) = \sin(0) = 0$$

这是一个**边界奇异构型**。在收敛过程中，随着 $q_2 \to 0$，雅可比矩阵趋于奇异，$J^{+}$ 的范数趋于无穷，导致：
- 收敛速度急剧下降
- 关节速度可能出现剧烈振荡
- 建议使用**阻尼伪逆**来改善数值行为

---

## 总结：关键公式速查

| 概念 | 公式 |
|---|---|
| 正运动学 | $x_e = f(q)$ |
| 逆微分运动学 | $\dot{q} = J^{+} \dot{x}_e$ |
| 右伪逆 | $J^{+} = J^T(JJ^T)^{-1}$ |
| 阻尼伪逆 | $J^{*} = J^T(JJ^T + \lambda^2 I)^{-1}$ |
| 冗余通解 | $\dot{q} = J^{+}\dot{x}_e + (I - J^{+}J)\dot{q}_0$ |
| 零空间投影 | $N = I - J^{+}J$ |
| 多任务优先级 | $\dot{q} = J_1^{+}\dot{x}_1 + N_1(J_2^{+}\dot{x}_2)$ |
| 迭代逆运动学 | $q^{(k+1)} = q^{(k)} + \alpha J^{+}\Delta x$ |
| 姿态误差 | $e_o = \theta \hat{n}$，$\theta = \arccos\frac{\text{tr}(C_{\text{rel}})-1}{2}$ |
| 轨迹控制 | $\dot{q} = J^{+}(\dot{x}_d + K_p e) + N\dot{q}_0$ |

---

> **下一讲预告**: Lecture 5 — Dynamics 1: 拉格朗日动力学 (Lagrangian Dynamics)，从运动学进入动力学，引入质量矩阵、科氏力与重力项。
