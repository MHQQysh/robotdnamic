# Lab 1b: 微分运动学 — ABB IRB 120

> ETH Zurich "Robot Dynamics" 课程实验指导
> Exercise 1b: Differential Kinematics of ABB IRB 120

---

## 1. 实验概述 (Lab Overview)

### 1.1 实验目标

本实验的核心目标是为 ABB IRB 120 六自由度工业机械臂实现**微分运动学 (Differential Kinematics)**，即推导并编程实现**几何雅可比矩阵 (Geometric Jacobian Matrix)**。

具体任务包括：

- 实现**位置雅可比矩阵** `jointToPosJac(q)` — 将关节速度映射到末端执行器的线速度
- 实现**旋转雅可比矩阵** `jointToRotJac(q)` — 将关节速度映射到末端执行器的角速度
- 理解雅可比矩阵在速度映射、力映射和奇异性分析中的作用

### 1.2 前置要求

本实验基于 **Exercise 1a** 的正运动学 (Forward Kinematics) 实现。你需要已经完成以下函数：

- `jointToRotMat(q)` — 计算末端执行器相对于惯性系的旋转矩阵
- `jointToPosition(q)` — 计算末端执行器在惯性系中的位置
- 各关节的齐次变换矩阵 `T_01, T_12, ..., T_6E`

### 1.3 ABB IRB 120 参数回顾

ABB IRB 120 是一个 6-DOF 全旋转关节 (revolute joints) 机械臂，DH 参数如下：

| 关节 i | θ_i  | d_i (m) | a_i (m) | α_i (rad) |
|--------|------|---------|---------|-----------|
| 1      | q1   | 0.290   | 0       | -π/2      |
| 2      | q2   | 0       | 0.270   | 0         |
| 3      | q3   | 0       | 0.070   | -π/2      |
| 4      | q4   | 0.302   | 0       | π/2       |
| 5      | q5   | 0       | 0       | -π/2      |
| 6      | q6   | 0.072   | 0       | 0         |

---

## 2. 几何雅可比矩阵理论 (Geometric Jacobian Theory)

### 2.1 基本概念

几何雅可比矩阵 (Geometric Jacobian) 建立了**关节速度**与**末端执行器速度**之间的线性映射关系：

$$
\begin{bmatrix} \mathbf{v}_E \ \boldsymbol{\omega}_E \end{bmatrix}
= \mathbf{J}(\mathbf{q}) \, \dot{\mathbf{q}}
$$

其中：
- $\mathbf{v}_E \in \mathbb{R}^3$ — 末端执行器的线速度 (linear velocity)
- $\boldsymbol{\omega}_E \in \mathbb{R}^3$ — 末端执行器的角速度 (angular velocity)
- $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{6 \times 6}$ — 几何雅可比矩阵
- $\dot{\mathbf{q}} \in \mathbb{R}^6$ — 关节速度向量

完整的几何雅可比矩阵由两部分组成：

$$
\mathbf{J}(\mathbf{q}) =
\begin{bmatrix} \mathbf{J}_P \ \mathbf{J}_R \end{bmatrix}
\in \mathbb{R}^{6 \times 6}
$$

- $\mathbf{J}_P \in \mathbb{R}^{3 \times 6}$ — **位置雅可比** (Positional/Translational Jacobian)
- $\mathbf{J}_R \in \mathbb{R}^{3 \times 6}$ — **旋转雅可比** (Rotational/Angular Jacobian)

### 2.2 位置雅可比推导 (Positional Jacobian Derivation)

位置雅可比的推导基于刚体速度公式。考虑第 $i$ 个关节对末端执行器线速度的贡献：

**刚体速度关系：**

$$
\mathbf{v}_E = \mathbf{v}_i + \boldsymbol{\omega}_i \times \mathbf{r}_{i \to E}
$$

其中 $\mathbf{r}_{i \to E} = \mathbf{r}_E - \mathbf{r}_i$ 是从第 $i$ 个关节到末端执行器的位置向量。

**对于不同类型的关节：**

| 关节类型 | 位置雅可比第 i 列 $\mathbf{J}_P[:,i]$ | 物理含义 |
|---------|--------------------------------------|---------|
| 旋转关节 (Revolute) | $\mathbf{n}_i \times (\mathbf{r}_E - \mathbf{r}_i)$ | 旋转引起的切向线速度 |
| 移动关节 (Prismatic) | $\mathbf{n}_i$ | 沿关节轴的直接平移 |

其中 $\mathbf{n}_i$ 是第 $i$ 个关节旋转轴在惯性系 (inertial frame) 中的单位方向向量。

**推导过程：**

对于旋转关节，当第 $i$ 个关节以角速度 $\dot{q}_i$ 旋转时，末端执行器获得的线速度为：

$$
\mathbf{v}_{E,i} = (\boldsymbol{\omega}_i) \times \mathbf{r}_{i \to E}
= (\dot{q}_i \, \mathbf{n}_i) \times (\mathbf{r}_E - \mathbf{r}_i)
$$

因此位置雅可比的第 $i$ 列为：

$$
\mathbf{J}_P[:,i] = \frac{\partial \mathbf{v}_E}{\partial \dot{q}_i}
= \mathbf{n}_i \times (\mathbf{r}_E - \mathbf{r}_i)
$$

### 2.3 旋转雅可比推导 (Rotational Jacobian Derivation)

旋转雅可比描述了关节速度对末端执行器角速度的贡献，基于**角速度传播 (angular velocity propagation)** 原理：

$$
\boldsymbol{\omega}_E = \sum_{i=1}^{n} \dot{q}_i \, \mathbf{n}_i
$$

**对于不同类型的关节：**

| 关节类型 | 旋转雅可比第 i 列 $\mathbf{J}_R[:,i]$ | 物理含义 |
|---------|--------------------------------------|---------|
| 旋转关节 (Revolute) | $\mathbf{n}_i$ | 绕关节轴的角速度贡献 |
| 移动关节 (Prismatic) | $\mathbf{0}$ | 平移不产生角速度 |

**推导过程：**

对于旋转关节，第 $i$ 个关节旋转时直接贡献角速度 $\dot{q}_i \, \mathbf{n}_i$，因此：

$$
\mathbf{J}_R[:,i] = \frac{\partial \boldsymbol{\omega}_E}{\partial \dot{q}_i} = \mathbf{n}_i
$$

> **注意：** ABB IRB 120 的所有 6 个关节均为旋转关节，因此本实验中只需使用旋转关节的公式。

### 2.4 关键几何量的计算

为了组装雅可比矩阵，需要计算以下几何量：

**旋转轴 $\mathbf{n}_i$：**

假设每个关节的局部旋转轴为 z 轴（DH 约定），则第 $i$ 个关节的旋转轴在惯性系中的表示为：

$$
\mathbf{n}_i = \mathbf{C}_{I,i-1} \, \mathbf{e}_z
= \mathbf{C}_{I,i-1} \begin{bmatrix} 0 \ 0 \ 1 \end{bmatrix}
$$

其中 $\mathbf{C}_{I,i-1}$ 是从第 $i-1$ 个坐标系到惯性系的旋转矩阵（即关节 $i$ 旋转之前的坐标系姿态）。

**关节位置 $\mathbf{r}_i$：**

第 $i$ 个关节在惯性系中的位置，可从齐次变换矩阵中提取：

$$
\mathbf{r}_i = \mathbf{T}_{0,i-1}[1:3, 4]
$$

**末端执行器位置 $\mathbf{r}_E$：**

$$
\mathbf{r}_E = \mathbf{T}_{0,E}[1:3, 4]
$$

---

## 3. 需要实现的函数

### 3.1 函数 `jointToPosJac(q)` — 位置雅可比矩阵

**函数签名：**

```matlab
function I_Jp = jointToPosJac(q)
% jointToPosJac 计算ABB IRB 120的位置雅可比矩阵 (Positional Jacobian)
%
% 输入:
%   q  - 6x1 关节角度向量 [rad]
%
% 输出:
%   I_Jp - 3x6 位置雅可比矩阵，表示在惯性系 (inertial frame) 中
%          将关节速度 dq 映射到末端执行器线速度 v_E:
%          v_E = I_Jp * dq
```

**几何方法实现：**

对于 ABB IRB 120 的每个旋转关节 $i = 1, 2, \ldots, 6$：

$$
\mathbf{J}_P[:,i] = \mathbf{n}_i \times (\mathbf{r}_E - \mathbf{r}_i)
$$

**完整 MATLAB 实现：**

```matlab
function I_Jp = jointToPosJac(q)
    % 获取各关节的齐次变换矩阵 (来自 Exercise 1a)
    T_I0 = getTransformI0();
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);
    T_6E = getTransform6E();

    % 计算从惯性系到各关节坐标系的累积变换
    T_I1 = T_I0 * T_01;
    T_I2 = T_I1 * T_12;
    T_I3 = T_I2 * T_23;
    T_I4 = T_I3 * T_34;
    T_I5 = T_I4 * T_45;
    T_I6 = T_I5 * T_56;
    T_IE = T_I6 * T_6E;

    % 提取各关节在惯性系中的位置 r_i
    r_I0 = T_I0(1:3, 4);
    r_I1 = T_I1(1:3, 4);
    r_I2 = T_I2(1:3, 4);
    r_I3 = T_I3(1:3, 4);
    r_I4 = T_I4(1:3, 4);
    r_I5 = T_I5(1:3, 4);

    % 末端执行器位置
    r_IE = T_IE(1:3, 4);

    % 提取各关节旋转轴在惯性系中的方向 n_i
    % 每个关节的旋转轴是其前一个坐标系的 z 轴
    % n_i = C_{I,i-1} * [0; 0; 1]
    n_1 = T_I0(1:3, 3);   % 关节1: 基座坐标系的z轴
    n_2 = T_I1(1:3, 3);   % 关节2: 坐标系1的z轴
    n_3 = T_I2(1:3, 3);   % 关节3: 坐标系2的z轴
    n_4 = T_I3(1:3, 3);   % 关节4: 坐标系3的z轴
    n_5 = T_I4(1:3, 3);   % 关节5: 坐标系4的z轴
    n_6 = T_I5(1:3, 3);   % 关节6: 坐标系5的z轴

    % 组装位置雅可比矩阵 (3x6)
    % J_P[:,i] = n_i x (r_E - r_i)  (旋转关节)
    I_Jp = zeros(3, 6);
    I_Jp(:, 1) = cross(n_1, r_IE - r_I0);
    I_Jp(:, 2) = cross(n_2, r_IE - r_I1);
    I_Jp(:, 3) = cross(n_3, r_IE - r_I2);
    I_Jp(:, 4) = cross(n_4, r_IE - r_I3);
    I_Jp(:, 5) = cross(n_5, r_IE - r_I4);
    I_Jp(:, 6) = cross(n_6, r_IE - r_I5);
end
```

### 3.2 函数 `jointToRotJac(q)` — 旋转雅可比矩阵

**函数签名：**

```matlab
function I_Jr = jointToRotJac(q)
% jointToRotJac 计算ABB IRB 120的旋转雅可比矩阵 (Rotational Jacobian)
%
% 输入:
%   q  - 6x1 关节角度向量 [rad]
%
% 输出:
%   I_Jr - 3x6 旋转雅可比矩阵，表示在惯性系 (inertial frame) 中
%          将关节速度 dq 映射到末端执行器角速度 omega_E:
%          omega_E = I_Jr * dq
```

**几何方法实现：**

对于 ABB IRB 120 的每个旋转关节 $i = 1, 2, \ldots, 6$：

$$
\mathbf{J}_R[:,i] = \mathbf{n}_i
$$

**完整 MATLAB 实现：**

```matlab
function I_Jr = jointToRotJac(q)
    % 获取各关节的齐次变换矩阵 (来自 Exercise 1a)
    T_I0 = getTransformI0();
    T_01 = jointToTransform01(q);
    T_12 = jointToTransform12(q);
    T_23 = jointToTransform23(q);
    T_34 = jointToTransform34(q);
    T_45 = jointToTransform45(q);
    T_56 = jointToTransform56(q);

    % 计算从惯性系到各关节坐标系的累积变换
    T_I1 = T_I0 * T_01;
    T_I2 = T_I1 * T_12;
    T_I3 = T_I2 * T_23;
    T_I4 = T_I3 * T_34;
    T_I5 = T_I4 * T_45;

    % 提取各关节旋转轴在惯性系中的方向 n_i
    % n_i = C_{I,i-1} * [0; 0; 1] = T_{I,i-1}(1:3, 3)
    n_1 = T_I0(1:3, 3);   % 关节1的旋转轴
    n_2 = T_I1(1:3, 3);   % 关节2的旋转轴
    n_3 = T_I2(1:3, 3);   % 关节3的旋转轴
    n_4 = T_I3(1:3, 3);   % 关节4的旋转轴
    n_5 = T_I4(1:3, 3);   % 关节5的旋转轴
    n_6 = T_I5(1:3, 3);   % 关节6的旋转轴

    % 组装旋转雅可比矩阵 (3x6)
    % J_R[:,i] = n_i  (旋转关节)
    I_Jr = [n_1, n_2, n_3, n_4, n_5, n_6];
end
```

> **实现要点：** 旋转雅可比比位置雅可比更简单——每一列直接就是对应关节的旋转轴方向向量，不需要叉乘运算。

---

## 4. 实现步骤详解 (Implementation Steps)

### 步骤 1: 计算所有关节的旋转矩阵 $\mathbf{C}_{I,k}$

从 Exercise 1a 中已有的齐次变换矩阵出发，逐级累乘得到每个关节坐标系相对于惯性系的变换：

```matlab
% 累积齐次变换
T_I0 = getTransformI0();          % 惯性系 -> 基座
T_I1 = T_I0 * jointToTransform01(q);  % 惯性系 -> 关节1后
T_I2 = T_I1 * jointToTransform12(q);  % 惯性系 -> 关节2后
T_I3 = T_I2 * jointToTransform23(q);  % 惯性系 -> 关节3后
T_I4 = T_I3 * jointToTransform34(q);  % 惯性系 -> 关节4后
T_I5 = T_I4 * jointToTransform45(q);  % 惯性系 -> 关节5后
T_I6 = T_I5 * jointToTransform56(q);  % 惯性系 -> 关节6后
T_IE = T_I6 * getTransform6E();       % 惯性系 -> 末端执行器

% 提取旋转矩阵部分
C_I0 = T_I0(1:3, 1:3);
C_I1 = T_I1(1:3, 1:3);
% ... 以此类推
```

### 步骤 2: 确定每个关节的旋转轴 $\mathbf{n}_i$

根据 DH 约定，每个关节的旋转轴是其**前一个**坐标系的 z 轴：

```matlab
% 关节旋转轴 (在惯性系中表示)
% n_i = C_{I,i-1} * e_z = T_{I,i-1} 的第3列 (前3行)
e_z = [0; 0; 1];

n_1 = C_I0 * e_z;   % 或直接 T_I0(1:3, 3)
n_2 = C_I1 * e_z;   % 或直接 T_I1(1:3, 3)
n_3 = C_I2 * e_z;
n_4 = C_I3 * e_z;
n_5 = C_I4 * e_z;
n_6 = C_I5 * e_z;
```

> **理解要点：** 为什么是 $\mathbf{C}_{I,i-1}$ 而不是 $\mathbf{C}_{I,i}$？因为关节 $i$ 的旋转轴在关节旋转**之前**就已经确定了，它是第 $i-1$ 个坐标系的 z 轴。变换矩阵 $\mathbf{T}_{I,i}$ 已经包含了关节 $i$ 的旋转，所以我们需要用 $\mathbf{T}_{I,i-1}$。

### 步骤 3: 计算每个关节的位置 $\mathbf{r}_i$

```matlab
% 关节位置 (在惯性系中)
r_0 = T_I0(1:3, 4);   % 关节1所在位置 (基座原点)
r_1 = T_I1(1:3, 4);   % 关节2所在位置
r_2 = T_I2(1:3, 4);   % 关节3所在位置
r_3 = T_I3(1:3, 4);   % 关节4所在位置
r_4 = T_I4(1:3, 4);   % 关节5所在位置
r_5 = T_I5(1:3, 4);   % 关节6所在位置
```

### 步骤 4: 计算末端执行器位置 $\mathbf{r}_E$

```matlab
r_E = T_IE(1:3, 4);   % 末端执行器位置
```

### 步骤 5: 组装雅可比矩阵

```matlab
% 位置雅可比 (3x6)
I_Jp = zeros(3, 6);
positions = [r_0, r_1, r_2, r_3, r_4, r_5];
axes = [n_1, n_2, n_3, n_4, n_5, n_6];

for i = 1:6
    I_Jp(:, i) = cross(axes(:,i), r_E - positions(:,i));
end

% 旋转雅可比 (3x6)
I_Jr = axes;  % 直接就是旋转轴的集合

% 完整几何雅可比 (6x6)
I_J = [I_Jp; I_Jr];
```

---

## 5. 雅可比矩阵的应用

### 5.1 速度映射 (Velocity Mapping)

雅可比矩阵最直接的应用是将关节空间速度映射到操作空间 (task space) 速度：

$$
\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \, \dot{\mathbf{q}}
$$

展开为：

$$
\begin{bmatrix} \mathbf{v}_E \ \boldsymbol{\omega}_E \end{bmatrix}
=
\begin{bmatrix} \mathbf{J}_P \ \mathbf{J}_R \end{bmatrix}
\dot{\mathbf{q}}
$$

```matlab
% 示例: 给定关节速度，计算末端执行器速度
q = [0; pi/4; -pi/6; 0; pi/3; 0];    % 关节角度
dq = [0.1; 0.2; -0.1; 0.3; 0; 0.1];  % 关节速度 [rad/s]

I_Jp = jointToPosJac(q);
I_Jr = jointToRotJac(q);

v_E = I_Jp * dq;      % 末端执行器线速度 [m/s]
omega_E = I_Jr * dq;   % 末端执行器角速度 [rad/s]

fprintf('末端执行器线速度: [%.4f, %.4f, %.4f] m/s\n', v_E);
fprintf('末端执行器角速度: [%.4f, %.4f, %.4f] rad/s\n', omega_E);
```


### 5.2 逆速度映射 (Inverse Velocity Mapping)

当雅可比矩阵可逆时（即不在奇异位形），可以从期望的末端执行器速度反求关节速度：

$$
\dot{\mathbf{q}} = \mathbf{J}^{-1}(\mathbf{q}) \, \dot{\mathbf{x}}
$$

```matlab
% 示例: 给定期望末端速度，求解关节速度
v_desired = [0.1; 0; -0.05];       % 期望线速度 [m/s]
omega_desired = [0; 0.1; 0];       % 期望角速度 [rad/s]
dx_desired = [v_desired; omega_desired];

I_J = [jointToPosJac(q); jointToRotJac(q)];  % 6x6 完整雅可比

% 检查是否接近奇异位形
det_J = det(I_J);
fprintf('det(J) = %.6f
', det_J);

if abs(det_J) > 1e-6
    dq_required = I_J \ dx_desired;  % 求解关节速度
    fprintf('所需关节速度: [');
    fprintf('%.4f ', dq_required);
    fprintf('] rad/s
');
else
    warning('机械臂接近奇异位形! det(J) ≈ 0');
end
```

### 5.3 力映射 — 虚功原理 (Force Mapping via Virtual Work)

雅可比矩阵的转置建立了**操作空间力/力矩**与**关节力矩**之间的映射关系。这基于虚功原理 (Principle of Virtual Work)：

$$
oldsymbol{	au} = \mathbf{J}^T(\mathbf{q}) \, \mathbf{F}
$$

其中：
- $oldsymbol{	au} \in \mathbb{R}^6$ — 关节力矩向量
- $\mathbf{F} \in \mathbb{R}^6$ — 末端执行器广义力（包含力和力矩）

**推导：** 根据虚功原理，关节空间虚功等于操作空间虚功：

$$
\delta W = oldsymbol{	au}^T \delta \mathbf{q}
= \mathbf{F}^T \delta \mathbf{x}
= \mathbf{F}^T \mathbf{J} \, \delta \mathbf{q}
$$

由于 $\delta \mathbf{q}$ 是任意的，因此：

$$
oldsymbol{	au} = \mathbf{J}^T \mathbf{F}
$$

```matlab
% 示例: 末端执行器施加力时所需的关节力矩
F_end = [0; 0; -10; 0; 0; 0];  % 末端z方向施加10N力 (力 + 力矩)

I_J = [jointToPosJac(q); jointToRotJac(q)];
tau = I_J' * F_end;  % 所需关节力矩

fprintf('所需关节力矩:
');
for i = 1:6
    fprintf('  关节%d: %.4f Nm
', i, tau(i));
end
```


### 5.4 奇异性分析 (Singularity Analysis)

当雅可比矩阵**奇异** (singular) 时，机械臂失去某些方向的运动能力。奇异性条件为：

$$
\det(\mathbf{J}(\mathbf{q})) = 0
$$

**奇异性的物理含义：**

- 在奇异位形下，末端执行器在某些方向上无法产生速度
- 逆运动学解不唯一或不存在
- 关节速度可能趋向无穷大
- 机械臂的可操作性 (manipulability) 降为零

**ABB IRB 120 的典型奇异位形：**

1. **肩部奇异 (Shoulder Singularity):** 末端执行器位于关节1轴线上
2. **肘部奇异 (Elbow Singularity):** 手臂完全伸直或完全折叠 ($q_3$ 使连杆共线)
3. **腕部奇异 (Wrist Singularity):** 关节4和关节6的旋转轴对齐 ($q_5 = 0$)

```matlab
% 奇异性分析示例
function analyzeSingularity(q)
    I_J = [jointToPosJac(q); jointToRotJac(q)];

    % 行列式
    det_J = det(I_J);
    fprintf('det(J) = %.6e\n', det_J);

    % 条件数 (condition number) — 衡量接近奇异的程度
    cond_J = cond(I_J);
    fprintf('cond(J) = %.2f\n', cond_J);

    % 奇异值分解 (SVD)
    sigma = svd(I_J);
    fprintf('奇异值: [');
    fprintf('%.4f ', sigma);
    fprintf(']\n');

    % 可操作性指标 (Yoshikawa manipulability index)
    w = sqrt(det(I_J * I_J'));
    fprintf('可操作性指标 w = %.6f\n', w);

    if abs(det_J) < 1e-6
        fprintf('警告: 机械臂处于或接近奇异位形!\n');
        % 找出最小奇异值对应的方向
        [U, S, V] = svd(I_J);
        fprintf('最受限的操作空间方向: [');
        fprintf('%.4f ', U(:, end));
        fprintf(']\n');
        fprintf('最受限的关节空间方向: [');
        fprintf('%.4f ', V(:, end));
        fprintf(']\n');
    end
end
```

```matlab
% 测试不同位形的奇异性
q_home = [0; 0; 0; 0; 0; 0];
q_wrist_singular = [0; pi/4; -pi/6; 0; 0; 0];  % q5=0 腕部奇异
q_elbow_singular = [0; 0; pi/2; 0; pi/3; 0];    % 肘部接近奇异

fprintf('=== 零位形 ===\n');
analyzeSingularity(q_home);

fprintf('\n=== 腕部奇异位形 (q5=0) ===\n');
analyzeSingularity(q_wrist_singular);
```

### 5.5 可操作性椭球 (Manipulability Ellipsoid)

雅可比矩阵还可以用来可视化机械臂在某一位形下的运动能力：

```matlab
% 绘制速度可操作性椭球
function plotManipulabilityEllipsoid(q)
    I_Jp = jointToPosJac(q);
    r_E = jointToPosition(q);

    % 计算椭球矩阵
    A = I_Jp * I_Jp';  % 3x3 对称正定矩阵

    % 特征值分解
    [V, D] = eig(A);
    radii = sqrt(diag(D));  % 椭球半轴长度

    % 生成单位球面
    [X, Y, Z] = sphere(30);

    % 变换为椭球
    pts = V * diag(radii) * [X(:)'; Y(:)'; Z(:)'];
    X_e = reshape(pts(1,:), size(X)) + r_E(1);
    Y_e = reshape(pts(2,:), size(Y)) + r_E(2);
    Z_e = reshape(pts(3,:), size(Z)) + r_E(3);

    surf(X_e, Y_e, Z_e, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    hold on;

    % 绘制主轴方向
    for i = 1:3
        arrow_end = r_E + radii(i) * V(:,i);
        plot3([r_E(1), arrow_end(1)], ...
              [r_E(2), arrow_end(2)], ...
              [r_E(3), arrow_end(3)], 'r-', 'LineWidth', 2);
    end

    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('速度可操作性椭球 (Velocity Manipulability Ellipsoid)');
    axis equal; grid on;
end
```

---

## 6. 测试和验证

### 6.1 数值微分验证 (Numerical Differentiation Check)

验证几何雅可比的最可靠方法是与**数值微分**结果进行比较。数值雅可比通过有限差分 (finite difference) 近似计算：

$$
\mathbf{J}_{P,\text{num}}[:,i]
\approx \frac{\mathbf{r}_E(\mathbf{q} + \epsilon \, \mathbf{e}_i) - \mathbf{r}_E(\mathbf{q} - \epsilon \, \mathbf{e}_i)}{2\epsilon}
$$

```matlab
function checkJacobian(q)
    % 解析雅可比 (你的实现)
    I_Jp_analytical = jointToPosJac(q);
    I_Jr_analytical = jointToRotJac(q);

    % 数值雅可比 (有限差分)
    epsilon = 1e-6;
    I_Jp_numerical = zeros(3, 6);
    I_Jr_numerical = zeros(3, 6);

    for i = 1:6
        % 正向扰动
        q_plus = q;
        q_plus(i) = q_plus(i) + epsilon;

        % 反向扰动
        q_minus = q;
        q_minus(i) = q_minus(i) - epsilon;

        % 位置雅可比 — 中心差分
        r_plus = jointToPosition(q_plus);
        r_minus = jointToPosition(q_minus);
        I_Jp_numerical(:, i) = (r_plus - r_minus) / (2 * epsilon);

        % 旋转雅可比 — 通过旋转矩阵差分
        C_plus = jointToRotMat(q_plus);
        C_minus = jointToRotMat(q_minus);
        % dC/dq_i ≈ (C_plus - C_minus) / (2*epsilon)
        % omega_hat = dC * C^T  =>  提取角速度分量
        dC = (C_plus - C_minus) / (2 * epsilon);
        C_current = jointToRotMat(q);
        omega_hat = dC * C_current';
        % 从反对称矩阵提取角速度向量
        I_Jr_numerical(:, i) = [omega_hat(3,2);
                                 omega_hat(1,3);
                                 omega_hat(2,1)];
    end

    % 比较结果
    pos_error = norm(I_Jp_analytical - I_Jp_numerical, 'fro');
    rot_error = norm(I_Jr_analytical - I_Jr_numerical, 'fro');

    fprintf('=== 雅可比矩阵验证 ===\n');
    fprintf('测试关节角度 q = [');
    fprintf('%.4f ', q);
    fprintf('] rad\n\n');

    fprintf('位置雅可比误差 (Frobenius norm): %.2e\n', pos_error);
    fprintf('旋转雅可比误差 (Frobenius norm): %.2e\n', rot_error);

    if pos_error < 1e-4 && rot_error < 1e-4
        fprintf('验证通过! 解析雅可比与数值雅可比一致。\n');
    else
        fprintf('验证失败! 请检查实现。\n');
        fprintf('\n解析位置雅可比:\n');
        disp(I_Jp_analytical);
        fprintf('数值位置雅可比:\n');
        disp(I_Jp_numerical);
        fprintf('\n解析旋转雅可比:\n');
        disp(I_Jr_analytical);
        fprintf('数值旋转雅可比:\n');
        disp(I_Jr_numerical);
    end
end
```


### 6.2 多位形测试

```matlab
% 在多个关节位形下测试
fprintf('========================================\n');
fprintf('  ABB IRB 120 雅可比矩阵验证测试\n');
fprintf('========================================\n\n');

% 测试1: 零位形
fprintf('--- 测试1: 零位形 ---\n');
q_test1 = [0; 0; 0; 0; 0; 0];
checkJacobian(q_test1);

% 测试2: 随机位形
fprintf('\n--- 测试2: 随机位形 ---\n');
q_test2 = [0.5; -0.3; 0.8; -1.2; 0.6; -0.4];
checkJacobian(q_test2);

% 测试3: 另一个随机位形
fprintf('\n--- 测试3: 另一个随机位形 ---\n');
q_test3 = [pi/4; pi/3; -pi/6; pi/2; -pi/4; pi/6];
checkJacobian(q_test3);

% 测试4: 接近奇异位形
fprintf('\n--- 测试4: 接近腕部奇异位形 ---\n');
q_test4 = [0.1; 0.5; -0.3; 0.7; 0.001; 0.2];  % q5 ≈ 0
checkJacobian(q_test4);
```

### 6.3 速度一致性测试

验证通过雅可比计算的速度与通过正运动学差分得到的速度一致：

```matlab
function checkVelocityConsistency(q, dq)
    dt = 1e-8;

    % 方法1: 通过雅可比计算
    v_jac = jointToPosJac(q) * dq;
    omega_jac = jointToRotJac(q) * dq;

    % 方法2: 通过正运动学差分
    r_now = jointToPosition(q);
    r_next = jointToPosition(q + dq * dt);
    v_diff = (r_next - r_now) / dt;

    C_now = jointToRotMat(q);
    C_next = jointToRotMat(q + dq * dt);
    dC = (C_next - C_now) / dt;
    omega_hat = dC * C_now';
    omega_diff = [omega_hat(3,2); omega_hat(1,3); omega_hat(2,1)];

    fprintf('线速度 (雅可比):  [%.6f, %.6f, %.6f]\n', v_jac);
    fprintf('线速度 (差分):    [%.6f, %.6f, %.6f]\n', v_diff);
    fprintf('线速度误差:       %.2e\n', norm(v_jac - v_diff));
    fprintf('角速度 (雅可比):  [%.6f, %.6f, %.6f]\n', omega_jac);
    fprintf('角速度 (差分):    [%.6f, %.6f, %.6f]\n', omega_diff);
    fprintf('角速度误差:       %.2e\n', norm(omega_jac - omega_diff));
end

% 运行速度一致性测试
q = [0.3; -0.5; 0.7; 1.1; -0.4; 0.8];
dq = [0.5; -0.3; 0.2; 0.8; -0.6; 0.1];
fprintf('\n=== 速度一致性测试 ===\n');
checkVelocityConsistency(q, dq);
```

---

## 7. 常见错误与调试技巧

### 7.1 常见错误

| 错误类型 | 症状 | 解决方法 |
|---------|------|---------|
| 旋转轴索引错误 | 雅可比某些列不正确 | 确认 $\mathbf{n}_i$ 使用的是 $\mathbf{T}_{I,i-1}$ 而非 $\mathbf{T}_{I,i}$ |
| 位置向量错误 | 位置雅可比整体偏差 | 确认 $\mathbf{r}_i$ 是关节 $i$ 的位置，而非连杆 $i$ 的末端 |
| 叉乘顺序错误 | 符号相反 | 确认是 $\mathbf{n}_i \times (\mathbf{r}_E - \mathbf{r}_i)$ 而非反过来 |
| 坐标系不一致 | 数值完全不对 | 确保所有向量都在同一坐标系（惯性系）中表示 |
| 忘记 $T_{I0}$ | 基座偏移未考虑 | 确认包含了从惯性系到基座的变换 |

### 7.2 逐列调试法

如果整体验证不通过，可以逐列检查：

```matlab
% 逐列调试: 每次只动一个关节
for joint = 1:6
    fprintf('\n--- 检查关节 %d 对应的雅可比列 ---\n', joint);
    dq_single = zeros(6, 1);
    dq_single(joint) = 1.0;  % 只有第 joint 个关节运动

    v_jac = jointToPosJac(q) * dq_single;
    omega_jac = jointToRotJac(q) * dq_single;

    % 数值验证
    eps = 1e-7;
    q_p = q; q_p(joint) = q_p(joint) + eps;
    q_m = q; q_m(joint) = q_m(joint) - eps;

    v_num = (jointToPosition(q_p) - jointToPosition(q_m)) / (2*eps);

    fprintf('  J_P(:,%d) 解析: [%.6f, %.6f, %.6f]\n', joint, v_jac);
    fprintf('  J_P(:,%d) 数值: [%.6f, %.6f, %.6f]\n', joint, v_num);
    fprintf('  误差: %.2e\n', norm(v_jac - v_num));
end
```


---

## 8. 总结与扩展

### 8.1 本实验核心公式总结

对于 ABB IRB 120（全旋转关节机械臂）：

| 矩阵 | 维度 | 第 i 列公式 | 含义 |
|------|------|------------|------|
| $\mathbf{J}_P$ | $3 \times 6$ | $\mathbf{n}_i \times (\mathbf{r}_E - \mathbf{r}_i)$ | 关节 $i$ 对末端线速度的贡献 |
| $\mathbf{J}_R$ | $3 \times 6$ | $\mathbf{n}_i$ | 关节 $i$ 对末端角速度的贡献 |
| $\mathbf{J}$ | $6 \times 6$ | $[\mathbf{J}_P; \mathbf{J}_R]$ | 完整几何雅可比 |

其中关键几何量：

- **旋转轴：** $\mathbf{n}_i = \mathbf{C}_{I,i-1} \begin{bmatrix} 0 \ 0 \ 1 \end{bmatrix} = \mathbf{T}_{I,i-1}(1:3, 3)$
- **关节位置：** $\mathbf{r}_i = \mathbf{T}_{I,i-1}(1:3, 4)$
- **末端位置：** $\mathbf{r}_E = \mathbf{T}_{I,E}(1:3, 4)$

### 8.2 雅可比矩阵的三大应用总结

```
速度映射 (正向):    ẋ  = J(q) · q̇        关节速度 → 末端速度
速度映射 (逆向):    q̇  = J⁻¹(q) · ẋ      末端速度 → 关节速度
力映射 (虚功原理):  τ  = Jᵀ(q) · F        末端力   → 关节力矩
奇异性分析:         det(J) = 0             失去自由度
```

### 8.3 与后续实验的关联

- **Exercise 2 (Inverse Kinematics):** 雅可比矩阵用于基于梯度的迭代逆运动学求解
  - $\mathbf{q}_{k+1} = \mathbf{q}_k + \mathbf{J}^{-1}(\mathbf{q}_k) \, \Delta \mathbf{x}$
  - 在奇异位形附近需要使用阻尼最小二乘法 (Damped Least Squares / Levenberg-Marquardt)
- **Exercise 3 (Dynamics):** 雅可比矩阵出现在操作空间动力学方程中
  - 操作空间惯性矩阵: $\mathbf{\Lambda} = (\mathbf{J} \mathbf{M}^{-1} \mathbf{J}^T)^{-1}$
- **Exercise 4 (Control):** 基于雅可比的操作空间阻抗控制和力控制

### 8.4 扩展阅读

- Siciliano, B. et al., *Robotics: Modelling, Planning and Control*, Chapter 3.
- Spong, M. W. et al., *Robot Modeling and Control*, Chapter 4.
- Corke, P., *Robotics, Vision and Control*, Chapter 8.
- ETH Robot Dynamics 课程讲义 Lecture 03: Kinematics 2

---

## 附录 A: 叉乘运算的矩阵形式

在实现中经常用到的叉乘运算可以写成矩阵形式。对于向量 $\mathbf{a} = [a_1, a_2, a_3]^T$：

$$
\mathbf{a} \times \mathbf{b} = \mathbf{a}^{\wedge} \, \mathbf{b}
= \begin{bmatrix} 0 & -a_3 & a_2 \ a_3 & 0 & -a_1 \ -a_2 & a_1 & 0 \end{bmatrix} \mathbf{b}
$$

其中 $\mathbf{a}^{\wedge}$ 称为反对称矩阵 (skew-symmetric matrix)。

MATLAB 辅助函数：

```matlab
function S = skew(v)
% skew 将3x1向量转换为3x3反对称矩阵
%   S = skew(v) 使得 S * w = cross(v, w)
    S = [  0,   -v(3),  v(2);
          v(3),  0,    -v(1);
         -v(2),  v(1),  0  ];
end
```

使用反对称矩阵，位置雅可比可以写成更紧凑的形式：

```matlab
% 等价写法
I_Jp(:, i) = cross(n_i, r_IE - r_i);
I_Jp(:, i) = skew(n_i) * (r_IE - r_i);  % 矩阵形式
```

## 附录 B: 完整测试脚本

```matlab
%% Lab 1b 完整测试脚本
% ABB IRB 120 微分运动学验证
clear; clc;

fprintf('============================================\n');
fprintf('  Lab 1b: ABB IRB 120 微分运动学测试\n');
fprintf('============================================\n\n');

%% 测试1: 多位形雅可比验证
test_configs = {
    [0; 0; 0; 0; 0; 0],                        '零位形';
    [0.5; -0.3; 0.8; -1.2; 0.6; -0.4],         '随机位形1';
    [pi/4; pi/3; -pi/6; pi/2; -pi/4; pi/6],    '随机位形2';
    [-0.8; 1.0; -0.5; 0.3; 1.2; -0.9],         '随机位形3';
};

all_passed = true;
for t = 1:size(test_configs, 1)
    q = test_configs{t, 1};
    name = test_configs{t, 2};
    fprintf('--- %s ---\n', name);

    % 解析雅可比
    Jp = jointToPosJac(q);
    Jr = jointToRotJac(q);

    % 数值雅可比
    eps_val = 1e-6;
    Jp_num = zeros(3, 6);
    Jr_num = zeros(3, 6);

    for i = 1:6
        q_p = q; q_p(i) = q_p(i) + eps_val;
        q_m = q; q_m(i) = q_m(i) - eps_val;

        Jp_num(:,i) = (jointToPosition(q_p) - jointToPosition(q_m)) / (2*eps_val);

        C_p = jointToRotMat(q_p);
        C_m = jointToRotMat(q_m);
        C_c = jointToRotMat(q);
        dC = (C_p - C_m) / (2*eps_val);
        W = dC * C_c';
        Jr_num(:,i) = [W(3,2); W(1,3); W(2,1)];
    end

    err_p = norm(Jp - Jp_num, 'fro');
    err_r = norm(Jr - Jr_num, 'fro');

    if err_p < 1e-4 && err_r < 1e-4
        fprintf('  通过  位置雅可比误差: %.2e  旋转雅可比误差: %.2e\n\n', err_p, err_r);
    else
        fprintf('  失败  位置雅可比误差: %.2e  旋转雅可比误差: %.2e\n\n', err_p, err_r);
        all_passed = false;
    end
end

%% 测试2: 奇异性检测
fprintf('--- 奇异性检测 ---\n');
q_singular = [0; pi/4; -pi/6; 0; 0; 0];  % q5=0
J_full = [jointToPosJac(q_singular); jointToRotJac(q_singular)];
fprintf('腕部奇异位形 (q5=0): det(J) = %.6e\n\n', det(J_full));

%% 总结
if all_passed
    fprintf('所有测试通过!\n');
else
    fprintf('存在未通过的测试，请检查实现。\n');
end
```

---

> **提示：** 完成本实验后，建议在不同关节位形下可视化雅可比矩阵的奇异值，直观感受机械臂在不同姿态下的运动能力差异。这对理解后续的轨迹规划和控制非常有帮助。
