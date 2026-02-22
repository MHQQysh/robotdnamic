# Lab 1a: ABB IRB 120 正运动学 (Forward Kinematics)

> ETH Zurich "Robot Dynamics" 课程 — 实验 1a

---

## 1. 实验概述 (Lab Overview)

### 1.1 实验目标

本实验的核心目标是为 **ABB IRB 120** 六自由度 (6-DOF) 工业机器人臂实现完整的 **正运动学 (Forward Kinematics)** 求解。正运动学的任务是：给定一组关节角度 $q \in \mathbb{R}^6$，计算末端执行器 (End-Effector) 在惯性坐标系 (Inertial Frame) 中的位姿 (位置 + 姿态)。

具体而言，你需要：

- 建立从惯性系 $I$ 到基座系 $0$ 的变换
- 建立每对相邻关节之间的齐次变换矩阵
- 通过链式乘法得到末端执行器的完整位姿
- 实现四元数 (Quaternion) 与旋转矩阵 (Rotation Matrix) 之间的相互转换
- 实现四元数乘法和用四元数旋转向量

### 1.2 工具与环境

- **编程语言**: MATLAB
- **机器人**: ABB IRB 120 — 一款紧凑型六自由度工业机器人臂，广泛用于装配、搬运等任务
- **可视化**: 通过 STL 模型进行 3D 渲染

### 1.3 ABB IRB 120 简介

ABB IRB 120 是一款小型工业机器人，具有以下特点：

| 参数 | 值 |
|------|-----|
| 自由度 (DOF) | 6 |
| 负载能力 | 3 kg |
| 工作半径 | 580 mm |
| 重复定位精度 | 0.01 mm |
| 关节类型 | 全部为旋转关节 (Revolute) |

### 1.4 文件结构

```
Lab01a/
├── init_workspace.m          % 初始化工作空间，设置路径和参数
├── evaluate_problems.m       % 自动评估所有函数的正确性
├── loadviz.m                 % 加载 STL 模型进行 3D 可视化
├── problems/                 % 需要你实现的函数（在此文件夹中编写代码）
│   ├── getTransformI0.m
│   ├── getTransform6E.m
│   ├── jointToTransform01.m
│   ├── jointToTransform12.m
│   ├── jointToTransform23.m
│   ├── jointToTransform34.m
│   ├── jointToTransform45.m
│   ├── jointToTransform56.m
│   ├── jointToRotMat.m
│   ├── jointToPosition.m
│   ├── jointToQuat.m
│   ├── quatToRotMat.m
│   ├── rotMatToQuat.m
│   ├── quatMult.m
│   └── rotVecWithQuat.m
└── solutions/                % 参考解（用于自动评估对比）
    └── ...
```

**工作流程**:

1. 运行 `init_workspace.m` 初始化环境
2. 在 `problems/` 文件夹中逐一实现各函数
3. 运行 `evaluate_problems.m` 检验实现是否正确
4. 使用 `loadviz.m` 进行可视化验证

---

## 2. 关键理论 (Key Theory)

### 2.1 齐次变换矩阵 (Homogeneous Transformation Matrix)

齐次变换矩阵是描述刚体位姿的核心工具。一个 $4 \times 4$ 齐次变换矩阵 $T$ 的结构为：

$$
T = \begin{bmatrix} C & r \ 0 \; 0 \; 0 & 1 \end{bmatrix} \in \mathbb{R}^{4 \times 4}
$$

其中：
- $C \in SO(3)$ 是 $3 \times 3$ 旋转矩阵 (Rotation Matrix)，描述姿态
- $r \in \mathbb{R}^3$ 是 $3 \times 1$ 平移向量 (Translation Vector)，描述位置

**链式乘法 (Chain Multiplication)**：从惯性系 $I$ 到末端执行器 $E$ 的总变换为：

$$
T_{IE} = T_{I0} \cdot T_{01} \cdot T_{12} \cdot T_{23} \cdot T_{34} \cdot T_{45} \cdot T_{56} \cdot T_{6E}
$$

这是正运动学的核心公式。每个 $T_{(k-1)k}$ 描述了从第 $k-1$ 个关节坐标系到第 $k$ 个关节坐标系的变换。

### 2.2 基本旋转矩阵

绕三个坐标轴的基本旋转矩阵：

**绕 x 轴旋转角度 $\theta$**:

$$
R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \ 0 & \cos\theta & -\sin\theta \ 0 & \sin\theta & \cos\theta \end{bmatrix}
$$

**绕 y 轴旋转角度 $\theta$**:

$$
R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \ 0 & 1 & 0 \ -\sin\theta & 0 & \cos\theta \end{bmatrix}
$$

**绕 z 轴旋转角度 $\theta$**:

$$
R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \ \sin\theta & \cos\theta & 0 \ 0 & 0 & 1 \end{bmatrix}
$$

### 2.3 ABB IRB 120 的 DH 参数

ABB IRB 120 的连杆参数 (Link Parameters) 如下表所示。这些参数定义了相邻关节坐标系之间的几何关系：

| 关节 $i$ | $\theta_i$ (关节角) | $d_i$ (mm) | $a_i$ (mm) | $\alpha_i$ (rad) |
|-----------|---------------------|-------------|-------------|-------------------|
| 1 | $q_1$ | 290 | 0 | $-\pi/2$ |
| 2 | $q_2$ | 0 | 270 | 0 |
| 3 | $q_3$ | 0 | 70 | $-\pi/2$ |
| 4 | $q_4$ | 302 | 0 | $\pi/2$ |
| 5 | $q_5$ | 0 | 0 | $-\pi/2$ |
| 6 | $q_6$ | 72 | 0 | 0 |

> **注意**: 实际实现中，你可以选择使用 DH 参数法，也可以根据机器人的几何结构直接构造变换矩阵。两种方法都是正确的，关键是最终结果一致。

使用标准 DH 参数法时，单个关节的变换矩阵为：

$$
T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \
0 & \sin\alpha_i & \cos\alpha_i & d_i \
0 & 0 & 0 & 1
\end{bmatrix}
$$

### 2.4 四元数 (Quaternion) 理论

四元数是描述三维旋转的一种紧凑且无奇异性 (Singularity-free) 的表示方法。一个单位四元数 $\boldsymbol{\xi}$ 定义为：

$$
\boldsymbol{\xi} = \begin{bmatrix} \xi_0 \ \tilde{\boldsymbol{\xi}} \end{bmatrix} = \begin{bmatrix} \xi_0 \ \xi_1 \ \xi_2 \ \xi_3 \end{bmatrix} \in \mathbb{R}^4, \quad \|\boldsymbol{\xi}\| = 1
$$

其中 $\xi_0$ 是标量部分 (scalar part)，$\tilde{\boldsymbol{\xi}} = [\xi_1, \xi_2, \xi_3]^T$ 是向量部分 (vector part)。

**四元数与旋转的关系**：绕单位轴 $\hat{n}$ 旋转角度 $\phi$ 对应的四元数为：

$$
\boldsymbol{\xi} = \begin{bmatrix} \cos(\phi/2) \ \hat{n} \sin(\phi/2) \end{bmatrix}
$$

**四元数到旋转矩阵的转换**：

$$
C = (2\xi_0^2 - 1)I_{3\times3} + 2\xi_0 [\tilde{\boldsymbol{\xi}}]_\times + 2\tilde{\boldsymbol{\xi}}\tilde{\boldsymbol{\xi}}^T
$$

其中 $[\tilde{\boldsymbol{\xi}}]_\times$ 是向量 $\tilde{\boldsymbol{\xi}}$ 的反对称矩阵 (Skew-symmetric Matrix)：

$$
[\tilde{\boldsymbol{\xi}}]_\times = \begin{bmatrix} 0 & -\xi_3 & \xi_2 \ \xi_3 & 0 & -\xi_1 \ -\xi_2 & \xi_1 & 0 \end{bmatrix}
$$

**四元数乘法**：

$$
\boldsymbol{q} \otimes \boldsymbol{p} = M_l(\boldsymbol{q}) \cdot \boldsymbol{p}
$$

其中左乘矩阵 $M_l(\boldsymbol{q})$ 为：

$$
M_l(\boldsymbol{q}) = \begin{bmatrix}
q_0 & -q_1 & -q_2 & -q_3 \
q_1 & q_0 & -q_3 & q_2 \
q_2 & q_3 & q_0 & -q_1 \
q_3 & -q_2 & q_1 & q_0
\end{bmatrix}
$$

**用四元数旋转向量**：

将向量 $v$ 用四元数 $\boldsymbol{\xi}$ 旋转：

$$
v_B = \boldsymbol{\xi} \otimes p_v \otimes \boldsymbol{\xi}^*
$$

其中 $p_v = [0; \; v]$ 是纯四元数 (pure quaternion)，$\boldsymbol{\xi}^* = [\xi_0; \; -\tilde{\boldsymbol{\xi}}]$ 是共轭四元数 (conjugate quaternion)。

---

## 3. 需要实现的函数 (Functions to Implement)

### 3.1 getTransformI0() — 惯性系到基座系的变换

**功能**: 返回从惯性坐标系 $I$ 到机器人基座坐标系 $0$ 的 $4 \times 4$ 齐次变换矩阵 $T_{I0}$。

**说明**: 通常情况下，如果机器人基座直接放置在世界坐标原点且无旋转，则 $T_{I0}$ 为单位矩阵。但在某些配置中，基座可能有偏移或旋转。

```matlab
function T_I0 = getTransformI0()
% getTransformI0() 返回惯性系到基座系的齐次变换矩阵
%
% 输出:
%   T_I0 : 4x4 齐次变换矩阵 (Homogeneous Transformation Matrix)

    % 如果基座与惯性系重合:
    T_I0 = eye(4);

    % 如果基座有偏移，例如:
    % T_I0 = [eye(3), [x; y; z]; 0 0 0 1];
end
```

### 3.2 getTransform6E() — 第6关节到末端执行器的变换

**功能**: 返回从第6关节坐标系到末端执行器坐标系 $E$ 的 $4 \times 4$ 齐次变换矩阵 $T_{6E}$。

**说明**: 这个变换描述了工具 (Tool) 相对于第6关节法兰 (Flange) 的安装位姿。

```matlab
function T_6E = getTransform6E()
% getTransform6E() 返回第6关节到末端执行器的齐次变换矩阵
%
% 输出:
%   T_6E : 4x4 齐次变换矩阵

    % 末端执行器相对于第6关节的变换
    % 根据工具的实际安装情况设定
    T_6E = eye(4);

    % 如果工具有偏移，例如沿 z 轴偏移 d_tool:
    % T_6E = [eye(3), [0; 0; d_tool]; 0 0 0 1];
end
```

### 3.3 jointToTransform01(q) ~ jointToTransform56(q) — 相邻关节间的变换

**功能**: 计算从第 $k-1$ 个关节坐标系到第 $k$ 个关节坐标系的 $4 \times 4$ 齐次变换矩阵。

**输入**: 关节角度向量 $q \in \mathbb{R}^{6 \times 1}$

**输出**: $4 \times 4$ 齐次变换矩阵 $T_{(k-1)k}$

每个函数只使用 $q$ 中对应的关节角度分量。例如 `jointToTransform01` 只使用 `q(1)`，`jointToTransform12` 只使用 `q(2)`，以此类推。

#### jointToTransform01 — 基座到关节1

```matlab
function T_01 = jointToTransform01(q)
% jointToTransform01(q) 计算从基座系0到关节1坐标系的变换
%
% 输入:
%   q : 6x1 关节角度向量 [rad]
%
% 输出:
%   T_01 : 4x4 齐次变换矩阵

    % 提取关节1的角度
    q1 = q(1);

    % 关节1绕 z 轴旋转，沿 z 轴有偏移 d1 = 290 mm
    % 使用 DH 参数: theta=q1, d=290, a=0, alpha=-pi/2
    T_01 = [cos(q1), -cos(-pi/2)*sin(q1),  sin(-pi/2)*sin(q1), 0;
            sin(q1),  cos(-pi/2)*cos(q1), -sin(-pi/2)*cos(q1), 0;
            0,        sin(-pi/2),           cos(-pi/2),          290;
            0,        0,                    0,                   1];

    % 简化后:
    % T_01 = [cos(q1),  0, -sin(q1), 0;
    %         sin(q1),  0,  cos(q1), 0;
    %         0,       -1,  0,       290;
    %         0,        0,  0,       1];
end
```

#### jointToTransform12 — 关节1到关节2

```matlab
function T_12 = jointToTransform12(q)
% jointToTransform12(q) 计算从关节1到关节2坐标系的变换
%
% 输入:
%   q : 6x1 关节角度向量 [rad]
%
% 输出:
%   T_12 : 4x4 齐次变换矩阵

    q2 = q(2);

    % DH 参数: theta=q2, d=0, a=270, alpha=0
    T_12 = [cos(q2), -sin(q2), 0, 270*cos(q2);
            sin(q2),  cos(q2), 0, 270*sin(q2);
            0,        0,       1, 0;
            0,        0,       0, 1];
end
```

#### jointToTransform23 — 关节2到关节3

```matlab
function T_23 = jointToTransform23(q)
% jointToTransform23(q) 计算从关节2到关节3坐标系的变换

    q3 = q(3);

    % DH 参数: theta=q3, d=0, a=70, alpha=-pi/2
    T_23 = [cos(q3),  0, -sin(q3), 70*cos(q3);
            sin(q3),  0,  cos(q3), 70*sin(q3);
            0,       -1,  0,       0;
            0,        0,  0,       1];
end
```

#### jointToTransform34 — 关节3到关节4

```matlab
function T_34 = jointToTransform34(q)
% jointToTransform34(q) 计算从关节3到关节4坐标系的变换

    q4 = q(4);

    % DH 参数: theta=q4, d=302, a=0, alpha=pi/2
    T_34 = [cos(q4), 0,  sin(q4), 0;
            sin(q4), 0, -cos(q4), 0;
            0,       1,  0,       302;
            0,       0,  0,       1];
end
```

#### jointToTransform45 — 关节4到关节5

```matlab
function T_45 = jointToTransform45(q)
% jointToTransform45(q) 计算从关节4到关节5坐标系的变换

    q5 = q(5);

    % DH 参数: theta=q5, d=0, a=0, alpha=-pi/2
    T_45 = [cos(q5),  0, -sin(q5), 0;
            sin(q5),  0,  cos(q5), 0;
            0,       -1,  0,       0;
            0,        0,  0,       1];
end
```

#### jointToTransform56 — 关节5到关节6

```matlab
function T_56 = jointToTransform56(q)
% jointToTransform56(q) 计算从关节5到关节6坐标系的变换

    q6 = q(6);

    % DH 参数: theta=q6, d=72, a=0, alpha=0
    T_56 = [cos(q6), -sin(q6), 0, 0;
            sin(q6),  cos(q6), 0, 0;
            0,        0,       1, 72;
            0,        0,       0, 1];
end
```

> **实现提示**: 构造变换矩阵时，务必注意单位一致性。DH 参数表中的 $d_i$ 和 $a_i$ 单位为 mm，确保在整个实现中保持统一。另外，`sin` 和 `cos` 函数的输入为弧度 (radians)。

### 3.4 jointToRotMat(q) — 关节角度到末端执行器旋转矩阵

**功能**: 给定关节角度向量 $q$，通过链式乘法计算末端执行器在惯性系中的旋转矩阵 $C_{IE}$。

**核心思路**: 先计算总的齐次变换矩阵 $T_{IE}$，再提取其中的旋转部分。

```matlab
function C_IE = jointToRotMat(q)
% jointToRotMat(q) 计算末端执行器相对于惯性系的旋转矩阵
%
% 输入:
%   q : 6x1 关节角度向量 [rad]
%
% 输出:
%   C_IE : 3x3 旋转矩阵 (Rotation Matrix)

    % 链式乘法计算总变换
    T_IE = getTransformI0() * ...
           jointToTransform01(q) * ...
           jointToTransform12(q) * ...
           jointToTransform23(q) * ...
           jointToTransform34(q) * ...
           jointToTransform45(q) * ...
           jointToTransform56(q) * ...
           getTransform6E();

    % 提取旋转矩阵 (左上角 3x3 子矩阵)
    C_IE = T_IE(1:3, 1:3);
end
```

### 3.5 jointToPosition(q) — 关节角度到末端执行器位置

**功能**: 给定关节角度向量 $q$，计算末端执行器在惯性系中的位置向量 ${}^I r_{IE}$。

```matlab
function I_r_IE = jointToPosition(q)
% jointToPosition(q) 计算末端执行器在惯性系中的位置
%
% 输入:
%   q : 6x1 关节角度向量 [rad]
%
% 输出:
%   I_r_IE : 3x1 位置向量 [mm]

    % 链式乘法计算总变换
    T_IE = getTransformI0() * ...
           jointToTransform01(q) * ...
           jointToTransform12(q) * ...
           jointToTransform23(q) * ...
           jointToTransform34(q) * ...
           jointToTransform45(q) * ...
           jointToTransform56(q) * ...
           getTransform6E();

    % 提取位置向量 (第4列的前3个元素)
    I_r_IE = T_IE(1:3, 4);
end
```

> **优化提示**: `jointToRotMat` 和 `jointToPosition` 都需要计算 $T_{IE}$。在实际项目中，可以编写一个辅助函数来避免重复计算。但在本实验中，为了保持函数接口的独立性，每个函数单独计算即可。


### 3.6 jointToQuat(q) — 关节角度到末端执行器四元数

**功能**: 给定关节角度向量 $q$，计算末端执行器姿态对应的单位四元数 $\boldsymbol{\xi} \in \mathbb{R}^{4 \times 1}$。

**核心思路**: 先通过 `jointToRotMat(q)` 获取旋转矩阵 $C_{IE}$，再调用 `rotMatToQuat(C)` 将其转换为四元数。

```matlab
function xi = jointToQuat(q)
% jointToQuat(q) 计算末端执行器姿态的四元数表示
%
% 输入:
%   q : 6x1 关节角度向量 [rad]
%
% 输出:
%   xi : 4x1 单位四元数 [xi0; xi1; xi2; xi3]
%        其中 xi0 为标量部分，[xi1; xi2; xi3] 为向量部分

    % 先获取旋转矩阵
    C_IE = jointToRotMat(q);

    % 再将旋转矩阵转换为四元数
    xi = rotMatToQuat(C_IE);
end
```

### 3.7 quatToRotMat(q) — 四元数到旋转矩阵

**功能**: 将单位四元数 $\boldsymbol{\xi}$ 转换为 $3 \times 3$ 旋转矩阵 $C$。

**公式**:

$$
C = (2\xi_0^2 - 1)I_{3 \times 3} + 2\xi_0 [\tilde{\boldsymbol{\xi}}]_\times + 2\tilde{\boldsymbol{\xi}}\tilde{\boldsymbol{\xi}}^T
$$

```matlab
function C = quatToRotMat(q)
% quatToRotMat(q) 将四元数转换为旋转矩阵
%
% 输入:
%   q : 4x1 单位四元数 [q0; q1; q2; q3]
%
% 输出:
%   C : 3x3 旋转矩阵

    % 提取标量部分和向量部分
    q0 = q(1);
    q_vec = q(2:4);  % 3x1 向量部分

    % 构造向量部分的反对称矩阵 (Skew-symmetric Matrix)
    q1 = q_vec(1);
    q2 = q_vec(2);
    q3 = q_vec(3);

    q_skew = [ 0,  -q3,  q2;
               q3,  0,  -q1;
              -q2,  q1,  0];

    % 应用转换公式
    C = (2*q0^2 - 1) * eye(3) + 2*q0 * q_skew + 2 * (q_vec * q_vec');
end
```

**验证方法**: 对于任意单位四元数，转换得到的旋转矩阵应满足：
- $C^T C = I_{3 \times 3}$ (正交性)
- $\det(C) = 1$ (右手系)

### 3.8 rotMatToQuat(C) — 旋转矩阵到四元数 (Shepperd 方法)

**功能**: 将 $3 \times 3$ 旋转矩阵 $C$ 转换为单位四元数 $\boldsymbol{\xi}$。

**关键**: 使用 **Shepperd 方法** 避免数值问题。直接从旋转矩阵的迹 (trace) 计算四元数时，当旋转角度接近 $\pi$ 时会出现数值不稳定。Shepperd 方法通过选择最大的分量来计算，确保数值稳定性。

**算法步骤**:

1. 计算四个候选值：$\text{tr}(C)$, $C_{11}$, $C_{22}$, $C_{33}$
2. 选择最大值对应的分量作为主元 (pivot)
3. 根据主元计算其余三个分量

```matlab
function q = rotMatToQuat(C)
% rotMatToQuat(C) 将旋转矩阵转换为四元数 (Shepperd方法)
%
% 输入:
%   C : 3x3 旋转矩阵
%
% 输出:
%   q : 4x1 单位四元数 [q0; q1; q2; q3]

    % 计算矩阵的迹
    tr = trace(C);

    % Shepperd 方法: 选择数值最稳定的计算路径
    % 比较 tr, C(1,1), C(2,2), C(3,3) 找到最大值
    vals = [tr, C(1,1), C(2,2), C(3,3)];
    [~, idx] = max(vals);

    switch idx
        case 1  % tr 最大 -> 用 q0 作为主元
            q0 = 0.5 * sqrt(1 + tr);
            q1 = (C(3,2) - C(2,3)) / (4 * q0);
            q2 = (C(1,3) - C(3,1)) / (4 * q0);
            q3 = (C(2,1) - C(1,2)) / (4 * q0);

        case 2  % C(1,1) 最大 -> 用 q1 作为主元
            q1 = 0.5 * sqrt(1 + 2*C(1,1) - tr);
            q0 = (C(3,2) - C(2,3)) / (4 * q1);
            q2 = (C(2,1) + C(1,2)) / (4 * q1);
            q3 = (C(1,3) + C(3,1)) / (4 * q1);

        case 3  % C(2,2) 最大 -> 用 q2 作为主元
            q2 = 0.5 * sqrt(1 + 2*C(2,2) - tr);
            q0 = (C(1,3) - C(3,1)) / (4 * q2);
            q1 = (C(2,1) + C(1,2)) / (4 * q2);
            q3 = (C(3,2) + C(2,3)) / (4 * q2);

        case 4  % C(3,3) 最大 -> 用 q3 作为主元
            q3 = 0.5 * sqrt(1 + 2*C(3,3) - tr);
            q0 = (C(2,1) - C(1,2)) / (4 * q3);
            q1 = (C(1,3) + C(3,1)) / (4 * q3);
            q2 = (C(3,2) + C(2,3)) / (4 * q3);
    end

    q = [q0; q1; q2; q3];

    % 确保 q0 >= 0 (四元数的双覆盖性: q 和 -q 表示同一旋转)
    if q(1) < 0
        q = -q;
    end
end
```

> **为什么使用 Shepperd 方法？** 考虑一个绕 z 轴旋转 $\pi$ 的情况，此时 $\text{tr}(C) = -1$，直接用 $q_0 = \frac{1}{2}\sqrt{1 + \text{tr}(C)}$ 会得到 $q_0 = 0$，导致后续除以 $4q_0$ 时出现除零错误。Shepperd 方法通过选择最大分量避免了这个问题。


### 3.9 quatMult(q1, q2) — 四元数乘法

**功能**: 计算两个四元数的 Hamilton 乘积 $\boldsymbol{q}_1 \otimes \boldsymbol{q}_2$。

**公式**: $\boldsymbol{q} \otimes \boldsymbol{p} = M_l(\boldsymbol{q}) \cdot \boldsymbol{p}$

```matlab
function q_result = quatMult(q, p)
% quatMult(q, p) 计算四元数乘法 q ⊗ p
%
% 输入:
%   q : 4x1 四元数
%   p : 4x1 四元数
%
% 输出:
%   q_result : 4x1 四元数乘积

    % 提取 q 的各分量
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

    % 构造左乘矩阵 M_l(q)
    M_l = [q0, -q1, -q2, -q3;
           q1,  q0, -q3,  q2;
           q2,  q3,  q0, -q1;
           q3, -q2,  q1,  q0];

    % 四元数乘法
    q_result = M_l * p;
end
```

**四元数乘法的性质**:
- **不满足交换律**: $\boldsymbol{q} \otimes \boldsymbol{p} \neq \boldsymbol{p} \otimes \boldsymbol{q}$ (一般情况下)
- **满足结合律**: $(\boldsymbol{q} \otimes \boldsymbol{p}) \otimes \boldsymbol{r} = \boldsymbol{q} \otimes (\boldsymbol{p} \otimes \boldsymbol{r})$
- **单位元**: $\boldsymbol{e} = [1; 0; 0; 0]$
- **逆元**: $\boldsymbol{q}^{-1} = \boldsymbol{q}^* / \|\boldsymbol{q}\|^2$，对于单位四元数 $\boldsymbol{q}^{-1} = \boldsymbol{q}^*$

### 3.10 rotVecWithQuat(q, v) — 用四元数旋转向量

**功能**: 使用四元数 $\boldsymbol{\xi}$ 将向量 $v$ 从一个坐标系旋转到另一个坐标系。

**公式**:

$$
v_B = \boldsymbol{\xi} \otimes p_v \otimes \boldsymbol{\xi}^*
$$

其中 $p_v = [0; \; v]$ 是纯四元数，$\boldsymbol{\xi}^* = [\xi_0; \; -\tilde{\boldsymbol{\xi}}]$ 是共轭四元数。

```matlab
function v_rot = rotVecWithQuat(q, v)
% rotVecWithQuat(q, v) 使用四元数旋转一个三维向量
%
% 输入:
%   q : 4x1 单位四元数
%   v : 3x1 向量
%
% 输出:
%   v_rot : 3x1 旋转后的向量

    % 将向量 v 构造为纯四元数 p_v = [0; v]
    p_v = [0; v];

    % 计算共轭四元数 q* = [q0; -q_vec]
    q_conj = [q(1); -q(2:4)];

    % 执行四元数旋转: q ⊗ p_v ⊗ q*
    result = quatMult(quatMult(q, p_v), q_conj);

    % 提取旋转后的向量 (忽略标量部分，理论上为0)
    v_rot = result(2:4);
end
```

> **注意**: 结果四元数的标量部分理论上应为零。如果不为零（由于浮点误差），其值应非常小（< 1e-10）。这可以作为一个简单的正确性检查。

---

## 4. 测试方法 (Testing)

### 4.1 自动评估

运行 `evaluate_problems.m` 脚本可以自动测试所有实现的函数：

```matlab
>> init_workspace    % 首先初始化工作空间
>> evaluate_problems % 运行自动评估
```

**评估机制**:

- 对每个函数进行 **1000 次随机测试**
- 每次测试使用随机生成的关节角度 $q \in [-\pi, \pi]^6$
- 将你的实现结果与 `solutions/` 文件夹中的参考解进行对比
- **容差 (Tolerance)**: $1 \times 10^{-6}$
- 如果所有测试的最大误差小于容差，则该函数通过测试

**输出示例**:

```
Testing getTransformI0...        PASSED (max error: 0.00e+00)
Testing getTransform6E...        PASSED (max error: 0.00e+00)
Testing jointToTransform01...    PASSED (max error: 2.22e-16)
Testing jointToTransform12...    PASSED (max error: 1.11e-16)
Testing jointToTransform23...    PASSED (max error: 2.22e-16)
Testing jointToTransform34...    PASSED (max error: 2.22e-16)
Testing jointToTransform45...    PASSED (max error: 2.22e-16)
Testing jointToTransform56...    PASSED (max error: 1.11e-16)
Testing jointToRotMat...         PASSED (max error: 3.55e-15)
Testing jointToPosition...       PASSED (max error: 2.84e-14)
Testing jointToQuat...           PASSED (max error: 4.44e-16)
Testing quatToRotMat...          PASSED (max error: 4.44e-16)
Testing rotMatToQuat...          PASSED (max error: 4.44e-16)
Testing quatMult...              PASSED (max error: 4.44e-16)
Testing rotVecWithQuat...        PASSED (max error: 8.88e-16)

All tests passed!
```


### 4.2 手动验证

除了自动评估，你还可以手动验证一些特殊情况：

```matlab
% 零位姿态 (Home Position): 所有关节角度为0
q_home = zeros(6, 1);
T_home = getTransformI0() * jointToTransform01(q_home) * ...
         jointToTransform12(q_home) * jointToTransform23(q_home) * ...
         jointToTransform34(q_home) * jointToTransform45(q_home) * ...
         jointToTransform56(q_home) * getTransform6E();
disp('Home position:');
disp(T_home(1:3, 4));  % 应该得到一个合理的末端位置

% 验证旋转矩阵的正交性
C = jointToRotMat(q_home);
disp('C^T * C (should be identity):');
disp(C' * C);
disp('det(C) (should be 1):');
disp(det(C));

% 验证四元数的单位性
xi = jointToQuat(q_home);
disp('||xi|| (should be 1):');
disp(norm(xi));

% 验证四元数与旋转矩阵的一致性
C_from_quat = quatToRotMat(xi);
disp('Max difference between C and quatToRotMat(jointToQuat(q)):');
disp(max(abs(C(:) - C_from_quat(:))));
```

### 4.3 常见错误排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 位置偏差很大 | DH 参数中的 $d$ 或 $a$ 值错误 | 仔细核对 DH 参数表 |
| 旋转矩阵不正交 | 矩阵构造有误 | 检查 $\sin$/$\cos$ 的位置和符号 |
| 四元数范数不为1 | `rotMatToQuat` 实现有误 | 检查 Shepperd 方法的公式 |
| 部分角度正确部分错误 | 某个 `jointToTransform` 有误 | 逐一测试每个变换函数 |
| 误差略大于容差 | 浮点精度累积 | 检查是否有不必要的中间计算 |

---

## 5. 可视化 (Visualization)

### 5.1 加载 3D 模型

`loadviz.m` 脚本加载 ABB IRB 120 的 STL 模型文件，并根据你实现的正运动学函数渲染机器人的 3D 模型：

```matlab
>> init_workspace
>> loadviz          % 加载可视化环境
```

可视化功能允许你：

- 直观地验证各关节变换矩阵是否正确
- 通过拖动滑块改变关节角度，观察机器人运动
- 检查末端执行器的位置和姿态是否合理
- 发现明显的坐标系方向错误

### 5.2 可视化调试技巧

如果可视化结果看起来不对，可以尝试以下方法：

```matlab
% 逐步可视化: 只显示前 N 个关节的变换
% 这有助于定位哪个关节的变换出了问题

% 例如，只计算到关节3的变换
q_test = [pi/4; pi/6; -pi/3; 0; 0; 0];
T_03 = getTransformI0() * jointToTransform01(q_test) * ...
       jointToTransform12(q_test) * jointToTransform23(q_test);
disp('Position at joint 3:');
disp(T_03(1:3, 4));
```

### 5.3 坐标系约定

在可视化中，坐标轴的颜色约定通常为：

- **红色 (Red)**: X 轴
- **绿色 (Green)**: Y 轴
- **蓝色 (Blue)**: Z 轴

确保你的变换矩阵产生的坐标系方向与可视化中显示的一致。

---

## 6. 实现建议与最佳实践

### 6.1 推荐实现顺序

1. `getTransformI0()` 和 `getTransform6E()` — 最简单，先完成
2. `jointToTransform01()` ~ `jointToTransform56()` — 核心部分，逐一实现并测试
3. `jointToRotMat(q)` 和 `jointToPosition(q)` — 调用上面的函数，链式乘法
4. `quatToRotMat(q)` — 直接套公式
5. `rotMatToQuat(C)` — Shepperd 方法，注意数值稳定性
6. `jointToQuat(q)` — 组合调用
7. `quatMult(q1, q2)` — 构造左乘矩阵
8. `rotVecWithQuat(q, v)` — 调用 `quatMult`

### 6.2 调试技巧

```matlab
% 使用 MATLAB 的 Symbolic Toolbox 验证公式
syms q1 q2 q3 q4 q5 q6 real
q_sym = [q1; q2; q3; q4; q5; q6];

% 符号计算变换矩阵，检查结构是否正确
T01_sym = jointToTransform01(q_sym);
disp(simplify(T01_sym));

% 验证旋转矩阵的正交性 (符号验证)
R = T01_sym(1:3, 1:3);
disp(simplify(R' * R));  % 应该化简为 eye(3)
```

### 6.3 常用 MATLAB 函数

```matlab
eye(n)          % n x n 单位矩阵
zeros(m, n)     % m x n 零矩阵
trace(A)        % 矩阵的迹
det(A)          % 矩阵的行列式
norm(v)         % 向量的范数
cross(a, b)     % 向量叉积
dot(a, b)       % 向量点积
atan2(y, x)     % 四象限反正切
```

---

## 7. 总结

本实验通过实现 ABB IRB 120 的正运动学，帮助你掌握以下核心概念：

- **齐次变换矩阵**的构造与链式乘法
- **DH 参数法**在工业机器人建模中的应用
- **四元数**作为旋转表示的优势及其与旋转矩阵的相互转换
- **Shepperd 方法**在数值计算中的重要性
- MATLAB 在机器人学中的实际应用

完成本实验后，你将具备为任意串联机器人实现正运动学的能力，这是后续逆运动学 (Inverse Kinematics)、雅可比矩阵 (Jacobian) 和动力学 (Dynamics) 学习的基础。

---

> **参考资料**:
> - ETH Zurich, Robot Dynamics 课程讲义
> - Siciliano, B. et al., *Robotics: Modelling, Planning and Control*, Springer
> - Shepperd, S.W., "Quaternion from Rotation Matrix", *Journal of Guidance and Control*, 1978
> - ABB IRB 120 产品数据手册
