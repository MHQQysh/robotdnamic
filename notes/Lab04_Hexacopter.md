# Lab 04: 六旋翼飞行器的建模与控制 (Modeling and Control of Hexacopter)

> ETH Zurich - Robot Dynamics 课程实验指导
> Exercise 4: Hexacopter Modeling and Control

---

## 1. 实验概述 (Lab Overview)

### 1.1 实验目标

本实验的核心目标是对六旋翼飞行器 (hexacopter) 进行完整的动力学建模，并设计分层控制器实现稳定飞行。具体包括：

- 建立六旋翼飞行器的刚体动力学模型
- 推导旋翼的推力与反扭矩模型
- 设计内外环控制架构（姿态控制 + 位置控制）
- 实现推力分配 (thrust allocation) 算法
- 在 Simulink 中完成仿真验证

### 1.2 工具与环境

- **MATLAB/Simulink**: 主要建模与仿真平台
- **Simulink 模型文件**: `hexacopter.slx`
- **辅助脚本**: `init_params.m`（参数初始化）、`plot_results.m`（结果可视化）

### 1.3 六旋翼飞行器简介

六旋翼飞行器 (hexacopter) 相比四旋翼 (quadrotor) 具有以下优势：

- **冗余性 (redundancy)**: 6 个执行器控制 4 个自由度，具有执行器冗余
- **载荷能力**: 更大的总推力，适合搭载更重的传感器或载荷
- **容错性 (fault tolerance)**: 单个旋翼失效时仍可维持飞行

---

## 2. 六旋翼建模 (Hexacopter Modeling)

### 2.1 坐标系定义 (Coordinate Frames)

建模需要定义两个基本坐标系：

**惯性坐标系 (Inertial Frame) $\{I\}$**:
- 固定在地面，满足牛顿力学的惯性参考系要求
- 坐标轴: $x_I$ 指向北，$y_I$ 指向东，$z_I$ 指向下（NED 约定）
- 也可采用 $z_I$ 指向上的 ENU 约定，本实验采用 **$z$ 轴向上** 的约定

**机体坐标系 (Body Frame) $\{B\}$**:
- 原点固定在飞行器质心 (center of mass, CoM)
- $x_B$ 指向飞行器前方
- $y_B$ 指向飞行器右方
- $z_B$ 指向飞行器上方（与推力方向一致）

**姿态表示 (Attitude Representation)**:

采用 ZYX 欧拉角 (Euler angles) 描述机体系相对于惯性系的姿态：

$$\boldsymbol{\Phi} = \begin{bmatrix} \phi \ \theta \ \psi \end{bmatrix}$$

其中：
- $\phi$ — 横滚角 (roll)，绕 $x$ 轴旋转
- $\theta$ — 俯仰角 (pitch)，绕 $y$ 轴旋转
- $\psi$ — 偏航角 (yaw)，绕 $z$ 轴旋转

从惯性系到机体系的旋转矩阵 (rotation matrix) 为：

$$\mathbf{C}_{IB} = \mathbf{R}_z(\psi) \, \mathbf{R}_y(\theta) \, \mathbf{R}_x(\phi)$$

展开为：

$$\mathbf{C}_{IB} = \begin{bmatrix} c\psi c\theta & c\psi s\theta s\phi - s\psi c\phi & c\psi s\theta c\phi + s\psi s\phi \ s\psi c\theta & s\psi s\theta s\phi + c\psi c\phi & s\psi s\theta c\phi - c\psi s\phi \ -s\theta & c\theta s\phi & c\theta c\phi \end{bmatrix}$$

其中 $c(\cdot) = \cos(\cdot)$，$s(\cdot) = \sin(\cdot)$。


### 2.2 广义坐标 (Generalized Coordinates)

六旋翼飞行器作为空间中的自由刚体，具有 6 个自由度 (degrees of freedom, DoF)。广义坐标向量定义为：

$$\mathbf{q} = \begin{bmatrix} \mathbf{r}_{IB} \ \boldsymbol{\Phi} \end{bmatrix} \in \mathbb{R}^6$$

其中：

- **位置向量 (position vector)**:

$$\mathbf{r}_{IB} = \begin{bmatrix} x \ y \ z \end{bmatrix}_I$$

表示机体质心在惯性系 $\{I\}$ 中的位置。

- **姿态向量 (attitude vector)**:

$$\boldsymbol{\Phi} = \begin{bmatrix} \phi \ \theta \ \psi \end{bmatrix}$$

表示机体系 $\{B\}$ 相对于惯性系 $\{I\}$ 的欧拉角。

对应的广义速度为：

$$\dot{\mathbf{q}} = \begin{bmatrix} \dot{\mathbf{r}}_{IB} \ \dot{\boldsymbol{\Phi}} \end{bmatrix} = \begin{bmatrix} \mathbf{v}_I \ \dot{\boldsymbol{\Phi}} \end{bmatrix}$$

> **注意**: 欧拉角的时间导数 $\dot{\boldsymbol{\Phi}}$ 与机体角速度 $\boldsymbol{\omega}_B$ 之间的关系为：
>
> $$\boldsymbol{\omega}_B = \mathbf{E}(\boldsymbol{\Phi}) \, \dot{\boldsymbol{\Phi}}$$
>
> 其中 $\mathbf{E}(\boldsymbol{\Phi})$ 是欧拉角速率到角速度的映射矩阵：
>
> $$\mathbf{E}(\boldsymbol{\Phi}) = \begin{bmatrix} 1 & 0 & -s\theta \ 0 & c\phi & c\theta s\phi \ 0 & -s\phi & c\theta c\phi \end{bmatrix}$$

### 2.3 旋翼模型 (Rotor Model)

#### 旋翼布局

六旋翼飞行器有 6 个旋翼，均匀分布在以质心为圆心、半径为 $L$ 的圆上。第 $i$ 个旋翼的位置角为：

$$\alpha_i = \frac{(i-1) \cdot 2\pi}{6}, \quad i = 1, 2, \ldots, 6$$

第 $i$ 个旋翼在机体系中的位置向量为：

$$\mathbf{r}_i^B = L \begin{bmatrix} \cos\alpha_i \ \sin\alpha_i \ 0 \end{bmatrix}$$

相邻旋翼的旋转方向交替排列：

| 旋翼编号 | 旋转方向 | 符号 $\sigma_i$ |
|:--------:|:--------:|:---------------:|
| 1 | 逆时针 (CCW) | $+1$ |
| 2 | 顺时针 (CW) | $-1$ |
| 3 | 逆时针 (CCW) | $+1$ |
| 4 | 顺时针 (CW) | $-1$ |
| 5 | 逆时针 (CCW) | $+1$ |
| 6 | 顺时针 (CW) | $-1$ |

#### 推力模型 (Thrust Model)

每个旋翼产生的推力与转速的平方成正比，方向沿机体系 $z_B$ 轴：

$$F_i = k_F \cdot \omega_i^2$$

其中：
- $k_F$ — 推力系数 (thrust coefficient)，单位 $[\text{N} \cdot \text{s}^2/\text{rad}^2]$
- $\omega_i$ — 第 $i$ 个旋翼的角速度，单位 $[\text{rad/s}]$

推力向量在机体系中表示为：

$$\mathbf{F}_i^B = F_i \cdot \mathbf{e}_{z_B} = k_F \omega_i^2 \begin{bmatrix} 0 \ 0 \ 1 \end{bmatrix}$$

#### 反扭矩模型 (Drag Torque Model)

每个旋翼由于空气阻力产生反扭矩 (reaction torque)，方向与旋转方向相反：

$$M_i = \sigma_i \cdot k_M \cdot \omega_i^2$$

其中：
- $k_M$ — 力矩系数 (moment coefficient)，单位 $[\text{N} \cdot \text{m} \cdot \text{s}^2/\text{rad}^2]$
- $\sigma_i \in \{+1, -1\}$ — 旋转方向符号

反扭矩向量在机体系中表示为：

$$\mathbf{M}_i^B = \sigma_i \, k_M \, \omega_i^2 \, \mathbf{e}_{z_B}$$


### 2.4 力和力矩合成 (Force and Torque Aggregation)

#### 总推力 (Total Thrust)

所有旋翼产生的总推力在机体系中为：

$$\mathbf{F}_{\text{total}}^B = \sum_{i=1}^{6} F_i \cdot \mathbf{e}_{z_B} = \left( \sum_{i=1}^{6} k_F \omega_i^2 \right) \mathbf{e}_{z_B}$$

定义标量总推力为：

$$F = \sum_{i=1}^{6} k_F \omega_i^2$$

#### 总力矩 (Total Torque)

每个旋翼对质心产生的力矩包含两部分：

1. **推力产生的力矩**: $\mathbf{r}_i^B \times F_i \mathbf{e}_{z_B}$
2. **反扭矩**: $\sigma_i k_M \omega_i^2 \mathbf{e}_{z_B}$

总力矩在机体系中为：

$$\mathbf{M}_{\text{total}}^B = \sum_{i=1}^{6} \left( \mathbf{r}_i^B \times F_i \mathbf{e}_{z_B} + \sigma_i k_M \omega_i^2 \mathbf{e}_{z_B} \right)$$

展开各分量：

$$M_x = \sum_{i=1}^{6} k_F \omega_i^2 \cdot L \sin\alpha_i$$

$$M_y = -\sum_{i=1}^{6} k_F \omega_i^2 \cdot L \cos\alpha_i$$

$$M_z = \sum_{i=1}^{6} \sigma_i \, k_M \, \omega_i^2$$

#### 分配矩阵 (Allocation Matrix)

将力和力矩的关系写成矩阵形式：

$$\begin{bmatrix} F \ M_x \ M_y \ M_z \end{bmatrix} = \mathbf{A} \begin{bmatrix} \omega_1^2 \ \omega_2^2 \ \omega_3^2 \ \omega_4^2 \ \omega_5^2 \ \omega_6^2 \end{bmatrix}$$

其中分配矩阵 $\mathbf{A} \in \mathbb{R}^{4 \times 6}$ 为：

$$\mathbf{A} = \begin{bmatrix} k_F & k_F & k_F & k_F & k_F & k_F \ k_F L \sin\alpha_1 & k_F L \sin\alpha_2 & k_F L \sin\alpha_3 & k_F L \sin\alpha_4 & k_F L \sin\alpha_5 & k_F L \sin\alpha_6 \ -k_F L \cos\alpha_1 & -k_F L \cos\alpha_2 & -k_F L \cos\alpha_3 & -k_F L \cos\alpha_4 & -k_F L \cos\alpha_5 & -k_F L \cos\alpha_6 \ \sigma_1 k_M & \sigma_2 k_M & \sigma_3 k_M & \sigma_4 k_M & \sigma_5 k_M & \sigma_6 k_M \end{bmatrix}$$

代入 $\alpha_i$ 和 $\sigma_i$ 的具体值，可以得到数值形式的分配矩阵。

> **关键性质**: 矩阵 $\mathbf{A}$ 的维度为 $4 \times 6$，这意味着系统具有 **执行器冗余 (actuator redundancy)**——6 个输入控制 4 个输出。这为推力分配提供了额外的自由度。

---

## 3. 运动方程 (Equations of Motion)

### 3.1 平动方程 (Translational Dynamics)

根据牛顿第二定律，飞行器质心在惯性系中的运动方程为：

$$m \ddot{\mathbf{r}}_{IB} = \mathbf{C}_{IB} \mathbf{F}_{\text{total}}^B + m g \mathbf{e}_{z_I}$$

展开为：

$$m \begin{bmatrix} \ddot{x} \ \ddot{y} \ \ddot{z} \end{bmatrix}_I = \mathbf{C}_{IB} \begin{bmatrix} 0 \ 0 \ F \end{bmatrix} + \begin{bmatrix} 0 \ 0 \ -mg \end{bmatrix}$$

> **注意**: 此处采用 $z$ 轴向上的约定，因此重力加速度为 $-g \mathbf{e}_{z_I}$。

将旋转矩阵代入，得到各分量的表达式：

$$m\ddot{x} = F(\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi)$$

$$m\ddot{y} = F(\sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi)$$

$$m\ddot{z} = F \cos\theta \cos\phi - mg$$

### 3.2 转动方程 (Rotational Dynamics)

根据欧拉方程 (Euler's equation)，机体系中的转动方程为：

$$\mathbf{I}_B \dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (\mathbf{I}_B \boldsymbol{\omega}_B) = \mathbf{M}_{\text{total}}^B$$

其中：
- $\mathbf{I}_B$ — 机体系中的惯性张量 (inertia tensor)
- $\boldsymbol{\omega}_B$ — 机体系中的角速度向量
- $\dot{\boldsymbol{\omega}}_B$ — 机体系中的角加速度向量

对于对称的六旋翼飞行器，惯性张量为对角矩阵：

$$\mathbf{I}_B = \begin{bmatrix} I_{xx} & 0 & 0 \ 0 & I_{yy} & 0 \ 0 & 0 & I_{zz} \end{bmatrix}$$

展开各分量：

$$I_{xx} \dot{\omega}_x = M_x - (I_{zz} - I_{yy}) \omega_y \omega_z$$

$$I_{yy} \dot{\omega}_y = M_y - (I_{xx} - I_{zz}) \omega_x \omega_z$$

$$I_{zz} \dot{\omega}_z = M_z - (I_{yy} - I_{xx}) \omega_x \omega_y$$

### 3.3 运动学方程 (Kinematics)

旋转矩阵的时间导数满足：

$$\dot{\mathbf{C}}_{IB} = \mathbf{C}_{IB} \, \tilde{\boldsymbol{\omega}}_B$$

其中 $\tilde{\boldsymbol{\omega}}_B$ 是角速度的反对称矩阵 (skew-symmetric matrix)：

$$\tilde{\boldsymbol{\omega}}_B = \begin{bmatrix} 0 & -\omega_z & \omega_y \ \omega_z & 0 & -\omega_x \ -\omega_y & \omega_x & 0 \end{bmatrix}$$

等价地，用欧拉角表示的运动学关系为：

$$\dot{\boldsymbol{\Phi}} = \mathbf{E}^{-1}(\boldsymbol{\Phi}) \, \boldsymbol{\omega}_B$$

其中：

$$\mathbf{E}^{-1}(\boldsymbol{\Phi}) = \begin{bmatrix} 1 & \sin\phi \tan\theta & \cos\phi \tan\theta \ 0 & \cos\phi & -\sin\phi \ 0 & \sin\phi / \cos\theta & \cos\phi / \cos\theta \end{bmatrix}$$

> **奇异性警告**: 当 $\theta = \pm 90°$ 时，$\mathbf{E}^{-1}$ 存在奇异性 (gimbal lock)。在实际飞行中应避免大俯仰角。

### 3.4 状态空间形式 (State-Space Form)

定义状态向量：

$$\mathbf{x} = \begin{bmatrix} \mathbf{r}_{IB} \ \boldsymbol{\Phi} \ \dot{\mathbf{r}}_{IB} \ \boldsymbol{\omega}_B \end{bmatrix} \in \mathbb{R}^{12}$$

系统的状态方程为：

$$\dot{\mathbf{x}} = \begin{bmatrix} \dot{\mathbf{r}}_{IB} \ \mathbf{E}^{-1}(\boldsymbol{\Phi}) \boldsymbol{\omega}_B \ \frac{1}{m} \mathbf{C}_{IB} \mathbf{F}^B + g \mathbf{e}_z \ \mathbf{I}_B^{-1} \left( \mathbf{M}^B - \boldsymbol{\omega}_B \times \mathbf{I}_B \boldsymbol{\omega}_B \right) \end{bmatrix}$$


---

## 4. 控制设计 (Control Design)

### 4.1 控制架构 (Control Architecture)

六旋翼飞行器采用经典的 **分层控制架构 (cascaded control architecture)**，由外到内依次为：

```
位置参考 r_des ──► [位置控制器] ──► 期望推力 F_des + 期望姿态 Φ_des
                                          │
                                          ▼
                        [姿态控制器] ──► 期望力矩 M_des
                                          │
                                          ▼
                        [推力分配] ──► 各旋翼转速 ω_1 ... ω_6
```

这种分层设计的核心思想是 **时间尺度分离 (time-scale separation)**：

- **内环 (inner loop)**: 姿态控制，带宽高，响应快
- **外环 (outer loop)**: 位置控制，带宽低，响应慢

内环的响应速度远快于外环，因此在外环设计时可以假设姿态能够被快速跟踪。

### 4.2 姿态控制 (Attitude Control)

#### 误差定义

姿态误差和角速度误差分别定义为：

$$\mathbf{e}_{\Phi} = \boldsymbol{\Phi}_{\text{des}} - \boldsymbol{\Phi}$$

$$\mathbf{e}_{\omega} = \boldsymbol{\omega}_{\text{des}} - \boldsymbol{\omega}_B$$

#### PD 控制律

姿态控制器采用 PD (Proportional-Derivative) 控制律：

$$\mathbf{M}_{\text{des}} = \mathbf{K}_{p,\text{att}} \, \mathbf{e}_{\Phi} + \mathbf{K}_{d,\text{att}} \, \mathbf{e}_{\omega}$$

其中：
- $\mathbf{K}_{p,\text{att}} = \text{diag}(k_{p,\phi}, \, k_{p,\theta}, \, k_{p,\psi})$ — 比例增益矩阵
- $\mathbf{K}_{d,\text{att}} = \text{diag}(k_{d,\phi}, \, k_{d,\theta}, \, k_{d,\psi})$ — 微分增益矩阵

#### 增益选择

对于每个轴，闭环特征方程为：

$$s^2 + k_d s + k_p = 0$$

选择临界阻尼 (critically damped) 响应，令阻尼比 $\zeta = 1$：

$$k_p = \omega_n^2, \quad k_d = 2 \zeta \omega_n = 2\omega_n$$

其中 $\omega_n$ 是期望的自然频率 (natural frequency)。

#### MATLAB 实现参考

```matlab
function M_des = attitudeController(Phi, Phi_des, omega, omega_des, params)
    % 姿态误差
    e_Phi = Phi_des - Phi;
    % 角速度误差
    e_omega = omega_des - omega;
    % PD 控制律
    Kp = diag([params.kp_phi, params.kp_theta, params.kp_psi]);
    Kd = diag([params.kd_phi, params.kd_theta, params.kd_psi]);
    M_des = Kp * e_Phi + Kd * e_omega;
end
```

### 4.3 位置控制 (Position Control)

#### 误差定义

位置误差和速度误差分别定义为：

$$\mathbf{e}_r = \mathbf{r}_{\text{des}} - \mathbf{r}_{IB}$$

$$\mathbf{e}_v = \mathbf{v}_{\text{des}} - \dot{\mathbf{r}}_{IB}$$

#### PD 控制律

位置控制器计算期望加速度：

$$\mathbf{a}_{\text{des}} = \mathbf{K}_{p,\text{pos}} \, \mathbf{e}_r + \mathbf{K}_{d,\text{pos}} \, \mathbf{e}_v + g \mathbf{e}_z$$

其中：
- $\mathbf{K}_{p,\text{pos}} = \text{diag}(k_{p,x}, \, k_{p,y}, \, k_{p,z})$ — 位置比例增益
- $\mathbf{K}_{d,\text{pos}} = \text{diag}(k_{d,x}, \, k_{d,y}, \, k_{d,z})$ — 位置微分增益
- $g \mathbf{e}_z$ — 重力前馈补偿项 (gravity feedforward)

#### 从期望加速度提取推力和姿态

期望加速度向量 $\mathbf{a}_{\text{des}}$ 包含了推力大小和方向的信息。提取过程如下：

**Step 1: 计算期望推力大小**

$$F_{\text{des}} = m \| \mathbf{a}_{\text{des}} \|$$

或者更精确地，沿当前机体 $z$ 轴方向的投影：

$$F_{\text{des}} = m \, \mathbf{a}_{\text{des}}^T \mathbf{C}_{IB} \mathbf{e}_{z_B}$$

**Step 2: 提取期望姿态角**

给定期望偏航角 $\psi_{\text{des}}$（通常由上层规划给定），可以从期望加速度中提取期望的横滚角和俯仰角：

$$\phi_{\text{des}} = \arcsin\left( \frac{a_{\text{des},x} \sin\psi_{\text{des}} - a_{\text{des},y} \cos\psi_{\text{des}}}{\|\mathbf{a}_{\text{des}}\|} \right)$$

$$\theta_{\text{des}} = \arctan\left( \frac{a_{\text{des},x} \cos\psi_{\text{des}} + a_{\text{des},y} \sin\psi_{\text{des}}}{a_{\text{des},z}} \right)$$

#### MATLAB 实现参考

```matlab
function [F_des, Phi_des] = positionController(r, r_des, v, v_des, psi_des, params)
    % 位置和速度误差
    e_r = r_des - r;
    e_v = v_des - v;
    % 期望加速度 (含重力补偿)
    Kp = diag([params.kp_x, params.kp_y, params.kp_z]);
    Kd = diag([params.kd_x, params.kd_y, params.kd_z]);
    a_des = Kp * e_r + Kd * e_v + [0; 0; params.g];
    % 期望推力
    F_des = params.m * norm(a_des);
    % 期望姿态角
    phi_des = asin((a_des(1)*sin(psi_des) - a_des(2)*cos(psi_des)) / norm(a_des));
    theta_des = atan2(a_des(1)*cos(psi_des) + a_des(2)*sin(psi_des), a_des(3));
    Phi_des = [phi_des; theta_des; psi_des];
end
```


### 4.4 推力分配 (Thrust Allocation)

#### 问题描述

推力分配的目标是根据期望的总推力和力矩，求解各旋翼的转速。数学上表示为：

$$\mathbf{A} \, \mathbf{u} = \mathbf{b}$$

其中：

$$\mathbf{u} = \begin{bmatrix} \omega_1^2 \ \omega_2^2 \ \omega_3^2 \ \omega_4^2 \ \omega_5^2 \ \omega_6^2 \end{bmatrix} \in \mathbb{R}^6, \quad \mathbf{b} = \begin{bmatrix} F_{\text{des}} \ M_{x,\text{des}} \ M_{y,\text{des}} \ M_{z,\text{des}} \end{bmatrix} \in \mathbb{R}^4$$

#### 冗余性分析

由于 $\mathbf{A} \in \mathbb{R}^{4 \times 6}$，方程组是 **欠定的 (underdetermined)**：4 个方程，6 个未知数。这意味着存在无穷多组解，系统具有 2 个自由度的冗余。

#### 伪逆求解 (Pseudo-Inverse Solution)

最常用的方法是使用 Moore-Penrose 伪逆 (pseudo-inverse)，求最小二范数解：

$$\mathbf{u}^* = \mathbf{A}^{\dagger} \mathbf{b} = \mathbf{A}^T (\mathbf{A} \mathbf{A}^T)^{-1} \mathbf{b}$$

该解在所有满足约束的解中，使 $\|\mathbf{u}\|_2$ 最小，即各旋翼转速的平方和最小。这在物理上对应于 **最小能量分配 (minimum energy allocation)**。

#### 带约束的分配

实际中旋翼转速有上下界约束：

$$0 \leq \omega_i^2 \leq \omega_{\max}^2, \quad i = 1, \ldots, 6$$

如果伪逆解违反约束，需要使用带约束的优化方法：

$$\min_{\mathbf{u}} \|\mathbf{u}\|^2 \quad \text{s.t.} \quad \mathbf{A}\mathbf{u} = \mathbf{b}, \quad \mathbf{0} \leq \mathbf{u} \leq \omega_{\max}^2 \mathbf{1}$$

#### 从转速平方到转速

求解得到 $\mathbf{u} = [\omega_1^2, \ldots, \omega_6^2]^T$ 后，各旋翼的实际转速为：

$$\omega_i = \sqrt{\max(u_i, \, 0)}, \quad i = 1, \ldots, 6$$

取 $\max$ 操作是为了防止数值误差导致的负值。

#### MATLAB 实现参考

```matlab
function omega = thrustAllocation(F_des, M_des, params)
    % 构建分配矩阵
    A = buildAllocationMatrix(params);
    % 期望力和力矩向量
    b = [F_des; M_des];
    % 伪逆求解
    u = pinv(A) * b;
    % 饱和处理
    u = max(u, 0);
    u = min(u, params.omega_max^2);
    % 转速
    omega = sqrt(u);
end

function A = buildAllocationMatrix(params)
    kF = params.kF;
    kM = params.kM;
    L  = params.L;
    A = zeros(4, 6);
    for i = 1:6
        alpha_i = (i-1) * 2*pi / 6;
        sigma_i = (-1)^(i+1);  % 交替正反转
        A(1, i) = kF;
        A(2, i) = kF * L * sin(alpha_i);
        A(3, i) = -kF * L * cos(alpha_i);
        A(4, i) = sigma_i * kM;
    end
end
```

---

## 5. Simulink 仿真说明 (Simulink Simulation Guide)

### 5.1 模型结构 (Model Structure)

打开 `hexacopter.slx`，模型的整体结构如下：

```
┌─────────────────────────────────────────────────────────────────┐
│                      hexacopter.slx                             │
│                                                                 │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────┐          │
│  │ Trajectory│───►│  Position    │───►│   Attitude    │          │
│  │ Generator │    │  Controller  │    │   Controller  │          │
│  └──────────┘    └──────────────┘    └───────┬───────┘          │
│                                              │                  │
│                                              ▼                  │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────┐          │
│  │  Scope / │◄───│  Hexacopter  │◄───│    Thrust     │          │
│  │  Logging │    │  Dynamics    │    │   Allocation  │          │
│  └──────────┘    └──────────────┘    └───────────────┘          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

各子系统的功能：

| 子系统 | 功能 | 输入 | 输出 |
|:------:|:----:|:----:|:----:|
| Trajectory Generator | 生成参考轨迹 | 时间 $t$ | $\mathbf{r}_{\text{des}}, \mathbf{v}_{\text{des}}, \psi_{\text{des}}$ |
| Position Controller | 位置外环控制 | 位置/速度误差 | $F_{\text{des}}, \boldsymbol{\Phi}_{\text{des}}$ |
| Attitude Controller | 姿态内环控制 | 姿态/角速度误差 | $\mathbf{M}_{\text{des}}$ |
| Thrust Allocation | 推力分配 | $F_{\text{des}}, \mathbf{M}_{\text{des}}$ | $\omega_1, \ldots, \omega_6$ |
| Hexacopter Dynamics | 六旋翼动力学 | $\omega_1, \ldots, \omega_6$ | 状态 $\mathbf{x}$ |
| Scope / Logging | 数据记录与可视化 | 状态 $\mathbf{x}$ | 图形输出 |

### 5.2 参数设置 (Parameter Configuration)

在运行仿真前，需要执行参数初始化脚本 `init_params.m`。典型参数如下：

```matlab
%% 物理参数 (Physical Parameters)
params.m   = 2.0;        % 质量 [kg]
params.g   = 9.81;       % 重力加速度 [m/s^2]
params.L   = 0.3;        % 旋翼臂长 [m]
params.kF  = 1.0e-5;     % 推力系数 [N·s^2/rad^2]
params.kM  = 1.0e-7;     % 力矩系数 [N·m·s^2/rad^2]

%% 惯性参数 (Inertia Parameters)
params.Ixx = 0.0123;     % [kg·m^2]
params.Iyy = 0.0123;     % [kg·m^2]
params.Izz = 0.0224;     % [kg·m^2]
params.I_B = diag([params.Ixx, params.Iyy, params.Izz]);

%% 旋翼限制 (Rotor Limits)
params.omega_max = 1500;  % 最大转速 [rad/s]
params.omega_min = 0;     % 最小转速 [rad/s]

%% 位置控制增益 (Position Control Gains)
params.kp_x = 4.0;   params.kd_x = 4.0;
params.kp_y = 4.0;   params.kd_y = 4.0;
params.kp_z = 8.0;   params.kd_z = 6.0;

%% 姿态控制增益 (Attitude Control Gains)
params.kp_phi   = 100;   params.kd_phi   = 20;
params.kp_theta = 100;   params.kd_theta = 20;
params.kp_psi   = 50;    params.kd_psi   = 15;
```

### 5.3 仿真步骤 (Simulation Procedure)

1. **初始化参数**: 在 MATLAB 命令窗口运行 `init_params`
2. **打开模型**: 双击 `hexacopter.slx` 或在命令窗口输入 `open_system('hexacopter')`
3. **设置仿真时间**: 建议 $T = 20 \, \text{s}$
4. **选择求解器**: 推荐使用 `ode45`（变步长）或 `ode4`（固定步长，步长 $0.001 \, \text{s}$）
5. **运行仿真**: 点击 Run 按钮或在命令窗口输入 `sim('hexacopter')`
6. **查看结果**: 运行 `plot_results` 脚本

### 5.4 仿真结果分析 (Result Analysis)

仿真完成后，应重点分析以下内容：

#### 位置跟踪性能

- 绘制 $x(t), y(t), z(t)$ 与参考轨迹的对比曲线
- 计算位置跟踪误差的均方根 (RMS):

$$e_{\text{RMS}} = \sqrt{\frac{1}{T} \int_0^T \|\mathbf{r}_{\text{des}}(t) - \mathbf{r}(t)\|^2 \, dt}$$

#### 姿态响应

- 绘制 $\phi(t), \theta(t), \psi(t)$ 的时间历程
- 检查姿态角是否保持在合理范围内（通常 $|\phi|, |\theta| < 30°$）
- 观察是否存在振荡或超调

#### 旋翼转速

- 绘制 $\omega_1(t), \ldots, \omega_6(t)$ 的时间历程
- 检查是否存在饱和 (saturation) 现象
- 分析悬停时各旋翼转速是否均匀

#### 典型测试场景

| 测试场景 | 描述 | 关注点 |
|:--------:|:----:|:------:|
| 悬停 (Hover) | 从初始位置起飞并悬停在目标高度 | 稳态误差、收敛速度 |
| 阶跃响应 (Step) | 位置阶跃指令 | 上升时间、超调量、调节时间 |
| 圆形轨迹 (Circle) | 水平面内匀速圆周运动 | 跟踪精度、姿态变化 |
| 八字轨迹 (Figure-8) | 水平面内八字形轨迹 | 动态跟踪性能 |

### 5.5 调参建议 (Tuning Tips)

- **先调内环，再调外环**: 确保姿态控制稳定后再调位置控制
- **从小增益开始**: 逐步增大比例增益 $k_p$，观察响应
- **阻尼比**: 保持 $\zeta \approx 0.7 \sim 1.0$，避免振荡
- **带宽分离**: 内环带宽应至少为外环的 3-5 倍
- **注意饱和**: 如果旋翼频繁饱和，需要降低控制增益或限制参考轨迹的激进程度

---

## 附录 A: 常用公式速查 (Quick Reference)

| 公式 | 表达式 |
|:----:|:------:|
| 推力 | $F_i = k_F \omega_i^2$ |
| 反扭矩 | $M_i = \sigma_i k_M \omega_i^2$ |
| 平动方程 | $m\ddot{\mathbf{r}} = \mathbf{C}_{IB} F \mathbf{e}_z - mg\mathbf{e}_z$ |
| 转动方程 | $\mathbf{I}_B \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathbf{I}_B \boldsymbol{\omega} = \mathbf{M}$ |
| 运动学 | $\dot{\mathbf{C}}_{IB} = \mathbf{C}_{IB} \tilde{\boldsymbol{\omega}}_B$ |
| 姿态 PD | $\mathbf{M}_{\text{des}} = \mathbf{K}_p \mathbf{e}_\Phi + \mathbf{K}_d \mathbf{e}_\omega$ |
| 位置 PD | $\mathbf{a}_{\text{des}} = \mathbf{K}_p \mathbf{e}_r + \mathbf{K}_d \mathbf{e}_v + g\mathbf{e}_z$ |
| 推力分配 | $\mathbf{u} = \mathbf{A}^\dagger \mathbf{b}$ |

## 附录 B: 符号表 (Notation Table)

| 符号 | 含义 | 单位 |
|:----:|:----:|:----:|
| $m$ | 飞行器质量 | kg |
| $g$ | 重力加速度 | m/s$^2$ |
| $L$ | 旋翼臂长 | m |
| $k_F$ | 推力系数 | N$\cdot$s$^2$/rad$^2$ |
| $k_M$ | 力矩系数 | N$\cdot$m$\cdot$s$^2$/rad$^2$ |
| $\mathbf{I}_B$ | 惯性张量 | kg$\cdot$m$^2$ |
| $\omega_i$ | 第 $i$ 个旋翼转速 | rad/s |
| $\phi, \theta, \psi$ | 欧拉角 (roll, pitch, yaw) | rad |
| $\mathbf{C}_{IB}$ | 旋转矩阵 (body to inertial) | — |
| $\boldsymbol{\omega}_B$ | 机体角速度 | rad/s |

---

> **参考文献 (References)**:
> 1. ETH Zurich, Robot Dynamics Lecture Notes
> 2. R. Mahony, V. Kumar, P. Corke, "Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor," IEEE Robotics & Automation Magazine, 2012
> 3. R. Siegwart, I. Nourbakhsh, D. Scaramuzza, "Introduction to Autonomous Mobile Robots," MIT Press
