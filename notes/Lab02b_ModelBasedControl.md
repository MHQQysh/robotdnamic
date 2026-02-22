# Exercise 2b: 基于模型的控制 — ABB IRB 120

> ETH Zurich · Robot Dynamics · Lab 02b: Model-based Control

---

## 1. 实验概述 (Lab Overview)

本实验基于 Exercise 2a 中建立的 ABB IRB 120 六自由度机械臂动力学模型，实现并验证三种基于模型的控制方法：

| 控制器 | 文件 | Simulink 模型 | 控制空间 |
|--------|------|---------------|----------|
| PD + 重力补偿 | `control_pd_g.m` | `abb_pd_g.mdl` | 关节空间 |
| 逆动力学控制 | `control_inv_dyn.m` | `abb_inv_dyn.mdl` | 操作空间 |
| 混合力/运动控制 | `control_op_space_hybrid.m` | `abb_op_space_hybrid.mdl` | 操作空间 |

实验目标：

- 理解关节空间与操作空间控制的区别
- 掌握逆动力学控制中任务空间动力学的推导
- 实现混合力/运动控制，处理机器人与环境的接触问题
- 在 Simulink 中对三种控制器进行仿真验证，观察跟踪性能

前置要求：完成 Exercise 2a 中的 `jointToPosition.m`、`jointToRotMat.m`、`jointToJac.m`、`jointToJacDot.m`、`gravityVector.m`、`massMatrix.m`、`coriolisMatrix.m` 等函数。

---

## 2. 控制器设计与实现

### 2.1 PD + 重力补偿控制 (control_pd_g.m)

这是最简单的基于模型的控制器。核心思想：用 PD 反馈跟踪关节空间期望轨迹，同时补偿重力项以消除稳态误差。

**控制律：**

$$
\tau = K_p (q_{des} - q) + K_d (\dot{q}_{des} - \dot{q}) + g(q)
$$

其中：
- $q_{des}, \dot{q}_{des}$：期望关节位置与速度
- $q, \dot{q}$：当前关节位置与速度
- $K_p \in \mathbb{R}^{6 \times 6}$：比例增益矩阵（正定对角阵）
- $K_d \in \mathbb{R}^{6 \times 6}$：微分增益矩阵（正定对角阵）
- $g(q) \in \mathbb{R}^{6}$：重力向量

**MATLAB 实现：**

```matlab
function tau = control_pd_g(trajectory, q, dq)
% CONTROL_PD_G  PD control with gravity compensation.
%
%   trajectory: struct with fields .q_des, .dq_des
%   q:          6x1 current joint positions [rad]
%   dq:         6x1 current joint velocities [rad/s]

    % --- 增益矩阵 ---
    % 每个关节可设置不同增益，此处使用对角阵
    Kp = diag([100, 100, 80, 50, 50, 30]);
    Kd = diag([ 20,  20, 18, 14, 14, 11]);

    % --- 期望轨迹 ---
    q_des  = trajectory.q_des;
    dq_des = trajectory.dq_des;

    % --- 关节误差 ---
    e   = q_des - q;      % 位置误差
    de  = dq_des - dq;    % 速度误差

    % --- 重力补偿 ---
    g = gravityVector(q);  % 来自 Exercise 2a

    % --- 控制律 ---
    tau = Kp * e + Kd * de + g;
end
```

**特点与注意事项：**

- 重力补偿项 $g(q)$ 确保机械臂在静止时不会因重力下垂
- 不补偿惯性力和科里奥利力，因此高速运动时跟踪精度下降
- 增益调节简单，适合作为基线控制器
- 每个关节独立控制，忽略关节间耦合


---

### 2.2 逆动力学控制 (control_inv_dyn.m)

操作空间逆动力学控制器在任务空间（末端执行器的位姿空间）中进行轨迹跟踪。通过将关节空间动力学投影到任务空间，实现对末端位置和姿态的精确控制。

**控制步骤：**

#### Step 1: 计算位姿误差

末端执行器的位姿误差由位置误差和姿态误差组成：

$$
\chi_{err} = \begin{bmatrix} r_{des} - r \ \text{orientation\_error} \end{bmatrix} \in \mathbb{R}^{6}
$$

其中位置误差直接相减，姿态误差通过旋转矩阵计算：

$$
C_{err} = C_{IE,des} \cdot C_{IE}^T
$$

$$
\text{orientation\_error} = \text{rotMatToRotVec}(C_{err})
$$

`rotMatToRotVec` 将旋转矩阵转换为旋转向量（rotation vector parameterization），即轴角表示 $\theta \hat{n}$，其中 $\hat{n}$ 为旋转轴，$\theta$ 为旋转角度。

```matlab
function rotVec = rotMatToRotVec(C)
% ROTMATTOROTVEC  Convert rotation matrix to rotation vector.
    theta = acos((trace(C) - 1) / 2);
    if abs(theta) < 1e-6
        rotVec = [0; 0; 0];
    else
        rotVec = theta / (2 * sin(theta)) * [C(3,2)-C(2,3);
                                               C(1,3)-C(3,1);
                                               C(2,1)-C(1,2)];
    end
end
```

#### Step 2: 计算期望加速度

在任务空间中使用 PD 反馈计算期望加速度：

$$
\ddot{w} = K_p \cdot \chi_{err} + K_d \cdot (\dot{w}_{des} - \dot{w})
$$

其中 $\dot{w} = J \dot{q}$ 为末端执行器的广义速度（包含线速度和角速度）。

#### Step 3: 任务空间动力学投影

关节空间动力学方程为：

$$
M(q)\ddot{q} + b(q, \dot{q}) + g(q) = \tau
$$

通过 Jacobian 投影到任务空间，定义任务空间惯性矩阵：

$$
\Lambda = (J \cdot M^{-1} \cdot J^T)^{-1}
$$

任务空间科里奥利/离心力项：

$$
\mu = \Lambda \cdot J \cdot M^{-1} \cdot b - \Lambda \cdot \dot{J} \cdot \dot{q}
$$

任务空间重力项：

$$
p = \Lambda \cdot J \cdot M^{-1} \cdot g
$$

#### Step 4: 计算关节力矩

$$
\tau = J^T (\Lambda \cdot \ddot{w} + \mu + p)
$$

**MATLAB 实现：**

```matlab
function tau = control_inv_dyn(trajectory, q, dq)
% CONTROL_INV_DYN  Operational space inverse dynamics control.
%
%   trajectory: struct with fields .r_des, .C_IE_des, .w_des, .dw_des
%   q:          6x1 current joint positions [rad]
%   dq:         6x1 current joint velocities [rad/s]

    % --- 增益矩阵 ---
    Kp = diag([100, 100, 100, 50, 50, 50]);
    Kd = diag([ 20,  20,  20, 14, 14, 14]);

    % --- 从动力学模型获取矩阵 ---
    M    = massMatrix(q);          % 6x6 惯性矩阵
    b    = coriolisVector(q, dq);  % 6x1 科里奥利/离心力向量
    g    = gravityVector(q);       % 6x1 重力向量
    J    = jointToJac(q);          % 6x6 Jacobian
    Jdot = jointToJacDot(q, dq);  % 6x6 Jacobian 时间导数

    % --- 当前末端位姿 ---
    r    = jointToPosition(q);     % 3x1 末端位置
    C_IE = jointToRotMat(q);       % 3x3 末端旋转矩阵

    % --- 期望轨迹 ---
    r_des    = trajectory.r_des;
    C_IE_des = trajectory.C_IE_des;
    w_des    = trajectory.w_des;    % 6x1 期望广义速度
    dw_des   = trajectory.dw_des;   % 6x1 期望广义加速度

    % --- Step 1: 位姿误差 ---
    pos_error = r_des - r;
    C_err     = C_IE_des * C_IE';
    ori_error = rotMatToRotVec(C_err);
    chi_err   = [pos_error; ori_error];

    % --- Step 2: 期望加速度 ---
    w    = J * dq;                  % 当前末端广义速度
    dw_d = Kp * chi_err + Kd * (w_des - w);

    % --- Step 3: 任务空间动力学 ---
    M_inv  = M \ eye(6);
    Lambda = (J * M_inv * J') \ eye(6);   % 任务空间惯性矩阵
    mu     = Lambda * J * M_inv * b ...
           - Lambda * Jdot * dq;           % 任务空间科里奥利力
    p      = Lambda * J * M_inv * g;       % 任务空间重力

    % --- Step 4: 关节力矩 ---
    tau = J' * (Lambda * dw_d + mu + p);
end
```


---

### 2.3 操作空间混合力/运动控制 (control_op_space_hybrid.m)

混合力/运动控制器是最复杂的控制方案。它允许机器人在某些方向上跟踪运动轨迹，同时在其他方向上施加期望的接触力。典型应用场景：末端执行器沿表面滑动（运动控制）同时保持法向接触力（力控制）。

**选择矩阵 (Selection Matrices)：**

选择矩阵用于将任务空间分解为运动控制子空间和力控制子空间。在环境坐标系 (Environment frame, E) 中定义：

$$
\Sigma_{Mp} = \text{diag}(\sigma_{mx}, \sigma_{my}, \sigma_{mz}) \quad \text{(平移运动选择)}
$$

$$
\Sigma_{Mr} = \text{diag}(\sigma_{m\alpha}, \sigma_{m\beta}, \sigma_{m\gamma}) \quad \text{(旋转运动选择)}
$$

$$
\Sigma_{Fp} = \text{diag}(\sigma_{fx}, \sigma_{fy}, \sigma_{fz}) \quad \text{(平移力选择)}
$$

$$
\Sigma_{Fr} = \text{diag}(\sigma_{f\alpha}, \sigma_{f\beta}, \sigma_{f\gamma}) \quad \text{(旋转力矩选择)}
$$

互补约束：在每个方向上，要么控制运动，要么控制力：

$$
\Sigma_{Mp} + \Sigma_{Fp} = I_{3 \times 3}, \quad \Sigma_{Mr} + \Sigma_{Fr} = I_{3 \times 3}
$$

例如，若末端执行器需要在 $z$ 方向施加力，在 $x, y$ 方向自由运动：

$$
\Sigma_{Mp} = \text{diag}(1, 1, 0), \quad \Sigma_{Fp} = \text{diag}(0, 0, 1)
$$

**从环境坐标系到惯性坐标系的变换：**

选择矩阵在环境坐标系中定义，需要变换到惯性坐标系中使用：

$$
S_m = C_{IE}^T \cdot \Sigma_M \cdot C_{IE} \quad \text{(运动选择矩阵)}
$$

$$
S_f = C_{IE}^T \cdot (I - \Sigma_F) \cdot C_{IE} \quad \text{(力选择矩阵)}
$$

其中 $\Sigma_M = \text{blkdiag}(\Sigma_{Mp}, \Sigma_{Mr})$，$\Sigma_F = \text{blkdiag}(\Sigma_{Fp}, \Sigma_{Fr})$。

注意：这里 $C_{IE}$ 是环境坐标系相对于惯性坐标系的旋转矩阵，而非末端执行器的旋转矩阵。

**控制律：**

$$
\tau = J^T \left( \Lambda \cdot S_m \cdot \ddot{w} + S_f \cdot F_{des} + \mu + p \right)
$$

其中：
- $\ddot{w}$：与逆动力学控制器相同的 PD 反馈期望加速度
- $F_{des}$：期望接触力/力矩（在惯性坐标系中表示）
- $S_m \cdot \ddot{w}$：仅在运动控制方向上施加加速度
- $S_f \cdot F_{des}$：仅在力控制方向上施加期望力

**MATLAB 实现：**

```matlab
function tau = control_op_space_hybrid(trajectory, q, dq, F_ext)
% CONTROL_OP_SPACE_HYBRID  Operational space hybrid force/motion control.
%
%   trajectory: struct with fields .r_des, .C_IE_des, .w_des, .dw_des,
%               .F_des, .Sigma_Mp, .Sigma_Mr, .Sigma_Fp, .Sigma_Fr, .C_IE_env
%   q:          6x1 current joint positions [rad]
%   dq:         6x1 current joint velocities [rad/s]
%   F_ext:      6x1 measured external force/torque (force sensor)

    % --- 增益矩阵 ---
    Kp = diag([100, 100, 100, 50, 50, 50]);
    Kd = diag([ 20,  20,  20, 14, 14, 14]);

    % --- 从动力学模型获取矩阵 ---
    M    = massMatrix(q);
    b    = coriolisVector(q, dq);
    g    = gravityVector(q);
    J    = jointToJac(q);
    Jdot = jointToJacDot(q, dq);

    % --- 当前末端位姿 ---
    r    = jointToPosition(q);
    C_IE = jointToRotMat(q);

    % --- 期望轨迹与力 ---
    r_des    = trajectory.r_des;
    C_IE_des = trajectory.C_IE_des;
    w_des    = trajectory.w_des;
    dw_des   = trajectory.dw_des;
    F_des    = trajectory.F_des;       % 6x1 期望力/力矩

    % --- 选择矩阵 ---
    Sigma_Mp = trajectory.Sigma_Mp;    % 3x3 平移运动选择
    Sigma_Mr = trajectory.Sigma_Mr;    % 3x3 旋转运动选择
    Sigma_Fp = trajectory.Sigma_Fp;    % 3x3 平移力选择
    Sigma_Fr = trajectory.Sigma_Fr;    % 3x3 旋转力矩选择
    C_IE_env = trajectory.C_IE_env;    % 3x3 环境坐标系旋转矩阵

    % --- 构建 6x6 选择矩阵 ---
    Sigma_M = blkdiag(Sigma_Mp, Sigma_Mr);
    Sigma_F = blkdiag(Sigma_Fp, Sigma_Fr);
    C_blk   = blkdiag(C_IE_env, C_IE_env);  % 6x6 块对角旋转

    S_m = C_blk' * Sigma_M * C_blk;         % 运动选择 (惯性系)
    S_f = C_blk' * (eye(6) - Sigma_F) * C_blk; % 力选择 (惯性系)

    % --- 位姿误差 (同逆动力学控制器) ---
    pos_error = r_des - r;
    C_err     = C_IE_des * C_IE';
    ori_error = rotMatToRotVec(C_err);
    chi_err   = [pos_error; ori_error];

    % --- 期望加速度 ---
    w    = J * dq;
    dw_d = Kp * chi_err + Kd * (w_des - w);

    % --- 任务空间动力学 ---
    M_inv  = M \ eye(6);
    Lambda = (J * M_inv * J') \ eye(6);
    mu     = Lambda * J * M_inv * b - Lambda * Jdot * dq;
    p      = Lambda * J * M_inv * g;

    % --- 混合控制律 ---
    tau = J' * (Lambda * S_m * dw_d + S_f * F_des + mu + p);
end
```


---

## 3. 关键理论

### 3.1 任务空间动力学推导

关节空间动力学方程：

$$
M(q)\ddot{q} + b(q, \dot{q}) + g(q) = \tau
$$

末端执行器的广义速度与关节速度通过 Jacobian 关联：

$$
\dot{w} = J(q) \dot{q}
$$

对上式求导得加速度关系：

$$
\ddot{w} = J \ddot{q} + \dot{J} \dot{q}
$$

由此可解出关节加速度：

$$
\ddot{q} = J^{-1}(\ddot{w} - \dot{J}\dot{q})
$$

代入关节空间动力学方程，并左乘 $J^{-T}$，得到任务空间动力学方程：

$$
\Lambda(q) \ddot{w} + \mu(q, \dot{q}) + p(q) = F
$$

其中：

| 符号 | 定义 | 物理含义 |
|------|------|----------|
| $\Lambda$ | $(J M^{-1} J^T)^{-1}$ | 任务空间惯性矩阵 |
| $\mu$ | $\Lambda J M^{-1} b - \Lambda \dot{J} \dot{q}$ | 任务空间科里奥利/离心力 |
| $p$ | $\Lambda J M^{-1} g$ | 任务空间重力 |
| $F$ | $J^{-T} \tau$ | 末端执行器广义力 |

任务空间惯性矩阵 $\Lambda$ 的性质：
- 对称正定
- 反映末端执行器在各方向上的等效惯性
- 随构型变化（configuration-dependent）

### 3.2 姿态误差计算

姿态误差的计算是操作空间控制中的关键环节。本实验使用旋转向量（rotation vector）参数化。

**旋转矩阵误差：**

$$
C_{err} = C_{IE,des} \cdot C_{IE}^T
$$

当 $C_{err} = I$ 时，当前姿态与期望姿态一致，误差为零。

**旋转向量提取：**

从旋转矩阵 $C_{err}$ 中提取旋转角度和旋转轴：

$$
\theta = \arccos\left(\frac{\text{tr}(C_{err}) - 1}{2}\right)
$$

$$
\hat{n} = \frac{1}{2\sin\theta} \begin{bmatrix} C_{err}(3,2) - C_{err}(2,3) \ C_{err}(1,3) - C_{err}(3,1) \ C_{err}(2,1) - C_{err}(1,2) \end{bmatrix}
$$

旋转向量为：

$$
\text{orientation\_error} = \theta \hat{n}
$$

**奇异情况处理：**

- 当 $\theta \approx 0$ 时，$\sin\theta \approx 0$，需要特殊处理（返回零向量）
- 当 $\theta \approx \pi$ 时，也存在数值问题，需要使用替代公式

```matlab
function rotVec = rotMatToRotVec(C)
% ROTMATTOROTVEC  Rotation matrix to rotation vector.
%   Handles the singularity at theta = 0 and theta = pi.

    val = (trace(C) - 1) / 2;
    val = max(-1, min(1, val));   % 数值裁剪，防止 acos 出错
    theta = acos(val);

    if theta < 1e-6
        % 小角度近似: rotVec ≈ vee(C - I)
        rotVec = [C(3,2) - C(2,3);
                  C(1,3) - C(3,1);
                  C(2,1) - C(1,2)] / 2;
    elseif abs(theta - pi) < 1e-6
        % theta ≈ pi: 使用对称部分提取旋转轴
        [~, idx] = max(diag(C));
        n = C(:, idx) + eye(3, 1) * (idx == 1) ...
                      + eye(3, 2) * (idx == 2) ...
                      + eye(3, 3) * (idx == 3);
        n = n / norm(n);
        rotVec = pi * n;
    else
        rotVec = theta / (2 * sin(theta)) * ...
                 [C(3,2) - C(2,3);
                  C(1,3) - C(3,1);
                  C(2,1) - C(1,2)];
    end
end
```

### 3.3 选择矩阵设计

选择矩阵的设计取决于具体的任务需求。以下是几个典型场景：

**场景 1：末端执行器在水平面上滑动，保持竖直方向接触力**

环境坐标系与惯性坐标系对齐（$C_{IE,env} = I$）：

```matlab
Sigma_Mp = diag([1, 1, 0]);  % x, y 方向运动控制
Sigma_Mr = diag([1, 1, 1]);  % 所有旋转方向运动控制
Sigma_Fp = diag([0, 0, 1]);  % z 方向力控制
Sigma_Fr = diag([0, 0, 0]);  % 无旋转力矩控制
% 验证互补性: Sigma_Mp + Sigma_Fp = I, Sigma_Mr + Sigma_Fr = I
```

**场景 2：沿倾斜表面滑动**

环境坐标系需要旋转以对齐表面法向量：

```matlab
% 假设表面法向量为 n = [0; sin(alpha); cos(alpha)]
alpha = pi/6;  % 表面倾斜 30 度
C_IE_env = [1,  0,           0;
            0,  cos(alpha), -sin(alpha);
            0,  sin(alpha),  cos(alpha)];

Sigma_Mp = diag([1, 1, 0]);  % 表面切向运动
Sigma_Fp = diag([0, 0, 1]);  % 表面法向力
```

**场景 3：纯运动控制（无力控制）**

```matlab
Sigma_Mp = diag([1, 1, 1]);  % 所有平移方向运动控制
Sigma_Mr = diag([1, 1, 1]);  % 所有旋转方向运动控制
Sigma_Fp = diag([0, 0, 0]);  % 无力控制
Sigma_Fr = diag([0, 0, 0]);  % 无力矩控制
% 此时退化为纯逆动力学控制器
```


---

## 4. 增益调节指南

### 4.1 增益选择原则

PD 控制器的增益直接影响系统的响应速度、超调量和稳定性。对于线性二阶系统：

$$
\ddot{e} + K_d \dot{e} + K_p e = 0
$$

其特征方程为：

$$
s^2 + K_d s + K_p = 0
$$

自然频率和阻尼比：

$$
\omega_n = \sqrt{K_p}, \quad \zeta = \frac{K_d}{2\omega_n} = \frac{K_d}{2\sqrt{K_p}}
$$

### 4.2 临界阻尼条件

临界阻尼（$\zeta = 1$）是最常用的设计目标，系统以最快速度收敛且无超调：

$$
K_d = 2\sqrt{K_p}
$$

| $K_p$ | $K_d$（临界阻尼） | $\omega_n$ [rad/s] | 响应特性 |
|-------|-------------------|---------------------|----------|
| 25    | 10                | 5                   | 较慢，平滑 |
| 100   | 20                | 10                  | 中等，推荐 |
| 400   | 40                | 20                  | 较快，对噪声敏感 |
| 900   | 60                | 30                  | 很快，需要高采样率 |

### 4.3 关节空间 vs 操作空间增益

**关节空间（PD + 重力补偿）：**

每个关节可以独立设置增益。靠近基座的关节（关节 1-3）承受更大的惯性和重力负载，通常需要更高的增益：

```matlab
% 推荐起始值
Kp = diag([100, 100, 80, 50, 50, 30]);
Kd = diag([ 20,  20, 18, 14, 14, 11]);
```

**操作空间（逆动力学 / 混合控制）：**

增益在任务空间中定义。平移方向和旋转方向通常使用不同的增益：

```matlab
% 平移方向增益（单位与位置误差 [m] 相关）
Kp_trans = 100;  % 较高，因为位置误差通常较小
Kd_trans = 20;

% 旋转方向增益（单位与角度误差 [rad] 相关）
Kp_rot = 50;     % 较低，因为角度误差可能较大
Kd_rot = 14;

Kp = diag([Kp_trans, Kp_trans, Kp_trans, Kp_rot, Kp_rot, Kp_rot]);
Kd = diag([Kd_trans, Kd_trans, Kd_trans, Kd_rot, Kd_rot, Kd_rot]);
```

### 4.4 调参建议

1. 从较低的增益开始，逐步增大
2. 先调 $K_p$ 使系统能跟踪轨迹，再调 $K_d$ 抑制振荡
3. 始终保持 $K_d \geq 2\sqrt{K_p}$（略过阻尼优于欠阻尼）
4. 观察力矩是否饱和——如果力矩达到电机极限，需要降低增益或减慢轨迹
5. 对于逆动力学控制器，由于模型补偿了大部分非线性，增益可以设置得相对较低

---

## 5. Simulink 仿真说明

### 5.1 仿真模型概览

本实验提供三个 Simulink 模型，分别对应三种控制器：

```
abb_pd_g.mdl              % PD + 重力补偿
abb_inv_dyn.mdl           % 逆动力学控制
abb_op_space_hybrid.mdl   % 混合力/运动控制
```

每个模型的基本结构如下：

```
┌─────────────┐    ┌────────────┐    ┌──────────────┐    ┌──────────┐
│  Trajectory  │───>│ Controller │───>│  Robot Plant │───>│  Sensors │──┐
│  Generator   │    │            │<───│  (ABB IRB120)│    │          │  │
└─────────────┘    └────────────┘    └──────────────┘    └──────────┘  │
                         ^                                              │
                         └──────────────────────────────────────────────┘
                                      反馈回路
```

### 5.2 运行仿真

**步骤 1：初始化工作空间**

在 MATLAB 命令窗口中运行初始化脚本，加载机器人参数：

```matlab
% 初始化 ABB IRB 120 参数
init_abb_irb120;

% 设置仿真参数
T_sim = 10;          % 仿真时长 [s]
dt    = 0.001;       % 仿真步长 [s]
q0    = [0; 0; 0; 0; 0; 0];  % 初始关节角度 [rad]
```

**步骤 2：打开并运行 Simulink 模型**

```matlab
% 打开模型（以 PD + 重力补偿为例）
open_system('abb_pd_g');

% 运行仿真
sim('abb_pd_g', T_sim);
```

**步骤 3：观察结果**

仿真完成后，可以通过 Scope 模块或导出数据进行分析：

```matlab
% 绘制关节角度跟踪曲线
figure;
subplot(2,1,1);
plot(t, q_des, '--', t, q, '-');
xlabel('Time [s]');
ylabel('Joint angle [rad]');
title('Joint Position Tracking');
legend('q_{des,1}','q_{des,2}','q_{des,3}','q_1','q_2','q_3');
grid on;

subplot(2,1,2);
plot(t, q_des - q);
xlabel('Time [s]');
ylabel('Tracking error [rad]');
title('Joint Position Error');
grid on;
```

### 5.3 各模型的特殊设置

**abb_pd_g.mdl：**
- 轨迹在关节空间中定义（`q_des`, `dq_des`）
- 无需 Jacobian 或任务空间计算
- 适合验证基本的关节空间跟踪

**abb_inv_dyn.mdl：**
- 轨迹在操作空间中定义（`r_des`, `C_IE_des`, `w_des`, `dw_des`）
- 需要正运动学将关节空间轨迹转换为操作空间轨迹
- 可以观察末端执行器在笛卡尔空间中的跟踪效果

```matlab
% 绘制末端执行器位置跟踪
figure;
plot3(r_log(1,:), r_log(2,:), r_log(3,:), 'b-', ...
      r_des_log(1,:), r_des_log(2,:), r_des_log(3,:), 'r--');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('End-Effector Position Tracking');
legend('Actual', 'Desired');
grid on; axis equal;
```

**abb_op_space_hybrid.mdl：**
- 需要额外配置选择矩阵和期望力
- 包含环境模型（弹簧-阻尼器模拟接触面）
- 可以同时观察运动跟踪和力跟踪效果

```matlab
% 绘制力跟踪曲线
figure;
subplot(2,1,1);
plot(t, F_des(3,:), 'r--', t, F_ext(3,:), 'b-');
xlabel('Time [s]');
ylabel('Force [N]');
title('Normal Force Tracking (z-direction)');
legend('F_{des}', 'F_{actual}');
grid on;

subplot(2,1,2);
plot(t, r_log(1,:), 'b-', t, r_des_log(1,:), 'r--');
xlabel('Time [s]');
ylabel('Position [m]');
title('Tangential Motion Tracking (x-direction)');
legend('Actual', 'Desired');
grid on;
```


### 5.4 控制器性能对比

通过仿真可以对比三种控制器在不同场景下的表现：

| 评价指标 | PD + 重力补偿 | 逆动力学控制 | 混合力/运动控制 |
|----------|---------------|--------------|-----------------|
| 关节空间跟踪精度 | 中等 | 高 | 高 |
| 操作空间跟踪精度 | 低（无任务空间反馈） | 高 | 高（运动方向） |
| 力控制能力 | 无 | 无 | 有 |
| 计算复杂度 | 低 | 高 | 最高 |
| 对模型精度的依赖 | 低（仅需重力模型） | 高（需要完整动力学模型） | 高 |
| 鲁棒性 | 较好 | 对模型误差敏感 | 对模型误差敏感 |
| 适用场景 | 简单定点运动 | 精确轨迹跟踪 | 接触任务 |

```matlab
% 对比三种控制器的跟踪误差
figure;
subplot(3,1,1);
plot(t, err_pd_g);
title('PD + Gravity Compensation');
ylabel('||e|| [rad]'); grid on;

subplot(3,1,2);
plot(t, err_inv_dyn);
title('Inverse Dynamics');
ylabel('||e|| [m]'); grid on;

subplot(3,1,3);
plot(t, err_hybrid);
title('Hybrid Force/Motion');
ylabel('||e|| [m]'); grid on;
xlabel('Time [s]');
```

---

## 6. 常见问题与调试

### 6.1 仿真发散

如果仿真中关节角度或力矩迅速增大至无穷，可能的原因：

- **增益过高**：降低 $K_p$ 和 $K_d$，从小值开始逐步增大
- **Jacobian 奇异**：当机械臂接近奇异构型时，$\Lambda = (J M^{-1} J^T)^{-1}$ 会出现数值问题。可以添加阻尼最小二乘（damped least squares）：

```matlab
% 阻尼最小二乘，避免奇异
lambda_dls = 0.01;
Lambda = (J * M_inv * J' + lambda_dls * eye(6)) \ eye(6);
```

- **符号错误**：检查控制律中各项的符号，特别是 $\mu$ 和 $p$ 的方向

### 6.2 稳态误差

- PD + 重力补偿：如果 $g(q)$ 计算不准确，会存在稳态误差。检查 `gravityVector.m` 的实现
- 逆动力学控制：如果动力学模型不精确（$M$, $b$, $g$ 有误差），PD 反馈无法完全补偿，会有残余误差。可以增大增益或添加积分项

### 6.3 力控制振荡

混合控制器中力跟踪出现振荡：

- 降低力控制方向的增益
- 检查环境模型的刚度参数——过高的环境刚度会导致力控制不稳定
- 确保选择矩阵的互补性：$\Sigma_M + \Sigma_F = I$

### 6.4 姿态跟踪问题

- 确认 `rotMatToRotVec` 正确处理了小角度和 $\pi$ 附近的奇异情况
- 检查旋转矩阵是否满足正交性：$C^T C = I$，$\det(C) = 1$
- 姿态误差的方向需要与 Jacobian 的角速度部分一致（body frame vs. spatial frame）

---

## 7. 扩展思考

### 7.1 冗余机械臂的零空间控制

对于自由度大于任务空间维度的冗余机械臂，可以在零空间中添加辅助任务：

$$
\tau = J^T F + (I - J^T J^{-T}) \tau_0
$$

其中 $\tau_0$ 为零空间力矩，可用于避障、关节极限回避或优化操作性指标。ABB IRB 120 为 6 自由度机械臂，任务空间也是 6 维，因此没有冗余自由度。但理解零空间概念对于处理更复杂的机器人系统非常重要。

### 7.2 自适应控制

当动力学模型参数不精确时，可以使用自适应控制在线估计参数：

$$
\tau = Y(q, \dot{q}, \ddot{q}_r) \hat{\pi} + K_d s
$$

其中 $Y$ 为回归矩阵，$\hat{\pi}$ 为估计的动力学参数，$s = \dot{e} + \lambda e$ 为滑模变量。参数更新律为：

$$
\dot{\hat{\pi}} = -\Gamma Y^T s
$$

### 7.3 阻抗控制

阻抗控制是混合力/运动控制的替代方案，它不直接控制力，而是控制末端执行器与环境之间的动态关系（阻抗）：

$$
\Lambda_d (\ddot{x} - \ddot{x}_d) + D_d (\dot{x} - \dot{x}_d) + K_d (x - x_d) = F_{ext}
$$

其中 $\Lambda_d$, $D_d$, $K_d$ 分别为期望的惯性、阻尼和刚度矩阵。阻抗控制不需要力的直接测量，对环境模型的依赖更小，但无法精确跟踪期望力。

---

## 参考资料

- ETH Zurich, Robot Dynamics Lecture Notes, Chapter 7: Model-based Control
- Siciliano, B. et al., *Robotics: Modelling, Planning and Control*, Springer, 2009
- Khatib, O., "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation," IEEE Journal of Robotics and Automation, 1987
- Craig, J.J., *Introduction to Robotics: Mechanics and Control*, Pearson, 2005

---

> 本文档为 ETH Zurich Robot Dynamics 课程 Exercise 2b 的实验指南。所有 MATLAB 代码仅供参考，具体实现需根据课程提供的框架代码进行调整。
