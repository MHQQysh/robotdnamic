# Lab 03: 足式机器人动力学与控制

> ETH Zurich - Robot Dynamics 课程  
> Exercise 3: Legged Robot Dynamics and Control  
> 框架: ProNEu (Projected Newton-Euler)

---

## 1. 实验概述

### 1.1 目标

本实验的核心目标是为一个**平面足式机器人 (planar legged robot)** 实现完整的动力学建模与控制流程。具体包括:

- 理解浮动基座 (floating base) 动力学方程
- 实现关节空间 PID 控制器
- 实现基于层次化 QP (Hierarchical Quadratic Programming) 的浮动基座全身控制
- 实现混合力/运动控制 (hybrid force/motion control)
- 在 ProNEu 仿真环境中验证上述控制器

### 1.2 机器人构型

该平面足式机器人带有一个机械臂 (manipulator arm), 总体构型如下:

```
            [机械臂]
           shoulder - elbow - wrist - [末端执行器]
              |
    [浮动基座 body]
     /              \
  前腿 hip         后腿 hip
    |                 |
  前腿 knee         后腿 knee
    |                 |
  [前足]            [后足]
```

**自由度分配:**

| 部件 | 关节 | 自由度 | 是否驱动 |
|------|------|--------|----------|
| 浮动基座 (floating base) | x 平移, z 平移, θ 旋转 | 3 DOF | 否 (unactuated) |
| 前腿 (front leg) | hip, knee | 2 DOF | 是 |
| 后腿 (hind leg) | hip, knee | 2 DOF | 是 |
| 机械臂 (arm) | shoulder, elbow, wrist | 3 DOF | 是 |
| **合计** | | **10 DOF** | **7 actuated** |

广义坐标向量:

```
q = [x, z, θ, q_FH, q_FK, q_HH, q_HK, q_SH, q_EL, q_WR]ᵀ ∈ ℝ¹⁰
```

其中:
- `x, z, θ`: 基座位置与姿态
- `q_FH, q_FK`: 前腿 hip 和 knee 关节角
- `q_HH, q_HK`: 后腿 hip 和 knee 关节角
- `q_SH, q_EL, q_WR`: 机械臂 shoulder, elbow, wrist 关节角

---

## 2. 文件结构

```
leggedrobot/
├── setup.m                          % 初始化路径和参数
├── simulate.m                       % 运行仿真主脚本
├── models/
│   ├── leggedrobot.m                % 机器人模型定义 (连杆参数, 惯性, 运动学)
│   └── ground_and_wall.m            % 地面和墙壁环境模型
├── dynamics/
│   ├── ccfd.m                       % 接触约束力动力学 (Contact Constrained Forward Dynamics)
│   └── wall.m                       % 墙壁接触力模型
├── controllers/
│   ├── jointspace_pid_control.m     % 关节空间 PID 控制器
│   ├── floating_base_control.m      % 浮动基座全身控制器
│   └── hybrid_force_motion_control.m% 混合力/运动控制器
├── common/
│   ├── hopt.m                       % 层次化 QP 求解器
│   ├── trajectory.m                 % 轨迹生成工具
│   └── visualization.m             % 可视化工具
└── params/
    └── robot_params.m               % 机器人物理参数
```

### 关键文件说明

- **setup.m**: 调用 `addpath` 添加所有子目录, 加载机器人参数, 初始化仿真环境
- **simulate.m**: 主循环, 依次调用动力学计算和控制器, 更新状态
- **ccfd.m**: 求解带接触约束的正向动力学, 输出广义加速度 `q̈` 和接触力 `f_c`
- **hopt.m**: 层次化 QP 求解器, 按优先级逐层求解优化问题

---

## 3. 关键理论

### 3.1 浮动基座动力学

足式机器人的基座不固定于世界坐标系, 其运动方程为:

```
M(q)·q̈ + b(q, q̇) + g(q) = Sᵀ·τ + J_cᵀ·f_c + J_EEᵀ·f_EE
```

各项含义:

| 符号 | 维度 | 含义 |
|------|------|------|
| `M(q)` | 10×10 | 广义质量矩阵 (mass matrix) |
| `b(q, q̇)` | 10×1 | 科氏力和离心力 (Coriolis & centrifugal) |
| `g(q)` | 10×1 | 重力项 (gravity) |
| `S` | 7×10 | 选择矩阵 (selection matrix) |
| `τ` | 7×1 | 关节驱动力矩 (joint torques) |
| `J_c` | n_c×10 | 接触点 Jacobian |
| `f_c` | n_c×1 | 接触力 (contact forces) |
| `J_EE` | m×10 | 末端执行器 Jacobian |
| `f_EE` | m×1 | 末端执行器外力 |

**选择矩阵 S** 将 7 个驱动力矩映射到 10 维广义坐标空间:

```
S = [0₇ₓ₃, I₇] ∈ ℝ⁷ˣ¹⁰
```

前 3 列为零 (基座不驱动), 后 7 列为单位阵 (关节驱动)。因此:

```
Sᵀ·τ = [0₃ₓ₁; τ] ∈ ℝ¹⁰
```

### 3.2 接触约束

当足部与地面接触且不滑动时, 接触点速度为零:

```
v_contact = J_c · q̇ = 0
```

对时间求导得到加速度级别的约束:

```
J_c · q̈ + J̇_c · q̇ = 0
→ J_c · q̈ = -J̇_c · q̇
```

这是控制器中足部接触约束的核心等式。`J̇_c` 为接触 Jacobian 的时间导数, 可通过数值差分或解析方法计算。

### 3.3 摩擦锥约束

为防止足部滑动, 接触力必须满足 Coulomb 摩擦锥约束:

```
|f_t| ≤ μ · f_n
```

其中 `f_t` 为切向力 (tangential), `f_n` 为法向力 (normal), `μ` 为摩擦系数。

由于二次锥约束不便于 QP 求解, 将其**线性化**为:

```
 f_t - μ · f_n ≤ 0
-f_t - μ · f_n ≤ 0
```

等价于:

```
[-μ  1] · [f_n]  ≤ 0
[-μ -1]   [f_t]
```

对于每个接触点 (前足、后足), 都需要施加上述约束。

### 3.4 层次化 QP 优化 (Hierarchical QP)

层次化 QP 的核心思想是**严格优先级 (strict priority)**: 高优先级任务的解不会被低优先级任务违反。

求解流程:

```
第 1 层: min ‖A₁·x - b₁‖²   s.t. 约束
         → 得到最优值 w₁*
第 2 层: min ‖A₂·x - b₂‖²   s.t. A₁·x = b₁ + w₁*, 约束
         → 得到最优值 w₂*
第 k 层: min ‖Aₖ·x - bₖ‖²   s.t. 前 k-1 层结果固定, 约束
         ...
```

每一层在不违反更高优先级任务的前提下, 尽可能满足当前任务。

在 MATLAB 中调用 `hopt.m`:

```matlab
% 定义层次化任务
tasks = {};
tasks{1} = struct('A', A1, 'b', b1, 'type', 'eq');   % 等式约束 (最高优先级)
tasks{2} = struct('A', A2, 'b', b2, 'type', 'eq');
tasks{3} = struct('A', A3, 'b', b3, 'type', 'cost');  % 最小化目标
% ...

% 不等式约束 (全局)
ineq = struct('A', A_ineq, 'b', b_ineq);

% 求解
x_opt = hopt(tasks, ineq);
```

---

## 4. 控制器详解

### 4.1 控制器 A: 关节空间 PID 控制 (jointspace_pid_control.m)

这是最简单的控制器, 直接在关节空间进行 PD 控制, 不考虑浮动基座动力学。

**控制律:**

```
τ = K_p · (q_des - q_j) - K_d · q̇_j
```

其中 `q_j` 为 7 个驱动关节的角度, `q_des` 为期望关节角度。

**MATLAB 实现:**

```matlab
function tau = jointspace_pid_control(q, dq, params)
    % 提取关节角度 (去掉前3个基座自由度)
    q_j  = q(4:10);
    dq_j = dq(4:10);
    
    % 期望关节角度 (站立姿态)
    q_des = params.q_home(4:10);
    
    % PD 增益
    K_p = diag([100, 100, 100, 100, 50, 50, 50]);  % 腿部增益较大
    K_d = diag([10,  10,  10,  10,  5,  5,  5]);
    
    % PD 控制
    tau = K_p * (q_des - q_j) - K_d * dq_j;
    
    % 力矩饱和
    tau_max = params.tau_max;
    tau = max(min(tau, tau_max), -tau_max);
end
```

**特点:**
- 实现简单, 无需动力学模型
- 无法控制基座位姿 (基座是 unactuated)
- 无法保证接触约束
- 适合作为 baseline 或调试用途

### 4.2 控制器 B: 浮动基座全身控制 (floating_base_control.m)

基于层次化 QP 的全身控制器 (whole-body controller), 同时优化关节加速度、接触力和关节力矩。

**优化变量:**

```
x_opt = [q̈; f_c; τ] ∈ ℝ⁽¹⁰⁺ⁿᶜ⁺⁷⁾
```

其中 `n_c` 为接触力维度 (双足接触时 `n_c = 4`, 每足 2 维: 法向 + 切向)。

**任务层次 (从高到低):**

| 优先级 | 任务 | 类型 | 方程 |
|--------|------|------|------|
| 1 (最高) | 运动方程 | 等式 | `M·q̈ - J_cᵀ·f_c - Sᵀ·τ = -b - g` |
| 2 | 足部接触约束 | 等式 | `J_c·q̈ = -J̇_c·q̇` |
| 3 | 基座运动跟踪 | 等式/代价 | `J_b·q̈ = ẅ_b_des` |
| 4 | 末端执行器跟踪 | 等式/代价 | `J_ee·q̈ = ẅ_ee_des` |
| 5 | 默认姿态 | 代价 | `q̈ = K_p·(q₀ - q) - K_d·q̇` |
| 6 | 力矩最小化 | 代价 | `min ‖τ‖²` |
| 7 (最低) | 接触力最小化 | 代价 | `min ‖f_c‖²` |

**不等式约束 (全局):**

```
力矩限制:  -τ_max ≤ τ ≤ τ_max
摩擦锥:    f_t - μ·f_n ≤ 0,  -f_t - μ·f_n ≤ 0  (每个接触点)
法向力非负: -f_n ≤ 0
```

**MATLAB 实现:**

```matlab
function tau = floating_base_control(q, dq, params, t)
    %% 从模型中提取动力学量
    M    = params.M;       % 质量矩阵 10x10
    b    = params.b;       % 科氏力/离心力 10x1
    g    = params.g;       % 重力项 10x1
    J_c  = params.J_c;     % 接触 Jacobian n_c x 10
    dJ_c = params.dJ_c;    % 接触 Jacobian 时间导数
    
    n_q  = length(q);      % 10
    n_c  = size(J_c, 1);   % 接触力维度 (通常为4)
    n_tau = 7;             % 驱动关节数
    n_x  = n_q + n_c + n_tau;  % 优化变量总维度
    
    %% 选择矩阵
    S = [zeros(n_tau, 3), eye(n_tau)];  % 7x10
    
    %% 构建优化变量索引
    idx_qdd = 1:n_q;                          % q̈ 的索引
    idx_fc  = n_q+1 : n_q+n_c;                % f_c 的索引
    idx_tau = n_q+n_c+1 : n_x;                % τ 的索引
    
    %% 任务 1: 运动方程约束 (最高优先级, 等式)
    % M·q̈ - J_cᵀ·f_c - Sᵀ·τ = -b - g
    A1 = zeros(n_q, n_x);
    A1(:, idx_qdd) = M;
    A1(:, idx_fc)  = -J_c';
    A1(:, idx_tau)  = -S';
    b1 = -b - g;
    
    %% 任务 2: 足部接触约束 (等式)
    % J_c·q̈ = -dJ_c·q̇
    A2 = zeros(n_c, n_x);
    A2(:, idx_qdd) = J_c;
    b2 = -dJ_c * dq;
    
    %% 任务 3: 基座运动跟踪
    % J_b·q̈ = ẅ_b_des
    J_b = eye(3, n_q);  % 基座 Jacobian (前3个广义坐标)
    
    % 期望基座加速度 (PD 控制)
    q_b     = q(1:3);
    dq_b    = dq(1:3);
    q_b_des = params.q_b_des;    % 期望基座位姿
    dq_b_des = params.dq_b_des;  % 期望基座速度
    
    Kp_b = diag([200, 400, 300]);
    Kd_b = diag([40, 80, 60]);
    ddq_b_des = Kp_b * (q_b_des - q_b) + Kd_b * (dq_b_des - dq_b);
    
    A3 = zeros(3, n_x);
    A3(:, idx_qdd) = J_b;
    b3 = ddq_b_des;
    
    %% 任务 4: 末端执行器运动跟踪
    J_ee = params.J_ee;  % 末端执行器 Jacobian
    
    % 期望末端执行器加速度 (PD 控制)
    p_ee     = params.p_ee;       % 当前末端位置
    dp_ee    = params.dp_ee;      % 当前末端速度
    p_ee_des = params.p_ee_des;   % 期望末端位置
    dp_ee_des = params.dp_ee_des; % 期望末端速度
    
    Kp_ee = diag([150, 150]);
    Kd_ee = diag([30, 30]);
    ddp_ee_des = Kp_ee * (p_ee_des - p_ee) + Kd_ee * (dp_ee_des - dp_ee);
    dJ_ee = params.dJ_ee;
    w_ee_des = ddp_ee_des - dJ_ee * dq;
    
    A4 = zeros(size(J_ee, 1), n_x);
    A4(:, idx_qdd) = J_ee;
    b4 = w_ee_des;
    
    %% 任务 5: 默认姿态 (null-space posture)
    q0 = params.q_home;
    Kp_posture = 50 * eye(n_q);
    Kd_posture = 10 * eye(n_q);
    ddq_posture = Kp_posture * (q0 - q) - Kd_posture * dq;
    
    A5 = zeros(n_q, n_x);
    A5(:, idx_qdd) = eye(n_q);
    b5 = ddq_posture;
    
    %% 任务 6: 力矩最小化
    A6 = zeros(n_tau, n_x);
    A6(:, idx_tau) = eye(n_tau);
    b6 = zeros(n_tau, 1);
    
    %% 任务 7: 接触力最小化
    A7 = zeros(n_c, n_x);
    A7(:, idx_fc) = eye(n_c);
    b7 = zeros(n_c, 1);
    
    %% 不等式约束
    % 力矩限制: -tau_max <= tau <= tau_max
    % 写成 A_ineq * x <= b_ineq 的形式
    tau_max = params.tau_max;
    
    A_tau_upper = zeros(n_tau, n_x);
    A_tau_upper(:, idx_tau) = eye(n_tau);
    b_tau_upper = tau_max;
    
    A_tau_lower = zeros(n_tau, n_x);
    A_tau_lower(:, idx_tau) = -eye(n_tau);
    b_tau_lower = tau_max;
    
    % 摩擦锥约束 (每个接触点)
    mu = params.mu;  % 摩擦系数
    n_contacts = n_c / 2;  % 接触点数量
    A_friction = [];
    b_friction = [];
    
    for i = 1:n_contacts
        fn_idx = idx_fc(2*i - 1);  % 法向力索引
        ft_idx = idx_fc(2*i);      % 切向力索引
        
        % f_t - mu * f_n <= 0
        row1 = zeros(1, n_x);
        row1(ft_idx) = 1;
        row1(fn_idx) = -mu;
        
        % -f_t - mu * f_n <= 0
        row2 = zeros(1, n_x);
        row2(ft_idx) = -1;
        row2(fn_idx) = -mu;
        
        % -f_n <= 0 (法向力非负)
        row3 = zeros(1, n_x);
        row3(fn_idx) = -1;
        
        A_friction = [A_friction; row1; row2; row3];
        b_friction = [b_friction; 0; 0; 0];
    end
    
    A_ineq = [A_tau_upper; A_tau_lower; A_friction];
    b_ineq = [b_tau_upper; b_tau_lower; b_friction];
    
    %% 组装层次化任务并求解
    tasks = {};
    tasks{1} = struct('A', A1, 'b', b1, 'type', 'eq');
    tasks{2} = struct('A', A2, 'b', b2, 'type', 'eq');
    tasks{3} = struct('A', A3, 'b', b3, 'type', 'cost');
    tasks{4} = struct('A', A4, 'b', b4, 'type', 'cost');
    tasks{5} = struct('A', A5, 'b', b5, 'type', 'cost');
    tasks{6} = struct('A', A6, 'b', b6, 'type', 'cost');
    tasks{7} = struct('A', A7, 'b', b7, 'type', 'cost');
    
    ineq = struct('A', A_ineq, 'b', b_ineq);
    
    x_opt = hopt(tasks, ineq);
    
    %% 提取结果
    % qdd_opt = x_opt(idx_qdd);  % 可用于调试
    % fc_opt  = x_opt(idx_fc);   % 可用于调试
    tau = x_opt(idx_tau);         % 输出关节力矩
end
```

**关键设计要点:**

1. 运动方程和接触约束作为最高优先级的等式约束, 保证物理一致性
2. 基座跟踪优先于末端执行器跟踪, 确保机器人稳定性
3. 默认姿态任务在 null-space 中执行, 不干扰高优先级任务
4. 力矩和接触力最小化作为正则化项, 避免解的奇异性

### 4.3 控制器 C: 混合力/运动控制 (hybrid_force_motion_control.m)

在浮动基座控制器的基础上, 扩展末端执行器的力控制能力。典型应用场景: 机械臂末端需要在某个方向上施加指定力 (如推墙), 同时在另一个方向上跟踪位置。

**优化变量 (扩展):**

```
x_opt = [q̈; f_c; τ; f_EE] ∈ ℝ⁽¹⁰⁺ⁿᶜ⁺⁷⁺ᵐ⁾
```

新增 `f_EE` 为末端执行器施加的外力, `m` 为末端力维度 (平面情况下 `m = 2`)。

**与控制器 B 的区别:**

| 项目 | 控制器 B | 控制器 C |
|------|----------|----------|
| 优化变量 | `[q̈, f_c, τ]` | `[q̈, f_c, τ, f_EE]` |
| 运动方程 | `M·q̈ - J_cᵀ·f_c - Sᵀ·τ = -b-g` | `M·q̈ - J_cᵀ·f_c - Sᵀ·τ - J_EEᵀ·f_EE = -b-g` |
| 末端控制 | 纯运动跟踪 | x 方向力控 + z 方向位置控 |
| 额外任务 | 无 | 末端力跟踪: `f_EE = f_EE_des` |

**混合力/运动分解:**

在末端执行器坐标系中, 将任务空间分解为:
- **力控方向 (force-controlled)**: x 方向 — 跟踪期望接触力 `f_EE_des_x`
- **运动控方向 (motion-controlled)**: z 方向 — 跟踪期望位置 `p_ee_des_z`

```
选择矩阵:
Σ_f = [1, 0; 0, 0]   % 力控方向 (x)
Σ_m = [0, 0; 0, 1]   % 运动控方向 (z)

满足: Σ_f + Σ_m = I
```

**MATLAB 实现:**

```matlab
function tau = hybrid_force_motion_control(q, dq, params, t)
    %% 动力学量提取 (同控制器 B)
    M    = params.M;
    b    = params.b;
    g    = params.g;
    J_c  = params.J_c;
    dJ_c = params.dJ_c;
    J_ee = params.J_ee;
    dJ_ee = params.dJ_ee;
    
    n_q   = length(q);       % 10
    n_c   = size(J_c, 1);    % 接触力维度
    n_tau = 7;               % 驱动关节数
    n_ee  = size(J_ee, 1);   % 末端力维度 (2)
    n_x   = n_q + n_c + n_tau + n_ee;  % 优化变量总维度
    
    %% 选择矩阵
    S = [zeros(n_tau, 3), eye(n_tau)];
    
    %% 优化变量索引
    idx_qdd  = 1:n_q;
    idx_fc   = n_q+1 : n_q+n_c;
    idx_tau  = n_q+n_c+1 : n_q+n_c+n_tau;
    idx_fee  = n_q+n_c+n_tau+1 : n_x;
    
    %% 任务 1: 运动方程 (含末端力)
    % M*qdd - J_c'*f_c - S'*tau - J_EE'*f_EE = -b - g
    A1 = zeros(n_q, n_x);
    A1(:, idx_qdd) = M;
    A1(:, idx_fc)  = -J_c';
    A1(:, idx_tau) = -S';
    A1(:, idx_fee) = -J_ee';
    b1 = -b - g;
    
    %% 任务 2: 足部接触约束
    A2 = zeros(n_c, n_x);
    A2(:, idx_qdd) = J_c;
    b2 = -dJ_c * dq;
    
    %% 任务 3: 基座运动跟踪 (同控制器 B)
    J_b = eye(3, n_q);
    q_b      = q(1:3);
    dq_b     = dq(1:3);
    q_b_des  = params.q_b_des;
    dq_b_des = params.dq_b_des;
    
    Kp_b = diag([200, 400, 300]);
    Kd_b = diag([40, 80, 60]);
    ddq_b_des = Kp_b * (q_b_des - q_b) + Kd_b * (dq_b_des - dq_b);
    
    A3 = zeros(3, n_x);
    A3(:, idx_qdd) = J_b;
    b3 = ddq_b_des;
```
    
    %% 任务 4: 末端执行器 — 运动控方向 (z 方向位置跟踪)
    % 只取 z 方向 (第2行)
    Sigma_m = [0, 0; 0, 1];  % 运动控选择矩阵
    J_ee_m  = Sigma_m * J_ee;
    dJ_ee_m = Sigma_m * dJ_ee;
    
    p_ee      = params.p_ee;
    dp_ee     = params.dp_ee;
    p_ee_des  = params.p_ee_des;
    dp_ee_des = params.dp_ee_des;
    
    Kp_ee = 150;
    Kd_ee = 30;
    % 只控制 z 方向
    ddp_ee_z_des = Kp_ee * (p_ee_des(2) - p_ee(2)) ...
                 + Kd_ee * (dp_ee_des(2) - dp_ee(2));
    w_ee_m = [0; ddp_ee_z_des] - dJ_ee_m * dq;
    
    A4 = zeros(n_ee, n_x);
    A4(:, idx_qdd) = J_ee_m;
    b4 = w_ee_m;
    
    %% 任务 5: 末端执行器 — 力控方向 (x 方向力跟踪)
    % f_EE_x = f_EE_des_x
    Sigma_f = [1, 0; 0, 0];  % 力控选择矩阵
    f_EE_des = params.f_EE_des;  % 期望末端力 [f_x; f_z]
    
    A5 = zeros(n_ee, n_x);
    A5(:, idx_fee) = Sigma_f;
    b5 = Sigma_f * f_EE_des;
    
    %% 任务 6: 默认姿态
    q0 = params.q_home;
    Kp_posture = 50 * eye(n_q);
    Kd_posture = 10 * eye(n_q);
    ddq_posture = Kp_posture * (q0 - q) - Kd_posture * dq;
    
    A6 = zeros(n_q, n_x);
    A6(:, idx_qdd) = eye(n_q);
    b6 = ddq_posture;
    
    %% 任务 7: 力矩最小化
    A7 = zeros(n_tau, n_x);
    A7(:, idx_tau) = eye(n_tau);
    b7 = zeros(n_tau, 1);
    
    %% 任务 8: 接触力最小化
    A8 = zeros(n_c, n_x);
    A8(:, idx_fc) = eye(n_c);
    b8 = zeros(n_c, 1);
    
    %% 不等式约束 (同控制器 B, 扩展维度)
    tau_max = params.tau_max;
    mu = params.mu;
    
    % 力矩限制
    A_tau_upper = zeros(n_tau, n_x);
    A_tau_upper(:, idx_tau) = eye(n_tau);
    A_tau_lower = zeros(n_tau, n_x);
    A_tau_lower(:, idx_tau) = -eye(n_tau);
    
    % 摩擦锥 (同控制器 B)
    n_contacts = n_c / 2;
    A_friction = [];
    b_friction = [];
    for i = 1:n_contacts
        fn_idx = idx_fc(2*i - 1);
        ft_idx = idx_fc(2*i);
        row1 = zeros(1, n_x); row1(ft_idx) =  1; row1(fn_idx) = -mu;
        row2 = zeros(1, n_x); row2(ft_idx) = -1; row2(fn_idx) = -mu;
        row3 = zeros(1, n_x); row3(fn_idx) = -1;
        A_friction = [A_friction; row1; row2; row3];
        b_friction = [b_friction; 0; 0; 0];
    end
    
    A_ineq = [A_tau_upper; A_tau_lower; A_friction];
    b_ineq = [tau_max; tau_max; b_friction];
    
    %% 组装并求解
    tasks = {};
    tasks{1} = struct('A', A1, 'b', b1, 'type', 'eq');
    tasks{2} = struct('A', A2, 'b', b2, 'type', 'eq');
    tasks{3} = struct('A', A3, 'b', b3, 'type', 'cost');
    tasks{4} = struct('A', A4, 'b', b4, 'type', 'cost');
    tasks{5} = struct('A', A5, 'b', b5, 'type', 'cost');
    tasks{6} = struct('A', A6, 'b', b6, 'type', 'cost');
    tasks{7} = struct('A', A7, 'b', b7, 'type', 'cost');
    tasks{8} = struct('A', A8, 'b', b8, 'type', 'cost');
    
    ineq = struct('A', A_ineq, 'b', b_ineq);
    x_opt = hopt(tasks, ineq);
    
    %% 提取结果
    tau = x_opt(idx_tau);
end
```

**力/运动混合控制的物理直觉:**

当机械臂末端接触墙壁时:
- x 方向 (水平): 无法自由运动, 改为力控制 — 施加期望推力
- z 方向 (竖直): 可以自由运动, 保持位置控制 — 跟踪期望高度

两个方向的控制目标互补, 通过选择矩阵 `Σ_f` 和 `Σ_m` 实现解耦。

---

## 5. 仿真说明

### 5.1 运行仿真

```matlab
%% 步骤 1: 初始化
setup;

%% 步骤 2: 选择控制器
% 在 simulate.m 中修改 controller_type:
%   'jointspace_pid'          — 关节空间 PID
%   'floating_base'           — 浮动基座全身控制
%   'hybrid_force_motion'     — 混合力/运动控制
params.controller_type = 'floating_base';

%% 步骤 3: 运行
simulate;
```

### 5.2 基座运动轨迹规划

浮动基座控制器需要期望的基座轨迹。典型的轨迹规划方式:

```matlab
function [q_b_des, dq_b_des] = base_trajectory(t, params)
    % 基座高度保持恒定
    z_des = params.z_nominal;  % 例如 0.4 m
    
    % 基座水平位置: 缓慢前移
    x_des = params.x_init + 0.1 * t;  % 0.1 m/s
    
    % 基座姿态: 保持水平
    theta_des = 0;
    
    q_b_des  = [x_des; z_des; theta_des];
    dq_b_des = [0.1; 0; 0];
end
```

对于更复杂的运动 (如蹲下、跳跃), 可使用五次多项式插值保证位置、速度、加速度连续:

```matlab
function [p, dp, ddp] = quintic_trajectory(t, t0, tf, p0, pf)
    T = tf - t0;
    s = (t - t0) / T;
    s = max(0, min(1, s));  % 限制在 [0, 1]
    
    % 5次多项式系数 (零初末速度和加速度)
    a0 = p0;
    a3 = 10 * (pf - p0);
    a4 = -15 * (pf - p0);
    a5 = 6 * (pf - p0);
    
    p   = a0 + a3*s^3 + a4*s^4 + a5*s^5;
    dp  = (3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4) / T;
    ddp = (6*a3*s + 12*a4*s^2 + 20*a5*s^3) / T^2;
end
```

### 5.3 可视化结果解读

仿真结束后, 可视化工具会生成以下图表:

```
┌─────────────────────────────────────────────┐
│  Figure 1: 机器人动画                        │
│  - 连杆、关节、接触点的实时动画               │
│  - 接触力矢量 (绿色箭头)                     │
│  - 末端执行器力矢量 (红色箭头)               │
├─────────────────────────────────────────────┤
│  Figure 2: 基座状态                          │
│  - x(t), z(t), θ(t) 及其期望值              │
│  - 跟踪误差                                  │
├─────────────────────────────────────────────┤
│  Figure 3: 关节力矩                          │
│  - 7个关节的力矩时间历程                     │
│  - 力矩限制 (虚线)                           │
├─────────────────────────────────────────────┤
│  Figure 4: 接触力                            │
│  - 法向力 f_n(t)                             │
│  - 切向力 f_t(t)                             │
│  - 摩擦锥边界 (虚线)                         │
├─────────────────────────────────────────────┤
│  Figure 5: 末端执行器 (仅混合控制)           │
│  - 位置跟踪 (z 方向)                         │
│  - 力跟踪 (x 方向)                           │
└─────────────────────────────────────────────┘
```

### 5.4 调试建议

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 机器人倒下 | 基座 PD 增益过低 | 增大 `Kp_b`, `Kd_b` |
| 足部滑动 | 摩擦系数设置过小 | 增大 `mu` 或检查摩擦锥约束 |
| 力矩饱和 | 期望运动过于激进 | 降低轨迹速度或增大力矩限制 |
| QP 无解 | 约束冲突 | 检查任务优先级, 放松低优先级约束 |
| 末端力振荡 | 力控增益不当 | 降低力控带宽, 增加阻尼 |
| 仿真发散 | 时间步长过大 | 减小 `dt` 或使用变步长积分器 |

---

## 6. 思考题

1. **为什么浮动基座的 3 个自由度不能直接驱动?**
   提示: 足式机器人通过足部接触力间接控制基座运动。基座没有电机, 其运动完全由地面反力和关节力矩的耦合效应决定。

2. **如果去掉摩擦锥约束会发生什么?**
   提示: 优化器可能给出物理上不可实现的切向力, 导致仿真中足部滑动或数值不稳定。

3. **层次化 QP 与加权 QP 的区别是什么?**
   提示: 加权 QP 将所有任务合并为一个代价函数 `sum(w_i * ||A_i*x - b_i||^2)`, 无法保证严格优先级。权重调参困难, 且低优先级任务可能干扰高优先级任务。

4. **混合力/运动控制中, 如何确定哪个方向用力控、哪个方向用运动控?**
   提示: 取决于环境约束 — 受约束方向 (如接触墙面的法向) 用力控, 自由方向 (如沿墙面滑动) 用运动控。选择矩阵 `Sigma_f` 和 `Sigma_m` 必须互补且正交。

5. **如果机器人需要抬起一条腿 (单足支撑), 控制器需要做哪些修改?**
   提示: 接触 Jacobian `J_c` 和接触力 `f_c` 的维度需要动态调整。同时需要重新规划 ZMP (Zero Moment Point) 以保证单足支撑下的平衡。

---

## 附录 A: hopt.m 求解器接口

```matlab
function x_opt = hopt(tasks, ineq)
% HOPT  层次化 QP 求解器 (Hierarchical Quadratic Programming)
%
% 输入:
%   tasks  - cell array, 每个元素为 struct:
%            .A    - 任务矩阵
%            .b    - 任务目标向量
%            .type - 'eq' (等式约束) 或 'cost' (最小化目标)
%   ineq   - struct:
%            .A    - 不等式约束矩阵
%            .b    - 不等式约束上界
%            满足: ineq.A * x <= ineq.b
%
% 输出:
%   x_opt  - 最优解向量
%
% 求解流程:
%   对于每一层 k:
%     如果 type == 'eq':
%       将 A_k * x = b_k 加入等式约束集
%     如果 type == 'cost':
%       min ||A_k * x - b_k||^2
%       s.t. 所有已有等式约束
%            ineq.A * x <= ineq.b
%     将当前层的最优残差固定, 传递给下一层
end
```

**使用示例:**

```matlab
% 2层示例: 等式约束 + 代价最小化
tasks = {};
tasks{1} = struct('A', A_eq, 'b', b_eq, 'type', 'eq');
tasks{2} = struct('A', A_cost, 'b', b_cost, 'type', 'cost');

ineq = struct('A', A_ineq, 'b', b_ineq);
x_opt = hopt(tasks, ineq);
```

## 附录 B: 符号速查表

| 符号 | 含义 | 维度 |
|------|------|------|
| `q` | 广义坐标 (generalized coordinates) | 10×1 |
| `dq` / `q̇` | 广义速度 (generalized velocities) | 10×1 |
| `ddq` / `q̈` | 广义加速度 (generalized accelerations) | 10×1 |
| `M(q)` | 质量矩阵 (mass matrix) | 10×10 |
| `b(q, q̇)` | 科氏力/离心力 (Coriolis & centrifugal) | 10×1 |
| `g(q)` | 重力项 (gravity) | 10×1 |
| `S` | 选择矩阵 (selection matrix) | 7×10 |
| `τ` | 关节力矩 (joint torques) | 7×1 |
| `J_c` | 接触 Jacobian (contact Jacobian) | n_c×10 |
| `J̇_c` / `dJ_c` | 接触 Jacobian 时间导数 | n_c×10 |
| `f_c` | 接触力 (contact forces) | n_c×1 |
| `J_EE` | 末端执行器 Jacobian | 2×10 |
| `f_EE` | 末端执行器力 (end-effector force) | 2×1 |
| `μ` | 摩擦系数 (friction coefficient) | scalar |
| `K_p` | 比例增益 (proportional gain) | 对角阵 |
| `K_d` | 微分增益 (derivative gain) | 对角阵 |
| `Σ_f` | 力控选择矩阵 (force selection) | 2×2 |
| `Σ_m` | 运动控选择矩阵 (motion selection) | 2×2 |
| `q₀` / `q_home` | 默认站立姿态 (home posture) | 10×1 |
| `n_c` | 接触力维度 (双足: 4, 单足: 2) | scalar |

## 附录 C: 接触约束力动力学 (CCFD)

`ccfd.m` 实现了带接触约束的正向动力学求解。其核心思路是将接触约束作为附加条件, 与运动方程联立求解。

**问题形式:**

给定当前状态 `(q, q̇)` 和关节力矩 `τ`, 求解广义加速度 `q̈` 和接触力 `f_c`:

```
M(q)·q̈ + b(q, q̇) + g(q) = Sᵀ·τ + J_cᵀ·f_c
J_c·q̈ = -J̇_c·q̇
```

将两个方程联立, 写成矩阵形式:

```
[ M    -J_cᵀ ] [ q̈  ]   [ Sᵀ·τ - b - g ]
[ J_c    0   ] [ f_c ] = [ -J̇_c·q̇       ]
```

这是一个线性方程组 (KKT 系统), 可直接求解:

```matlab
function [qdd, f_c] = ccfd(M, b, g, S, tau, J_c, dJ_c, dq)
    n_q = size(M, 1);
    n_c = size(J_c, 1);
    
    % 构建 KKT 系统
    A = [M,    -J_c';
         J_c,  zeros(n_c, n_c)];
    
    rhs = [S' * tau - b - g;
           -dJ_c * dq];
    
    % 求解
    sol = A \ rhs;
    
    qdd = sol(1:n_q);
    f_c = sol(n_q+1:end);
end
```

**注意事项:**
- 当接触点数量变化时 (如足部离地/着地), `J_c` 的维度会动态变化
- KKT 矩阵可能接近奇异 (如接触 Jacobian 秩亏), 需要正则化处理
- 实际实现中通常加入小的正则化项: `A(n_q+1:end, n_q+1:end) = -eps * eye(n_c)`

## 附录 D: 墙壁接触模型 (wall.m)

墙壁接触模型用于混合力/运动控制场景。当末端执行器接触墙壁时, 产生接触力。

**弹簧-阻尼器模型:**

```matlab
function f_wall = wall_contact(p_ee, dp_ee, params)
    x_wall = params.x_wall;  % 墙壁 x 坐标
    
    % 穿透深度
    delta = p_ee(1) - x_wall;
    
    if delta > 0  % 接触发生
        k_wall = params.k_wall;  % 墙壁刚度 (例如 5000 N/m)
        d_wall = params.d_wall;  % 墙壁阻尼 (例如 100 Ns/m)
        
        % 法向接触力 (x 方向, 指向机器人)
        f_n = -k_wall * delta - d_wall * dp_ee(1);
        f_n = min(f_n, 0);  % 只能推, 不能拉
        
        % 切向无摩擦 (简化)
        f_t = 0;
        
        f_wall = [f_n; f_t];
    else
        f_wall = [0; 0];  % 无接触
    end
end
```

**在动力学方程中的体现:**

墙壁接触力通过末端执行器 Jacobian 映射到广义坐标空间:

```
M·q̈ + b + g = Sᵀ·τ + J_cᵀ·f_c + J_EEᵀ·f_wall
```

在混合力/运动控制器中, `f_wall` 即为优化变量 `f_EE`。

---

## 附录 E: 三种控制器对比总结

### E.1 控制器架构对比

```
控制器 A: 关节空间 PID
┌──────────┐    ┌─────────┐    ┌──────────┐
│ q_des    │───>│ PD 控制 │───>│ τ        │
│ (期望角度)│    │         │    │ (关节力矩)│
└──────────┘    └─────────┘    └──────────┘

控制器 B: 浮动基座全身控制
┌──────────┐    ┌───────────────┐    ┌──────────┐
│ 基座轨迹  │───>│               │    │          │
│ 末端轨迹  │───>│ 层次化 QP     │───>│ τ        │
│ 默认姿态  │───>│ (7层优先级)    │    │ (关节力矩)│
│ 动力学模型│───>│               │    │          │
└──────────┘    └───────────────┘    └──────────┘

控制器 C: 混合力/运动控制
┌──────────┐    ┌───────────────┐    ┌──────────┐
│ 基座轨迹  │───>│               │───>│ τ        │
│ 末端位置  │───>│ 层次化 QP     │    │ (关节力矩)│
│ 末端力    │───>│ (8层优先级)    │───>│ f_EE     │
│ 动力学模型│───>│               │    │ (末端力)  │
└──────────┘    └───────────────┘    └──────────┘
```

### E.2 性能对比

| 指标 | 控制器 A (PID) | 控制器 B (浮动基座) | 控制器 C (混合力/运动) |
|------|---------------|-------------------|---------------------|
| 实现复杂度 | 低 | 高 | 最高 |
| 基座位姿控制 | 不可控 | 可控 | 可控 |
| 接触力优化 | 无 | 有 | 有 |
| 末端力控制 | 无 | 无 | 有 |
| 物理一致性 | 不保证 | 保证 | 保证 |
| 摩擦锥约束 | 不考虑 | 满足 | 满足 |
| 力矩限制 | 简单饱和 | QP 约束 | QP 约束 |
| 适用场景 | 调试/简单站立 | 运动控制 | 接触操作 |

### E.3 优化变量维度对比

```
控制器 A:  无优化, 直接计算 τ ∈ ℝ⁷
控制器 B:  x = [q̈, f_c, τ]ᵀ ∈ ℝ²¹  (10 + 4 + 7)
控制器 C:  x = [q̈, f_c, τ, f_EE]ᵀ ∈ ℝ²³  (10 + 4 + 7 + 2)
```

---

## 附录 F: 常见 MATLAB 实现技巧

### F.1 Jacobian 时间导数的数值计算

当解析求导困难时, 可用数值差分近似 `J̇_c`:

```matlab
function dJ = numerical_jacobian_dot(J_func, q, dq, dt_num)
    % J_func: 返回 Jacobian 的函数句柄 J = J_func(q)
    % dt_num: 数值差分步长 (例如 1e-6)
    
    if nargin < 4
        dt_num = 1e-6;
    end
    
    J_current = J_func(q);
    J_perturbed = J_func(q + dt_num * dq);
    
    dJ = (J_perturbed - J_current) / dt_num;
end
```

### F.2 安全的矩阵求逆

在求解 KKT 系统或计算 Jacobian 伪逆时, 避免直接求逆:

```matlab
% 不推荐: 直接求逆
% x = inv(A) * b;

% 推荐: 使用左除运算符
x = A \ b;

% 对于可能奇异的矩阵, 使用伪逆
x = pinv(A) * b;

% 或使用带阻尼的伪逆 (Damped Least Squares)
lambda = 0.01;  % 阻尼系数
x = A' * ((A * A' + lambda^2 * eye(size(A,1))) \ b);
```

### F.3 选择矩阵的构建

```matlab
% 方法 1: 直接构建
n_tau = 7;
n_q = 10;
S = [zeros(n_tau, 3), eye(n_tau)];  % 7x10

% 方法 2: 从单位阵中提取行
S = eye(n_q);
S = S(4:end, :);  % 取第4到第10行, 得到 7x10

% 验证: S' * tau 应该在前3个分量为零
tau_test = ones(7, 1);
result = S' * tau_test;  % [0; 0; 0; 1; 1; 1; 1; 1; 1; 1]
```

### F.4 QP 任务的快速构建模板

```matlab
% 通用模板: 构建一个任务层
function task = build_task(A_block, idx, b_vec, n_x, type)
    % A_block: 任务矩阵 (只对应部分优化变量)
    % idx: A_block 对应的优化变量索引
    % b_vec: 目标向量
    % n_x: 优化变量总维度
    % type: 'eq' 或 'cost'
    
    A = zeros(size(A_block, 1), n_x);
    A(:, idx) = A_block;
    
    task = struct('A', A, 'b', b_vec, 'type', type);
end

% 使用示例
task_eom = build_task(M, idx_qdd, -b-g, n_x, 'eq');
% 注意: 对于涉及多个变量块的任务, 需要手动构建完整的 A 矩阵
```

---

## 附录 G: 参考文献与扩展阅读

1. **Bellicoso, C.D. et al.** "Dynamic Locomotion Through Online Nonlinear Motion Optimization for Quadrupedal Robots." IEEE Robotics and Automation Letters, 2018.
   - 四足机器人全身控制的经典论文

2. **Hutter, M. et al.** "ANYmal - A Highly Mobile and Dynamic Quadrupedal Robot." IEEE/RSJ IROS, 2016.
   - ETH RSL 的 ANYmal 四足机器人平台

3. **Raibert, M.H.** "Legged Robots That Balance." MIT Press, 1986.
   - 足式机器人控制的奠基性著作

4. **Siciliano, B. et al.** "Robotics: Modelling, Planning and Control." Springer, 2009.
   - 第9章: 力控制 (Force Control), 混合力/运动控制的理论基础

5. **De Luca, A. & Manes, C.** "Modeling of Robots in Contact with a Dynamic Environment." IEEE Transactions on Robotics and Automation, 1994.
   - 接触动力学建模

6. **ETH Robot Dynamics 课程主页:**
   - https://rsl.ethz.ch/education-students/lectures/robot-dynamics.html

---

> 本文档基于 ETH Zurich Robot Dynamics 课程 Exercise 3 编写。
> ProNEu 框架由 ETH Robotic Systems Lab (RSL) 开发。
> 仅供学习参考, 请勿直接提交为作业答案。
