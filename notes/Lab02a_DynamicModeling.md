# Lab 02a: ABB IRB 120 动力学建模 (Dynamic Modeling)

> ETH Zurich "Robot Dynamics" 课程 — Exercise 2a
>
> 目标: 推导并实现 ABB IRB 120 六自由度工业机器人的完整动力学模型

---

## 1. 实验概述 (Lab Overview)

本实验的核心任务是为 ABB IRB 120 机械臂推导和实现**刚体动力学运动方程 (Equations of Motion)**。我们采用**投影牛顿-欧拉法 (Projected Newton-Euler Method)** 结合 MATLAB **Symbolic Math Toolbox** 进行符号推导, 最终得到标准形式的运动方程:

$$
M(q)\ddot{q} + b(q, \dot{q}) + g(q) = \tau
$$

其中:
- $q \in \mathbb{R}^6$ — 广义坐标 (generalized coordinates), 即六个关节角度
- $M(q) \in \mathbb{R}^{6 \times 6}$ — 质量矩阵 (mass/inertia matrix), 对称正定
- $b(q, \dot{q}) \in \mathbb{R}^6$ — 科里奥利力与离心力向量 (Coriolis and centrifugal terms)
- $g(q) \in \mathbb{R}^6$ — 重力向量 (gravity vector)
- $\tau \in \mathbb{R}^6$ — 广义力/关节力矩 (generalized forces/joint torques)

### 工作流程

```
generate_params.m   →  机器人物理参数 (质量, 惯性, 几何)
        ↓
generate_kin.m      →  正运动学, 齐次变换矩阵 T_0k
        ↓
generate_jac.m      →  平动/转动雅可比矩阵 Jp_k, Jr_k
        ↓
generate_eom.m      →  运动方程 M, b, g, H   ← 【你需要实现】
        ↓
matlabFunction()    →  导出为可调用的 .m 函数
        ↓
Simulink 仿真验证
```

---

## 2. 文件结构 (File Structure)

```
exercise_2a/
├── generate_model.m              % 主脚本: 依次调用各生成器, 导出模型
├── model/
│   └── utils/
│       ├── generate_params.m     % 定义 ABB IRB 120 的 DH 参数、质量、惯性张量
│       ├── generate_kin.m        % 计算各连杆的齐次变换矩阵 T_0k
│       ├── generate_jac.m        % 计算各连杆的平动/转动雅可比矩阵
│       ├── generate_gen_cor.m    % 定义广义坐标 q, dq, ddq (符号变量)
│       └── dAdt.m                % 工具函数: 计算矩阵对时间的导数
├── problems/
│   └── generate_eom.m            % 【需要实现】生成运动方程
├── solutions/                    % 参考解 (用于对比验证)
├── abb_irb120.mdl                % Simulink 仿真模型
└── run_simulation.m              % 运行仿真脚本
```

### 关键文件说明

**generate_params.m** — 定义机器人的全部物理参数:

```matlab
% 连杆质量 (link masses) [kg]
dyn.m = [m1, m2, m3, m4, m5, m6];

% 各连杆在自身坐标系下的惯性张量 (inertia tensors in body frame)
dyn.I{k} = [Ixx, Ixy, Ixz;
            Ixy, Iyy, Iyz;
            Ixz, Iyz, Izz];  % 对称矩阵, k = 1..6

% 重力加速度向量 (gravity acceleration in world frame)
dyn.g_acc = [0; 0; -9.81];
```

**generate_kin.m** — 计算运动学:

```matlab
% T_0k: 从世界坐标系到第 k 个连杆质心的齐次变换矩阵
kin.T{k} = T_01 * T_12 * ... * T_(k-1)k;

% 提取旋转矩阵和位置向量
kin.C{k} = T_0k(1:3, 1:3);   % C_0k: 旋转矩阵 (rotation matrix)
kin.r{k} = T_0k(1:3, 4);     % r_k:  质心位置 (CoM position)
```

**generate_jac.m** — 计算雅可比矩阵:

```matlab
% Jp_k: 第 k 个连杆质心的平动雅可比 (translational Jacobian)
% Jr_k: 第 k 个连杆的转动雅可比 (rotational Jacobian)
jac.Jp{k} = jacobian(kin.r{k}, gen_cor.q);   % 3×6
jac.Jr{k} = ...;                               % 3×6 (由旋转轴构造)
```

**dAdt.m** — 矩阵时间导数工具:

```matlab
function dAdt = dAdt(A, gen_cor)
% 计算符号矩阵 A 对时间的全导数
% 利用链式法则: dA/dt = Σ (∂A/∂qᵢ)·q̇ᵢ
    q  = gen_cor.q;
    dq = gen_cor.dq;
    dAdt = sym(zeros(size(A)));
    for i = 1:length(q)
        dAdt = dAdt + diff(A, q(i)) * dq(i);
    end
end
```

---

## 3. 需要实现的函数 (Functions to Implement)

### 3a. generate_eom — 生成运动方程 (核心任务)

这是本实验最关键的函数。你需要利用已有的运动学和雅可比信息, 通过**投影牛顿-欧拉法**推导出完整的运动方程。

```matlab
function eom = generate_eom(gen_cor, kin, dyn, jac)
% generate_eom  生成 ABB IRB 120 的运动方程
%
% 输入:
%   gen_cor — 广义坐标结构体
%       .q    [6×1 sym] 关节角度
%       .dq   [6×1 sym] 关节角速度
%       .ddq  [6×1 sym] 关节角加速度
%   kin — 运动学结构体
%       .C{k}  [3×3 sym] 第 k 个连杆的旋转矩阵 C_0k (world frame)
%       .r{k}  [3×1 sym] 第 k 个连杆质心位置 r_k (world frame)
%   dyn — 动力学参数结构体
%       .m(k)      标量, 第 k 个连杆质量
%       .I{k}      [3×3] 第 k 个连杆在 body frame 下的惯性张量
%       .g_acc     [3×1] 重力加速度向量
%   jac — 雅可比矩阵结构体
%       .Jp{k}  [3×6 sym] 第 k 个连杆质心的平动雅可比
%       .Jr{k}  [3×6 sym] 第 k 个连杆的转动雅可比
%
% 输出:
%   eom — 运动方程结构体
%       .M    [6×6 sym] 质量矩阵
%       .b    [6×1 sym] 科里奥利和离心力向量
%       .g    [6×1 sym] 重力向量
%       .H    [1×1 sym] 哈密顿量 (总能量)

    q   = gen_cor.q;
    dq  = gen_cor.dq;
    nDof = length(q);   % 自由度数 = 6

    %% =============================================
    %  质量矩阵 M(q)  —  Mass/Inertia Matrix
    %  =============================================
    %  公式:
    %    M = Σ_{k=1}^{6} [ mₖ · Jp_kᵀ · Jp_k
    %                     + Jr_kᵀ · C_Ikᵀ · Iₖ · C_Ik · Jr_k ]
    %
    %  其中 C_Ik = kin.C{k} 是从 body frame 到 world frame 的旋转矩阵,
    %  Iₖ 是在 body frame 下表示的惯性张量。
    %  乘积 C_Ik' * Iₖ * C_Ik 将惯性张量变换到 world frame 后再投影。
    %  注意: 这里 C_Ik' 表示转置, 即 world→body 的旋转。

    M = sym(zeros(nDof, nDof));
    for k = 1:nDof
        Jp_k = jac.Jp{k};          % 3×6 平动雅可比
        Jr_k = jac.Jr{k};          % 3×6 转动雅可比
        m_k  = dyn.m(k);           % 标量质量
        I_k  = dyn.I{k};           % 3×3 body frame 惯性张量
        C_Ik = kin.C{k};           % 3×3 旋转矩阵 (body → world)

        % 平动贡献 + 转动贡献
        M = M + m_k * (Jp_k.' * Jp_k) ...
              + Jr_k.' * C_Ik.' * I_k * C_Ik * Jr_k;
    end
    M = simplify(M);

    %% =============================================
    %  重力向量 g(q)  —  Gravity Vector
    %  =============================================
    %  公式:
    %    g = -Σ_{k=1}^{6} Jp_kᵀ · mₖ · g_acc
    %
    %  物理含义: 重力在广义坐标方向上的投影 (取负号是因为
    %  运动方程中 g(q) 出现在左侧, 与 τ 相对)

    g_acc = dyn.g_acc;             % [0; 0; -9.81]
    g_vec = sym(zeros(nDof, 1));
    for k = 1:nDof
        Jp_k = jac.Jp{k};
        m_k  = dyn.m(k);
        g_vec = g_vec - Jp_k.' * m_k * g_acc;
    end
    g_vec = simplify(g_vec);

    %% =============================================
    %  科里奥利和离心力向量 b(q, q̇)
    %  Coriolis and Centrifugal Terms
    %  =============================================
    %  公式:
    %    b = Σ_{k=1}^{6} [ mₖ · Jp_kᵀ · dJp_k · q̇
    %                     + Jr_kᵀ · C_Ikᵀ · Iₖ · C_Ik · dJr_k · q̇
    %                     + Jr_kᵀ · (ωₖ × Iₖ_world · ωₖ) ]
    %
    %  三项分别对应:
    %    (1) 平动加速度的科里奥利/离心效应
    %    (2) 转动加速度的科里奥利/离心效应
    %    (3) 陀螺力矩 (gyroscopic term): ω × Iω

    b = sym(zeros(nDof, 1));
    for k = 1:nDof
        Jp_k  = jac.Jp{k};
        Jr_k  = jac.Jr{k};
        m_k   = dyn.m(k);
        I_k   = dyn.I{k};
        C_Ik  = kin.C{k};

        % 雅可比矩阵的时间导数 (利用 dAdt 工具函数)
        dJp_k = dAdt(Jp_k, gen_cor);   % 3×6
        dJr_k = dAdt(Jr_k, gen_cor);   % 3×6

        % 将惯性张量变换到 world frame
        I_k_world = C_Ik.' * I_k * C_Ik;

        % 第 k 个连杆的角速度 (world frame)
        omega_k = Jr_k * dq;           % 3×1

        % 三项累加
        b = b + m_k * Jp_k.' * dJp_k * dq ...        % 平动项
              + Jr_k.' * I_k_world * dJr_k * dq ...   % 转动项
              + Jr_k.' * cross(omega_k, I_k_world * omega_k);  % 陀螺项
    end
    b = simplify(b);

    %% =============================================
    %  哈密顿量 H(q, q̇)  —  Total Energy
    %  =============================================
    %  公式:
    %    H = T + U
    %    T = ½ q̇ᵀ M q̇           (动能, kinetic energy)
    %    U = -Σ_{k=1}^{6} rₖᵀ · mₖ · g_acc   (势能, potential energy)
    %
    %  注意: g_acc = [0;0;-9.81], 所以 U = Σ mₖ·g·hₖ (高度越高势能越大)

    T = 0.5 * dq.' * M * dq;      % 动能

    U = sym(0);                     % 势能
    for k = 1:nDof
        r_k = kin.r{k};            % 第 k 个连杆质心位置
        m_k = dyn.m(k);
        U = U - r_k.' * m_k * g_acc;
    end

    H = simplify(T + U);

    %% 输出
    eom.M = M;
    eom.b = b;
    eom.g = g_vec;
    eom.H = H;
end
```


### 3b. 导出的函数接口 (Exported Function Interfaces)

`generate_model.m` 主脚本会调用 `matlabFunction()` 将符号表达式导出为高效的数值函数。你需要理解这些导出函数的接口, 以便在仿真和测试中正确调用。

**M_fun(q) — 质量矩阵函数**

```matlab
function M = M_fun(q)
% M_fun  计算给定关节角度下的质量矩阵
%
% 输入:
%   q  [6×1 double] 关节角度 (rad)
%
% 输出:
%   M  [6×6 double] 质量矩阵, 对称正定
%
% 使用示例:
%   q = [0; pi/4; -pi/3; 0; pi/6; 0];
%   M = M_fun(q);
%   eig(M)   % 所有特征值应 > 0 (正定性验证)

    % 由 matlabFunction() 自动生成
    % 内部展开为 q(1)..q(6) 的三角函数表达式
end
```

**b_fun(q, dq) — 非线性项函数**

```matlab
function b = b_fun(q, dq)
% b_fun  计算科里奥利力和离心力向量
%
% 输入:
%   q   [6×1 double] 关节角度 (rad)
%   dq  [6×1 double] 关节角速度 (rad/s)
%
% 输出:
%   b   [6×1 double] 科里奥利和离心力向量
%
% 性质:
%   - b 与 dq 呈二次关系: b(q, α·dq) = α²·b(q, dq)
%   - 当 dq = 0 时, b = 0 (静止时无科里奥利/离心力)

    % 由 matlabFunction() 自动生成
end
```

**g_fun(q) — 重力向量函数**

```matlab
function g = g_fun(q)
% g_fun  计算重力向量
%
% 输入:
%   q  [6×1 double] 关节角度 (rad)
%
% 输出:
%   g  [6×1 double] 重力向量
%
% 性质:
%   - 仅依赖于 q, 不依赖于 dq
%   - 物理含义: 维持当前姿态所需的关节力矩 (静力学平衡)
%   - τ_hold = g(q) 即可保持机器人静止

    % 由 matlabFunction() 自动生成
end
```

**hamiltonian_fun(q, dq) — 能量函数**

```matlab
function H = hamiltonian_fun(q, dq)
% hamiltonian_fun  计算系统总能量 (哈密顿量)
%
% 输入:
%   q   [6×1 double] 关节角度 (rad)
%   dq  [6×1 double] 关节角速度 (rad/s)
%
% 输出:
%   H   标量, 系统总机械能 = 动能 T + 势能 U
%
% 用途:
%   - 能量守恒验证: 无外力 (τ=0) 时 dH/dt = 0
%   - 数值积分精度检验: H 的漂移量反映积分误差

    % 由 matlabFunction() 自动生成
end
```

### 3c. generate_model.m 主脚本流程

```matlab
%% generate_model.m — 主脚本: 生成 ABB IRB 120 完整动力学模型
clear; clc;

%% Step 1: 定义广义坐标 (符号变量)
gen_cor = generate_gen_cor();
% gen_cor.q   = [q1; q2; q3; q4; q5; q6]   — sym
% gen_cor.dq  = [dq1; dq2; dq3; dq4; dq5; dq6]
% gen_cor.ddq = [ddq1; ddq2; ddq3; ddq4; ddq5; ddq6]

%% Step 2: 定义机器人参数
dyn = generate_params();
% dyn.m    — 连杆质量
% dyn.I    — 惯性张量 (cell array)
% dyn.g_acc — 重力加速度

%% Step 3: 计算运动学
kin = generate_kin(gen_cor, dyn);
% kin.T{k} — 齐次变换矩阵
% kin.C{k} — 旋转矩阵
% kin.r{k} — 质心位置

%% Step 4: 计算雅可比矩阵
jac = generate_jac(gen_cor, kin, dyn);
% jac.Jp{k} — 平动雅可比
% jac.Jr{k} — 转动雅可比

%% Step 5: 生成运动方程 【调用你实现的函数】
eom = generate_eom(gen_cor, kin, dyn, jac);

%% Step 6: 导出为数值函数
fprintf('Exporting M_fun...\n');
matlabFunction(eom.M, 'File', 'model/M_fun', 'Vars', {gen_cor.q});

fprintf('Exporting b_fun...\n');
matlabFunction(eom.b, 'File', 'model/b_fun', 'Vars', {gen_cor.q, gen_cor.dq});

fprintf('Exporting g_fun...\n');
matlabFunction(eom.g, 'File', 'model/g_fun', 'Vars', {gen_cor.q});

fprintf('Exporting hamiltonian_fun...\n');
matlabFunction(eom.H, 'File', 'model/hamiltonian_fun', ...
               'Vars', {gen_cor.q, gen_cor.dq});

fprintf('Model generation complete.\n');
```

---

## 4. 关键理论 (Key Theory)

### 4a. 投影牛顿-欧拉法 (Projected Newton-Euler Method)

这是本实验采用的动力学建模方法。其核心思想是: 先对每个刚体写出牛顿-欧拉方程, 再通过雅可比矩阵投影到广义坐标空间。

**单个刚体 k 的牛顿-欧拉方程 (在 world frame 下):**

```
线性动量定理:  fₖ = mₖ · aₛₖ           (Newton)
角动量定理:    τₖ = Iₖ · α_k + ωₖ × Iₖωₖ   (Euler)
```

其中:
- $a_{s_k}$ 是第 k 个连杆质心的加速度
- $\alpha_k$ 是角加速度
- $\omega_k$ 是角速度
- $I_k$ 是在 world frame 下表示的惯性张量

**利用雅可比矩阵表达速度和加速度:**

```
质心速度:   vₛₖ = Jp_k · q̇
质心加速度: aₛₖ = Jp_k · q̈ + dJp_k · q̇

角速度:     ωₖ  = Jr_k · q̇
角加速度:   αₖ  = Jr_k · q̈ + dJr_k · q̇
```

**虚功原理 (Principle of Virtual Work) 投影:**

对于虚位移 $\delta q$, 虚功为:

$$
\delta W = \sum_{k=1}^{6} \left[ \delta r_k^T \cdot f_k + \delta \phi_k^T \cdot \tau_k \right]
$$

其中 $\delta r_k = J_{p_k} \cdot \delta q$, $\delta \phi_k = J_{r_k} \cdot \delta q$, 代入并整理得:

$$
\delta q^T \left[ \sum_{k=1}^{6} \left( J_{p_k}^T f_k + J_{r_k}^T \tau_k \right) \right] = \delta q^T \cdot \tau_{ext}
$$

由于 $\delta q$ 任意, 括号内即为广义力, 展开后得到标准运动方程。

**从牛顿-欧拉到 M, b, g 的推导过程:**

```matlab
% 对第 k 个刚体:
%   fₖ = mₖ·(Jp_k·q̈ + dJp_k·q̇) - mₖ·g_acc
%   τₖ = Iₖ·(Jr_k·q̈ + dJr_k·q̇) + ωₖ×(Iₖ·ωₖ)
%
% 投影到广义坐标:
%   Jp_kᵀ·fₖ + Jr_kᵀ·τₖ = τ_ext_k
%
% 对所有 k 求和, 整理为 M·q̈ + b + g = τ:
%
%   M·q̈ 项:  Σ [mₖ·Jp_kᵀ·Jp_k + Jr_kᵀ·Iₖ·Jr_k] · q̈
%   b 项:     Σ [mₖ·Jp_kᵀ·dJp_k·q̇ + Jr_kᵀ·Iₖ·dJr_k·q̇ + Jr_kᵀ·(ωₖ×Iₖωₖ)]
%   g 项:     Σ [-Jp_kᵀ·mₖ·g_acc]
```

### 4b. 质量矩阵的性质 (Properties of the Mass Matrix)

质量矩阵 $M(q)$ 具有以下重要性质:

**1. 对称性 (Symmetry): $M = M^T$**

```matlab
% 验证对称性
M_test = M_fun(q_test);
assert(norm(M_test - M_test', 'fro') < 1e-10, '质量矩阵不对称!');
```

这是因为动能 $T = \frac{1}{2}\dot{q}^T M \dot{q}$ 是标量, 而 $\dot{q}^T M \dot{q} = \dot{q}^T M^T \dot{q}$, 所以 $M$ 必须对称。

**2. 正定性 (Positive Definiteness): $M > 0$**

```matlab
% 验证正定性
eigenvalues = eig(M_test);
assert(all(eigenvalues > 0), '质量矩阵不正定!');
fprintf('最小特征值: %.6f\n', min(eigenvalues));
fprintf('最大特征值: %.6f\n', max(eigenvalues));
fprintf('条件数: %.2f\n', max(eigenvalues)/min(eigenvalues));
```

正定性保证了: 任何非零速度 $\dot{q} \neq 0$ 都对应正的动能 $T > 0$。

**3. 配置依赖性 (Configuration Dependence): $M = M(q)$**

质量矩阵随关节角度变化。例如, 当机械臂完全伸展时, 关于基座关节的等效惯量最大; 当折叠时最小。

```matlab
% 观察 M 随配置变化
q_extended = [0; 0; 0; 0; 0; 0];        % 伸展构型
q_folded   = [0; pi/2; -pi/2; 0; 0; 0]; % 折叠构型
M_ext = M_fun(q_extended);
M_fld = M_fun(q_folded);
fprintf('伸展构型 M(1,1) = %.4f\n', M_ext(1,1));  % 较大
fprintf('折叠构型 M(1,1) = %.4f\n', M_fld(1,1));  % 较小
```



### 4c. 科里奥利和离心力 (Coriolis and Centrifugal Forces)

向量 $b(q, \dot{q})$ 包含三个物理来源不同的贡献项:

**第一项: 平动雅可比导数贡献**

$$
b_1 = \sum_{k=1}^{6} m_k \cdot J_{p_k}^T \cdot \dot{J}_{p_k} \cdot \dot{q}
$$

物理含义: 由于连杆质心的运动路径弯曲 (雅可比随配置变化), 即使关节角加速度为零, 质心仍会产生加速度。这就是离心力和科里奥利力的平动部分。

**第二项: 转动雅可比导数贡献**

$$
b_2 = \sum_{k=1}^{6} J_{r_k}^T \cdot I_k^{world} \cdot \dot{J}_{r_k} \cdot \dot{q}
$$

物理含义: 类似于第一项, 但作用在转动自由度上。转动雅可比的变化导致角加速度, 进而产生力矩。

**第三项: 陀螺力矩 (Gyroscopic Term)**

$$
b_3 = \sum_{k=1}^{6} J_{r_k}^T \cdot (\omega_k \times I_k^{world} \cdot \omega_k)
$$

物理含义: 这是欧拉方程中的经典陀螺效应项。旋转刚体的角动量方向变化会产生力矩, 即使角速度大小不变。这一项在高速旋转时尤为显著。



**科里奥利力 vs 离心力的区分:**

在 $b(q, \dot{q})$ 中, 可以将其写成 $b = C(q, \dot{q}) \cdot \dot{q}$ 的形式, 其中 $C$ 是科里奥利矩阵。$C$ 的对角元素对应离心力 (centrifugal), 非对角元素对应科里奥利力 (Coriolis):



**重要性质: dM/dt - 2C 是反对称矩阵**

$$
\dot{q}^T (\dot{M} - 2C) \dot{q} = 0
$$

这个性质在自适应控制和无源性 (passivity) 分析中非常重要。

### 4d. 能量守恒验证 (Energy Conservation)

哈密顿量 (总机械能) 定义为:

$$
H(q, \dot{q}) = T(q, \dot{q}) + U(q)
$$

其中:
- 动能: $T = \frac{1}{2} \dot{q}^T M(q) \dot{q}$
- 势能: $U = -\sum_{k=1}^{6} r_k^T \cdot m_k \cdot g_{acc}$

**能量守恒定理:**

当没有外力作用 ($\tau = 0$) 时, 系统总能量守恒:

$$
\frac{dH}{dt} = 0
$$

**证明思路:**

$$
\frac{dH}{dt} = \frac{\partial T}{\partial \dot{q}} \ddot{q} + \frac{\partial T}{\partial q} \dot{q} + \frac{\partial U}{\partial q} \dot{q}
$$

利用运动方程 $M\ddot{q} = \tau - b - g$ 和 $\dot{M} - 2C$ 反对称的性质, 可以证明当 $\tau = 0$ 时 $\frac{dH}{dt} = 0$。

**数值验证方法:**

```matlab
% 在 Simulink 仿真中验证能量守恒
% 设置: tau = 0 (无外力), 给定初始 q0 和 dq0

q0  = [0.1; 0.2; -0.3; 0.1; 0.05; 0];   % 初始角度
dq0 = [0.5; -0.3; 0.2; 0.1; -0.1; 0.4]; % 初始角速度

% 仿真后提取 q(t) 和 dq(t), 计算 H(t)
H_series = zeros(length(t), 1);
for i = 1:length(t)
    H_series(i) = hamiltonian_fun(q_log(:,i), dq_log(:,i));
end

% 检查 H 是否恒定
H_drift = max(H_series) - min(H_series);
fprintf('能量漂移: %.2e J
', H_drift);
fprintf('相对漂移: %.2e
', H_drift / H_series(1));

% 绘图
figure;
plot(t, H_series, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('总能量 H (J)');
title('能量守恒验证 (tau = 0)');
grid on;

% 期望: H_drift < 1e-6 (数值积分误差范围内)
```

---

## 5. Simulink 仿真 (Simulation)

### 5a. 仿真模型结构

`abb_irb120.mdl` 包含以下模块:

```
+-----------------------------------------------------+
|                  Simulink Model                      |
|                                                      |
|  +----------+    +--------------+    +-----------+   |
|  | tau      |--->|  Forward     |--->| Integrator|   |
|  | (input)  |    |  Dynamics    |    | ddq->dq->q|   |
|  +----------+    |              |    +-----------+   |
|                  | ddq = M^{-1} |         |          |
|                  |  (tau-b-g)   |         |          |
|                  +--------------+    +----v----+     |
|                       ^              |  Scope  |     |
|                       |              | (output)|     |
|                       +--------------+---------+     |
|                        feedback: q, dq               |
+-----------------------------------------------------+
```

**Forward Dynamics (正动力学) 核心计算:**

```matlab
function ddq = forward_dynamics(q, dq, tau)
% forward_dynamics  计算关节角加速度
%   ddq = M(q)^{-1} * [tau - b(q,dq) - g(q)]

    M = M_fun(q);
    b = b_fun(q, dq);
    g = g_fun(q);

    ddq = M \ (tau - b - g);   % 等价于 inv(M) * (tau - b - g), 但数值更稳定
end
```

### 5b. 仿真实验

**实验 1: 自由落体 (Free Fall)**

```matlab
% 设置: tau = 0, 从水平位置释放
% 预期: 机械臂在重力作用下摆动, 总能量守恒

tau = zeros(6, 1);
q0  = [0; 0; 0; 0; 0; 0];    % 水平初始位置
dq0 = zeros(6, 1);            % 静止释放
```

**实验 2: 重力补偿 (Gravity Compensation)**

```matlab
% 设置: tau = g(q), 即力矩恰好抵消重力
% 预期: 机械臂保持静止 (ddq = 0, dq = 0)

% 在 Simulink 中使用 MATLAB Function block:
function tau = gravity_comp(q)
    tau = g_fun(q);
end

% 验证: q(t) = q0 (恒定)
```

**实验 3: 关节空间 PD 控制 (Joint Space PD Control)**

```matlab
% 设置: 简单 PD 控制器跟踪目标位置
q_des = [pi/4; pi/3; -pi/6; 0; pi/4; 0];  % 目标角度
Kp = diag([100, 100, 80, 50, 50, 30]);     % 比例增益
Kd = diag([20, 20, 15, 10, 10, 5]);        % 微分增益

function tau = pd_controller(q, dq, q_des, Kp, Kd)
    e  = q_des - q;       % 位置误差
    de = -dq;             % 速度误差 (目标速度为 0)
    tau = Kp * e + Kd * de + g_fun(q);  % PD + 重力补偿
end
```


---

## 6. 测试方法 (Testing)

### 6a. 能量守恒检验

这是验证运动方程正确性的最可靠方法:

```matlab
function test_energy_conservation()
% test_energy_conservation  验证 tau=0 时总能量守恒
%
% 原理: 如果 M, b, g 推导正确, 则无外力时 dH/dt = 0

    % 随机初始条件
    rng(42);
    q0  = randn(6, 1) * 0.5;
    dq0 = randn(6, 1) * 0.3;

    % 数值积分 (ODE45)
    tspan = [0, 5];
    x0 = [q0; dq0];

    [t, x] = ode45(@(t, x) robot_ode(t, x, zeros(6,1)), tspan, x0);

    % 计算能量时间序列
    H = zeros(length(t), 1);
    for i = 1:length(t)
        qi  = x(i, 1:6).';
        dqi = x(i, 7:12).';
        H(i) = hamiltonian_fun(qi, dqi);
    end

    % 检验
    H_error = max(abs(H - H(1)));
    fprintf('最大能量偏差: %.2e J
', H_error);
    fprintf('相对偏差:     %.2e
', H_error / abs(H(1)));

    assert(H_error / abs(H(1)) < 1e-6, ...
           '能量不守恒! 请检查 M, b, g 的推导。');
    fprintf('能量守恒检验通过!
');
end

function dx = robot_ode(~, x, tau)
% robot_ode  机器人状态方程 (用于 ODE45)
    q  = x(1:6);
    dq = x(7:12);

    M = M_fun(q);
    b = b_fun(q, dq);
    g = g_fun(q);

    ddq = M \ (tau - b - g);
    dx = [dq; ddq];
end
```

### 6b. 与参考解对比 (Solution Comparison)

```matlab
function test_against_solution()
% test_against_solution  与参考解进行数值对比

    % 多组随机测试点
    rng(123);
    n_tests = 50;
    tol = 1e-8;

    for i = 1:n_tests
        q  = randn(6, 1);
        dq = randn(6, 1);

        % 你的实现
        M_yours = M_fun(q);
        b_yours = b_fun(q, dq);
        g_yours = g_fun(q);
        H_yours = hamiltonian_fun(q, dq);

        % 参考解 (solutions/ 目录下)
        M_ref = M_fun_solution(q);
        b_ref = b_fun_solution(q, dq);
        g_ref = g_fun_solution(q);
        H_ref = hamiltonian_fun_solution(q, dq);

        % 对比
        assert(norm(M_yours - M_ref, 'fro') < tol, ...
               sprintf('Test %d: M 不匹配! 误差 = %.2e', i, norm(M_yours - M_ref, 'fro')));
        assert(norm(b_yours - b_ref) < tol, ...
               sprintf('Test %d: b 不匹配! 误差 = %.2e', i, norm(b_yours - b_ref)));
        assert(norm(g_yours - g_ref) < tol, ...
               sprintf('Test %d: g 不匹配! 误差 = %.2e', i, norm(g_yours - g_ref)));
        assert(abs(H_yours - H_ref) < tol, ...
               sprintf('Test %d: H 不匹配! 误差 = %.2e', i, abs(H_yours - H_ref)));
    end

    fprintf('全部 %d 组测试通过!
', n_tests);
end
```

### 6c. 质量矩阵性质检验

```matlab
function test_mass_matrix_properties()
% test_mass_matrix_properties  验证质量矩阵的数学性质

    rng(456);
    n_tests = 100;

    for i = 1:n_tests
        q = randn(6, 1);
        M = M_fun(q);

        % 1. 对称性
        sym_error = norm(M - M.', 'fro');
        assert(sym_error < 1e-10, ...
               sprintf('Test %d: 对称性失败, 误差 = %.2e', i, sym_error));

        % 2. 正定性
        eigenvalues = eig(M);
        assert(all(eigenvalues > 0), ...
               sprintf('Test %d: 正定性失败, 最小特征值 = %.2e', i, min(eigenvalues)));

        % 3. 条件数 (不应过大, 否则数值不稳定)
        cond_num = max(eigenvalues) / min(eigenvalues);
        if cond_num > 1e6
            warning('Test %d: 条件数过大 (%.2e), 可能存在数值问题', i, cond_num);
        end
    end

    fprintf('质量矩阵性质检验通过! (%d 组测试)
', n_tests);
end
```


---

## 7. 常见错误与调试技巧 (Common Mistakes and Debugging)

### 错误 1: 惯性张量坐标系混淆

```matlab
% 错误写法: 直接使用 body frame 惯性张量, 忘记坐标变换
M_wrong = M_wrong + Jr_k.' * I_k * Jr_k;  % WRONG!

% 正确写法: 先将惯性张量变换到 world frame
I_k_world = C_Ik.' * I_k * C_Ik;
M_correct = M_correct + Jr_k.' * I_k_world * Jr_k;  % CORRECT
```

注意: `C_Ik` 是从 body frame 到 world frame 的旋转矩阵。惯性张量的坐标变换公式为 $I^{world} = C^T \cdot I^{body} \cdot C$。这里使用转置是因为惯性张量是二阶张量, 变换规则与向量不同。

### 错误 2: 重力向量符号错误

```matlab
% 错误写法: 忘记负号
g_wrong = g_wrong + Jp_k.' * m_k * g_acc;  % WRONG! 符号相反

% 正确写法: 注意负号
g_correct = g_correct - Jp_k.' * m_k * g_acc;  % CORRECT
```

运动方程为 $M\ddot{q} + b + g = \tau$, 其中 $g$ 出现在左侧。重力加速度 $g_{acc} = [0; 0; -9.81]$ 已经包含了方向信息, 负号来自于势能对广义坐标的偏导数。

### 错误 3: 陀螺项遗漏

```matlab
% 错误写法: 只计算了前两项, 遗漏陀螺效应
b_incomplete = b_incomplete + m_k * Jp_k.' * dJp_k * dq ...
             + Jr_k.' * I_k_world * dJr_k * dq;  % 缺少第三项!

% 正确写法: 必须包含 omega x I*omega 项
omega_k = Jr_k * dq;
b_complete = b_complete + m_k * Jp_k.' * dJp_k * dq ...
           + Jr_k.' * I_k_world * dJr_k * dq ...
           + Jr_k.' * cross(omega_k, I_k_world * omega_k);  % CORRECT
```

### 错误 4: dAdt 中使用错误的变量

```matlab
% 错误写法: 对 dq 求导而不是对 q 求导
dJp_wrong = diff(Jp_k, dq(i)) * dq(i);  % WRONG!

% 正确写法: 对 q 求导, 乘以 dq (链式法则)
dJp_correct = diff(Jp_k, q(i)) * dq(i);  % CORRECT
```

### 调试技巧: 逐步验证

```matlab
%% 调试策略: 从简单到复杂

% Step 1: 先验证单个连杆 (k=1)
M_link1 = dyn.m(1) * jac.Jp{1}.' * jac.Jp{1} ...
        + jac.Jr{1}.' * kin.C{1}.' * dyn.I{1} * kin.C{1} * jac.Jr{1};
M_link1_simplified = simplify(M_link1);
disp('Link 1 contribution to M:');
disp(M_link1_simplified);

% Step 2: 检查 M 的维度和对称性 (符号层面)
assert(all(size(M) == [6, 6]), 'M 维度错误!');
M_sym_check = simplify(M - M.');
assert(isa(M_sym_check, 'sym') && all(M_sym_check(:) == 0), 'M 不对称!');

% Step 3: 代入具体数值检查量级是否合理
q_test = zeros(6, 1);
M_numeric = double(subs(M, gen_cor.q, q_test));
fprintf('M(q=0) 对角元素: ');
fprintf('%.4f  ', diag(M_numeric));
fprintf('
');
% 期望: 对角元素为正, 量级在 0.01 ~ 10 之间 (取决于机器人尺寸)

% Step 4: 能量守恒是最终的 "黄金标准" 测试
```

---

## 8. 总结与要点 (Summary)

| 组件 | 公式 | 物理含义 |
|------|------|----------|
| 质量矩阵 $M(q)$ | $\sum m_k J_{p_k}^T J_{p_k} + J_{r_k}^T I_k^w J_{r_k}$ | 广义坐标空间中的惯性 |
| 重力向量 $g(q)$ | $-\sum J_{p_k}^T m_k g_{acc}$ | 重力在关节空间的投影 |
| 非线性项 $b(q,\dot{q})$ | 平动 + 转动 + 陀螺 (三项之和) | 速度相关的虚拟力 |
| 哈密顿量 $H$ | $\frac{1}{2}\dot{q}^T M \dot{q} + U(q)$ | 系统总机械能 |

**实现检查清单 (Checklist):**

- [ ] 质量矩阵 $M$: 包含平动和转动两部分, 惯性张量已做坐标变换
- [ ] 重力向量 $g$: 符号正确 (负号)
- [ ] 非线性项 $b$: 三项齐全 (dJp, dJr, gyroscopic)
- [ ] 哈密顿量 $H$: 动能 + 势能
- [ ] 能量守恒测试通过
- [ ] 与参考解数值一致
