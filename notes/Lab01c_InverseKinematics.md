# Lab 1c: 逆运动学与运动学运动控制 (Inverse Kinematics & Kinematic Motion Control)

**课程**: ETH Zurich - Robot Dynamics  
**练习**: Exercise 1c  
**前置要求**: Exercise 1a (正运动学), Exercise 1b (雅可比矩阵)

---

## 1. 实验概述 (Lab Overview)

### 1.1 目标

本实验的核心目标是实现**逆运动学 (Inverse Kinematics)** 和**运动学运动控制 (Kinematic Motion Control)**。具体包括:

- 实现**阻尼伪逆矩阵 (Damped Pseudo-Inverse)** 计算
- 基于迭代雅可比方法实现**逆运动学求解器**
- 实现基于前馈+反馈结构的**运动学运动控制器**

### 1.2 与前序实验的关系

本实验建立在 Exercise 1a 和 1b 的基础之上:

| 前序实验 | 提供的函数 | 本实验中的用途 |
|---------|-----------|--------------|
| Ex 1a | `jointToPosition(q)` | 计算当前末端执行器位置 $r(q)$ |
| Ex 1b | `jointToPosJac(q)` | 计算位置雅可比矩阵 $J_P(q)$ |

逆运动学是正运动学的"反问题"：给定期望的末端位置 $r_{des}$，求解满足条件的关节角度 $q$。

### 1.3 问题背景

对于一个 6-DOF 机械臂，正运动学映射为：

$$r = f(q), \quad q \in \mathbb{R}^6, \quad r \in \mathbb{R}^3$$

逆运动学需要求解：

$$q = f^{-1}(r_{des})$$

由于该映射通常是非线性的且可能存在多解，我们采用基于雅可比矩阵的**迭代数值方法**。

---

## 2. 需要实现的函数

### 2.1 阻尼伪逆矩阵: `pseudoInverseMat(J, lambda)`

#### 函数签名

```matlab
function J_pinv = pseudoInverseMat(J, lambda)
% pseudoInverseMat - 计算阻尼伪逆矩阵 (Damped Least Squares Pseudo-Inverse)
%
% 输入:
%   J      - m×n 雅可比矩阵 (Jacobian matrix)
%   lambda - 阻尼系数 (damping factor), 标量
%
% 输出:
%   J_pinv - n×m 阻尼伪逆矩阵
```

#### 数学公式

阻尼伪逆 (Damped Least Squares, DLS) 定义为：

$$J^{+} = J^T (J J^T + \lambda^2 I)^{-1}$$

其中：
- $J \in \mathbb{R}^{m \times n}$ 为雅可比矩阵
- $\lambda \in \mathbb{R}$ 为阻尼系数
- $I \in \mathbb{R}^{m \times m}$ 为单位矩阵

#### 参考实现

```matlab
function J_pinv = pseudoInverseMat(J, lambda)
    [m, ~] = size(J);
    
    % 阻尼伪逆: J^T * (J * J^T + lambda^2 * I)^(-1)
    J_pinv = J' / (J * J' + lambda^2 * eye(m));
end
```

#### 特殊情况

- 当 `lambda = 0` 时，公式退化为标准的 **Moore-Penrose 伪逆**：

$$J^{+} = J^T (J J^T)^{-1}$$

- MATLAB 内置函数 `pinv(J)` 可用于验证 `lambda = 0` 时的结果

#### 验证方法

```matlab
% 验证: lambda=0 时应与 pinv() 结果一致
J_test = rand(3, 6);
J_pinv_custom = pseudoInverseMat(J_test, 0);
J_pinv_matlab = pinv(J_test);
fprintf('误差: %e\n', norm(J_pinv_custom - J_pinv_matlab));
% 期望输出: 误差接近机器精度 (~1e-14)
```

---

### 2.2 逆运动学: `inverseKinematics(r_des, q_init)`

#### 函数签名

```matlab
function q = inverseKinematics(r_des, q_init)
% inverseKinematics - 基于迭代雅可比方法的逆运动学求解
%
% 输入:
%   r_des  - 3×1 期望末端执行器位置 (desired end-effector position)
%   q_init - 6×1 初始关节角度猜测值 (initial joint angle guess)
%
% 输出:
%   q      - 6×1 满足逆运动学的关节角度解
```

#### 迭代算法流程

逆运动学求解采用基于雅可比的迭代数值方法，核心思路是在每一步用线性近似修正关节角度：

```
初始化: q = q_init
重复以下步骤:
    1. 计算当前末端位置:     r_current = jointToPosition(q)
    2. 计算位置误差:         Δr = r_des - r_current
    3. 检查收敛条件:         若 ||Δr|| < tol, 则停止
    4. 计算雅可比矩阵:       J = jointToPosJac(q)
    5. 计算伪逆:             J⁺ = pseudoInverseMat(J, lambda)
    6. 更新关节角度:         q = q + α · J⁺ · Δr
```

#### 参考实现

```matlab
function q = inverseKinematics(r_des, q_init)
    % 参数设置
    alpha = 1.0;          % 步长 (step size)
    lambda = 0.001;       % 阻尼系数 (damping factor)
    tol = 1e-6;           % 收敛容差 (convergence tolerance)
    max_iter = 1000;      % 最大迭代次数

    q = q_init;

    for i = 1:max_iter
        % Step 1: 计算当前末端位置
        r_current = jointToPosition(q);

        % Step 2: 计算位置误差
        dr = r_des - r_current;

        % Step 3: 检查收敛
        if norm(dr) < tol
            fprintf('逆运动学在第 %d 次迭代收敛, 误差: %e\n', i, norm(dr));
            return;
        end

        % Step 4: 计算雅可比矩阵
        J = jointToPosJac(q);

        % Step 5: 计算阻尼伪逆
        J_pinv = pseudoInverseMat(J, lambda);

        % Step 6: 更新关节角度
        q = q + alpha * J_pinv * dr;
    end

    warning('逆运动学未在 %d 次迭代内收敛, 最终误差: %e', max_iter, norm(dr));
end
```

#### 参数选择指南

| 参数 | 符号 | 典型值 | 说明 |
|------|------|--------|------|
| 步长 | $\alpha$ | 0.1 ~ 1.0 | 过大导致振荡，过小导致收敛慢 |
| 阻尼系数 | $\lambda$ | 0.001 ~ 0.01 | 逆运动学中通常取较小值 |
| 收敛容差 | tol | $10^{-6}$ | 位置误差的欧几里得范数阈值 |
| 最大迭代次数 | max_iter | 500 ~ 2000 | 防止无限循环 |

#### 注意事项

- **多解性**: 不同的 `q_init` 可能导致收敛到不同的解，这是因为 6-DOF 机械臂对于 3D 位置目标存在冗余自由度
- **步长调节**: 若 `alpha = 1.0` 导致不收敛，可尝试减小至 0.5 或 0.1
- **奇异性**: 在奇异构型附近，标准伪逆会产生极大的关节速度，此时阻尼伪逆的优势体现出来

```matlab
% 使用示例
r_desired = [0.5; 0.3; 0.2];       % 期望末端位置
q_initial = zeros(6, 1);            % 从零位开始
q_solution = inverseKinematics(r_desired, q_initial);

% 验证
r_achieved = jointToPosition(q_solution);
fprintf('位置误差: %e\n', norm(r_desired - r_achieved));
```

---

### 2.3 运动学运动控制: `kinematicMotionControl(q, r_des, v_des)`

#### 函数签名

```matlab
function Dq = kinematicMotionControl(q, r_des, v_des)
% kinematicMotionControl - 运动学运动控制器 (前馈+反馈)
%
% 输入:
%   q     - 6×1 当前关节角度 (current joint angles)
%   r_des - 3×1 期望末端执行器位置 (desired end-effector position)
%   v_des - 3×1 期望末端执行器速度 (desired end-effector velocity, feedforward)
%
% 输出:
%   Dq    - 6×1 关节速度命令 (joint velocity command)
```

#### 控制律

运动学运动控制器采用**前馈 + 反馈 (Feedforward + Feedback)** 结构：

$$\dot{q} = J^{+} \left( v_{des} + K_p (r_{des} - r_{current}) \right)$$

其中：
- $v_{des}$: 期望末端速度（前馈项，提供跟踪性能）
- $K_p (r_{des} - r_{current})$: 位置误差反馈项（消除稳态误差）
- $K_p$: 位置增益矩阵（通常取对角阵或标量）
- $J^{+}$: 阻尼伪逆矩阵

#### 参考实现

```matlab
function Dq = kinematicMotionControl(q, r_des, v_des)
    % 控制参数
    K_p = 5.0;        % 位置增益 (proportional gain)
    lambda = 0.1;     % 阻尼系数 (damping factor for pseudo-inverse)

    % 计算当前末端位置
    r_current = jointToPosition(q);

    % 计算位置误差
    dr = r_des - r_current;

    % 计算雅可比矩阵
    J = jointToPosJac(q);

    % 计算阻尼伪逆
    J_pinv = pseudoInverseMat(J, lambda);

    % 控制律: q_dot = J_pinv * (v_des + K_p * dr)
    Dq = J_pinv * (v_des + K_p * dr);
end
```

#### 控制结构框图

```
                    +----------+
  r_des ---->(+)-->| K_p      |--->(+)---> J⁺ ---> q_dot ---> 机器人
              ^-   +----------+     ^+
              |                     |
  r_current --+        v_des ------+
  (jointToPosition)   (前馈项)
```

#### 参数选择

| 参数 | 符号 | 推荐值 | 说明 |
|------|------|--------|------|
| 位置增益 | $K_p$ | 5.0 | 过大导致振荡，过小导致跟踪误差大 |
| 阻尼系数 | $\lambda$ | 0.1 | 运动控制中取较大值以保证平滑性 |

注意：运动控制中的 $\lambda$ 通常比逆运动学中的大（0.1 vs 0.001），因为运动控制更关注**平滑性和稳定性**，而逆运动学更关注**精度**。

#### 与逆运动学的对比

| 特性 | 逆运动学 (IK) | 运动学运动控制 |
|------|--------------|--------------|
| 输出 | 关节角度 $q$ | 关节速度 $\dot{q}$ |
| 执行方式 | 离线迭代求解 | 在线实时控制 |
| 适用场景 | 求解目标构型 | 轨迹跟踪 |
| 阻尼系数 | 较小 (~0.001) | 较大 (~0.1) |

---

## 3. 关键理论

### 3.1 伪逆与阻尼伪逆

#### Moore-Penrose 伪逆

对于满秩矩阵 $J \in \mathbb{R}^{m \times n}$（$m < n$，即欠定系统），Moore-Penrose 伪逆为：

$$J^{+} = J^T (J J^T)^{-1}$$

该伪逆给出的解 $\dot{q} = J^{+} v$ 是所有满足 $J \dot{q} = v$ 的解中**范数最小**的：

$$\dot{q}^{*} = \arg\min_{\dot{q}} \|\dot{q}\| \quad \text{s.t.} \quad J\dot{q} = v$$

#### 阻尼伪逆 (Damped Least Squares, DLS)

当雅可比矩阵接近奇异时，$J J^T$ 接近奇异，标准伪逆会产生极大的关节速度。阻尼伪逆通过引入正则化项解决这一问题：

$$J^{+}_{\lambda} = J^T (J J^T + \lambda^2 I)^{-1}$$

其优化目标为：

$$\dot{q}^{*} = \arg\min_{\dot{q}} \left( \|J\dot{q} - v\|^2 + \lambda^2 \|\dot{q}\|^2 \right)$$

这是一个**精度与关节速度之间的权衡**：
- $\lambda$ 较小：优先保证末端精度
- $\lambda$ 较大：优先限制关节速度

#### 奇异值分解 (SVD) 视角

通过 SVD 可以更直观地理解阻尼的作用。设 $J = U \Sigma V^T$，则：

- 标准伪逆: $J^{+} = V \Sigma^{-1} U^T$，其中 $\Sigma^{-1}$ 的对角元素为 $1/\sigma_i$
- 阻尼伪逆: 对角元素变为 $\sigma_i / (\sigma_i^2 + \lambda^2)$

当 $\sigma_i \to 0$ 时：
- 标准伪逆: $1/\sigma_i \to \infty$（关节速度爆炸）
- 阻尼伪逆: $\sigma_i / (\sigma_i^2 + \lambda^2) \to 0$（关节速度被抑制）

```matlab
% SVD 视角的验证
J = jointToPosJac(q);
[U, S, V] = svd(J);
fprintf('雅可比矩阵的奇异值:\n');
disp(diag(S)');

% 观察: 当某个奇异值接近0时, 机械臂处于奇异构型
```

### 3.2 迭代逆运动学

#### 基本原理

逆运动学的迭代方法基于**一阶泰勒展开**。在当前关节角度 $q$ 处，末端位置的线性近似为：

$$r(q + \Delta q) \approx r(q) + J(q) \Delta q$$

令 $r(q + \Delta q) = r_{des}$，得到：

$$\Delta q = J^{+}(q) \cdot (r_{des} - r(q)) = J^{+}(q) \cdot \Delta r$$

由于这只是线性近似，单步通常无法精确到达目标，因此需要**迭代**。

#### 收敛性分析

迭代逆运动学的收敛性取决于以下因素：

1. **初始值 $q_{init}$**: 距离目标解越近，收敛越快且越可靠
2. **步长 $\alpha$**:
   - $\alpha = 1.0$: 完整 Newton 步，收敛最快但可能不稳定
   - $\alpha < 1.0$: 阻尼 Newton 步，更稳定但收敛较慢
3. **奇异性**: 若迭代路径经过奇异构型，可能导致发散
4. **工作空间边界**: 目标点在工作空间边界附近时收敛困难

```matlab
% 演示: 不同初始值导致不同解
r_des = [0.4; 0.0; 0.3];

% 初始值 1: 从零位开始
q_sol1 = inverseKinematics(r_des, zeros(6,1));

% 初始值 2: 从不同构型开始
q_init2 = [pi/4; -pi/6; pi/3; 0; pi/4; 0];
q_sol2 = inverseKinematics(r_des, q_init2);

% 两个解可能不同, 但末端位置相同
r1 = jointToPosition(q_sol1);
r2 = jointToPosition(q_sol2);
fprintf('解1的末端位置: [%.4f, %.4f, %.4f]\n', r1);
fprintf('解2的末端位置: [%.4f, %.4f, %.4f]\n', r2);
fprintf('关节角度差异: %.4f rad\n', norm(q_sol1 - q_sol2));
```

#### 冗余自由度

对于 6-DOF 机械臂求解 3D 位置（3个约束），系统存在 $6 - 3 = 3$ 个冗余自由度。这意味着：

- 存在无穷多组关节角度满足同一末端位置
- Moore-Penrose 伪逆给出的是**最小范数解**（关节角度变化量最小）
- 可以利用冗余自由度实现次要目标（如避障、关节极限回避等）

利用零空间投影实现次要目标：

$$\dot{q} = J^{+} v + (I - J^{+} J) \dot{q}_{null}$$

其中 $(I - J^{+} J)$ 是雅可比零空间的投影矩阵，$\dot{q}_{null}$ 是次要目标的梯度方向。

```matlab
% 零空间投影示例
J = jointToPosJac(q);
J_pinv = pseudoInverseMat(J, 0.001);
N = eye(6) - J_pinv * J;  % 零空间投影矩阵

% 次要目标: 使关节角度趋向零位 (关节极限回避)
q_null_desired = -0.5 * q;  % 负梯度方向

% 带零空间优化的关节速度
dq = J_pinv * dr + N * q_null_desired;
```

### 3.3 运动学运动控制

#### 控制器设计

运动学运动控制器的设计基于以下假设：

- 机器人的低层控制器能够精确跟踪关节速度命令
- 即机器人可视为一个**速度可控系统 (velocity-controlled system)**

控制律为：

$$\dot{q} = J^{+} \left( \dot{r}_{des} + K_p (r_{des} - r) \right)$$

#### 误差动力学

定义位置误差 $e = r_{des} - r$，则误差动力学为：

$$\dot{e} = \dot{r}_{des} - \dot{r} = \dot{r}_{des} - J\dot{q}$$

将控制律代入（假设 $J J^{+} \approx I$）：

$$\dot{e} = \dot{r}_{des} - (\dot{r}_{des} + K_p e) = -K_p e$$

这是一个**指数收敛**的一阶线性系统：

$$e(t) = e(0) \cdot e^{-K_p t}$$

- $K_p > 0$ 保证误差指数衰减
- $K_p$ 越大，收敛越快，但对噪声和建模误差越敏感

#### 前馈项与反馈项的作用

| 项 | 公式 | 作用 |
|----|------|------|
| 前馈项 | $J^{+} v_{des}$ | 提供期望运动的基本关节速度，减小跟踪误差 |
| 反馈项 | $J^{+} K_p (r_{des} - r)$ | 修正由建模误差、数值积分等引起的偏差 |

- 仅有前馈：跟踪性能好但无法消除稳态误差
- 仅有反馈：能消除稳态误差但跟踪延迟大
- 前馈+反馈：兼具两者优点

```matlab
% 运动控制仿真示例
dt = 0.01;           % 时间步长
T = 5.0;             % 总仿真时间
t = 0:dt:T;
q = zeros(6, 1);     % 初始关节角度

% 圆形轨迹
radius = 0.1;
center = [0.4; 0.0; 0.3];

for i = 1:length(t)
    % 期望位置 (圆形轨迹)
    r_des = center + radius * [cos(t(i)); sin(t(i)); 0];

    % 期望速度 (轨迹的时间导数)
    v_des = radius * [-sin(t(i)); cos(t(i)); 0];

    % 计算关节速度命令
    Dq = kinematicMotionControl(q, r_des, v_des);

    % 积分更新关节角度 (欧拉法)
    q = q + Dq * dt;

    % 记录数据用于绘图
    r_actual(:, i) = jointToPosition(q);
    r_desired(:, i) = r_des;
end
```

---

## 4. 可视化

### 4.1 motion_control_visualization.m

以下脚本用于可视化运动学运动控制的效果，展示末端执行器的期望轨迹与实际轨迹的对比。

```matlab
%% motion_control_visualization.m
% 运动学运动控制可视化脚本
% 展示末端执行器跟踪圆形轨迹的效果

clear; clc; close all;

%% 仿真参数
dt = 0.01;              % 时间步长 [s]
T  = 10.0;              % 总仿真时间 [s]
t  = 0:dt:T;
N  = length(t);

%% 初始化
q = zeros(6, 1);        % 初始关节角度

% 数据记录
r_actual  = zeros(3, N);
r_desired = zeros(3, N);
e_record  = zeros(1, N);
dq_record = zeros(6, N);

%% 轨迹定义: 圆形轨迹
radius = 0.1;                          % 圆的半径 [m]
omega  = 2 * pi / 5;                   % 角速度 [rad/s], 周期5秒
center = jointToPosition(q);           % 以初始位置为圆心

%% 仿真主循环
for i = 1:N
    % 期望位置 (圆形轨迹)
    r_des = center + radius * [cos(omega * t(i)) - 1;
                                sin(omega * t(i));
                                0];

    % 期望速度 (解析导数)
    v_des = radius * omega * [-sin(omega * t(i));
                                cos(omega * t(i));
                                0];

    % 运动学运动控制器
    Dq = kinematicMotionControl(q, r_des, v_des);

    % 欧拉积分
    q = q + Dq * dt;

    % 记录数据
    r_actual(:, i)  = jointToPosition(q);
    r_desired(:, i) = r_des;
    e_record(i)     = norm(r_des - r_actual(:, i));
    dq_record(:, i) = Dq;
end

%% 绘图

% --- 图1: 3D 轨迹对比 ---
figure('Name', '3D Trajectory Tracking', 'Position', [100 100 800 600]);
plot3(r_desired(1,:), r_desired(2,:), r_desired(3,:), 'b--', 'LineWidth', 1.5);
hold on;
plot3(r_actual(1,:), r_actual(2,:), r_actual(3,:), 'r-', 'LineWidth', 1.5);
plot3(r_actual(1,1), r_actual(2,1), r_actual(3,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(r_actual(1,end), r_actual(2,end), r_actual(3,end), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
hold off;
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('End-Effector Trajectory Tracking');
legend('Desired', 'Actual', 'Start', 'End', 'Location', 'best');
view(3);

% --- 图2: 各轴位置跟踪 ---
figure('Name', 'Position Tracking per Axis', 'Position', [100 100 800 600]);
axis_labels = {'X', 'Y', 'Z'};
for k = 1:3
    subplot(3, 1, k);
    plot(t, r_desired(k,:), 'b--', 'LineWidth', 1.2);
    hold on;
    plot(t, r_actual(k,:), 'r-', 'LineWidth', 1.2);
    hold off;
    grid on;
    ylabel([axis_labels{k} ' [m]']);
    legend('Desired', 'Actual');
    if k == 1
        title('Position Tracking per Axis');
    end
    if k == 3
        xlabel('Time [s]');
    end
end

% --- 图3: 跟踪误差 ---
figure('Name', 'Tracking Error', 'Position', [100 100 800 400]);
semilogy(t, e_record, 'k-', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Position Error ||e|| [m]');
title('End-Effector Tracking Error');

% --- 图4: 关节速度 ---
figure('Name', 'Joint Velocities', 'Position', [100 100 800 600]);
for k = 1:6
    subplot(3, 2, k);
    plot(t, dq_record(k,:), 'LineWidth', 1.0);
    grid on;
    ylabel(['\dot{q}_' num2str(k) ' [rad/s]']);
    if k >= 5
        xlabel('Time [s]');
    end
    if k == 1
        title('Joint Velocities');
    end
end

fprintf('仿真完成.\n');
fprintf('最大跟踪误差: %.6f m\n', max(e_record));
fprintf('最终跟踪误差: %.6f m\n', e_record(end));
fprintf('平均跟踪误差: %.6f m\n', mean(e_record));
```

### 4.2 预期结果

运行可视化脚本后，应观察到以下现象：

1. **3D 轨迹图**: 红色实际轨迹应紧密跟踪蓝色期望轨迹，初始阶段可能有短暂偏差
2. **各轴位置图**: 各轴的实际位置应与期望位置高度吻合
3. **跟踪误差图**: 误差应在初始瞬态后迅速衰减至较小值（取决于 $K_p$）
4. **关节速度图**: 关节速度应平滑且有界，无突变或发散

### 4.3 参数调节实验

建议尝试以下参数组合，观察对控制效果的影响：

```matlab
% 实验1: 改变 K_p
% K_p = 1.0   -> 跟踪误差较大, 但关节速度平滑
% K_p = 5.0   -> 良好的跟踪性能 (推荐)
% K_p = 20.0  -> 跟踪误差极小, 但关节速度可能较大
% K_p = 50.0  -> 可能出现振荡或不稳定

% 实验2: 改变 lambda
% lambda = 0.001 -> 高精度, 但奇异性附近不稳定
% lambda = 0.1   -> 良好的平衡 (推荐)
% lambda = 1.0   -> 过度阻尼, 跟踪精度下降

% 实验3: 去掉前馈项
% v_des = zeros(3,1) -> 仅靠反馈, 观察跟踪延迟增大

% 实验4: 去掉反馈项
% K_p = 0 -> 仅靠前馈, 观察误差无法消除
```

---

## 5. 常见问题与调试

### 5.1 逆运动学不收敛

**可能原因与解决方案**:

| 原因 | 症状 | 解决方案 |
|------|------|---------|
| 目标点在工作空间外 | 误差不减小 | 检查目标点是否可达 |
| 步长过大 | 误差振荡 | 减小 $\alpha$（如 0.5 或 0.1） |
| 经过奇异构型 | 关节角度突变 | 增大 $\lambda$ 或更换初始值 |
| 初始值不佳 | 收敛到局部极小 | 尝试多个不同的 $q_{init}$ |

```matlab
% 调试: 打印每次迭代的误差
function q = inverseKinematics_debug(r_des, q_init)
    alpha = 0.5;
    lambda = 0.01;
    tol = 1e-6;
    max_iter = 1000;
    q = q_init;

    errors = zeros(max_iter, 1);

    for i = 1:max_iter
        r_current = jointToPosition(q);
        dr = r_des - r_current;
        errors(i) = norm(dr);

        if errors(i) < tol
            fprintf('收敛于第 %d 次迭代\n', i);
            break;
        end

        J = jointToPosJac(q);
        J_pinv = pseudoInverseMat(J, lambda);
        q = q + alpha * J_pinv * dr;
    end

    % 绘制收敛曲线
    figure;
    semilogy(1:i, errors(1:i), 'b-o', 'MarkerSize', 3);
    grid on;
    xlabel('Iteration');
    ylabel('||error||');
    title('Inverse Kinematics Convergence');
end
```

### 5.2 运动控制中的振荡

若末端执行器出现振荡，通常是 $K_p$ 过大或 $\lambda$ 过小导致的。建议：

1. 降低 $K_p$（如从 10 降至 5）
2. 增大 $\lambda$（如从 0.01 增至 0.1）
3. 检查仿真时间步长 $dt$ 是否过大

### 5.3 维度检查清单

实现过程中常见的维度错误，请确认：

```matlab
% 维度检查
q       % 应为 6×1
r_des   % 应为 3×1
v_des   % 应为 3×1
J       % 应为 3×6 (位置雅可比)
J_pinv  % 应为 6×3
Dq      % 应为 6×1
dr      % 应为 3×1
```

---

## 6. 总结

### 本实验的核心公式

| 公式 | 用途 |
|------|------|
| $J^{+} = J^T(JJ^T + \lambda^2 I)^{-1}$ | 阻尼伪逆矩阵 |
| $q_{k+1} = q_k + \alpha J^{+} (r_{des} - r(q_k))$ | 迭代逆运动学 |
| $\dot{q} = J^{+}(v_{des} + K_p(r_{des} - r))$ | 运动学运动控制 |

### 实现顺序建议

```
1. pseudoInverseMat()       -- 基础工具函数
       |
       v
2. inverseKinematics()      -- 依赖 pseudoInverseMat + Ex1a/1b
       |
       v
3. kinematicMotionControl() -- 依赖 pseudoInverseMat + Ex1a/1b
       |
       v
4. 运行可视化脚本验证       -- 验证所有实现的正确性
```

### 延伸思考

- 如何处理**关节极限 (joint limits)**？提示：利用零空间投影
- 如何扩展到**6-DOF 任务空间**（位置+姿态）？提示：使用完整的 $6 \times 6$ 雅可比矩阵
- **阻尼系数自适应**：能否根据奇异值动态调整 $\lambda$？参考 Nakamura & Hanafusa (1986)

---

> **参考文献**:
> - Siciliano, B. et al., *Robotics: Modelling, Planning and Control*, Springer, 2009, Chapter 3.
> - Nakamura, Y. & Hanafusa, H., "Inverse Kinematic Solutions with Singularity Robustness for Robot Manipulator Control", *ASME J. Dynamic Systems*, 1986.
> - Buss, S.R., "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods", 2004.
> - ETH Zurich, Robot Dynamics Course Materials.
