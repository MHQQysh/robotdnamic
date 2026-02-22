# Lecture 02: 运动学 I — 旋转与角速度；刚体描述与变换

> ETH Zurich — Robot Dynamics  
> 主题: Kinematics 1 — Rotation and Angular Velocity; Rigid Body Formulation, Transformation

---

## 目录

1. [位置参数化回顾 (Position Parameterization)](#1-位置参数化回顾)
2. [旋转参数化 (Rotation Parameterization)](#2-旋转参数化)
3. [角速度与时间导数 (Angular Velocity and Time Derivatives)](#3-角速度与时间导数)
4. [单刚体的位置与姿态 (Position and Orientation of a Single Body)](#4-单刚体的位置与姿态)
5. [串联运动链 (Serial Kinematic Linkages)](#5-串联运动链)
6. [广义坐标 (Generalized Coordinates)](#6-广义坐标)
7. [正运动学 (Forward Kinematics)](#7-正运动学)

---

## 1. 位置参数化回顾

### 1.1 位置向量与参数化

在三维空间中，一个质点 $P$ 相对于惯性坐标系 $I$ 的位置可以用位置向量表示:

$$
{}_I \mathbf{r}_P = \mathbf{r}(\boldsymbol{\chi}_P) \in \mathbb{R}^3
$$

其中 $\boldsymbol{\chi}_P$ 是位置参数向量 (position parameters)。不同的参数化方式对应不同的坐标系统。

### 1.2 三种常见坐标系统

**笛卡尔坐标 (Cartesian Coordinates)**

最直接的参数化方式，使用三个正交方向的投影:

$$
\boldsymbol{\chi}_P = \begin{pmatrix} x \ y \ z \end{pmatrix}, \quad
{}_I \mathbf{r}_P = x \, \mathbf{e}_x + y \, \mathbf{e}_y + z \, \mathbf{e}_z
$$

参数数量: 3，无奇异性 (no singularity)，映射矩阵 $\mathbf{E} = \mathbf{I}_{3 \times 3}$。

**柱坐标 (Cylindrical Coordinates)**

$$
\boldsymbol{\chi}_P = \begin{pmatrix} \rho \ \varphi \ z \end{pmatrix}, \quad
{}_I \mathbf{r}_P = \begin{pmatrix} \rho \cos\varphi \ \rho \sin\varphi \ z \end{pmatrix}
$$

其中 $\rho \geq 0$ 为径向距离，$\varphi \in [0, 2\pi)$ 为方位角，$z$ 为高度。

奇异性: 当 $\rho = 0$ 时，$\varphi$ 不确定。

**球坐标 (Spherical Coordinates)**

$$
\boldsymbol{\chi}_P = \begin{pmatrix} r \ \varphi \ \vartheta \end{pmatrix}, \quad
{}_I \mathbf{r}_P = \begin{pmatrix} r \cos\varphi \sin\vartheta \ r \sin\varphi \sin\vartheta \ r \cos\vartheta \end{pmatrix}
$$

其中 $r \geq 0$ 为径向距离，$\varphi \in [0, 2\pi)$ 为方位角，$\vartheta \in [0, \pi]$ 为极角。

奇异性: 当 $r = 0$ 或 $\vartheta = 0, \pi$ 时存在奇异性。

### 1.3 线速度与参数微分的关系

对位置向量求时间导数，利用链式法则 (chain rule):

$$
{}_I \dot{\mathbf{r}}_P = \frac{\partial \mathbf{r}}{\partial \boldsymbol{\chi}_P} \dot{\boldsymbol{\chi}}_P = \mathbf{E}(\boldsymbol{\chi}_P) \cdot \dot{\boldsymbol{\chi}}_P
$$

其中 $\mathbf{E}(\boldsymbol{\chi}_P) \in \mathbb{R}^{3 \times n}$ 称为映射矩阵 (mapping matrix)。

**柱坐标的映射矩阵推导:**

$$
{}_I \dot{\mathbf{r}}_P = \frac{d}{dt}\begin{pmatrix} \rho\cos\varphi \ \rho\sin\varphi \ z \end{pmatrix} = \begin{pmatrix} \dot{\rho}\cos\varphi - \rho\dot{\varphi}\sin\varphi \ \dot{\rho}\sin\varphi + \rho\dot{\varphi}\cos\varphi \ \dot{z} \end{pmatrix}
$$

写成矩阵形式:

$$
{}_I \dot{\mathbf{r}}_P = \underbrace{\begin{pmatrix} \cos\varphi & -\rho\sin\varphi & 0 \ \sin\varphi & \rho\cos\varphi & 0 \ 0 & 0 & 1 \end{pmatrix}}_{\mathbf{E}(\boldsymbol{\chi}_P)} \begin{pmatrix} \dot{\rho} \ \dot{\varphi} \ \dot{z} \end{pmatrix}
$$

当 $\rho = 0$ 时，$\mathbf{E}$ 的第二列变为零向量，矩阵降秩 (rank-deficient)，这正是柱坐标的奇异点。

> **Quiz 1:** 请写出球坐标的映射矩阵 $\mathbf{E}(\boldsymbol{\chi}_P)$，并指出其奇异点。
>
> **解答:**
> $$\mathbf{E} = \begin{pmatrix} \cos\varphi\sin\vartheta & -r\sin\varphi\sin\vartheta & r\cos\varphi\cos\vartheta \ \sin\varphi\sin\vartheta & r\cos\varphi\sin\vartheta & r\sin\varphi\cos\vartheta \ \cos\vartheta & 0 & -r\sin\vartheta \end{pmatrix}$$
> 奇异点: $r = 0$ 或 $\sin\vartheta = 0$（即 $\vartheta = 0, \pi$）。

---

## 2. 旋转参数化

### 2.1 旋转矩阵 (Rotation Matrix)

**定义与性质**

旋转矩阵 $\mathbf{C}_{AB} \in SO(3)$ 描述坐标系 $B$ 相对于坐标系 $A$ 的姿态 (orientation)。它将向量从 $B$ 系表示变换到 $A$ 系表示:

$$
{}_A \mathbf{r} = \mathbf{C}_{AB} \, {}_B \mathbf{r}
$$

特殊正交群 $SO(3)$ 的定义:

$$
SO(3) = \left\{ \mathbf{C} \in \mathbb{R}^{3\times 3} \;\middle|\; \mathbf{C}^T \mathbf{C} = \mathbf{I}, \; \det(\mathbf{C}) = +1 \right\}
$$

**参数计数:**

- 旋转矩阵有 $3 \times 3 = 9$ 个元素
- 正交性约束 $\mathbf{C}^T\mathbf{C} = \mathbf{I}$ 提供 6 个独立约束（对称矩阵的上三角部分）
- 因此自由度 (degrees of freedom) = $9 - 6 = 3$

**基本性质:**

| 性质 | 表达式 |
|------|--------|
| 逆变换 | $\mathbf{C}_{AB}^{-1} = \mathbf{C}_{AB}^T = \mathbf{C}_{BA}$ |
| 链式法则 | $\mathbf{C}_{AC} = \mathbf{C}_{AB} \cdot \mathbf{C}_{BC}$ |
| 行列式 | $\det(\mathbf{C}) = +1$ |
| 列向量正交归一 | $\mathbf{c}_i^T \mathbf{c}_j = \delta_{ij}$ |

**基本旋转矩阵 (Elementary Rotation Matrices):**

绕 $x$ 轴旋转角度 $\alpha$:

$$
\mathbf{C}_x(\alpha) = \begin{pmatrix} 1 & 0 & 0 \ 0 & \cos\alpha & -\sin\alpha \ 0 & \sin\alpha & \cos\alpha \end{pmatrix}
$$

绕 $y$ 轴旋转角度 $\beta$:

$$
\mathbf{C}_y(\beta) = \begin{pmatrix} \cos\beta & 0 & \sin\beta \ 0 & 1 & 0 \ -\sin\beta & 0 & \cos\beta \end{pmatrix}
$$

绕 $z$ 轴旋转角度 $\gamma$:

$$
\mathbf{C}_z(\gamma) = \begin{pmatrix} \cos\gamma & -\sin\gamma & 0 \ \sin\gamma & \cos\gamma & 0 \ 0 & 0 & 1 \end{pmatrix}
$$

### 2.2 欧拉角 (Euler Angles)

欧拉角使用三次绕坐标轴的连续旋转来参数化任意旋转。根据旋转轴的选取方式，共有 12 种组合。

**分类:**

- **固有欧拉角 (Proper Euler Angles):** 首末旋转轴相同，如 ZYZ, ZXZ, XYX, XZX, YXY, YZY
- **Tait-Bryan 角:** 三次旋转轴各不相同，如 ZYX, XYZ, YZX, ZXY, XZY, YXZ

#### 2.2.1 ZYZ 欧拉角

参数: $\boldsymbol{\chi}_R = (\alpha, \beta, \gamma)^T$

旋转顺序: 先绕 $z$ 轴转 $\alpha$，再绕新 $y'$ 轴转 $\beta$，最后绕新 $z''$ 轴转 $\gamma$。

$$
\mathbf{C}_{ZYZ}(\alpha, \beta, \gamma) = \mathbf{C}_z(\alpha) \cdot \mathbf{C}_y(\beta) \cdot \mathbf{C}_z(\gamma)
$$

展开计算:

$$
\mathbf{C}_{ZYZ} = \begin{pmatrix}
c_\alpha c_\beta c_\gamma - s_\alpha s_\gamma & -c_\alpha c_\beta s_\gamma - s_\alpha c_\gamma & c_\alpha s_\beta \
s_\alpha c_\beta c_\gamma + c_\alpha s_\gamma & -s_\alpha c_\beta s_\gamma + c_\alpha c_\gamma & s_\alpha s_\beta \
-s_\beta c_\gamma & s_\beta s_\gamma & c_\beta
\end{pmatrix}
$$

其中 $c_\alpha = \cos\alpha$, $s_\alpha = \sin\alpha$，以此类推。

**奇异性 (Gimbal Lock):** 当 $\beta = 0$ 或 $\beta = \pi$ 时，第一次和第三次旋转轴重合，$\alpha$ 和 $\gamma$ 不再独立可辨。此时只能确定 $\alpha + \gamma$（或 $\alpha - \gamma$），丢失一个自由度。

#### 2.2.2 ZYX 欧拉角 (Tait-Bryan Angles)

这是航空航天中最常用的约定，也称为 yaw-pitch-roll:

- $\psi$ — 偏航角 (yaw)，绕 $z$ 轴
- $\theta$ — 俯仰角 (pitch)，绕 $y'$ 轴
- $\phi$ — 滚转角 (roll)，绕 $x''$ 轴

参数: $\boldsymbol{\chi}_R = (\psi, \theta, \phi)^T$

$$
\mathbf{C}_{ZYX}(\psi, \theta, \phi) = \mathbf{C}_z(\psi) \cdot \mathbf{C}_y(\theta) \cdot \mathbf{C}_x(\phi)
$$

完整展开:

$$
\mathbf{C}_{ZYX} = \begin{pmatrix}
c_\psi c_\theta & c_\psi s_\theta s_\phi - s_\psi c_\phi & c_\psi s_\theta c_\phi + s_\psi s_\phi \
s_\psi c_\theta & s_\psi s_\theta s_\phi + c_\psi c_\phi & s_\psi s_\theta c_\phi - c_\psi s_\phi \
-s_\theta & c_\theta s_\phi & c_\theta c_\phi
\end{pmatrix}
$$

**奇异性:** 当 $\theta = \pm\frac{\pi}{2}$ 时，$\cos\theta = 0$，矩阵退化，$\psi$ 和 $\phi$ 不再独立。

#### 2.2.3 从旋转矩阵提取欧拉角 (Inverse Problem)

给定旋转矩阵 $\mathbf{C} = (c_{ij})$，提取 ZYX 欧拉角:

$$
\theta = \text{atan2}\!\left(-c_{31}, \sqrt{c_{11}^2 + c_{21}^2}\right)
$$

$$
\psi = \text{atan2}\!\left(c_{21}, c_{11}\right)
$$

$$
\phi = \text{atan2}\!\left(c_{32}, c_{33}\right)
$$

其中 $\text{atan2}(y, x)$ 是四象限反正切函数 (four-quadrant inverse tangent)，返回值范围为 $(-\pi, \pi]$，能正确处理所有象限。

> **注意:** 使用 $\text{atan2}$ 而非 $\arctan$ 是因为后者无法区分 $(y, x)$ 和 $(-y, -x)$ 的情况。


### 2.3 轴角表示 (Angle-Axis Representation)

**基本思想:**

根据欧拉旋转定理 (Euler's Rotation Theorem)，任意旋转都可以表示为绕某一固定轴旋转某一角度。用单位向量 $\mathbf{n} \in \mathbb{R}^3$（$\|\mathbf{n}\| = 1$）表示旋转轴，$\theta \in [0, \pi]$ 表示旋转角度。

**Rodrigues 旋转公式:**

旋转后的向量 $\mathbf{r}'$ 可以通过以下公式计算:

$$
\mathbf{r}' = \cos\theta \, \mathbf{r} + (1 - \cos\theta)(\mathbf{n} \cdot \mathbf{r})\mathbf{n} + \sin\theta \, (\mathbf{n} \times \mathbf{r})
$$

**推导过程:**

将向量 $\mathbf{r}$ 分解为平行于 $\mathbf{n}$ 和垂直于 $\mathbf{n}$ 的分量:

$$
\mathbf{r}_\parallel = (\mathbf{n} \cdot \mathbf{r})\mathbf{n}, \quad \mathbf{r}_\perp = \mathbf{r} - (\mathbf{n} \cdot \mathbf{r})\mathbf{n}
$$

平行分量在旋转中不变: $\mathbf{r}'_\parallel = \mathbf{r}_\parallel$

垂直分量在垂直于 $\mathbf{n}$ 的平面内旋转:

$$
\mathbf{r}'_\perp = \cos\theta \, \mathbf{r}_\perp + \sin\theta \, (\mathbf{n} \times \mathbf{r}_\perp)
$$

注意 $\mathbf{n} \times \mathbf{r}_\perp = \mathbf{n} \times \mathbf{r}$（因为 $\mathbf{n} \times \mathbf{r}_\parallel = \mathbf{0}$），合并得:

$$
\mathbf{r}' = \mathbf{r}'_\parallel + \mathbf{r}'_\perp = (\mathbf{n} \cdot \mathbf{r})\mathbf{n} + \cos\theta[\mathbf{r} - (\mathbf{n} \cdot \mathbf{r})\mathbf{n}] + \sin\theta(\mathbf{n} \times \mathbf{r})
$$

整理即得 Rodrigues 公式。

**轴角旋转矩阵:**

将 Rodrigues 公式写成矩阵形式。引入反对称矩阵 (skew-symmetric matrix):

$$
\mathbf{n}^\times = [\mathbf{n}]_\times = \begin{pmatrix} 0 & -n_3 & n_2 \ n_3 & 0 & -n_1 \ -n_2 & n_1 & 0 \end{pmatrix}
$$

使得 $\mathbf{n} \times \mathbf{r} = \mathbf{n}^\times \mathbf{r}$。

同时注意 $(\mathbf{n} \cdot \mathbf{r})\mathbf{n} = \mathbf{n}\mathbf{n}^T \mathbf{r}$，因此:

$$
\boxed{\mathbf{C}(\mathbf{n}, \theta) = \cos\theta \, \mathbf{I} + \sin\theta \, \mathbf{n}^\times + (1 - \cos\theta) \, \mathbf{n}\mathbf{n}^T}
$$

这就是 **Rodrigues 旋转公式的矩阵形式**。

**验证性质:**

- 当 $\theta = 0$ 时: $\mathbf{C} = \mathbf{I}$（恒等旋转）
- $\mathbf{C}^T\mathbf{C} = \mathbf{I}$（正交性，可通过展开验证）
- $\det(\mathbf{C}) = +1$

**从旋转矩阵提取轴角:**

旋转角度:

$$
\theta = \arccos\!\left(\frac{\text{tr}(\mathbf{C}) - 1}{2}\right)
$$

旋转轴（当 $\theta \neq 0, \pi$ 时）:

$$
\mathbf{n} = \frac{1}{2\sin\theta}\begin{pmatrix} c_{32} - c_{23} \ c_{13} - c_{31} \ c_{21} - c_{12} \end{pmatrix}
$$

等价地，$\mathbf{n}^\times = \frac{1}{2\sin\theta}(\mathbf{C} - \mathbf{C}^T)$。

> **Quiz 2:** 证明 $(\mathbf{n}^\times)^2 = \mathbf{n}\mathbf{n}^T - \mathbf{I}$，并利用此关系将 Rodrigues 公式改写为指数映射形式。
>
> **解答:**
> 直接展开 $(\mathbf{n}^\times)^2$:
> $$(\mathbf{n}^\times)^2 = \begin{pmatrix} -n_2^2 - n_3^2 & n_1 n_2 & n_1 n_3 \ n_1 n_2 & -n_1^2 - n_3^2 & n_2 n_3 \ n_1 n_3 & n_2 n_3 & -n_1^2 - n_2^2 \end{pmatrix} = \mathbf{n}\mathbf{n}^T - \mathbf{I}$$
>
> 其中利用了 $\|\mathbf{n}\|^2 = n_1^2 + n_2^2 + n_3^2 = 1$。
>
> 代入 Rodrigues 公式:
> $$\mathbf{C} = \cos\theta \, \mathbf{I} + \sin\theta \, \mathbf{n}^\times + (1 - \cos\theta)(\mathbf{I} + (\mathbf{n}^\times)^2)$$
> $$= \mathbf{I} + \sin\theta \, \mathbf{n}^\times + (1 - \cos\theta)(\mathbf{n}^\times)^2$$
>
> 这与矩阵指数的 Taylor 展开一致:
> $$\mathbf{C} = e^{\theta \mathbf{n}^\times} = \mathbf{I} + \theta \mathbf{n}^\times + \frac{\theta^2}{2!}(\mathbf{n}^\times)^2 + \cdots = \mathbf{I} + \sin\theta \, \mathbf{n}^\times + (1 - \cos\theta)(\mathbf{n}^\times)^2$$

### 2.4 旋转向量 (Rotation Vector)

将轴角表示合并为一个三维向量:

$$
\boldsymbol{\varphi} = \theta \, \mathbf{n} \in \mathbb{R}^3
$$

其中 $\|\boldsymbol{\varphi}\| = \theta$ 为旋转角度，$\boldsymbol{\varphi} / \|\boldsymbol{\varphi}\|$ 为旋转轴。

旋转矩阵可以通过指数映射 (exponential map) 表示:

$$
\mathbf{C} = \exp(\boldsymbol{\varphi}^\times) = \mathbf{I} + \frac{\sin\|\boldsymbol{\varphi}\|}{\|\boldsymbol{\varphi}\|} \boldsymbol{\varphi}^\times + \frac{1 - \cos\|\boldsymbol{\varphi}\|}{\|\boldsymbol{\varphi}\|^2} (\boldsymbol{\varphi}^\times)^2
$$

反之，对数映射 (logarithmic map) 从旋转矩阵恢复旋转向量:

$$
\boldsymbol{\varphi}^\times = \log(\mathbf{C}) = \frac{\theta}{2\sin\theta}(\mathbf{C} - \mathbf{C}^T)
$$

**优点:** 仅用 3 个参数，无冗余。

**缺点:** 当 $\theta = 0$ 时旋转轴不确定；当 $\theta = \pi$ 时 $\sin\theta = 0$，提取旋转轴需要特殊处理。


### 2.5 四元数 (Quaternions)

#### 2.5.1 定义与基本运算

四元数 (quaternion) 是一种四维超复数 (hypercomplex number)，由 Hamilton 于 1843 年发明:

$$
\xi = \xi_0 + \xi_1 \mathbf{i} + \xi_2 \mathbf{j} + \xi_3 \mathbf{k}
$$

其中 $\xi_0 \in \mathbb{R}$ 为标量部分 (scalar part)，$\tilde{\boldsymbol{\xi}} = (\xi_1, \xi_2, \xi_3)^T \in \mathbb{R}^3$ 为向量部分 (vector part)。

虚数单位满足:

$$
\mathbf{i}^2 = \mathbf{j}^2 = \mathbf{k}^2 = \mathbf{i}\mathbf{j}\mathbf{k} = -1
$$

由此推导出:

$$
\mathbf{i}\mathbf{j} = \mathbf{k}, \quad \mathbf{j}\mathbf{k} = \mathbf{i}, \quad \mathbf{k}\mathbf{i} = \mathbf{j}
$$

$$
\mathbf{j}\mathbf{i} = -\mathbf{k}, \quad \mathbf{k}\mathbf{j} = -\mathbf{i}, \quad \mathbf{i}\mathbf{k} = -\mathbf{j}
$$

注意四元数乘法是**非交换的** (non-commutative)。

紧凑记法:

$$
\xi = \begin{pmatrix} \xi_0 \ \tilde{\boldsymbol{\xi}} \end{pmatrix} \in \mathbb{R}^4
$$

#### 2.5.2 单位四元数与旋转

用于表示旋转的四元数必须满足**单位约束** (unit constraint):

$$
\|\xi\|^2 = \xi_0^2 + \xi_1^2 + \xi_2^2 + \xi_3^2 = 1
$$

单位四元数构成三维球面 $S^3 \subset \mathbb{R}^4$。

**从轴角到四元数:**

给定旋转轴 $\mathbf{n}$ 和旋转角 $\theta$:

$$
\xi = \begin{pmatrix} \cos(\theta/2) \ \sin(\theta/2) \, \mathbf{n} \end{pmatrix} = \begin{pmatrix} \xi_0 \ \tilde{\boldsymbol{\xi}} \end{pmatrix}
$$

可以验证 $\|\xi\| = \cos^2(\theta/2) + \sin^2(\theta/2) \|\mathbf{n}\|^2 = 1$。

**关键优势:**

- 4 个参数 + 1 个约束 = 3 个自由度
- **无奇异性** (singularity-free)，这是相比欧拉角的最大优势
- 计算效率高，数值稳定
- 双覆盖性: $\xi$ 和 $-\xi$ 表示同一旋转

#### 2.5.3 四元数乘法

两个四元数 $\mathbf{q}$ 和 $\mathbf{p}$ 的乘积（Hamilton 积）:

$$
\mathbf{q} \otimes \mathbf{p} = \begin{pmatrix} q_0 p_0 - \tilde{\mathbf{q}}^T \tilde{\mathbf{p}} \ q_0 \tilde{\mathbf{p}} + p_0 \tilde{\mathbf{q}} + \tilde{\mathbf{q}} \times \tilde{\mathbf{p}} \end{pmatrix}
$$

这可以用矩阵形式表示:

**左乘矩阵 (Left multiplication matrix):**

$$
\mathbf{q} \otimes \mathbf{p} = \mathbf{M}_l(\mathbf{q}) \, \mathbf{p}
$$

$$
\mathbf{M}_l(\mathbf{q}) = \begin{pmatrix} q_0 & -\tilde{\mathbf{q}}^T \ \tilde{\mathbf{q}} & q_0 \mathbf{I} + \tilde{\mathbf{q}}^\times \end{pmatrix} \in \mathbb{R}^{4 \times 4}
$$

**右乘矩阵 (Right multiplication matrix):**

$$
\mathbf{q} \otimes \mathbf{p} = \mathbf{M}_r(\mathbf{p}) \, \mathbf{q}
$$

$$
\mathbf{M}_r(\mathbf{p}) = \begin{pmatrix} p_0 & -\tilde{\mathbf{p}}^T \ \tilde{\mathbf{p}} & p_0 \mathbf{I} - \tilde{\mathbf{p}}^\times \end{pmatrix} \in \mathbb{R}^{4 \times 4}
$$

注意两者的区别仅在于叉积项的符号: $\mathbf{M}_l$ 中为 $+\tilde{\mathbf{q}}^\times$，$\mathbf{M}_r$ 中为 $-\tilde{\mathbf{p}}^\times$。

**四元数共轭 (Conjugate):**

$$
\xi^* = \begin{pmatrix} \xi_0 \ -\tilde{\boldsymbol{\xi}} \end{pmatrix}
$$

对于单位四元数，共轭等于逆: $\xi^{-1} = \xi^*$，且 $\xi \otimes \xi^* = \xi^* \otimes \xi = \begin{pmatrix} 1 \ \mathbf{0} \end{pmatrix}$。

#### 2.5.4 用四元数旋转向量

将三维向量 $\mathbf{r}$ 嵌入为纯四元数 (pure quaternion):

$$
\mathbf{p}_r = \begin{pmatrix} 0 \ \mathbf{r} \end{pmatrix}
$$

旋转操作:

$$
{}_B\mathbf{p}_r = \boldsymbol{\xi}_{BI} \otimes {}_I\mathbf{p}_r \otimes \boldsymbol{\xi}_{BI}^*
$$

即:

$$
\begin{pmatrix} 0 \ {}_B\mathbf{r} \end{pmatrix} = \boldsymbol{\xi}_{BI} \otimes \begin{pmatrix} 0 \ {}_I\mathbf{r} \end{pmatrix} \otimes \boldsymbol{\xi}_{BI}^*
$$

结果的标量部分恒为零，向量部分即为旋转后的向量。

**推导验证:**

利用矩阵形式:

$$
\boldsymbol{\xi} \otimes \mathbf{p}_r \otimes \boldsymbol{\xi}^* = \mathbf{M}_l(\boldsymbol{\xi}) \, \mathbf{M}_r(\boldsymbol{\xi}^*) \, \mathbf{p}_r
$$

展开 $\mathbf{M}_l(\boldsymbol{\xi}) \, \mathbf{M}_r(\boldsymbol{\xi}^*)$ 的 $(2:4, 2:4)$ 子块，即可得到旋转矩阵。

#### 2.5.5 从四元数推导旋转矩阵

通过展开四元数旋转公式，可以得到旋转矩阵的四元数表达:

$$
\boxed{\mathbf{C}(\boldsymbol{\xi}) = (2\xi_0^2 - 1)\mathbf{I} + 2\xi_0 \tilde{\boldsymbol{\xi}}^\times + 2\tilde{\boldsymbol{\xi}}\tilde{\boldsymbol{\xi}}^T}
$$

**详细推导:**

从 $\boldsymbol{\xi} \otimes \mathbf{p}_r \otimes \boldsymbol{\xi}^*$ 出发，先计算 $\boldsymbol{\xi} \otimes \mathbf{p}_r$:

$$
\boldsymbol{\xi} \otimes \mathbf{p}_r = \begin{pmatrix} -\tilde{\boldsymbol{\xi}}^T \mathbf{r} \ \xi_0 \mathbf{r} + \tilde{\boldsymbol{\xi}} \times \mathbf{r} \end{pmatrix}
$$

再右乘 $\boldsymbol{\xi}^*$，取向量部分:

$$
\mathbf{r}' = (\xi_0^2 - \tilde{\boldsymbol{\xi}}^T\tilde{\boldsymbol{\xi}})\mathbf{r} + 2(\tilde{\boldsymbol{\xi}}^T\mathbf{r})\tilde{\boldsymbol{\xi}} + 2\xi_0(\tilde{\boldsymbol{\xi}} \times \mathbf{r})
$$

利用单位约束 $\xi_0^2 + \tilde{\boldsymbol{\xi}}^T\tilde{\boldsymbol{\xi}} = 1$，即 $\xi_0^2 - \tilde{\boldsymbol{\xi}}^T\tilde{\boldsymbol{\xi}} = 2\xi_0^2 - 1$:

$$
\mathbf{r}' = (2\xi_0^2 - 1)\mathbf{r} + 2(\tilde{\boldsymbol{\xi}}^T\mathbf{r})\tilde{\boldsymbol{\xi}} + 2\xi_0(\tilde{\boldsymbol{\xi}} \times \mathbf{r})
$$

写成矩阵形式:

$$
\mathbf{r}' = \left[(2\xi_0^2 - 1)\mathbf{I} + 2\tilde{\boldsymbol{\xi}}\tilde{\boldsymbol{\xi}}^T + 2\xi_0 \tilde{\boldsymbol{\xi}}^\times\right] \mathbf{r}
$$

因此 $\mathbf{C}(\boldsymbol{\xi}) = (2\xi_0^2 - 1)\mathbf{I} + 2\xi_0 \tilde{\boldsymbol{\xi}}^\times + 2\tilde{\boldsymbol{\xi}}\tilde{\boldsymbol{\xi}}^T$。

**展开为元素形式:**

$$
\mathbf{C}(\boldsymbol{\xi}) = \begin{pmatrix}
1 - 2(\xi_2^2 + \xi_3^2) & 2(\xi_1\xi_2 - \xi_0\xi_3) & 2(\xi_1\xi_3 + \xi_0\xi_2) \
2(\xi_1\xi_2 + \xi_0\xi_3) & 1 - 2(\xi_1^2 + \xi_3^2) & 2(\xi_2\xi_3 - \xi_0\xi_1) \
2(\xi_1\xi_3 - \xi_0\xi_2) & 2(\xi_2\xi_3 + \xi_0\xi_1) & 1 - 2(\xi_1^2 + \xi_2^2)
\end{pmatrix}
$$

> **Quiz 3:** 验证当 $\boldsymbol{\xi} = (\cos(\theta/2), \sin(\theta/2), 0, 0)^T$（绕 $x$ 轴旋转 $\theta$）时，$\mathbf{C}(\boldsymbol{\xi})$ 化简为 $\mathbf{C}_x(\theta)$。
>
> **解答:**
> 代入 $\xi_0 = \cos(\theta/2)$, $\xi_1 = \sin(\theta/2)$, $\xi_2 = \xi_3 = 0$:
> $$\mathbf{C} = \begin{pmatrix} 1 & 0 & 0 \ 0 & 1 - 2\sin^2(\theta/2) & -2\cos(\theta/2)\sin(\theta/2) \ 0 & 2\cos(\theta/2)\sin(\theta/2) & 1 - 2\sin^2(\theta/2) \end{pmatrix}$$
> 利用二倍角公式 $\cos\theta = 1 - 2\sin^2(\theta/2)$, $\sin\theta = 2\sin(\theta/2)\cos(\theta/2)$:
> $$\mathbf{C} = \begin{pmatrix} 1 & 0 & 0 \ 0 & \cos\theta & -\sin\theta \ 0 & \sin\theta & \cos\theta \end{pmatrix} = \mathbf{C}_x(\theta) \quad \checkmark$$

### 2.6 旋转参数化方法总结

| 方法 | 参数数 | 约束数 | 自由度 | 奇异性 | 备注 |
|------|--------|--------|--------|--------|------|
| 旋转矩阵 | 9 | 6 | 3 | 无 | 冗余最多，但计算直接 |
| 欧拉角 | 3 | 0 | 3 | 有 (Gimbal Lock) | 直观，但有奇异性 |
| 轴角 | 4 | 1 ($\|\mathbf{n}\|=1$) | 3 | $\theta=0,\pi$ | 几何意义清晰 |
| 旋转向量 | 3 | 0 | 3 | $\theta=0,\pi$ | 紧凑，适合小角度 |
| 四元数 | 4 | 1 ($\|\xi\|=1$) | 3 | **无** | 数值最优，广泛使用 |

> **重要结论:** 根据拓扑学，不存在用最少参数（3个）且无奇异性的旋转参数化方法。要避免奇异性，必须使用冗余参数（如四元数的 4 个参数或旋转矩阵的 9 个参数）。

---

## 3. 角速度与时间导数

### 3.1 角速度的定义

刚体的角速度 (angular velocity) $\boldsymbol{\omega}$ 描述了刚体姿态随时间变化的速率。与线速度类似，角速度与旋转参数的时间导数之间存在映射关系:

$$
\boldsymbol{\omega} = \mathbf{E}(\boldsymbol{\chi}_R) \cdot \dot{\boldsymbol{\chi}}_R
$$

其中 $\mathbf{E}(\boldsymbol{\chi}_R) \in \mathbb{R}^{3 \times n}$ 是角速度映射矩阵，$\boldsymbol{\chi}_R$ 是旋转参数。

**角速度与旋转矩阵的关系:**

旋转矩阵满足正交性 $\mathbf{C}\mathbf{C}^T = \mathbf{I}$，对时间求导:

$$
\dot{\mathbf{C}}\mathbf{C}^T + \mathbf{C}\dot{\mathbf{C}}^T = \mathbf{0}
$$

因此 $\dot{\mathbf{C}}\mathbf{C}^T$ 是反对称矩阵 (skew-symmetric matrix)，定义:

$$
\boldsymbol{\omega}^\times = \dot{\mathbf{C}}\mathbf{C}^T \quad \Longleftrightarrow \quad \dot{\mathbf{C}} = \boldsymbol{\omega}^\times \mathbf{C}
$$

这里 $\boldsymbol{\omega}$ 是在**固定坐标系** (space frame) 中表示的角速度。

如果在**体坐标系** (body frame) 中表示角速度 $\boldsymbol{\Omega}$:

$$
\boldsymbol{\Omega}^\times = \mathbf{C}^T \dot{\mathbf{C}} \quad \Longleftrightarrow \quad \dot{\mathbf{C}} = \mathbf{C} \boldsymbol{\Omega}^\times
$$

两者的关系: $\boldsymbol{\omega} = \mathbf{C} \boldsymbol{\Omega}$。

### 3.2 欧拉角的角速度映射

#### 3.2.1 ZYX 欧拉角的 E 矩阵推导

对于 ZYX 欧拉角 $\boldsymbol{\chi}_R = (\psi, \theta, \phi)^T$，旋转矩阵为:

$$
\mathbf{C} = \mathbf{C}_z(\psi) \cdot \mathbf{C}_y(\theta) \cdot \mathbf{C}_x(\phi)
$$

角速度是三次旋转各自贡献的叠加。每次旋转贡献的角速度沿其旋转轴方向:

- 绕 $z$ 轴旋转 $\psi$: 贡献 $\dot{\psi} \, \mathbf{e}_z$（在惯性系中）
- 绕 $y'$ 轴旋转 $\theta$: 贡献 $\dot{\theta} \, \mathbf{e}_{y'}$（在第一次旋转后的坐标系中）
- 绕 $x''$ 轴旋转 $\phi$: 贡献 $\dot{\phi} \, \mathbf{e}_{x''}$（在第二次旋转后的坐标系中）

将所有贡献统一到惯性系 $I$ 中表示:

$$
{}_I\boldsymbol{\omega} = \dot{\psi} \begin{pmatrix} 0 \ 0 \ 1 \end{pmatrix} + \dot{\theta} \, \mathbf{C}_z(\psi) \begin{pmatrix} 0 \ 1 \ 0 \end{pmatrix} + \dot{\phi} \, \mathbf{C}_z(\psi)\mathbf{C}_y(\theta) \begin{pmatrix} 1 \ 0 \ 0 \end{pmatrix}
$$

展开计算:

$$
\mathbf{C}_z(\psi) \begin{pmatrix} 0 \ 1 \ 0 \end{pmatrix} = \begin{pmatrix} -\sin\psi \ \cos\psi \ 0 \end{pmatrix}
$$

$$
\mathbf{C}_z(\psi)\mathbf{C}_y(\theta) \begin{pmatrix} 1 \ 0 \ 0 \end{pmatrix} = \begin{pmatrix} \cos\psi\cos\theta \ \sin\psi\cos\theta \ -\sin\theta \end{pmatrix}
$$

因此:

$$
{}_I\boldsymbol{\omega} = \underbrace{\begin{pmatrix} \cos\psi\cos\theta & -\sin\psi & 0 \ \sin\psi\cos\theta & \cos\psi & 0 \ -\sin\theta & 0 & 1 \end{pmatrix}}_{\mathbf{E}_{ZYX}(\psi, \theta, \phi)} \begin{pmatrix} \dot{\phi} \ \dot{\theta} \ \dot{\psi} \end{pmatrix}
$$

**注意参数顺序:** 这里 $\dot{\boldsymbol{\chi}}_R = (\dot{\phi}, \dot{\theta}, \dot{\psi})^T$。

**奇异性分析:**

$$
\det(\mathbf{E}_{ZYX}) = \cos\theta
$$

当 $\theta = \pm\frac{\pi}{2}$ 时，$\det(\mathbf{E}) = 0$，映射矩阵奇异，无法从 $\boldsymbol{\omega}$ 唯一求解 $\dot{\boldsymbol{\chi}}_R$。这就是万向节锁 (Gimbal Lock) 的数学本质。

#### 3.2.2 ZYZ 欧拉角的 E 矩阵

类似推导，对于 ZYZ 欧拉角 $(\alpha, \beta, \gamma)$:

$$
{}_I\boldsymbol{\omega} = \dot{\alpha} \begin{pmatrix} 0 \ 0 \ 1 \end{pmatrix} + \dot{\beta} \, \mathbf{C}_z(\alpha) \begin{pmatrix} 0 \ 1 \ 0 \end{pmatrix} + \dot{\gamma} \, \mathbf{C}_z(\alpha)\mathbf{C}_y(\beta) \begin{pmatrix} 0 \ 0 \ 1 \end{pmatrix}
$$

$$
\mathbf{E}_{ZYZ} = \begin{pmatrix} -\sin\alpha\sin\beta & \cos\alpha & 0 \ \cos\alpha\sin\beta & \sin\alpha & 0 \ \cos\beta & 0 & 1 \end{pmatrix}
$$

奇异性: $\det(\mathbf{E}_{ZYZ}) = -\sin\beta$，当 $\beta = 0, \pi$ 时奇异。

### 3.3 旋转矩阵的时间导数

由前面的推导:

$$
\dot{\mathbf{C}} = \boldsymbol{\omega}^\times \mathbf{C}
$$

这是一个矩阵微分方程。对于旋转矩阵，其 9 个元素的导数由 3 个角速度分量完全确定。

### 3.4 轴角表示的时间导数

对于轴角表示 $(\mathbf{n}, \theta)$，角速度为:

$$
\boldsymbol{\omega} = \dot{\theta} \, \mathbf{n} + \sin\theta \, \dot{\mathbf{n}} + (1 - \cos\theta)(\mathbf{n} \times \dot{\mathbf{n}})
$$

当旋转轴固定（$\dot{\mathbf{n}} = \mathbf{0}$）时，简化为:

$$
\boldsymbol{\omega} = \dot{\theta} \, \mathbf{n}
$$

这与直觉一致: 绕固定轴旋转时，角速度方向沿旋转轴，大小为角度的变化率。

### 3.5 旋转向量的时间导数

对于旋转向量 $\boldsymbol{\varphi} = \theta \mathbf{n}$:

$$
\boldsymbol{\omega} = \mathbf{A}(\boldsymbol{\varphi}) \, \dot{\boldsymbol{\varphi}}
$$

其中映射矩阵:

$$
\mathbf{A}(\boldsymbol{\varphi}) = \mathbf{I} + \frac{1 - \cos\|\boldsymbol{\varphi}\|}{\|\boldsymbol{\varphi}\|^2} \boldsymbol{\varphi}^\times + \frac{\|\boldsymbol{\varphi}\| - \sin\|\boldsymbol{\varphi}\|}{\|\boldsymbol{\varphi}\|^3} (\boldsymbol{\varphi}^\times)^2
$$

当 $\|\boldsymbol{\varphi}\| \to 0$ 时，$\mathbf{A} \to \mathbf{I}$，即小角度近似下 $\boldsymbol{\omega} \approx \dot{\boldsymbol{\varphi}}$。

### 3.6 四元数的时间导数

对单位四元数 $\boldsymbol{\xi}$ 求时间导数，利用 $\dot{\mathbf{C}} = \boldsymbol{\omega}^\times \mathbf{C}$ 的关系，可以推导出:

$$
\dot{\boldsymbol{\xi}} = \frac{1}{2} \boldsymbol{\omega}_\xi \otimes \boldsymbol{\xi}
$$

其中 $\boldsymbol{\omega}_\xi = \begin{pmatrix} 0 \ \boldsymbol{\omega} \end{pmatrix}$ 是角速度对应的纯四元数。

用矩阵形式表示:

$$
\dot{\boldsymbol{\xi}} = \frac{1}{2} \mathbf{M}_l(\boldsymbol{\omega}_\xi) \, \boldsymbol{\xi} = \frac{1}{2} \begin{pmatrix} 0 & -\boldsymbol{\omega}^T \ \boldsymbol{\omega} & \boldsymbol{\omega}^\times \end{pmatrix} \boldsymbol{\xi}
$$

等价地，用体坐标系角速度 $\boldsymbol{\Omega}$ 表示:

$$
\dot{\boldsymbol{\xi}} = \frac{1}{2} \boldsymbol{\xi} \otimes \boldsymbol{\Omega}_\xi = \frac{1}{2} \mathbf{M}_r(\boldsymbol{\Omega}_\xi) \, \boldsymbol{\xi}
$$

**角速度映射矩阵:**

将上式改写为 $\boldsymbol{\omega} = \mathbf{E}_\xi \, \dot{\boldsymbol{\xi}}$ 的形式:

$$
\boldsymbol{\omega} = 2 \begin{pmatrix} -\tilde{\boldsymbol{\xi}}^T \ \xi_0 \mathbf{I} + \tilde{\boldsymbol{\xi}}^\times \end{pmatrix}^{-T} \dot{\boldsymbol{\xi}}
$$

由于单位四元数约束，实际上:

$$
\boldsymbol{\omega} = 2 \begin{pmatrix} -\xi_1 & -\xi_2 & -\xi_3 \ \xi_0 & -\xi_3 & \xi_2 \ \xi_3 & \xi_0 & -\xi_1 \ -\xi_2 & \xi_1 & \xi_0 \end{pmatrix}^T \dot{\boldsymbol{\xi}} = \mathbf{E}_\xi(\boldsymbol{\xi}) \, \dot{\boldsymbol{\xi}}
$$

> **Quiz 4:** 证明单位四元数的时间导数满足约束 $\boldsymbol{\xi}^T \dot{\boldsymbol{\xi}} = 0$。
>
> **解答:**
> 对单位约束 $\boldsymbol{\xi}^T \boldsymbol{\xi} = 1$ 两边求时间导数:
> $$\frac{d}{dt}(\boldsymbol{\xi}^T \boldsymbol{\xi}) = 2\boldsymbol{\xi}^T \dot{\boldsymbol{\xi}} = 0$$
> 因此 $\boldsymbol{\xi}^T \dot{\boldsymbol{\xi}} = 0$，即 $\dot{\boldsymbol{\xi}}$ 始终与 $\boldsymbol{\xi}$ 正交。
> 这也可以从 $\dot{\boldsymbol{\xi}} = \frac{1}{2}\boldsymbol{\omega}_\xi \otimes \boldsymbol{\xi}$ 验证:
> $$\boldsymbol{\xi}^T \dot{\boldsymbol{\xi}} = \frac{1}{2}\boldsymbol{\xi}^T \mathbf{M}_l(\boldsymbol{\omega}_\xi)\boldsymbol{\xi} = 0$$
> 因为 $\mathbf{M}_l(\boldsymbol{\omega}_\xi)$ 对于纯四元数 $\boldsymbol{\omega}_\xi$ 是反对称的。

---

## 4. 单刚体的位置与姿态

### 4.1 刚体的完整描述

一个刚体 (rigid body) 在三维空间中的构型 (configuration) 由两部分完全确定:

1. **位置 (Position):** 体坐标系原点相对于惯性系的位置向量 ${}_I\mathbf{r}_{IB}$
2. **姿态 (Orientation):** 体坐标系相对于惯性系的旋转 $\mathbf{C}_{IB}$

因此，刚体的构型空间 (configuration space) 为:

$$
SE(3) = \mathbb{R}^3 \times SO(3)
$$

总自由度: $3 + 3 = 6$。

### 4.2 刚体上任意一点的位置

设刚体 $B$ 上一点 $P$ 在体坐标系中的位置为 ${}_B\mathbf{r}_{BP}$（常量，因为是刚体），则该点在惯性系中的位置为:

$$
{}_I\mathbf{r}_{IP} = {}_I\mathbf{r}_{IB} + \mathbf{C}_{IB} \, {}_B\mathbf{r}_{BP}
$$

这个公式是刚体运动学的基础，它将体坐标系中的固定向量通过旋转和平移变换到惯性系中。

### 4.3 刚体上任意一点的速度

对上式求时间导数:

$$
{}_I\dot{\mathbf{r}}_{IP} = {}_I\dot{\mathbf{r}}_{IB} + \dot{\mathbf{C}}_{IB} \, {}_B\mathbf{r}_{BP}
$$

注意 ${}_B\mathbf{r}_{BP}$ 是常量（刚体假设），且 $\dot{\mathbf{C}}_{IB} = {}_I\boldsymbol{\omega}_{IB}^\times \, \mathbf{C}_{IB}$，因此:

$$
{}_I\dot{\mathbf{r}}_{IP} = {}_I\dot{\mathbf{r}}_{IB} + {}_I\boldsymbol{\omega}_{IB}^\times \, \mathbf{C}_{IB} \, {}_B\mathbf{r}_{BP}
$$

$$
= {}_I\dot{\mathbf{r}}_{IB} + {}_I\boldsymbol{\omega}_{IB} \times ({}_I\mathbf{r}_{IP} - {}_I\mathbf{r}_{IB})
$$

这就是经典的**刚体速度公式** (rigid body velocity formula):

$$
\boxed{{}_I\mathbf{v}_P = {}_I\mathbf{v}_B + {}_I\boldsymbol{\omega}_{IB} \times {}_I\mathbf{r}_{BP}}
$$

其中 ${}_I\mathbf{r}_{BP} = \mathbf{C}_{IB} \, {}_B\mathbf{r}_{BP}$ 是从体坐标系原点到点 $P$ 的向量在惯性系中的表示。

### 4.4 广义速度向量

刚体的运动状态可以用 6 维广义速度向量 (generalized velocity vector / twist) 描述:

$$
\mathbf{V}_B = \begin{pmatrix} {}_I\mathbf{v}_B \ {}_I\boldsymbol{\omega}_{IB} \end{pmatrix} \in \mathbb{R}^6
$$

利用此向量，刚体上任意一点的速度可以紧凑地表示为:

$$
{}_I\mathbf{v}_P = \begin{pmatrix} \mathbf{I} & -{}_I\mathbf{r}_{BP}^\times \end{pmatrix} \mathbf{V}_B
$$

> **Quiz 5:** 一个刚体绕惯性系原点以角速度 $\boldsymbol{\omega} = (0, 0, \omega)^T$ 旋转（纯旋转，无平移），体坐标系原点位于 ${}_I\mathbf{r}_{IB} = (R, 0, 0)^T$。求体坐标系原点的线速度。
>
> **解答:**
> 纯旋转意味着惯性系原点是固定点，因此:
> $${}_I\mathbf{v}_B = \boldsymbol{\omega} \times {}_I\mathbf{r}_{IB} = \begin{pmatrix} 0 \ 0 \ \omega \end{pmatrix} \times \begin{pmatrix} R \ 0 \ 0 \end{pmatrix} = \begin{pmatrix} 0 \ \omega R \ 0 \end{pmatrix}$$
> 速度大小为 $\omega R$，方向沿 $y$ 轴，与直觉一致。

---

## 5. 串联运动链

### 5.1 基本概念

串联运动链 (serial kinematic linkage / chain) 是机器人学中最基本的机构形式，由一系列通过关节 (joints) 连接的连杆 (links) 组成。

**结构特征:**

- 开链结构 (open chain): 从基座 (base) 到末端执行器 (end-effector) 只有一条运动路径
- 每个连杆通过一个关节与前一个连杆相连
- 基座（连杆 0）固定在惯性系中

### 5.2 关节类型

**旋转关节 (Revolute Joint, R)**

- 自由度: 1 DOF
- 运动: 绕关节轴的纯旋转
- 关节变量: 旋转角度 $\theta_i$
- 约束数: 5（限制了 3 个平移和 2 个旋转）
- 符号: 通常用圆柱形表示

**移动关节 (Prismatic Joint, P)**

- 自由度: 1 DOF
- 运动: 沿关节轴的纯平移
- 关节变量: 平移距离 $d_i$
- 约束数: 5（限制了 2 个平移和 3 个旋转）
- 符号: 通常用方块形表示

**其他关节类型（了解）:**

| 关节类型 | DOF | 描述 |
|----------|-----|------|
| 球关节 (Spherical, S) | 3 | 三轴旋转 |
| 万向节 (Universal, U) | 2 | 两轴旋转 |
| 圆柱关节 (Cylindrical, C) | 2 | 一轴旋转 + 一轴平移 |
| 平面关节 (Planar, F) | 3 | 平面内两平移 + 一旋转 |

### 5.3 自由度计算

对于由 $n_j$ 个单自由度关节（旋转或移动）连接的串联运动链:

- 连杆数: $n_l = n_j + 1$（包括基座）
- 活动连杆数: $n_l - 1 = n_j$

**Grubler-Kutzbach 公式（空间机构）:**

$$
\text{DOF} = 6(n_l - 1) - \sum_{i=1}^{n_j} c_i
$$

其中 $c_i$ 是第 $i$ 个关节的约束数。

对于全部由 1-DOF 关节组成的串联链，每个关节约束 5 个自由度:

$$
\text{DOF} = 6 \cdot n_j - 5 \cdot n_j = n_j
$$

即**串联机器人的自由度等于关节数**。这是一个非常简洁的结果。

**典型工业机器人:** $n_j = 6$，恰好具有 6 个自由度，可以在三维空间中实现任意位置和姿态。

- $n_j < 6$: 欠驱动 (under-actuated)，无法到达任务空间中的所有构型
- $n_j > 6$: 冗余 (redundant)，同一末端构型对应多组关节角

> **Quiz 6:** 一个平面机器人臂（所有关节轴平行）有 3 个旋转关节。它的自由度是多少？它能在平面内实现任意位置和姿态吗？
>
> **解答:**
> 自由度 = 3（等于关节数）。平面内的位置和姿态需要 3 个自由度（$x, y, \theta$），因此 3 个旋转关节恰好足够在平面内实现任意位置和姿态（在工作空间范围内）。但它无法实现空间中的运动。

---

## 6. 广义坐标

### 6.1 定义与性质

广义坐标 (generalized coordinates) $\mathbf{q} = (q_1, q_2, \ldots, q_n)^T$ 是一组用于完整描述系统构型的独立参数。

**两个核心要求:**

1. **完备性 (Completeness):** 广义坐标必须能够唯一确定系统中每个刚体的位置和姿态。即给定 $\mathbf{q}$，系统的几何构型完全确定。

2. **独立性 (Independence):** 广义坐标之间不存在约束关系，即不存在形如 $f(\mathbf{q}) = 0$ 的等式约束。广义坐标的数量等于系统的自由度。

**数学表述:**

对于一个具有 $n$ 个自由度的系统，广义坐标 $\mathbf{q} \in \mathbb{R}^n$ 满足:

$$
\text{rank}\left(\frac{\partial \mathbf{r}_i}{\partial \mathbf{q}}\right) = n, \quad \forall i
$$

即系统中任意一点的位置对广义坐标的 Jacobian 矩阵满秩。

### 6.2 关节空间与任务空间

**关节空间 (Joint Space / Configuration Space)**

对于串联机器人，最自然的广义坐标选择是关节变量:

$$
\mathbf{q} = \begin{pmatrix} q_1 \ q_2 \ \vdots \ q_n \end{pmatrix} \in \mathbb{R}^n
$$

其中 $q_i$ 是第 $i$ 个关节的旋转角度（旋转关节）或平移距离（移动关节）。

关节空间的维度等于机器人的自由度 $n$。

**任务空间 (Task Space / Operational Space)**

任务空间描述末端执行器 (end-effector) 的位置和姿态:

$$
\mathbf{x} = \begin{pmatrix} \mathbf{r}_E \ \boldsymbol{\chi}_R \end{pmatrix} \in \mathbb{R}^m
$$

其中 $\mathbf{r}_E \in \mathbb{R}^3$ 是末端执行器的位置，$\boldsymbol{\chi}_R$ 是姿态参数。

对于空间机器人: $m = 6$（3 个位置 + 3 个姿态）。
对于平面机器人: $m = 3$（2 个位置 + 1 个姿态）。

**两个空间的关系:**

正运动学 (forward kinematics) 建立了从关节空间到任务空间的映射:

$$
\mathbf{x} = f(\mathbf{q})
$$

逆运动学 (inverse kinematics) 则是反向映射:

$$
\mathbf{q} = f^{-1}(\mathbf{x})
$$

逆运动学通常更困难，可能存在多解、无解或奇异性。

### 6.3 广义坐标的选择原则

| 原则 | 说明 |
|------|------|
| 最小性 | 参数数量等于自由度 |
| 独立性 | 参数之间无约束 |
| 物理直观性 | 尽量选择有物理意义的量 |
| 避免奇异性 | 在工作空间内尽量避免参数化奇异 |
| 计算便利性 | 便于推导运动方程 |

对于串联机器人，关节变量是最自然且最常用的广义坐标选择，因为它们天然满足独立性和完备性。

---

## 7. 正运动学

### 7.1 齐次变换矩阵 (Homogeneous Transformation Matrix)

为了同时处理旋转和平移，引入齐次变换矩阵 (homogeneous transformation matrix):

$$
\mathbf{T}_{AB} = \begin{pmatrix} \mathbf{C}_{AB} & {}_A\mathbf{r}_{AB} \ \mathbf{0}^T & 1 \end{pmatrix} \in SE(3) \subset \mathbb{R}^{4 \times 4}
$$

其中:
- $\mathbf{C}_{AB} \in SO(3)$: 坐标系 $B$ 相对于 $A$ 的旋转矩阵
- ${}_A\mathbf{r}_{AB} \in \mathbb{R}^3$: 坐标系 $B$ 的原点在 $A$ 系中的位置向量

**齐次坐标 (Homogeneous Coordinates):**

将三维向量扩展为四维齐次坐标:

$$
\bar{\mathbf{r}} = \begin{pmatrix} \mathbf{r} \ 1 \end{pmatrix} \in \mathbb{R}^4
$$

则坐标变换可以统一为矩阵乘法:

$$
{}_A\bar{\mathbf{r}}_P = \mathbf{T}_{AB} \, {}_B\bar{\mathbf{r}}_P
$$

展开即为:

$$
\begin{pmatrix} {}_A\mathbf{r}_P \ 1 \end{pmatrix} = \begin{pmatrix} \mathbf{C}_{AB} & {}_A\mathbf{r}_{AB} \ \mathbf{0}^T & 1 \end{pmatrix} \begin{pmatrix} {}_B\mathbf{r}_P \ 1 \end{pmatrix} = \begin{pmatrix} \mathbf{C}_{AB} \, {}_B\mathbf{r}_P + {}_A\mathbf{r}_{AB} \ 1 \end{pmatrix}
$$

**基本性质:**

逆变换:

$$
\mathbf{T}_{AB}^{-1} = \mathbf{T}_{BA} = \begin{pmatrix} \mathbf{C}_{AB}^T & -\mathbf{C}_{AB}^T \, {}_A\mathbf{r}_{AB} \ \mathbf{0}^T & 1 \end{pmatrix}
$$

链式法则:

$$
\mathbf{T}_{AC} = \mathbf{T}_{AB} \cdot \mathbf{T}_{BC}
$$

恒等变换:

$$
\mathbf{T}_{AA} = \mathbf{I}_{4 \times 4}
$$

### 7.2 正运动学的链式结构

对于串联机器人，从基座到末端执行器的变换是各关节变换的连乘:

$$
\mathbf{T}_{IE} = \mathbf{T}_{I0} \cdot \mathbf{T}_{01}(q_1) \cdot \mathbf{T}_{12}(q_2) \cdots \mathbf{T}_{(n-1)n}(q_n) \cdot \mathbf{T}_{nE}
$$

其中:
- $\mathbf{T}_{I0}$: 惯性系到基座坐标系的固定变换
- $\mathbf{T}_{(i-1)i}(q_i)$: 第 $i$ 个关节引起的变换，仅依赖于关节变量 $q_i$
- $\mathbf{T}_{nE}$: 最后一个连杆坐标系到末端执行器坐标系的固定变换

**关键观察:** 每个 $\mathbf{T}_{(i-1)i}(q_i)$ 仅依赖于一个关节变量，这使得正运动学的计算具有递推结构。

### 7.3 基本关节变换

**旋转关节（绕 $z$ 轴）:**

$$
\mathbf{T}_{(i-1)i}(\theta_i) = \begin{pmatrix} \cos\theta_i & -\sin\theta_i & 0 & 0 \ \sin\theta_i & \cos\theta_i & 0 & 0 \ 0 & 0 & 1 & 0 \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

**移动关节（沿 $z$ 轴）:**

$$
\mathbf{T}_{(i-1)i}(d_i) = \begin{pmatrix} 1 & 0 & 0 & 0 \ 0 & 1 & 0 & 0 \ 0 & 0 & 1 & d_i \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

实际机器人中，关节变换通常还包含连杆几何参数（如 DH 参数），这将在后续课程中详细讨论。


### 7.4 平面机器人臂示例 (Planar Robot Arm Example)

考虑一个平面 3R 机器人臂（三个旋转关节，所有关节轴平行于 $z$ 轴），这是理解正运动学的经典示例。

**系统描述:**

- 连杆 0: 基座（固定）
- 连杆 1: 长度 $l_1$，关节角 $q_1$
- 连杆 2: 长度 $l_2$，关节角 $q_2$
- 连杆 3: 长度 $l_3$，关节角 $q_3$
- 末端执行器 $E$ 位于连杆 3 的末端

所有运动在 $xy$ 平面内进行。

**坐标系定义:**

- 坐标系 $I$（惯性系）: 原点在基座，$x$ 轴水平，$y$ 轴竖直
- 坐标系 $0$: 与惯性系重合，$\mathbf{T}_{I0} = \mathbf{I}$
- 坐标系 $i$: 原点在第 $i$ 个关节处，$x_i$ 轴沿连杆 $i$ 方向

**各关节变换矩阵:**

$$
\mathbf{T}_{01}(q_1) = \begin{pmatrix} \cos q_1 & -\sin q_1 & 0 & 0 \ \sin q_1 & \cos q_1 & 0 & 0 \ 0 & 0 & 1 & 0 \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

$$
\mathbf{T}_{12}(q_2) = \begin{pmatrix} \cos q_2 & -\sin q_2 & 0 & l_1 \ \sin q_2 & \cos q_2 & 0 & 0 \ 0 & 0 & 1 & 0 \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

$$
\mathbf{T}_{23}(q_3) = \begin{pmatrix} \cos q_3 & -\sin q_3 & 0 & l_2 \ \sin q_3 & \cos q_3 & 0 & 0 \ 0 & 0 & 1 & 0 \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

$$
\mathbf{T}_{3E} = \begin{pmatrix} 1 & 0 & 0 & l_3 \ 0 & 1 & 0 & 0 \ 0 & 0 & 1 & 0 \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

**正运动学计算:**

$$
\mathbf{T}_{IE} = \mathbf{T}_{I0} \cdot \mathbf{T}_{01}(q_1) \cdot \mathbf{T}_{12}(q_2) \cdot \mathbf{T}_{23}(q_3) \cdot \mathbf{T}_{3E}
$$

由于 $\mathbf{T}_{I0} = \mathbf{I}$，逐步计算:

**第一步: $\mathbf{T}_{02} = \mathbf{T}_{01} \cdot \mathbf{T}_{12}$**

定义 $q_{12} = q_1 + q_2$，利用三角恒等式:

$$
\mathbf{T}_{02} = \begin{pmatrix} \cos q_{12} & -\sin q_{12} & 0 & l_1 \cos q_1 \ \sin q_{12} & \cos q_{12} & 0 & l_1 \sin q_1 \ 0 & 0 & 1 & 0 \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

**第二步: $\mathbf{T}_{03} = \mathbf{T}_{02} \cdot \mathbf{T}_{23}$**

定义 $q_{123} = q_1 + q_2 + q_3$:

$$
\mathbf{T}_{03} = \begin{pmatrix} \cos q_{123} & -\sin q_{123} & 0 & l_1 \cos q_1 + l_2 \cos q_{12} \ \sin q_{123} & \cos q_{123} & 0 & l_1 \sin q_1 + l_2 \sin q_{12} \ 0 & 0 & 1 & 0 \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

**第三步: $\mathbf{T}_{IE} = \mathbf{T}_{0E} = \mathbf{T}_{03} \cdot \mathbf{T}_{3E}$**

$$
\mathbf{T}_{IE} = \begin{pmatrix} \cos q_{123} & -\sin q_{123} & 0 & l_1 \cos q_1 + l_2 \cos q_{12} + l_3 \cos q_{123} \ \sin q_{123} & \cos q_{123} & 0 & l_1 \sin q_1 + l_2 \sin q_{12} + l_3 \sin q_{123} \ 0 & 0 & 1 & 0 \ 0 & 0 & 0 & 1 \end{pmatrix}
$$

**提取末端执行器位姿:**

末端执行器位置:

$$
{}_I\mathbf{r}_{IE} = \begin{pmatrix} l_1 \cos q_1 + l_2 \cos(q_1 + q_2) + l_3 \cos(q_1 + q_2 + q_3) \ l_1 \sin q_1 + l_2 \sin(q_1 + q_2) + l_3 \sin(q_1 + q_2 + q_3) \ 0 \end{pmatrix}
$$

末端执行器姿态（平面内旋转角）:

$$
\alpha_E = q_1 + q_2 + q_3
$$

这是一个非常直观的结果: 末端执行器的朝向角等于所有关节角之和。

**正运动学的紧凑形式:**

在平面情况下，任务空间坐标为 $\mathbf{x} = (x_E, y_E, \alpha_E)^T$:

$$
\mathbf{x} = f(\mathbf{q}) = \begin{pmatrix} \sum_{k=1}^{3} l_k \cos\!\left(\sum_{j=1}^{k} q_j\right) \[6pt] \sum_{k=1}^{3} l_k \sin\!\left(\sum_{j=1}^{k} q_j\right) \[6pt] \sum_{j=1}^{3} q_j \end{pmatrix}
$$

这个公式可以直接推广到 $n$ 个关节的平面机器人臂。

### 7.5 各连杆质心的正运动学

在动力学分析中，我们不仅需要末端执行器的位姿，还需要每个连杆质心 (center of mass) 的位置。

设连杆 $i$ 的质心在连杆坐标系 $i$ 中的位置为 ${}_i\mathbf{r}_{i,\text{com}}$，则质心在惯性系中的位置为:

$$
{}_I\mathbf{r}_{i,\text{com}} = {}_I\mathbf{r}_{Ii} + \mathbf{C}_{Ii} \, {}_i\mathbf{r}_{i,\text{com}}
$$

其中 ${}_I\mathbf{r}_{Ii}$ 和 $\mathbf{C}_{Ii}$ 可以从 $\mathbf{T}_{Ii}$ 中提取:

$$
\mathbf{T}_{Ii} = \mathbf{T}_{I0} \cdot \mathbf{T}_{01}(q_1) \cdots \mathbf{T}_{(i-1)i}(q_i)
$$

对于平面 3R 机器人，假设每个连杆的质心位于连杆中点:

$$
{}_I\mathbf{r}_{1,\text{com}} = \begin{pmatrix} \frac{l_1}{2}\cos q_1 \ \frac{l_1}{2}\sin q_1 \ 0 \end{pmatrix}
$$

$$
{}_I\mathbf{r}_{2,\text{com}} = \begin{pmatrix} l_1\cos q_1 + \frac{l_2}{2}\cos q_{12} \ l_1\sin q_1 + \frac{l_2}{2}\sin q_{12} \ 0 \end{pmatrix}
$$

$$
{}_I\mathbf{r}_{3,\text{com}} = \begin{pmatrix} l_1\cos q_1 + l_2\cos q_{12} + \frac{l_3}{2}\cos q_{123} \ l_1\sin q_1 + l_2\sin q_{12} + \frac{l_3}{2}\sin q_{123} \ 0 \end{pmatrix}
$$

> **Quiz 7:** 对于平面 2R 机器人（$l_1 = l_2 = 1$ m），当 $q_1 = \pi/4$, $q_2 = \pi/2$ 时，求末端执行器的位置。
>
> **解答:**
> $$x_E = l_1\cos q_1 + l_2\cos(q_1 + q_2) = \cos\frac{\pi}{4} + \cos\frac{3\pi}{4} = \frac{\sqrt{2}}{2} - \frac{\sqrt{2}}{2} = 0$$
> $$y_E = l_1\sin q_1 + l_2\sin(q_1 + q_2) = \sin\frac{\pi}{4} + \sin\frac{3\pi}{4} = \frac{\sqrt{2}}{2} + \frac{\sqrt{2}}{2} = \sqrt{2}$$
> 因此 ${}_I\mathbf{r}_{IE} = (0, \sqrt{2}, 0)^T \approx (0, 1.414, 0)^T$ m。
> 
> 几何解释: 第一个连杆指向 $45°$ 方向，第二个连杆再转 $90°$ 指向 $135°$ 方向，两者的 $x$ 分量恰好抵消，$y$ 分量叠加。

### 7.6 正运动学的一般性讨论

**计算复杂度:**

正运动学的计算本质上是一系列 $4 \times 4$ 矩阵的连乘，计算复杂度为 $O(n)$，其中 $n$ 是关节数。这使得正运动学在实时控制中非常高效。

**递推计算:**

定义中间变换:

$$
\mathbf{T}_{0i} = \prod_{k=1}^{i} \mathbf{T}_{(k-1)k}(q_k)
$$

则 $\mathbf{T}_{0(i+1)} = \mathbf{T}_{0i} \cdot \mathbf{T}_{i(i+1)}(q_{i+1})$，具有递推结构。

**与逆运动学的对比:**

| 特性 | 正运动学 | 逆运动学 |
|------|----------|----------|
| 映射方向 | $\mathbf{q} \to \mathbf{x}$ | $\mathbf{x} \to \mathbf{q}$ |
| 解的存在性 | 总是存在 | 可能无解（超出工作空间） |
| 解的唯一性 | 唯一 | 通常多解 |
| 计算方法 | 解析（矩阵连乘） | 解析或数值迭代 |
| 计算复杂度 | $O(n)$ | 通常更高 |

---

## 总结与要点

本讲覆盖了机器人运动学的基础理论，核心要点如下:

1. **位置参数化**是运动学的起点，不同坐标系统各有优劣，映射矩阵 $\mathbf{E}$ 建立了参数微分与物理速度之间的桥梁。

2. **旋转参数化**是运动学中最具挑战性的部分。旋转矩阵（9 参数）、欧拉角（3 参数，有奇异性）、轴角（4 参数）、旋转向量（3 参数）和四元数（4 参数，无奇异性）各有适用场景。拓扑学告诉我们: **最小参数化必然存在奇异性**。

3. **角速度**通过映射矩阵 $\mathbf{E}$ 与旋转参数的时间导数相关联。四元数的时间导数公式 $\dot{\boldsymbol{\xi}} = \frac{1}{2}\boldsymbol{\omega}_\xi \otimes \boldsymbol{\xi}$ 在数值仿真中广泛使用。

4. **齐次变换矩阵**统一了旋转和平移，使得串联运动链的正运动学可以通过矩阵连乘高效计算: $\mathbf{T}_{IE} = \prod_i \mathbf{T}_{(i-1)i}(q_i)$。

5. **广义坐标**的选择（通常为关节变量）决定了运动学和动力学方程的形式。关节空间与任务空间之间的映射是机器人控制的核心问题。

---

> **下一讲预告:** Lecture 03 将讨论微分运动学 (Differential Kinematics)，包括 Jacobian 矩阵的推导、速度映射、奇异性分析以及逆运动学的数值方法。
