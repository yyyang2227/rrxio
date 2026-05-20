# DVC-RRxIO 技术方案设计文档

## Degeneration-aware and Vision-Coupled Covariance Recalibrated Radar-Visual-Inertial Odometry

---

## 1. 算法概述与创新点提炼

### 1.1 算法名称

本文方法命名为：

**DVC-RRxIO: Degeneration-aware and Vision-Coupled Covariance Recalibrated RRxIO**

中文名称建议为：

**退化感知与视觉质量耦合的雷达速度协方差重标定 RRxIO 方法**

该方法不是重新设计一套紧耦合雷达-视觉-惯性系统，而是在 RRxIO 的既有框架上进行低侵入式增强。具体而言，保留 RRxIO/ROVIO 的 IMU 传播、视觉更新与 REVE 雷达速度估计，仅在 REVE 输出的雷达速度协方差与 EKF 雷达速度更新之间加入一个退化感知协方差重标定层。

---

### 1.2 学术故事线与 Research Gap

现有 RRxIO 类方法的基本假设是：雷达前端 REVE 可以输出雷达自速度估计

$$
\hat{\mathbf v}_{R,k}\in \mathbb R^3
$$

以及相应速度协方差

$$
\mathbf\Sigma^{\text{REVE}}_{v,k}\in \mathbb R^{3\times 3}
$$

然后将其作为雷达速度观测接入 ROVIO/EKF 后端。该思路工程上简洁、实时性好、易于集成，但存在一个核心缺陷：

> REVE 给出的速度协方差并不一定在复杂退化环境下保持统计一致性。

具体表现为：

1. 雷达点云稀疏、视线方向分布退化、Doppler 残差异常时，$\mathbf\Sigma^{\text{REVE}}_{v,k}$ 可能低估弱可观方向的不确定性；
2. 视觉退化与雷达退化往往并非独立发生，实际环境中低纹理、雾、玻璃、多径、动态物体、强反射、遮挡等因素可能同时影响视觉和雷达；
3. 原始 RRxIO 主要将雷达速度协方差作为前端给定量使用，缺少基于滤波创新一致性的在线协方差反馈机制；
4. 若系统在退化场景中仍以固定置信度使用雷达速度更新，可能造成 EKF 过度自信、NIS 异常增大、局部轨迹突跳甚至滤波发散。

因此，DVC-RRxIO 的核心学术问题不是“如何加入更多传感器观测”，而是：

> 如何在不破坏 RRxIO 实时松耦合结构的前提下，使雷达速度测量协方差在退化环境中保持一致性与可信度。

---

### 1.3 三大核心创新点

#### Contribution 1：退化感知的 REVE 雷达速度协方差重标定

针对 REVE 雷达速度协方差在点云稀疏、方向几何退化和离群残差增大时可能失配的问题，本文构造雷达退化特征：

$$
\boldsymbol\phi_{R,k}=
[
N_{\text{in},k},
\eta_{\text{in},k},
\kappa_{U,k},
\lambda_{\min,k},
\sigma_{\text{res},k},
r_{\text{med},k}
]
$$

并通过非负退化评分：

$$
d_{R,k}=\operatorname{softplus}(\mathbf w_R^\top \mathbf g_R(\boldsymbol\phi_{R,k})+b_R)
$$

生成保守协方差膨胀因子：

$$
\alpha_{R,k}\ge 1
$$

使雷达几何越差、内点越少、Doppler 残差越大时，滤波器对雷达速度观测的置信度自动下降。

---

#### Contribution 2：可观测方向选择性的各向异性协方差塑形

本文不只使用标量因子放大雷达协方差，而是根据雷达视线方向矩阵：

$$
\mathbf U_k=
\begin{bmatrix}
\mathbf u_{1,k}^{R\top}\\
\cdots\\
\mathbf u_{N,k}^{R\top}
\end{bmatrix}
$$

构造几何可观测矩阵：

$$
\mathbf G_{R,k}=\mathbf U_k^\top \mathbf U_k+\epsilon_\lambda\mathbf I
$$

并进行特征分解：

$$
\mathbf G_{R,k}=\mathbf V_k\mathbf\Lambda_k\mathbf V_k^\top
$$

对于弱可观方向，即：

$$
\lambda_{\ell,k}<\tau_{\text{obs}}
$$

构造方向性膨胀：

$$
s_{\ell,k}=
1+c_{\text{obs}}
\max\left(0,\frac{\tau_{\text{obs}}-\lambda_{\ell,k}}{\tau_{\text{obs}}+\epsilon}\right)
$$

得到：

$$
\mathbf S_k=\mathbf V_k\operatorname{diag}(s_{1,k},s_{2,k},s_{3,k})\mathbf V_k^\top
$$

最终对 REVE 原始协方差进行各向异性塑形：

$$
\mathbf S_k\mathbf\Sigma^{\text{REVE}}_{v,k}\mathbf S_k^\top
$$

该设计表达的是：雷达速度估计在哪些方向几何支撑不足，就在哪些方向保守增大不确定性，而不是整体性削弱雷达观测。

---

#### Contribution 3：视觉质量耦合的跨模态雷达更新机制与 NIS 一致性反馈

针对视觉退化和雷达退化常由同一环境因素共同触发的问题，本文引入视觉退化评分：

$$
d_{V,k}=\operatorname{softplus}(\mathbf w_V^\top \mathbf g_V(\boldsymbol\phi_{V,k})+b_V)
$$

并构造跨模态协方差耦合项：

$$
\zeta_{RV,k}=
1+\operatorname{softplus}(\theta_0+\theta_1d_{R,k}+\theta_2d_{V,k}+\theta_3d_{R,k}d_{V,k})
$$

其中交叉项 $d_{R,k}d_{V,k}$ 是本方法区别于简单自适应权重的关键。它允许系统区分“视觉差但雷达可靠”和“视觉与雷达同时退化”两类情形。

同时，利用雷达速度观测的 NIS：

$$
\gamma_{R,k}=\mathbf r_{R,k}^{\top}\mathbf S_{R,k}^{-1}\mathbf r_{R,k}
$$

构造在线一致性反馈因子 $\alpha^{\text{NIS}}_k$，使系统在创新异常时自动增大雷达观测协方差，在创新恢复正常时缓慢回落，从而降低 EKF 过度自信风险。

---

## 2. 数学建模与理论推导

### 2.1 坐标系定义

定义如下坐标系：

- $\{W\}$：世界坐标系；
- $\{B\}$：IMU/body 坐标系；
- $\{C\}$：相机坐标系；
- $\{R\}$：毫米波雷达坐标系。

外参定义为：

$$
\mathbf T_{BR}=(\mathbf R_{BR},\mathbf p_{BR})
$$

表示雷达相对于 body 的位姿。本文雷达速度观测统一表达在雷达坐标系 $\{R\}$ 中，因此 REVE 输出 $\hat{\mathbf v}_{R,k}$、观测函数 $\mathbf h_R(\mathbf x_k)$ 以及协方差 $\mathbf\Sigma^{\text{REVE}}_{v,k}$ 必须位于同一坐标系。

若后端使用 body 坐标系雷达速度，则需要进行协方差旋转：

$$
\mathbf\Sigma^B_{v,k}=\mathbf R_{BR}\mathbf\Sigma^R_{v,k}\mathbf R_{BR}^{\top}
$$

---

### 2.2 状态向量与误差状态定义

DVC-RRxIO 继承 RRxIO/ROVIO 的滤波结构。基础 IMU 状态定义为：

$$
\mathbf x_k=
\left[
\mathbf p_{WB,k}^{W},
\mathbf v_{WB,k}^{W},
\mathbf q_{WB,k},
\mathbf b_{g,k},
\mathbf b_{a,k},
\boldsymbol\lambda_k
\right]
$$

其中：

- $\mathbf p_{WB,k}^{W}\in\mathbb R^3$：body 在世界坐标系下的位置；
- $\mathbf v_{WB,k}^{W}\in\mathbb R^3$：body 在世界坐标系下的速度；
- $\mathbf q_{WB,k}\in SO(3)$：body 到 world 的姿态四元数；
- $\mathbf b_{g,k}\in\mathbb R^3$：陀螺仪零偏；
- $\mathbf b_{a,k}\in\mathbb R^3$：加速度计零偏；
- $\boldsymbol\lambda_k$：ROVIO 中维护的视觉特征参数，例如逆深度、patch 仿射光照参数或其他视觉特征状态。

误差状态定义为：

$$
\delta\mathbf x_k=
\left[
\delta\mathbf p_k^\top,
\delta\mathbf v_k^\top,
\delta\boldsymbol\theta_k^\top,
\delta\mathbf b_{g,k}^\top,
\delta\mathbf b_{a,k}^\top,
\delta\boldsymbol\lambda_k^\top
\right]^\top
$$

姿态误差采用李代数小扰动：

$$
\mathbf R_{WB}=\hat{\mathbf R}_{WB}\operatorname{Exp}(\delta\boldsymbol\theta)
$$

状态流形加法记为：

$$
\mathbf x\boxplus\delta\mathbf x
$$

---

### 2.3 IMU 系统演化方程

IMU 连续时间模型为：

$$
\dot{\mathbf p}_{WB}^{W}=\mathbf v_{WB}^{W}
$$

$$
\dot{\mathbf v}_{WB}^{W}=\mathbf R_{WB}(\mathbf a_m-\mathbf b_a-\mathbf n_a)+\mathbf g^W
$$

$$
\dot{\mathbf R}_{WB}=\mathbf R_{WB}[\boldsymbol\omega_m-\mathbf b_g-\mathbf n_g]_\times
$$

$$
\dot{\mathbf b}_g=\mathbf n_{wg},\qquad
\dot{\mathbf b}_a=\mathbf n_{wa}
$$

离散传播为：

$$
\hat{\mathbf x}_{k|k-1}=f(\hat{\mathbf x}_{k-1|k-1},\mathcal U_{k-1:k})
$$

$$
\mathbf P_{k|k-1}=\mathbf F_k\mathbf P_{k-1|k-1}\mathbf F_k^\top+\mathbf G_k\mathbf Q_k\mathbf G_k^\top
$$

DVC-RRxIO 不修改 IMU 传播部分。

---

### 2.4 视觉残差项

若采用 ROVIO 风格的直接法 patch 残差，可抽象写为：

$$
\mathbf r_{V,k}=\mathbf z_{V,k}-\mathbf h_V(\mathbf x_k)
$$

线性化：

$$
\mathbf r_{V,k}\approx\mathbf r_{V,k}^{0}-\mathbf H_{V,k}\delta\mathbf x_k
$$

视觉更新对应的加权最小二乘项为：

$$
J_V(\delta\mathbf x_k)=
\frac{1}{2}
\left\|
\mathbf r_{V,k}^{0}-\mathbf H_{V,k}\delta\mathbf x_k
\right\|^2_{\mathbf R_{V,k}^{-1}}
$$

DVC-RRxIO 不改变视觉残差本身，而是从视觉前端额外提取视觉质量特征：

$$
\boldsymbol\phi_{V,k}=
[
N_{f,k},
\bar g_k,
H_{f,k},
\bar\ell_k,
\sigma_{\text{photo},k},
p_k
]
$$

这些特征不直接作为硬约束进入状态估计，而是参与雷达协方差条件建模。

---

### 2.5 雷达速度残差项

REVE 前端输出雷达自速度观测：

$$
\hat{\mathbf v}_{R,k}\in\mathbb R^3
$$

原始协方差：

$$
\mathbf\Sigma^{\text{REVE}}_{v,k}\in\mathbb R^{3\times3}
$$

雷达坐标系下的预测速度为：

$$
\mathbf h_R(\mathbf x_k)=
\mathbf R_{RB}\mathbf R_{BW,k}\mathbf v_{WB,k}^{W}
+
\mathbf R_{RB}(\boldsymbol\omega_{B,k}\times\mathbf p_{BR})
$$

雷达速度残差为：

$$
\mathbf r_{R,k}=\hat{\mathbf v}_{R,k}-\mathbf h_R(\hat{\mathbf x}_{k}^{-})
$$

线性化：

$$
\mathbf r_{R,k}\approx\mathbf r_{R,k}^{0}-\mathbf H_{R,k}\delta\mathbf x_k
$$

雷达观测加权最小二乘项为：

$$
J_R(\delta\mathbf x_k)=
\frac{1}{2}
u_{R,k}
\left\|
\mathbf r_{R,k}^{0}-\mathbf H_{R,k}\delta\mathbf x_k
\right\|^2_{\tilde{\mathbf R}_{R,k}^{-1}}
$$

其中 $\nu_{R,k}\in\{0,1\}$ 表示是否执行雷达更新。

---

### 2.6 EKF 更新的等价代价函数

在每个雷达更新时刻，滤波更新等价于求解局部高斯最小二乘问题：

$$
\delta\mathbf x_k^\star=
\arg\min_{\delta\mathbf x_k}
\frac{1}{2}
\left\|\delta\mathbf x_k\right\|^2_{(\mathbf P_k^{-})^{-1}}
+
\frac{1}{2}
u_{R,k}
\left\|
\mathbf r_{R,k}^{0}-\mathbf H_{R,k}\delta\mathbf x_k
\right\|^2_{\tilde{\mathbf R}_{R,k}^{-1}}
$$

正规方程为：

$$
\left(
(\mathbf P_k^{-})^{-1}+
u_{R,k}\mathbf H_{R,k}^{\top}\tilde{\mathbf R}_{R,k}^{-1}\mathbf H_{R,k}
\right)\delta\mathbf x_k
=
\nu_{R,k}\mathbf H_{R,k}^{\top}\tilde{\mathbf R}_{R,k}^{-1}\mathbf r_{R,k}^{0}
$$

DVC-RRxIO 实质上是在动态调节雷达速度因子的 information matrix：

$$
\tilde{\mathbf\Omega}_{R,k}=\tilde{\mathbf R}_{R,k}^{-1}
$$

当系统退化时：

$$
\tilde{\mathbf R}_{R,k}\uparrow
\quad\Rightarrow\quad
\tilde{\mathbf\Omega}_{R,k}\downarrow
$$

从而降低不可信雷达观测对状态估计的拉动强度。

---

### 2.7 雷达退化评分

雷达质量特征定义为：

$$
\boldsymbol\phi_{R,k}=
[
N_{\text{in},k},
\eta_{\text{in},k},
\kappa_{U,k},
\lambda_{\min,k},
\sigma_{\text{res},k},
r_{\text{med},k}
]
$$

其中：

$$
\eta_{\text{in},k}=\frac{N_{\text{in},k}}{N_{\text{all},k}+\epsilon}
$$

$$
\mathbf G_{R,k}=\mathbf U_k^\top\mathbf U_k+\epsilon_\lambda\mathbf I
$$

$$
\kappa_{U,k}=\frac{\lambda_{\max}(\mathbf G_{R,k})}{\lambda_{\min}(\mathbf G_{R,k})}
$$

归一化退化特征：

$$
\mathbf g_R(\boldsymbol\phi_{R,k})=
\left[
\frac{1}{N_{\text{in},k}+\epsilon},
\frac{1}{\eta_{\text{in},k}+\epsilon},
\log(\kappa_{U,k}+\epsilon),
\frac{1}{\lambda_{\min,k}+\epsilon},
\sigma_{\text{res},k},
r_{\text{med},k}
\right]^\top
$$

雷达退化评分：

$$
d_{R,k}=\operatorname{softplus}(\mathbf w_R^\top\mathbf g_R(\boldsymbol\phi_{R,k})+b_R)
$$

---

### 2.8 视觉退化评分

视觉质量特征定义为：

$$
\boldsymbol\phi_{V,k}=
[
N_{f,k},
\bar g_k,
H_{f,k},
\bar\ell_k,
\sigma_{\text{photo},k},
p_k
]
$$

归一化视觉退化特征为：

$$
\mathbf g_V(\boldsymbol\phi_{V,k})=
\left[
\frac{1}{N_{f,k}+\epsilon},
\frac{1}{\bar g_k+\epsilon},
\frac{1}{H_{f,k}+\epsilon},
\frac{1}{\bar\ell_k+\epsilon},
\sigma_{\text{photo},k},
\frac{1}{p_k+\epsilon}
\right]^\top
$$

视觉退化评分：

$$
d_{V,k}=\operatorname{softplus}(\mathbf w_V^\top\mathbf g_V(\boldsymbol\phi_{V,k})+b_V)
$$

---

### 2.9 退化感知协方差重标定

雷达自身退化膨胀因子：

$$
\alpha_{R,k}=1+\operatorname{softplus}(a_R d_{R,k}+b_{R\alpha})
$$

跨模态视觉-雷达耦合因子：

$$
\zeta_{RV,k}=1+\operatorname{softplus}(\theta_0+\theta_1d_{R,k}+\theta_2d_{V,k}+\theta_3d_{R,k}d_{V,k})
$$

方向性协方差塑形矩阵：

$$
\mathbf S_k=\mathbf V_k\operatorname{diag}(s_{1,k},s_{2,k},s_{3,k})\mathbf V_k^\top
$$

最终重标定协方差：

$$
\boxed{
\tilde{\mathbf R}_{R,k}=
\alpha_{R,k}\alpha^{\text{NIS}}_{k-1}\zeta_{RV,k}
\mathbf S_k\mathbf\Sigma^{\text{REVE}}_{v,k}\mathbf S_k^\top
+
\sigma^2_{v,\min}\mathbf I
}
$$

由于 $\alpha_{R,k}\ge1$、$\alpha^{\text{NIS}}_{k-1}\ge1$、$\zeta_{RV,k}\ge1$ 且 $s_{\ell,k}\ge1$，该协方差重标定为保守型修正。若 $\mathbf\Sigma^{\text{REVE}}_{v,k}$ 半正定，则加入 $\sigma^2_{v,\min}\mathbf I$ 后可保证 $\tilde{\mathbf R}_{R,k}$ 严格正定。

---

### 2.10 NIS 一致性反馈律

雷达创新协方差：

$$
\mathbf S^{\text{innov}}_{R,k}=\mathbf H_{R,k}\mathbf P_k^{-}\mathbf H_{R,k}^{\top}+\tilde{\mathbf R}_{R,k}
$$

NIS 定义为：

$$
\gamma_{R,k}=\mathbf r_{R,k}^{\top}(\mathbf S^{\text{innov}}_{R,k})^{-1}\mathbf r_{R,k}
$$

在滤波一致、高斯噪声和线性化误差较小的假设下，三维速度观测满足近似关系：

$$
\gamma_{R,k}\sim\chi^2(3),\qquad \mathbb E[\gamma_{R,k}]=3
$$

反馈律为：

$$
\alpha^{\text{NIS}}_{k}=
\operatorname{clip}
\left[
\alpha^{\text{NIS}}_{k-1}
\exp\left(\eta\left(\frac{\gamma_{R,k}}{3}-1\right)\right),
1,
\alpha_{\max}
\right]
$$

若当前帧跳过雷达更新，则采用遗忘机制：

$$
\alpha^{\text{NIS}}_{k}=1+\rho(\alpha^{\text{NIS}}_{k-1}-1),\qquad 0<\rho<1
$$

---

### 2.11 雷达更新门控

定义可靠度：

$$
q_{R,k}=\exp(-d_{R,k}),\qquad q_{V,k}=\exp(-d_{V,k})
$$

由于 $d_{R,k}\ge0$ 且 $d_{V,k}\ge0$，因此：

$$
q_{R,k},q_{V,k}\in(0,1]
$$

更新策略为：

$$
\nu_{R,k}=
\begin{cases}
1,&q_{R,k}>\tau_R^{high}\\
1,&q_{R,k}>\tau_R^{low}\ \text{and}\ q_{V,k}<\tau_V^{low}\\
0,&\text{otherwise}
\end{cases}
$$

其中 $\tau_R^{low}<\tau_R^{high}$。

---

## 3. 核心算法流水线与详尽伪代码

### 3.1 模块级流水线

```text
IMU measurements
      │
      ▼
IMU propagation
      │
      ▼
Camera image ──► ROVIO visual tracking/update ──► visual quality φ_V,k ──► d_V,k
      │
      ▼
Radar point cloud + Doppler ──► REVE ──► v_R,k, Σ_REVE,k, φ_R,k ──► d_R,k
      │
      ▼
Observability analysis UᵀU ──► S_k
      │
      ▼
Covariance recalibration:
R_R,k = α_R,k α_NIS,k-1 ζ_RV,k S_k Σ_REVE,k S_kᵀ + σ²_min I
      │
      ▼
Radar update scheduling ν_R,k
      │
      ▼
EKF radar velocity update
      │
      ▼
NIS consistency feedback α_NIS,k
```

---

### 3.2 规范伪代码

```text
Algorithm 1: DVC-RRxIO
Degeneration-aware and Vision-Coupled Covariance Recalibrated RRxIO

Inputs:
    I_k                       : current camera image
    R_k                       : current radar point cloud with Doppler
    U_{k-1:k}                 : IMU measurements between t_{k-1} and t_k
    x_{k-1|k-1}               : previous posterior state
    P_{k-1|k-1}               : previous posterior covariance
    α_NIS,k-1                 : previous NIS feedback inflation factor
    T_BR, T_BC                : radar/body and camera/body extrinsic parameters
    Θ                         : algorithm hyperparameters

Outputs:
    x_{k|k}                   : current posterior state
    P_{k|k}                   : current posterior covariance
    α_NIS,k                   : updated NIS feedback factor
    diagnostic_k              : quality scores, NIS, gating status, runtime statistics

Procedure:

1:  x_imu,k, P_imu,k ← PropagateIMU(x_{k-1|k-1}, P_{k-1|k-1}, U_{k-1:k})

2:  visual_result ← TrackVisualFeaturesOrPatches(I_k, x_imu,k)
3:  φ_V,k ← ExtractVisualQuality(visual_result)
4:  d_V,k ← softplus(w_Vᵀ g_V(φ_V,k) + b_V)

5:  if visual_result.is_valid then
6:      x_V,k, P_V,k ← VisualEKFUpdate(x_imu,k, P_imu,k, visual_result)
7:  else
8:      x_V,k ← x_imu,k
9:      P_V,k ← P_imu,k
10: end if

11: x_k^- ← x_V,k
12: P_k^- ← P_V,k

13: reve_result ← REVE(R_k)
14: v_R,k ← reve_result.velocity
15: Σ_REVE,k ← reve_result.velocity_covariance
16: inliers_k ← reve_result.inliers
17: residuals_k ← reve_result.doppler_residuals

18: U_k ← ConstructRadarLOSMatrix(inliers_k)
19: G_R,k ← U_kᵀ U_k + ε_λ I
20: V_k, Λ_k ← EigenDecomposition(G_R,k)
21: φ_R,k ← ExtractRadarQuality(inliers_k, residuals_k, G_R,k)
22: d_R,k ← softplus(w_Rᵀ g_R(φ_R,k) + b_R)

23: for ℓ = 1 to 3 do
24:     λ_ℓ,k ← Λ_k(ℓ,ℓ)
25:     s_ℓ,k ← 1 + c_obs · max(0, (τ_obs - λ_ℓ,k)/(τ_obs + ε))
26:     s_ℓ,k ← clip(s_ℓ,k, 1, s_max)
27: end for
28: S_k ← V_k diag(s_1,k, s_2,k, s_3,k) V_kᵀ

29: α_R,k ← 1 + softplus(a_R d_R,k + b_Rα)
30: ζ_RV,k ← 1 + softplus(θ_0 + θ_1 d_R,k + θ_2 d_V,k + θ_3 d_R,k d_V,k)

31: R_R,k ← α_R,k · α_NIS,k-1 · ζ_RV,k · S_k Σ_REVE,k S_kᵀ + σ²_v,min I
32: R_R,k ← 0.5 · (R_R,k + R_R,kᵀ)
33: R_R,k ← ProjectToSPDIfNeeded(R_R,k, σ²_v,min)

34: q_R,k ← exp(-d_R,k)
35: q_V,k ← exp(-d_V,k)

36: if q_R,k > τ_R^high then
37:     use_radar_update ← true
38: else if q_R,k > τ_R^low and q_V,k < τ_V^low then
39:     use_radar_update ← true
40: else
41:     use_radar_update ← false
42: end if

43: h_R,k ← PredictRadarVelocity(x_k^-, T_BR)
44: r_R,k ← v_R,k - h_R,k
45: H_R,k ← LinearizeRadarVelocityModel(x_k^-, T_BR)
46: S_innov,k ← H_R,k P_k^- H_R,kᵀ + R_R,k
47: γ_cand,k ← r_R,kᵀ S_innov,k^{-1} r_R,k

48: if use_radar_update == true then
49:     K_R,k ← P_k^- H_R,kᵀ S_innov,k^{-1}
50:     x_{k|k} ← x_k^- ⊞ K_R,k r_R,k
51:     P_{k|k} ← (I - K_R,k H_R,k) P_k^- (I - K_R,k H_R,k)ᵀ
52:                + K_R,k R_R,k K_R,kᵀ
53:     P_{k|k} ← 0.5 · (P_{k|k} + P_{k|k}ᵀ)
54:     γ_R,k ← γ_cand,k
55: else
56:     x_{k|k} ← x_k^-
57:     P_{k|k} ← P_k^-
58:     γ_R,k ← null
59: end if

60: if use_radar_update == true then
61:     α_NIS,k ← clip(α_NIS,k-1 · exp(η · (γ_R,k / 3 - 1)), 1, α_max)
62: else
63:     α_NIS,k ← 1 + ρ · (α_NIS,k-1 - 1)
64: end if

65: diagnostic_k ← {
66:     d_R,k, d_V,k, q_R,k, q_V,k,
67:     α_R,k, ζ_RV,k, α_NIS,k,
68:     s_1,k, s_2,k, s_3,k,
69:     γ_cand,k, γ_R,k,
70:     use_radar_update
71: }

72: return x_{k|k}, P_{k|k}, α_NIS,k, diagnostic_k
```

---

### 3.3 关键步骤注释

第 1 行：IMU 传播完全沿用 RRxIO/ROVIO，不改变系统动力学和状态定义，降低工程风险。

第 3–4 行：视觉质量不作为额外观测残差，而是作为环境退化代理变量参与雷达协方差条件建模。

第 18–20 行：通过雷达视线方向矩阵构造 $\mathbf U^\top\mathbf U$，评估 Doppler 自速度估计的方向几何可观测性。

第 23–28 行：方向性膨胀只在弱可观方向触发，避免与 $\Sigma^{\text{REVE}}$ 中已有不确定性重复作用。

第 30 行：$\zeta_{RV,k}$ 中的 $d_Rd_V$ 是跨模态耦合的核心，防止系统在视觉退化时无条件增强雷达观测。

第 31 行：最终协方差始终采用保守膨胀，不人为压缩 REVE 原始协方差。

第 36–42 行：门控逻辑区分“雷达高度可靠”和“视觉退化但雷达尚可”。

第 46–47 行：即使雷达更新被拒绝，也可以计算候选 NIS 用于日志分析，但不用于更新 $\alpha^{\text{NIS}}$。

第 60–64 行：NIS 反馈只在实际使用雷达更新时执行；跳过更新时采用遗忘机制，使历史膨胀因子缓慢回到 1。

---

### 3.4 时间复杂度分析

设每帧雷达点数为 $N_R$，视觉特征数为 $N_V$，状态维度为 $n_x$。

新增雷达质量计算复杂度：

$$
\mathcal O(N_R)
$$

构造 $\mathbf U^\top\mathbf U$ 的复杂度为：

$$
\mathcal O(3^2N_R)=\mathcal O(N_R)
$$

$3\times3$ 特征分解复杂度为常数：

$$
\mathcal O(1)
$$

视觉质量统计复杂度：

$$
\mathcal O(N_V)
$$

雷达 EKF 更新中，雷达观测维度为 3，主要计算复杂度约为：

$$
\mathcal O(3n_x^2)
$$

整体新增复杂度主要是：

$$
\mathcal O(N_R+N_V)
$$

以及常数级 $3\times3$ 矩阵运算。相较于视觉 patch 更新、REVE RANSAC 和 EKF 主体更新，新增模块开销较低。

---

### 3.5 空间复杂度分析

新增存储包括：

$$
\mathbf G_R,\mathbf S_k,\mathbf R_R\in\mathbb R^{3\times3}
$$

以及若干标量质量指标。工程实现中可不显式保存 $\mathbf U_k$，而是边遍历雷达点边累加：

$$
\mathbf G_R=\sum_i \mathbf u_i\mathbf u_i^\top
$$

此时新增空间复杂度为：

$$
\mathcal O(1)
$$

---

## 4. 工程落地与系统实现指南

### 4.1 推荐 ROS 节点架构

```text
/dvc_rrxio_node
    ├── imu_callback()
    ├── image_callback()
    ├── radar_callback()
    ├── sync_and_process()
    ├── rovio_frontend_
    ├── reve_frontend_
    ├── quality_evaluator_
    ├── covariance_recalibrator_
    ├── ekf_backend_
    └── diagnostics_logger_
```

推荐 topic：

```text
/imu/data
/camera/image_raw
/radar/pointcloud
/ground_truth/pose
/dvc_rrxio/odometry
/dvc_rrxio/diagnostics
/dvc_rrxio/nis
/dvc_rrxio/quality
```

推荐发布诊断量：

```text
d_R
d_V
q_R
q_V
alpha_R
zeta_RV
alpha_NIS
s1, s2, s3
N_in
eta_in
lambda_min
kappa_U
sigma_res
gamma_cand
gamma_used
use_radar_update
trace_R_recalibrated
runtime_recalibration
```

---

### 4.2 C++ 核心类设计

```cpp
struct RadarQuality {
  int num_points = 0;
  int num_inliers = 0;
  double inlier_ratio = 0.0;
  double lambda_min = 0.0;
  double lambda_max = 0.0;
  double condition_number = 0.0;
  double residual_std = 0.0;
  double residual_median_abs = 0.0;
  double d_r = 0.0;
  double q_r = 0.0;
  Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d V = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lambda = Eigen::Vector3d::Ones();
};

struct VisualQuality {
  int num_features = 0;
  double mean_gradient = 0.0;
  double feature_entropy = 0.0;
  double mean_track_age = 0.0;
  double photometric_residual_std = 0.0;
  double parallax = 0.0;
  double d_v = 0.0;
  double q_v = 0.0;
};

struct RecalibrationOutput {
  Eigen::Matrix3d R_radar;
  Eigen::Matrix3d S_dir;
  double alpha_r = 1.0;
  double zeta_rv = 1.0;
  double alpha_nis = 1.0;
  Eigen::Vector3d s = Eigen::Vector3d::Ones();
};

class RadarQualityEvaluator {
public:
  RadarQuality evaluate(const ReveResult& reve_result,
                        const RadarPointCloud& cloud,
                        const Params& params);
};

class VisualQualityEvaluator {
public:
  VisualQuality evaluate(const VisualTrackingResult& visual_result,
                         const Params& params);
};

class CovarianceRecalibrator {
public:
  RecalibrationOutput recalibrate(
      const Eigen::Matrix3d& Sigma_reve,
      const RadarQuality& rq,
      const VisualQuality& vq,
      double alpha_nis_prev,
      const Params& params);
};

class RadarUpdateScheduler {
public:
  bool shouldUpdate(const RadarQuality& rq,
                    const VisualQuality& vq,
                    const Params& params);
};

class NisFeedback {
public:
  double update(bool used_radar,
                double gamma,
                double alpha_nis_prev,
                const Params& params);
};
```

---

### 4.3 Eigen 实现要点

$3\times3$ 矩阵建议使用：

```cpp
Eigen::Matrix3d
```

向量使用：

```cpp
Eigen::Vector3d
```

特征分解使用：

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(G);
```

示例：

```cpp
Eigen::Matrix3d G = U.transpose() * U;
G += params.eps_lambda * Eigen::Matrix3d::Identity();

Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(G);

if (es.info() != Eigen::Success) {
  G = params.eps_lambda * Eigen::Matrix3d::Identity();
}

Eigen::Vector3d lambda = es.eigenvalues();
Eigen::Matrix3d V = es.eigenvectors();
```

协方差对称化：

```cpp
R = 0.5 * (R + R.transpose());
```

SPD 投影：

```cpp
Eigen::Matrix3d projectToSPD(const Eigen::Matrix3d& A, double min_eig) {
  Eigen::Matrix3d S = 0.5 * (A + A.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(S);
  Eigen::Vector3d evals = es.eigenvalues();
  Eigen::Matrix3d evecs = es.eigenvectors();

  for (int i = 0; i < 3; ++i) {
    evals(i) = std::max(evals(i), min_eig);
  }

  return evecs * evals.asDiagonal() * evecs.transpose();
}
```

---

### 4.4 Ceres Solver / 因子图版本绑定建议

若后续从 EKF 扩展为滑窗优化版本，可将 DVC-RRxIO 的雷达速度观测写成 Ceres residual block。

雷达速度残差：

$$
\mathbf r_R=\hat{\mathbf v}_R-\mathbf h_R(\mathbf x)
$$

白化残差：

$$
\mathbf e_R=\mathbf L_R^{-1}\mathbf r_R
$$

其中：

$$
\tilde{\mathbf R}_R=\mathbf L_R\mathbf L_R^\top
$$

Ceres 中使用 Cholesky 分解：

```cpp
Eigen::LLT<Eigen::Matrix3d> llt(R_radar);
Eigen::Matrix3d L = llt.matrixL();
Eigen::Matrix3d sqrt_info = L.inverse();
```

残差输出：

```cpp
residual = sqrt_info * (v_r_meas - h_r(state));
```

第一阶段不建议直接切换到 Ceres/FGO。更稳妥的路线是先在 EKF/RRxIO 中实现协方差重标定，完成在线实时验证和消融实验，再将雷达速度因子移植到滑窗优化版本中。

---

### 4.5 参数配置建议

```yaml
dvc_rrxio:
  eps: 1.0e-6
  eps_lambda: 1.0e-4

  sigma_v_min: 0.03

  obs:
    tau_obs: 5.0
    c_obs: 1.0
    s_max: 5.0

  gating:
    tau_R_high: 0.65
    tau_R_low: 0.35
    tau_V_low: 0.45

  nis:
    eta: 0.03
    rho: 0.98
    alpha_max: 10.0

  radar_score:
    w_inv_num_inliers: 0.8
    w_inv_inlier_ratio: 1.0
    w_log_condition: 0.7
    w_inv_lambda_min: 0.8
    w_residual_std: 1.0
    w_residual_median: 0.5
    bias: -1.0

  visual_score:
    w_inv_num_features: 0.8
    w_inv_gradient: 0.7
    w_inv_entropy: 0.5
    w_inv_track_age: 0.3
    w_photo_std: 1.0
    w_inv_parallax: 0.5
    bias: -1.0

  coupling:
    theta0: -1.0
    theta1: 0.5
    theta2: 0.3
    theta3: 0.8
```

推荐调参顺序：

1. 固定 $\zeta_{RV}=1$，只调 $\alpha_R$ 和 $\mathbf S_k$；
2. 固定 $\alpha^{\text{NIS}}=1$，验证退化感知协方差是否改善 NIS；
3. 加入 $\alpha^{\text{NIS}}$，调 $\eta,\rho,\alpha_{\max}$；
4. 最后加入视觉耦合项 $\zeta_{RV}$。

---

### 4.6 数值稳定性要求

必须加入以下保护：

1. 对所有除法加入 $\epsilon$；
2. 对 $\mathbf G_R$ 加入 $\epsilon_\lambda I$；
3. 对 $s_\ell$ 设置上界 $s_{\max}$；
4. 对 $\alpha^{\text{NIS}}$ 设置上界 $\alpha_{\max}$；
5. 对最终 $\tilde{\mathbf R}_{R,k}$ 做对称化；
6. 对最终 $\tilde{\mathbf R}_{R,k}$ 做 SPD 投影；
7. 当 REVE 输出协方差非法时，回退到默认协方差。

非法条件包括：

```cpp
!R.allFinite()
min_eigenvalue <= 0
trace(R) too large
trace(R) too small
```

---

### 4.7 多线程与时间同步建议

推荐线程结构：

```text
Thread 1: IMU buffer
Thread 2: image frontend
Thread 3: radar frontend / REVE
Thread 4: estimator backend
Thread 5: logger
```

同步策略：

1. IMU 高频进入 ring buffer；
2. 图像作为主时钟；
3. 对每帧图像，取最近雷达帧或插值后的雷达速度；
4. IMU 传播到图像时间；
5. 若雷达时间与图像时间差超过阈值，则跳过雷达更新。

时间差阈值建议：

$$
|\Delta t_{radar-camera}|<30\text{ ms}
$$

具体阈值需要根据数据集频率调整。

---

### 4.8 日志与可视化

建议保存以下 CSV 日志：

```text
timestamp
ATE_local
velocity_error_if_gt_available
d_R
d_V
q_R
q_V
alpha_R
zeta_RV
alpha_NIS
s1
s2
s3
N_in
eta_in
lambda_min
condition_number
sigma_res
gamma_cand
gamma_used
use_radar_update
trace_Sigma_REVE
trace_R_recalibrated
runtime_reve
runtime_recalibration
runtime_backend
```

推荐绘图：

1. $d_R,d_V$ 随时间变化；
2. $q_R,q_V$ 随时间变化；
3. $\operatorname{tr}(\Sigma^{REVE})$ 与 $\operatorname{tr}(\tilde R)$ 对比；
4. NIS 与 $\chi^2_{3,0.95}$ 门限对比；
5. $s_1,s_2,s_3$ 与 $\lambda_1,\lambda_2,\lambda_3$ 对比；
6. 轨迹误差与雷达更新门控状态对比；
7. 视觉退化区间内 radar update ratio。

---

## 5. 实验验证、消融设计与预期效果

### 5.1 Baseline 设置

| 方法 | 目的 |
|---|---|
| VIO / ROVIO only | 证明视觉退化下单视觉惯性系统的失败模式 |
| REVE / RIO only | 评估雷达速度估计本身边界 |
| 原始 RRxIO | 核心 baseline |
| RRxIO + 固定倍率协方差膨胀 | 排除“只是调大 R 就有效”的质疑 |
| RRxIO + 仅雷达退化评分 $\alpha_R$ | 验证雷达退化建模 |
| RRxIO + $\alpha_R+\mathbf S_k$ | 验证方向性可观测膨胀 |
| RRxIO + $\alpha_R+\mathbf S_k+\alpha^{NIS}$ | 验证一致性反馈 |
| DVC-RRxIO full | 验证完整视觉耦合方案 |

扩展消融：

| 方法 | 目的 |
|---|---|
| RRxIO + only visual gate | 排除“只靠视觉门控”的质疑 |
| RRxIO + only NIS feedback | 排除“只靠 NIS 自适应”的质疑 |
| DVC-RRxIO without $d_Rd_V$ | 验证跨模态交叉项必要性 |

---

### 5.2 数据场景划分

| 场景 | 验证目标 |
|---|---|
| 正常光照 / 正常纹理 | 不损伤原始 RRxIO 性能 |
| 低光 / 黑暗 | 视觉退化时雷达速度补偿是否有效 |
| 雾 / 烟雾 | 跨模态退化建模是否有效 |
| 强光 / 直射光 | 视觉质量评分是否能反映退化 |
| 开阔雷达稀疏 | 雷达几何退化时是否避免过度自信 |
| 动态物体较多 | Doppler 残差异常时是否保守 |
| 高速运动 / 快速转弯 | 速度残差与 NIS 是否稳定 |

---

### 5.3 轨迹精度指标

ATE：

$$
\operatorname{ATE}=\sqrt{\frac{1}{N}\sum_{k=1}^{N}\|\mathbf p_k-\mathbf p_k^{gt}\|^2}
$$

RPE：

$$
\operatorname{RPE}(\Delta)=
\sqrt{
\frac{1}{N-\Delta}
\sum_{k=1}^{N-\Delta}
\|
(\mathbf T_k^{-1}\mathbf T_{k+\Delta})^{-1}
((\mathbf T_k^{gt})^{-1}\mathbf T_{k+\Delta}^{gt})
\|^2
}
$$

还应报告平移相对误差 RTE 和旋转相对误差 RRE。

---

### 5.4 速度精度指标

速度 RMSE：

$$
\operatorname{RMSE}_{v}=\sqrt{\frac{1}{N}\sum_{k=1}^{N}\|\hat{\mathbf v}_k-\mathbf v_k^{gt}\|^2}
$$

若 ground truth 速度不可直接获得，可通过 ground truth pose 差分并低通滤波获得近似速度，但论文中必须说明差分方法和滤波窗口。

---

### 5.5 一致性指标

雷达 NIS：

$$
\operatorname{NIS}_{R,k}=\mathbf r_{R,k}^{\top}(\mathbf S^{\text{innov}}_{R,k})^{-1}\mathbf r_{R,k}
$$

对三维速度观测，统计 95% 卡方门限超限率：

$$
\rho_{\text{NIS}}=
\frac{1}{N}
\sum_{k=1}^{N}
\mathbb I[
\operatorname{NIS}_{R,k}>\chi^2_{3,0.95}
]
$$

理想一致情况下：

$$
\rho_{\text{NIS}}\approx5\%
$$

如果原始 RRxIO 过度自信，通常会出现 $\rho_{\text{NIS}}\gg5\%$。DVC-RRxIO 的目标是让该比例更接近理论比例，同时不显著损失轨迹精度。

---

### 5.6 协方差可信度指标

定义雷达速度观测误差：

$$
e_{v,k}=\|\hat{\mathbf v}_{R,k}-\mathbf v_{R,k}^{gt}\|
$$

协方差强度：

$$
c_{R,k}=\operatorname{tr}(\tilde{\mathbf R}_{R,k})
$$

报告相关系数：

$$
\rho_{ce}=\operatorname{corr}(c_{R,k},e_{v,k}^{2})
$$

若 DVC-RRxIO 有效，应看到：

$$
\rho_{ce}^{\text{DVC}}>\rho_{ce}^{\text{RRxIO}}
$$

即重标定后的协方差更能反映真实误差。

---

### 5.7 实时性指标

记录：

$$
t_{\text{REVE}},\quad t_{\text{visual}},\quad t_{\text{recalib}},\quad t_{\text{EKF}},\quad t_{\text{total}}
$$

新增开销比例：

$$
\Delta_{\text{runtime}}=
\frac{t_{\text{DVC-RRxIO}}-t_{\text{RRxIO}}}{t_{\text{RRxIO}}}\times100\%
$$

工程目标：

$$
\Delta_{\text{runtime}}<5\%\sim10\%
$$

---

### 5.8 预期实验现象

合理预期如下：

1. 正常场景中，DVC-RRxIO 与原始 RRxIO 的 ATE/RPE 基本持平；
2. 低光、强光、雾、运动模糊场景中，DVC-RRxIO 的 ATE 和 RPE 应优于原始 RRxIO；
3. 雷达稀疏或几何退化场景中，DVC-RRxIO 的 NIS 超限率应显著低于原始 RRxIO；
4. 双模态退化场景中，DVC-RRxIO 不一定显著提高精度，但应减少轨迹突跳和滤波异常；
5. $\operatorname{tr}(\tilde R)$ 应在雷达残差异常、内点数下降、方向几何退化时增大；
6. $\alpha^{NIS}$ 不应长期饱和在 $\alpha_{\max}$；
7. $s_1,s_2,s_3$ 应只在 $\lambda_\ell<\tau_{\text{obs}}$ 时触发，不应全程大于 1。

---

### 5.9 失败模式与修正策略

#### 失败模式 1：协方差过度膨胀，雷达几乎不起作用

现象：$\operatorname{tr}(\tilde R_R)$ 长期过大，雷达更新后状态变化很小。

修正：

1. 降低 $c_{\text{obs}}$；
2. 降低 $s_{\max}$；
3. 降低 $\theta_3$；
4. 检查 $d_R,d_V$ 是否归一化异常；
5. 检查 $\tau_{\text{obs}}$ 是否过高。

---

#### 失败模式 2：NIS 长期超限

现象：$\rho_{\text{NIS}}\gg5\%$。

修正：

1. 增大 $\sigma^2_{v,\min}$；
2. 增大 $\alpha_{\max}$；
3. 增大 $\eta$；
4. 检查雷达外参；
5. 检查时间同步；
6. 检查雷达速度坐标系是否与观测模型一致。

---

#### 失败模式 3：视觉退化时没有触发雷达补偿

现象：$d_V$ 没有明显升高，或 $q_V$ 没有明显降低。

修正：

1. 增大视觉退化评分中的 $\sigma_{\text{photo}}$ 权重；
2. 增大 $1/N_f$ 权重；
3. 引入图像梯度均值；
4. 引入特征分布熵；
5. 调高 $\tau_V^{low}$。

---

#### 失败模式 4：视觉差且雷达差时仍然频繁更新

现象：$q_R$ 偏高但实际雷达误差大。

修正：

1. 增大 Doppler 残差标准差权重；
2. 增大 $\log\kappa_U$ 权重；
3. 降低 $\tau_R^{low}$；
4. 增大 $\theta_3$，强化双退化惩罚；
5. 增加动态物体过滤或残差中位数指标。

---

### 5.10 推荐开发里程碑

#### Milestone 1：日志化原始 RRxIO

目标：

1. 跑通原始 RRxIO；
2. 输出 REVE 速度、协方差、残差、内点数；
3. 输出视觉特征数量、光度残差；
4. 输出雷达 NIS。

验收标准：原始系统可稳定复现实验数据，所有诊断量可画图。

---

#### Milestone 2：实现雷达退化评分与方向性膨胀

目标：

$$
\tilde R_R=\alpha_R\mathbf S\Sigma^{REVE}\mathbf S^\top+\sigma^2_{\min}I
$$

暂不加入视觉耦合和 NIS。

验收标准：雷达稀疏或几何退化场景中 NIS 超限率下降。

---

#### Milestone 3：加入 NIS 一致性反馈

目标：实现 $\alpha^{NIS}$ 在线调节。

验收标准：不同序列之间参数泛化性提高，NIS 长期统计更接近理论区间。

---

#### Milestone 4：加入视觉质量耦合与更新调度

目标：启用 $\zeta_{RV}$ 和 $\nu_R$。

验收标准：视觉退化场景中轨迹误差下降；双退化场景中轨迹突跳减少。

---

#### Milestone 5：完整消融与论文图表

目标：完成所有 baseline、消融实验、NIS 图、协方差可信度图、实时性表格。

验收标准：能够支撑以下主张：

> DVC-RRxIO 在保持 RRxIO 实时松耦合结构的同时，提高了雷达速度协方差可信度和退化环境下的滤波一致性。

---

## 总结

DVC-RRxIO 的本质不是“在 RRxIO 上叠加若干经验模块”，而是围绕一个明确的测量统计问题展开：

> 雷达速度观测协方差在退化环境下如何保持可信、一致和可解释。

其工程路径低风险：不改状态维度，不重写视觉前端，不引入点级 Doppler，不做雷达辅助视觉逆深度初始化。

其学术路径也足够清晰：从雷达几何可观测性、视觉退化代理变量和 NIS 一致性三个层面对 REVE 雷达速度协方差进行保守重标定。

最应强调的论文主线是：

$$
\boxed{
\text{退化感知的各向异性协方差重标定}
+
\text{视觉质量耦合的雷达更新调度}
+
\text{NIS 驱动的在线一致性反馈}
}
$$

该路线能够有效避开 RadVIO 的直接冲突，同时保留 RRxIO 的实时性和工程简洁性，适合作为面向仪器仪表、智能感知与多源融合导航方向的完整研究方案。

---

## 附录A：当前工程实施状态快照（2026-05-19）

- W1-W4：已完成并通过门禁（`Gate-W2/Gate-W4 = PASS`）。
- W5-W6（Contribution-1，`alpha_R` 各向同性重标定）：已完成并通过严格门禁（`Gate-W6 = PASS`）。
- W5-W6 全量实验规模：`9 序列 × 2 模态 × 3 次重复 × 3 模式 = 162 runs`，`SUCCESS=162`。
- W5-W6 关键结论（`alpha_r` 对比 `base`）：
  - NIS 超限率：`5.7756% -> 2.2623%`（相对下降 `60.83%`，绝对下降 `3.513pp`）
  - ATE/RPE 中位数：未恶化
  - 运行时中位数：未超预算（`-0.48%`）
- 证据目录：`/home/yyy/datasets/irs_rtvi_datasets_2021/results/dvc_rrxio_publish/dvc_w6_alpha_r`
