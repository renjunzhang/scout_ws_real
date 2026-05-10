# Scout 当前晃液模型与 Ferrari 2026 模型差异总结

日期：2026-05-10

相关代码：

```text
src/scout_apps/control/slosh_models/
src/scout_apps/control/scout_local_planner/
```

相关论文：

```text
Ferrari et al., 2026, Time-Optimal Anti-Sloshing Trajectory Planning for Multiple Liquid-Filled Containers Subject to SCARA Motion
```

## 1. 总体结论

Scout 当前模型和 Ferrari 2026 同属 MSD（mass-spring-damper）晃液建模路线，都使用一阶模态位移幅值估计液面高度：

$$
r_{\mathrm{modal}}=\sqrt{\eta_x^2+\eta_y^2}
$$

但两者目标不同：

```text
Ferrari:
  更完整的 4D SCARA 晃液动力学模型，用于离线时间最优轨迹规划和硬约束验证。

Scout 当前模型:
  更轻量的在线/准在线一阶线性模态模型，用于 MPC 调试、/slosh/height 发布、
  GeoRef/OSCRS 候选路径评分和 hard gate。
```

最关键差异是液面高度映射系数。

Scout 当前默认 `observer_linear`：

$$
c_{h,\mathrm{current}}=\frac{4 h m_n}{m_F R}
$$

Ferrari closed-form：

$$
c_{h,\mathrm{Ferrari}}=\frac{\xi^2 h m_n}{m_F R}
$$

一阶模态：

$$
\xi=1.8412
$$

因此：

$$
\frac{c_{h,\mathrm{current}}}{c_{h,\mathrm{Ferrari}}}
=\frac{4}{\xi^2}
=1.1799
$$

即在相同模态位移下，Scout 当前默认高度估计约比 Ferrari 闭式高 18%，更保守。

## 2. 动力学模型差异

### 2.1 Scout 当前模型

当前 `LiquidSloshModel` 使用解耦的二维一阶线性 MSD：

$$
\ddot{\eta}_x+2\zeta\omega_n\dot{\eta}_x+\omega_n^2\eta_x=-a_x
$$

$$
\ddot{\eta}_y+2\zeta\omega_n\dot{\eta}_y+\omega_n^2\eta_y=-a_y
$$

状态：

$$
\mathbf{x}_{\mathrm{slosh}}
=
\begin{bmatrix}
\eta_x & \dot{\eta}_x & \eta_y & \dot{\eta}_y
\end{bmatrix}^{\mathsf T}
$$

输入：

$$
\mathbf{u}_{\mathrm{slosh}}
=
\begin{bmatrix}
a_x & a_y
\end{bmatrix}^{\mathsf T}
$$

若容器相对 yaw 轴有偏置 `(offset_x, offset_y)`，代码中支持：

$$
a_{x,\mathrm{eff}}=a_x-\alpha_z\,o_y-\omega_z^2 o_x
$$

$$
a_{y,\mathrm{eff}}=a_y+\alpha_z\,o_x-\omega_z^2 o_y
$$

但当前实物默认 `offset_x=0, offset_y=0`，因此大多数在线实验等价于容器位于底盘 yaw 轴中心。

### 2.2 Ferrari 模型

Ferrari 使用更完整的 4D SCARA MSD 模型：

```text
3D translation + yaw rotation
container offset on tray
modal mass constrained on paraboloid
```

其 EOM 中显式包含：

```text
平动加速度项
yaw 角加速度项
yaw 角速度平方离心项
模态位移之间的非线性/抛物面耦合项
```

因此 Ferrari 更适合解释“容器在旋转 tray 上不同位置”的真实激励差异，也更适合多容器 SCARA 场景。Scout 当前模型保留了 offset 激励的一阶工程近似，但没有完整引入 Ferrari 的 4D 抛物面耦合矩阵。

## 3. 参数公式差异

两者的一阶模态根、固有频率、模态质量基本同源：

$$
\xi_{11}=1.8412
$$

$$
\omega_n=\sqrt{\frac{g\xi}{R}\tanh\left(\frac{\xi h}{R}\right)}
$$

$$
m_n=m_F\frac{2R}{\xi h(\xi^2-1)}
\tanh\left(\frac{\xi h}{R}\right)
$$

差异主要在阻尼和液面高度映射。

### 3.1 阻尼

Scout 当前在线口径：

$$
\zeta=0.05
$$

Ferrari 论文给出物理/半经验阻尼公式，依赖液体黏度、密度、容器半径和液位。当前 `oscrs_container.yaml` 已预留：

```text
damping_ratio_mode:
  manual
  ferrari_physics
```

但在线 post-processor 仍使用 manual `damping_ratio`；`ferrari_physics` 主要用于离线 ablation/oracle。

### 3.2 液面高度系数

当前 C++ 中已经有两个函数。

`computeHeightCoeffLinear`：

$$
c_{h,\mathrm{current}}=\frac{4 h m_n}{m_F R}
$$

`computeHeightCoeffNonlinear`：

$$
c_{h,\mathrm{Ferrari}}=\frac{\xi^2 h m_n}{m_F R}
$$

默认配置：

```text
use_linear_model: true
```

所以当前 `/slosh/height` 和 MPC/OSCRS 默认使用 `observer_linear`。如果改成 `use_linear_model: false`，C++ 高度系数会切到 Ferrari closed-form 口径。但在线 OSCRS post-processor 当前直接读 `slosh_score.height_coeff`，不会自动按 `height_coeff_mode` 重算；`height_coeff_mode=ferrari_closed_form` 目前主要给离线 `ferrari_oracle` 使用。

## 4. 高度输出差异

Scout 当前高度：

$$
h_{\mathrm{modal}}
=
c_{h,\mathrm{current}}\sqrt{\eta_x^2+\eta_y^2}
$$

$$
h_{\mathrm{parabola}}
=
\frac{\omega^2 R^2}{4g}
$$

$$
h_{\mathrm{total}}
=
h_{\mathrm{modal}}+h_{\mathrm{parabola}}
$$

Ferrari closed-form 主模态高度：

$$
h_{\mathrm{modal,Ferrari}}
=
c_{h,\mathrm{Ferrari}}\sqrt{\eta_x^2+\eta_y^2}
$$

因此，在相同 `eta_x, eta_y` 下：

$$
h_{\mathrm{modal,current}}
\approx
1.18\,h_{\mathrm{modal,Ferrari}}
$$

这说明当前工程口径更保守，但不一定更贴合实物视觉真值。实物验证后，应离线比较：

```text
visual ground truth
vs /slosh/height current observer_linear
vs Ferrari closed-form height
vs current/Ferrari + parabola term
```

## 4.1 实物参数实例（mpc_params.yaml）

当前实物参数来自：

```text
src/scout_apps/control/scout_local_planner/config/mpc_params.yaml
```

对应配置：

```text
container_radius: 0.0185 m
liquid_height: 0.058 m
liquid_density: 1000.0 kg/m^3
damping_ratio: 0.05
mode_index: 1
offset_x: 0.0 m
offset_y: 0.0 m
use_parabola_term: true
use_linear_model: true
```

按这些实物参数计算：

$$
m_F=\rho\pi R^2h=0.06236\ \mathrm{kg}
$$

$$
\omega_n=31.246\ \mathrm{rad/s}
$$

$$
f_n=\frac{\omega_n}{2\pi}=4.973\ \mathrm{Hz}
$$

$$
m_n=0.00904\ \mathrm{kg}
$$

$$
\frac{m_n}{m_F}=0.14497
$$

当前工程默认高度系数：

$$
c_{h,\mathrm{current}}=1.81794
$$

Ferrari closed-form 高度系数：

$$
c_{h,\mathrm{Ferrari}}=1.54071
$$

两者比值：

$$
\frac{c_{h,\mathrm{current}}}{c_{h,\mathrm{Ferrari}}}=1.17993
$$

因此，当前实物参数下，`observer_linear` 在相同模态位移上约比 Ferrari closed-form 高 18%。

抛物面项在当前试管半径下量级较小：

$$
h_{\mathrm{parabola}}(\omega=1.0)=0.0087\ \mathrm{mm}
$$

$$
h_{\mathrm{parabola}}(\omega=1.5)=0.0196\ \mathrm{mm}
$$

所以在 phase4 这种毫米级视觉晃动中，抛物面项存在但不是主导项；主差异仍来自模态高度系数和模态状态本身。

当前实物 `mpc` 主控参数还需要注意：

```text
Q_slosh: 0.0
Q_slosh_eta_dot: 0.0
enable_slosh_box_constraint: false
risk_scheduler.enable: false
slosh_speed_governor.enable: false
```

也就是说，当前实物主实验中，`slosh_models` 仍会用于 `/slosh/height`、debug 发布和 OSCRS/GeoRef 外层评分；但 normal MPC 本体没有启用液体晃动软代价、盒约束、risk scheduler 或速度治理。RAW/FIXED/OSCRS 的核心差异应归因到参考路径选择层，而不是 MPC 内部晃动代价。

## 5. 在 scout_local_planner 中的使用差异

当前使用位置有三类。

### 5.1 MPC 软代价

MPC 中不是直接惩罚高度，而是用高度系数换算成模态位移权重：

$$
J_{\mathrm{slosh}}
=
Q_{\mathrm{slosh}}h^2
\approx
Q_{\mathrm{slosh}}c_h^2(\eta_x^2+\eta_y^2)
$$

$$
Q_{\mathrm{slosh},\eta}
=
Q_{\mathrm{slosh}}c_h^2
$$

因此高度系数从 current 切到 Ferrari 后：

$$
\frac{Q_{\mathrm{slosh},\eta,\mathrm{Ferrari}}}
{Q_{\mathrm{slosh},\eta,\mathrm{current}}}
=
\left(
\frac{c_{h,\mathrm{Ferrari}}}{c_{h,\mathrm{current}}}
\right)^2
\approx
0.718
$$

同样的 `Q_slosh` 下，Ferrari 系数会让 MPC 晃动惩罚弱约 28%。

### 5.2 模态盒约束

若启用一阶盒约束，代码将液面高度上限换算成模态位移上限：

$$
\bar{\eta}
=
\frac{h_{\mathrm{budget}}}{c_h\sqrt{2}}
$$

切到 Ferrari 系数后，因为 `ch` 变小：

$$
\bar{\eta}_{\mathrm{Ferrari}}
\approx
1.18\,\bar{\eta}_{\mathrm{current}}
$$

也就是说，约束会更宽松。若直接切换公式，必须同步检查 `slosh_height_max`、`eta_lim_mm` 和 residual gate。

### 5.3 OSCRS 候选评分与 hard gate

在线 post-processor rollout 使用：

$$
h_{\mathrm{total}}
=
c_h\sqrt{\eta_x^2+\eta_y^2}
+
h_{\mathrm{parabola}}
$$

并输出：

```text
slosh_h_p95
slosh_h_max
slosh_h_residual_max
slosh_eta_dot_rms
slosh_energy_rms
slosh_terminal_E
```

OSCRS hard gate 使用：

$$
h_{\max}\le \eta_{\lim}
$$

$$
h_{\mathrm{residual,max}}
\le
r_{\mathrm{residual}}\eta_{\lim}
$$

因此公式切换会直接改变候选是否通过 gate，以及 OSCRS 是否 takeover。

## 6. 与 Ferrari 的工程取舍

| 维度    | Ferrari 2026         | Scout 当前                                |
| ----- | -------------------- | --------------------------------------- |
| 任务    | 离线时间最优轨迹规划           | 在线候选路径选择 + MPC 跟踪                       |
| 平台    | SCARA 4D 机械臂/托盘      | Scout Mini 差速底盘                         |
| 模型复杂度 | 更完整，含 4D 旋转/偏置/抛物面耦合 | 一阶线性解耦 MSD，支持 offset 工程修正               |
| 高度系数  | `xi^2*h*mn/(mF*R)`   | 默认 `4*h*mn/(mF*R)`                      |
| 保守性   | 一阶闭式更标准              | 当前默认约高 18%，更保守                          |
| 计算用途  | NLP hard constraint  | `/slosh/height`、MPC 代价、OSCRS score/gate |
| 在线可用性 | 原始 NLP 不适合高频在线       | 当前 rollout 轻量，可在线/准在线                   |
| 验证方式  | GoPro/实验液面 vs model  | RealSense 红色液体视觉 vs `/slosh/height`     |

## 7. 建议验证路线

### 7.1 离线先做公式 ablation

不要先改在线控制代码。建议先用 phase4 和后续实物 bag 离线重放同一模态状态/激励，比较：

```text
A. current observer_linear
B. Ferrari closed-form
C. current + parabola
D. Ferrari + parabola
```

指标使用：

```text
gamma_model_pct
RMSE
e_p95 / U_p95
U_max
A_rank
under_ratio
threshold-sweep recall
```

如果 Ferrari closed-form 在视觉真值上同时满足：

```text
RMSE 更低
U_p95 / U_max 不变差
A_rank 更高
OSCRS vs RAW 的方向与视觉更一致
```

才考虑进入工程切换。

### 7.2 工程和论文可先分离

短期建议：

```text
论文:
  可以报告 Ferrari closed-form 的离线保真度结果，
  作为更标准 MSD 映射的对照。

工程:
  继续保留 observer_linear 默认口径，
  因为它更保守，且当前 MPC/OSCRS 阈值均按该口径调过。
```

中期建议：

```text
给 OSCRS 增加 height_coeff_mode 在线选项：
  observer_linear
  ferrari_closed_form
  manual

并在 candidate_report 中记录实际使用的 height_coeff_mode 和 height_coeff。
```

长期建议：

```text
若 Ferrari closed-form 被视觉验证为更稳定：
  1. 切换 OSCRS rollout 的 height_coeff；
  2. 同步重标 eta_lim_mm / residual_ratio；
  3. 同步重算 MPC Q_slosh_eta 和 eta_bar；
  4. 保留 observer_linear 作为 conservative fallback。
```

## 8. 推荐论文表述

英文版本：

```text
Our online OSCRS implementation uses a lightweight first-mode MSD observer
with an observer-linear height map ch = 4hmn/(mF R), which is about 18%
more conservative than the Ferrari-style closed-form coefficient
ch = xi^2 hmn/(mF R) for the first mode. This choice keeps the online
gate conservative and computationally cheap. In the offline validation,
we additionally evaluate the Ferrari-style closed-form height map against
the RealSense red-liquid visual ground truth to quantify whether the
standard MSD mapping better explains the physical experiment.
```

中文版本：

```text
当前 OSCRS 在线实现采用轻量一阶 MSD 观测器，并使用
ch = 4hmn/(mF R) 的 observer-linear 高度映射。对一阶模态 xi=1.8412，
该系数约比 Ferrari-style 闭式映射 ch = xi^2 hmn/(mF R) 高 18%，
因此在线 gate 更保守、计算更便宜。实物验证阶段，我们将离线比较
Ferrari 闭式映射与 RealSense 红色液体视觉真值的拟合度，以判断是否
需要在 OSCRS 中切换高度映射，或保留为用户可选项。
```
