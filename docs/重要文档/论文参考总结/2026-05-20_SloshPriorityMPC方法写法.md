# Slosh-Priority MPC 方法写法：把 ax/jerk 纳入晃动抑制目标

日期：2026-05-20

用途：给论文 Method 部分提供一种严谨写法，把 `eta/eta_dot` 模态状态项和 `ax/jerk` 激励整形项统一包装成“晃动优先 MPC 目标”，避免把工程平滑项写得像普通调参，也避免过度声称 slosh cost 单项已经独立证明真实液面峰值下降。

## 1. 核心论文口径

不要把方法写成：

```text
我们只加入了 slosh cost，并且 slosh cost 单独降低了真实液面峰值。
```

建议写成：

```text
我们提出一种 slosh-priority MPC objective。
它由两类互补项组成：
1. modal slosh-state penalty：惩罚预测液体模态状态 eta / eta_dot；
2. excitation-shaping penalty：惩罚产生晃动的纵向加速度和 jerk。
```

人话：

```text
eta/eta_dot 是“液体已经被激起来之后的状态”；
ax/jerk 是“把液体激起来的源头”。

把二者放进同一个晃动抑制目标里是合理的：
前者压残余晃动，后者压输入激励。
```

这类写法在优化论文中很常见。一个物理目标通常不会只由单一状态误差表示，而是写成：

```text
状态风险项 + 激励/输入正则项 + 执行平滑项
```

例如 Ferrari-style anti-sloshing trajectory planning 中，液面高度约束是核心，但时间最优目标中仍含 jerk 平滑项。它并不削弱“anti-sloshing trajectory planning”的定位，反而让轨迹可执行。

## 2. 推荐方法名称

推荐名称：

```text
Slosh-Priority MPC
Slosh-Aware Objective Rebalancing
Modal-Excitation Slosh Suppression Objective
```

最推荐：

```text
Slosh-Priority MPC
```

因为它允许目标函数同时包含：

```text
tracking
speed tracking
control effort
control smoothness
modal slosh state
excitation shaping
```

而不是把论文锁死在“单一 slosh state cost”上。

## 3. 总体优化问题写法

可以先写标准 MPC：

$$
\min_{\mathbf{u}_{0:N-1}}
\sum_{k=0}^{N-1}
\ell(\mathbf{x}_k,\mathbf{u}_k)
+ \ell_N(\mathbf{x}_N)
$$

其中状态包含车辆状态和液体模态状态：

$$
\mathbf{x}_k
=
\left[
x_k,\ y_k,\ \theta_k,\ v_k,
\eta_{x,k},\ \dot{\eta}_{x,k},\ \eta_{y,k},\ \dot{\eta}_{y,k}
\right]^T
$$

控制输入可写成：

$$
\mathbf{u}_k
=
\left[
a_k,\ \omega_k
\right]^T
$$

然后把 stage cost 分成四块：

$$
\ell
=
\ell_{\text{track}}
+ \ell_{\text{ctrl}}
+ \ell_{\text{modal}}
+ \ell_{\text{exc}}
$$

其中：

```text
ell_track: 路径跟踪和速度跟踪
ell_ctrl: 控制输入和控制变化率
ell_modal: 液体模态状态风险
ell_exc: 晃动激励源头整形
```

这样 `ax/jerk` 不是“额外凑出来的工程项”，而是 `ell_exc` 的一部分。

## 4. 跟踪项

当前可以写为：

$$
\ell_{\text{track}}
=
Q_{\text{lag}} e_l^2
+ Q_{\text{contour}} e_c^2
+ Q_{\theta} e_{\theta}^2
+ Q_v (v_k - v_{\text{ref},k})^2
$$

论文解释：

```text
contour 和 heading 保证任务仍是路径跟踪；
Q_v 不设得过硬，让速度跟踪服从晃动风险。
```

这是 Slosh-Priority MPC 和普通 tracking MPC 的关键差别之一：不是死追 `v_ref`，而是在保持路径误差可接受的前提下降低液体激励。

## 5. 控制与平滑项

基础控制项：

$$
\ell_{\text{ctrl}}
=
R_a a_k^2
+ R_{\omega} \omega_k^2
+ R_{\Delta a}(a_k-a_{k-1})^2
+ R_{\Delta \omega}(\omega_k-\omega_{k-1})^2
$$

解释口径：

```text
R_a / R_omega 不是用来证明晃动模型有效的主项；
它们防止控制输入过大。

R_Delta_a / R_Delta_omega 让控制连续，
避免 bang-bang 行为把模型内的 eta 压低、却在实物中激起液面。
```

注意：不要为了凸显 slosh 项而把 `R_a` / `R_omega` 降得很小。那会让 MPC 用很激烈的控制动作“投机”，不符合真实晃液抑制。

## 6. 模态晃动状态项

模态状态项写成：

$$
\ell_{\text{modal}}
=
Q_{\eta}
\left(
\frac{h_c^2(\eta_{x,k}^2+\eta_{y,k}^2)}
{h_{\text{ref}}^2}
\right)
+ Q_{\dot{\eta}}
\left(
\frac{h_c^2(\dot{\eta}_{x,k}^2+\dot{\eta}_{y,k}^2)}
{\omega_n^2 h_{\text{ref}}^2}
\right)
$$

也可以合并为：

$$
\ell_{\text{modal}}
=
Q_{\text{slosh}}
\left[
\frac{h_c^2(\eta_{x,k}^2+\eta_{y,k}^2)}
{h_{\text{ref}}^2}
+ \lambda_{\dot{\eta}}
\frac{h_c^2(\dot{\eta}_{x,k}^2+\dot{\eta}_{y,k}^2)}
{\omega_n^2 h_{\text{ref}}^2}
\right]
$$

解释：

```text
h_c:
  模态位移到液面高度的映射系数。

h_ref:
  参考液面高度，用于归一化，使毫米级晃动能被 MPC 看见。

omega_n:
  一阶固有频率，用 eta_dot / omega_n 把模态速度换成等效位移。

lambda_eta_dot:
  残余振荡速度项相对于位移项的重要性。
```

论文中要强调：

```text
modal term penalizes the predicted liquid state accumulated through the second-order slosh dynamics.
```

也就是它有记忆，不是瞬时加速度阈值。

## 7. 激励整形项：把 ax/jerk 包装进晃动抑制目标

推荐写法：

$$
\ell_{\text{exc}}
=
Q_{a_x}
\left(
\frac{a_{x,k}}{a_{x,\text{ref}}}
\right)^2
+ Q_{j_x}
\left(
\frac{j_{x,k}}{j_{x,\text{ref}}}
\right)^2
$$

其中：

$$
j_{x,k}
=
\frac{a_{x,k}-a_{x,k-1}}{\Delta t}
$$

如果考虑横向激励，也可以扩展为：

$$
\ell_{\text{exc}}
=
Q_{a_x}
\left(
\frac{a_{x,k}}{a_{x,\text{ref}}}
\right)^2
+ Q_{a_y}
\left(
\frac{a_{y,k}}{a_{y,\text{ref}}}
\right)^2
+ Q_{j_x}
\left(
\frac{j_{x,k}}{j_{x,\text{ref}}}
\right)^2
$$

其中 `a_y` 可以由底盘运动近似：

$$
a_{y,k}
\approx
v_k \omega_k
$$

或在路径曲率口径下：

$$
a_{y,k}
\approx
v_k^2 \kappa_k
$$

解释口径：

```text
Liquid sloshing is driven by base acceleration. Therefore, a slosh-suppression objective should not only penalize the predicted liquid state, but also shape the acceleration inputs that excite the state.
```

中文：

```text
液体晃动由底盘加速度激励产生。
因此，晃动抑制目标不应只惩罚预测出的液体状态，
还应约束产生该状态的加速度输入。
```

这就是把 `ax/jerk` 包装成“激励源头项”的核心。

## 8. 为什么 ax/jerk 不是普通平滑调参

论文里可以这样区分：

```text
普通 smooth cost:
  目标是让控制输入数值更平滑。

excitation-shaping term:
  目标是降低液体动力学的输入激励。
```

虽然数学上都可能包含 `a_k`、`\Delta a_k` 或 `j_k`，但解释不同：

```text
R_Delta_a:
  控制执行层面的平滑正则。

Q_jx:
  液体激励层面的 jerk 抑制。
```

如果实现里暂时共用了同一个变量或权重，也可以在论文里写成：

```text
The acceleration and jerk penalties serve as excitation-shaping regularizers, since longitudinal acceleration is the dominant input to the slosh oscillator in the tested S-curve task.
```

但要避免说：

```text
ax/jerk 是 Ferrari 模型的液面高度项。
```

它不是液面高度项，而是激励项。

## 9. 与 Ferrari-style 方法的关系

Ferrari-style anti-sloshing trajectory planning 的核心是：

```text
用液面高度 eta 作为约束或评价指标；
同时通过轨迹 jerk / motion law 让轨迹可执行、不过激。
```

Scout 的在线 MPC 不能直接复制 Ferrari 的离线 NLP，但可以借鉴这个结构：

```text
Ferrari:
  liquid height constraint + time/jerk objective

Scout:
  modal slosh-state penalty + acceleration/jerk excitation shaping
```

可写成：

```text
Inspired by anti-sloshing trajectory optimization, we separate the suppression objective into a predicted liquid-state term and an excitation-shaping term. The former penalizes the modal response of the liquid, while the latter reduces the acceleration inputs that excite the slosh dynamics.
```

这比单纯说 “inspired by Ferrari” 更清楚：借鉴的是“液面风险 + 可执行运动整形”的结构，而不是照搬机械臂离线优化。

## 10. 推荐 Method 英文段落

可以直接作为论文初稿：

```text
We formulate the local tracking problem as a slosh-priority MPC problem. In addition to the conventional contouring, heading, velocity-tracking, and control-effort terms, the proposed objective includes two complementary slosh-related components. The first component penalizes the predicted modal slosh state, including both the modal displacement and the modal velocity. The modal velocity is normalized by the natural frequency of the first slosh mode and is therefore interpreted as an equivalent displacement term. This component accounts for residual liquid motion accumulated through the second-order slosh dynamics.

The second component shapes the excitation applied to the liquid. Since the dominant input to the slosh oscillator is the base acceleration, we penalize longitudinal acceleration and jerk within the prediction horizon. This term is not introduced as a generic smoothing heuristic; rather, it suppresses the acceleration pulses that excite the liquid state. The resulting objective therefore combines modal-state suppression with excitation shaping, while preserving path-tracking performance through contour and heading penalties.
```

更短版：

```text
The proposed Slosh-Priority MPC combines a modal slosh-state penalty with excitation-shaping penalties. The modal term penalizes the predicted liquid displacement and residual modal velocity, whereas the excitation term penalizes longitudinal acceleration and jerk, which are the dominant inputs that excite the slosh dynamics. This formulation allows the controller to trade speed tracking for reduced liquid excitation while maintaining path-tracking accuracy.
```

## 11. 推荐中文论文表述

```text
本文将局部轨迹跟踪问题表述为晃动优先 MPC。与传统路径跟踪 MPC 不同，本文的目标函数不仅包含轮廓误差、航向误差、速度跟踪误差和控制输入代价，还引入了两类与晃动抑制相关的项：液体模态状态项和激励整形项。

液体模态状态项惩罚预测域内的模态位移和模态速度，用于抑制由二阶晃动动力学累积形成的残余液体运动。模态速度项通过一阶固有频率归一化为等效位移，从而与模态位移项在同一尺度下进入优化。

激励整形项则直接作用于产生晃动的输入源头。由于底盘纵向加速度及其突变会激励液体模态响应，本文在预测域内惩罚纵向加速度和 jerk，从源头上降低液体晃动的激励强度。该项不是普通的经验平滑调参，而是作为晃动动力学输入侧的抑制项，与模态状态项共同构成晃动优先目标。
```

## 12. 实验解释口径

建议实验分组写成：

```text
C: Smooth-speed relaxed MPC
   降低速度跟踪刚性，保持基本路径跟踪。

D: Modal slosh-state MPC
   在 C 的基础上加入 eta / eta_dot 模态状态项。

E: Excitation-shaping MPC
   在 C 的基础上加入 ax / jerk 激励整形项，不加入模态状态项。

F: Slosh-priority MPC
   同时加入模态状态项和激励整形项。
```

论文结论建议写：

```text
The combined Slosh-Priority MPC achieved the most consistent reduction in visual high-percentile slosh height, model-predicted slosh, and lateral excitation.
```

中文：

```text
组合式晃动优先 MPC 在视觉高分位液面高度、模型预测晃动和横向激励上取得了最一致的降低。
```

不要写：

```text
slosh-state cost alone reduced the true peak sloshing height.
```

当前数据更适合支撑：

```text
slosh-state term + excitation-shaping term 的组合有效。
```

## 13. 如何处理“工程优化”的质疑

如果审稿人认为 `ax/jerk` 只是工程调参，可以这样回答：

```text
The acceleration and jerk terms are not introduced solely for numerical smoothness. They correspond to the input channel of the slosh dynamics. The modal state penalty acts on the predicted response, whereas the excitation-shaping terms act on the cause of that response. The ablation study separates these effects by comparing the modal-only, excitation-only, and combined objectives.
```

中文：

```text
加速度和 jerk 项并非单纯为了数值平滑，而是对应晃动动力学的输入通道。模态状态项作用于液体响应，激励整形项作用于产生该响应的原因。消融实验通过 modal-only、excitation-only 和 combined 三组对比区分二者贡献。
```

这个回答的关键是：必须保留 E 组，否则无法证明 `ax/jerk` 不是偷偷替代 slosh cost 的普通平滑项。

## 14. 当前 20260520 实物结果的安全写法

基于当前 C/D/E/F 数据，推荐写：

```text
In fixed-path real-robot trials, the combined objective produced the lowest visual p95/RMS slosh height, the lowest model-predicted p95 slosh height, and the lowest lateral excitation among the tested groups. The modal-only objective reduced high-percentile visual slosh but did not consistently reduce the visual peak, while the excitation-only objective also improved the visual high-percentile metrics. These results suggest that modal-state suppression and excitation shaping are complementary.
```

中文：

```text
在固定路径实物实验中，组合目标在视觉 p95/RMS 液面高度、模型预测 p95 晃动高度和横向激励上均取得最低值。仅加入模态状态项时，视觉高分位指标有所降低，但视觉峰值并不稳定；仅加入激励整形项时，视觉高分位指标也有改善。这说明模态状态抑制和激励整形具有互补作用。
```

避免写：

```text
slosh cost 单独显著降低了真实液面峰值。
```

可以写：

```text
slosh-priority objective reduced high-percentile visual slosh.
```

## 15. 最终推荐公式汇总

论文主公式可以压缩成：

$$
\ell
=
\ell_{\text{track}}
+ \ell_{\text{ctrl}}
+ \ell_{\text{modal}}
+ \ell_{\text{exc}}
$$

其中：

$$
\ell_{\text{modal}}
=
Q_{\text{slosh}}
\left[
\frac{h_c^2(\eta_x^2+\eta_y^2)}
{h_{\text{ref}}^2}
+ \lambda_{\dot{\eta}}
\frac{h_c^2(\dot{\eta}_x^2+\dot{\eta}_y^2)}
{\omega_n^2 h_{\text{ref}}^2}
\right]
$$

$$
\ell_{\text{exc}}
=
Q_{a_x}
\left(
\frac{a_x}{a_{x,\text{ref}}}
\right)^2
+ Q_{j_x}
\left(
\frac{j_x}{j_{x,\text{ref}}}
\right)^2
$$

如果最后实现中没有显式 `Q_ax / Q_jx`，而是通过 `R_a / R_Delta_a` 或硬约束体现，也可以写成：

```text
In implementation, the excitation-shaping term is realized through bounded longitudinal acceleration and jerk-aware control regularization.
```

中文：

```text
在实现中，激励整形项通过纵向加速度边界和 jerk-aware 控制正则实现。
```

这样既能把 `ax/jerk` 纳入晃动抑制方法，又不会把它伪装成液体模态高度项。
