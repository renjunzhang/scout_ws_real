# 2026-05-20 Slosh-Priority MPC 阶段性总结

日期：2026-05-20

范围：

```text
本总结只覆盖 scout_local_planner 中的 MPC cost、terminal 收敛、固定路径实物对比和论文 method 写法。
不覆盖 OSCRS / GeoRef / path post-processor 主线。
```

核心结论：

```text
1. 当前主线已经从“单纯 slosh cost”转向 Slosh-Priority MPC：
   用液体模态状态项 eta/eta_dot + 激励源头项 ax/jerk 一起抑制晃动。

2. 2026-05-20 固定 P2_s_curve 实物结果中，F 组是当前最稳候选：
   F = slosh cost + ax/jerk shaping。

3. G 组 one-step preview 提高了 slosh cost 占比，
   但没有带来更好的 RGB 液面结果，因此 slosh_preview_factor 继续默认 0.0。

4. 真实液面结论以 RGB 视觉为准；
   /slosh/height 只作为模型侧诊断量，不能单独作为论文效果证据。
```

## 1. 当前 MPC 结构

当前 MPC 输入是参考路径序列，包括几何参考和参考速度。MPC 输出底盘控制命令，核心优化目标可以概括为：

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
ell_track:
  路径跟踪、姿态跟踪、速度跟踪。

ell_ctrl:
  控制输入大小和控制变化率。

ell_modal:
  液体模态状态 eta / eta_dot 的风险项。

ell_exc:
  纵向加速度 ax 和 jerk 等激励源头整形项。
```

当前代价函数主干：

$$
\begin{aligned}
\ell(x_k,u_k)=&
Q_{\text{lag}}e_{l,k}^2
+Q_{\text{contour}}e_{c,k}^2
+Q_{\theta}e_{\theta,k}^2
+Q_v(v_k-v_{\text{ref},k})^2 \\
&+R_a a_k^2
+R_{\omega}\omega_k^2
+R_{\Delta a}(a_k-a_{k-1})^2
+R_{\Delta\omega}(\omega_k-\omega_{k-1})^2 \\
&+\ell_{\text{modal}}
+\ell_{\text{exc}}
\end{aligned}
$$

`ell_modal` 采用归一化后的晃动状态项，避免毫米级量级在 QP 中太小：

```text
slosh_height_ref:
  液面高度参考尺度，用于把 eta/eta_dot 转成无量纲风险。

slosh_eta_dot_ratio:
  eta_dot 等效位移项相对于 eta 项的比例。

Q_slosh:
  模态晃动状态总权重。

Q_slosh_eta_dot:
  旧接口保留，但当前主要语义已经转向 Q_slosh + slosh_eta_dot_ratio。
```

## 2. 终点逻辑现状

当前 terminal 不再作为证明 slosh cost 效果的窗口。它是安全收敛机制，主效果统计只看：

```text
TRACKING start -> 第一次进入 terminal/capture 相关状态之前
```

当前 terminal 处理：

```text
1. terminal_slowdown_distance 内启用运动学停车包络：
   v_env(d)=min(v_max, sqrt(2*a_brake*max(0,d-goal_tolerance)))

2. pre-MPC：
   v_des=min(raw_v_des, v_env)

3. post-MPC：
   cmd_v=min(MPC_cmd_v, v_env)
   如果 GoalInfo.dx <= 0，cmd_v 强制为 0。

4. terminal phase 内对 cmd_v 做变化率限制：
   用 path_handler/max_tan_decel 限制输出层纵向减速度，
   避免 terminal 处速度阶跃制造 ax 脉冲。

5. REACHED：
   必须 speed_low 和 goal_position_reached 同时成立。
```

阶段判断：

```text
terminal smoke 已达到进入固定路径 cost 对比的最低门槛；
但 terminal 内仍可能有 ax/jerk 脉冲。
因此论文效果统计必须排除 terminal 段。
```

## 3. 2026-05-20 固定路径实验

实验目录：

```text
/data/a/slosh_bags/real/20260520_fixed_path_cost
```

分析输出：

```text
/data/a/slosh_bags/real/20260520_fixed_path_cost/red_visual_analysis_20260520
```

标注口径：

```text
三标尺标定：
  calib/red_3ruler_20260520_fixed_path.yaml

HSV：
  hue1=[0,11]
  hue2=[168,179]
  sat_min=136
  val_min=81
```

分组：

```text
C:
  SMOOTH_SPEED_RELAXED
  降低速度刚性 + 平滑控制，不加 slosh。

D:
  SLOSH_PRIORITY_MPC
  加 modal slosh cost。

E:
  AX_JERK_ONLY
  只做 ax/jerk 激励整形，不加 slosh。

F:
  SLOSH_PLUS_AX_JERK
  modal slosh cost + ax/jerk 激励整形。

G:
  SLOSH_PLUS_AX_JERK_PREVIEW
  F + slosh_preview_factor=0.5。
```

统计窗口：

```text
TRACKING start 到第一次 terminal/capture 相关状态之前；
terminal 停车段不进入主效果统计。
```

## 4. 实验结果

主表：

```text
condition | n | v_p95 | ax_p95 | ay_p95 | jerk_p95 | /slosh p95 mm | RGB p95 mm | duration
C         | 3 | 1.381 | 0.318  | 0.440  | 2.819    | 1.121         | 0.786      | 13.92
D         | 3 | 1.159 | 0.284  | 0.305  | 3.245    | 1.006         | 0.678      | 17.19
E         | 3 | 1.338 | 0.322  | 0.421  | 3.121    | 1.106         | 0.592      | 14.64
F         | 3 | 1.117 | 0.281  | 0.277  | 3.263    | 0.865         | 0.576      | 17.82
G         | 3 | 1.089 | 0.312  | 0.273  | 3.320    | 0.946         | 0.673      | 18.38
```

相对 C 组：

```text
D:
  RGB p95 降低，速度和 ay 明显下降；
  说明 slosh-priority 调参方向有效，但不是最优。

E:
  RGB p95 也降低；
  说明 ax/jerk 激励整形本身对真实液面有贡献。

F:
  RGB p95 最低；
  同时 /slosh p95 最低；
  是当前最稳主线候选。

G:
  速度和 ay 更低，但 RGB p95 不如 F；
  preview 没有转化成真实液面收益。
```

F 相对 C：

```text
v_p95:          1.381 -> 1.117
ax_p95:         0.318 -> 0.281
ay_p95:         0.440 -> 0.277
/slosh p95 mm:  1.121 -> 0.865
RGB p95 mm:     0.786 -> 0.576
duration:       13.92s -> 17.82s
```

G 相对 F：

```text
v_p95:          1.117 -> 1.089  (-2.5%)
ax_p95:         0.281 -> 0.312  (+11.0%)
ay_p95:         0.277 -> 0.273  (-1.4%)
jerk_p95:       3.263 -> 3.320  (+1.8%)
/slosh p95 mm:  0.865 -> 0.946  (+9.4%)
RGB p95 mm:     0.576 -> 0.673  (+16.9%)
RGB RMS mm:     0.257 -> 0.290  (+13.0%)
RGB peak mm:    1.167 -> 1.519  (+30.2%)
```

判断：

```text
G 当前不进入主线；
slosh_preview_factor 继续默认 0.0；
后续如果继续试 preview，应只作为 ablation，不作为默认配置。
```

## 5. Cost Contribution Check

TRACKING 阶段 cost 占比：

```text
condition | pct_v_mean | pct_control_mean | pct_smooth_mean | pct_slosh_mean
C         | 44.88      | 32.29            | 2.47            | 0.00
D         | 41.97      | 15.40            | 0.85            | 20.47
E         | 43.47      | 35.81            | 1.99            | 0.00
F         | 42.33      | 20.74            | 0.80            | 16.23
G         | 42.66      | 17.24            | 0.64            | 19.47
```

关键观察：

```text
1. D/F/G 的 slosh cost 在 QP 中确实可见，不是 1% 以下的摆设。

2. G 的 slosh cost 占比高于 F：
   16.23% -> 19.47%
   但 RGB 结果变差。

3. 因此不能用“slosh cost 占比变高”证明方法更好；
   必须以 RGB 视觉指标为主。
```

## 6. 论文 method 如何描述

不要写成：

```text
我们只加入 slosh cost，且 slosh cost 单独降低了真实液面峰值。
```

建议写成：

```text
We propose a Slosh-Priority MPC objective that combines a modal slosh-state penalty
with excitation-shaping penalties. The modal term penalizes the predicted liquid
oscillation states, while the excitation-shaping terms reduce longitudinal acceleration
and jerk that directly excite the sloshing dynamics.
```

中文口径：

```text
本文提出 Slosh-Priority MPC。
它不是只把 Q_slosh 拉大，而是重新分配路径跟踪、速度跟踪、控制平滑和晃动风险之间的优先级。
其中 eta/eta_dot 描述液体已经被激起后的模态响应，
ax/jerk 描述把液体激起来的输入源头。
二者共同组成晃动抑制目标。
```

推荐公式结构：

$$
\ell_{\text{slosh-priority}}
=
\ell_{\text{modal}}
+\ell_{\text{exc}}
$$

其中：

```text
ell_modal:
  modal slosh-state penalty
  惩罚 eta/eta_dot。

ell_exc:
  excitation-shaping penalty
  惩罚 ax/jerk 或控制变化率中会诱发纵向激励的部分。
```

论文中应明确：

```text
1. /slosh/height 是模型内部预测量，只用于解释和诊断。
2. RGB visual height 是外部真实液面指标。
3. terminal 段不进入主效果统计，因为 terminal 有独立停车安全逻辑。
4. F 组结果支持“modal + excitation shaping”的组合有效。
5. G 组说明更高 slosh cost 占比不必然带来更低真实液面。
```

## 7. 对比实验设计

推荐主实验分组：

```text
C. SMOOTH_SPEED_RELAXED
   低速度刚性 + 平滑控制；
   不加 slosh，用于排除“只是控制更平滑”的解释。

D. MODAL_SLOSH_ONLY / SLOSH_PRIORITY_BASE
   加 eta/eta_dot 模态项；
   验证模型侧晃动状态项能否改变行为。

E. EXCITATION_SHAPING_ONLY
   只加 ax/jerk；
   验证输入激励源头整形本身的贡献。

F. MODAL_PLUS_EXCITATION
   eta/eta_dot + ax/jerk；
   当前主结果组。

G. PREVIEW_ABLATION
   F + slosh_preview_factor；
   仅作为消融，不作为默认主线。
```

主指标：

```text
真实液面：
  RGB p95
  RGB RMS
  RGB peak
  max_lcr_peak 作为辅助峰值指标

控制激励：
  v_p95
  ax_p95 / ax_rms
  ay_p95 / ay_rms
  jerk_p95

模型侧：
  /slosh/height p95
  /slosh/height peak

任务代价：
  tracking duration
  轨迹 overlay
  终点是否 REACHED / 是否 GOAL_PASSED

QP 解释：
  cost contribution check
  pct_slosh / pct_v / pct_control / pct_smooth
```

统计原则：

```text
1. 每组至少 3 包。
2. 所有组使用同一条固定 P2_s_curve 路径。
3. 统计窗口统一为 TRACKING_PRE_TERMINAL。
4. static bag 只用于视觉零点/噪声参考，不进入动态组均值。
5. 若 RGB 和 /slosh/height 冲突，以 RGB 为准。
```

## 8. 当前主线建议

当前建议：

```text
1. 主线参数继续以 F 组为基础：
   modal slosh cost + ax/jerk shaping。

2. slosh_preview_factor 保持默认 0.0。
   G 组不作为默认。

3. 下一轮不要继续堆 slosh cost。
   更应该围绕 ax 脉冲、jerk 和 terminal 排除窗口做结构性治理。

4. 若需要更明显对比，优先做：
   C vs E vs F 的重复录制；
   证明 excitation shaping 与 modal term 的互补性。
```

尚未解决的问题：

```text
1. RGB 图像清晰度受曝光/运动模糊影响，下一轮实物应固定曝光。
2. terminal 内仍有 ax/jerk 脉冲，不能纳入主效果统计。
3. /slosh/height 与 RGB 并不总一致，模型保真度仍需继续作为单独问题分析。
4. F 组降低晃动的代价是完成时间变长，需要在论文中诚实报告 trade-off。
```

