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

2. 2026-05-20 固定 P2_s_curve 实物结果中，按新论文主窗口
   paper_main_excl_terminal_1s 评价，F 组是当前最稳候选：
   F = slosh cost + ax/jerk shaping。

3. G 组 one-step preview 提高了 slosh cost 占比，
   但没有带来更好的 RGB 液面结果，因此 slosh_preview_factor 继续默认 0.0。

4. 真实液面结论以 RGB 视觉为准；
   /slosh/height 只作为模型侧诊断量，不能单独作为论文效果证据。

5. terminal 前 1.0s 必须单独作为 terminal_approach_1s 诊断窗口；
   不混入论文主效果窗口。
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

当前 terminal 不再作为证明 slosh cost 效果的窗口。它是安全收敛机制，论文主效果统计改为：

```text
paper_main_excl_terminal_1s:
  TRACKING start -> 第一次进入 terminal/capture 相关状态前 1.0s

terminal_approach_1s:
  第一次进入 terminal/capture 相关状态前 1.0s -> 第一次进入 terminal/capture
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
因此论文效果统计必须排除 terminal 前 1.0s；
terminal 前 1.0s 只用于诊断停车是否平顺。
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
旧窗口：
  pre_terminal_full = TRACKING start -> first terminal/capture

新论文主窗口：
  paper_main_excl_terminal_1s = TRACKING start -> first terminal/capture - 1.0s

terminal 诊断窗口：
  terminal_approach_1s = first terminal/capture - 1.0s -> first terminal/capture
```

## 4. 实验结果

旧 pre_terminal_full 主表：

```text
condition | n | v_p95 | ax_p95 | ay_p95 | jerk_p95 | /slosh p95 mm | RGB p95 mm | duration
C         | 3 | 1.381 | 0.318  | 0.440  | 2.819    | 1.121         | 0.786      | 13.92
D         | 3 | 1.159 | 0.284  | 0.305  | 3.245    | 1.006         | 0.678      | 17.19
E         | 3 | 1.338 | 0.322  | 0.421  | 3.121    | 1.106         | 0.592      | 14.64
F         | 3 | 1.117 | 0.281  | 0.277  | 3.263    | 0.865         | 0.576      | 17.82
G         | 3 | 1.089 | 0.312  | 0.273  | 3.320    | 0.946         | 0.673      | 18.38
```

新论文主窗口 `paper_main_excl_terminal_1s`：

```text
condition | n | v_p95 | ax_p95 | ay_p95 | jerk_p95 | model p95 | model peak | RGB p95 | RGB peak | duration
C         | 3 | 1.359 | 0.281  | 0.426  | 2.743    | 1.096     | 1.658      | 0.811   | 1.238    | 12.92
D         | 3 | 1.136 | 0.256  | 0.313  | 3.061    | 0.973     | 1.452      | 0.690   | 1.460    | 16.19
E         | 3 | 1.319 | 0.281  | 0.430  | 2.820    | 1.075     | 1.659      | 0.599   | 1.214    | 13.64
F         | 3 | 1.096 | 0.241  | 0.285  | 3.042    | 0.811     | 1.366      | 0.558   | 1.167    | 16.82
G         | 3 | 1.070 | 0.264  | 0.280  | 3.092    | 0.890     | 1.379      | 0.685   | 1.519    | 17.38
```

terminal 诊断窗口 `terminal_approach_1s`：

```text
condition | n | v_p95 | ax_p95 | ay_p95 | jerk_p95 | model p95 | model peak | RGB p95 | RGB peak
C         | 3 | 1.395 | 0.955  | 0.497  | 4.548    | 1.222     | 1.639      | 0.366   | 0.392
D         | 3 | 1.176 | 1.067  | 0.253  | 4.707    | 1.725     | 1.932      | 0.263   | 0.333
E         | 3 | 1.352 | 1.037  | 0.362  | 5.140    | 1.565     | 1.889      | 0.400   | 0.421
F         | 3 | 1.136 | 0.993  | 0.248  | 5.242    | 1.741     | 1.972      | 0.491   | 0.543
G         | 3 | 1.109 | 0.927  | 0.229  | 4.768    | 1.763     | 1.894      | 0.275   | 0.329
```

相对 C 组：

```text
D:
  在新主窗口中，RGB p95、model p95、model peak、ay_p95 均下降；
  说明单独 modal slosh cost 有效。
  但 D 的 RGB peak 不稳定，D run02 抬高了均值，因此不能声称单独 slosh cost 稳定降低 RGB peak。

E:
  RGB p95 也降低；
  说明 ax/jerk 激励整形本身对真实液面有贡献。

F:
  新主窗口中 RGB p95、RGB peak、model p95、model peak、ay_p95 均下降；
  是当前最稳主线候选。

G:
  速度和 ay 更低，但 RGB p95 / RGB peak 不如 F；
  preview 没有转化成真实液面收益。
```

F 相对 C，按新主窗口：

```text
v_p95:          1.359 -> 1.096
ax_p95:         0.281 -> 0.241
ay_p95:         0.426 -> 0.285
model p95 mm:   1.096 -> 0.811
model peak mm:  1.658 -> 1.366
RGB p95 mm:     0.811 -> 0.558
RGB peak mm:    1.238 -> 1.167
duration:       12.92s -> 16.82s
```

D 相对 C，按新主窗口：

```text
v_p95:          1.359 -> 1.136
ax_p95:         0.281 -> 0.256
ay_p95:         0.426 -> 0.313
model p95 mm:   1.096 -> 0.973
model peak mm:  1.658 -> 1.452
RGB p95 mm:     0.811 -> 0.690
RGB peak mm:    1.238 -> 1.460
duration:       12.92s -> 16.19s
```

G 相对 F，按旧 pre_terminal_full：

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

F 当前是论文主窗口最干净的主结果组；
D 可作为“modal slosh cost 单独有效但 peak 不稳”的消融组；
E 可作为“激励源头整形本身有效”的消融组。
```

## 4.1 Peak 回放诊断

新增自动诊断脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/analysis/analyze_slosh_peak_context.py
```

20260520 结果：

```text
pre_terminal peaks: 15
CURRENT_K0: 7 (46.7%)
FUTURE_K_GT_0: 8 (53.3%)
terminal_near within pre_terminal: 11 (73.3%)
```

关键分组：

```text
F pre_terminal:
  peak mean = 1.972 mm
  CURRENT_K0 = 3/3
  terminal_near = 3/3
  time_to_terminal_mean = 0.085 s

F main_excluding_terminal_margin:
  peak mean = 1.366 mm
  FUTURE_K_GT_0 = 3/3
```

判读：

```text
1. F/G 在旧 pre_terminal_full 里的 model peak 主要来自 terminal 前 0.03-0.13s 的 k=0 当前状态峰值。
2. 这些 terminal-near peak 不应混入论文主效果窗口。
3. 去掉 terminal 前 1.0s 后，F 的 model peak 也下降。
4. 如果后续主窗口内 FUTURE_K_GT_0 peak 仍多，才考虑 slosh height soft constraint + slack。
5. 如果 peak 主要是 CURRENT_K0，说明峰值来自过去激励累积，应继续向前查 ax/jerk 源头。
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
3. terminal 前 1.0s 不进入主效果统计，因为 terminal 有独立停车安全逻辑。
4. F 组结果支持“modal + excitation shaping”的组合有效。
5. G 组说明更高 slosh cost 占比不必然带来更低真实液面。
6. D 组支持 modal slosh cost 单独能降低整体晃动水平，但 RGB peak 不稳定。
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
3. 论文主窗口统一为 paper_main_excl_terminal_1s。
4. static bag 只用于视觉零点/噪声参考，不进入动态组均值。
5. 若 RGB 和 /slosh/height 冲突，以 RGB 为准。
6. terminal_approach_1s 单独统计，只用于 terminal 诊断。
```

## 7.1 数据保留与剔除规则

后续录包不能采用“只保留正向结果、丢弃有效负面结果”的方式。可以做的是：

```text
1. 探索/调参包：
   可用于调参数、修流程、定位问题；
   不进入正式论文统计；
   需要单独标记为 exploratory / tuning。

2. 正式验证包：
   一旦按预先写好的方案开始录制，就必须保留所有有效结果；
   正向和负向都要进统计或作为补充材料解释。

3. 无效包可以剔除，但必须按预先规则：
   - 相机未录到 /camera/color/image_raw；
   - camera_info 分辨率错误；
   - 三标尺/HSV 明显失效；
   - 定位丢失或起点明显偏离；
   - 固定路径未正确发布；
   - MPC 未进入 TRACKING；
   - bag 缺少关键话题；
   - safety/recovery/terminal 逻辑提前接管导致主窗口不存在。

4. 剔除必须记录：
   bag 名称、剔除原因、证据截图或诊断输出。
```

论文口径：

```text
可以报告“预注册有效性筛选后的正式包结果”；
不能把有效的负面包悄悄删掉。

如果某组结果不稳定，应诚实写成：
  modal-only reduces p95/RMS but peak suppression is not robust;
  modal + excitation shaping gives the most consistent reduction.
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

5. 后续报告默认采用：
   paper_main_excl_terminal_1s 作为论文主窗口；
   terminal_approach_1s 作为 terminal 诊断窗口；
   peak report 作为解释 /slosh/height peak 的自动调试证据。
```

尚未解决的问题：

```text
1. RGB 图像清晰度受曝光/运动模糊影响，下一轮实物应固定曝光。
2. terminal 内仍有 ax/jerk 脉冲，不能纳入主效果统计。
3. /slosh/height 与 RGB 并不总一致，模型保真度仍需继续作为单独问题分析。
4. F 组降低晃动的代价是完成时间变长，需要在论文中诚实报告 trade-off。
```
