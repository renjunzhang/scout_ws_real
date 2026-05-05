# 2026-05-01 Anti-Slosh 轨迹几何层重设计方案

## 0. 2026-05-05 方案升级结论

当前文档的原始版本已经完成一轮闭环证伪：

```text
C1 waypoint smoothing
C2 turn radius inflation / radius path
C3 简单 low-jerk ramp / VEHICLE_V_MAX 调速
```

结论：

```text
低复杂度 C1/C2/C3 版本失败；
轨迹/参考生成层方向未失败。
```

失败边界：

```text
P3:
  radius path 给出正信号；
  h / eta_dot / energy / ay / track_dist 均改善，tracking_time +4.7%。

P2:
  低速 v175 能降晃，但 tracking_time +34.7%；
  提速 v205 能过时间，但 h_max / eta_dot / track_dist 失败。
```

因此下一版不再做：

```text
手工 smoothing；
手工 turn radius inflation；
继续扫 VEHICLE_V_MAX；
继续调 PROFILE_ENERGY；
继续录同类 P2 radius bag。
```

升级后的目标是：

```text
Constrained anti-slosh reference optimization

在离线层同时优化 p(s) 与 v(s) / t(s)，并显式约束：
  time
  trackability
  ax / jerk
  kappa / dkappa
  terminal residual
  modal slosh response
```

只有离线优化同时满足这些约束，才进入 ROS 闭环仿真。

## 1. 背景

`PROFILE_ENERGY` 已经验证：

```text
速度剖面层 + slosh rollout correction 能降低 model-estimated slosh 指标；
但在给定几何路径不变的前提下，会在 tracking_time、ay_p95、tracking_p95 上付出代价。
```

因此下一步不再继续调 `energy_profile_*` 参数。

新的方向是把防晃逻辑进一步前移到轨迹/路径几何生成层：

```text
目标点 / 障碍约束
    ↓
anti-slosh path / trajectory generation
    ↓
速度剖面与 MPC tracking
```

不是：

```text
固定几何路径
    ↓
只改 v(s)
```

## 2. 核心假设

当前失败不是因为 slosh rollout 完全无效，而是因为固定路径几何给 MPC 留下的可行空间太小：

```text
为了降 slosh，需要降低或平滑激励；
但固定 S 弯 / mixed 几何仍要求一定转向和加减速；
MPC 为追踪路径会重新产生 ay / ax / tracking 代价。
```

所以应该允许优化：

```text
路径曲率 kappa(s)
曲率变化 dkappa(s)
局部转弯半径
waypoint timing
terminal rest-to-rest 段
```

而不只是优化：

```text
v_ref(s)
```

## 3. 最小可证伪目标

先不写在线控制器。

只做离线 candidate 生成和回放验证：

```text
输入：
  起点、终点、固定障碍/边界、当前 P2/P3 路径

输出：
  一条候选几何路径 p(s)
  一条候选速度/时间参数 v(s) 或 p(t)

验证：
  linear modal slosh rollout
  geometry risk metrics
  tracking proxy
```

通过门槛：

```text
tracking_time_proxy <= NOM * 1.15
h_p95_pred 下降
energy_pred 下降
eta_dot_pred 下降
ay_p95_pred 不升
kappa / dkappa / jerk 不恶化
```

任一不通过：

```text
不进入控制器代码；
不录新 bag；
先修离线生成器或承认该路径族不可达。
```

## 4. Step 0：路径几何审查

目标：

```text
判断当前 P2/P3 的 slosh 峰值到底由哪些几何/执行事件触发。
```

对已有 bag 和 fixed path 统计：

```text
kappa_p95 / kappa_max
dkappa_p95 / dkappa_max
ay_ref_p95 = v_ref^2 * kappa
ax_ref_p95 = dv/dt
alpha_ref = ax*kappa + v^2*dkappa
jerk_ref = d ax / dt
odom_ax / odom_ay
track_dist_p95
height / eta_dot 峰值前 0.5~1.0s 的事件窗口
```

输出：

```text
P2/P3 哪些路径段需要几何改造；
是曲率太大、dkappa 太尖、纵向 timing 太急，还是 MPC tracking 放大。
```

## 5. Step 1：离线 anti-slosh candidate 生成

先做简单版本，不做复杂全局优化。

候选方法：

```text
1. waypoint smoothing:
   对固定路径 waypoint 做曲率连续化和 dkappa 限制。

2. turn radius inflation:
   在允许空间内放大转弯半径，降低 kappa 和 dkappa。

3. rest-to-rest terminal segment:
   末端增加低 jerk 减速段，降低 terminal eta / eta_dot。

4. timing retiming:
   用 slosh rollout 对 p(s) 的 v(s) 重新定时，但不强行在坏几何上补救。
```

不做：

```text
不直接实现 flatness + HSMC；
不直接重写 MPC；
不在 QP 内加新 slosh cost；
不写路径分类器。
```

## 6. Step 2：离线 rollout 验证

使用现有 `PROFILE_ENERGY` rollout 思路，但验证对象换成完整 candidate：

```text
candidate p(s), v(s)
    ↓
signed ax / ay / alpha / jerk sequence
    ↓
eta_x / eta_y / eta_dot / E rollout
```

必须保留 signed 输入，不使用纯绝对值窗口替代。

验收：

```text
P2:
  h_p95_pred / energy_pred / eta_dot_pred 下降
  ay_p95_pred 不升
  time_proxy <= +15%

P3:
  eta_x energy ratio 下降
  ax_p95 / jerk_p95 不升
  track_dist proxy 不恶化
```

## 7. Step 3：闭环仿真进入条件

只有当 Step 2 通过，才进入 ROS 仿真。

仿真最小集：

```text
P2 anti-slosh trajectory x2
P3 anti-slosh trajectory x2
NOM matched-time slow x1
```

不再为了“凑三包”盲目录 bag。

如果 P2 已失败：

```text
不跑 P3。
```

## 8. 论文定位

如果轨迹几何层成功，论文可以写：

```text
Reference-generation-first anti-slosh MPC tracking
```

更准确表述：

```text
We show that slosh suppression should be handled at the reference generation level.
Instead of relying on soft slosh costs inside MPC or post-hoc command clipping,
we generate low-excitation geometric/timed references and use MPC as the constrained tracker.
```

如果仍失败，论文应转为：

```text
系统建模、消融实验与失败机理分析；
说明给定 /cmd_vel tracking 架构下，哪些 anti-slosh 方法不能闭环满足完整约束。
```

## 9. 当前不做的事

```text
不继续 PROFILE_ENERGY 参数扫；
不录 PROFILE_ENERGY P3；
不提交为“成功控制器”；
不把 /slosh/height 写成真实液面，除非完成视觉验证；
不把 MPC slosh cost 写成主贡献。
```

## 10. 2026-05-05 Step 0 执行结果

已新增离线诊断脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/analyze_path_geometry_slosh_triggers.py
```

已分析：

```text
P2_s_curve NOM x3
P3_mixed NOM x3
peak_signal = height / eta_dot
lookback = 1.0s
```

输出：

```text
docs/Claude/分析数据/2026-05-05_step0_path_geometry_triggers_p2_p3_nom.csv
docs/Claude/分析数据/2026-05-05_step0_path_geometry_triggers_p2_p3_nom_eta_dot.csv
```

主要结论：

```text
P2:
  早期峰值与 high_kappa / sharp_dkappa / longitudinal_timing 有关；
  后段峰值与 longitudinal_timing + lateral_excitation + tracking 距离增大有关。

P3:
  第一峰值几何 kappa/dkappa 很低，但 ax 很高，属于纵向 timing/加速度脉冲；
  后续峰值同时出现 high_kappa、sharp_dkappa、longitudinal_timing 和 lateral_excitation。
```

因此 Step 1 不应只做单一 candidate：

```text
只做 timing retiming：不能解决 P3 后段 high kappa/dkappa；
只做 waypoint smoothing：不能解决早期 ax 脉冲；
只做 turn radius inflation：不能解决 terminal/起步 timing。
```

下一步候选生成应限制为三类低复杂度组合：

```text
C1 waypoint smoothing：压 dkappa；
C2 turn radius inflation：压 kappa 和 ay；
C3 low-jerk timing / terminal rest-to-rest：压 ax、jerk、terminal eta_dot。
```

## 11. 2026-05-05 Step 1 第一小步执行结果

已新增离线候选生成脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/generate_anti_slosh_path_candidates.py
```

本轮只生成几何候选：

```text
C1 smooth：轻量 waypoint smoothing
C2 radius：更强 smoothing / turn radius inflation proxy
```

暂缓 C3：

```text
C3 low-jerk timing / terminal rest-to-rest 需要 v(s) 或 p(t)；
应在几何候选通过基础指标后再做，避免把 path 与 timing 的效果混在一起。
```

输出候选：

```text
/data/a/fixed_paths/candidates/P2_s_curve_smooth.json
/data/a/fixed_paths/candidates/P2_s_curve_radius.json
/data/a/fixed_paths/candidates/P3_mixed_smooth.json
/data/a/fixed_paths/candidates/P3_mixed_radius.json
```

几何对比：

```text
docs/Claude/分析数据/2026-05-05_step1_candidate_geometry_summary.csv
```

摘要：

```text
P2_s_curve:
  original_resampled kappa_p95=1.565 dkappa_p95=4.422
  smooth             kappa_p95=1.231 dkappa_p95=2.615 drift=0.023m
  radius             kappa_p95=1.109 dkappa_p95=2.102 drift=0.052m

P3_mixed:
  original_resampled kappa_p95=1.742 dkappa_p95=4.713
  smooth             kappa_p95=1.469 dkappa_p95=2.881 drift=0.025m
  radius             kappa_p95=1.271 dkappa_p95=3.084 drift=0.061m
```

结论：

```text
C1/C2 几何指标有效改善；
起终点 position / orientation 保持原始 fixed path；
可以进入 Step 2：对 original/smooth/radius 做 signed slosh rollout + tracking proxy 验证。
```

## 12. 2026-05-05 Step 2 第一版执行结果

已新增离线 rollout/proxy 脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/evaluate_anti_slosh_path_candidates.py
```

本轮口径：

```text
v_ref = 1.2 m/s constant
只比较路径几何导致的 lateral input:
  uy = v_ref^2 * signed kappa(s)
  alpha proxy = v_ref^2 * dkappa(s)
不包含纵向 ax timing。
```

输出：

```text
docs/Claude/分析数据/2026-05-05_step2_candidate_rollout_proxy.csv
```

结果摘要：

```text
P2 smooth:
  h_p95 -65.5%
  energy_p95 -92.9%
  ay_p95 -34.9%
  alpha_p95 39.82 -> 4.55

P2 radius:
  h_p95 -70.0%
  energy_p95 -94.4%
  ay_p95 -41.1%
  alpha_p95 39.82 -> 4.19

P3 smooth:
  h_p95 -62.4%
  energy_p95 -90.3%
  ay_p95 -34.5%
  alpha_p95 47.39 -> 4.66

P3 radius:
  h_p95 -68.2%
  energy_p95 -93.4%
  ay_p95 -38.0%
  alpha_p95 47.39 -> 5.15
```

判断：

```text
C1/C2 对几何诱导 lateral slosh 是强正信号；
说明路径几何层确实比固定路径速度 retiming 更有潜力。
```

但该结论不能直接进入闭环仿真：

```text
Step 0 已显示 P2/P3 第一峰强相关 longitudinal ax；
当前 Step 2 第一版没有建模 ax / jerk / terminal timing；
因此必须补 C3 low-jerk timing proxy，再做完整 signed ax/ay rollout。
```

## 13. 2026-05-05 Step 2 第二版执行结果

已把 rollout/proxy 脚本扩展为 signed ax/ay 双通道：

```text
src/scout_apps/control/scout_local_planner/scripts/evaluate_anti_slosh_path_candidates.py
```

新增 timing 口径：

```text
constant:
  v_ref = 1.2 m/s

low_jerk:
  起点和终点各 1.0m smootherstep ramp
  v_floor = 0.15 m/s 用于 rollout dt
```

rollout 输入：

```text
eta_x:
  ax = (v_i^2 - v_{i-1}^2) / (2 ds)

eta_y:
  ay = v_i^2 * signed kappa_i

yaw excitation proxy:
  alpha = ax * kappa + v_i^2 * dkappa
```

输出：

```text
docs/Claude/分析数据/2026-05-05_step2_candidate_rollout_proxy_with_timing.csv
```

关键结果：

```text
P2 original constant:
  time=7.65s h_p95=0.009917 energy_p95=0.05346 ay_p95=2.835

P2 smooth constant:
  h_p95 -65.5%
  energy_p95 -92.9%
  ay_p95 -34.8%
  alpha_p95 39.85 -> 4.61

P2 radius constant:
  h_p95 -70.0%
  energy_p95 -94.4%
  ay_p95 -41.2%
  alpha_p95 39.85 -> 4.22

P3 original constant:
  time=7.73s h_p95=0.01074 energy_p95=0.05721 ay_p95=3.772

P3 smooth constant:
  h_p95 -62.4%
  energy_p95 -90.3%
  ay_p95 -34.0%
  alpha_p95 47.40 -> 4.67

P3 radius constant:
  h_p95 -68.2%
  energy_p95 -93.4%
  ay_p95 -37.9%
  alpha_p95 47.40 -> 5.16
```

但 C3 naive low_jerk timing 不通过：

```text
P2 original low_jerk:
  time=12.10s，约 +58%
  h_p95 -8.8%，energy_p95 -8.8%

P2 smooth low_jerk:
  time=12.08s，约 +58%
  h_p95 -65.2%，energy_p95 -93.2%

P3 original low_jerk:
  time=12.18s，约 +58%
  h_p95 基本不变
```

判断：

```text
C1/C2 几何候选通过离线 signed ax/ay rollout 的第一层筛选；
它们明显降低 kappa / dkappa / ay / alpha 和 predicted modal response。

当前 C3 low_jerk 只是一个过慢的起终点 ramp；
虽然能降低部分 ay 或 h_p95，但 time_proxy 远超 +15% 验收门槛；
不能进入闭环仿真。
```

下一步修正：

```text
不使用 naive 1m low_jerk ramp 作为最终 C3；
改成 matched-time timing：
  time_proxy <= original * 1.15
  只在起步、终端、Step0 peak lookback 窗口局部降低 jerk/ax
  不破坏 C1/C2 已获得的几何收益
```

当前状态：

```text
不改控制器；
不录新 bag；
继续做离线 timing candidate，直到 Step 2 满足 time_proxy / h / energy / eta_dot / ay / jerk 门槛。
```

## 14. 2026-05-05 Step 2C：matched-time timing 初筛

固定 `v_ref=1.2` 扫描不同 ramp 后得到：

```text
ramp=0.10m:
  time 增量小；
  但 energy / eta_dot 容易反向变差，说明短 ramp 激发纵向模态。

ramp=0.25m:
  time 约 +14%，满足 +15%；
  但 ax_max≈6.1~6.5，jerk_max≈104，局部脉冲过大。

ramp=0.50m / 0.75m:
  ax_max / jerk_max 明显下降；
  但 time 约 +29% / +43%，不满足 +15%。
```

因此尝试 matched-time proxy：

```text
使用 C1/C2 几何候选降低 kappa 后释放的横向 ay 余量；
把候选巡航速度提高到 v_ref=1.4m/s；
使用 ramp=0.50m；
约束：time <= +15%，ay_p95 不超过 original constant。
```

输出：

```text
docs/Claude/分析数据/2026-05-05_step2_candidate_rollout_proxy_v140_ramp050.csv
```

结果：

```text
P2 smooth:
  time +13.9%
  h_p95 -46.2%
  energy_p95 -83.4%
  eta_dot_p95 -74.0%
  ay_p95 -13.7%

P2 radius:
  time +13.5%
  h_p95 -50.6%
  energy_p95 -83.8%
  eta_dot_p95 -75.3%
  ay_p95 -20.7%

P3 smooth:
  time +13.0%
  h_p95 -44.0%
  energy_p95 -79.5%
  eta_dot_p95 -77.2%
  ay_p95 -10.1%

P3 radius:
  time +12.0%
  h_p95 -51.3%
  energy_p95 -85.7%
  eta_dot_p95 -80.0%
  ay_p95 -15.4%
```

共同风险：

```text
ax_max≈4.6 m/s^2
jerk_max≈48.6 m/s^3
```

判断：

```text
这是第一个同时满足 time_proxy、ay_p95、h_p95、energy_p95、eta_dot_p95 的离线组合；
说明“几何降激励 + matched-time timing”比固定几何 PROFILE_ENERGY 更接近可行方案。

但 ax_max / jerk_max 仍偏高；
因此还不能直接进入闭环仿真。
```

下一步：

```text
做小网格：
  v_ref = 1.35 / 1.40 / 1.45
  ramp = 0.45 / 0.50 / 0.60 / 0.65

目标：
  保持 time <= +15% 和 ay_p95 不升；
  尽量降低 ax_max / jerk_max；
  选择 smooth/radius 中更稳的一个进入闭环仿真。
```

## 15. 2026-05-05 Step 2D：timing 小网格筛选结果

新增离线筛选脚本：

```text
src/scout_apps/control/scout_local_planner/scripts/sweep_anti_slosh_timing_candidates.py
```

输出：

```text
docs/Claude/分析数据/2026-05-05_step2c_timing_candidate_sweep.csv
```

扫描范围：

```text
v_ref = 1.35 / 1.40 / 1.45
ramp = 0.45 / 0.50 / 0.60 / 0.65
```

pass 条件：

```text
time_delta <= +15%
h_p95_pred 下降
energy_p95_pred 下降
eta_dot_p95_pred 下降
ay_p95_pred 不升
```

结果：

```text
P2 smooth 最小 jerk pass:
  v=1.40 ramp=0.50
  time +13.9%
  h -46.2%
  energy -83.4%
  eta_dot -74.0%
  ay -13.8%
  ax_max=4.61
  jerk_max=48.6

P2 radius 最小 jerk pass:
  v=1.40 ramp=0.50
  time +13.5%
  h -50.6%
  energy -83.8%
  eta_dot -75.3%
  ay -20.7%
  ax_max=4.61
  jerk_max=48.6

P3 smooth 最小 jerk pass:
  v=1.40 ramp=0.50
  time +13.0%
  h -44.0%
  energy -79.5%
  eta_dot -77.2%
  ay -10.1%
  ax_max=4.60
  jerk_max=48.6

P3 radius 最小 jerk pass:
  v=1.45 ramp=0.60
  time +14.4%
  h -48.1%
  energy -83.8%
  eta_dot -83.0%
  ay -9.3%
  ax_max=4.15
  jerk_max=40.0
```

统一参数候选：

```text
v=1.40 ramp=0.50 可同时通过 P2/P3 smooth/radius；
其中 radius 的 slosh 降幅和 ay 降幅更稳。
```

下一步闭环前置检查：

```text
1. 确认 fixed_path runner 能否加载 /data/a/fixed_paths/candidates/*_radius.json；
2. 确认 v_ref=1.40 与 ramp=0.50 如何注入当前 PathHandler / launch 参数；
3. 若当前 runner 不支持 path/timing 注入，先补最小脚本入口，不直接改 MPC。
```

当前推荐第一闭环候选：

```text
P2/P3 radius path
v_ref = 1.40 m/s
ramp = 0.50 m
```

保留风险：

```text
ax_max≈4.6 m/s^2、jerk_max≈48.6 m/s^3；
闭环时必须检查 odom_ax、v_ref tracking、track_dist 和 solve_success_ratio；
如果 MPC tracking 放大 ax/jerk，则该路线回到离线设计层，不继续调控制器。
```

## 16. 2026-05-05 闭环 smoke 入口

现有固定路径 runner 已支持：

```text
PATH_FILE=/path/to/candidate.json
```

本轮只补了速度参数注入：

```text
slosh_experiment_sim.launch:
  vehicle_v_max arg

run_sim_fixed_path_bag.sh:
  VEHICLE_V_MAX env
```

映射关系：

```text
local_planner_ros:
  v_nominal = vehicle/v_max * 0.8

离线候选:
  v_ref = 1.40

闭环命令:
  VEHICLE_V_MAX = 1.75
```

第一闭环 smoke 只跑 radius：

```bash
PATH_ID=P2_s_curve_radius CONDITION=CUSTOM RUN_ID=geom01 START_DELAY=30 APPROACH_START_ENABLE=false \
PATH_FILE=/data/a/fixed_paths/candidates/P2_s_curve_radius.json VEHICLE_V_MAX=1.75 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh

PATH_ID=P3_mixed_radius CONDITION=CUSTOM RUN_ID=geom01 START_DELAY=30 APPROACH_START_ENABLE=false \
PATH_FILE=/data/a/fixed_paths/candidates/P3_mixed_radius.json VEHICLE_V_MAX=1.75 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

验收时额外关注：

```text
odom_ax_abs_p95 / odom_ax_max
jerk proxy
track_dist_p95
solve_success_ratio
tracking_time
/slosh/height 与 eta_dot
```

停止条件：

```text
如果 h/energy/eta_dot 未降，或 tracking_time > +15%，或 track_dist 明显恶化；
不继续录更多包；
回到离线 candidate/timing 层。
```

## 17. 2026-05-05 radius path smoke 结果修正

已录：

```text
P2:
  /data/a/slosh_bags/sim/20260505/20260505_P2_s_curve_radius_CUSTOM_rungeom01_133337.bag

P3:
  /data/a/slosh_bags/sim/20260505/20260505_P3_mixed_radius_CUSTOM_rungeom01_133631.bag
```

本轮配置：

```text
PATH_FILE = /data/a/fixed_paths/candidates/*_radius.json
CONDITION = CUSTOM
VEHICLE_V_MAX = 1.75

实际：
  v_des_eff_mean ≈ 1.40
```

P2 结果：

```text
h_rms            -47.2%
h_p95            -49.4%
eta_dot_rms      -38.3%
energy_rms       -46.6%
odom_ay_p95      -46.9%

tracking_time    +34.7%  FAIL
track_dist_p95   +72.9%  FAIL
```

P3 结果：

```text
h_rms            -16.6%
h_p95            -11.9%
eta_dot_rms      -18.2%
energy_rms       -16.8%
odom_ay_p95      -6.1%
track_dist_p95   -31.2%
tracking_time    +4.7%
solve_success    1.000
```

判断：

```text
P3 是有效正信号；
P2 不通过完整验收。
```

需要修正的前提：

```text
Step 2D 离线 sweep 之前用 v_ref=1.2 作为 baseline；
但历史 NOM bag 的 v_des_eff_mean≈2.37。

因此 “v=1.40 ramp=0.50 满足 time<=+15%” 是相对低速离线 baseline 的结论；
不是相对真实 NOM 的结论。
```

用真实速度口径重跑：

```text
baseline_v_ref=2.4
candidate v_ref=1.6/1.8/2.0/2.2/2.4
ramp=0.20/0.30/0.40/0.50

结果：
  0 / 40 pass_all
```

阶段结论：

```text
几何 radius path 能降低 slosh；
但目前只在明显降速时成立；
尚未证明它能在 tracking_time <= NOM * 1.15 内成立。
```

下一步若继续：

```text
只做速度口径校准，不改 MPC：
  P2 radius, VEHICLE_V_MAX=2.05 或 2.10

目标：
  P2 tracking_time <= 6.8s
  h / eta_dot / energy 仍下降
  track_dist 不继续恶化

如果 P2 仍失败：
  停止 radius path 闭环；
  将其写为“低速几何降激励有效，但无法满足时间约束”的负结果。
```

## 18. 2026-05-05 P2 radius v205 速度校准结果

已录：

```text
/data/a/slosh_bags/sim/20260505/20260505_P2_s_curve_radius_CUSTOM_rungeom_v205_134622.bag
```

目的：

```text
验证提高 VEHICLE_V_MAX 后，P2 radius 能否在 tracking_time <= NOM * 1.15 内保住降晃。
```

结果：

```text
tracking_time    +13.6%  PASS
h_rms            -27.4%  PASS
h_p95            -38.0%  PASS
h_max            +16.9%  FAIL
eta_dot_rms      +3.8%   FAIL
energy_rms       -24.9%  PASS
odom_ay_p95      -41.4%  PASS
track_dist_p95   +56.7%  FAIL
solve_success    0.993   OK
```

主峰诊断：

```text
height peak @ 11.53s = 0.008831
hint = longitudinal_timing + tracking_amplification
ax_p95 = 6.955
jerk_p95 = 946.876
track ≈ 1.900m
eta_dot = 0.06982
```

结论：

```text
P2 radius v205 不通过完整验收。
```

解释：

```text
低速 v175:
  防晃有效，但 tracking_time +34.7%，时间失败。

提速 v205:
  时间通过，但 h_max / eta_dot / track_dist 失败。

说明当前 radius path 存在结构性 trade-off；
不是继续扫 VEHICLE_V_MAX 可以稳定解决的问题。
```

决策：

```text
停止 radius path 同类速度扫；
不继续录 P2 radius v210/v220；
不把 C1/C2 radius smoothing 写成成功主方法。
```

论文/方案定位修正：

```text
trajectory geometry layer 仍是正确前移方向；
但当前低复杂度 C1/C2 只证明了“几何低激励有正信号”，没有形成通用通过方案。

如果继续走轨迹层，必须从简单 smoothing/radius inflation 升级为带约束的 trajectory optimization：
  同时约束 time、trackability、ax/jerk、terminal residual 和 modal response。
```

## 19. V2：约束式 anti-slosh reference optimization

### 19.1 为什么要升级

现有 C1/C2/C3 的核心缺陷：

```text
它们是启发式局部变形；
没有在同一个问题里同时约束 time、tracking、ax、jerk 和 slosh；
因此会出现“低速降晃但时间失败”或“提速达时但 eta_dot / h_max / track_dist 失败”。
```

这说明下一版不能继续靠参数扫补救，而应把问题写成离线约束优化：

```text
给定起点、终点、障碍/边界和 NOM 时间预算；
求一条可跟踪的低激励参考 trajectory；
再让 MPC 只做 constrained tracker。
```

### 19.2 优化变量

第一版只做离线，不进 ROS 控制器。

变量：

```text
几何：
  waypoint offsets δx_i, δy_i
  或 control points P_i

时间：
  v_i = v(s_i)
  或 Δt_i
```

不直接优化：

```text
MPC cost 权重；
cmd_vel 输出裁剪；
路径分类器；
在线学习参数。
```

### 19.3 目标函数

推荐目标函数：

```text
min
  w_E      * p95(E_slosh)
+ w_h      * p95(|h|)
+ w_deta   * rms(|eta_dot|)
+ w_ax     * p95(|ax|)
+ w_jerk   * p95(|jerk|)
+ w_track  * trackability_proxy
+ w_dev    * path_deviation_from_nominal
```

其中 slosh rollout 使用 signed linear modal ODE：

```text
eta_x_ddot + 2ζω_n eta_x_dot + ω_n^2 eta_x = -ax
eta_y_ddot + 2ζω_n eta_y_dot + ω_n^2 eta_y = -ay

ay = v^2 * kappa
alpha = ax * kappa + v^2 * dkappa
jerk = d ax / dt
```

### 19.4 硬约束

必须作为 hard gate，而不是只放进 soft cost：

```text
time <= NOM_time * 1.15
h_p95_pred <= NOM_h_p95
eta_dot_rms_pred <= NOM_eta_dot_rms
energy_rms_pred <= NOM_energy_rms
ay_p95_pred <= NOM_ay_p95
ax_p95_pred <= NOM_ax_p95 或 <= 实测安全上界
jerk_p95_pred <= NOM_jerk_p95 或 <= 实测安全上界
kappa_max <= vehicle_omega_max / v_min
dkappa_p95 不高于 NOM
path_deviation <= 允许边界
terminal |eta| / |eta_dot| 不高于 NOM
```

P2 额外 hard gate：

```text
trackability_proxy 不允许恶化；
否则会复现 v205 的 tracking_amplification。
```

### 19.5 trackability proxy

闭环失败表明只看几何 kappa/dkappa 不够，需要加入 MPC 可跟踪性代理：

```text
trackability_proxy =
  p95(|kappa|)
+ p95(|dkappa|)
+ p95(|alpha|)
+ p95(|ax|)
+ p95(|jerk|)
+ terminal_heading_change_penalty
+ large_curvature_near_goal_penalty
```

更严格版本：

```text
用简化 unicycle tracking rollout：
  v_cmd follows v_ref with a_max / j_max
  omega_cmd = v * kappa clipped by omega_max / alpha_max
  积分得到 x_exec, y_exec
  计算 predicted track_dist_p95
```

V2 第一版建议先实现严格版本，因为 P2 已经出现 `track_dist_p95 +56.7%`。

### 19.6 求解方式

不需要一开始上复杂 NLP。推荐两阶段离线搜索：

```text
Stage A: candidate family generation
  生成有限个几何候选：
    original
    smooth
    radius
    waypoint-offset variants
    endpoint / terminal segment variants

Stage B: constrained timing search
  对每条几何候选搜索 v(s):
    time matched to NOM * 1.15
    jerk-limited S-curve timing
    local slowdowns only around high-risk windows
    cruise speed 自动补偿时间
```

筛选方式：

```text
先 hard constraints；
再按 objective 排序；
只保留 top 1~2 个进入闭环。
```

暂不做：

```text
CasADi / IPOPT 全量优化；
在线 MPC 内优化 slosh；
动态路径分类器。
```

### 19.7 Step V2-0：修正离线基线

先修正当前离线评估口径：

```text
baseline_time 和 baseline_speed 必须来自真实 NOM bag；
不能再用 v_ref=1.2 这种人为低速 baseline。
```

输入：

```text
P2 NOM x3
P3 NOM x3
```

输出：

```text
每条路径的 baseline:
  tracking_time_mean
  h_p95_mean
  h_max_mean
  eta_dot_rms_mean
  energy_rms_mean
  ay_p95_mean
  ax_p95 / jerk proxy
  track_dist_p95_mean
  v_des_eff_mean
```

V2 所有离线 pass/fail 都必须相对这些真实 baseline。

### 19.8 Step V2-1：实现 constrained offline optimizer

新增脚本建议：

```text
src/scout_apps/control/scout_local_planner/scripts/optimize_anti_slosh_reference.py
```

输入：

```text
--path-id P2_s_curve / P3_mixed
--nom-metrics docs/Claude/分析数据/...
--path-file /data/a/fixed_paths/sim/*.json
--output-dir /data/a/fixed_paths/optimized
```

输出：

```text
/data/a/fixed_paths/optimized/P2_s_curve_opt_v2.json
/data/a/fixed_paths/optimized/P3_mixed_opt_v2.json
docs/Claude/分析数据/2026-xx-xx_v2_reference_candidates.csv
```

候选 JSON 至少包含：

```text
poses
metadata:
  method
  expected_time
  expected_h_p95_delta
  expected_eta_dot_delta
  expected_energy_delta
  expected_trackability_proxy
  v_profile_summary
```

### 19.9 Step V2-2：离线验收

进入 ROS 闭环前必须满足：

```text
P2:
  time_proxy <= NOM * 1.15
  h_p95_pred < NOM
  h_max_pred <= NOM
  eta_dot_rms_pred <= NOM
  energy_rms_pred <= NOM
  ay_p95_pred <= NOM
  trackability_proxy <= NOM

P3:
  同上；
  额外要求 eta_x / ax peak 不恶化。
```

如果 P2 不通过：

```text
不跑 P3；
不录 bag；
直接判定 V2 候选不可用。
```

### 19.10 Step V2-3：闭环仿真

最小闭环实验：

```text
P2_OPT_V2 x2
P3_OPT_V2 x2
matched-time slow baseline x1
```

验收：

```text
tracking_time <= NOM * 1.15
h_rms / h_p95 / h_max 下降
eta_dot_rms 下降
energy_rms 下降
ay_p95 不升
track_dist_p95 不升
solve_success_ratio >= 0.97
```

任一失败：

```text
不继续调 VEHICLE_V_MAX；
不改 MPC；
回到 V2 offline optimizer。
```

### 19.11 当前论文定位

在 V2 成功前，论文不能写：

```text
本文提出的轨迹几何层方法已成功抑制晃动。
```

当前只能写：

```text
启发式几何低激励参考在 P3 上给出正信号，但在 P2 上暴露出 time-tracking-slosh trade-off。
该结果表明防晃参考生成必须以约束优化形式同时处理可跟踪性、时间和模态响应。
```

若 V2 成功，论文主贡献可改为：

```text
Constrained low-excitation reference generation for slosh-aware MPC tracking.
```

## 20. 2026-05-05 V2-0 / V2-1 第一版结果

已实现：

```text
src/scout_apps/control/scout_local_planner/scripts/optimize_anti_slosh_reference.py
```

输出：

```text
docs/Claude/分析数据/2026-05-05_v2_nom_baseline.csv
docs/Claude/分析数据/2026-05-05_v2_reference_optimization_sweep_tracking_window.csv
```

真实 NOM baseline：

```text
P2_s_curve:
  tracking_time=5.90s
  h_p95=0.006105
  h_max=0.007554
  eta_dot_rms=0.014333
  ay_p95=2.516
  track_dist_p95=1.245
  v_des_eff_mean=2.374

P3_mixed:
  tracking_time=6.02s
  h_p95=0.005335
  h_max=0.008507
  eta_dot_rms=0.014881
  ay_p95=1.463
  track_dist_p95=0.736
  v_des_eff_mean=2.366
```

V2 第一版搜索：

```text
candidate family:
  original
  smooth
  radius

search:
  cruise_speed = 1.6 / 1.8 / 2.0 / 2.2 / 2.4
  ay_ratio = 0.6 / 0.8 / 1.0
  accel_limit = 1.0 / 1.5 / 2.0
```

结果：

```text
pass_all = 0 / 270
```

分解：

```text
P2:
  pass_time = 124 / 135
  pass_h = 57 / 135
  pass_eta_dot = 0 / 135
  pass_ay = 111 / 135
  time+h+ay = 50 / 135

P3:
  pass_time = 24 / 135
  pass_h = 58 / 135
  pass_eta_dot = 0 / 135
  pass_ay = 90 / 135
  time+h+ay = 0 / 135
```

代表性 P2 边界：

```text
P2 radius v=1.8 ay_ratio=0.6 accel=1.0:
  time +0.2%
  h_p95 -49.4%
  h_max -38.1%
  ay -41.9%
  eta_dot +9.5%  -> FAIL
```

代表性 P3 边界：

```text
P3 radius v=1.6 ay_ratio=1.0 accel=1.0:
  time +14.8%
  h_p95 -32.8%
  h_max -42.3%
  ay +4.6%       -> FAIL
  eta_dot +63.9% -> FAIL
```

重要口径修正：

```text
optimizer 默认不再强制起终点速度为 0；
因为当前闭环验收统计的是 TRACKING + terminal NONE 主段；
终端 rest-to-rest residual 后续单独评估。
```

当前 blocker：

```text
eta_dot rollout fidelity 不足。
```

证据：

```text
P3 radius 闭环 smoke 实测：
  eta_dot_rms -18.2%

V2 离线 rollout 对相近候选：
  eta_dot_rms 预测为明显上升。
```

因此不能继续把当前 rollout 的 eta_dot 作为唯一离线 hard gate。

下一步决策点：

```text
A. 修正 eta_dot rollout fidelity
   先解释为什么离线 eta_dot 与闭环 bag 符号相反；
   可能需要用 odom replay / closed-loop execution profile 代替 pure reference rollout。

B. 调整 gate 层级
   离线 hard gate 只保留 time / h / h_max / ay / geometry；
   eta_dot 作为 closed-loop hard gate；
   只允许极少量 top candidate 进仿真。

C. 停止模型优化路线
   转向真液面视觉或更高保真 slosh model。
```

当前推荐：

```text
先做 A。

理由：
  如果 eta_dot rollout 方向错，任何依赖该模型的 constrained optimizer 都会误筛；
  直接把 eta_dot 降级进闭环会重新变成“录 bag 赌结果”。
```

## 21. 2026-05-05 V2-2 eta_dot rollout 口径修正

执行选项 A 后，`eta_dot rollout fidelity 不足` 的主要原因被定位为时间对齐问题，而不是 slosh ODE 本身失效。

诊断命令输出：

```text
docs/Claude/分析数据/2026-05-05_eta_dot_fidelity_shift_m005.csv
docs/Claude/分析数据/2026-05-05_eta_dot_fidelity_shift_000.csv
docs/Claude/分析数据/2026-05-05_eta_dot_fidelity_shift_p005.csv
docs/Claude/分析数据/2026-05-05_eta_dot_fidelity_default_shift.csv
```

关键结果：

```text
easp_input_shift = 0.00s:
  profile_energy_rollout = 0 / 8 PASS
  eta_dot_corr_mean = 0.117
  eta_dot_best_corr_mean = 0.951 @ +0.05s

easp_input_shift = +0.05s:
  profile_energy_rollout = 8 / 8 PASS
  h_p95_err_mean = 0.022
  peak_dt_mean = 0.000s
  eta_dot_corr_mean = 0.971
  risk_recall_mean = 1.000
```

结论：

```text
/slosh/ax_est 与 /slosh/ay_est 的 bag 时间戳相对 /slosh/state 需要 +0.05s 对齐。
之前 eta_dot_corr 低不是模型主失配，而是输入/状态时间戳错位。
```

代码口径修正：

```text
offline_pmg_replay.py:
  --easp-input-shift 默认值从 0.00 改为 +0.05

optimize_anti_slosh_reference.py:
  time / ay 仍相对真实 NOM baseline 约束
  h / h_max / eta_dot 改为相对同一 rollout 模型下的 original-path baseline 约束
```

修正后的 V2 搜索输出：

```text
docs/Claude/分析数据/2026-05-05_v2_nom_baseline_model_gate.csv
docs/Claude/分析数据/2026-05-05_v2_reference_optimization_model_gate.csv
```

结果：

```text
pass_all = 93 / 270

P2_s_curve:
  pass_all = 91 / 135
  original = 11
  smooth   = 35
  radius   = 45

P3_mixed:
  pass_all = 2 / 135
  original = 0
  smooth   = 0
  radius   = 2
```

P3 可进入闭环 smoke 的两个离线候选：

```text
P3 radius v=2.4 ay_ratio=0.8 accel=2.0:
  time +13.8%
  h_p95 -69.0%
  eta_dot -61.6%
  ay -10.5%

P3 radius v=2.2 ay_ratio=0.8 accel=2.0:
  time +14.3%
  h_p95 -69.8%
  eta_dot -61.4%
  ay -10.5%
```

当前判断：

```text
V2 constrained reference generation 重新进入可验证状态。
但这些是离线模型内通过，不等价于闭环通过。
下一步只应选 1 个 P3 radius top candidate + 1 个 P2 radius sanity candidate 做闭环 smoke。
若闭环 smoke 的 eta_dot / track_dist 再次失败，说明问题转移到 MPC execution fidelity，而不是离线筛选。
```

## 22. 2026-05-05 V2-3 闭环 smoke 入口

为了避免把 V2 继续混入旧的 MPC slosh cost / output cap，新增独立运行条件：

```text
CONDITION=PROFILE_REF_V2
```

该条件只启用 PathHandler 的参考速度剖面：

```text
Q_slosh = 0
Q_slosh_eta_dot = 0
risk_scheduler = false
input_shaping = false
energy_profile_enable = true
energy_profile_omega_max = 999
energy_profile_alpha_max = 999
energy_profile_min_v = 0
```

离线 candidate 到闭环参数的映射：

```text
cruise_speed = vehicle_v_max * 0.8
ay_limit     = ENERGY_PROFILE_LAT_ACCEL
accel_limit  = ENERGY_PROFILE_AX_MAX
decel_limit  = ENERGY_PROFILE_DECEL_MAX * 0.8
```

注意：

```text
C++ PathHandler 内部对 decel pass 使用 decel_safety_factor=0.8。
因此若离线 candidate 的 decel_limit=2.0，运行脚本里 ENERGY_PROFILE_DECEL_MAX 应设为 2.5。
```

第一批只跑两个 smoke：

```text
P3 top:
  PATH_FILE=/data/a/fixed_paths/candidates/P3_mixed_radius.json
  VEHICLE_V_MAX=3.0
  ENERGY_PROFILE_LAT_ACCEL=1.1701333333333332
  ENERGY_PROFILE_AX_MAX=2.0
  ENERGY_PROFILE_DECEL_MAX=2.5

P2 sanity:
  PATH_FILE=/data/a/fixed_paths/candidates/P2_s_curve_radius.json
  VEHICLE_V_MAX=3.0
  ENERGY_PROFILE_LAT_ACCEL=2.5156666666666667
  ENERGY_PROFILE_AX_MAX=1.0
  ENERGY_PROFILE_DECEL_MAX=1.25
```

闭环 smoke 判定：

```text
不是正式验收，不做多包统计；
只判断 V2 candidate 能否在真实 MPC 执行层保持：
  tracking_time <= NOM * 1.15
  h_p95 / eta_dot 不反向
  ay_p95 不升
  track_dist_p95 不显著恶化
```

## 23. 2026-05-05 V2-4 闭环 smoke 失败原因

实测 bag：

```text
P3:
  /data/a/slosh_bags/sim/20260505/20260505_P3_mixed_radius_PROFILE_REF_V2_runsmoke01_143137.bag

P2:
  /data/a/slosh_bags/sim/20260505/20260505_P2_s_curve_radius_PROFILE_REF_V2_runsmoke01_143352.bag
```

结果：

```text
P2:
  tracking_time -5.1%
  h_rms +10.5%
  h_p95 +3.9%
  h_max +19.7%
  eta_dot -2.8%
  energy +9.6%
  ay_p95 +1.9%
  track_p95 +38.6%
  -> FAIL

P3:
  tracking_time -11.1%
  h_rms +4.3%
  h_p95 +12.0%
  h_max -8.5%
  eta_dot +22.5%
  energy +6.0%
  ay_p95 +18.3%
  track_p95 -24.2%
  -> FAIL
```

配置确认：

```text
PROFILE_REF_V2 参数已正确注入；
Q_slosh=0，Q_slosh_eta_dot=0，risk=false，input_shaping=false；
energy_profile_enable=true；
P3 ay_limit=1.1701，ax_limit=2.0；
P2 ay_limit=2.5157，ax_limit=1.0。
```

失败机理：

```text
不是 V2 参数没有生效，而是执行层没有按离线候选的 ax / ay / jerk 假设执行。
```

证据：

```text
P3 peak1:
  ax_p95=11.086
  jerk_p95=1125.685

P3 peak2:
  ax_p95=5.294
  ay_p95=1.652
  jerk_p95=571.856

P2 peak2:
  ax_p95=7.896
  ay_p95=2.556
  jerk_p95=930.461
  track=1.776
```

这说明：

```text
离线 constrained reference generation 仍然有模型内候选；
但当前 MPC tracking 层会把低激励参考重新转化成高 ax / jerk 执行。
只继续调 path_handler profile 参数会变成盲调。
```

下一步方向：

```text
必须从“生成更好的 v(s)”转到“reference execution fidelity”诊断。
最小下一步不是再录 PROFILE_REF_V2，而是加/用诊断输出：
  ref v_path / v_ref / kappa / implied ax / implied ay
  odom ax / odom ay / jerk
  ref-vs-odom execution error

若 ref 本身已低激励但 odom 高激励：
  问题在 MPC tracking / acceleration constraint / Q_v / R_a / terminal interaction。

若 ref 本身仍高激励：
  问题在 PathHandler profile 生成与离线 optimizer 不一致。
```

## 24. 2026-05-05 V2-5 诊断输出已补

已增加 reference execution fidelity instrumentation：

```text
/reference/v_ref
/reference/v_path
/reference/kappa
/reference/s
/reference/implied_ax
/reference/implied_ay
/reference/implied_jerk
/reference/implied_ax_abs_p95
/reference/implied_ay_abs_p95
/reference/implied_jerk_abs_p95
```

这些 topic 的含义：

```text
v_ref / v_path / kappa / s:
  MPC 当前周期第一个 reference point。

implied_ax / implied_ay / implied_jerk:
  按 MPC reference horizon 和 dt 反推的第一拍参考激励。

*_abs_p95:
  当前 MPC reference horizon 内的绝对值 p95。
```

分析脚本新增执行一致性字段：

```text
ref_implied_ax_abs_p95
ref_implied_ay_abs_p95
ref_implied_jerk_abs_p95
odom_ax_abs_p95
odom_jerk_abs_p95
exec_ax_to_ref_p95_ratio
exec_ay_to_ref_p95_ratio
exec_jerk_to_ref_p95_ratio
```

下一步只跑 P3 fidelity 包：

```bash
PATH_ID=P3_mixed_radius CONDITION=PROFILE_REF_V2 RUN_ID=fidelity01 START_DELAY=30 APPROACH_START_ENABLE=false \
PATH_FILE=/data/a/fixed_paths/candidates/P3_mixed_radius.json \
VEHICLE_V_MAX=3.0 ENERGY_PROFILE_LAT_ACCEL=1.1701333333333332 ENERGY_PROFILE_AX_MAX=2.0 ENERGY_PROFILE_DECEL_MAX=2.5 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

判定逻辑：

```text
若 ref_implied_ax/ay/jerk 已高：
  PathHandler C++ profile 与离线 optimizer 不一致，修 reference generation。

若 ref_implied_ax/ay/jerk 低但 odom_ax/ay/jerk 高：
  MPC execution fidelity 失败，检查 Q_v / R_a / acceleration constraints / terminal interaction。
```

## 25. 2026-05-05 V2-6 C++ reference 与离线 optimizer 对齐修正

`fidelity01` 结果：

```text
ref_implied_ax_abs_p95=11.319
ref_implied_ay_abs_p95=2.000
odom_ax_abs_p95=7.031
odom_ay_abs_p95=1.654
```

判断：

```text
reference 本身已高激励；
不是 MPC 放大了低激励 reference。
```

定位到的 C++/离线不一致：

```text
PathHandler::getReferencePoints()
  geometry 用 s_progress + lookahead_distance；
  speed profile 用 s_progress；

V2 离线 optimizer
  speed / kappa / ay / ax 都按同一 s 序列计算。
```

因此 C++ 中会出现：

```text
低曲率段速度 + lookahead 高曲率几何
=> reference ay 超过 V2 ay_limit。
```

已修正：

```text
energy_profile_enable=true 时：
  speed profile 采样弧长 = s_geom_global；
  v_curve_cap 使用 energy_profile_lat_accel；
  goal_capture_min_speed 后重新应用 curve/v_plan/v_exec cap。
```

下一包只跑：

```bash
PATH_ID=P3_mixed_radius CONDITION=PROFILE_REF_V2 RUN_ID=fidelity02 START_DELAY=30 APPROACH_START_ENABLE=false \
PATH_FILE=/data/a/fixed_paths/candidates/P3_mixed_radius.json \
VEHICLE_V_MAX=3.0 ENERGY_PROFILE_LAT_ACCEL=1.1701333333333332 ENERGY_PROFILE_AX_MAX=2.0 ENERGY_PROFILE_DECEL_MAX=2.5 \
rosrun scout_local_planner run_sim_fixed_path_bag.sh
```

先看：

```text
ref_implied_ay_abs_p95 是否从 2.0 降到接近 1.17；
ref_implied_ax_abs_p95 是否从 11.3 显著下降。
```
