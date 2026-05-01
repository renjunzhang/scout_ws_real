# 2026-05-01 Anti-Slosh 轨迹几何层重设计方案

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
