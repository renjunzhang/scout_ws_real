# PROFILE_ENERGY 边界结论与后续方向

日期：2026-05-01

## 1. 一句话结论

`PROFILE_ENERGY` 证明了“参考生成层比 MPC 内部 slosh cost 更有信号”，但没有通过完整验收。

它可以降低 `/slosh/height`、modal energy 和 `eta_dot`，但在当前给定几何路径 + MPC tracking 架构下，无法同时满足：

```text
tracking_time <= NOM * 1.15
h_rms / h_p95 / energy / eta_dot 全下降
ay_p95 不升
tracking 不明显恶化
```

因此停止继续录制同类 `PROFILE_ENERGY` bag，不再做参数扫。

## 2. 关键结果

### 2.1 geometry-only baseline

`PROFILE_ENERGY_GEO_FAST`：

```text
tracking +20.3%
h_rms    -11.0%
h_p95    +0.3%
energy   -11.5%
eta_dot  -18.7%
ay_p95   -5.1%
```

结论：

```text
几何层能降低一部分 rms/energy/eta_dot，
但 h_p95 不降，且 tracking_time 已超过 +15%。
```

### 2.2 rollout correction

`PROFILE_ENERGY` time-budget 修正后 run05/run06：

```text
tracking +23.3%
h_rms    -20.4%
h_p95    -8.7%
energy   -20.7%
eta_dot  -25.6%
ay_p95   -8.7%
```

结论：

```text
rollout correction 相对 GEO_FAST 有明确增益：
  h_p95 从 +0.3% 改到 -8.7%
  energy / eta_dot 进一步下降

但总时间仍失败。
```

### 2.3 轻几何 + rollout

`PROFILE_ENERGY` run07/run08：

```text
tracking  +16.5%
h_rms     -25.7%
h_p95     -8.0%
energy    -25.1%
eta_dot   -17.6%
ay_p95    +5.4%
track_p95 +34.4%
```

结论：

```text
轻几何把 tracking_time 拉近到 +15% 边界，
但 ay_p95 和 tracking_p95 明显恶化。
```

这说明当前方法已经碰到三方权衡：

```text
slosh 降低
tracking_time
路径跟踪 / ay 执行质量
```

不是继续调阈值可以单调解决的问题。

## 3. 为什么停止 PROFILE_ENERGY

按 `2026-04-29_PROFILE_ENERGY速度剖面改进方案.md` 的停止条件，以下条件已触发：

```text
P2 tracking_time > +15%
ay_p95 不满足“不升”
tracking_p95 明显恶化
需要超过 2 轮参数扫才接近通过
```

继续调参的风险：

```text
更慢的 profile 可以继续降 slosh，但会失去“无显著减速”意义；
更快的 profile 可以接近时间门槛，但 ay/tracking 会恶化；
单纯调整窗口/阈值无法改变这个结构性权衡。
```

## 4. 论文中如何保留价值

`PROFILE_ENERGY` 不应写成成功主方法，但可以作为高价值消融：

```text
Reference-first profile-layer 方法能比 MPC 内部 slosh cost 更直接影响 slosh 指标；
signed slosh rollout correction 相对 geometry-only profile 有明确增益；
但在给定几何路径和当前 MPC tracking 执行层下，速度剖面修正会引入 tracking/ay 代价。
```

建议写法：

```text
The rollout-corrected speed profile reduces model-estimated slosh indicators,
but the closed-loop validation reveals a coupling between slosh reduction,
tracking time, and lateral acceleration execution. This motivates moving the
anti-slosh design further upstream to trajectory/path generation rather than
only modifying the path-speed profile.
```

不能写：

```text
PROFILE_ENERGY 成功抑制了晃动。
MPC 代价函数主动降低了液体晃动。
真实液面高度已经被证明降低。
```

## 5. 后续方向

下一步不继续 speed-profile 参数扫。

应转向更上游的轨迹/路径几何层：

```text
给定目标点和障碍约束，
先生成低 slosh 激励的几何轨迹 / 时序轨迹，
再让 MPC 只负责跟踪。
```

最小可证伪路径：

```text
Step 0: 不改控制器，先做离线路径几何审查
  比较 NOM/P2/P3 的 kappa、dkappa、ax_ref、ay_ref、jerk_ref 与 slosh 峰值窗口。

Step 1: 离线生成 anti-slosh trajectory candidate
  不只修改 v(s)，而是允许改变路径几何或 waypoint timing。

Step 2: 用同一 slosh rollout 验证 candidate 是否同时降低：
  h_p95 / energy / eta_dot / ay_p95 / tracking_time proxy。

Step 3: 只有离线通过后，才进入闭环仿真。
```

如果论文时间不足，则更现实的论文主线应改为：

```text
Slosh-aware MPC architecture and failure analysis for liquid-carrying mobile robots
```

而不是宣称一个已经闭环通过的 anti-slosh controller。
