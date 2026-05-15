# 2026-05-15 terminal 与 ax 脉冲优先治理方案

## 一句话

下一步先别继续堆很多新的 cost 项，优先把 terminal 过渡和纵向 `ax` 脉冲处理干净。

## 为什么

2026-05-15 实物重复实验说明：

```text
slosh cost 已经能影响车辆行为，尤其 ay 会变；
但 RGB 真实液面没有稳定下降；
/slosh/height 和 RGB 排序一致性仍然不够好；
terminal 附近和 ax 脉冲很可能把降晃效果盖掉。
```

所以现在的主要矛盾不是“再加多几个 cost 项”，而是：

```text
车到终点附近不能突然刹、突然切状态、突然调头；
正常跟踪段也不能出现明显纵向加减速脉冲。
```

## 优先级

### 1. 先修 terminal 过渡

目标：

```text
TRACKING -> terminal/capture/recovery 的切换要平滑；
位置到达后先稳住线速度；
yaw 对齐不要阻止停车；
terminal recovery 不要制造新的 ax/omega 脉冲。
```

重点看：

```text
terminal 前 2-3 秒 cmd_v 是否突降；
odom_v 是否冲过终点；
ax peak / ax p95 / jerk-like 指标是否突然变大；
RGB 液面是否在 terminal 附近被重新激发。
```

### 2. 再治理 ax 脉冲

优先用结构方法，不先加一堆新 cost：

```text
参考速度提前平滑降下来；
PathHandler 生成 v_ref 时考虑制动距离；
MPC 保持合理加速度硬约束；
cmd_v 输出加 rate limit；
必要时再加轻量 jerk-like 约束。
```

先把小车物理上“不容易猛加猛刹”做好。

当前先落地最小结构改动：

```text
对执行层 v_des_cmd 增加统一变化率限制：
  v_des_rate_limit_enable=true
  v_des_accel_limit=0.60 m/s²
  v_des_decel_limit=0.80 m/s²

作用位置：
  risk / terminal_slowdown / feasibility / curvature cap 之后；
  PathHandler getReferencePoints 之前。

含义：
  不管 slosh_speed_governor 是否开启，进入 horizon 的 v_des 上限都不能一帧突跳。
```

新增诊断话题：

```text
/reference/v_des_raw
/reference/v_des_target
/reference/v_des_eff
/reference/v_des_rate_limited
```

下一轮 bag 用这些话题判断 ax 脉冲来源：

```text
raw -> target 突降：上游限速/terminal/curvature 在压速度；
target -> eff 被削：v_des rate limiter 正在平滑突变；
eff 平滑但 odom_ax 仍大：问题在 MPC/底盘执行或 cmd_vel 输出侧。
```

### 3. 最后再评估 slosh cost

等 terminal 和 ax 脉冲干净以后，再做：

```text
C: SMOOTH_ONLY
D: MODAL_ONLY，也就是当前 eta/eta_dot slosh cost
```

如果这时 D 比 C 稳定降低 RGB，说明之前是 terminal/ax 脉冲盖住了 slosh cost 效果。

如果仍然不降，再考虑加入显式 `ax/ay` excitation cost。

## 暂时不做

```text
不急着加 Q_ax / Q_ay / Q_da / Q_domega 一整套新 cost；
不靠 Q_slosh=100/1000 制造效果；
不靠 /slosh/height 下降证明真实液面下降；
不降低 R_a/R_omega 来让 slosh 项看起来更强。
```

## 主评价窗口

有效性统计继续优先使用：

```text
TRACKING start -> 第一次 terminal/capture
```

full bag 只做补充，因为 full bag 会混入 terminal recovery、capture stop、调头等终点行为。

## 减法后的 terminal 主线

先把实物主实验收成一条清楚链路：

```text
PathHandler 末端速度剖面
  -> MPC 正常跟踪
  -> terminal_slowdown 保护层（暂保留，等 v_ref_horizon 证据）
  -> CAPTURE_BRAKE 限速捕获
  -> REACHED
```

默认不再把 `terminal_recovery` 放进主实验路径：

```text
terminal_recovery_enable=false
```

原因：

```text
20260515 的 23 个旧 bag 里 recovery 实际没有进入；
真正污染 terminal 的是 capture/goal-stop 附近的速度和 ax 脉冲；
继续让 ALIGN/APPROACH/FINAL_YAW 默认开着，会让终点机制更难解释。
```

`terminal_recovery` 暂时只保留为手动兜底：

```text
如果实物出现明显冲过终点后无法停稳、必须调头回点，
再显式加 terminal_recovery_enable:=true 单独验证。
```

## 期望结果

先得到一个更干净的实物基线：

```text
终点不过冲；
terminal 切换不激发液面；
ax/jerk 峰值明显下降；
tracking 仍能完成；
RGB 曲线不再被终点段污染。
```

做到这个以后，再判断 slosh cost 是否真的有额外抑晃收益。
