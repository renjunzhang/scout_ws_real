# 2026-05-07 Online GeoRef 当前阶段性总结

## 1. 当前主线

当前 anti-slosh 主线已经收束为：

```text
MBF 全局路径
  -> anti_slosh_path_post_processor 在线几何后处理
  -> /scout/global_path_anti_slosh
  -> 普通 MPC tracking
  -> /cmd_vel
```

核心判断：

```text
防晃逻辑放在 MPC 前端的路径几何参考生成层；
MPC 不再承担主动消晃决策，只负责在车辆约束下跟踪低激励几何参考。
```

当前成功口径中，MPC 内部防晃项全部关闭：

```text
Q_slosh=0
Q_slosh_eta_dot=0
enable_slosh_box_constraint=false
risk_scheduler_enable=false
energy_profile_enable=false
input_shaping_enable=false
slosh_speed_governor_enable=false
```

因此论文不能写成：

```text
MPC 代价函数主动抑制了液体晃动。
```

应写成：

```text
本文将防晃逻辑前移到低激励几何参考生成层，并使用普通 MPC 跟踪该参考。
```

## 2. 防晃逻辑如何实现

`anti_slosh_path_post_processor.py` 订阅 MBF 生成的 `/scout/global_path`，生成多个几何候选：

```text
original
mild smoothing
medium smoothing
strong smoothing
```

候选路径通过以下指标筛选：

```text
max_drift
length_ratio
endpoint_error
min_segment_length
path direction
max_candidate_level
collision check，实物/maze 使用 global costmap inflation
predicted ay ratio
```

选择逻辑不是“越直越好”，而是在不明显绕路、不撞障碍、不偏离过大的前提下，降低：

```text
kappa
dkappa
预测横向激励 ay ≈ v²·kappa
路径突变造成的执行激励
```

如果没有候选通过 gate，则直接 fallback 到 original。`selected=original` 的包不能作为 GeoRef 有效样本。

## 3. 已有仿真正结果

主验证场景：

```text
SIM_ENV=open
open_user_goal:
  x=-3.1570560932159424
  y=-2.897411346435547
  qz=-0.978164583074326
  qw=0.2078317791364693
```

`GEOREF_TUNED` 相对 `RAW_TUNED` 三包均值：

```text
active_s       +5.5%
h_rms          -18.7%
h_p95          -18.6%
h_max          -6.6%
eta_dot_rms    -11.5%
energy_rms     -18.2%
ay_p95         -38.5%
ax_p95         -18.5%
alpha_p95      -13.7%
ref_ay_p95     -42.0%
ref_ax_p95     -91.4%
ref_jerk_p95   -80.5%
```

阶段性判断：

```text
在 open_user_goal 上，Online GeoRef 在 active time +15% 门槛内，
同时降低 /slosh/height、eta_dot、modal energy 和主要执行激励。
```

## 4. Baseline 排除项

### RAW_SLOW_MATCHED

目的：排除“只是慢了”。

结果：

```text
SLOW vs RAW:
  active_s       +3.3%
  h_p95          -1.7%
  h_max          +22.0%
  eta_dot_rms    +40.6%
```

结论：

```text
简单降速不能解释 GeoRef 收益。
慢速 baseline 甚至会让 h_max 和 eta_dot 变差。
```

### GEOREF_ORIGINAL

目的：排除“只是换 topic / post-processor chain”。

修正后语义：

```text
max_candidate_level=original 时，/scout/global_path_anti_slosh 直接转发原始 MBF path；
不再发布重采样 original。
```

结果：

```text
GEOREF vs ORIGINAL_FIXED:
  h_rms          -8.8%
  h_p95          -18.1%
  h_max          -19.1%
  eta_dot_rms    -36.3%
  energy_rms     -12.7%
```

结论：

```text
GeoRef 收益来自 geometry smoothing candidate selection，
不是来自 topic chain、latching 或 original fallback。
```

## 5. 05-07 负结果：Reference-Constrained MPC 不继续

曾尝试的增强路线：

```text
Online GeoRef
  -> 发布 v/a/jerk reference budget
  -> MPC 将 reference budget 作为时变硬边界
```

三包结果相对 `GEOREF_TUNED`：

```text
tracking_time   +8.41%
h_rms           +13.73%
h_p95           +6.29%
h_max           +10.30%
eta_dot_rms     +28.85%
energy_rms      +14.90%
odom_ay_p95     -9.09%
```

判断：

```text
v/a/jerk hard-bound 确实降低了部分执行侧 ay 指标，
但 /slosh/height、eta_dot、modal energy 相对普通 GeoRef + MPC 变差。
```

因此：

```text
GEOREF_CONSTRAINED 不作为主线；
不进入 slack 版本；
不作为 proposed method；
只作为 failure analysis 保留。
```

工程处理：

```text
reference-constrained MPC 实现和运行入口已外科撤回；
保留 diagnose_georef_budget_gap.py 作为诊断工具；
保留方案文档和日志作为负结果证据。
```

## 6. 旧 MPC 防晃路线的定位

以下路线已降级为消融或负结果：

```text
Q_slosh / Q_slosh_eta_dot / terminal slosh cost
Q_modal_energy / Q_ay_pred
risk scheduler
OUTPUT_GUARD
PMG lateral / longitudinal / combined
PROFILE_ENERGY speed-only profile
PROFILE_REF_V2 fixed-geometry speed/reference correction
reference-constrained MPC v/a/jerk hard-bound
```

共同问题：

```text
把防晃放在 MPC soft cost 或 cmd_vel 后处理层，会与 tracking、执行延迟、底盘动力学补偿产生冲突；
只改速度或输出裁剪容易把能量转移到 eta_dot / modal energy；
固定几何路径下可行空间有限，单纯调 MPC cost 很难稳定跨路径压低 /slosh/height。
```

## 7. 当前边界

已成立：

```text
open_user_goal 仿真三包均值正结果；
matched slow baseline 不能解释收益；
original-only baseline 不能解释收益；
reference-constrained MPC 增强路线已被负结果排除。
```

未成立：

```text
任意目标泛化
maze / 窄通道安全验证
实物真实液面验证
RealSense 真液面与 /slosh/height 一致性
```

重要边界：

```text
当前仿真结论基于 /slosh/height 作为液面晃动观测量；
如果实物视觉液面与 /slosh/height 改善方向不一致，不能声明真实液面晃动被抑制。
```

## 8. 实物验证口径

下一步优先做开阔场地实物验证：

```text
RAW_REAL:
  /scout/global_path -> 普通 MPC

GEOREF_REAL:
  /scout/global_path -> anti_slosh_path_post_processor
  -> /scout/global_path_anti_slosh -> 普通 MPC
```

两组必须保持：

```text
同起点
同终点
同地图
同定位
同 MPC 参数
Q_slosh=0
Q_slosh_eta_dot=0
所有旧 anti-slosh 控制项关闭
```

每组至少 3 包，GEOREF 相对 RAW 的通过标准：

```text
h_rms 下降
h_p95 下降，目标 >= 10%
modal_energy_rms 下降，目标 >= 10%
eta_dot_rms 不上升
tracking_time 不超过 RAW +15%
ay_p95 不上升
solve_success >= 0.97
无碰撞、无人工接管、无明显定位丢失
```

实物正式录包前应补进 GeoRef 诊断话题：

```text
/scout/global_path_anti_slosh
/anti_slosh_path/candidate_report
/anti_slosh_path/debug/original
/anti_slosh_path/debug/mild
/anti_slosh_path/debug/medium
/anti_slosh_path/debug/strong
```

## 9. 论文当前写法

推荐标题方向：

```text
Online Low-Excitation Geometric Reference Generation for Slosh-Aware MPC Tracking
```

推荐贡献表述：

```text
本文提出一种在线低激励几何参考后处理层，
在 MBF 全局路径和 MPC tracking 之间生成低曲率、低曲率变化、低预测横向激励的候选路径，
并通过普通 MPC 在车辆约束下跟踪该参考，从而降低液体晃动观测指标。
```

不要写：

```text
MPC soft cost 主动抑制了液体晃动。
输出层裁剪实现了通用防晃。
方法已证明对任意路径泛化。
方法已证明实物真液面下降。
```

## 10. 下一步

优先级：

```text
1. 补充 record_slosh_experiment.sh 的 GeoRef 诊断话题。
2. 在开阔实物场地做 RAW_REAL x3 vs GEOREF_REAL x3。
3. 每包后立即检查 candidate_report，selected=original 不计为 GeoRef 有效样本。
4. 若实物 open 通过，再做第二 open 目标和 RealSense 真液面交叉验证。
5. maze / 窄通道只在 raw tracking 和 collision safety 本身稳定后再进入，不作为当前主结论。
```
