# 2026-06-22 LT-DWA-v2-inspired S 曲线失败证据冻结

**冻结日期：** 2026-06-23
**工作目录：** `/home/a/scout_ws`
**当前分支：** `diag/lt-dwa-collision-tracking`
**远端分支：** `origin/diag/lt-dwa-collision-tracking`
**当前 HEAD：** `4337444 更新地形晃液经验地图方案状态`

## 1. 冻结目的

本文件用于 Phase 0 证据冻结：在转向作者公开的 official LT-DWA ROS Noetic 路线之前，固化当前 Scout-owned `LT-DWA-v2-inspired` adapter 在手点 S 曲线中的失败证据、论文口径和安全边界。

冻结后的结论是：

```text
当前 LT-DWA-v2-inspired 只作为 experimental / appendix asset 保留；
不能把它称为 official LT-DWA 复现；
不能进入正式论文主表；
后续 official LT-DWA 路线必须重新以官方代码原样复现、接口审计和 fresh-sim Gate 为准。
```

## 2. 相关提交

```text
b5663c1 接入LT-DWA-v2并记录严格烟测门禁
  - 新增 Scout-owned lt_dwa_v2_adapter。
  - 当时 strict fresh-sim 结果仍为 TRACKING_DIVERGED_AT_START，不得进主表。

dec7aae 修复LT-DWA-v2全局层跟踪与严格烟测指标
  - 收紧 progress matching / lookahead。
  - 强化 scoring / speed cost / frontier pruning。
  - 单 case strict fresh-sim 通过，但仍保持 conditional / not direct main-table eligible。

5d447bd 记录LT-DWA-v2手点S曲线跟踪发散问题
  - 记录手点 S 曲线可视化失败。
  - 判断失败主因不是 omega 上限，而是 scoring / local target / lateral recovery / progress 假进展。
```

当前 HEAD 还包含 Map-vref 相关提交，但本冻结文件只针对 LT-DWA-v2-inspired。Map-vref 不纳入本轮判断。

## 3. 当前方法命名冻结

正式命名应为：

```text
LT-DWA-v2-inspired experimental adapter
```

不得称为：

```text
official LT-DWA
完整 LT-DWA 复现
作者公开 ROS Noetic 实现
可进入主表的现代 LT-DWA baseline
```

原因：当前 adapter 的设计是：

```text
DWA 工程框架
+ LT-DWA 长视域候选采样/评分思想
```

但没有完整保留官方论文/代码中的：

```text
Reference Navigation Path
Time-Varying Distance Fields
Long-Term DWA state-cost tree
voxel sampling
EB-MPC graph optimization
```

因此，当前失败只能说明 `LT-DWA-v2-inspired` 作为不完整 inspired adapter 在该 S 曲线场景下不稳定，不能作为 official LT-DWA 性能结论。

## 4. 证据目录

最新手点 S 曲线证据目录：

```text
/data/a/scout_sim_replacement/logs/lt_dwa_v2_visual_manual_goal_20260622_230318
```

关键文件：

```text
manual_clicked_s_curve.json
manual_goal_visual.bag
manual_path_generator.log
lt_dwa_v2.launch.log
env_launcher.log
```

## 5. 证据文件 hash

```text
d7d1ba3de1f87cf3e65cc6abe2b82e4b64e17be2356cf66f0e4fbaef9ccdd6d2  manual_clicked_s_curve.json
6f1845752c7a853369aa81f1b86f2863031c46efee1bb5ab492379420fd3a4d6  manual_goal_visual.bag
26703bbfba3f24e848e26dbc61c5c132fdf2a773592883c8d40b74cf0c7d72dc  manual_path_generator.log
4c8aa53194c2d4fbd31f68dc40903574810d74a4818debf7a6acaf9eaa834be4  lt_dwa_v2.launch.log
fb31d641c0ec572f0bd4cbeb4e0f8133ab60d878d42320bcc5c53c34255aba66  env_launcher.log
```

完整路径对应：

```text
/data/a/scout_sim_replacement/logs/lt_dwa_v2_visual_manual_goal_20260622_230318/manual_clicked_s_curve.json
/data/a/scout_sim_replacement/logs/lt_dwa_v2_visual_manual_goal_20260622_230318/manual_goal_visual.bag
/data/a/scout_sim_replacement/logs/lt_dwa_v2_visual_manual_goal_20260622_230318/manual_path_generator.log
/data/a/scout_sim_replacement/logs/lt_dwa_v2_visual_manual_goal_20260622_230318/lt_dwa_v2.launch.log
/data/a/scout_sim_replacement/logs/lt_dwa_v2_visual_manual_goal_20260622_230318/env_launcher.log
```

## 6. 仿真环境冻结

本次可视化测试使用隔离仿真环境：

```text
sim_root=/data/a/scout_sim_replacement
ROS_MASTER_URI=http://localhost:11328
GAZEBO_MASTER_URI=http://localhost:11362
MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream
```

路径生成方式：

```text
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
PATH_AMPLITUDE_RATIO=0.18
PATH_SIDE=left
```

控制输出入口：

```text
/benchmark/cmd_vel_raw
```

本轮是人工 RViz 手点 `/scout/goal` 后的可视化诊断证据。若后续使用已运行仿真复查，只能标记为 `current-sim diagnostics`，不能标记为 strict fresh-sim Gate 1。

## 7. 失败现象冻结

手点 goal 后，`template_fixed_path_generator.py` 成功生成 S 曲线全局路径，并发布到：

```text
/scout/global_path_fixed
```

路径生成日志摘要：

```text
Received goal in frame map: x=6.980 y=0.018
Generated template path s_curve with 162 poses in frame map (start_heading=current)
Saved generated fixed path to .../manual_clicked_s_curve.json
```

实际闭环现象：

```text
S 弯转弯处存在明显切弯 / 过冲；
中后段 lateral error 持续扩大；
最终触发 TRACKING_DIVERGED；
安全 gate 起作用，进入 NO-GO 停止状态。
```

最终状态近似：

```text
TRACKING_DIVERGED expanded=0 valid=0 goal_dist≈3.26 goal_yaw_err≈2.08
```

active tracking 时间线：

```text
t≈166.6  GOAL_RECEIVED / PATH_RECEIVED
t≈167.4  TRACKING 开始
t≈215.6  TRACKING_DIVERGED
active tracking ≈48.18 s
```

关键诊断：

```text
match_progress_s: 0.00 -> 5.90 m
match_dist:       0.00 -> 1.59 m
match_heading_err: 最高约 81 deg，末端约 75 deg
```

即：progress 继续推进，但横向误差和航向误差逐渐失控。

## 8. 角速度上限判断冻结

当前证据不支持“主要由 `omega_max_radps=1.2` 硬上限导致失败”。

bag 诊断摘要：

```text
cmd_w mean_abs ≈ 0.217 rad/s
cmd_w p95_abs  ≈ 0.45 rad/s
cmd_w max_abs  ≈ 0.54 rad/s
cmd_w >= 1.14 rad/s 的比例 = 0

odom_w mean_abs ≈ 0.217 rad/s
odom_w p95_abs  ≈ 0.45 rad/s
odom_w max_abs  ≈ 0.54 rad/s
odom_w >= 1.14 rad/s 的比例 = 0
```

若是角速度硬限制导致，应该观察到：

```text
cmd_w 长时间贴近 ±1.2 rad/s
```

或：

```text
cmd_w 已经很大，但 odom_w 明显跟不上
```

本轮两者都没有出现。

## 9. 当前根因判断冻结

更可能的问题是当前 inspired adapter 的局部规划策略结构不完整：

```text
1. scoring / local target 对 S 弯约束不足；
2. 候选轨迹切弯或过冲后仍能获得 progress reward；
3. lateral error 变大后没有足够强的 “减速 + 大角速度回正” recovery；
4. progress 继续推进，说明存在“偏着走也算进展”的假进展；
5. 当前没有官方 LT-DWA 的完整 reference navigation path + state-cost tree + EB-MPC 优化链路。
```

因此，继续盲调当前 adapter 的 `path_lateral / heading / progress / lookahead_distance` 权重不应作为主线。

## 10. 为什么当时没有直接 clone / 接入 official LT-DWA

原因不是不知道官方仓库，而是当时采取了更保守的 source-only vendor 和 Scout-owned adapter 路线：

```text
1. 官方仓库不是标准 nav_core local planner 插件；
2. 官方主程序偏作者 demo / test loop，含 ORCA/static/crowd 测试逻辑；
3. 官方仓库包含 local_planner、navigation 等 catkin package 名称，直接放入 src/ 会与当前 workspace 冲突；
4. 尚未完成官方 demo 原样复现、依赖审计、接口审计、smoke gate；
5. 项目红线要求不能污染 ROS DWA、不能改实物默认链路、不能绕过 benchmark gate；
6. 因此先把 upstream 作为 source-only 参考快照保存，再另建 Scout-owned adapter 做实验性接入。
```

这一路线的结果是当前 `lt_dwa_v2_adapter` 成为了 inspired adapter，而不是官方完整复现。现在 S 曲线失败后，应该停止把它当作正式 LT-DWA baseline 调参，并转向 official LT-DWA 的隔离原样复现。

## 11. `third_party/LT_DWA` 是什么

当前仓库已有：

```text
third_party/LT_DWA/
```

它是官方仓库 `<https://github.com/flztiii/LT_DWA>` 的 source-only vendor 快照。

根据 `third_party/LT_DWA/SCOUT_VENDOR_NOTES.md` 和 `third_party/README.md`，其状态冻结为：

```text
upstream inspected commit: 6f49cce
import date: 2026-06-19
.git metadata stripped: yes
tracked as ordinary repo files: yes
main catkin workspace build: no
benchmark adapter: pending
strict-fresh runnable baseline: no
```

它被放在 `third_party/` 而不是 `src/` 的原因是：

```text
官方仓库内含 local_planner、navigation 等 catkin packages；
这些包名会与当前 Scout workspace 中已有 package 冲突；
直接 symlink/copy 到 src/ 会污染主 catkin workspace；
因此当前只作为 source-only upstream reference，不参与编译和 benchmark。
```

所以：

```text
third_party/LT_DWA 不是当前正在跑的 LT-DWA-v2-inspired adapter；
third_party/LT_DWA 不是已接入 benchmark 的 official baseline；
third_party/LT_DWA 只是官方源码快照 / 参考资产。
```

当前正在跑的 inspired adapter 是：

```text
src/scout_apps/control/lt_dwa_v2_adapter/
```

## 12. 后续 official 路线边界冻结

转向 official LT-DWA 时，第一步不应继续把 `third_party/LT_DWA` 直接搬进 `src/` 编译，也不应马上接 `/benchmark/cmd_vel_raw`。

推荐顺序：

```text
Phase 1: 在 /data/a 新建隔离 catkin workspace，原样 clone/复现 official LT-DWA。
Phase 2: 完成 official code interface audit。
Phase 3: 仅在必要时抽出 library / wrapper seam，并用 patch 记录 upstream deviation。
Phase 4: 新建 Scout bridge，默认 shadow-only / diagnostics-only。
Phase 5: current-sim diagnostics。
Phase 6: strict fresh-sim 分级 Gate。
```

在进入正式主表前，official LT-DWA 必须通过：

```text
1. 官方 demo 原样复现；
2. upstream commit / license / dependency / config hash 固化；
3. Scout bridge 不污染官方核心；
4. 不订阅或消费 /slosh/* / /benchmark/slosh_monitor/*；
5. 不修改实物 launch 默认行为；
6. 不修改底盘实际控制入口；
7. 不修改 /cmd_vel 生产链路；
8. 不修改 SPMPC OCP 参数输入链路；
9. command gate clamp ratio、freshness、monitor isolation 均满足 Gate；
10. 直线、左右单弯、低曲率 S 弯、正式 S 弯 fresh-sim N≥5 分级通过。
```

## 13. 论文和报告口径冻结

推荐写法：

```text
We first built a Scout-owned LT-DWA-inspired adapter to test long-horizon DWA-style sampling and scoring under the fixed-path benchmark. This adapter is not an official reproduction of LT-DWA. In manual S-curve visualization it exhibited cut-corner / overshoot behavior and eventually triggered TRACKING_DIVERGED, while commanded and measured yaw rates stayed far below the configured yaw-rate limit. We therefore treat this adapter as an experimental appendix asset and evaluate the official LT-DWA implementation separately after original demo reproduction and Scout bridge validation.
```

中文口径：

```text
当前失败不是 official LT-DWA 的失败，而是不完整 inspired adapter 的失败。
它说明单纯移植“长视域采样/评分思想”不足以作为完整 LT-DWA baseline。
后续若要在主文比较 LT-DWA，应以作者公开 ROS Noetic 实现为基础，先做原样复现和隔离 Scout bridge，再按 common-limit / fresh-sim Gate 决定是否进入主表。
```

## 14. 冻结后禁止事项

在没有新 Phase / micro-patch 明确授权前，禁止：

```text
继续盲调 lt_dwa_v2_adapter 的 path/progress/scoring 权重并声称修复 official LT-DWA；
把 third_party/LT_DWA 直接 symlink/copy 到 src/；
把 official LT-DWA 直接发布到 /cmd_vel；
修改 ROS DWA；
修改 third_party/LT_DWA 核心算法作为 Scout 业务代码；
修改实物 launch 默认行为；
修改底盘实际控制入口；
修改 /cmd_vel 生产链路；
修改 SPMPC OCP 参数输入链路；
让 slosh monitor 输出进入 planner / profile generator / command gate / OCP / cmd_vel 链路。
```

## 15. 本文件修改范围

本 Phase 0 只新增证据冻结文档。

未修改：

```text
src/scout_apps/control/lt_dwa_v2_adapter/
third_party/LT_DWA/
ROS DWA
实物 launch 默认行为
底盘控制入口
/cmd_vel 生产链路
SPMPC OCP 输入链路
slosh monitor 相关链路
```
