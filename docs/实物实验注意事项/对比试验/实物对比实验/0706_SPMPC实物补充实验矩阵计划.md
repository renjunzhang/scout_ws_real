# 0706 SPMPC 实物补充实验矩阵计划

> 目的：对照当前论文实验章节，补齐 20260705 已完成 `B0 / B_slosh / B_ours` N=3 fixed-path 实物实验之外仍缺的关键矩阵。  
> 当前优先级不是继续调参，而是让实物证据链能支撑“显式晃液状态预测不只是平滑控制”的主张。

相关已完成记录：

```text
docs/实物实验注意事项/对比试验/实物对比实验/0705B0_fixed_path基础跟踪调试指南.md
docs/实物实验注意事项/对比试验/实物对比实验/0705B0_B_slosh_B_ours_SPMPC内部晃动高度实物验证指南.md
docs/实物实验注意事项/对比试验/实物对比试验分析/20260705_fixed_path_N3_RGB_模型综合分析.md
```

---

## 1. 对照论文实验章节，目前还缺什么

论文实验章节需要回答的核心问题是：

```text
显式晃液状态预测是否能在单纯平滑控制之外提供可解释防晃作用，
并且不以不可接受的 tracking、任务时间或实时性代价为条件。
```

20260705 已经完成并相对干净的证据：

```text
1. B0 / B_slosh / B_ours formal N=3；
2. 9 个 bag 全部 GOAL_REACHED；
3. fixed_closed_loop 0.15 / 0.22 全部生效；
4. hard constraint disabled；
5. internal model: B_slosh < B_ours < B0；
6. RGB H_vis: B_ours 最低；
7. command safety 没有 tracking_safety zero 污染。
```

但还缺下面几类证据：

| 优先级 | 缺口 | 为什么重要 | 是否需要新实物 run |
|---|---|---|---|
| P0 | `B_smooth` 实物矩阵 | 证明 `B_ours` 不是仅靠更平滑控制；补齐 2x2 内部消融 | 需要 |
| P0 | 同日 `B_ours` bridge | 避免用 0705 `B_ours` 与 0706 `B_smooth` 跨日比较 RGB/液位状态 | 推荐需要 |
| P1 | 模型--RGB 一致性指标 | 支撑 `H_model` 只是诊断代理，真实结论由 RGB 支持 | 主要离线分析 |
| P1 | 在线求解/执行诊断统计 | 支撑“可在线部署”，排除 gate/limiter 污染 | 主要离线分析 |
| P2 | 实物外部 baseline，如 DWA/TEB | 支撑普通局部规划器对照，但安全和脚本成本更高 | 可选 |
| P3 | hard constraint 或 stress 场景 | 当前论文第一版不是必须；会引入可行性变量 | 暂不建议 0706 跑 |

因此 0706 的主线不是再跑 delay off，也不是 hard，而是：

```text
先补 B_smooth，再用同日 B_ours 做桥接。
```

---

## 2. 0706 主线矩阵：B_smooth vs B_ours

### 2.1 实验目的

该矩阵直接服务论文中的内部消融问题：

```text
B_smooth vs B0      -> 只增强平滑性带来的变化；
B_slosh vs B0       -> 显式晃液状态预测的独立作用；
B_ours vs B_smooth  -> 完整方法是否超越 smooth-only；
B_ours vs B_slosh   -> 平滑项是否改善真实液面/执行平滑性。
```

20260705 已有 `B0 / B_slosh / B_ours`，但缺 `B_smooth`。由于 RGB 真实液面会受光照、液位、相机姿态、初始静稳程度影响，0706 不建议只跑 `B_smooth` 后直接和 0705 的 `B_ours` 做强结论。最低也要跑一个同日 `B_ours` bridge；推荐跑 `B_smooth / B_ours` 各 N=3。

### 2.2 推荐矩阵

推荐 0706 正式矩阵：

```text
Round 1: B_smooth -> B_ours
Round 2: B_ours   -> B_smooth
Round 3: B_smooth -> B_ours
```

命名：

```text
Bsmooth_fixed_150_220_0706_r1
Bours_fixed_150_220_0706_r1
Bours_fixed_150_220_0706_r2
Bsmooth_fixed_150_220_0706_r2
Bsmooth_fixed_150_220_0706_r3
Bours_fixed_150_220_0706_r3
```

这样可以形成：

```text
B_smooth N=3
B_ours   N=3，同日 bridge
```

如果现场时间不足，最低交付降级为：

```text
B0 fixed 0.15/0.22 gate N=1
B_smooth N=3
B_ours same-day bridge N=1
```

但用于论文主表时，仍推荐补足 `B_ours` 同日 N=3。

---

## 3. 0706 统一控制口径

所有 0706 主线 run 必须显式写完整控制口径，不依赖脚本默认值：

```text
GOAL_X=-5.424
GOAL_Y=-4.736
GOAL_YAW=0.0
PATH_TEMPLATE=s_curve
PATH_START_HEADING=current
PATH_AMPLITUDE_RATIO=0.18
PATH_MIN_AMPLITUDE=0.25
PATH_MAX_AMPLITUDE=1.20
PATH_SIDE=left
PATH_SMOOTH_ITERATIONS=3
PATH_SPACING=0.05

V_REF=0.20
ALPHA_MAX=1.2
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true
SHARED_LINEAR_ACCEL_MAX=0.6
SHARED_ANGULAR_LIMIT_ENABLE=true
SHARED_ANGULAR_RATE_MAX=1.2
SHARED_ANGULAR_ACCEL_MAX=1.2

DELAY_PHASE_MODE=fixed_closed_loop
DELAY_PHASE_LINEAR_DELAY_SEC=0.15
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22

RECORD_TOPIC_INFO=false
RECORDER_STARTUP_SEC=8
RECORD_RGB=true
RECORD_SEC=60
MAX_RECORD_SEC=60
hard constraint disabled
IMU disabled / not introduced
```

判据：

```text
1. 必须 GOAL_REACHED；
2. zero_due_to_tracking_safety = 0；
3. delay_compensation_applied_frac 接近 1.0；
4. effective_config 与上面控制口径一致；
5. RGB topic 覆盖 first ACADOS_OK -> first GOAL_REACHED，并保留 post-goal 残余段；
6. 如果明显离轨、急停、TF/odom/map 异常，该 run 不作为 clean paper sample。
```

---

## 4. 一键矩阵启动方式

已经把 0706 主线矩阵整理进今天使用的一键脚本：

```text
src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

使用 `MATRIX_PRESET=0706_bsmooth_bours` 即可按本文件推荐顺序执行：

```text
B0 gate -> B_smooth r1 -> B_ours r1 -> B_ours r2 -> B_smooth r2 -> B_smooth r3 -> B_ours r3
```

正式启动命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

MATRIX_PRESET=0706_bsmooth_bours \
MATRIX_LABEL_TAG=0706 \
MATRIX_INCLUDE_GATE=true \
MATRIX_WAIT_BETWEEN_RUNS=true \
MATRIX_CONTINUE_ON_FAIL=false \
CMD_TOPIC=/cmd_vel \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_SEC=60 \
MAX_RECORD_SEC=60 \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

脚本会在每个 run 前暂停，提示现场人员：

```text
回起点标记 -> 对齐 heading -> 液体静稳 60~90s -> 检查 RGB/TF/odom -> 按 Enter 开始下一包
```

注意：

```text
1. B0 gate 自动使用 RECORD_RGB=false；
2. B_smooth / B_ours 正式 run 自动使用 RECORD_RGB=true；
3. 每个子 run 仍调用同一个 one-click trial 流程；
4. 任一 run 失败时默认停止后续矩阵，避免把现场异常继续扩大。
```

如果当天已经人工完成 B0 gate，只想跑 `B_smooth / B_ours` 六包，可设：

```bash
MATRIX_INCLUDE_GATE=false
```

---

## 5. 0706 前置 gate：B0 fixed baseline 快速复核

0706 开始正式矩阵前，建议先跑一个 B0 gate，确认当天定位、底盘、电池、液体状态没有明显变化。

命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B0 \
RUN_LABEL=B0_fixed_150_220_0706_gate01 \
CMD_TOPIC=/cmd_vel \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_RGB=false \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

通过建议：

```text
GOAL_REACHED；
projection p95 约 3cm 量级；
zero_due_to_tracking_safety = 0；
delay_compensation_applied_frac = 1.0。
```

如果 B0 gate 失败，不进入 `B_smooth / B_ours` 正式矩阵，先排查定位、起点、底盘状态和 recorder/planner 时序。

---

## 6. B_smooth 正式命令模板

每轮只改 `RUN_LABEL`。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B_smooth \
RUN_LABEL=Bsmooth_fixed_150_220_0706_r1 \
CMD_TOPIC=/cmd_vel \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_RGB=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

目的：只看 smooth-only 是否已经足够降低 RGB 液面和 internal model；后续与 `B_ours` 同日 bridge 比较。

---

## 7. B_ours 同日 bridge 命令模板

每轮只改 `RUN_LABEL`。

```bash
source /opt/ros/noetic/setup.bash
source /home/geist/scout_ws/devel/setup.bash
cd /home/geist/scout_ws

ALG=B_ours \
RUN_LABEL=Bours_fixed_150_220_0706_r1 \
CMD_TOPIC=/cmd_vel \
V_REF=0.20 \
ALPHA_MAX=1.2 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true \
SHARED_LINEAR_ACCEL_MAX=0.6 \
SHARED_ANGULAR_LIMIT_ENABLE=true \
SHARED_ANGULAR_RATE_MAX=1.2 \
SHARED_ANGULAR_ACCEL_MAX=1.2 \
DELAY_PHASE_MODE=fixed_closed_loop \
DELAY_PHASE_LINEAR_DELAY_SEC=0.15 \
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22 \
RECORD_TOPIC_INFO=false \
RECORDER_STARTUP_SEC=8 \
RECORD_RGB=true \
bash src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
```

目的：同日桥接 `B_smooth`，避免把 0706 的 RGB 环境直接和 0705 的 `B_ours` 强比较。

---

## 8. 每个 run 之间的现场要求

每个 run 之间执行：

```text
1. 脚本结束后确认 /cmd_vel 为 0；
2. 遥控回到起点标记附近；
3. heading 尽量对齐路径起点方向；
4. 液体静稳 60~90s；
5. 相机、光源、容器、液位不要变化；
6. 确认 RealSense RGB topic 正常；
7. 再执行下一 run。
```

RGB 前置检查：

```bash
rostopic echo -n 1 /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info
rostopic hz /camera/color/image_raw
```

---

## 9. 0706 后处理必须输出的指标

统计窗口沿用 20260705：

```text
first ACADOS_OK -> first GOAL_REACHED，
并按 /spmpc/debug/stage0_reference.s0 取 path progress 10%~90%。
```

必须输出：

```text
1. GOAL_REACHED / goal time；
2. effective_config 一致性；
3. delay_compensation_applied_frac；
4. zero_due_to_tracking_safety / published_zero_frac；
5. /spmpc/slosh_height p95 / max_10_90 / RMS；
6. RGB H_vis p95 / max / RMS；
7. projection p95 / max；
8. |omega| p95 / max；
9. cost share: progress_abs / contour / v / control / smooth / slosh；
10. slosh eta / eta-dot share；
11. B_ours vs B_smooth 的降幅表。
```

核心判读：

```text
如果 B_ours 的 RGB H_vis 明显低于 B_smooth，且 tracking/time 没有不可接受恶化，
就能支撑“显式晃液状态预测 + 平滑控制优于单纯平滑控制”。

如果 B_smooth 与 B_ours RGB 接近，但 internal model B_ours 更低，
则实物真实液面结论要保守，只能说模型侧证据更强。

如果 B_smooth RGB 反而更低，
要优先检查 B_ours 是否因为 slosh model 相位/幅值误差导致优化目标与真实液面不一致。
```

---

## 10. 不建议 0706 主线做的矩阵

### 9.1 不建议继续扩展 delay 矩阵

0705 已证明：

```text
B0 + fixed_closed_loop 0.15/0.22 最干净；
B0 + delay off 在保守角向限制下失败；
B0 + delay off + 放开角向限制可以到点但 tracking 更差。
```

0706 主线应固定 delay 口径，不要再把 delay 当主动变量。

### 9.2 不建议跑 hard constraint

hard 会引入 feasibility、active set、ACADOS fail 等变量。当前论文第一版的实物证据应先证明 soft cost 与 smooth-only 的差异；hard-cap 可以作为后续 stress/safety appendix。

### 9.3 不建议临时加入 IMU

IMU 是后续改进项。0706 不应在正式矩阵里新增 IMU 观测源，否则和 0705 不再同口径。

---

## 11. 可选 P2：实物外部 baseline

如果 0706 完成 `B_smooth / B_ours` 后仍有时间，可以考虑实物外部 baseline，但优先级低于 P0。

候选：

```text
DWA 或 TEB，固定同一路径/终点/速度约束，录 RGB。
```

但只有在已有经过验证的一键实物 baseline 脚本时才跑。不要现场临时拼 launch 直接驱动 `/cmd_vel`。外部 baseline 至少需要：

```text
1. 同一 fixed S-curve；
2. v_max / omega_max / accel limits 与 SPMPC 对齐；
3. 只允许一个 /cmd_vel 发布者；
4. 先 N=1 smoke，GOAL_REACHED 后再考虑 N=3；
5. 不使用 slosh feedback；
6. RGB 和 command/tracking 完整记录。
```

如果没有稳定脚本，0706 不跑外部 baseline，把它留给仿真实验或后续专门实物 baseline 任务。

---

## 12. 0706 推荐交付物

完成后建议新增或更新：

```text
docs/实物实验注意事项/对比试验/实物对比试验分析/20260706_Bsmooth_Bours_RGB_模型对比分析.md
```

报告中必须明确：

```text
1. 0706 是补齐 smooth-only 实物消融，不是 delay/hard/IMU 实验；
2. 所有正式样本使用 fixed_closed_loop 0.15/0.22；
3. 真实液面结论看 RGB H_vis；
4. /spmpc/slosh_height 只作为 internal model evidence；
5. B_ours vs B_smooth 是论文“不是单纯平滑控制”的关键实物证据。
```
