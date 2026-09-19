# 两层协同四组配置

更新：2026-09-18。Full 已有开发级 Gazebo 到点结果，当前默认 N40 / 30 Hz；[最新定位修复](../../../../../../../docs_for_offlineslosh/仿真实验分析/02_问题定位与修复/20260918_Full首弯停滞与预算停车定位修复.md)记录完整正负例。这是 `feat/spmpc-liquid-control` 的软件候选，尚未实物验收；尚未证明相对 raw 稳定降晃或全部场景满足30 Hz。[方法](../../../README_METHOD.md)、[实物脚本索引](../../../scripts/README.md)、[验证记录](../../../../../../../docs_for_offlineslosh/实现与修复/20260916_两层协同实现与软件验证记录.md)。

## 选择 profile

| Profile | 下层 variant | 参考 | 几何目标 / contour | 下层液体 |
| --- | --- | --- | --- | --- |
| [raw_mpcc](raw_mpcc.yaml) | B0 | 原路线、cruise | 关闭 / 1.0 | 全部决策关闭 |
| [geometry_mpcc](geometry_mpcc.yaml) | B0 | 原路线、cruise | 开启 / 0.02 | 关闭 |
| [planned_mpcc](planned_mpcc.yaml) | B0 | 完整计划、progress | 开启 / 0.02 | 关闭 |
| [planned_slosh](planned_slosh.yaml) | B_slosh | 完整计划、progress | 开启 / 1.0；曲率变化权重0.0001，全程goal=0 | w_slosh=5，硬约束默认关闭 |

主比较为原路线直接进入普通 raw MPCC 与完整方法。所有组都必须给同一个显式物理区域；raw 使用区域不等于新增路径优化。当前 Full 的贴线、曲率变化与全程终点吸引权重已修正，`planned_mpcc` 尚未同步；即使两组共用上层液体计划，也不能直接当作仅差下层液体项的消融。需先对齐非液体条件。若要分离上层液体贡献，另生成 `objective.liquid=0`、`objective.liquid_terminal=0` 的计划。

## 文件和参数归属

`acados/rti_min_iterations` 默认 1，范围为 `[1,rti_iterations]`。实时预算不足以完成该次数时拒绝结果并走已有故障停车；不会绕过截止时间。最小次数与上限同为 3 时，模型测试与 ROS 都要求三次，关闭“可行即提前退出”。单次仿真脚本可用 `--fixed-rti-iterations 3` 覆盖并核对 live 参数；通用实物配置不变。

性能验证需显式用 `-DCMAKE_BUILD_TYPE=Release` 构建 native 和 ROS（不启用 fast-math）。native `geometry_trial` 在原有参数后可追加 `[solve-budget-ms] [rti-min-iterations]`，例如本次 `30.3333333333333 3`；它记录每拍实际迭代次数及分段耗时，但不包含 ROS 状态处理与发布，仍需独立核对 Gazebo 发布时序。

| 文件 / 参数 | 用途 |
| --- | --- |
| [trajectory_common.yaml](trajectory_common.yaml) | 四组共同目标/停车容差、jerk=1、RTI最多5次、actual_v_min=-0.002；关闭重复shared limiter并启用改写拒绝；deadline默认0，必须覆盖 |
| `region_config` | ROS YAML 中的 `planning.region`：id、frame、非空有序凸cells、包络半径及margin；节点不从地图/路线推断区域 |
| `task_overlay_file` | ROS YAML：至少提供匹配的 `planning.task_deadline_sec>0`；可以覆盖评价窗等共同条件 |
| `plan_file` | `generate_trajectory_plan.py` 产生的完整 JSON；planned组必需，raw不使用 |
| `planner_overlay_file` | 最后加载的实验调参 YAML，如观察器来源、几何/液体权重；四组共同条件应保持一致 |
| `planning.geometry.*` | Full：curvature_weight=0.05、curvature_rate_weight=0.0001、speed_regularization=0.05、goal_weight=0；另两几何组仍为rate=0.01、goal=2；目标归一化默认0.3 m/1 rad |
| `planning.projection.lookahead` | 默认2.0个路线进度单位，连续分支搜索窗口；不按横向误差或v×dt限制切角 |
| `terminal.goal_pose_weight` | 四组共同为2；预测进度进入既有1.2 m末端区后引导终点位置/朝向，与几何目标权重取较大值，不重复叠加；raw沿途几何目标仍关闭 |
| `terminal.complete_stop.max_tail_prediction_sec`、`quiet_v/quiet_omega` | 默认8 s、0.001 m/s与0.001 rad/s；区域普通制动与完整停车共享，预算不足会明确失败 |
| `command_history.source`、`external_audit_topic` | 默认published；无输出回放可用external_audit和独立输入topic，launch参数为command_history_source/external_audit_topic |
| `platform.shared_constraints.linear_accel_limit_enable`、`execution_contract.fail_closed_on_post_limit_change` | trajectory共同覆盖为false/true；最终命令应与已核验候选一致 |
| `variants/<variant>/*` | 非液体权重、slosh_enable、w_slosh、slosh_constraint_enable等；正式消融前须核对并对齐两组非液体项 |

发布参数在 `execution_contract`：`max_result_age_sec=0` 表示一个控制/模型周期，`publish_reserve_sec=0.006` 留给求解后处理；改变周期或时限须两组共同冻结。单次RTI不可抢占，超期结果拒绝，不能由预算开关推断已满足30 Hz。`state_timing.max_position_innovation_m=0.20`、`max_yaw_innovation_rad=0.35` 只检查定位相对运动的一致性。液体断流后无自动零状态恢复，须静置后重启；B0不因评价观察器失效而引入液体控制门。

2026-09-17：控制状态的 TF 查询改为非阻塞。目标时刻暂缺 TF 时，只允许在原有
`max_robot_extrapolation_sec=0.010` 内，用带时间戳的 TF 位姿和对应里程计相对运动传播；
缺少历史或超限仍拒绝。2026-09-18 补充：若两次查询之间 TF 已更新至目标时刻之后，再做一次非阻塞精确查询，避免把新位姿当作旧锚点；拒绝时也记录锚点与目标的时间差。`debug/control_cycle_wall_timing` 按 cycle_id 记录墙钟分段耗时和
位姿传播时长，原 `ControlCycleAudit` schema 2 消息定义保持不变，兼容既有命令历史回放。
本次末端姿态目标是新的共同实验条件，不能把修复前后的 raw 结果直接混入同一组统计。

共享配置明确 `terminal.mpc_stop_handoff_enable=true`、`terminal.complete_stop.enable=false`：默认做真实目标停车和队列释放，不启用完整液体稳定等待。开启液体等待或硬液面约束是额外实验条件，需显式记录，不能让 raw 消费液体。

区域启用时普通停车仍检查完整尾段，不能关闭terminal handoff或jerk来绕过。制动固定当前cell，候选首命令会耗尽停车余量时改用已核验区域尾段的制动命令，接管保持至任务重置；重新发布同一路线不会让车重新开动。停车诊断追加区域核验、余量和FIFO前缀越界字段。本轮补全尾段速度与子步液面峰值、普通数值失败停车接管、参考端点及发布时效，历史反例见[再次复盘](../../../../../../../docs_for_offlineslosh/实现与修复/20260916_当前主线复盘与剩余缺口.md)；原五项状态见[修复记录](../../../../../../../docs_for_offlineslosh/实现与修复/20260916_局部规划器五项修复与回归.md)。

共同普通权重：w_lag=0.2、w_progress=0.2、w_v=1、w_vs=0.3、v_ref=0.25、w_control=0.1、w_accel=0、w_smooth=0.1、w_alpha/w_du_a/w_du_vs=0.1。新几何组关闭参考曲率限速，raw保留。当前权重是候选；曲率权重1曾妨碍终点朝向修正，不能仅凭“更缓”认定更好。

用户确认当前实物运行7d17f6c，已包含显式执行器、四子步RK4和液面硬约束入口，实际启用条件以运行配置为准。81e191d仅是已归档两包的版本：该Full的processed-IMU、v_ref=0.2、jerk=0.6、w_slosh=1，与新planned_slosh默认odom、0.25、1、5不同，不能将历史条件自动当作当前实物配置。新四组还存在发布时效、定位/observer恢复及失败停车缺口；硬液面约束启用前需处理周期内峰值漏检。配置差异和修复依赖见[合并审查](../../../../../../../docs_for_offlineslosh/实现与修复/20260916_当前主线复盘与剩余缺口.md)，不能直接用新默认值与旧bag比较方法收益，也不应整份套用含其他液体条件的历史overlay。

[corner_task.example.yaml](corner_task.example.yaml) 是 **ROS deadline覆盖层**，不是上层优化器任务。上层完整任务样例是 [test/native/scenarios/corner.json](../../../../../../../test/native/scenarios/corner.json)。[corner_region.example.yaml](corner_region.example.yaml) 只是该软件场景区域，不能当实测空闲地图。

## 启动与录制

以下均在仓库根目录执行，先在 ROS1 环境 source 对应工作区。`trajectory_mpcc.launch` 仅启动 planner，传感器、定位、底盘和原路线 publisher 使用原有流程。先检查不发布速度的节点接口：

```bash
roslaunch spmpc_local_planner trajectory_mpcc.launch \
  profile:=planned_mpcc region_config:=/path/region.yaml \
  plan_file:=/path/plan.json task_overlay_file:=/path/task_deadline.yaml \
  publish_cmd_vel:=false
```

`publish_cmd_vel=false`仅关闭本节点输出。完整执行器回放还需传入 `command_history_source:=external_audit`，并把原 bag 的 `/spmpc/debug/control_cycle_audit` remap 到 `external_audit_topic`（默认 `/spmpc/replay/control_cycle_audit`）。只接纳 schema 2 中确实发布过的最终命令及其 `command_publish_stamp`，包括安全门真正发出的零；不接纳无时间戳的 `/cmd_vel`。具体步骤见[脚本索引的无运动回放](../../../scripts/README.md#无运动回放)。

实际运行时 `publish_cmd_vel` 默认 true，节点可能发运动命令。换 `planned_slosh` 会自动选择 B_slosh，无需重复指定 variant。raw/geometry 不提供 `plan_file`。

实际[launch](../../../launch/trajectory_mpcc.launch)加载顺序为：common → variants → platform → container → profile → task_config → task_overlay → region → planner_overlay → 显式 variant/publish/history source/history topic/plan 参数。`task_config`默认共享配置；若自行替换，应保留全部共同合同。recorder尚未模拟最后两个history覆盖参数，外部audit回放需要显式launch参数与planner overlay同时写相同值，操作见[无运动回放](../../../scripts/README.md#无运动回放)。

只校验录制输入（不访问 ROS，生成 manifest/artifact 副本）：

```bash
VALIDATE_ONLY=true PROFILE=planned_mpcc PUBLISH_CMD_VEL=false \
REGION_CONFIG=/path/region.yaml PLAN_FILE=/path/plan.json \
TASK_OVERLAY_FILE=/path/task_deadline.yaml \
OUT_DIR=/path/new_run NAME=planned_mpcc_r01 \
bash src/scout_apps/control/spmpc_local_planner/scripts/trajectory/record_trajectory_mpcc_comparison.sh
```

实际录制时去掉 `VALIDATE_ONLY=true`，`PROFILE`、`TASK_CONFIG`、`TASK_OVERLAY_FILE`、`REGION_CONFIG`、`PLANNER_OVERLAY_FILE`、`PLAN_FILE`、`PLANNER_VARIANT` 和 `PUBLISH_CMD_VEL` 须与已启动节点一致。脚本只录包，不 launch、不发速度；以 Ctrl-C 结束，没有 `RECORD_SEC` 定时参数。

manifest冻结完整合并配置、artifact副本/hash、Git SHA/status/diff；开包前逐叶核对 `/spmpc_local_planner` live参数。`LAUNCH_ARGS` 是声明备注，不会替代配置加载。latched `/spmpc/debug/planning_config` 仅含planning字段，property_tree标量可能为字符串；**完整配置以manifest及核对后的live参数为准**。

bag记录原路线、实际命令、预测/快照/audit、代价、停车和液体观察器。相机和NOKOV原始数据需原有采集链另录；不要假设该recorder自动包含RGB或动捕。所有计划、区域、任务期限和评价窗口须按同一物理任务核对。
