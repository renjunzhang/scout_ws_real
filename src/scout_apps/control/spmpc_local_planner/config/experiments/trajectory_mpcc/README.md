# 两层协同四组配置

更新：2026-09-16。这是 `feat/spmpc-liquid-control` 的软件候选，尚未实物验收；已有模型闭环未证明相对 raw 稳定降晃，最坏周期也未满足30 Hz。[方法](../../../README_METHOD.md)、[实物脚本索引](../../../scripts/README.md)、[验证记录](../../../../../../../docs/实物实验注意事项/对比试验/解决问题的思路/20260916_两层协同实现与软件验证记录.md)。

## 选择 profile

| Profile | 下层 variant | 参考 | 几何目标 / contour | 下层液体 |
| --- | --- | --- | --- | --- |
| [raw_mpcc](raw_mpcc.yaml) | B0 | 原路线、cruise | 关闭 / 1.0 | 全部决策关闭 |
| [geometry_mpcc](geometry_mpcc.yaml) | B0 | 原路线、cruise | 开启 / 0.02 | 关闭 |
| [planned_mpcc](planned_mpcc.yaml) | B0 | 完整计划、progress | 开启 / 0.02 | 关闭 |
| [planned_slosh](planned_slosh.yaml) | B_slosh | 同一完整计划、progress | 相同 / 0.02 | w_slosh=5，硬约束默认关闭 |

主比较为原路线直接进入普通 raw MPCC 与完整方法。所有组都必须给同一个显式物理区域；raw 使用区域不等于新增路径优化。planned 两组共用上层液体计划，仅隔离下层液体项。若要分离上层液体贡献，另生成 `objective.liquid=0`、`objective.liquid_terminal=0` 的计划。

## 文件和参数归属

| 文件 / 参数 | 用途 |
| --- | --- |
| [trajectory_common.yaml](trajectory_common.yaml) | 四组共同目标/停车容差、jerk=1、RTI=5、actual_v_min=-0.002；deadline默认0，必须覆盖 |
| `region_config` | ROS YAML 中的 `planning.region`：id、frame、非空有序凸cells、包络半径及margin；节点不从地图/路线推断区域 |
| `task_overlay_file` | ROS YAML：至少提供匹配的 `planning.task_deadline_sec>0`；可以覆盖评价窗等共同条件 |
| `plan_file` | `generate_trajectory_plan.py` 产生的完整 JSON；planned组必需，raw不使用 |
| `planner_overlay_file` | 最后加载的实验调参 YAML，如观察器来源、几何/液体权重；四组共同条件应保持一致 |
| `planning.geometry.*` | curvature_weight=0.05、curvature_rate_weight=0.01、speed_regularization=0.05、goal_weight=2；目标归一化默认0.3 m/1 rad |
| `planning.projection.lookahead` | 默认2.0个路线进度单位，连续分支搜索窗口；不按横向误差或v×dt限制切角 |
| `terminal.complete_stop.max_tail_prediction_sec`、`quiet_v/quiet_omega` | 默认8 s、0.001 m/s与0.001 rad/s；区域普通制动与完整停车共享，预算不足会明确失败 |
| `command_history.source`、`external_audit_topic` | 默认published；无输出回放可用external_audit和独立输入topic，launch参数为command_history_source/external_audit_topic |
| `platform.shared_constraints.linear_accel_limit_enable`、`execution_contract.fail_closed_on_post_limit_change` | 当前合并默认true/false；存在求解后命令改写缺口，尚未继承旧explicit-actuator runner的false/true合同 |
| `variants/<variant>/*` | 非液体权重、slosh_enable、w_slosh、slosh_constraint_enable等；planned两组除液体项保持匹配 |

共享配置明确 `terminal.mpc_stop_handoff_enable=true`、`terminal.complete_stop.enable=false`：默认做真实目标停车和队列释放，不启用完整液体稳定等待。开启液体等待或硬液面约束是额外实验条件，需显式记录，不能让 raw 消费液体。

区域启用时普通停车仍检查完整尾段，不能关闭terminal handoff或jerk来绕过。制动固定当前cell，候选首命令会耗尽停车余量时改用已核验区域尾段的制动命令，接管保持至任务重置；重新发布同一路线不会让车重新开动。停车诊断追加区域核验、余量和FIFO前缀越界字段。当前仍缺全尾段速度边界核验，普通参考末端也存在切向/曲率退化，见[再次复盘](../../../../../../../docs/实物实验注意事项/后续改进/20260916_当前主线复盘与剩余缺口.md)；原五项状态见[修复记录](../../../../../../../docs/实物实验注意事项/后续改进/20260916_局部规划器五项修复与回归.md)。

共同普通权重：w_lag=0.2、w_progress=0.2、w_v=1、w_vs=0.3、v_ref=0.25、w_control=0.1、w_accel=0、w_smooth=0.1、w_alpha/w_du_a/w_du_vs=0.1。新几何组关闭参考曲率限速，raw保留。当前权重是候选；曲率权重1曾妨碍终点朝向修正，不能仅凭“更缓”认定更好。

用户确认当前实物运行7d17f6c，已包含显式执行器、四子步RK4和液面硬约束入口，实际启用条件以运行配置为准。81e191d仅是已归档两包的版本：该Full的processed-IMU、v_ref=0.2、jerk=0.6、w_slosh=1，与新planned_slosh默认odom、0.25、1、5不同，不能将历史条件自动当作当前实物配置。新四组还存在发布时效、定位/observer恢复及失败停车缺口；硬液面约束启用前需处理周期内峰值漏检。配置差异和修复依赖见[合并审查](../../../../../../../docs/实物实验注意事项/后续改进/20260916_当前主线复盘与剩余缺口.md)，不能直接用新默认值与旧bag比较方法收益，也不应整份套用含其他液体条件的历史overlay。

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
bash src/scout_apps/control/spmpc_local_planner/scripts/record_trajectory_mpcc_comparison.sh
```

实际录制时去掉 `VALIDATE_ONLY=true`，`PROFILE`、`TASK_CONFIG`、`TASK_OVERLAY_FILE`、`REGION_CONFIG`、`PLANNER_OVERLAY_FILE`、`PLAN_FILE`、`PLANNER_VARIANT` 和 `PUBLISH_CMD_VEL` 须与已启动节点一致。脚本只录包，不 launch、不发速度；以 Ctrl-C 结束，没有 `RECORD_SEC` 定时参数。

manifest冻结完整合并配置、artifact副本/hash、Git SHA/status/diff；开包前逐叶核对 `/spmpc_local_planner` live参数。`LAUNCH_ARGS` 是声明备注，不会替代配置加载。latched `/spmpc/debug/planning_config` 仅含planning字段，property_tree标量可能为字符串；**完整配置以manifest及核对后的live参数为准**。

bag记录原路线、实际命令、预测/快照/audit、代价、停车和液体观察器。相机和NOKOV原始数据需原有采集链另录；不要假设该recorder自动包含RGB或动捕。所有计划、区域、任务期限和评价窗口须按同一物理任务核对。
