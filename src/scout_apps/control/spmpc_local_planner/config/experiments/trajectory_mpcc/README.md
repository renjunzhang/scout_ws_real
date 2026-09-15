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
| `variants/<variant>/*` | 非液体权重、slosh_enable、w_slosh、slosh_constraint_enable等；planned两组除液体项保持匹配 |

共享配置明确 `terminal.mpc_stop_handoff_enable=true`、`terminal.complete_stop.enable=false`：默认做真实目标停车和队列释放，不启用完整液体稳定等待。开启液体等待或硬液面约束是额外实验条件，需显式记录，不能让 raw 消费液体。

共同普通权重：w_lag=0.2、w_progress=0.2、w_v=1、w_vs=0.3、v_ref=0.25、w_control=0.1、w_accel=0、w_smooth=0.1、w_alpha/w_du_a/w_du_vs=0.1。新几何组关闭参考曲率限速，raw保留。当前权重是候选；曲率权重1曾妨碍终点朝向修正，不能仅凭“更缓”认定更好。

[corner_task.example.yaml](corner_task.example.yaml) 是 **ROS deadline覆盖层**，不是上层优化器任务。上层完整任务样例是 [test/native/scenarios/corner.json](../../../../../../../test/native/scenarios/corner.json)。[corner_region.example.yaml](corner_region.example.yaml) 只是该软件场景区域，不能当实测空闲地图。

## 启动与录制

以下均在仓库根目录执行，先在 ROS1 环境 source 对应工作区。`trajectory_mpcc.launch` 仅启动 planner，传感器、定位、底盘和原路线 publisher 使用原有流程。先检查不发布速度的节点接口：

```bash
roslaunch spmpc_local_planner trajectory_mpcc.launch \
  profile:=planned_mpcc region_config:=/path/region.yaml \
  plan_file:=/path/plan.json task_overlay_file:=/path/task_deadline.yaml \
  publish_cmd_vel:=false
```

当前`publish_cmd_vel=false`缺少外部最终命令历史接入，会停在显式执行器`NO_CMD_HISTORY`；上例仅检查节点接口，尚不能取得完整OCP回放证据，详见[REVIEW-05](../../../../../../../docs/实物实验注意事项/后续改进/20260916_局部规划器复审与避障接入梳理.md)。

实际运行时 `publish_cmd_vel` 默认 true，节点可能发运动命令。换 `planned_slosh` 会自动选择 B_slosh，无需重复指定 variant。raw/geometry 不提供 `plan_file`。

加载顺序与 [launch](../../../launch/trajectory_mpcc.launch) 一致：common → variants → platform → container → profile → task_config → task_overlay → region → planner_overlay → 显式 variant/publish/plan 参数。`task_config` 默认共享配置；若自行替换，应保留全部共同合同。

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
