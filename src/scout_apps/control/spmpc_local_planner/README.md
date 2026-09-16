# spmpc_local_planner

更新：2026-09-16，`feat/spmpc-liquid-control`。前五项修复之后，已补发布预算与时效、可信状态恢复、完整停车及参考端点修复；本轮证据见[执行链定向回归](../../../../docs/实物实验注意事项/后续改进/20260916_执行链收敛修复与定向回归.md)。

当前主线是 **原路线引导 + 显式运动区域 + 全程几何/速度/液体规划 + 在线 MPCC 自主缓弯**。原路线提供任务方向和进度坐标；机器人可以在允许区域内偏离它，选择曲率更小的运动。路径跟踪误差用于诊断，主要评价任务完成、区域可行性、实际曲率、耗时和液体响应。

这是 **ROS1/catkin** 包（`roscpp`、`roslaunch`），尚未迁移到 ROS2。本轮在 ROS2 Jazzy 主机验证独立生产核心和真实 acados：stub 18/18、真实 acados 19/19 个测试程序及35项Python定向检查通过；未完成ROS1整节点编译。

本轮单拐角定向复跑中，B0和Full完成；两个geometry组仍在10 s求解失败，随后接管停车并最终超时。Full曲率更小、耗时更短，但模型液面峰值更高，尚无稳定降晃收益，也未通过30 Hz及实物验收。前一轮24组22完成、最长82.2 ms的记录保留在[前五项修复与回归](../../../../docs/实物实验注意事项/后续改进/20260916_局部规划器五项修复与回归.md)；最新数据见上述执行链报告。

## 从哪里开始

| 要做的事 | 入口 |
| --- | --- |
| 找最近实物使用的 C03、Full/Smooth、动捕、录包与分析脚本 | [scripts/README.md](scripts/README.md) |
| 配置四组新主线、生成计划、核对启动和录制参数 | [trajectory_mpcc 配置 README](config/experiments/trajectory_mpcc/README.md) |
| 理解模型、目标和上下层职责 | [README_METHOD.md](README_METHOD.md) |
| 查看实现状态、尚未关闭的问题与下一步 | [代码改进方向与现状](代码改进方向与现状.md) |
| 无 ROS 验证核心、生成真实 solver、复现模型闭环 | [test/native/README.md](../../../../test/native/README.md) |
| 查框架的论文依据与原修改方案 | [框架思路](../../../../docs/实物实验注意事项/对比试验/解决问题的思路/20260915_轨迹层与MPCC协同框架思路.md)、[代码方案](../../../../docs/实物实验注意事项/对比试验/解决问题的思路/20260915_轨迹层与MPCC协同代码修改方案.md) |

`diag/lt-dwa-collision-tracking` 是此前一直使用的实物分支。旧 Full/Smooth、C03 和 RGB bag 保留当时版本身份，不能算作当前两层方法的实物验证；其近期脚本差异见脚本 README。本轮按用户要求仅本地提交，不 push。

[9月16日复审](../../../../docs/实物实验注意事项/后续改进/20260916_局部规划器复审与避障接入梳理.md)的五项代码修复已落地：共享区域停车尾段检查、所有组车体同 epoch、连续分支投影、到达条件逐拍重检，以及有原始发布审计的无输出回放。核心反例已有定向回归；ROS1的TF、回放接线和实物执行仍需验证。

[再次复盘](../../../../docs/实物实验注意事项/后续改进/20260916_当前主线复盘与剩余缺口.md)已合并两窗口证据：审查确认的运行问题包括停车速度、参考端点、发布时效/改写、定位与液体状态恢复、普通求解失败停车接管及停车周期内峰值漏检。用户确认实物当前运行7d17f6c、既有CAN链路正常；scout_base/SDK与feat无代码差异，CAN断流边界不列近期修复。本轮已按四项收敛范围修复，B0预测高度另有独立旁路评价入口；外部回放配置双传的便捷性问题暂缓。软件回归不代表ROS1整条运行链已验收。

## 当前运行链

```text
原路线 + 显式区域 + 完整初态 + 目标/期限/物理参数
    → 离线全任务优化及重新传播校验 → plan.json
                                               ↓
ROS 状态/IMU/命令历史 → 时间对齐与实际状态前推
原路线/区域/计划 → 持续任务时钟、连续进度与当前停车尾段检查
    → N+1 阶段参考 → 在线 MPCC → 求解后约束核验
    → 候选命令停车尾段核验 / 必要时制动接管
    → 发布契约与安全门 → 最终 cmd_vel / 审计

完整预测末端 → 离线名义后缀重放/重新优化 → 可接续性报告
```

上层采用固定 30 Hz 网格的全状态优化，覆盖车辆、命令、延迟队列、液体和停车尾段。下层用当前真实状态重新预测，自主决定局部几何与速度。后缀诊断目前不进入在线目标或命令，不能提供递归可行性保证。显式区域由任务给出，尚无从地图自动生成区域或在线搜索绕障拓扑的完整链路。

## 四组对比

| Profile | 路线/参考 | 下层自主几何 | 下层液体决策 |
| --- | --- | --- | --- |
| `raw_mpcc` | 原始路线，巡航参考 | 新曲率与目标软代价关闭，保留普通 MPCC 权重 | 全部关闭 |
| `geometry_mpcc` | 原始路线，巡航参考 | 开启、弱 contour | 关闭 |
| `planned_mpcc` | 上层完整计划，progress 参考 | 开启、弱 contour | 关闭 |
| `planned_slosh` | 与 planned_mpcc 同一计划 | 相同设置 | 开启液体代价，硬液面约束默认关闭 |

主比较是 `raw_mpcc` 与 `planned_slosh`：未经新增路径处理的普通 MPCC 对比完整方法。四组共同使用同一空间区域、运动限制、目标容差、期限和评价窗口；共同期限不等于实际耗时匹配。`planned_mpcc` 与 `planned_slosh` 隔离的是**下层液体项**；上层液体贡献需另生成液体目标关闭的计划，不能由这四组直接认定。

## 代码职责与版本

| 位置 | 职责 |
| --- | --- |
| `scripts/planning/` | 任务合同、全程优化、重新传播验证与真实状态后缀诊断 |
| `reference/`（include 与 src） | 原路线、显式区域、完整计划、progress/time 采样与阶段参考 |
| `planning/ocp_planning_adapter` | 运行时与计划合同检查、区域/目标装配、预测核验 |
| `dynamics/`、`scripts/acados/` | actual/command 分离、延迟/FIFO、旋转液体共源核与符号 OCP |
| `core/`、`terminal/`、`warm_start/` | 任务时钟、停车/恢复策略、完整状态热启动及问题编排 |
| `solvers/` | acados 参数装配、RTI、解提取及代价/约束核验 |
| `ros/`、`msg/` | ROS 参数、传感/路径/地图接纳、发布契约和带版本诊断 |
| `scripts/experiments/` | 新四组录制的配置合并、artifact 冻结和 live 参数核对 |

当前配置选择 `solver_backend: continuous_mpcc_acados`、`execution_model.mode: explicit_actuator`；运行诊断 backend 为 `continuous_mpcc_acados_explicit_actuator`。`continuous_mpcc_direct_omega_legacy` 与 `primitive` 保留作历史诊断，不能混入同版本液体消融。

| 合同 | 当前值 |
| --- | --- |
| 网格 | 60 个控制阶段、61 个状态节点，dt = 1/30 s |
| B0 / slosh 状态宽度 NX | 24 / 28 |
| 控制 NU | 3：`a_cmd, alpha_cmd, v_s` |
| 每阶段参数 NP | 92 / 104，以生成 `parameter_names` 为准 |
| 版本 | 液体 1，代价 3；PreSolveSnapshot / PredictedHorizon schema 8 |
| 工具链 | acados v0.5.4，CasADi 3.7.2 |

B0 的 24 维为位姿、实际线/角速度、进度、命令线/角速度、5 拍线速度 FIFO、10 拍角速度 FIFO 和 `a_cmd_memory`；slosh 再追加四维液体模态。`a_cmd/alpha_cmd` 积分命令，实际运动由延迟执行器传播，不能按旧 6/10 维无执行器模型解释。旧 snapshot 参数不得直接喂给本版 solver。

## 配置与运行边界

平台和杯体分别放在 [scout_mini.yaml](config/platforms/scout_mini.yaml) 与 [tube_default.yaml](config/containers/tube_default.yaml)；普通配置在 [common.yaml](config/planner/common.yaml)，新实验覆盖在 [trajectory_mpcc/](config/experiments/trajectory_mpcc/README.md)。实际参数以完整合并配置和 live 私有参数为准，不能只看某一个 YAML 默认值。

`trajectory_mpcc.launch` 启动 planner，默认允许发布速度；传感器、定位、底盘、路径 publisher 和独立液面采集需要已有运行链。区域、正 deadline、planned 组的计划均需显式提供，示例区域只是软件样例。可先用 `publish_cmd_vel:=false` 检查节点接口；完整回放另需录制的最终命令审计，见[无运动回放](scripts/README.md#无运动回放)。区域尾段检查在 `complete_stop.enable=false` 时也生效，制动接管保持至任务重置。

`record_trajectory_mpcc_comparison.sh` 只录制已启动节点，保存合并配置、hash、Git 状态和消息；`VALIDATE_ONLY=true` 只做输入校验并生成 manifest。相机与 NOKOV 入口分别见 [RealSense README](../../sensors/realsense_liquid_measurement/README.md) 和 [NOKOV README](../../sensors/nokov_mocap_monitor/README.md)。内部模态高度和在线视觉 proxy 均需保留来源；模型内降晃不能替代独立液面测量。

构建时须重新生成 B0/slosh solver 并整体重编 ROS1 消息和本包；生成共享库不纳入 Git。无 ROS 的构建和软件复现命令集中在 native README，实物采集命令集中在 scripts README，避免多处命令随版本漂移。
