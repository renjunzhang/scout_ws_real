# Phase-Rejoining 仿真开发参数 v1 与独立 Plant 记录

- 日期：2026-08-21
- freeze id：`SIM_EXEC_PLANAR_R03_DEV_V1`
- 证据等级：**single-trial simulation-development candidate**
- 放行边界：`simulation_only=true`、`formal_robot_release=false`、`real_robot_enforce_allowed=false`
- 正式 C0--C4：**NO-GO，未启动正式 trial**

## 1. planar_r03 只读辨识

源数据始终只读：

```text
/media/a/ZRJ/slosh_bags/20260729_mocap_imu_calib/imu_mocap_planar_r03_153448.bag
SHA-256 faab5c00082b185fe23729d4e957f62be7a727e31266a38f81612aefc04567c1
```

最终重跑目录：

```text
/data/a/spmpc_exec_identification/planar_r03_frozen_v1/
```

工具从 mocap pose 派生车体系 `v/omega`，纯直线和原地旋转段用于拟合，组合 S 形段只做外推检查。0.15 s 平滑窗的 v1 结果为：

| 通道 | delay | tau | K+ | K- | 纯通道 RMSE | S 形 RMSE |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| linear | `0.10167 s` | `0.09011 s` | `0.92503` | `0.91914` | `0.01878 m/s` | `0.02627 m/s` |
| angular | `0.00000 s` | `0.34225 s` | `1.03684` | `1.06283` | `0.05225 rad/s` | `0.10073 rad/s` |

线通道在 0.05--0.20 s 平滑窗敏感性中约为 `delay=0.1000--0.1024 s`、`tau=0.0901--0.0915 s`。角通道 full directional-gain 模型的 delay 接近 0，unity-gain 模型约 `0.009 s`。带 deadzone 的全参数模型只得到很小 RMSE 改善，却产生明显 gain/deadzone/tau 共线，因此 v1 固定 deadzone 为 0。

最终工具和 JSON 与首次结果字节一致：

```text
tool SHA-256   246028cd284618cf70bdad65c1647137ba67f2450adf5c431822c126da8153cd
report SHA-256 347620d78e0f05de4f9f62f1a991955afabf9a922a3ee024dfaad6fdc54033f1
```

提交前审查发现：v1 对离散 `/cmd_vel` 使用了线性插值，会在真实发布时间前生成命令斜坡，并让
`delay` 与 `tau` 吸收这部分偏差。当前脚本已升级为 v2：命令使用因果右连续 ZOH，执行器在延迟后
的 ZOH 事件之间做精确一阶传播，并新增边界、延迟阶跃和初态回归测试。由于源外接盘本轮已不在
系统中，不能伪造一次“修正后重跑”；上表和配置继续明确绑定旧工具 hash，只能作为独立 Plant 的
粗略 stress 参数。下次接回源 bag 后必须用 v2 写入全新目录，不能覆盖 v1。

此外，单 trial 无法估计跨 trial、电量、载荷、地面和安装漂移，且未激励饱和；所以这些数不能
进入实物 `enforce`。

## 2. 仿真开发参数 v1

当前配置身份为：

```text
src/scout_apps/control/spmpc_local_planner/config/simulation/phase_rejoin_development_v1.yaml
SHA-256 81a26b1bd748c07c88a0d4778a2f0eef44a6e8303e3df9525c03423dbab3401f
status development_candidate_unbound
```

`development_candidate_unbound` 是刻意的身份：campaign 只允许仿真开发，不允许把它解释为 formal
资产冻结或实物放行。配置只保留独立 Plant 真正消费的参数；已经删除重复且未消费的
`controller_internal_model` 和 `real_robot_launch_overrides`。辨识来源改为逻辑 ID 和 hash，明确
`provenance.runtime_verified=false`；runner 会记录这些字段，但不会假装已在运行时重新打开并核验
源 bag、辨识报告或工具。

控制器内部模型的唯一事实源是生成的 solver manifest，而不是仿真 YAML 的副本。记录时 manifest
身份为 `delay_augmented_phase_acados_online_v2 / delay_augmented_phase_parameter_image_v2`，线/角
执行模型为；生成头文件记录时 SHA-256 为
`39e3c3b8e0b881105724f4e416319728053e32670bbba15548eaa40a56940ba0`：

| 通道 | delay | tau | gain | output |
| --- | ---: | ---: | ---: | ---: |
| linear | `0.15 s` | `0` | `1` | `[0, 0.8] m/s` |
| angular | `0.22 s` | `0` | `1` | `[-1.2, 1.2] rad/s` |

external Plant 保留倒车能力，并故意与控制器不同：

| 通道 | delay | tau | K+ | K- | output |
| --- | ---: | ---: | ---: | ---: | ---: |
| linear | `0.102 s` | `0.091 s` | `0.925` | `0.919` | `[-0.8, 0.8] m/s` |
| angular | `0.010 s` | `0.342 s` | `1.037` | `1.063` | `[-1.2, 1.2] rad/s` |

角通道 `0.010 s` 是仿真开发用的保守 stress 值：使用 unity-gain 结构看到的约 9 ms delay，
同时保留 directional-gain 结构的 tau/gain；它不是“正式辨识点估计”。液体 Plant 使用偏移后的
主模态参数和一个额外二阶模态，也不声称 CFD 真值。

## 3. 最小 C++ 独立 Scout＋液体

独立库只消费最终发布命令，不读取 solver 预测状态，包含：

- 线/角独立的 delay queue、tau、方向增益、deadzone 和 saturation；
- 有界 command transport jitter；
- Scout 平面运动学；
- 参数偏移的主液体模态和额外二阶模态；
- 有明确单位的线/角加速度过程扰动与液面测量噪声；
- 固定 seed 后，在同一冻结配置、同一 executable 与同一 STL/浮点环境内可精确复现；
- CSV 原始状态与 JSON smoke 指标。

接管审查和实跑共纠正了四类随机/时间因果问题：

1. 初稿把过程噪声按积分子步直接加到速度，命令事件切步曾产生约 `282 m/s²` 假加速度；现改为
   `m/s²`、`rad/s²` 扰动，并按固定 `2 ms` 物理噪声时钟推进。
2. jitter、线过程、角过程和液面噪声使用五个独立 RNG 流；正态采样器不跨流共享缓存。
   新增只读 disturbance diagnostic，测试可直接比较同一 seed、同一物理时刻的 interval index 和
   三项外生扰动，证明冗余命令事件不会重采样噪声。
3. tail 不再从 7.5 s 命令边界直接开始。先找到第一个不早于 7.5 s 的控制周期并发布零命令，
   再等待 `max(linear_delay, angular_delay) + jitter_limit`，之后才开始固定尾窗。
4. `publishCommand` 先在 jitter RNG 副本上生成线/角事件；若 effective time 导致 queue reorder，
   整个发布失败且不提交 RNG、queue 或上次发布时间。因此一次被拒绝的发布不会改变下一次
   成功发布看到的随机实现。

配置合法性还明确约束：某通道 `tau=0` 表示代数直达，此时该通道的过程噪声必须为 0；
`tau=0` 且过程噪声大于 0 的配置会被拒绝，避免一个没有动态状态的通道仍“消费”加速度噪声。

当前 CSV 时间契约为 `publish_sample_effective_epochs_v1`：每行分别记录
`publish_time_sec` 和 `sample_time_sec`，并记录线/角两路的 `effective_time_sec` 与
`transport_jitter_sec`。campaign 逐行检查时间单调性、`publish <= sample`、jitter 边界和
`effective = publish + delay + jitter`，不再用一个含义模糊的 `t` 同时代表发布与采样时刻。

`完全复现` 在本记录中只表示上述同一运行环境内的字节/数值复现。C++ 标准不要求
`std::normal_distribution` 在不同 STL 实现中生成位级相同的序列，因此跨 libstdc++/libc++、编译器或
浮点环境只能要求同一统计契约，不承诺文件 SHA-256 一致。

smoke CSV/JSON 使用原子 create-new 预留，已存在任一输出时拒绝覆盖。campaign 也拒绝复用目录，
并在创建目录前完成配置、可执行文件和 seed 预检；运行中失败则保留新目录作为 partial 证据。

当前 11 项 C++ 测试全部通过，除上述原有边界外，新增覆盖 `tau=0 + process noise`
拒绝、publish/effective/jitter receipt 精确关系，以及 queue reorder 失败不消费 RNG。Python 共
13 项测试全部通过（未注入 C++ smoke 路径时集成项会跳过），覆盖严格 `uint32` seed、v3 CSV
时间契约、缺失 formal 资产仍 NO-GO，以及绑定完整时可达但不执行 trial 的
`READY_NOT_EXECUTED`。注入当前 C++ smoke 后 13 项均实际执行并通过。

## 4. 已生成的 Smoke 与 development pilot v2

本次所有输出均写入全新目录，没有覆盖旧 v1 结果：

```text
smoke /data/a/spmpc_exec_identification/independent_plant_smoke_v2_20260821T122947Z/
repro /data/a/spmpc_exec_identification/independent_plant_smoke_repro_v2_20260821T122947Z/
pilot /data/a/spmpc_exec_identification/independent_plant_pilot_v2_20260821T122947Z/
```

这些是升级前已冻结的 v2 历史产物，不会追溯改名成 v3。当前代码产生的任何新 smoke/pilot
必须使用新目录和 v3 schema，不得与下列 v2 CSV/JSON 按同一 schema 合并。

固定 30 Hz、4 s tail 的实际窗口为：

| 字段 | 值 |
| --- | ---: |
| motion command end | `7.500 s` |
| first zero publication | `7.500 s` |
| tail start | `7.606 s` |
| end | `11.606 s` |
| total samples | `349` |
| tail samples | `121` |

单次 smoke（seed 1001）：

| 指标 | 结果 |
| --- | ---: |
| external Q95 | `1.461878 mm` |
| external peak | `3.411320 mm` |
| external RMS | `0.696018 mm` |
| tail RMS | `0.514813 mm` |
| max `|v|` | `0.139084 m/s` |
| max `|omega|` | `0.371228 rad/s` |

独立 repro 目录中的 seed 1001 CSV/JSON 与首个 smoke 字节一致：

```text
CSV  SHA-256 236210ba855dacc048dbe7441748f37a8d51e79d1e49ab17a06d459d8012fd46
JSON SHA-256 0f86ddd835fa9432c25082ca9670868100683bd038eebdef51beeb46ca94c37d
```

8-seed pilot（2101--2108）：

| 指标 | min | median | max |
| --- | ---: | ---: | ---: |
| Q95 [mm] | `1.477684` | `1.490931` | `1.532167` |
| peak [mm] | `3.476925` | `3.488892` | `3.598975` |
| RMS [mm] | `0.701880` | `0.707639` | `0.716309` |
| tail RMS [mm] | `0.514721` | `0.525410` | `0.541811` |

smoke seed `1001`、pilot seeds `2101--2108` 和保留的 formal seeds `3101--3116` 两两互斥，
formal 表在配置中锁定。所有 trial 均标记：

```text
status=COMPLETE_DEVELOPMENT_SMOKE | COMPLETE_DEVELOPMENT_PILOT
completion_scope=execution_and_artifact_integrity_only
effect_claim=false
formal_method_comparison=false
```

所以这些数只证明固定激励下的 Plant 运行完整性、有限性、随机分离度和可复现性。它们不比较任何
控制方法，不能据此说 Phase-Rejoining 防晃有效。旧 v1 数值和目录继续保留，但因噪声流与 tail
语义已改变，不得与 v2 混合作为同一 campaign。

## 5. 正式仿真为什么仍是 NO-GO

最新只读 readiness 结果为：

```text
/data/a/spmpc_exec_identification/formal_simulation_readiness_v2_20260821T122947Z/formal_readiness.json
status=NO_GO
formal_trials_started=false
runner exit=4（预期的未放行结果）
```

实际缺口为：

1. 当前配置仍是 unbound development candidate，没有 formal session/asset binding；
2. C0--C4/IS 条件没有 runtime binding；
3. `expected_recovery_artifact_hash` 为空；
4. `phase_rejoin/artifact_path` 为空；
5. held-out execution compatibility/recovery gate 报告不存在。

因此本轮完成的是 identification、参数候选、独立 Plant、smoke 和 development pilot；没有正式
C0--C4 结果，也没有防晃改善结论。`COMPLETE` 只表示程序和产物契约完整，不是效果 `PASS`。

formal runner 的状态机还区分两种“没有开始 trial”：任一必需资产、C0--C4/IS binding、
runtime provenance 或 hash 缺失时仍为 `NO_GO`；只有审计输入全部完整时才返回
`READY_NOT_EXECUTED`。后者只证明 readiness 分支可达，依然返回非零且不调用 trial 子进程，
不等于已放行或已运行正式仿真。

## 6. 可追溯身份

上述已生成的 development v2 campaign 历史产物绑定：

```text
config SHA-256     81a26b1bd748c07c88a0d4778a2f0eef44a6e8303e3df9525c03423dbab3401f
executable SHA-256 846e995de51ee99eb77960bbce866584b3ec249c06ad3e33a687d7314c527b82
smoke schema       spmpc_independent_plant_smoke_v2
campaign schema    spmpc_independent_plant_campaign_v2
```

当前源码与测试冻结的新输出契约为：

```text
smoke schema       spmpc_independent_plant_smoke_v3
campaign schema    spmpc_independent_plant_campaign_v3
CSV time contract  publish_sample_effective_epochs_v1
```

v3 必须在 manifest 中重新记录当次 config/executable/STL 环境和各产物 hash；不得复用上面的
v2 executable hash，也不得把跨 STL 的同 seed 视为位级复现承诺。

每个上述输出目录均含独立 `campaign_manifest.json`，记录配置、可执行文件、每个 CSV/JSON 的路径
和 SHA-256。对已有 smoke campaign 目录的复用尝试返回 `rc=2`，原 manifest 前后 hash 均为
`14b8c0a8629e415e80563492d3ccfb438f3f3de4f256e44e4dbcdb172b429372`。

外接盘只读取源 bag；本轮报告、CSV、JSON 和 manifest 均写入
`/data/a/spmpc_exec_identification/` 下的 create-new 目录。

## 7. 与旧 DualSPHysics 尝试的关系

旧尝试的工具链和液体单独验证后来达到 development-only PASS，但小车耦合、方法比较和实物保真
没有闭合。详细复盘见[旧 DualSPHysics 尝试复盘与当前仿真主线边界](../仿真实验/旧DualSPHysics尝试复盘与当前仿真主线边界.md)。当前独立 Plant 的职责是先以低成本完成完整的
C0--C4/IS 控制证据链；它不替代 CFD。若低阶主比较通过，再决定是否用 DualSPHysics 对少量
预注册代表轨迹做离线复核。
