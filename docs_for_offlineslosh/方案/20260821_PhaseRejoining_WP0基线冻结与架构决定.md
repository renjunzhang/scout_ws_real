# Phase-Rejoining WP0 基线冻结与架构决定

- 日期：2026-08-21
- 状态：Accepted
- 对应方案：`20260821_PhaseRejoining正式方法闭环实施方案.md`
- 冻结 revision：`8d577e9cadec377acd023185fa1db3bddd193283`
- 放行状态：**development simulation release；formal 实物闭环 NO-GO**

本文只冻结进入 WP1 前的可复现行为、资产身份和架构决定，不把 development artifact、零延迟 proxy 或单元测试升级为正式防晃证据。

## 1. 冻结基线

### 1.1 版本化对象

| 对象 | Git tree/blob ID |
| --- | --- |
| acados codegen 源 | `5b2ce24b2bf8889da39c7298b9f02b414ac1c092` |
| `generated/acados` | `679667a272ef6ff42be957c14d0907aaa5a1b03e` |
| planner config | `773f6f5c2ea891ec5e5c91f3b728b79d4bd31bb8` |
| ROS launch | `c71ca9f94fbf19952d9e167b35e549326140f6c6` |
| `control_cycle_engine.csv` golden | `7fe9f8218527c0a6653c6d8c3575a07f7cc16316` |

主线 `b0`、`slosh` 和 `phase_rejoin` 已按 README 顺序重新生成；生成后工作树无差异。WP0 同时修复了短窗生成覆盖共享 `kMainHorizonSteps` 的顺序依赖，并以 Python 单测锁定主窗 60、短窗 3 的双 horizon 合同。

### 1.2 测试结果

在本工作区执行：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
catkin_make run_tests_spmpc_local_planner -j1
catkin_test_results build/test_results/spmpc_local_planner
  Summary: 504 tests, 0 errors, 0 failures, 0 skipped

python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 92 tests, OK
```

C++ 结果对应与 `8d577e9c` 相同的 C++/generated tree；新增的第 92 项 Python 测试验证 codegen 顺序无关性。

### 1.3 development 输入资产

| 资产 | 位置 | SHA-256 |
| --- | --- | --- |
| Gazebo 地图 | `/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream` | `fd065fcc95b1ed2c25dd355b8c312b0d2f84d96e379d137b67997bd225f8ead0` |
| MBF 路径 bag | `/data/a/scout_sim_replacement/logs/phase_rejoin_mbf_20260820/path_capture_turn_seed4200/mbf_path_capture.bag` | `3f96f56b10548e075e3cf54e67454af85407933b84e3fd0367747b23416426f2` |
| development v2 artifact | `/data/a/scout_sim_replacement/logs/phase_rejoin_mbf_20260820/artifact_v2/mbf_global_4x2_complete_tail_dev_v2.csv` | `2f73e4a3c9ed706de2f7dea414e5ac5dd29e55495f11e3ab79a94ed79a8e1ef5` |

这些绝对路径只用于复现当前机器上的历史 characterization；WP4 必须用 typed session 的相对资产绑定和 hash 取代它们。

### 1.4 `off/monitor/enforce` golden 证据

历史三组零延迟 proxy 位于：

```text
/data/a/scout_sim_replacement/logs/
phase_rejoin_refactor_92cd2eac_20260821_rerun4
```

每次运行的 `run_manifest.txt` 保存提交、seed、路径和 artifact 绑定；三份 paired result 的 SHA-256 为：

| seed | paired result SHA-256 |
| ---: | --- |
| 4220 | `7c0ba65802bd9df87a121136dbe792af4c203f995044898d3863655db4b05b7c` |
| 4221 | `42964df2466cbd2cac4289416f28dc3f0852f0045aa2aaa56f489bdb167de42c` |
| 4222 | `6516e0a3ec57e008c680c26cf29b18ccd338931537a2aac2660ea9c1e5f71531` |

ROS launch 的展开真值由冻结的 launch/config tree 与六份 `run_manifest.txt` 共同定位。WP4 完成前不得把 Shell/launch 环境视为正式参数真源。上述运行实际为零纯延迟、零时间常数和瞬时跟随 proxy，只冻结旧行为，不验证 Scout 执行因果性。

## 2. ADR-PR-001：正式方法架构决定

### 2.1 显式 delay-augmented OCP

正式主线采用显式增广 solver。当前新发布命令必须作为决策量进入线、角两路 pending buffer，再传播执行器、车体和液体状态。旧 history-only predictor 只保留作 development/off/monitor 诊断。

只有显式模型经稀疏结构和 codegen 优化后仍无法满足冻结的控制 deadline，才允许采用数值等价的凝聚 bridge；切换必须有独立 ADR 和逐节点/Jacobian 等价证据。

### 2.2 唯一命令发布事务

一次控制周期只能产生一个 `FinalCommand` 并调用一次 `ICommandSink::publish()`。limiter、执行合同和安全硬包络只能在 sink 之前执行一次。只有成功 `PublicationReceipt` 才能更新 command history、limiter 已发布状态和 Phase-Rejoin progress。

ROS adapter 只实现消息转换、时间获取和 sink 交付，不得在 engine 返回后再次限幅或替换命令。

### 2.3 同一执行模型

在线 solver、OfflineSloshOCP、独立仿真 plant、实物标定和 actual-input replay 共享一个版本化 `ExecutionModelContract`。`d_c/d_v/d_omega/tau/K`、fractional delay、deadzone、saturation 和有效域不得在 YAML、Shell、C++、CasADi 中分别定义不同语义。

### 2.4 formal 严格配置

`ResolvedExperimentSession` 是正式运行的唯一真值。formal 模式拒绝未知、缺失、重复、非有限、越界或需要 clamp 的方法参数，拒绝 env/CLI 方法覆盖和隐式默认。preflight、runtime ACK、recorder 和 postflight 必须使用相同 session hash。

## 3. 参数权威来源迁移

| 参数/合同 | 当前 development 来源 | 计划中的唯一正式所有者 | 关闭工作包 |
| --- | --- | --- | --- |
| 最终 `u_pub` | engine decision + ROS `finalizeCommand()` | `CommandPipeline` + `ICommandSink` receipt | WP1 |
| `d_c` 与预计发布时间 | 未建模 | `PublishLatencyModel` | WP2 |
| `d_v/d_omega/tau/K` | YAML + launch/Shell override | `ExecutionModelContract` artifact | WP2/WP4 |
| pending buffer/actuator state | 不完整 | `ExecutionAugmentedState` | WP2 |
| solver horizon/stage epoch | YAML + generated manifest | formal solver execution horizon context | WP3 |
| session/asset hash | run manifest + Shell | `ResolvedExperimentSession` | WP4 |
| 正式 PASS/FAIL | 分散脚本和人工检查 | C++ preflight/postflight | WP4 |
| G0 参数和有效域 | 尚未冻结 | execution calibration artifact | WP5 |
| nominal/gate/B_exec | development v2 CSV | formal loader + release manifest | WP6 |

## 4. 缺口追踪矩阵

| ID | 主模块 | 最低自动测试 | 证据/报告 | 状态 |
| --- | --- | --- | --- | --- |
| IMP-01 | `controller/command`, `control_cycle_engine`, ROS sink | fake sink、发布失败、limiter 改写、phase commit 时序 | cycle audit/replay | WP1 待做 |
| IMP-02 | `runtime/timing` | 预计/实际发布、deadline、时钟倒退 | `d_c` 误差报告 | WP2 待做 |
| IMP-03 | `runtime/execution_prediction`, solver input | 第一拍线/角因果 Jacobian | B0 因果报告 | WP2/WP3 待做 |
| IMP-04 | fractional-delay kernel | impulse、step、13.3 ms remainder | 连续事件对照 | WP2/WP3 待做 |
| IMP-05 | formal acados solver | 随机逐步一致性、terminal epoch、deadline | solver benchmark | WP3 待做 |
| IMP-06 | OfflineSloshOCP tool/loader | schema、完整尾段、确定性重建 | nominal artifact report | WP6，依赖 G0 |
| IMP-07 | `execution_compatibility_set` | buffer/actuator 内外边界 | B_exec coverage | WP6，依赖 G0 |
| IMP-08 | empirical gate evaluator | trial 隔离、confusion matrix | held-out report | WP6，依赖 G0 |
| IMP-09 | typed session/preflight/postflight | mutation、hash、ACK、topic 完整性 | session/postflight report | WP4 待做 |
| IMP-10 | independent plant | 非零延迟、偏差、噪声、故障注入 | C0-C4 双通道报告 | WP3/WP7 待做 |
| IMP-11 | Scout/IMU/RGB 标定链 | held-out replay、第一拍灵敏度 | G0 GO/NO-GO | WP5，需实物数据 |

## 5. WP0 退出核查

- [x] 文档、离线分析与实施方案已从执行链修改中独立提交；
- [x] 干净 revision 可重生成三套主线 solver，生成后无 Git 差异；
- [x] C++ 504 项与 Python 92 项通过；
- [x] 旧控制周期 golden 和三组固定 seed characterization 有可定位 hash；
- [x] 四项架构决定与关键参数未来权威来源已冻结；
- [x] IMP-01 至 IMP-11 已映射到模块、测试、报告和工作包；
- [x] formal 状态保持 NO-GO。

因此 WP0 可以关闭，下一提交从 IMP-01 的 characterization test 开始。WP1 完成前不得删除旧 golden；若必要行为变化，必须同时保留“旧行为为何被替换”的审计记录。
