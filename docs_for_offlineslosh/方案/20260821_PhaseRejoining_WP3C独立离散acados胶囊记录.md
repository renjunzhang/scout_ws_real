# Phase-Rejoining WP3C 独立离散 acados 胶囊记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- 入口基线：`668148be`（WP3B CasADi/C++ 离散转移一致性）
- 对应缺口：IMP-03/IMP-04/IMP-05 的独立 generated optimizer、命令硬约束和 capability gate 切片
- 结论：WP3C 候选 capsule 通过；在线默认不变，terminal 9D gate/$\mathcal B^{\mathrm{exec}}$、formal nominal-relative cost/parameters、在线 history 初态构造和独立 plant 尚未完成；WP2、WP3、B0 和 formal 仍未关闭，状态继续为 G0 NO-GO

## 1. 独立 DISCRETE capsule

新增 `generate_delay_augmented_phase_acados.py`，直接复用 WP3B 的 CasADi `transition_expression()` 构造 acados `DISCRETE` model，不修改现有 10 维、`N=3` development Phase-Rejoin capsule。冻结维度为：

```text
model       = spmpc_delay_augmented_phase
nx / nu     = 22 / 3
N           = 10 = n_f(7) + N_l(3)
integrator  = DISCRETE
NLP         = SQP_RTI
QP          = PARTIAL_CONDENSING_HPIPM
contract    = e198d2a7e0b4d8b2b530e2bfafb33871f3f9f0cb1baab12e6739b6c35191783d
```

本地 acados revision `dc6668f85` 已实际生成并编译：

```text
generated/acados/spmpc_delay_augmented_phase/
  acados_ocp_spmpc_delay_augmented_phase.json
  acados_solver_spmpc_delay_augmented_phase.[ch]
  libacados_ocp_solver_spmpc_delay_augmented_phase.so
  generated dynamics/cost/constraint C sources
```

依照包内既有 codegen 策略，solver C 源和本地编译二进制仍由 `generated/acados/.gitignore` 忽略，不作为跨平台源码真值；版本库冻结生成器和小型 `spmpc_delay_augmented_phase_solver_manifest.h`。无生成 `.so` 时 C++ owner 编译为 stub，不影响包的其他后端。

## 2. 命令和执行硬边界

候选 OCP 已在 generated JSON/C capsule 中建立：

```text
q=[a, alpha, v_s]
a       in [-0.6, 0.6] m/s^2
alpha   in [-1.2, 1.2] rad/s^2
v_s     in [0, 0.8] m/s

robot/actuator v      in [0, 0.8] m/s
robot/actuator omega  in [-1.2, 1.2] rad/s
linear pending[5]     in [0, 0.8] m/s
angular pending[7]    in [-1.2, 1.2] rad/s

u_pub_v     = linear_pending.back() + a*dt
u_pub_omega = angular_pending.back() + alpha*dt
```

`u_pub` 的两个表达在 stage 0–9 作为非线硬约束；14 个 robot/pending-command 状态边界作用于中间和 terminal node，初态 22 维全部固定。因此加速度边界和积分后的 published-command 边界是两套独立硬约束，不依赖发布后 limiter 补救。

当前 external cost 只是让候选 capsule 可数值求解的控制/液体小权重正则项。它还没有接入 formal artifact 的逐 stage nominal-relative cost 和 terminal 参数，不能把当前求解结果写成已完成 Phase-Rejoin 优化目标。

## 3. 严格合同和能力门

新增无 ROS `DelayAugmentedPhaseAcadosSolver`。分配 capsule 之前会逐字段检查：

- execution schema/id/hash、$dt$、双通道 $d/\tau/K$、死区、饱和、整步和 fractional delay；
- 22/3/10 维度、两路 buffer cardinality、$n_f/N_l/N_e$；
- initial/physical-front/grid-front/terminal epoch；
- 初始 robot/slosh/actuator/pending state 的 finite、同步和边界。

manifest 的 WP3C capability mask 为 `0x1f`，只声明：

```text
DISCRETE dynamics
complete augmented initial state consumer
published-command bounds
robot/pending speed bounds
published-command rate bounds
```

formal 所需 mask 还包含 terminal empirical gate 和 execution compatibility set。当前请求 formal mask 会在分配 capsule 前 fail closed，不允许因为“capsule 能解”就提前放行。generated header 的 `NX/NU/N/NP/NBX/NBU/NH` 也通过 C++ `static_assert` 与 manifest 绑定。

## 4. 默认行为与自动化证据

本轮没有把新 owner 加入 `SolverFactory`，没有新增在线 backend key，也没有修改 YAML/launch 默认值。因此现有 off/monitor/enforce 仍使用原 development 链。

新增 3 项 C++ 测试，覆盖 capability 拒绝 formal 提前放行、hash/state/epoch/bounds mutation fail closed，以及生成 capsule 对 held feasible context 的实际创建和求解。新增 3 项 Python 测试，覆盖维度/约束表达、capability mask 和 manifest 确定性。

完整回归结果：

```text
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
  PASS

catkin_test_results build/test_results/spmpc_local_planner
  Summary: 578 tests, 0 errors, 0 failures, 0 skipped

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/scout_apps/control/spmpc_local_planner/test/python \
  -p 'test_*.py'
  Ran 96 tests, OK

git diff --check
  PASS
```

## 5. 下一切片

WP3C 建立了独立 optimizer capsule 和不可越过的 capability gate，但还不是 formal solver。下一切片应：

1. 从在线 source state 和完整 published-command history 构造 expected-publish-epoch `ExecutionHorizonContext`；
2. 将 formal artifact 的逐 stage nominal augmented state/control 作为求解参数和代价真值；
3. 在 terminal 同时加入 9D empirical gate 和 $\mathcal B^{\mathrm{exec}}$ 硬约束后，再升级 capability mask；
4. 完成带约束 optimizer/C++ rollout 一致性、30 Hz 时延统计和独立 plant 故障注入。

在这些条件全部闭合前，formal capability 请求必须继续失败。
