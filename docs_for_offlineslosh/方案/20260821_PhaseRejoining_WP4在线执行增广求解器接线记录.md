# Phase-Rejoining WP4 在线执行增广求解器接线记录

- 日期：2026-08-21
- 分支：`offline-slosh-plan-online-tracking`
- HEAD：`cf1e3179`；本文所述修改仍在工作树中，未提交
- 状态：**`nx=22,N=10,np=64` 源码、定向回归和隔离交付矩阵已接通；fresh clone 默认为 stub，真实 capsule 仅限显式未验证开发开关；正式 solver/recovery release 与 C0--C4 仿真仍为 NO-GO**

## 1. 结论

新的 `nx=22, nu=3, np=64, N=10` 执行增广 acados 源码接线已不再只是独立测试骨架。ROS loader、backend policy、solver factory、完整 history context、正式逐阶段参数、terminal 9D gate 与 $\mathcal B^{\mathrm{exec}}$ 均已接线；在本地完成 codegen、重新配置并显式启用 development-only capsule 后，在线周期可调用 `DelayAugmentedPhaseOnlineSolver`，最终命令仍经过唯一 publication transaction。该事实不提供 capsule 二进制身份或 formal release 证明。

默认行为没有改变：

```yaml
solver_backend: continuous_mpcc_acados
delay_augmented_phase:
  enabled: false
  expected_recovery_artifact_hash: ""
```

因此默认仍走历史连续 MPCC；只改 backend 名、只开 enabled、缺 recovery hash、合同/hash/宽度/capability 任一不匹配时都会拒绝配置或求解，不能隐式进入新链。

构建层也默认 fail-closed：即使工作树残留一个被 git 忽略的 capsule，未显式打开开发选项时仍编译 stub，不能让本地残留文件静默改变已安装 wrapper 的 ELF 依赖。

## 2. 实际在线路径

```text
ROS strict params
  → SolverParams / backend policy
  → SolverFactory::makeSolver()
  → DelayAugmentedPhaseOnlineSolver::configure()
  → ControlCycleInputPreparer 构造 expected-publish ExecutionHorizonContext
  → PhaseRejoinCoordinator 准备逐阶段 formal context
  → DelayAugmentedPhaseParameterBuilder 形成 11×64 参数图像
  → DelayAugmentedPhaseAcadosSolver 调用 nx=22 capsule
  → terminal 9D gate + current/terminal execution compatibility
  → ControlCycleEngine 唯一 safety/pipeline/publication transaction
  → 实际 publish receipt 成功后 commit command history 与 phase
```

定向在线周期测试覆盖：

- pre-solve snapshot backend 为 `delay_augmented_phase_acados`；
- state/control/parameter width 分别为 `22/3/64`；
- predicted horizon 含 `N+1=11` 个状态；
- solver capability mask 为 `0xff`，包含 published-command residual hard bound；
- current execution、terminal execution 和 terminal empirical gate 均实际参与决策；
- 只产生一个最终命令真值，sink、result、history 和 telemetry 的 `cycle_id` 与命令一致。

## 3. 正式目标与 admission 边界

每个 stage 的 64 维参数严格按编译 manifest 排列：

- 22 维 nominal augmented state；
- 3 维 nominal control；
- 2 维 nominal published command `u_pub`；
- 2 维 published-command residual bounds；
- 12 个 nominal-relative quadratic weights；
- 9 个 terminal empirical recovery radii；
- 14 个 execution compatibility bounds。

控制 stage 还在 acados OCP 内直接约束线/角 `u_pub` 相对 nominal `u_pub` 的双边 residual；coordinator 的命令检查保留为第二道防线，而不是唯一限幅位置。正式 capability mask 因此冻结为 `0xff`。

求解器和 coordinator 同时校验 execution contract、parameter schema、state/control/horizon 宽度、recovery artifact hash、cost image 与 capability mask。当前状态和终端状态分别检查执行器输出及线/角 pending-command buffer；终端 9 维状态缺失、执行兼容越界或 artifact evidence 不是 `empirical_held_out` 时不会正常接受恢复。

以下输入均有 fail-closed 回归：

- history 不完整、时间倒退或 target-epoch 污染；
- expected publish estimate、execution hash 或 stage cardinality 变异；
- recovery hash 为空/不一致；
- state/control/parameter schema 宽度不一致；
- cost 权重与已审计 parameter image 不一致；
- current/terminal execution compatibility 越界；
- terminal 9 维 gate 不可用；
- development artifact 试图进入 enforce。

## 4. 唯一 codegen、fresh clone 与安装合同

delay-augmented capsule 的唯一完整生成命令是：

```bash
cd /home/a/scout_ws
ACADOS_SOURCE_DIR=/absolute/path/to/acados \
PYTHONDONTWRITEBYTECODE=1 python3 \
  src/scout_apps/control/spmpc_local_planner/tools/codegen/acados/generate_delay_augmented_phase_acados.py
```

该入口同时生成 manifest、C/H/JSON 和 `.so`。`generated/acados/spmpc_delay_augmented_phase/` 整体被 git 忽略，因此 fresh clone 只有已提交 manifest，没有生成 capsule。生成后必须重新配置 CMake；只有本地开发验证才可显式传入：

```bash
-DSPMPC_BUILD_UNVERIFIED_DELAY_AUGMENTED_CAPSULE=ON
```

默认 `OFF` 时 wrapper 是可配置 stub，依赖真实 capsule 的 4 个在线用例 skip，另外 2 个纯 admission/history 用例仍执行；stub 安装不包含 capsule，也不产生 capsule `NEEDED`。显式 `ON` 时，缺 acados 或 `.so` 会在配置期报错，真实 wrapper 与 capsule 会作为同一个安装单元进入 install space。安装验证必须确认：

- capsule build 的 install/lib 中存在 `libacados_ocp_solver_spmpc_delay_augmented_phase.so`；
- 设置 install/lib 与 acados lib 的 loader 搜索路径后，wrapper 的 `ldd` 不得把该 capsule 报为 `not found`；
- stub build 的 wrapper 没有 capsule `NEEDED`，install/lib 中也没有该 capsule；
- 两种 install 均不得包含 source-only `include/spmpc_local_planner/simulation/`。

当前 manifest 没有可信的 capsule source hash 或 binary hash，`.so` 内也没有可独立核验的合同身份。不能用文件存在、ABI static assertion 或一次本地测试替代该身份链；`SPMPC_BUILD_UNVERIFIED_DELAY_AUGMENTED_CAPSULE=ON` 明确只用于开发，正式/机器人交付保持 **NO-GO**。

## 5. 验证结果

当前源码树的最新结果为：Python 117 项通过、1 项 skip；C++ 全量 682 项全部通过。codegen `--check` 验证 `nx=22,nu=3,N=10,np=64,nbx=0,nh=6` 和 capability `0xff`，且该检查不会创建或重写生成文件。

另在两个全新隔离目录完成了交付矩阵：

- 默认 stub：在 acados 路径故意无效且未打开开发开关时仍可配置、构建、测试和安装；4 项底层测试通过，online 为 2 项通过、4 项按预期 skip；安装产物没有 capsule，也没有 capsule 动态依赖；
- 显式 development capsule：10 项定向测试全部通过；100 次 30 Hz 求解的 P95 为 `0.631666 ms`、最大值为 `1.00391 ms`、deadline miss 为 0；源码 capsule 与安装 capsule 的 SHA256 一致，隔离 `ldd` 和 `dlopen` 均通过，且没有回落到工作区 `devel`；
- 两种安装均未带入 source-only 的独立仿真头文件。

这些结果验证当前源码、默认 fail-closed 行为和本地 development capsule 的交付边界，不替代完整 ROS 端到端发布时延验收，也不提供可信 capsule 身份链，更不证明防晃收益。plant smoke/pilot 同样只说明对象和链路可运行，不是 C0--C4 方法对比或防晃正向证据。

## 6. 文档与实际代码差异

接管时路线图和仿真实验方案仍写着“只完成 WP3C 骨架，未进入 factory、缺正式参数与 terminal gate”。这与实际工作树不一致。当前源码和定向回归表明这些接线已经完成；相应当前态文档已同步，WP3C/WP3D 正文则继续作为历史切片保留。

## 7. 尚未完成，不能混称已通过

仓库默认 recovery hash 和 phase artifact path 仍为空，当前只有单元测试构造的 V3 held-out 形态，并没有真实冻结的仿真/实物 recovery release。以下仍缺失：

- 仿真专用 OfflineSloshOCP 完整 nominal、settle 和 zero-hold tail；
- 按 rollout 隔离的 recovery fit/tune/test 数据及 held-out 报告；
- C0--C4/IS 条件绑定、统一 runner 和正式 session；
- 实车多 trial 执行参数与总 lead held-out 验证。

所以“在线接线完成”不等于“正式 Phase-Rejoin 放行”。默认关闭和实物 `enforce` 禁止状态保持不变。
