# 唯一主线 Stage 3-D2d：模块化 CasADi 符号图

> 日期：2026-09-03
>
> 分支：`spmpc-mainline`
>
> 起始提交：`b8d2282 文档：记录数值模型真值并推进唯一主线`
>
> 状态：`GRAPH_BUILT / NO_ARTIFACT / CONSTRAINT_VALUES_DEV_UNVALIDATED`

## 1. 本阶段解决的问题

Stage 3-D2c 已有 backend-neutral 数值真值，但还没有可交给 acados OCP
装配层的固定维度符号表达式。D2d 完成的边界是“可检查的 CasADi graph”，不是
solver artifact：

- 输入固定为 `NX=48/NU=3/NP=162/N=60`；
- dynamics、reference、stage/terminal cost 和 constraints 与 D2c 数值 oracle
  同构；
- 顶层导入不加载 CasADi 或 `acados_template`；
- 不创建 `AcadosOcp`，不 codegen，不写 C/header/动态库/manifest。

因此本阶段可以在没有 bag、ROS、实车门禁或 production 参数的情况下完成；七个
显式约束值仍是 `EXPLICIT_CALLER_SUPPLIED/DEV_UNVALIDATED`，不能据此晋级生产。

## 2. 模块边界

符号路径没有堆进单一生成脚本，而是拆成五个窄模块：

```text
casadi_adapter.py
  lazy dependency + typed source validation + graph orchestration
        |
        +-- casadi_graph_contract.py
        |     immutable bundle + metadata/status/semantic identity
        +-- casadi_dynamics.py
        |     issue + three-slot FOPDT/pose/liquid/progress/queue map
        +-- casadi_reference.py
        |     normalized cubic + chain-rule tangent + MPCC errors
        +-- casadi_objective.py
              stage cost + terminal cost + h/idxbu/xi contracts
```

后四个模块都由 adapter 注入 `ca` backend object，自身不直接导入 CasADi。
数值 oracle 也不调用这些 symbolic builder，避免 parity 测试变成同源自证。

## 3. 完整离散图

`build_casadi_graph(capacity, development_layout, constraint_bounds)` 只接受 D0/D1
canonical typed authority；参数布局由这两个输入重新构建，state/control/parameter
offset 不在符号代码中维护第二份数字表。

单拍传播固定为：

```text
x_k/u_k
  -> pre-issue q_issue/a_issue
  -> taps = [issued, q_prev, older...]
  -> 三个固定物理时间槽的 selector target
  -> rho=exp(-z), 1-rho=-expm1(-z) 的解析 FOPDT
  -> midpoint actual 位姿传播
  -> actual(t) 激励的液体 RK4
  -> s += duration*v_s
  -> publisher 更新与旧 q_prev queue shift
  -> x_{k+1}
```

三个槽始终符号展开，不对 SX duration 使用 Python 分支。零时长尾槽用 CasADi
`if_else` 显式选择原 physical state，保持与数值 oracle 的 identity 语义；进度
增量同时为零。FOPDT 保留 `expm1` 小量稳定形式，并按数值 oracle 的
`steady_state=gain*target` 运算顺序构造。

## 4. Reference、代价与 terminal

stage reference 与 terminal reference 分别进入自己的语义容器，二者都只消费
相应节点传入的 `x/p`：

- stage：`x_k/u_k/p_k`；
- terminal：`x_N/p_N`，表达式树中不存在 `u`；
- stage 与 terminal 分别暴露 `xi_k`、`xi_N` 的 `[0,1]` 域约束；
- 不 clamp、不在图中静默外推；
- stage liquid running 使用同一个 `x_next`；
- boundary 使用当前 `x_k`；
- terminal 只有四项机器人 cost，液体 terminal 恒等于零。

普通 stage cost 仍拆成 `robot_running/liquid_running/liquid_boundary/total`，便于
后续 wrapper 对实际 acados objective 做逐项重建。

## 5. Acados-ready 约束接口

本阶段输出三个明确分层的接口：

```text
stage nonlinear h order
  = [q_issue_v, q_issue_omega, a_issue, alpha_issue]

control box order / idxbu
  = [j_issue_v, j_issue_omega, v_s] / [0,1,2]

reference domain
  = stage xi_k in [0,1]
  = terminal xi_N in [0,1]
```

`h` 的上下界由四个显式 symmetric limit 构造；control box 为两个 symmetric jerk
bound 与固定 `v_s>=0`、显式 `v_s_max`。D2c 的 14 项 lower/upper residual 继续
保留，但 metadata 明确标为
`PARITY_AND_DIAGNOSTICS_ONLY_NOT_ACADOS_H`，不会冒充完整 nonlinear `h`。
B0/Bslosh 均无液体 hard constraint。

## 6. Graph 身份与 runtime 值边界

bundle metadata 记录：

- model/discretization/reference/cost/constraint schema；
- capacity raw hash、development layout hash、parameter layout hash；
- `N/N+1/NX/NU/NP`、精确 release period 和全部 ordered names；
- stage/terminal reference domain、nonlinear h、control box、diagnostic residual
  的顺序和固定结构；
- terminal `NO_U_N_ACCESS`；
- `GRAPH_BUILT/NO_ARTIFACT`；
- 实际 caller-supplied bounds snapshot 及其独立 SHA256。

`graph_semantic_sha256` 只覆盖 typed graph structure，排除某次 trial 的 runtime
`p[k]` 数值和显式 bound 数值；后二者变化不伪装成图结构改变。固定的
`xi∈[0,1]`、`v_s>=0`、h/control/residual 顺序与政策均进入 graph hash。runtime
参数必须先由 canonical assembler 校验再注入，但 graph 内不重新计算 delay、
`m/beta` 或 selector。

## 7. 依赖与失败行为

adapter 先校验 typed capacity/layout/bounds 和可编码身份，再 lazy import CasADi。
因此错误 typed authority 不会被“本机缺 CasADi”掩盖。缺依赖时抛
`CasadiDependencyError`；表达式构造或内部顺序漂移统一包装为
`CasadiGraphConstructionError`。

隔离临时目录测试同时覆盖缺依赖和成功建图路径，二者都不产生文件。即使本机
存在 `libacados.so` 或 `acados_template`，D2d 也不会将其解释为 artifact 已生成。

## 8. 分阶段提交

```text
604dfcc 主线合同：集中模型身份与符号约束顺序权威
e2e2958 主线生成：建立模块化CasADi符号图与数值同构验证
```

## 9. 当前验证

- mainline Python discovery：142/142；
- adapter（当前 CasADi 环境）：8/8；
- 显式 `PYTHONNOUSERSITE=1` CasADi 3.7.2 环境：8/8；
- Stage 2 execution golden：10/10；
- Stage 2 history/prefix/projection golden：14/14；
- mainline Ruff：通过；
- D2d 文件 Ruff format：通过；
- Python 3.8 grammar 扫描：通过；
- `git diff --check`：通过。

CMake 已登记 `test_mainline_stage3d2d_casadi_adapter`。测试在缺 CasADi 的普通
环境只跳过真实 graph 数值项，lazy import、typed authority 和明确失败路径仍执行。

## 10. 下一阶段

下一阶段进入 Stage 3-D3，但仍分开权限：

1. 先建立纯内存 `AcadosOcp` 装配合同和 solver option snapshot；
2. 验证 discrete dynamics、external stage/terminal cost、h/idxbu/xi 的节点覆盖；
3. 再单独实现真实 codegen、统一 `model_contract.json`、generated header 和
   artifact hash；
4. artifact 保持 `DEV_UNVALIDATED`，后续 replay/仿真/性能/实参验证通过后才进入
   Stage 3-P；
5. wrapper、cost diagnostics、warm start 和 ROS 接入继续独立提交，避免生成器、
   runtime 与发布路径重新耦成一个文件。
