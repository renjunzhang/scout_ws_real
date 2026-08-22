# Phase-Rejoining ACADOS 伴随量诊断与 KKT 合同闭合执行方案

- 日期：2026-08-22
- 工作区：`/home/a/scout_ws`
- 分支：`offline-slosh-plan-online-tracking`
- 当前 HEAD：`92882dc5 阶段性修复：闭合重接候选的执行兼容筛选`
- 任务性质：仿真第 3 阶段的单 seed development 闭环诊断与修复
- 当前结论：**无量纲变量缩放已把 stationarity 从 2.20009 降到 0.015864，但完整 SQP 仍未满足四项 KKT 合同；是否存在伴随量 `pi` 更新或评价问题尚未由持久证据证明。**

> 使用方式：新 agent 应完整阅读本文后再执行。本文不是“继续调参”的授权，而是一个严格限定范围的诊断、修复和验收合同。

## 1. 本轮唯一目标

在完全不改变 P2/V3/recovery/Plant/控制权重和硬约束语义的前提下，查清 seed 8601 cycle 2 剩余 stationarity 地板的来源，并在证据支持时修复 ACADOS 完整 SQP 的伴随量生命周期或残差评价链路。

只有满足以下条件，才能继续运行 seed 8601 两次完整闭环：

1. 同一 cycle 2 快照上 ACADOS 返回成功状态；
2. raw stationarity/equality/inequality/complementarity 四项均满足冻结的 `1e-6` 合同；
3. 独立 C++ KKT 分解与 ACADOS 的逐 stage `res_stat` 一致；
4. 所有具名 stage/terminal primal 硬约束通过；
5. ACADOS raw 状态与独立 causal rollout 最大误差不超过 `1e-6`。

本轮不负责正式 C0--C4 多种子实验，不负责 C1 公平匹配或 C3 消融冻结。

## 2. 开始前：保护现有工作树

当前工作树**预期不干净**。这些修改属于已经形成的 B_exec 后续完整 SQP、诊断和无量纲缩放工作，禁止覆盖。

开始时只读执行：

```bash
cd /home/a/scout_ws
git status --short
git log -2 --oneline
git diff --check
git diff --stat
```

预期 HEAD 为 `92882dc5`。预期未提交内容至少包括：

- 完整 SQP 与 RTI reference 双 capsule codegen；
- 四项 KKT residual 审计；
- raw `x/u` 和具名硬约束诊断；
- codegen 层无量纲变量缩放及 C++ capsule 边界 scale/unscale；
- `perStageStationarity()`；
- 路线文档和 KKT 分量归因文档；
- 本执行方案。

若 HEAD 不同、上述修改消失，或出现无法确认归属的覆盖性改动，立即停止并报告。禁止执行：

- `git reset`；
- `git checkout -- <file>`；
- `git restore`；
- `git clean`；
- 删除或覆盖现有 `/data/a/...` 证据目录。

本轮修改继续保持未提交，等待用户审查；不要 commit，不要 push。

## 3. 必须先阅读的文件

```text
docs_for_offlineslosh/思路/防晃论文的仿真到实物验证思路.md
docs_for_offlineslosh/方案/20260822_PhaseRejoining_KKT站定性分量归因与无量纲尺度推导.md
src/scout_apps/control/spmpc_local_planner/tools/codegen/acados/generate_delay_augmented_phase_acados.py
src/scout_apps/control/spmpc_local_planner/tools/codegen/acados/spmpc_delay_augmented_phase_model.py
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h
src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/solver/acados/delay_augmented_phase_diagnostics.h
src/scout_apps/control/spmpc_local_planner/src/solver/acados/delay_augmented_phase_solver.cpp
src/scout_apps/control/spmpc_local_planner/src/solvers/delay_augmented_phase_online_solver.cpp
src/scout_apps/control/spmpc_local_planner/tools/simulation/run_phase_rejoin_closed_loop_trial.cpp
```

为确认当前 ACADOS 版本的真实语义，还应只读检查：

```text
/home/a/acados/acados/ocp_nlp/ocp_nlp_common.c
/home/a/acados/acados/ocp_nlp/ocp_nlp_sqp.c
/home/a/acados/acados/ocp_nlp/ocp_nlp_globalization_common.c
/home/a/acados/acados/ocp_nlp/ocp_nlp_globalization_merit_backtracking.c
/home/a/acados/interfaces/acados_c/ocp_nlp_interface.c
```

不得直接修改 `/home/a/acados` 安装树。若怀疑 ACADOS 上游行为，只能先形成仓库内最小复现和证据，再报告。

## 4. 已知事实与证据边界

### 4.1 已经可靠确认

- cycle 10 的 phase 10 已由完整 22D B_exec 前置筛选排除，phase 9 被选择；对应提交为 `92882dc5`。
- 旧 RTI 在 cycle 10 的 `nlp_status=0`、`qp_status=0`，但 stationarity=23.2993、inequality=0.306303，最终被 residual gate 正确拒绝。
- 完整 SQP 未缩放版本在 cycle 2 达到 20 次上限：stationarity=2.20009、equality≈1.8e-13、inequality=0、complementarity≈4.7e-6。
- 独立 C++ primal 审计显示上述完整 SQP 解没有 empirical/B_exec/stage 发布约束违反，causal rollout 最大差异约 `1.1e-13`。
- 无量纲变量缩放后，cycle 2 stationarity 降为 0.0158641，equality≈2.09e-12、inequality=0、complementarity≈2.58e-6；仍为 `NLP_STATUS_2`。
- 当前生成 capsule 已恢复为 `SQP + MERIT_BACKTRACKING + GAUSS_NEWTON + print_level=0`。
- 逐 stage `res_stat` 的最大分量位于 terminal stage 10 的 x 位置，数值约 `-1.5867e-2`；它与 terminal cost gradient 数值相等。

### 4.2 尚未可靠确认

以下内容只能作为待验证假设，不得写成最终根因：

- `pi[9]` 或全部 `pi` 在完整 SQP 结束时等于或接近 0；
- QP 已产生非零 dual，但 globalization 没有把它写回 NLP output；
- `ocp_nlp_eval_residuals()` 使用了陈旧的 dynamics adjoint；
- raw ACADOS `res_stat` 不适合作为 KKT 指标；
- 只对 `res_stat` 再做一次归一化即可安全替代当前完整性合同。

关键原则：ACADOS 的 `res_stat` 本身就是 KKT stationarity。若 terminal `res_stat` 恰好等于 terminal cost gradient，只能说明对应的 dynamics/inequality adjoint 没有抵消该梯度；在拿到 dual 与分解前，不能把它解释成“只是跟踪误差，不是 KKT 残差”。

### 4.3 现有证据位置

只读、不得覆盖：

```text
/data/a/spmpc_exec_identification/phase_rejoin_p2_recovery_final_dev_v1_20260822.cElpPI
/data/a/spmpc_exec_identification/phase_rejoin_seed8601_bexec_fix_dev_20260822.5x9pe6
/data/a/spmpc_exec_identification/phase_rejoin_seed8601_sqp_probe_20260822.S9cifq
```

缩放、FUNNEL 和逐 stage 诊断目前只存在 `/tmp/seed8601_*`，属于临时证据，不能作为最终审计输入。新运行必须写到 `/data/a/spmpc_exec_identification` 下的新随机后缀目录，例如：

```text
/data/a/spmpc_exec_identification/phase_rejoin_seed8601_dual_kkt_diag_dev_20260822.<随机后缀>
```

必须保存配置、命令、git HEAD、dirty diff hash、生成 capsule hash、输入资产 hash、seed、summary、CSV 和诊断报告。

## 5. 固定输入，不得修改

```text
Plant:
/data/a/spmpc_exec_identification/phase_rejoin_p2_recovery_final_dev_v1_20260822.cElpPI/inputs/plant_config.yaml

路径:
/data/a/spmpc_exec_identification/phase_rejoin_p2_recovery_final_dev_v1_20260822.cElpPI/inputs/P2_s_curve.json

V3:
/data/a/spmpc_exec_identification/phase_rejoin_p2_recovery_final_dev_v1_20260822.cElpPI/v3/phase_rejoin_p2_v3.csv

C4 development 条件:
/data/a/spmpc_exec_identification/phase_rejoin_p2_recovery_final_dev_v1_20260822.cElpPI/inputs/C4_phase_rejoin_full.yaml

seed: 8601
```

控制器只能使用已有 execution horizon 状态。外部 Plant 液体真值只允许用于 rollout 标签和最终评价，禁止进入 solver 特征、observer、warm-start、dual 初始化、恢复动作或成功判据。

## 6. 第一阶段：把 cycle 2 固化成独立回归快照

不要依赖完整 closed-loop campaign 才能复现。应从当前失败证据中固化一个不读取外部 Plant 液体真值的 C++ snapshot fixture，至少包含：

- 完整 22D initial state；
- 11 个 shooting node 的 64 项参数图像；
- warm-start `x[0:N]` 和 `u[0:N-1]`；
- x0、控制边界、stage 6 项发布约束及 terminal 15 项约束；
- solver/backend ID、缩放向量、执行合同 hash；
- 期望 phase/terminal index；
- 独立 causal rollout 的参考结果。

快照必须能在不启动 Plant 的情况下稳定重放，并能分别选择 `FullSqp` 与 `RtiReference` backend。

快照数据可放在小型测试 fixture 中；不得把 `/data` 大型 CSV 加入 Git。

## 7. 第二阶段：补齐 dual 生命周期诊断

### 7.1 必须记录的 dual

在同一 cycle 2 snapshot 上记录：

- NLP output `pi[0:N-1]`；
- NLP output `lam[0:N]`；
- 每次 QP 解后的 QP `pi/lam`；
- 每个 stage 的 `pi` inf-norm、最大分量索引和值；
- warm-start 前、warm-start 后、第一次 QP 后、globalization 接受 step 后、最终 solve 返回后、显式 residual evaluation 后六个时间点的 dual 摘要。

大体积 dual 只写入失败证据或测试失败输出，正常运行只保留摘要。

### 7.2 必须独立计算的 KKT 分解

不能只读取 aggregate `res_stat`。通过 codegen 产生只读诊断函数，或使用当前 ACADOS 版本稳定公开的接口，独立得到：

> 接口事实提示（2026-08-22 已核对 `interfaces/acados_c/ocp_nlp_interface.c`）：`pi` 与 `lam` 可通过 `ocp_nlp_out_get(out, stage, "pi"/"lam")` 读取；但 `cost_grad`、`dyn_adj`、`ineq_adj` 位于 acados 内部 `ocp_nlp_memory`，**不通过公共 C 接口暴露**。因此「独立分解」只能由自有 CasADi 模型重算（`Jᵀy`、`(∇g)ᵀπ`、`(∇h)ᵀλ`），不能从 acados 直接读这三个量。

- 每 stage cost gradient；
- 离散 dynamics 对 x/u 的 Jacobian；
- stage/terminal constraint Jacobian；
- dynamics adjoint contribution；
- inequality adjoint contribution；
- 独立 stationarity vector。

禁止手工编辑 ACADOS 生成 C 文件。若增加 CasADi 诊断函数，必须从现有生成脚本产生，并由 C++ 测试调用。

在 terminal stage 必须逐分量验证当前版本的符号约定：

```text
res_stat_N            = cost_grad_N - ineq_adj_N          (terminal 无 dynamics，dyn_adj_N 不存在，等价于零)
res_stat_i (i=0..N-1) = cost_grad_i - dyn_adj_i - ineq_adj_i
```

具体到 acados 的内存语义（`ocp_nlp_res_compute`，`ocp_nlp_common.c:3652`）：`dyn_adj` 的 `blasfeo_daxpy` 在长度 `nu[i]+nx[i]` 上执行，terminal `nu[N]=nx[N]=0`，所以 `dyn_adj_N` 是零长度向量，**不是「有待确定的 `dyn_adj_N`」**。因此 §7.1 记录的 `pi[0:N-1]` 与 terminal 无 `pi[N]` 是一致的，不冲突。

执行者必须额外注意的维度约定（与读接口无关，是「独立分解」的固有错位）：

1. `pi[stage]`（`out->pi[stage]`，长度 `nx[stage+1]`）**伴随的是 x[stage+1]**，不是 x[stage]。要把它折算回 `res_stat[stage]`（长度 `nv[stage]=nx[stage]+nu[stage]`），必须先经过 `(∂g_stage/∂x_stage)ᵀ` 与 `(∂g_stage/∂u_stage)ᵀ` 投影，不能把 `pi[stage]` 与 `res_stat[stage]` 按同 index 直接相减。这是本诊断最易栽跟头的一步。
2. `lam[stage]` 长度是 `2*ni[stage]`（含 box 双边界），只覆盖 `out->lam[stage]`。

独立结果与 `ocp_nlp_get_at_stage(..., "res_stat", ...)` 的逐分量误差阈值应区分两种来源：

- 若直接读 acados 内部 memory（布局一致），目标 `1e-10`；
- 若由自有 CasADi 模型重算 `Jᵀy`、`(∇g)ᵀπ`、`(∇h)ᵀλ`，acados 内部 blasfeo 单精度 HPIPM 与自有双精度重算之间存在舍入差异，可放宽到 `1e-8`；不要把 `1e-8` 级的重算 mismatch 误判成逻辑错误而触发 §14 停止。

### 7.3 四类判定

诊断后按证据进入且只进入一个分支：

1. **QP dual 非零，NLP dual 为零或未更新**：定位 QP→NLP step update/globalization dual 更新路径。
2. **NLP dual 非零，但 residual 中 dyn_adj 为零或陈旧**：定位 residual evaluation 前的 submodule/memory refresh 时序。
3. **QP 与 NLP dual 都近零**：检查离散 dynamics Jacobian、QP dual读取方式和生成模型的 adjoint接口；不能直接给 `pi` 填经验值。
4. **dual、Jacobian 和独立分解全部一致，stationarity 仍为 0.0159**：这是完整 SQP 的真实未收敛，立即报告具体分量和迭代轨迹；不要通过新指标宣布成功。

> **§7 实测记录（cycle 2，缩放基，完整 SQP，20 迭代 NLP_STATUS_2）**
>
> 结论：进入 **§7.3 分支 4**（dual、Jacobian、独立分解全部一致，stationarity 为一个真实收敛的跟踪残差）。
>
> 证据链条：
>
> ① `pi`（dynamics 伴随，`out->pi`，长度 22）在 stage 0..9 全部精确 0.0（`argmax=-1`）；`lam`（inequality 伴随，长度 `2*ni`）非零，stage 1/2 最大 `3.3e-3`，终端约 `1.3e-7`。见 `test_delay_augmented_phase_kkt_snapshot.cpp` 的 `perStagePi/perStageLam` dump。
>
> ② terminal `res_stat[0] = -1.586413e-2`，与 `w_x·(x_N−nom_x)/scale_x = 1.0·(−2.379620e-3)/0.15 = −1.586413e-2` 逐位相等（factor 是 `/scale`，与 acados NLS 实测一致，不是 `/scale²`）。
>
> ③ cond_N 判别：把 `qp_solver_cond_N` 从 `N=10` 降到 `5` 重 codegen 并重放，`pi` 变为在 stage 0/2/4/6/8 非零（`8.3e-4…5.7e-3`，奇 stage 仍 0）。证明 costate 确由 QP 求出，满 condensing（cond_N=N=10）时只未写回 `out->pi` —— 这是 condensing 表示层现象，非 QP 真零对偶。**但此项与 terminal 站定性无关**（terminal 无 dynamics）。
>
> ④ 满 condensing 恢复后把 max_iter 从 20 → 60 重 codegen：`stationarity = 0.015867` vs 二十迭代 `0.015864`（几乎不变），terminal x 误差仍 `-2.38mm`。**证明 terminal 0.01586 是收敛后的真实固定点，不是迭代数不足的未收敛。**
>
> 最终判定：terminal stage 的站定性残差 = terminal x 跟踪代价梯度，由 terminal x 与 nominal 之间 `-2.380mm` 的 x 位置误差直接导致，是几何上真实存在的跟踪残差；后续按 §13/§16 评估 GO/NO-GO，而不是通过新指标或「补 pi」伪修复宣告满足。


## 8. 第三阶段：允许的结构性修复

只有第二阶段证据指向明确实现错误时才可修改。

允许的修复范围：

- 修正 capsule wrapper 对 `pi/lam` 的初始化、读取或生命周期；
- 修正 globalization step 后 NLP dual 没有按当前 ACADOS API 更新的问题；
- 修正 residual 评价前必要但遗漏的公开 refresh/evaluate 调用；
- 若证明每周期新建 capsule 使完整 SQP dual 初始化不正确，可加入由同一 nominal warm-start 轨迹及其模型 Jacobian计算的确定性 backward-adjoint warm-start；公共输入仍必须是控制器内部状态，不能读取 Plant 真值；
- 修正缩放基下 dual 的变换或 C++/codegen 边界换算错误。

任何 dual warm-start 都必须：

- 有明确数学推导；
- 在相同 snapshot 上可重复；
- 不改变 primal OCP；
- 不把上一次不同 phase 的 dual 无条件复用；
- phase 跳变、启动和无历史情况有明确失败关闭策略；
- 最终仍以求解后的四项 KKT 和独立分解验收，而不是以“提供过 warm-start”验收。

不允许的“修复”：

- 放宽 `1e-6` raw residual 阈值；
- 删除 stationarity 或 complementarity 检查；
- 把 terminal tracking error 当作替代 stationarity 并直接放行；
- 修改 beta、radii、residual authority、N、权重、V3 或 Plant；
- 增加 SQP 最大迭代数超过 20；
- 反复求解直到偶然通过；
- 吞掉 ACADOS 非零状态；
- 用 stored recovery 掩盖求解失败；
- 先调 LM、HPIPM 容差、正则化或 globalization 参数；
- 手工修改生成 capsule；
- 让 Plant 液体真值进入控制器。

若证据显示是 ACADOS 上游缺陷，应形成最小复现并停止，不要直接修改共享 ACADOS 安装。

## 9. 关于“无量纲 KKT 合同”的严格边界

当前无量纲变量缩放是合理且有明显效果的结构性改进，但不能自动推出“raw `res_stat` 应被替换”。

在以下条件全部满足前，禁止改变 production 成功判据：

1. `pi/lam` 生命周期已被持久证据证明正确；
2. 独立 KKT 分解与 ACADOS raw residual 一致；
3. 已有一个真正紧收敛的参考解，或明确的 ACADOS 上游 residual 语义缺陷；
4. 新合同包含 primal、dual、complementarity 和 stationarity，而不是只检查控制量或跟踪误差；
5. 新合同在 identity scaling 与当前 scaling 下给出等价结论；
6. 对人工注入的错误 dual、错误 primal 和约束违反都能失败关闭；
7. raw ACADOS 四项 residual 继续原样记录。

若确实需要替换 production KKT 合同，这属于成功定义变更。新 agent 必须先停止并向用户提交：公式、尺度变换、反例测试、阈值依据和对现有结果的影响，获得明确许可后才能实现。

不能用“terminal x 误差需要小于 0.15 µm”单独证明 raw stationarity 不合理，因为正确 costate 本应在 KKT 条件中抵消 terminal cost gradient。

## 10. 缩放实现必须补齐的合同

当前缩放改变了 OCP 内部变量基，但 solver ID/config hash 仍未包含该语义。若保留缩放，必须：

- 更新 solver ID 或模型语义版本；
- 让配置/模型 hash 包含完整 `scale_x`、`scale_u`、scaled dynamics/cost/constraint contract；
- 由生成脚本更新 manifest，禁止手工改 hash；
- 记录生成命令与生成输入 hash；
- 验证物理参数图像、V3、gate radius 和 B_exec 的 hash/语义保持不变；
- 增加 scale→unscale round-trip 测试；
- 增加 identity scaling 与当前 scaling 的物理 cost、transition、constraint 等价性测试；
- 增加 raw ACADOS state/control 解码回物理单位后的 causal rollout 测试；
- 增加逐 stage stationarity `[u, x]` 布局测试，terminal stage 明确只有 `[x]`。

## 11. 必须新增的测试

至少覆盖：

1. cycle 2 snapshot 在旧未缩放 backend 上复现 stationarity≈2.2；
2. 同一 snapshot 在当前缩放 backend 上复现 stationarity≈0.0159；
3. scale/unscale 对 22D state 和 3D control 精确 round-trip；
4. 缩放前后物理 transition、cost 和所有硬约束逐项等价；
5. `perStageStationarity()` 的宽度、顺序和 terminal index 正确；
6. dual capture 不改变 solver output；
7. 独立 KKT 分解逐 stage、逐分量匹配 ACADOS `res_stat`；
8. 人工置零/扰动 `pi` 时，独立 KKT 和 residual gate 必须检测失败；
9. 修复后 cycle 2 完整 SQP status=0 且四项 residual≤`1e-6`；
10. 求解后具名 current/terminal execution gate 仍保留；
11. raw solver 状态和 coordinator 状态仍分别记录；
12. legacy/non-augmented selector 和 solver 路径不受影响；
13. 外部 Plant 液体真值隔离测试继续通过。

不要为了让旧人工 fixture 变绿而降低断言或成功标准。

## 12. 构建和验证顺序

构建最多使用 `-j2`。

1. 运行 codegen 自检；
2. 用生成脚本重新生成完整 SQP 与 RTI reference capsule；
3. `make shared_lib -j2` 构建两个 capsule；
4. 构建受影响的 `spmpc_local_planner` 目标；
5. 运行新增 snapshot/scale/dual/KKT 分解测试；
6. 运行聚焦测试：
   - `test_phase_rejoin`
   - `test_control_cycle_engine`
   - `test_delay_augmented_phase_online`
   - `test_delay_augmented_phase_acados_solver`
   - `test_phase_rejoin_closed_loop_trial`
   - `test_delay_augmented_phase_codegen_consistency`
7. `git diff --check`；
8. 在同一 cycle 2 fixture 上做 RTI 与完整 SQP 对照；
9. 只有 cycle 2 验收全部通过后，才运行 seed 8601 完整 development 闭环两次；
10. 两次输出必须写入新的 `/data/a/...` 目录，不覆盖任何旧证据。

## 13. 单 seed 最终验收

两次 seed 8601 都必须满足：

- 不出现 `ACADOS_MINSTEP`；
- 不出现 `RESIDUAL_REJECTED`；
- 不出现非零 NLP/QP status；
- raw 四项 KKT residual 均满足冻结合同；
- 独立 KKT 分解通过；
- 所有 stage 和 terminal 硬约束通过；
- causal rollout 最大误差≤`1e-6`；
- `solver_failures=0`；
- `controlled_stops=0`；
- `sequence_completed=true`；
- `task_success=true`；
- 两次运行结果在固定 seed 下可重复；
- `plant_truth_visible_to_controller=false`；
- `external_liquid_truth_used_for_control=false`。

汇总完整 backend P50/P95/P99/max wall time，并与 30 Hz 的 33.3 ms 比较。若正确性通过但 P99 超过 33.3 ms，只能标记“仿真正确性 GO、实物实时 release NO-GO”，本轮不要继续重构 RTI 生命周期。

## 14. 立即停止条件

出现任一情况就停止修改并报告，不继续试参数：

- 无法用持久证据确认 `pi/lam` 的实际值和更新时间点；
- 独立 KKT 分解无法与 ACADOS 逐分量 residual 对齐；
- 发现缩放前后物理 OCP 不等价；
- 修复一个明确 dual 生命周期错误后，完整 SQP 仍不满足 KKT；
- 出现新的具名硬约束违反或 causal mismatch；
- 需要改变成功定义或 residual 合同时尚未获得用户批准；
- 需要修改 ACADOS 安装树；
- 需要修改 recovery 数据、held-out gate、V3、Plant 或正式实验定义；
- 工作树出现来源不明或与任务冲突的改动。

停止报告必须给出：失败 snapshot、完整参数、raw dual、QP/NLP dual 对照、逐分量 KKT 分解、Jacobian/尺度、迭代轨迹和复现命令。

## 15. 明确禁止事项

- 不运行正式 C0--C4 多种子 campaign；
- 不开始 C1 公平匹配；
- 不开始 C3 消融；
- 不重新生成 recovery 数据；
- 不重新拟合 held-out gate；
- 不修改 V3 半径或 recovery policy；
- 不修改 Plant 参数；
- 不使用 DualSPHysics；
- 不接触实车；
- 不修改论文性能结论；
- 不发起高并发任务；
- 不提交，不 push。

## 16. 最终交付内容

最后更新：

```text
docs_for_offlineslosh/思路/防晃论文的仿真到实物验证思路.md
```

并向用户报告：

1. `pi≈0` 是否由持久回归证据确认；
2. QP→NLP dual 更新和 residual evaluation 的真实时序；
3. terminal x stationarity 的完整 `cost_grad/dyn_adj/ineq_adj` 分解；
4. 根因与实际修复；
5. 缩放前后及修复前后的同 snapshot 对照；
6. raw 四项 KKT 与独立 KKT 结果；
7. seed 8601 两次完整结果；
8. P50/P95/P99/max backend wall time；
9. 修改文件、生成 hash、证据目录和测试结果；
10. 是否具备进入 C1/C3 定义冻结的明确 GO/NO-GO；
11. 所有仍然存在的 NO-GO。

核心原则：**先证明 dual 与 KKT 链条数学上正确，再讨论 residual 合同；不能用重新定义指标替代求解器正确性。**
