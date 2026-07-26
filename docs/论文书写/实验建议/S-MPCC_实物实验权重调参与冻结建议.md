# S-MPCC 实物实验权重调参与冻结建议

> 文档定位：本文档用于正式 88 次实物实验开始前的控制器开发、pilot 筛选和参数冻结。pilot 数据不计入正式样本，不与后续正式结果混合。
>
> 上位协议：`docs/论文书写/实验章节设计/S-MPCC_experimental_design.md` 和 `docs/论文书写/实验章节设计/SPMPC实验矩阵设计.md`。若本文档与正式 freeze artifacts 不一致，以正式归档配置为准。
>
> 当前执行口径：已与现场协议 `SMPCC-REAL-40-88-v1.0`（`0717_S-MPCC正式实物实验矩阵_先40后88.md`）同步。P3 固定为 H0 上 W1/W2/W5 的 15 次无 RGB 模型侧筛选；W10 不进入 v1.0；正式 RGB 只承担独立物理验证，不能反向选权。
>
> 本轮权重只对当前已实现的液体动力学、状态传播和 slosh cost 有效。若后续引入 rotation-consistent dynamics、相位能量/有符号功率代价或其他方法结构变更，必须新建 release 并重新执行独立 pilot，不得沿用本轮 `FREEZE_ID`。

## 1. 核心目标

正式实验前需要找到一个能够让 Baseline MPCC、Smooth-only MPCC 和 S-MPCC 呈现出清晰机制差异的实物工作点。调参目标应表述为：

> 在任务、安全、实时性和执行层准入条件下，选择能稳定产生液体状态相关规划行为和内部模型响应的最小拐点权重；真实液面改善留给后续正式 RGB 独立验证。

不应把目标表述为“选择使 S-MPCC 比 baseline 好得最多的参数”。差距应来自高风险工况下的机制作用，而不是故意削弱 baseline、选择性查看结果或过度牺牲任务性能。

## 2. 推荐的单参数：`w_slosh`

建议把论文中的模态位移权重 \(w_\eta\)（代码配置中的 `w_slosh`）作为主要单参数扫描对象。S-MPCC 的液体代价为

\[
J_{\mathrm{slosh}}
=w_\eta
\frac{\lVert\boldsymbol\eta\rVert_2^2}{\eta_{\mathrm{ref}}^2}
+\rho_{\dot\eta}w_\eta
\frac{\lVert\dot{\boldsymbol\eta}\rVert_2^2}
{\dot\eta_{\mathrm{ref}}^2},
\qquad
\rho_{\dot\eta}=0.3.
\]

选择 `w_slosh` 的原因是：

- Baseline MPCC 和 Smooth-only MPCC 中 `w_slosh=0`；
- S-MPCC 中 `w_slosh>0`，它直接调节本文特有的液体动态代价；
- 保持 \(\rho_{\dot\eta}\) 和所有非液体权重不变时，扫描只引入一个可解释因素；
- 它比调节 `v_ref`、tracking 权重或执行层 limiter 更少引入完成时间、跟踪和实际执行混杂。

## 3. 调权前的强制前置验收

在执行大规模 `w_slosh` 扫描之前，必须先确认以下项目。否则所谓“最佳权重”可能只是在补偿尺度错误、延迟错误或未生效配置。

### 3.1 模态速度尺度统一

确认 C++ runtime、Python/codegen 和论文公式统一使用

\[
\dot\eta_{\mathrm{ref}}
=\omega_1\eta_{\mathrm{ref}}
=\frac{\omega_1H_{\mathrm{ref}}}{c_h}.
\]

需要通过运行时 diagnostics 或最小回放检查确认 `w_slosh_eta_dot` 的实际数值与 \(\rho_{\dot\eta}w_\eta\) 一致。

### 3.2 延迟与相位补偿验收

使用 `/cmd_vel` 与 odometry 估计线速度和角速度的实际执行延迟。相位补偿的配置必须适用于当前底盘、负载、地面和速度区间。如果延迟估计偏差过大，优化器的反相抑制可能在真实系统中变成同相激励。

### 3.3 配置传递和代价生效验收

至少检查：

- 日志中的实际 `w_slosh` 与启动参数一致；
- `J_slosh_eta` 和 `J_slosh_eta_dot` 已进入 cost breakdown；
- 增大 `w_slosh` 能在同一 pre-solve 状态下改变预测 horizon 或 optimized first action；
- raw solver command、post-gate command 和 published command 可分层比较；
- 正式相机、时间同步和液面提取链可另行通过 P0/P8 smoke；P3 本身不录 RGB；
- 失败、fallback、solve-budget overrun、observed inter-arrival gap proxy 和执行层记录链完整；不把 bag 到达间隔称为严格 control-cycle deadline miss。

## 4. 不参与本轮扫描的冻结量

调节 `w_slosh` 时，以下内容必须保持不变：

- \(w_c,w_l,w_p,w_v,w_{v_s},w_a,w_\omega,w_\alpha\)；
- \(w_{\Delta a},w_{\Delta v_s}\) 和 `gamma_ac`；
- \(\rho_{\dot\eta}=0.3\)；
- nominal `v_ref`、\(v_{\max},\omega_{\max},a_{\max},\alpha_{\max}\)；
- horizon、integration、solver、warm start 和数值容差；
- terminal、gate、rate limiter、fallback 和发布层限制；
- 容器、液体、安装位置、路径、相机位姿与 RGB 处理参数。

Smooth-only MPCC 的增强平滑权重应在本轮扫描前先独立调整到“明显比 Baseline 平滑，但仍能正常完成路径”的合理对照工作点，随后与 Baseline 一起冻结。不得为了放大 S-MPCC 的优势而故意弱化或过度平滑 Smooth-only。

## 5. 建议的 pilot 扫描矩阵

### 5.1 固定条件

推荐使用：

- 名义容器 \(C_1\)；
- 与正式 H1 分离、已冻结且机制激励相似的唯一 H0 开发路径；
- 相同起点、方向、液深、静止准入和 post-arrival 窗口；
- 每个候选至少 3 次独立完整 trial；
- 方法和权重顺序轮换或平衡，避免液温、地面、电量和时间漂移与候选权重绑定。

### 5.2 候选组

| 组别 | 配置 | 建议重复 | 作用 |
| --- | --- | ---: | --- |
| P-B0 | Baseline MPCC，`w_slosh=0` | 3 | 确认名义 MPCC 工作点 |
| P-Smooth | Smooth-only MPCC，`w_slosh=0` | 3 | 确认增强平滑对照确实改变命令规律性 |
| P-W1 | S-MPCC，`w_slosh=1` | 3 | 弱液体代价候选 |
| P-W2 | S-MPCC，`w_slosh=2` | 3 | 中弱液体代价候选 |
| P-W5 | S-MPCC，`w_slosh=5` | 3 | 当前默认量级候选 |

pilot 固定为 15 次，顺序以现场协议 v1.0 为准。W10 和任意临时候选不得现场追加；这些 trial 只服务于模型侧参数决策，不进入正式 40/64/88 个矩阵单元。

## 6. 记录与比较指标

### 6.1 强制准入指标

每个候选首先通过以下准入检查：

- 任务 success 和统一到达判据；
- path-error p95 不超过预先冻结的容差；
- completion time 位于可接受区间；
- solver failure、solve-budget overrun、observed inter-arrival gap proxy 和 fallback 不超过准入阈值；
- \(r_{\mathrm{int}}\) 不超过 \(r_{\mathrm{int,max}}\)；
- 无长时限速覆盖、反复方向反转、终端振荡或安全中止；
- odometry、TF、冻结路径、solver、内部 slosh、完整 horizon、raw/post/published command 和失败日志完整；
- RealSense 随基础栈保持启动，但 P3 固定 `RECORD_RGB=false`。

未通过准入的候选不参与后续“收益大小”比较。

### 6.2 参数选择指标

正式定义中 B0 和 B_smooth 关闭控制器液体状态，因此它们不存在可与 S-MPCC 在线信号等价比较的 `/spmpc/slosh_height` 或液体预测 horizon；缺失信号不得填零。为了估计跨 run 重复性，P3 的全部候选必须一致启动方法无关的 standalone slosh monitor，并在每次运动前 reset。Standalone `/slosh/*` 只用于重复性、噪声和执行运动诊断，不得伪称为 B0/B_smooth 的控制器内部状态。

对通过准入的候选比较：

- 仅对 W1/W2/W5，比较控制器在线 \(H_{\mathrm{modal}}\) 与预测 horizon peak、RMS 和 post-arrival RMS；
- 对全部候选，检查统一 reset 的 standalone monitor 输出是否完整，并仅用其预先估计重复性/噪声容差和排查执行异常；
- executed \(v,\omega,a_x,a_y\) 的路径进度分布；
- optimized first action 和完整预测 horizon 差异；
- raw/post/published 三层命令的变化率和干预比例；
- completion time、path-error p95 和 success；
- solve-time p95/overrun、observed inter-arrival gap proxy、failure 和 fallback。

不得只看 cost breakdown 或单次最小内部高度选权。必须联合完整内部时序、预测 horizon、optimized first action、任务、tracking、实时性和执行层干预；该步骤只能得到 `model-side frozen weight`，不能声称真实液面已经改善。

## 7. 候选权重决策规则

建议使用以下分层规则，不建议将多个指标临时凑成一个可任意调整的加权总分。

### 第一层：准入门槛

仅保留任务、跟踪、实时性、命令链和数据完整性全部合格的候选。

### 第二层：机制可辨识性

候选权重应当能在高曲率或曲率反转区段稳定改变 optimized first action、局部速度或激励分配，而不是只把全程速度一致降低。

### 第三层：模型响应

仅在 W1/W2/W5 三个 S-MPCC 候选之间比较控制器在线 \(H_{\mathrm{modal}}\)、预测 horizon 和 post-arrival 响应。在三个 block 中，改善方向应稳定且不依赖某一次极端 trial；需同时查看原始 trial 点和过程曲线，不能只看平均值。B0/B_smooth 只用于任务、执行和方法无关 monitor 重复性参考，不进入在线 \(H_{\mathrm{modal}}\)/horizon 排序。

### 第四层：最小拐点原则

`delta_model` 固定来自 P3 全部 15 个矩阵单元中一致启用、每个 trial 运动前 reset 的方法无关 standalone `/slosh/*` monitor 重复性。对每个已冻结指标 \(q\in\{\mathrm{peak},\mathrm{RMS},\mathrm{post\mbox{-}arrival\ RMS}\}\)，v1.0 固定使用

\[
\delta_{\mathrm{repeat},q}
=\max\!\left(
\operatorname{range}_b q_{\mathrm{standalone}}(B0,b),
\operatorname{range}_b q_{\mathrm{standalone}}(B_{\mathrm{smooth}},b)
\right).
\]

未预注册独立 noise smoke 时，\(\delta_{\mathrm{model},q}=\delta_{\mathrm{repeat},q}\)；若在产生任何 P3 数据前已预注册，则固定为 \(\delta_{\mathrm{model},q}=\max(\delta_{\mathrm{repeat},q},\delta_{\mathrm{noise},q})\)。只有进度/时间窗的具体边界、数值结果和实现脚本 hash 待填；不得临场更换统计量或合并方式，也不得把方法间差异当作噪声。不得用 B0/B_smooth 缺失或人工置零的控制器在线 \(H_{\mathrm{modal}}\) 估计，也不得使用正式 RGB 回填。S-MPCC 的在线 `/spmpc/*` \(H_{\mathrm{modal}}\) 和预测 horizon 只作为 W 候选的机制与排序附加证据，不用来定义 `delta_model` 本身。

若相邻 W 候选的额外在线模型收益低于已冻结的 `delta_model`，选择较小的 `w_slosh`。更大权重只有在提供稳定额外模型收益且不恶化任务、tracking、实时性或执行层干预时才能入选。

## 8. 避免结果导向调参的规则

- pilot 目录、bag、配置、脚本和分析结果全部归档；
- 事先写明候选集、准入门槛、\(\delta_{\mathrm{pilot}}\) 和权重决策规则；
- 不得在查看候选结果后追加一个“刚好更优”的权重，除非将整轮标记为探索性并用新的独立 pilot 重新验证；
- 不得调低 Baseline 或 Smooth-only 的任务性能以制造差距；
- 不得用正式 RGB 或正式区组的中间结果继续调权；
- 不得把 pilot 试次补进正式 \(n=8\)；
- 正式论文可在补充材料说明“权重由独立 pilot 按预定义工作点准则冻结”，但不把 pilot 当成新的正式效果证据。

## 9. Smooth-match 的执行顺序

Smooth-match 只能在最终 S-MPCC `w_slosh` 冻结后调整。固定顺序为：

1. 冻结 Baseline 和 Smooth-only；
2. 完成 `w_slosh` pilot 扫描；
3. 冻结 S-MPCC 权重；
4. 按现场协议第 5.4 节执行固定 12 次 pilot，只根据 completion time 在 M-/M0/M+ 中选择；
5. 不查看 Smooth-match pilot 的 RGB 结果；
6. 冻结 Smooth-match 配置。

Smooth-match 不是第四个核心方法，也不参与 `w_slosh` 的选择。

## 10. 正式冻结物

最终权重选定后，至少归档：

- Baseline、Smooth-only、S-MPCC 和 Smooth-match 的独立只读配置；
- 所有权重、normalizer、运动约束和容器参数；
- `variants.yaml` 或正式等价配置的 Git commit 和文件 hash；
- solver/codegen 产物、编译信息和软件 revision；
- 路径文件、容器信息、相机标定和同步配置；
- pilot 候选矩阵、决策记录和最终选择理由；
- 正式随机区组表和首个正式 trial 开始时间。

冻结后，\(C_1\) 与 \(C_2\)、低风险与高风险路径、普通与长时运行都必须使用同一组对应方法权重。跨容器只允许按冻结协议更新 \(\omega_1,c_h,\eta_{\mathrm{ref}},\dot\eta_{\mathrm{ref}}\) 等容器模型参数。

## 11. 正式实验开始后的处理规则

### 11.1 方法失败

如果冻结配置在某个正式 trial 中出现 solver failure、tracking failure、timeout、fallback 或 safety termination，该 trial 按预先协议记为方法失败，不能调权后删除原 trial 并补跑。

### 11.2 与方法无关的采集故障

如果是明确与方法无关的相机断开、日志损坏或外部安全事件，按上位协议记录排除理由，并在同一 block 内补采。

### 11.3 必须重新调权的情况

如果正式实验开始后发现实现错误、尺度错误或安全问题，使原冻结方法不再具有可解释性，应：

1. 立即停止正式采集；
2. 创建新的方法/协议版本；
3. 使用新的独立 pilot 重新调参和验收；
4. 重新归档配置、hash、随机化表和采集起点；
5. 原版本数据作为 development-only 或单独失败版本记录，不与新版本正式数据混合。

## 12. 正式冻结检查表

- [ ] \(\dot\eta_{\mathrm{ref}}\) 和 \(\rho_{\dot\eta}\) 在公式、codegen 和 runtime 中一致；
- [ ] 线速度/角速度执行延迟和相位补偿已验收；
- [ ] `w_slosh` 在日志和 solver cost 中的实际生效值可核对；
- [ ] Baseline 和 Smooth-only 已先行冻结，且 Smooth-only 是有竞争力的平滑对照；
- [ ] `w_slosh=1/2/5` 已按固定 15 次 pilot 采集，未加入 W10 或临时候选；
- [ ] standalone `/slosh/*` monitor 在 P3 全部 15 次中一致启用且每个 trial 前 reset，`delta_model` 按 v1.0 固定 within-method range/max 公式执行，其时间窗、脚本 hash 和待填数值在查看 W 在线排序前预注册；
- [ ] B0/B_smooth 缺失的控制器在线 \(H_{\mathrm{modal}}\)/horizon 没有被填零或用于候选排序；
- [ ] 所有候选的原始 trial 点、过程曲线、失败和执行层干预已审查；
- [ ] 最终权重按“准入→机制→模型响应→最小拐点”规则选定，并标为模型侧冻结值；
- [ ] Smooth-match 在最终 S-MPCC 冻结后按固定 12 次 pilot 只按完成时间冻结；
- [ ] 四种论文配置、Git commit、hash、codegen 和编译信息已归档；
- [ ] 正式区组和正式 RGB 结果尚未用于任何权重选择。

## 13. 当前推荐结论

当前最稳妥的路线是：

\[
\boxed{
\text{修正尺度与相位口径}
\rightarrow
\text{冻结共享权重和 Smooth-only}
\rightarrow
\text{扫描 }w_{\eta}\in\{1,2,5\}
\rightarrow
\text{选择最小稳定拐点}
\rightarrow
\text{冻结 Smooth-match}
\rightarrow
\text{开始正式 88 次}
}
\]

参数扫描的目的是让液体动态机制在实物中具有可辨识性，而不是保证正式数据中 S-MPCC 一定获胜。一旦进入正式区组，所有权重、容差、路径、容器和分析规则必须保持冻结，并如实报告正向、负向和失败结果。
