# S-MPCC 正式实验矩阵与执行速查表

> 本文件是正式实验矩阵的简明执行索引，不再独立定义论文贡献或实验逻辑。
>
> 唯一上位依据：[S-MPCC_experimental_design.md](./S-MPCC_experimental_design.md)
>
> 当前版本日期：2026-07-17
>
> 当前状态：与实验设计冻结版 v1.0 同步
>
> 文件名保留旧的SPMPC写法仅为兼容现有链接；论文方法名和本文内容统一使用S-MPCC。
>
> 若本文件与上位协议、正式配置快照或 freeze artifacts 不一致，以上位协议和 freeze artifacts 为准。

---

## 1. 冻结方法与代码映射

正式主矩阵只包含以下三种方法：

| 论文方法 | 冻结代码映射 | 液体状态 | Slosh cost | 通用平滑 | 实验角色 |
| --- | --- | ---: | ---: | ---: | --- |
| Baseline MPCC | B0 | 关闭 | 关闭 | Nominal | 基础MPCC对照 |
| Smooth-only MPCC | B_smooth | 关闭 | 关闭 | Enhanced | 通用增强平滑对照 |
| S-MPCC | B_slosh | 开启 | 开启 | Nominal | 本文方法 |

正式采集要求：

- 三种方法分别归档只读配置快照；
- 三种方法共享机器人模型、路径、运动约束、solver、horizon和部署执行层；
- Reference Governor、modal hard cap等液体感知扩展在核心比较中关闭；
- B_ours是“slosh + enhanced smoothing”的开发变体，不属于正式主矩阵；
- 图表和正文使用论文方法名，代码映射只在方法定义表和补充材料中出现。

RQ2另设Smooth-match MPCC。它不是第四个核心方法，而是通过独立pilot只按完成时间调节并冻结的B_smooth速度参考配置。

---

## 2. 研究问题—实验矩阵映射

| RQ | 主要比较 | 物理或计算证据 | 核心作用 |
| --- | --- | --- | --- |
| RQ1：物理有效性与任务性能 | Baseline、Smooth-only、S-MPCC | 两条路径的视觉液面、执行运动、激励、跟踪和成功率 | 检验真实物理效果并区分普通平滑 |
| RQ2：完成时间混杂 | Smooth-match、S-MPCC | 等完成时间下的液面与局部速度/激励分配 | 排除“只是整体更慢” |
| RQ3：跨容器迁移 | \(C_1/C_2\)中的三方法 | 跨容器物理结果、参数切换和mismatch replay | 检验无权重重调的有限迁移 |
| RQ4：状态相关在线规划 | 四相位、actual/zero replay、runtime | 完整预测horizon、optimized first action、求解日志 | 证明液体动态记忆参与决策并满足实时性 |

贡献证据对应关系：

\[
\begin{aligned}
\mathrm{C1}&\leftarrow \mathrm{RQ1+RQ2+RQ4+runtime},\\
\mathrm{C2}&\leftarrow \mathrm{RQ3+RQ4+counterfactual\ replay},\\
\mathrm{C3}&\leftarrow \mathrm{RQ1\mbox{--}RQ4\ as\ a\ matched\ whole}.
\end{aligned}
\]

---

## 3. 完整物理实验矩阵

正式设计采用 \(n=8\) 个随机区组，共88次主方案物理实验。

| ID | 区组类型 | 容器与路径 | 方法 | 每个block内容 | block数 | 物理次数 | 用途 |
| --- | --- | --- | --- | --- | ---: | ---: | --- |
| E1 | 低风险区组 | \(C_1\)+低风险路径 | 三种核心方法 | 3 trials | 8 | 24 | RQ1双路径一致性 |
| E2 | 容器super-block | \(C_1/C_2\)+高风险路径 | 三种核心方法 | 2容器×3方法=6 trials | 8 | 48 | RQ1高风险结果与RQ3 |
| E3 | 等时间区组 | \(C_1\)+高风险路径 | Smooth-match、S-MPCC | 2 trials | 8 | 16 | RQ2 |
| 合计 | — | — | — | — | — | **88** | RQ1–RQ4与实时性 |

计数说明：

\[
8\times3
+8\times(2\times3)
+8\times2
=24+48+16
=88.
\]

E2中的高风险\(C_1\)数据同时用于RQ1和RQ3，不重复采集。

### 3.1 可选物理mismatch组

若论文需要检验“正确容器参数对物理迁移效果是否必要”，可在每个E2 super-block中增加：

| ID | 条件 | 方法 | block数 | 新增次数 | 完整总数 |
| --- | --- | --- | ---: | ---: | ---: |
| E4（可选） | \(C_2\)容器、高风险路径、错误使用\(C_1\)参数 | S-MPCC | 8 | 8 | 96 |

没有E4时，只能主张：

- 无权重重调的有限跨容器迁移；
- 计算计划对容器参数敏感。

不能主张“几何参数更新对真实物理迁移具有必要作用”。

---

## 4. 分阶段证据包

| 证据包 | 正式组成 | 累计物理次数 | 可以保留的主要结论 |
| --- | --- | ---: | --- |
| 核心机制包 | 高风险\(C_1\)三方法24次+等时间16次 | 40 | RQ1高风险、RQ2、RQ4、实时性 |
| 双路径包 | 核心机制包+低风险\(C_1\)三方法24次 | 64 | 增加两条路径一致性 |
| 完整主方案 | 双路径包+高风险\(C_2\)三方法24次 | 88 | 完整RQ1–RQ4及有限跨容器迁移 |
| 参数必要性扩展 | 完整主方案+E4物理mismatch | 96 | 可检验正确参数对物理迁移是否必要 |

如果最终只完成40或64次，摘要、贡献、RQ列表、结果表和结论必须同步删除或降级未获得数据支持的跨路径或跨容器主张。

---

## 5. 不计入88次的计算实验

| ID | 计算实验 | 固定条件 | 改变条件 | 主要输出 | 作用 |
| --- | --- | --- | --- | --- | --- |
| K1 | 纵向四相位规划 | 机器人状态、路径、容器、模态能量 | \(x\)方向四种相位 | 完整horizon与第一控制量 | 检验纵向状态相关规划 |
| K2 | 横向四相位规划 | 机器人状态、路径、容器、模态能量 | \(y\)方向四种相位 | 完整horizon与第一控制量 | 检验转弯前状态相关规划 |
| K3 | 实际传播状态反事实replay | 同一pre-solve solver快照及全部非液体输入 | actual state、zero state、可选phase flip | optimized first-action difference | 检验正式传播状态是否实际影响优化 |
| K4 | 容器参数切换 | 机器人状态、路径、归一化模态状态 | \(C_1/C_2\)参数集 | 预测运动、模态响应、optimized first action | 解释参数如何改变规划 |
| K5 | 计算mismatch replay | \(C_2\)日志和相同solver上下文 | 正确\(C_2\)参数、错误\(C_1\)参数 | 计划差异 | 只证明规划对参数敏感 |
| K6 | 模型一致性与敏感性 | 正式日志和冻结分析流程 | \(\omega_1,\zeta,c_h\)、延迟、初态等 | 偏差、趋势与敏感性 | 支持模型适用范围诊断 |

K1/K2至少冻结两个机制检查点：

- 纵向检查点：\(Z_1\)起步或\(Z_4\)重新加速前；
- 横向检查点：\(Z_2\)入弯或\(Z_3\)曲率反转前。

K3必须满足：

- actual与zero从同一pre-solve快照克隆，不能在同一个可变solver实例上顺序求解；
- actual分支先在冻结容差内复现在线solver status、第一控制量和原始solver command；
- replay工具导出完整\(v,\omega,v_s,\eta,\dot\eta,a,\alpha\) horizon；
- 统计单位为trial，不把控制周期当作独立样本；
- 未经过共享执行层回放时，只称optimized first-action difference，不称counterfactual executed command。

---

## 6. 随机化与区组规则

### 6.1 E1低风险区组

每个block包含：

1. Baseline MPCC；
2. Smooth-only MPCC；
3. S-MPCC。

三种方法顺序随机化。

### 6.2 E2容器super-block

每个super-block包含：

\[
\{C_1,C_2\}
\times
\{\mathrm{Baseline,Smooth\mbox{-}only,S\mbox{-}MPCC}\}.
\]

要求：

- 容器顺序随机化或在8个super-block中平衡；
- 每个容器内部的方法顺序随机化；
- 同一super-block尽量在相近环境和设备状态下完成；
- 不使用没有同期区组的历史\(C_1/C_2\)数据直接比较。

### 6.3 E3等时间区组

每个block包含：

1. Smooth-match；
2. S-MPCC。

顺序随机化。Smooth-match只能使用独立pilot的完成时间调节，正式RGB不得用于继续调参。

---

## 7. 主要指标和分析窗口

### 7.1 Primary outcome

从首次有效运动时刻\(t_{\mathrm{move}}\)到统一到达时刻\(t_{\mathrm{arrival}}\)定义全运动窗口：

\[
\mathcal W_{\mathrm{full}}
=[t_{\mathrm{move}},t_{\mathrm{arrival}}].
\]

Primary outcome为：

\[
H_{\mathrm{vis,p95}}^{\mathrm{full}}
=Q_{0.95}
\left(
H_{\mathrm{vis}}(t):t\in\mathcal W_{\mathrm{full}}
\right).
\]

该窗口包括起步\(Z_1\)和制动\(Z_5\)，但不包含开始前等待和到达后观察。

### 7.2 关键次要和敏感性指标

- 10%–90%路径进度内的\(H_{\mathrm{vis}}\) p95；
- 全运动窗口\(H_{\mathrm{vis}}\) RMS；
- 到达后5 s的\(H_{\mathrm{vis}}\) RMS；
- completion time；
- contour error p95；
- success/failure rate；
- \(Z_1\)–\(Z_5\)预注册区段摘要。

### 7.3 机制和部署指标

- \(v(\sigma)\)、\(\omega(\sigma)\)；
- \(a_x(\sigma)\)、\(a_y(\sigma)\)；
- \(H_{\mathrm{modal}}(\sigma)\)和\(H_{\mathrm{vis}}(\sigma)\)；
- raw solver、post-gate和published command；
- 执行层干预比例\(r_{\mathrm{int}}\)；
- solve-time median、p95、maximum和ECDF；
- deadline miss、solver failure、fallback和实际控制频率。

---

## 8. 预注册比较与统计

### 8.1 比较层级

| RQ | 比较层级 |
| --- | --- |
| RQ1第一主比较 | S-MPCC − Smooth-only |
| RQ1关键次比较 | S-MPCC − Baseline |
| RQ1机制诊断 | Smooth-only − Baseline |
| RQ2主比较 | S-MPCC − Smooth-match |
| RQ3 | 每个容器内的相同配对比较，再描述method×container interaction |
| RQ4 | 四相位计划差异及actual−zero trial-level replay摘要 |

### 8.2 统计规则

- 一个完整trial是一个统计样本；
- 每个block内计算paired difference；
- 报告原始trial点、配对线、效应大小和相对变化；
- 95%区间按完整block重采样；
- 第一主比较增加exact paired sign-flip/randomization inference；
- 增加leave-one-block-out敏感性；
- 控制周期、视频帧和路径网格点不能当作独立样本；
- 不能根据结果好坏选择trial、路径区段或代表曲线。

---

## 9. 必须记录的数据

| 层次 | 必须记录的信号 |
| --- | --- |
| 路径 | 权威路径、\(s\)、\(\sigma\)、\(\kappa(s)\)、\(Z_1\)–\(Z_5\)标签 |
| 机器人 | \(x,y,\theta,v,\omega\)、contour/lag error |
| OCP输入 | solver输入机器人状态、传播液体状态、\(s_{\min}\)、有效\(v_{\mathrm{ref}}\) |
| OCP输出 | 第一\(a,\alpha,v_s\)、solver status、solve time |
| 执行层 | raw solver、post-gate、published command及全部limiter/fallback标志 |
| 液体模型 | \(\eta_x,\dot\eta_x,\eta_y,\dot\eta_y,H_{\mathrm{modal}}\) |
| 物理参考 | \(H_{\mathrm{vis}}\)、相机同步、缺帧和标定状态 |
| Replay | pre-solve solver/warm-start快照及完整预测horizon |

---

## 10. 推荐执行顺序

1. 冻结三方法只读配置、\(C_1/C_2\)、两条路径和\(Z_1\)–\(Z_5\)；
2. 验证相机帧率相对\(f_1\)足够，完成标定和同步；
3. 验证raw/post-gate/published command均进入正式日志；
4. 验收replay快照克隆、actual复现和完整horizon导出；
5. 完成Smooth-match独立pilot并冻结；
6. 生成E1/E2/E3随机顺序；
7. 优先采集40次核心机制包；
8. 执行K1/K2/K3并检查贡献一、二的机制证据；
9. 扩展到64次双路径包；
10. 扩展到88次完整主方案并执行K4/K5；
11. 仅在需要物理必要性主张时增加E4至96次；
12. 生成配对图、过程链、效应区间和runtime结果后再撰写结论。

---

## 11. 不再属于正式主矩阵的旧方案

以下内容不再进入当前主论文实验矩阵：

- B_ours及其Governor、hard-cap或anti-slosh开发变体；
- Reference Governor核心消融；
- 将terminal handling包装为S-MPCC独有方法贡献的消融；
- DWA、TEB、LT-DWA或其他外部planner排行榜；
- 每组3次以上的旧小样本方案；
- 用模型代理量代替视觉物理结果；
- 旧三次pilot作为正式\(n=8\)的一部分。

这些内容如需保留，只能标为development-only或supplementary，不得改变88次主矩阵和RQ1–RQ4的结论层级。

---

## 12. 正式采集前矩阵冻结检查

- [ ] 三方法映射为B0、B_smooth和B_slosh，主矩阵不使用B_ours；
- [ ] \(C_1/C_2\)尺寸、液深、freeboard和模态参数已归档；
- [ ] \(f_{\mathrm{cam}}/f_1\)满足冻结的视觉测量准入标准；
- [ ] 两条权威路径和\(Z_1\)–\(Z_5\)已冻结；
- [ ] 全运动窗口primary及10%–90%敏感性窗口已冻结；
- [ ] \(\epsilon_v,\epsilon_\omega,r_{\mathrm{int,max}}\)已冻结；
- [ ] RQ1/RQ2主比较和统计规则已冻结；
- [ ] Smooth-match只利用独立pilot的完成时间调节；
- [ ] E1/E2/E3的8个block随机顺序已经生成；
- [ ] 方法失败、采集故障和补采规则已冻结；
- [ ] K1/K2相位方向、幅值和检查点已冻结；
- [ ] K3 actual/zero replay能够从相同pre-solve快照分叉；
- [ ] actual replay能够复现在线结果并导出完整horizon；
- [ ] 正式代码、配置、文档、随机表和分析脚本已纳入版本管理。

---

## 13. 矩阵冻结结论

当前正式实验矩阵固定为：

\[
\boxed{
88\ \text{physical trials}
+\text{pre-registered computational experiments}
}
\]

其中：

- 40次是最小核心机制包；
- 64次增加双路径一致性；
- 88次形成完整RQ1–RQ4证据；
- 96次只用于额外检验参数更新的物理必要性。

正式实验的核心不是比较谁的平均速度最低，而是检验S-MPCC是否在相同几何路径上依据曲率和传播液体状态重新分配局部速度与激励，并以独立视觉液面测量验证其物理效果。
