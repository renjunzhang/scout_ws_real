# G3R2 逐预测步 RGB 与 IMU observer 匹配度初筛及完整曲线方案

日期：2026-08-02

范围：G3R2 W5_S10 两条 W5 bag；Bsmooth 两条 bag 仅用于冻结 RGB 可见投影映射

状态：`DESCRIPTIVE_POST_HOC_COMPLETE`；已用 0801 bags 完成 native-RGB-event、high-accuracy planned replay 与 actual-IMU software replay；未修改控制代码、solver、参数或实验协议

> 第 5 节保留旧插值 prototype 作为审计对照，不得形成 forecast claim。正式 native-event descriptive 结果见第 8 节；分析 manifest 已绑定 analyzer、replay helper、bags 和 postflight hash，汇总 manifest 另行绑定派生报告的输入与输出 hash。

## 1. 结论

应当先完成逐预测步匹配曲线，再决定是否修改 observer、MPC future model 或启用液面约束。

现有 G3R2 数据足够做 post-hoc 初筛：两条 W5 bag 分别有 `1052`、`1049` 个有效 acados horizon，每个 horizon 为 `61` 个状态节点、`60` 个控制节点，`dt=33.333 ms`，覆盖 `2 s`；RGB、processed-IMU observer、selection、solver input、cmd、odom 和 IMU 激励均已录制。

初筛结果表明：

1. processed-IMU observer 与 RGB 的**单相机可见液面投影**存在可重复对应；这说明模型不是完全失效，但还不足以证明四维液体状态或 future model 正确。
2. 原 prototype 的 `j=1` 通常不是未来预测：horizon 发布时 q0 已旧约 `41--44 ms`，而 `j=1` 只前进 `33.3 ms`。两包中 `j=1` 具有正因果 lead 的比例仅为 `16.38% / 11.28%`。
3. `j=2` 几乎都已进入未来，但仍有少数负 lead；按当前 header proxy，`j=3` 才是两包所有样本都严格未来的首个网格点。
4. 原表显示虚拟网格相关性随步长下降，但它混有 stale-q0、RGB 稀疏线性插值、planned input 未执行和重规划等影响，不能直接解释为液体 ODE 的 forecast fidelity。
5. 当前消息没有共享 `cycle_id` 和权威 `horizon_origin_stamp`；horizon header 是 solve 后发布时间。因此本批只能作描述性筛查，不能冻结正式 forecast claim。
6. 单相机只能验证一个液面投影，不能证明完整二维 `eta_x/eta_y` 径向状态正确。

正式分析应形成下面的误差链，同时保留直接 operational 曲线：

```text
q_ocp_planned
  vs q_high_accuracy_planned_replay
    -> SQP/转录/积分器数值误差

q_high_accuracy_planned_replay
  vs q_realized_input_replay
    -> 重规划、执行器与输入实现差异

q_realized_input_replay
  vs q_IMU_observer
    -> 时间/离散实现一致性；同模型同输入时应近似恒等

h_RGB(q_realized_input_replay)
  vs RGB
    -> q0、液体模型与相机映射的联合物理残差
```

另外直接报告：

```text
q_ocp_planned -> future observer / native RGB  # 实际 MPC 的 operational forecast
per-node cost/constraint + first-action sensitivity
```

这样才能区分“solver 数值传播”“计划没有实现”“observer 实现不一致”和“真实液体模型不符”，并判断 first action 是否主要受低可信远期节点驱动。

## 2. 数据与资格

数据目录：

```text
/data/a/slosh_bags/real/20260801_spmpc_g3r2_w5s10_paired_confirmation/H0/
```

W5 horizon 完整性：

| bag | horizon 消息 | 有效 `B_slosh_ACADOS_OK` | `GOAL_REACHED` 空 horizon | 有效数组形状 |
| --- | ---: | ---: | ---: | --- |
| W5 Block 01 | 1505 | 1052 | 453 | state `61`，control `60` |
| W5 Block 02 | 1492 | 1049 | 443 | state `61`，control `60` |

逐步初筛仅使用：

- `valid=true`；
- `slosh_enabled=true`；
- `solver_status=B_slosh_ACADOS_OK`；
- `dt=0.033333333 s`；
- `len(t/eta/h_modal)=61`、`len(a/alpha/v_s)=60`；
- horizon origin 位于 first effective motion 到 first `GOAL_REACHED`；
- selected observer 为有效 processed-IMU，无 fallback/reset；
- RGB 为 `valid + zero_locked + STATUS_OK + no clipping`。

prototype 以 `horizon.header.stamp` 进入 motion window、且目标落在序列宽松范围内为 admission，两条 W5 分别保留 `1050`、`1046` 个描述性 origins；若改用 `selected_state_stamp` 判断 motion window，则为 `1049/1045`。这些数字不是每个 `j` 的 RGB 有效样本数：即使施加旧的 75 ms 插值 gate，各步也仅约为 Block 01 `930--936`、Block 02 `980--986`；正式 native-event 方法必须逐 bin 报告实际 coverage。是否属于 causal forecast 还必须逐点检查第 3.3 节的 lead。`453/443` 条 terminal 空 horizon 不参与精度曲线，但必须进入 availability ledger，不能当成零预测或静默删除。

## 3. 当前时间合同及限制

### 3.1 消息时间语义

- observer 四维状态使用 `/spmpc/debug/slosh_observer_imu.state_stamp`；
- RGB 使用 `/liquid/measurement.header.stamp`，它复制源图像曝光时间；
- `PredictedHorizon.t[j]=j*dt` 只是相对网格时间；
- `PredictedHorizon.header.stamp` 是 solve 完成后构造诊断消息时的 `ros::Time::now()`，不是 solver tick 或 stage-0 状态时刻；
- snapshot/horizon 没有跨 topic 共享 `cycle_id`。

本批逐步表只对两条 W5 按发布序号配对；这两包 selection/horizon 消息数和单调顺序一致，可用于 post-hoc screening，但不能替代正式 cycle contract。两条 Bsmooth 分别存在 `1522 vs 1521`、`1486 vs 1485` 的消息数差异，不允许套用序号配对；projection 拟合必须按 source/state stamp 和显式 gap gate 重新完成。

### 3.2 x0 在 horizon 发布时已经较旧

对两条 W5 有效 horizon 的只读复算：

| 时间差 | Block 01 P50/P95/max | Block 02 P50/P95/max |
| --- | --- | --- |
| horizon header - selector header | 15.25 / 22.86 / 33.01 ms | 17.63 / 26.33 / 39.18 ms |
| selector header - selected state | 25.85 / 34.80 / 39.12 ms | 26.05 / 35.08 / 41.19 ms |
| horizon header - selected state | **41.25 / 54.07 / 67.54 ms** | **43.57 / 56.93 / 76.10 ms** |

因此不能把：

```text
horizon.header.stamp + j*dt
```

当成权威液体 target time。当前描述性主口径采用：

```text
t_liquid,0 = selection.selected_state_stamp
t_target(i,j) = t_liquid,0(i) + j*dt
```

但 `fixed_robot_only` 的 robot x0 是 0.22 s rollout 后状态，liquid x0 是 current selected state，本身没有统一物理 epoch。这一限制必须写入 claim。

### 3.3 网格步长不等于可用时刻后的预测 lead

对每个 cycle `i`、节点 `j` 同时定义：

```text
model_grid_lead(i,j) = j*dt

causal_available_lead(i,j)
  = selected_state_stamp(i) + j*dt - horizon.header.stamp(i)
```

其中 `header.stamp` 只是当前 bag 中可获得的 conservative availability proxy；正式 release 应改用权威 `solve_end_stamp/horizon_available_stamp`。分类规则为：

```text
causal_available_lead <= 0  -> hindcast / smoothing consistency
causal_available_lead > 0   -> 描述性 causal forecast
```

当前两条 W5 的实测结果：

| j | model-grid lead | Block 01 | Block 02 | 因果解释 |
| ---: | ---: | --- | --- | --- |
| 0 | 0 ms | 正 lead 0% | 正 lead 0% | q0 接线恒等式；全为过去状态 |
| 1 | 33.3 ms | 正 lead 16.38%，P05/P50/P95 `-20.7/-7.9/+3.8 ms` | 正 lead 11.28%，P05/P50/P95 `-23.5/-10.2/+2.7 ms` | 大多数为 hindcast，不是 one-step forecast |
| 2 | 66.7 ms | 正 lead 99.90%，P05/P50/P95 `+12.6/+25.4/+37.1 ms` | 正 lead 99.62%，P05/P50/P95 `+9.8/+23.1/+36.0 ms` | 几乎全为未来，但仍非严格全因果 |
| 3 | 100 ms | 正 lead 100%，min/P50 `32.46/58.75 ms` | 正 lead 100%，min/P50 `23.90/56.45 ms` | 当前 proxy 下首个全样本严格未来节点 |

因此后文必须同时按 `j/model_grid_lead` 和 `causal_available_lead` 出图。任何“33 ms、67 ms、100 ms forecast”结论都应改写成 horizon 真正可用以后还有多少正 lead；hindcast 样本不得混入 forecast 指标。

`+12 ms` alignment sensitivity 会让对应 RGB source target 比 q target 再早 `12 ms`：此时 `j=1` 两包均为 `0%` causal，`j=2` 为 `95.90% / 90.54%` causal，`j=3` 起才全部 causal；`j=3` 的 min/P50 分别为 `20.46/46.75 ms` 与 `11.90/44.45 ms`。这也是 primary 必须固定 `delta=0`、sensitivity 必须单列 coverage 的原因。

## 4. 初筛测量口径

### 4.1 不用 H_vis 直接验证 signed phase

冻结 outcome：

```text
H_vis = max(0, causal_median_5(height_max_lcr_mm))
```

经过 max、median 和截零，不能验证符号、方向和相位。逐预测步模型筛查改用同一 RGB 帧的可见斜率：

```text
z_RGB = (height_R - height_L) - static_slope_baseline
```

本次快速初筛实际使用 zero-corrected `height_R-height_L`，静止 slope offset 由回归截距 `b0` 吸收；正式 analyzer 应显式冻结 pre-motion slope baseline，不再让 test intercept 吸收它。

本次初筛用线性相机投影：

```text
z_RGB(t) ~= b0 + bx*eta_x(t + delta_observer_rgb_alignment)
                  + by*eta_y(t + delta_observer_rgb_alignment)
```

其中 `delta_observer_rgb_alignment`、`b0/bx/by` 只能在 training bags 上确定，test W5 不参与拟合。

### 4.2 训练/验证隔离

初筛采用：

```text
training = 两条 Bsmooth bag
test     = 两条 W5 bag
```

training 得到：

```text
delta_observer_rgb_alignment = +12 ms
z_pred_mm = -0.282 - 128.749*eta_x - 1823.484*eta_y
training R^2 = 0.621
```

这里的符号约定为：

```text
z_RGB(t) ~= projection(q_observer(t + 12 ms))
```

所以 horizon 的状态时刻为 `t_q` 时，对应 RGB source time 为 `t_q-12 ms`。该量必须称为 **observer--RGB 复合对齐偏移**，不能称作纯 camera delay：它同时可能吸收 observer 输入滤波、模型相位误差、RealSense source-stamp 语义和视觉映射误差。它只是 train-only post-hoc sensitivity，不是当前软件已有的权威时间常数；正式分析必须以 `delta=0` 为主口径，并单列 calibrated-alignment sensitivity。

当前 prototype 的 bag-wise/leave-one-bag-out observer→RGB sensitivity 范围如下；它不是冻结 release evidence：

```text
bag-wise correlation = 0.764--0.802
bag-wise RMSE        = 0.225--0.251 mm
bag-wise R^2         = 0.571--0.638
best train lag   = about +10 ms
```

这说明 observer 至少在单相机可见方向有中等偏强的 physical correspondence。不能仅凭 `|by| >> |bx|` 断言相机主要看见 `eta_y`：若两状态共线或激励不足，回归系数会不稳定。正式 projection 报告还必须包含 design-matrix condition number、standardized coefficients、按 block 的 profile/bootstrap 区间，以及独立 block 上 gain/intercept 的稳定性。当前只有两条 Bsmooth training run，具体系数和 `+12 ms` 都不能冻结为 release 常数。

## 5. 两条 W5 的 stale-epoch 虚拟网格初筛

下表在 Bsmooth 拟合的 signed projection 和 `+12 ms` alignment sensitivity 下，以 `selected_state_stamp + j*dt` 为虚拟网格，把 W5 horizon 节点与 observer/RGB 对齐。RMSE 单位均为相机投影毫米。

本次 prototype 把稀疏 RGB 线性插值到 33 ms 网格且没有 gap gate。G3R2 RGB source gap 的 P95 已约 `66.4--66.7 ms`、max 约 `133--135 ms`，而液体主频约 `5 Hz`；75 ms 和 140 ms 分别跨约 `135°` 和 `250°`。该插值会削峰、移动零交叉并改变 correlation/RMSE/phase。因此下表只证明“数据管线与大致衰减趋势值得继续分析”，**不得作为正式 RGB forecast 曲线或阈值依据**。

| j | horizon | horizon→observer corr | observer RMSE | horizon→RGB corr | RGB RMSE |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 0 ms | 1.000* | 0.000 | 0.747--0.765 | 0.230--0.253 |
| 1 | 33 ms | 0.985 | 0.053--0.058 | 0.743--0.754 | 0.235--0.249 |
| 2 | 67 ms | 0.908--0.912 | 0.123--0.134 | 0.672--0.687 | 0.254--0.275 |
| 3 | 100 ms | 0.866--0.871 | 0.155--0.168 | 0.623--0.644 | 0.268--0.296 |
| 5 | 167 ms | 0.751--0.761 | 0.193--0.211 | 0.522--0.525 | 0.293--0.323 |
| 10 | 333 ms | 0.552--0.585 | 0.244--0.273 | 0.319--0.365 | 0.324--0.368 |
| 15 | 500 ms | 0.394--0.411 | 0.288--0.306 | 0.177--0.200 | 0.347--0.375 |
| 30 | 1.0 s | 0.070--0.132 | 0.312--0.332 | -0.012--0.052 | 0.351--0.376 |
| 60 | 2.0 s | 0.072--0.113 | 0.296--0.323 | 0.134--0.145 | 0.333--0.366 |

`*` `j=0` 整行都不是 forecast：horizon stage 0 与 selected observer 相同是 `x0` 注入造成的恒等关系，RGB 数值只是 current/stale-state projection mapping；它们只能作接线与测量映射 sanity check。

### 5.1 当前允许的解释

- observer visible projection 与 RGB 有可重复对应；
- 以 stale liquid-state epoch 为原点，虚拟网格的匹配度总体随 `j` 增大而下降；
- `j=1` 的高相关主要是 hindcast consistency，不能称为 one-step forecast；
- `j=2` 基本进入 causal 区，`j=3` 才是当前 availability proxy 下首个全样本严格未来的网格点；
- `j=5` 后的虚拟网格相关性明显衰减，值得用正式 event-based replay 复核；
- 2 s terminal prediction 不能被当前实物数据当成可靠 physical forecast；
- 当前 slosh cost 使用完整 60-step horizon，因此有必要检查其 first-action gradient 是否由低可信远期 cancellation 驱动。

### 5.2 当前不允许的解释

- 不能说“167 ms 后液体模型必然错误”；
- 不能把表中 `j=1` 称作 33 ms causal forecast；
- 不能把 j>1 的全部误差归给 ODE，因为 planned controls 没有完整执行；
- 不能从经过长 gap 插值得到的 RGB 序列计算 phase、peak 或 zero-cross；
- 不能用单相机投影证明 `sqrt(eta_x^2+eta_y^2)` 正确；
- 不能把约两千个重叠 control cycles 当独立统计样本；
- 不能用 post-hoc 最优 lag 形成正式 claim。

## 6. 完整逐预测步曲线方案

### 6.1 分析对象

令：

```text
i = solver cycle
j = horizon step, 0..60
tau_j = j*dt
q_pred(i,j) = solver predicted liquid state
q_obs(t) = processed-IMU observer state
z_rgb(t) = signed RGB visible projection
```

当前 bag 的 provisional 时间：

```text
t_q(i,j) = selected_state_stamp(i) + tau_j
t_available(i) = horizon.header.stamp(i)       # 仅作当前 conservative proxy
lead_q(i,j) = t_q(i,j) - t_available(i)
```

正式 analyzer 输出两个互补产品：

```text
1. node-index curve（主要用于 observer/solver）
   在每个原生 model node t_q(i,j) 评价 observer；
   分开 hindcast 与 causal 样本，并同时画 j 与 lead_q。

2. native-RGB causal-lead curve（physical 主口径）
   保留每个 RGB source stamp t_rgb,k，绝不把 RGB 插值到 33 ms 网格；
   对每个在 horizon 覆盖内且 lead>0 的 RGB event，
   用 dense/high-accuracy replay 求同一物理时刻的 q_pred；
   再按 causal lead bin 汇总。
```

为保持“逐预测步”可读性，可把 native RGB event 按最近的 `j` 网格中心分箱，但误差必须在事件的**真实 source time**上计算，而不是先合成一个 30 Hz RGB 序列。每行同时保留 `j_nearest`、`model_grid_lead`、`causal_available_lead`；主图横轴用后者。

primary 使用：

```text
t_state_target = t_rgb,k
delta_observer_rgb_alignment = 0
```

train-only sensitivity 才使用 `t_state_target=t_rgb,k+delta_observer_rgb_alignment`，并重新计算 causal coverage。

正式 release 应新增：

```text
cycle_id
solver_tick_stamp
robot_x0_stamp
liquid_x0_stamp
horizon_origin_stamp
node_abs_stamp[j]
```

并由同一 cycle ID 贯通 selected-state、solver-input、pre-solve snapshot、solved horizon、first command 和 intervention。还要明确 `solve_start/solve_end/horizon_available`；否则 node stamp 仍无法回答“预测何时对控制器可用”。

### 6.2 四层 replay 分解与直接 operational 曲线

#### A. OCP 节点对高精度 planned replay：数值层

从相同 q0 出发，用 OCP 的 piecewise planned input，以更小内部步长/更高阶积分器重放到每个 model node 和原生 RGB event：

```text
e_numeric = q_ocp_planned - q_high_accuracy_planned_replay
```

它隔离 SQP-RTI、multiple-shooting 转录和 OCP 积分器 defect。若这一层已经不一致，先修 solver/integrator，不能归因给实物液体。

#### B. High-accuracy planned replay 对 realized-input replay：计划实现层

从相同 q0 出发，另一条 replay 使用未来实际 processed-IMU excitation：

```text
e_plan_realization
  = q_high_accuracy_planned_replay - q_realized_input_replay
```

actual-input replay 必须按原始 IMU event 的实际 `dt` 逐事件积分，再在 model/RGB 目标时刻采样；不能先压成 33 ms 单步。它使用了未来 IMU，是 **retrospective oracle diagnostic**，不是可在线部署的 forecast。

planned/realized input 不得点样本直接相减。应先统一：

- 坐标系与符号；
- 车体参考点、IMU 杠臂补偿；
- input 的物理语义与滤波带宽；
- actuator/communication delay；
- 每个控制区间的平均值或 impulse。

`horizon.a/alpha` 不能与 `/cmd_vel` 直接做 delta。命令层应比较同语义量，例如 wrapper 发布的 first command、预测 `v1/omega1`，或通过冻结 actuator model 后的量。

#### C. Realized-input replay 对 online observer：实现一致性层

```text
e_implementation = q_realized_input_replay - q_IMU_observer
```

两者使用同一模型和输入时应近似恒等；残差主要检查 timestamp、reset、离散化和代码路径，而不是物理模型正确性。

#### D. Realized-input replay 投影对 native RGB：物理层

```text
e_physical(t_rgb,k)
  = h_RGB(q_realized_input_replay(t_rgb,k)) - z_RGB(t_rgb,k)
```

这一层才接触独立物理测量，但残差仍联合包含 q0 误差、液体动力学误差和相机映射误差。必须在原生 RGB source stamp 上评价，不得插值 RGB。

#### E. 直接 operational forecast

实际 MPC 性能还要直接报告：

```text
q_ocp_planned -> future q_IMU_observer
h_RGB(q_ocp_planned dense prediction) -> native RGB
```

每个样本必须标出 `hindcast/causal`。除 persistence、hold-q0、zero-input/high-accuracy replay 等基线外，还应报告 skill score；仅有高 correlation 只能称 association/consistency，不能称 forecast gain。

#### F. Slosh cost、约束与 first-action relevance

用 pre-solve snapshot 的冻结参数和 generated objective 口径，逐节点重算：

```text
J_eta(i,j)
J_eta_dot(i,j)
J_slosh(i,j)
cumulative_slosh_cost_fraction(i,j)
modal_constraint_margin(i,j)
predicted_peak_step(i)
```

不能直接把现有 aggregate `cost_breakdown` 平均分摊到 60 步。per-node cost mass 只能说明代价值出现在哪里，不能证明哪些远期节点驱动了 first action。最终报告应同时给出：

```text
cost mass inside/outside T_operational 与 T_physical
peak/constraint-active node 是否位于对应 trust prefix 内
first-action sensitivity / adjoint
```

若当前工具链难以导出 adjoint，采用预冻结的 horizon truncation/discount ablation：固定同一 snapshot，依次截断或折扣 slosh 项，比较 first command 的变化。只有 sensitivity/ablation 能回答远期节点是否造成 gradient cancellation。这一步连接“模型匹配曲线”和“为什么 W5 选择当前 first action”。

### 6.3 Unconditional、replay 与 adherence-conditioned 的关系

完整 planned horizon 只在 solver 内存在，下一周期通常会重新规划。建议输出：

```text
unconditional curve:
  所有合格 origins，评价实际 MPC forecast

low-input-mismatch curve:
  只保留 cumulative_excitation_RMSE 低于 training-frozen 阈值的 origins
  仅作补充 sensitivity
```

模型本体分解应以 actual-input replay 为主；`low-input-mismatch` 子集会偏向低激励、低曲率、容易预测的 origins，不能称为无偏的“纯模型性能”。不能看 test RGB 后选择 mismatch threshold，阈值应由 training bags 的 realized-excitation noise/actuator distribution 冻结。

若 mismatch 是随 lead 累积的，逐 bin 筛选会让样本组成随横轴变化。应优先冻结一个所有 lead 共用的 origin set；若做不到，必须逐 bin 报告条件样本的 run、振幅、曲率、速度和输入分布，避免把样本变容易误判为模型变好。

解释规则：

- unconditional 差、conditioned 好：主要是重规划/执行器输入失配；
- 两者都差：时间、坐标或液体传播模型需要调整；
- observer 曲线好、RGB 曲线差：observer/solver 共享的内部模型没有得到物理液面支持，或相机测量映射错误；
- 两条曲线短期都好但 W5 无收益：回到 cost、matched-weight 和控制 trade-off，而不是继续改 observer 接线。

### 6.4 事件采样、dense prediction 与缺测

正式 primary：

```text
RGB:
  保留原生 image source stamp；不插值、不 hold-last-value
  每个有效 RGB event 产生一条 residual

model prediction:
  用 high-accuracy replay/dense output 到 RGB 的同一时刻
  记录距 OCP node 的左右 span 与 dense/replay 方法

observer:
  优先按原始 IMU event 实际 dt 重放到目标时刻
  或用 eta/eta_dot 做受 gap 限制的 Hermite 插值
  单独用 synthetic replay 量化插值误差

horizon:
  valid ACADOS_OK only；失败与空 horizon 留在 availability ledger
```

G3R2 RGB source gap P95 约 `66.4--66.7 ms`，max 约 `133--135 ms`。`75 ms/140 ms` 只能用于报告“如果要求邻近相机事件，coverage/missingness 如何变化”；尤其 `140 ms` 不得生成 phase、peak、zero-cross 或 primary RMSE。peak/zero-cross timing 只能在原生 RGB 时间序列上提取。

### 6.5 相机映射和 lag 的训练合同

禁止在同一 test bag 上同时拟合 `C_cam/delta_observer_rgb_alignment` 又报告匹配度。建议：

1. block/run 为 split 单元；
2. training 上拟合静止 slope baseline、`b0/C_cam` 和可选 `delta_observer_rgb_alignment`；
3. validation 选定 projection/lag 版本；
4. held-out test 只算一次曲线；
5. primary 强制 `delta=0`；calibrated lag 仅作预注册 sensitivity，除非另有独立时间标定冻结它；
6. 报告 design-matrix condition number、standardized coefficients、block-wise gain/intercept 和 profile/block-bootstrap interval；
7. 报告 observability rank/Gramian condition，不能用高 correlation 冒充四维可观。

当前只有两条 Bsmooth training run，不能同时提供可靠的 projection 拟合、validation 选型和不偏 test。故现有系数仅用于 feasibility sensitivity；下一批应至少预留独立 calibration/validation block，或先做几何相机标定降低自由参数。

### 6.6 每个 causal-lead bin / j 的指标

每条 bag、每个 step 至少输出：

```text
n_origins
n_native_events
coverage_fraction
causal_coverage_fraction
bias
MAE
RMSE / NRMSE
Pearson correlation
skill vs persistence / hold-q0 / zero-input baseline
calibration slope/intercept
peak timing error
zero-cross timing error（signed RGB only）
prediction interval coverage（若已有 covariance）
planned/realized excitation mismatch
per-node/cumulative slosh-cost relevance
```

相关系数不能单独决定模型正确；幅值 bias、gain 和 phase 必须同时通过。

统计单位不能是重叠 control cycle。当前 W5 只有两条 run，无法形成有意义的 run-cluster CI；必须分别画两条独立曲线并报告范围，不把约两千个重叠 origins 当成两千份独立证据。等独立 run 足够后，再按 bag/block/turn event 做 clustered bootstrap。

分别定义三类 trust horizon：

```text
T_operational  # OCP planned horizon 对实际闭环 observer/RGB
T_model        # realized-input replay 对 observer 的实现一致性
T_physical     # realized-input replay 投影对独立 RGB
```

每个 `T` 必须是**连续可信前缀**：`T_operational/T_physical` 从首个 causal bin 开始，retrospective `T_model` 从 replay origin 开始。corr/RMSE/skill gate、最少 event 数以及“连续多少个 bin 判定跌出”均在 test 前冻结；一旦跌出，不得因振荡相关在远期周期性回升而恢复 trust。hindcast 区只报告 consistency，不并入 `T_operational/T_physical`。

### 6.7 输出文件合同

建议新 analyzer：

```text
src/scout_apps/control/spmpc_local_planner/scripts/analysis/
  analyze_horizon_future_liquid_alignment.py
```

建议输出：

```text
analysis/horizon_future_alignment/
  manifest.json
  availability_ledger.csv
  per_cycle_node.csv
  per_cycle_rgb_event.csv
  per_step_metrics.csv
  per_causal_lead_metrics.csv
  per_run_metrics.csv
  projection_fit.json
  horizon_corr_vs_time.png
  horizon_rmse_bias_vs_time.png
  excitation_mismatch_vs_time.png
  representative_overlays.png
  report.md
```

`per_cycle_node.csv` 至少包含：

```text
run_id, block, condition,
pair_method, cycle_index,
selection_stamp, selected_state_stamp, horizon_header_stamp,
j, tau_sec, target_q_stamp,
model_grid_lead_sec, causal_available_lead_sec, causal_class,
pred_eta_x, pred_eta_x_dot, pred_eta_y, pred_eta_y_dot,
obs_eta_x, obs_eta_x_dot, obs_eta_y, obs_eta_y_dot,
pred_h_modal_mm, obs_h_modal_mm,
observer_gap_sec,
q_ocp, q_high_accuracy_plan, q_realized_input, q_observer,
planned_interval_impulse, realized_interval_impulse,
stage_slosh_cost, cumulative_slosh_cost_fraction, modal_constraint_margin,
command_intervened, valid, exclusion_reason
```

`per_cycle_rgb_event.csv` 每行是一个**原生 RGB event**，至少包含：

```text
run_id, cycle_index, rgb_event_id, rgb_source_stamp,
state_target_stamp, horizon_available_stamp,
causal_available_lead_sec, causal_lead_bin, j_nearest,
rgb_signed_slope_mm, rgb_status,
q_ocp_dense, q_high_accuracy_plan, q_realized_input,
pred_rgb_ocp_mm, pred_rgb_realized_mm,
projection_version, alignment_sensitivity_sec,
dense_method, nearest_node_span_sec,
valid, exclusion_reason
```

`availability_ledger.csv` 必须为每个预期 control cycle 保留 `missing/invalid/GOAL_REACHED/solve_failure/intervened` 状态，避免只在成功 horizon 上报告精度。

manifest 必须绑定 bag SHA-256、git revision、脚本 hash、配置、split、lag、gap gate 和单位。

### 6.8 实现复用与测试

可复用：

- `g4_replay_from_g3.py:598-614`：完整 horizon 数组解析；
- `analyze_g2s_source_selection.py`：source/state-stamp 读取与 observer gap-gated 对齐；不得复用其思路把 RGB 插值到每个 horizon node；
- `validate_g3_online_rgb_trial.py`：motion/arrival/RGB admission；
- `OnlineLiquidMeasurement.height_lcr_mm[3]`：signed projection 原始量。

新增测试至少包括：

1. synthetic linear oscillator 的已知 one-/multi-step error；
2. lag 正负号与单位转换；
3. `causal_available_lead` 的 hindcast/causal 边界；
4. j=0 selected-state identity，且整行不计 forecast；
5. native RGB event 不被重采样，dropout 只改变 coverage；
6. irregular-`dt` actual-input replay 与解析解/高精度基准一致；
7. invalid/GOAL/solve-failure horizon 进入 ledger；
8. selection/horizon 数量或顺序错位 fail-closed；
9. observer interpolation/replay gap fail-closed；
10. train/test leakage 检查；
11. persistence/hold/zero-input baseline 方向与 skill 计算；
12. overlapping cycles 不被当作独立置信区间样本。

### 6.9 建议执行顺序

```text
Phase A：不改运行时
  冻结 bag manifest、projection split、单位、lead 与 exclusion contract
  完成 native-RGB-event + dense replay analyzer
  输出 availability ledger、两条 W5 独立曲线及 baseline skill
  primary delta=0；+12ms 仅作单列 sensitivity
  header.stamp 只作 availability proxy，结论标为 descriptive

Phase B：只补诊断合同
  增加 cycle_id、solve start/end、available、robot/liquid-x0、node stamps
  不改变控制命令和 OCP

Phase C：新 matched-weight development
  同一 10-state solver、公共权重相同，仅 w_slosh=0/5
  预冻结 projection、lag、gap、threshold 和 split
  同时保存 OCP planned、高精度 planned replay、irregular-dt actual replay

Phase D：形成正式 fidelity gate
  输出 held-out causal curves、三类 trust horizon、plan-adherence 分解
  用 adjoint 或预冻结 truncation/discount ablation 检查 first action
  再决定 model/actuator/cost/constraint 修改
```

Phase A 能回答当前 bag 的描述性问题；只有完成 B/C/D，才有资格把逐预测步曲线作为新 release 的正式模型证据。

## 7. 决策门

正式阈值不能由本次 test 结果反向设定，应先根据 RGB 重复性、signed slope noise 和 matched-weight training 数据冻结。决策顺序为：

1. observer→RGB held-out visible projection 先通过；否则先修测量映射/液体模型。
2. 从 **first causally available step/bin** 开始检查 observer/RGB forecast 与 persistence/hold/zero-input baseline；当前数据不能固定写成 `j=1`。
3. 分别得到 `T_operational/T_model/T_physical`：从首个 causal bin 开始的连续可信前缀。
4. 用 per-node cost 加 first-action sensitivity/adjoint，或预冻结 truncation/discount ablation，检查 slosh 项与约束的决策作用是否位于可信前缀内。
5. 若 first action 主要受可信区外节点影响，候选才是缩短/折扣 slosh horizon、修 actuator/model；不能只凭 cost mass 宣称远期 cancellation。
6. 只有 constraint-relevant peak time 落入 `T_physical`、且 held-out false-safe 得到校准后，才讨论 soft/hard liquid cap。

当前描述性结果只支持：

```text
observer->RGB current-state mapping     = 有重复对应
j=1                                    = 主要是 hindcast，非 forecast
j=2                                    = 基本 causal，仍需按 coverage 与 baseline 复算
j=3                                    = 当前 header proxy 下首个全 causal 网格点
远期虚拟网格                           = association 衰减，尚未完成物理归因
full 2 s horizon                        = 尚未验证
```

## 8. 0801 native-event 正式描述性复算结果

### 8.1 可复现产物

Analyzer：

```text
src/scout_apps/control/spmpc_local_planner/scripts/analysis/
  analyze_horizon_future_liquid_alignment.py
  horizon_liquid_replay.py
  summarize_horizon_future_liquid_alignment.py
```

最终输出：

```text
/data/a/slosh_bags/real/
  20260801_spmpc_g3r2_w5s10_paired_confirmation/H0/
  analysis/horizon_future_alignment/native_rgb_replay_20260802_v3/
```

主要阅读入口为：

```text
report.md
native_rgb_correlation_vs_causal_lead.png
native_rgb_rmse_vs_causal_lead.png
observer_projection_vs_grid_step.png
selected_metrics.csv
summary_manifest.json
```

本次使用 `1048/1045` 个 `motion_start..motion_end` W5 origins；每个 horizon 的 stage 0 与 `selected_state_stamp` 精确对应的 READY observer 四维 q 最大误差均为 `0`。RGB 始终保留原生 source stamp，没有生成 30 Hz 插值 RGB。

两条 Bsmooth 在每 run 先扣除 motion 前 3 s 的 `median(R-L)` 静态 slope baseline，再用 motion 到 tail 的 `2157` 个原生 RGB event 拟合。Primary `delta=0` 得到：

```text
z_RGB_centered ~= 0.0265 -155.094*eta_x -1619.030*eta_y
train corr      = 0.729
train RMSE      = 0.253 mm
train R²        = 0.531
eta_x/eta_y corr = 0.040
standardized feature condition number = 1.041
```

`+12 ms` observer--RGB alignment sensitivity 为：

```text
train corr = 0.789
train RMSE = 0.227 mm
train R²   = 0.622
```

它仍是复合 alignment sensitivity，不是纯 camera delay；下面结果全部以 `delta=0` 为主口径。

### 8.2 按真正 causal lead 的 native RGB 曲线

下表为 `correlation / RMSE(mm)`；每条 run 单独报告，没有把重叠 event--horizon pairs 当独立实验重复。

| horizon 可用后的 causal lead | W5 Block 01 OCP | W5 Block 02 OCP | Block 01 zero-input free | Block 02 zero-input free |
| --- | --- | --- | --- | --- |
| 0--33 ms | `0.646 / 0.317` | `0.626 / 0.302` | `0.624 / 0.325` | `0.615 / 0.304` |
| 33--67 ms | `0.588 / 0.342` | `0.593 / 0.312` | `0.558 / 0.352` | `0.586 / 0.312` |
| 67--100 ms | `0.559 / 0.350` | `0.582 / 0.313` | `0.545 / 0.354` | `0.577 / 0.313` |
| 100--167 ms | `0.422 / 0.385` | `0.448 / 0.347` | `0.439 / 0.381` | `0.469 / 0.342` |
| 167--333 ms | `0.263 / 0.412` | `0.326 / 0.370` | `0.257 / 0.413` | `0.339 / 0.367` |
| 333--500 ms | `0.110 / 0.429` | `0.158 / 0.390` | `0.098 / 0.431` | `0.168 / 0.389` |
| 500--1000 ms | `0.039 / 0.428` | `-0.018 / 0.399` | `-0.029 / 0.433` | `-0.080 / 0.401` |

描述性 operational 分区因此为：

```text
0--100 ms    中等可见方向信息
100--167 ms  明显进入衰减区
167--333 ms  较弱
333 ms 后    不宜作为可靠 physical forecast
500 ms 后    两条 run 均无稳定线性预测信息
```

这不是预冻结 release threshold，因此不能把 `100 ms` 直接写入控制器作为正式常数；但它足以否定“当前完整 2 s horizon 已被实物验证”。

### 8.3 四层 replay 分解结果

#### 数值层：OCP 对 high-accuracy planned replay

piecewise `a/alpha`、连续 `v/omega`、最大 3.333 ms RK4 子步的 high-accuracy replay 与 OCP Hermite projection 全时域差异为：

```text
Block 01 RMSE/max = 0.03465 / 0.19572 mm
Block 02 RMSE/max = 0.03434 / 0.20396 mm
```

RK4 会轻微改善部分 100--333 ms correlation，但 500 ms 后仍接近零。所以 solver 转录/积分误差存在，却不是远期 forecast 衰减的主因。

#### 计划层：OCP 相对 zero-input free response

OCP 相对同一 q0 的 zero-input free-response MSE skill：

| causal lead | Block 01 | Block 02 |
| --- | ---: | ---: |
| 0--33 ms | `+4.6%` | `+1.2%` |
| 33--67 ms | `+5.2%` | `+0.1%` |
| 67--100 ms | `+2.2%` | `-0.2%` |
| 100--167 ms | `-2.1%` | `-3.1%` |
| 167--333 ms | `+0.5%` | `-1.4%` |

也就是说，短期可预测性主要来自 q0 和自然自由振荡；当前 W5 planned future controls 相对 free response 的新增预测收益很小，在部分区间反而略差。不能把短期相关全部归功于 W5 anti-slosh planning。

#### Observer 实现层：actual-IMU software replay

从 selected q0 开始，按每条 READY IMU observer 的原始不规则 `sample_dt`、`ax/ay`、连续 update count 和同一 reset epoch 做 exact-ZOH replay：

```text
endpoint 四维状态 max error:
  Block 01 = 2.03e-15
  Block 02 = 1.14e-15
```

因此 observer 时间步进、输入符号和名义 ODE 的软件实现是一致的；这条链没有发现需要重写 observer/solver 液体状态结构的证据。

#### 物理层：actual-input replay / future observer 对 RGB

actual-input replay 与 future observer 的 RGB 投影曲线数值近似重合，并在整个分析 lead 内保持约：

```text
Block 01 corr/RMSE ~= 0.73--0.74 / 0.28--0.29 mm
Block 02 corr/RMSE ~= 0.71--0.73 / 0.26--0.27 mm
```

这使用了未来实际 IMU，只是 retrospective oracle，不能在线部署。它说明“同一液体 ODE + 实际输入”仍与 RGB 保持中等稳定对应，而 planned OCP 曲线随 lead 快速衰减；当前主要矛盾更接近 future planned motion 与未来实际闭环 motion 不一致，而不是液体 ODE 方向完全错误。

### 8.4 当前模型修改决策

现阶段不建议重写液体 ODE，也不建议周期性用 observer 覆盖 future nodes。优先顺序为：

1. 保留每周期 q0 的 observer 注入；补权威 cycle/time contract。
2. 用同语义、同参考点、区间 impulse 方式继续定位 planned/realized input 差异。
3. 做预冻结的 slosh-horizon truncation/discount 或 first-action adjoint，确认 100--333 ms 以后节点是否实质驱动 first action。
4. 若远期低可信节点驱动控制，先缩短/折扣 slosh cost 的有效作用区，而不是提高 W5。
5. 以后有新实物时，再用独立 calibration/validation/test bags 判断是否只需重标定固有频率、阻尼和输入增益。
6. 在 `T_physical`、false-safe 和 constraint-relevant peak 未独立验证前，不启用无 slack hard cap。

## 9. 与上一份解决方案的关系

本文把上一份方案中的 P0 具体化：

[20260802_IMU_observer与solver液体状态一致性解决方案分析.md](./20260802_IMU_observer与solver液体状态一致性解决方案分析.md)

在本曲线完成前：

- 不做低频 observer 定时重置；
- 不把当前 measurement 覆盖 future nodes；
- 不直接启用 1 mm 无 slack hard cap；
- 不增加 W5 权重追求内部 horizon 下降；
- 不把单相机结果外推为完整二维液面保证。

优先级冻结为：

```text
逐预测步曲线
  -> matched-weight 因果隔离
  -> 时间/actuator/model 修正
  -> 可观测 fusion 或 bounded disturbance
  -> soft constraint
  -> optional hard cap
```
