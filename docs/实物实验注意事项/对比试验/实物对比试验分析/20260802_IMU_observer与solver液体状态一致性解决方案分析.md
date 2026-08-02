# IMU observer 与 solver 液体状态一致性解决方案分析

日期：2026-08-02

范围：G3/G3R2 development 数据、当前 `continuous_mpcc_acados` 实现与液面约束架构

状态：**仅方案分析；未修改控制代码、参数、生成 solver 或实验协议，未启动新实验**

代码审计基线：当前 `HEAD=bcedcf7`；G3R2 paired-confirmation 运行 release 为 `5f40d38`。工作区中的其他未提交改动不属于本文。

## 1. 结论先行

当前问题不应再表述为“IMU observer 没有送进 solver”。对 G3R2 的 W5 solver，本控制周期 selector 时刻取得的最新合格 processed-IMU snapshot 已经在**每次实际 acados 求解**时写入 MPC stage 0，并通过 `lbx=ubx=x0` 成为等式约束。warm start 只能改变求解初猜，不能覆盖该等式。selector 之后、acados 调用之前仍可能到达新 IMU，因此这里的“最新”必须限定在 selector 时刻。

G3R2 四包的 bag 复核也支持这一点：每个 `/spmpc/debug/solver_input_state` 样本中的 `eta_x / eta_x_dot / eta_y / eta_y_dot / h_modal`，都在 Float32 诊断消息精度下与本周期选中的 processed-IMU observer snapshot 完全相同。bag 直接支持 raw-selected 到 solver-input diagnostic 的恒等关系；实际 acados stage-0 等式则由同一 release 的源码/生成物审计证明。尚未完成的是 bag 级 `cycle_id` 下的 solved-x0 直接 gate。

接口赋值问题排除后，剩余候选原因至少包括：observer 没有真实液面 correction、solver future forecast 的输入/时间/坐标/参数失真、G3R2 两方法的控制权重没有完全匹配，以及低幅 RGB 的量化与重复性。现有数据不能把 W5 未确认唯一归因于其中任何一项。

推荐方案不是“隔一段时间再用 observer 重置 MPC”，而是：

1. 保留每次求解时 `x0_liquid = xhat_observer` 的现状，不做低频定时重置，也不把当前测量覆盖到未来 horizon 节点。
2. 先做 matched-weight 消融：保持 solver、平滑/控制权重和状态链一致，只切换 `w_slosh=0/5`，关闭当前 `w_control=0.3/0.1` 的配置混杂。
3. 建立三层残差：接口残差、one-step forecast innovation、RGB physical innovation，先把误差发生在哪一层量化出来。
4. 在可观测性和可辨识性通过后，用 IMU + 未截零、未取 max 的逐帧 RGB L/C/R 特征做分阶段离线辨识；不能一次自由估计所有尺度、旋转、bias 和 residual。
5. 在线若需要 RGB correction，必须按 source stamp 处理 delayed/out-of-sequence measurement；fixed-lag EKF/UKF、短窗 MHE 或降维估计器是候选，而不是默认四维状态一定可观。
6. MPC 可增加定义清楚、低维、受边界约束的 disturbance 状态，使稳定预测偏差能沿 horizon 保持或衰减；这不能代替 `omega_n/zeta` 等相位模型修正。
7. 液面约束先做带 slack 的软约束，再用 held-out RGB 一侧误差形成概率型 tightening；当前无 slack 的 modal hard cap 只能作为最后的模型安全层，不能直接当作真实 RGB 液面保证。

性质判断：**不是 observer→solver 赋值链的 bug，也不能据此否定 receding-horizon MPC 方法本身；优先需要补齐 matched-weight 因果隔离，以及状态估计、时间语义、输入/液体模型和不确定性约束。当前 W5 release 未通过 development confirmation，不能作为有效 anti-slosh 方法放行。**

## 2. IMU、observer、solver 和 RGB 的准确关系

### 2.1 四条链不是四个独立真值

当前关系可以写成：

```text
processed IMU ax/ay
        |
        v
同一二阶模态模型的变步长积分器
        |
        +----> processed-IMU observer 当前状态 qhat_k
                         |
                         +----> 每次求解的 solver stage-0 x0_liquid

MPC 候选 a、v*omega
        |
        v
同一名义二阶模态模型的 horizon 传播
        |
        +----> q_(1|k), ..., q_(N|k) 和 slosh cost / modal cap

RGB L/C/R 液面特征
        |
        +----> 当前独立 outcome H_vis；尚未反馈到 observer/MPC
```

令液体状态为：

```text
q = [eta_x, eta_x_dot, eta_y, eta_y_dot]
```

processed-IMU observer 使用接受的传感器时间间隔积分：

```text
qhat_(k+1) = F(dt_imu, theta) qhat_k + G(dt_imu, theta) a_imu,k
```

其中 `a_imu=[ax, ay]` 是去重力、偏置、滤波和杠臂处理后的实测激励。当前实现没有 `z_liquid - h(qhat)` 形式的液面 innovation，所以它更准确的名称是 **IMU-driven liquid model state**，而不是直接测得的液体状态。

MPC horizon 使用：

```text
eta_x_ddot = -2*zeta*omega_n*eta_x_dot - omega_n^2*eta_x - kappa_x*a
eta_y_ddot = -2*zeta*omega_n*eta_y_dot - omega_n^2*eta_y - kappa_y*v*omega
```

因此 observer 与 solver 共享名义固有频率、阻尼和单位输入增益，但激励来源不同：前者是已经发生的 IMU 加速度，后者是 solver 候选控制产生的理想 `a` 和 `v*omega`。这正是“当前状态一致但未来预测仍可能错”的原因。

G3R2 当前冻结 RGB 主指标为：

```text
H_vis = max(0, causal_median(height_max_lcr_mm, 5))
```

其中 `height_max_lcr_mm` 已经减去静止零点和 `height_bias_mm`，不能再次减 `h0`。它更接近真实液面 outcome，但 max、5 点因果中值和截零会丢失方向、相位与部分测量噪声结构，不能直接唯一还原四维 `q`。

`OnlineLiquidMeasurement` 实际保留了同一相机帧的三列 `height_lcr_raw_mm[3]`、零点修正后的 `height_lcr_mm[3]` 和 source header。估计器若使用视觉，应读取逐帧 L/C/R，而不是 outcome `H_vis`；带符号斜率还应减去静止期 slope baseline。共同 level/offset 主要用于零点和 nuisance 检查，不能默认当作第二个正交液体模态。

### 2.2 G3R2 的 observer→solver 链已经闭合

源码关系为：

1. 每个可进入求解的控制周期重新选择最新有效 observer snapshot，并写入 `input.slosh`：`spmpc_local_planner_ros.cpp:1335-1384`。
2. `fixed_robot_only` 只替换机器人状态，液体状态保持 selected observer 原值：`delay_phase_types.h:260-285`。
3. acados wrapper 每次用 `input.slosh` 填充 `x0[6..9]`：`continuous_mpcc_solver_acados.cpp:833-840`。
4. stage 0 每次设置 `lbx=x0`、`ubx=x0`：`continuous_mpcc_solver_acados.cpp:191-199`。
5. 生成 solver 满足 `NX=10`、`NBX0=10`、`idxbx0=0..9`，所以 stage-0 等式覆盖全部机器人和液体状态：`generated/acados/spmpc_slosh/acados_solver_spmpc_slosh.c:624`。
6. shifted/flatness/conservative warm start 本身也重置 stage 0；即使 capsule primal guess 较旧，也不能改变新的 stage-0 等式。

“每次求解”不等于每个 timer tick 都一定求解：缺 odom/reference/TF、observer 无效、到达终点或投影失败时会提前返回。IMU 约 50 Hz、控制约 30 Hz，相邻求解也可能在 freshness 门内复用同一个 snapshot。但只要真正进入 W5 acados solve，液体 `x0` 就来自本周期 selector。

G3R2 四包按发布序号配对、按诊断消息 Float32 精度核对如下：

| Row | motion+tail 内 solver-input diagnostic 样本 | 与 raw-selected 状态五字段完全相同 |
| ---: | ---: | ---: |
| 01 | 1195 | 1195 / 1195 |
| 02 | 1200 | 1200 / 1200 |
| 03 | 1196 | 1196 / 1196 |
| 04 | 1190 | 1190 / 1190 |

四包 `source_code=2`，liquid-delay rollout 均为 0。bag 目录为：

```text
/data/a/slosh_bags/real/20260801_spmpc_g3r2_w5s10_paired_confirmation/H0/
```

本批 `use_parabola_term=0`，所以 `/spmpc/slosh_height` 与 solver-input `h_modal_mm` 数值相同；若以后启用 parabola term，不能把二者一般化为同一指标。

还需要注意：Bsmooth 虽然也发布 paired observer/solver-input 诊断，但 `slosh_enable=false`，运行的是 non-slosh OCP，四维液体量只作 pre-solve 配对诊断，并不是 Bsmooth 优化器的实际 stage-0 状态；只有 W5 实际消费。

这些 diagnostic 在 `problem_.solve()` 前发布，motion+tail 内也可能包含 terminal controller 直接返回 `GOAL_REACHED`、没有调用 acados 的周期。因此上表不是 acados solve count。当前两个诊断数组也没有共同 Header/cycle ID；bag 支持 raw-selected→solver-input 恒等，actual solve→solved-x0 仍由源码和生成约束证明。

### 2.3 G3R2 中 solver 的方法方向

按 motion 到首次 `GOAL_REACHED` 后 5 s 的冻结窗口，solver-input diagnostic modal proxy P95 为：

| Block | Bsmooth | W5 | `Bsmooth-W5` | 方向 |
| ---: | ---: | ---: | ---: | --- |
| 01 | 1.04922 mm | 1.26962 mm | -0.22040 mm | W5 更差 |
| 02 | 1.09116 mm | 1.08182 mm | +0.00934 mm | W5 略好 |
| 条件平均 | 1.07019 mm | 1.17572 mm | **-0.10553 mm** | W5 总体更差 |

三条链总体都不支持 W5：

| 指标 | `Bsmooth-W5` | 解释 |
| --- | ---: | --- |
| RGB P95 | -0.0060 mm | 独立视觉 outcome 不支持 W5 |
| processed-IMU observer P95 | -0.0640 mm | IMU-driven model state 不支持 W5 |
| solver-input diagnostic modal P95 | -0.1055 mm | 30 Hz pre-solve 诊断的描述性条件均值不支持 W5；仅 W5 侧被优化器消费 |

observer 和 solver-input P95 不完全相等，是因为前者约 50 Hz、后者约 30 Hz，P95 统计使用的采样网格和样本数不同；不是 compose 又修改了 `q`。逐 block 方向也不是三条链完全相同，所以只能说总体均未提供 W5 收益，不能说 RGB 已证明所有内部时序逐点一致，也不能说 W5 普遍使液面更差。

这张表回答的是 **solver stage-0 输入方向**，不是 future horizon fidelity。Bsmooth 没有液体状态 horizon，无法与 W5 做同口径未来液体预测比较；因此 G3R2 仍不能证明 `q_(1..N|k)`、slosh cost gradient 或预测 peak 与后续 RGB 一致。该问题必须用 matched 10-state `w_slosh=0/5` 和 one-step/multi-step forecast gate 重新回答。

还有一个会影响机制归因的配置混杂：G3R2 实际比较的 `B_slosh` 默认 `w_control=0.1`，`B_smooth` 默认 `w_control=0.3`，runner 覆盖了 `w_smooth/w_alpha/w_du_*`，但没有覆盖 `w_control`；四包 summary 的 effective config 也分别记录为约 `0.1/0.3`。所以该批足以否决**这个实际 release**的放行，却不能把负结果唯一解释为 slosh model/cost 本身。后续纯 slosh-cost 消融应使用同一个 10-state slosh solver、完全相同公共权重和状态链，只切换 `w_slosh=0/5`。

旧 G3 的四-block **descriptive** 结构性分歧则是：delay rollout 后 solver-input 为 `+0.1876 mm`，认为 W5 四个描述性 block 都更好；rollout 前 observer 为 `-0.1732 mm`，RGB 为 `-0.0681 mm`。其中 Row 06/08 各有资格失败，不能当作四个严格有效配对。只保留两个 strictly eligible block 时，solver/observer/RGB 分别为 `+0.1625 / -0.1898 / -0.0566 mm`，排序分歧仍存在。

G3R2 批次不再观察到旧 G3 的总体排序翻转，这与移除 liquid rollout 一致；但跨批还同时改变了数据、配置和实验顺序，不能把差异识别为 robot-only 改动的单独因果效果。它没有为 W5 提供可重复 RGB 收益，因此移除错误 liquid rollout 只是必要的隔离步骤，不是完整模型保真方案。

## 3. 应把“不一致”拆成三层

若继续只比较三个 P95 标量，很难知道问题究竟发生在接口、预测还是物理映射。建议冻结三个不同残差。

### 3.1 接口残差：检查代码链，不评价物理正确性

```text
e_interface,k = q_solver,0(k) - q_selected(k)
```

G3R2 中它应逐分量为零。该 gate 用于发现 selector、delay compose、wrapper 或 solved stage-0 的接线错误。建议以后增加共同 `cycle_id` 和 Header，并在 postflight 直接验证 solved horizon stage 0，而不是像当前一样依赖两个无 Header 数组按发布序号间接配对。

### 3.2 one-step forecast innovation：检查 solver 对下一拍的预测

```text
nu_q,k = q_selected(t_k) - interpolate(q_(.|k-1), t_k)
```

它比较“上一拍 solver 对 `t_k` 的预测”与“这一拍 IMU-driven state”。必须先把两者变换到同一 modal/robot 坐标基，再按 horizon 节点的物理时间插值；不能简单拿数组 `k=1` 与下一个 ROS 消息近邻相减。

这个 innovation 会混合：

- 计划控制与实际执行加速度的差异；
- 执行器滞后和 command queue；
- measurement/state/solver 时间戳误差；
- 固定步长和变步长传播差异；
- 模型参数与坐标传播差异。

因此还应记录输入残差：

```text
nu_a,k = a_processed_IMU,k - a_model(realized v, omega, command history)
```

只有同时看 `nu_a` 和 `nu_q`，才能区分“输入没有按计划实现”与“相同输入下液体模型传播错误”。

### 3.3 RGB physical innovation：检查模型与真实液体

```text
nu_rgb,k = z_RGB(t_k) - h_RGB(qhat(t_k), theta_rgb)
```

这是 observer 当前缺少的真正液面 correction。由于 processed-IMU observer 和 solver 使用同一物理核，仅让两者互相吻合无法证明真实液面正确；必须引入 RGB、第二相机或其他液体敏感测量。

当前非负 `H_vis` 只能形成幅值残差。若要修正相位、正负号和速度状态，应优先从同一帧未截零、未取 max 的 L/C/R 构造：

```text
z_slope = (height_R - height_L) - static_slope_baseline
z_common = robust_common_level(L, C, R)  # 主要作为 offset/nuisance 检查
```

并通过 tank、modal、camera 三个坐标系建立 `h_RGB(q)`。单相机 L/C/R 是同一像平面内的三个横向采样点，通常只给出一个液面斜率投影。若两个模态具有相同二阶动力学、相机投影固定：

```text
y = c_x*eta_x + c_y*eta_y
```

则四维系统的 observability rank 通常只有 2。增加时间窗、UKF 或 MHE **不会创造缺失的观测方向**。在设计 estimator 前必须对实际时变路径和坐标定义计算 observability matrix/Gramian 的 rank 与 condition number：

- 若足秩且条件良好，再估计完整四维状态；
- 若秩不足，把在线视觉 correction 降到可观测的二维投影模态，并让未观方向只由模型传播、显式保留较大不确定性；
- 若必须控制/验证二维径向液面，则增加非共线第二视角或其他液体敏感传感器。

不能用一个无符号标量强行重置四个状态，也不能用“采用 MHE”替代可观测性证明。

## 4. 对用户提出的两个方案的判断

| 候选方案 | 判断 | 原因 |
| --- | --- | --- |
| 每隔一段时间用 IMU observer 修正 MPC 当前值 | **不采用低频定时版本；保留现有每次求解注入** | 当前已在约 30 Hz 的每次 solve 用 observer 固定 stage 0；降低修正频率只会让状态更旧 |
| 用当前 observer 覆盖 future horizon 节点 | **不采用** | future node 表示不同未来时刻，不能用同一个当前测量覆盖；会破坏动力学语义和优化梯度 |
| 用上一拍预测与本拍估计形成 innovation | **采用，P0** | 能直接量化 future forecast mismatch，并为输入/offset 模型提供残差 |
| matched-weight `w_slosh=0/5` 消融 | **采用，P0** | 先排除当前 `w_control=0.3/0.1` 配置混杂，避免把控制正则差异误归因于液体模型 |
| 用 RGB + IMU 做 output-feedback estimator | **条件采用，P1** | RGB 才提供独立液面信息；必须先证明测量方向可观，并正确处理延迟/OOSM |
| 无 slack 的液面硬约束 | **当前不采用** | 约束仍依赖有偏内部模型，且可能不可行、触发突然零指令 |
| 带 slack、误差裕量的液面约束 | **估计器通过后采用，P2/P3** | 可先作为可审计的风险层，并逐步验证 false-safe 与可行性 |

标准 output-feedback MPC 本来就是：

```text
x0,k = xhat_k
solve horizon
execute first control
receive next measurements
x0,k+1 = xhat_k+1
```

所以“每个周期修正当前状态”是正确结构，当前也已经做到。需要补的是高质量 `xhat`、正确时间语义和带残差的未来传播，而不是在一个 horizon 内反复硬重置。

## 5. 推荐架构

### 5.1 P0：先建立单一时间语义和预测审计链

每个状态和输入必须明确以下时间：

```text
t_source
t_measurement
t_phase_effective
t_state
t_solver_tick
t_command_effective
t_horizon[j]
```

当前存在三个需要先关闭的语义缺口：

1. processed-IMU pipeline 已计算 accel/gyro/alpha 的 phase-effective stamp，但 observer snapshot 仍以 `measurement_stamp` 作为 state stamp；freshness 可能看起来比滤波后的有效相位更“新”。
2. `fixed_robot_only` 的联合 `x0` 是 delay-predicted robot state 加 current liquid state，两个子状态并非天然位于同一个物理时刻。它是 G3R2 为移除错误 liquid rollout 所采用的隔离方案，不应被误写成最终一致时域模型。
3. predictor 用 `max(0.15, 0.22)=0.22 s` 作为统一时长，并在整段 0.22 s 同时传播线速度和角速度；它没有分别按线/角通道的 0.15/0.22 s 截止。

对 G3R2 四包 `/spmpc/debug/slosh_observer_selection` 的只读复算显示，有效 processed-IMU snapshot 在 selector 时刻的 `imu_state_age_sec` 为：P50 `25.66--26.06 ms`、P95 `34.58--35.03 ms`、max `39.10--41.19 ms`。这是约 15 ms sensor-delay 口径叠加 50 Hz 采样相位后的实际基线；以后不能用不现实的 `P95<=20 ms` 笼统 gate 掩盖时间语义。

建议的长期时间结构是把执行器一阶滞后/command queue 作为 MPC 状态或可审计输入模型，而不是在 MPC 外把机器人和液体分别变换到含糊的“未来生效时刻”。短期至少应做到：

- observer/estimator 只用已经发生的事件传播到 `t_solver_tick`，并分别保留“估计状态 epoch”和“最后有效测量 age”；
- robot 与 liquid `x0` 对齐同一 epoch；
- 尚未发生的执行延迟由 MPC 内部 actuator model 预测；
- 每个 horizon node 发布绝对预测时间；
- raw-selected、solver-input、solved-x0 使用同一 `cycle_id`；
- postflight 强制 `e_interface=0`、时间单调、source/fallback contract 完整。

### 5.2 P1a：先离线重识别，不直接在线自由调参

用现有和后续 development bag 建立离线数据集：

```text
input:
  processed IMU ax/ay at tube center
  omega/alpha, cmd_vel, odom

measurement:
  RGB L/C/R source-stamped heights
  directional slope / level features

candidate parameters（候选集合，不是同时全部放开）:
  omega_n, zeta, kappa_x, kappa_y
  IMU-to-modal planar rotation and bias
  actuator/input delay and first-order lag
  RGB scale, offset, projection and measurement delay
  optional slow additive model residual
```

这些参数存在结构性 gauge ambiguity：state scale、`kappa` 与 RGB gain 可以互相缩放；IMU scale 与 `kappa` 混淆；modal rotation 与 camera projection 通常只能辨识相对角；input bias、process residual 和 RGB offset 也会互相吸收。进入 MHE 前必须先固定：

- 一个状态尺度基准，例如冻结 `c_h` 或一个 `kappa`；
- IMU scale 的外部标定或固定值；
- 唯一坐标系基准角；
- RGB 静止 offset/slope baseline；
- 每个阶段只开放一种低维 bias/residual。

建议分阶段辨识：先时间/延迟，再 `omega_n/zeta`，再单一 gain/相对旋转，最后才测试低维 residual。每一步做 structural identifiability、profile likelihood、多初值和跨 block practical-identifiability 检查。当前 H0 弱激励数据不足以支撑所有参数同时自由拟合；若需要更丰富激励，应另建安全、预注册的 development 数据集。

在这些前置条件满足后，才使用离线 MHE、batch nonlinear least squares 或 prediction-error identification。训练、验证、最终测试按 bag/block 隔离；不得看过正式方法结果后再调整测量模型。

当前最应优先检查的结构假设为：

1. solver 用 `a`、`v*omega`，observer 用 tube-center processed IMU；两者是否经过同一 actuator/lever-arm/axis 定义。
2. `SloshDynamics::step()` 当前忽略 `omega_z`，没有旋转坐标系中的状态 transport/coupling；S 曲线转向时该假设是否造成相位或轴向投影错误。
3. `imu_to_base_yaw_rad=0` 仍是 development nominal choice，完整六轴外参和加速度 scale 尚未冻结。
4. 固定 `zeta=0.05`、几何推导 `omega_n` 和单位 `kappa=1` 是否能解释 held-out RGB 的频率、衰减、相位和幅值。

只有 held-out one-step/multi-step RGB 预测通过，模型才有资格继续产生 slosh cost 梯度或安全约束。

### 5.3 P1b：可观测降维 + delayed/OOS RGB-IMU 状态估计

第一步不是先选 EKF、UKF 还是 MHE，而是先证明测量模型可观。若单视角只观测投影模态，在线 estimator 应先降为：

```text
q_c = [eta_camera_projection, eta_camera_projection_dot]
```

完整四维 `q` 的未观方向继续由 IMU 模型传播并携带不确定性；若控制目标必须使用径向二维液面，则先增加第二个非共线液体测量。只有 observability/detectability 通过后，才增加一个低维 disturbance 或 bias；不要默认使用难以辨识的 `q+b_a+d_q` 大增广状态。

一个最小候选模型为：

```text
q_(k+1) = f(q_k, a_imu,k, theta) + E*d_k
d_(k+1) = rho(dt)*d_k + w_d
z_rgb,k = h_rgb(q_at_exposure_time, theta_rgb) + v_k
```

其中 `d` 必须先定义为 acceleration-like forcing、离散 state increment 或 output bias 中的一种，并给出单位、作用矩阵和 detectability；不能同时让 input bias、process bias 和 output offset 吸收同一 RGB residual。

delayed/out-of-sequence RGB 的完整运行逻辑应是：

1. IMU 约 50 Hz，用于高频 process prediction；solver 约 30 Hz 读取当前估计及 covariance。
2. ring buffer 保存覆盖最坏 source-to-publication age、抖动和 clock skew 的**全部有序事件**，以及 state/covariance、transition Jacobian/过程噪声；UKF 则保存或重新生成 sigma points。
3. RGB 约 27 Hz 到达时，以图像曝光/source stamp 插入历史位置，做 measurement update。
4. 从该位置开始按时间顺序重放后续所有 IMU **和 RGB** 事件，不能只重放 IMU；或者使用正式 augmented-state fixed-lag smoother/OOSM covariance 公式。
5. 显式处理过旧、未来、重复、乱序、clipped 和 dropout measurement；同帧 L/C/R 高度相关且有量化，应使用完整 `R`、innovation gate 或 robust loss。
6. 分别发布最后 IMU age、最后 RGB correction age、fixed-lag depth、state epoch 和 replay count，不能只发布一个看似“当前”的 state stamp。

`processing_latency_ms` 实际由 `now - image source stamp` 得到，包含相机/ROS 运输、排队和检测，不是纯 detector compute time。历史 G2S 三包的 source-to-publication age P95 为 `112.33 ms`；G3R2 四包只读复算的 bag-time publish-lag P95 为 `123.14--139.68 ms`，单包最大值为 `190.51--200.19 ms`。直接把 RGB 到达时刻当曝光时刻会产生明显相位错误，buffer 也必须按 tail 而不是只按 P95 定长。

必要的是正确处理 source-stamped delayed/OOS measurement；fixed-lag EKF、UKF、短窗 MHE 或显式 OOS smoother 都是候选。考虑到 G3R2 W5 solver P95 已为 `17.48 ms`，可先用离线 MHE 建模，在线 estimator 异步运行，控制主线程只读取已完成且带 age/covariance 的 snapshot。

若 RGB 只保留 `H_vis>=0`，不要上线普通四维 EKF 硬更新；只能将其作为幅值/风险 envelope 的低信息测量，并承认不可观部分。

同一相机被用于反馈和评价并不会自动让 efficacy 评价无效，但它不能再作为独立的物理模型验证。冻结离线处理同一图像只能叫“分析链隔离”，仍共享视角、遮挡和相机噪声。当前 formal no-video recorder 也没有原始图像；若要使用冻结离线图像 outcome，必须先建立新的 development/formal 录制协议。真正独立验证优先使用第二视角、盲人工标注或其他液体传感器。

### 5.4 P1c：bounded disturbance-augmented MPC

每次 MPC 仍执行：

```text
q_(0|k) = qhat_fused(t_solver_tick)
```

若 disturbance detectability 通过，未来传播可改为真正的增广动力学：

```text
q_(0|k) = qhat_fused
d_(0|k) = dhat

q_(j+1|k) = f(q_(j|k), a_model(u_j, actuator_state_j), theta) + E*d_(j|k)
d_(j+1|k) = rho(dt)*d_(j|k)
rho(dt) = exp(-dt/tau)
```

这样做的目的不是“伪造未来测量”，而是让已经观测到的稳定 forecast bias 在 horizon 内继续存在。`d` 的定义、单位、作用维度和时间常数必须冻结，并在 50 Hz estimator 与 30 Hz MPC 中使用一致的连续时间衰减。仅重置 `x0` 而不带 residual 时，模型从第一个未来节点开始就会重新回到原有系统性偏差；但若 fused `q` 和 `d` 同时吸收同一 correction，也会双重补偿。

这里要区分两类 correction：

- `nu_a` 主要用于 actuator/input model；
- `nu_rgb` 才能用于真实液体模型 residual。

不能只用 observer 与 solver 的差值宣称学到了物理模型误差，因为两者共享同一液体模型。`dhat`、`rho` 和参数 adaptation 都必须有幅值边界、变化率边界、stale/fallback 行为和 fail-closed gate；在离线 held-out 通过前，不允许在线参数自由漂移。

慢变 additive disturbance 只能修正低频 bias，不能修复 `omega_n/zeta` 错误造成的振荡频率和相位误差。没有 disturbance detectability、稳态 target calculation 和零稳态误差证明前，本文只称其为 **bounded disturbance-augmented MPC**，不把“offset-free”当成已经成立的性质。

### 5.5 P2/P3：约束从 soft 到不确定性 tightening，再到可选 hard

当前 hard variant 已实现：

```text
eta_x^2 + eta_y^2 <= eta_max^2
H_modal = c_h*sqrt(eta_x^2 + eta_y^2) <= H_max
```

stage 0 放宽，stage `1..N` 和 terminal 生效；当前没有 slack。这个约束只限制 modal proxy，不包含 RGB 真值。若 solver 不可行，当前 wrapper 返回零速度/零角速度；突然零指令本身也可能造成额外液体激励，因此“约束更严格”不自动等于“实物更安全”。

建议顺序如下。

第一步，显式 soft nonlinear constraint：

```text
H_pred,j + Delta_j <= H_safe + s_j
s_j >= 0
J_slack = lambda_1*s_j + lambda_2*s_j^2
```

acados 可通过 upper nonlinear slack 的 `idxsh` 与 `z/Z` penalty 实现，不一定把 slack 建成显式动力学状态。原生 `idxsh` 只提供非负 upper slack，没有 `s_j<=s_max` 上界。建议先冻结一个求解后的 acceptance/intervention threshold `s_accept`：必须发布每个节点的 slack、最大 violation、active node、margin 和 solver status；若 `max(s_j)>s_accept`，拒绝正常控制并进入明确的受控减速/recovery/fallback。

如果必须在 OCP 内真正约束 `s_j<=s_max`，需要把 slack 建成显式辅助决策变量并加 box bound、重新建模/codegen；此时所需 violation 大于 `s_max` 仍会不可行。slack 的作用是提高获得连续、可诊断减险解的可行性，不是保证 OCP 必然有解。有限 penalty 也会与 tracking cost 交换；若称 safety layer，需采用分层/lexicographic priority 或验证 exact-penalty 条件，并对 stage、terminal 和现有按 `N` scaling 分别冻结权重。

第二步，研究包含速度状态的 **free-response energy radius**。当前 cap 只看 `eta`，没有直接看即将把液面推向峰值的 `eta_dot`。在当前两轴同固有频率假设下，可定义：

```text
E_free = c_h^2*(
    eta_x^2 + eta_y^2
    + eta_x_dot^2/omega_x^2
    + eta_y_dot^2/omega_y^2)
H_free = sqrt(E_free)
```

`E_free` 对无外力名义线性振子的自由响应有能量/幅值含义，平方形式也避免 `sqrt` 在零点的梯度问题。它主要适合作为 terminal/recovery 指标；存在未来 `a`、`v*omega` 强迫时，它不是未来峰值上界。若用于全 horizon safety，必须逐节点使用受迫预测状态，并加入 input-to-state/reachable-set 上界。单相机投影也不能验证完整二维径向 `E_free`。

第三步，用 held-out RGB 一侧误差或 estimator covariance 形成**概率型** tightening：

```text
Delta_j = one_sided_error_quantile(
    speed, curvature, phase, observer_age, horizon_index)
```

或：

```text
Delta_j = beta*sqrt(J_h*P_j*J_h^T) + bounded_model_error_j
```

目标是控制 false-safe：模型判定安全但 RGB 已越过安全阈值。covariance 使用前必须通过 NIS/NEES consistency，且 `P_j` 应包含状态、disturbance 和冻结参数不确定性。条件 margin 若依赖 speed/curvature 等优化变量，查表可能不光滑且会被优化器利用；更安全的是预先冻结保守的 stage/regime envelope。

经验 quantile、conformal margin 或 `beta*sqrt(JPJ')` 给出的是 chance/probabilistic 语义，不是严格 robust guarantee。只有给出有界 disturbance set、tube/reachable set 和递归不变性证明时，才能称为 robust。对 `N=60` 也不能用逐节点 99% 代替整条 horizon 99%；应按独立 run/block/turn event，用每条 rollout 的 `max_j error_j` 做 joint calibration，并报告一侧置信下界。若零 false-safe 且希望在 95% 置信度下证明失败率小于 1%，粗略需要至少 299 个独立事件，当前几个 block 显然不够。

没有足够 held-out joint-coverage 证据时，`Delta_j=0` 的 hard cap 不能作为真实液面安全保证。

第四步，只有在 soft constraint、tightening、恢复轨迹和 fallback 全部通过后，才评估真正 hard cap。若当前状态已经高于阈值，可研究有明确语义的 stage-dependent recovery envelope，而不是让 stage 1 突然要求不可达的 1 mm；但“早期允许 slack、随后单调收紧”本身不保证可达或递归可行，仍需 viability/reachability 检查和 jerk-limited backup controller。

当前证据对直接 hard cap 并不乐观：

- N=9 的六个内部消融 variants 均 9/9 到点，其中两个 hard variants 使用 1 mm cap；这只证明同一模型代理下可跑通；
- 0.85 mm 仿真有更多 acados transient failures；
- 历史实物 hard 组出现 `B_slosh_hard=64`、`B_ours_hard=57` 次 solve failure，且存在 2.5 mm 参数没有真正落到 solver、实际仍可能为 1 mm 的疑点；
- G3R2 observer P95 已约为 `1.05--1.25 mm`，直接启用 1 mm、无 slack 的 stage-1 cap 有较高不可行风险；
- 内部模型若低估幅值或相位错误，即使 OCP 满足 cap，RGB 仍可超限。

所以 hard cap 是最后的 safety layer，不是修复 observer/solver forecast mismatch 的替代品。

### 5.6 若单相机不可观或模型 fidelity 仍不通过

这时不应强行上线四维融合或 1 mm model-hard cap。可退回一个声明边界更窄、但工程上可验证的保守方案：

1. 使用 matched-weight 的 non-slosh/Bsmooth 控制基线，冻结更保守的 `v/a/alpha/jerk` 和路径速度 profile。
2. RGB 可观投影、IMU age 和 model uncertainty 只驱动带 hysteresis 的 reference governor；风险升高或测量 stale 时平滑降低 `v_ref`，不突然把命令归零。
3. 未观的正交液体方向只作 shadow，不宣称二维径向液面 hard guarantee；若论文/安全目标必须覆盖它，增加第二视角或液体传感器。
4. 继续记录 one-step/multi-step residual，为下一轮模型升级提供数据，但不把内部 proxy 当 efficacy truth。

该 fallback 牺牲一部分速度和方法新颖性，却比“用不可观、未校准状态做无 slack hard constraint”更符合当前证据。

## 6. 建议的开发矩阵

以下矩阵用于区分到底是哪一层产生收益，全部属于后续 development，不计入正式 `40/64/88`：

| ID | 状态/模型 | MPC residual | 液面约束 | 目的 |
| --- | --- | --- | --- | --- |
| V0a | 当前 G3R2 release，保留 `w_control` 混杂 | 无 | 无 | 冻结“不放行”的实际 release 基线，不作机制归因 |
| V0b | 同一 10-state solver、公共权重完全相同，仅 `w_slosh=0/5` | 无 | 无 | 纯 slosh-cost matched-weight 消融 |
| V1 | V0b + 统一 timestamp/epoch，当前物理参数 | 无 | 无 | 单独验证时间语义修正 |
| V2 | V1 + 离线重识别参数/旋转/输入模型 | 无 | 无 | 验证名义模型 fidelity |
| V3 | V2 + observability-gated RGB-IMU OOS estimator | 无 | 无 | 验证更好的 `x0` 是否有用 |
| V4 | V3 | bounded low-dimensional disturbance | 无 | 验证 horizon bias correction |
| V5 | V4 | bounded disturbance | soft cap + audited slack threshold | 验证可行、连续的风险约束 |
| V6 | V5 + held-out uncertainty | bounded disturbance | probabilistically tightened soft / optional model hard | 最后验证安全层 |

不建议跳过 V0b--V4 直接从当前 release 跳到 hard cap，否则 hard 结果仍无法区分“公共 cost 配置差异”“模型真的安全”“模型低估真实液面”与“solver 因不可行频繁归零”。

### 6.1 建议预冻结 gate

以下是进入新一轮实车 development 前应冻结的建议门槛，不是对现有数据的追认：

| 层 | 建议 gate |
| --- | --- |
| causal contract | matched row 的 solver/state chain 和全部公共 cost/limit 必须相同，只允许预注册的单一因子变化；禁止再混入 `w_control=0.3/0.1` |
| interface | 每个有效 W5 acados solve 的 selected→solver-input→solved-x0 四维误差在数值容差内为零；共同 cycle ID；source coverage 100%，reset/fallback 0 |
| observability/ID | 对实际时变测量模型报告 observability rank/Gramian condition；对开放参数报告 structural/practical identifiability，秩不足不得进入四维融合 |
| time | 分开报告 phase-effective IMU age、传播后 state-epoch error、最后 RGB correction age、fixed-lag depth 和 RGB max gap；IMU age 建议不差于当前 P95 `35 ms`/max `45 ms`，而不是错误要求 P95 `20 ms` |
| model | held-out observable signed output 报告 bias、gain、RMSE/NRMSE、约 5 Hz coherence/phase、peak/zero-cross timing、multi-step interval coverage 和最差 block；相关系数只作辅证，若保留则至少 `0.81`（覆盖相对 G3 `0.706` 提高 `0.10`） |
| runtime | estimator 异步运行；测量/TF→command publish 的真实 critical path、锁等待和 timer jitter 一起统计，建议 P95 `<=25 ms`、P99.9 `<33.3 ms`、deadline miss `<0.1%` 并给置信界 |
| constraint | stress replay/sim 中 solver failure 0；原生 slack 的 `s_accept`（或显式辅助变量的 `s_max`）、最大 command jump、恢复时间、fallback 成功率和最差 tracking 冻结；slack/margin/active node 100% 可审计 |
| chance safety | 用独立 run/block/turn event 的 horizon-max residual 做 joint calibration 并给一侧置信下界；在样本量不足前不得声称 99% real-liquid coverage 或 robust guarantee |
| efficacy | 重新走 matched-weight 冻结 ABBA；至少 2/3 RGB 正向 block，平均 RGB P95 改善建议 `>=0.10 mm`、RGB RMS 不恶化，并检查区间和 single-block dominance；当前约 `0.076923 mm` 主量化梯级下不再使用 `0.05 mm` 作为实质收益门槛 |

若 V2 仍不能在 held-out RGB 上复现正确相位、幅值和方法方向，应停止 W5/hard-cap 实车调权，升级液体/输入/测量模型。若 matched-weight V0b 与 V2--V4 的预测 fidelity 已通过但 ABBA 仍无效，届时才更有证据判断当前 slosh cost/控制方法本身没有实物收益。

## 7. 明确不建议做的事

1. 不把“每隔 0.5 s 或 1 s 重置一次”当成改进；当前已经每个 solve 重置 stage 0，低频版本更差。
2. 不把当前 observer 值复制到全部 future nodes；这会把不同未来时刻错误地设成同一状态。
3. 不用无符号 `H_vis` 直接重置 `eta_x/eta_y/eta_dot`，也不把速度状态随意清零。
4. 不在模型 fidelity 未通过前继续放大 `w_slosh`，否则只是更强地优化错误梯度。
5. 不把当前 1 mm modal hard cap 写成“真实液面不超过 1 mm”或“无溢出保证”。
6. 不允许在线自由漂移 `omega_n/zeta/kappa/delay`；所有 adaptation 必须有边界、速率限制和 fail-closed。
7. 同一 RGB 可以继续报告控制 efficacy，但不能同时被称为独立物理模型验证；若主张独立真值，需要第二视角、盲标注或其他液体传感器。
8. 不把 G3R2 的 robot-only 混合时刻 `x0` 当作最终物理一致结构；它只是相对旧 G3 更可证伪的 development 隔离方案。
9. 不用 MHE/UKF 的时间窗替代 observability-rank 证明，也不在同一阶段自由拟合 state scale、`kappa`、IMU scale、RGB gain、rotation 和多种 bias。
10. 不把逐帧/逐节点经验 99% coverage 写成整条 horizon 的 robust 99%；必须说明独立统计单元、joint calibration 和置信度。

## 8. 方法问题还是代码结构问题

当前证据支持的准确回答是：

```text
observer -> solver-input diagnostic：G3R2 bag 按发布序号的五个 Float32 字段一致。

solver-input -> actual stage-0：release 源码/生成物证明全 10 状态等式约束；尚缺 bag 级 cycle-ID solved-x0 gate。

observer 本身：目前不是带液面 correction 的 observer，而是 IMU 驱动的开环模型状态。

solver future model：输入实现、时间、旋转坐标和真实 RGB 映射尚未闭环验证。

当前 G3R2 比较：B_slosh/B_smooth 的 w_control=0.1/0.3，并非纯 slosh-cost 单因素实验。

当前 W5 实例：原计划 6 条的 development paired-confirmation 在完成 4 条后作 post-hoc futility stop，冻结 gate 未确认，不能放行；不能外推成 W5 普遍更差。

广义 MPC / output-feedback 方法：尚不能由本批否定；需要先完成估计器、模型残差和不确定性约束结构。
```

因此第一步是补齐 **matched-weight 因果隔离和时间/预测审计**；其后才是估计—预测—真值之间的软件/模型架构。不是继续修一个不存在的 observer→x0 赋值问题，也不是立刻用硬约束掩盖预测误差。

## 9. 证据与实现入口

本地已有足够的实现先例，当前无需为了“有没有方案”优先外网搜索：

- G3 八包链路分歧：[20260801_G3八包_RGB与液体状态链路分歧分析.md](./20260801_G3八包_RGB与液体状态链路分歧分析.md)
- G3 delay 根因与 G3R 决策：[20260801_G3延迟状态失配与G3R放行分析.md](./20260801_G3延迟状态失配与G3R放行分析.md)
- G3R2 paired futility：[20260801_G3R2_W5_S10四条ABBA配对确认与Futility停止分析.md](./20260801_G3R2_W5_S10四条ABBA配对确认与Futility停止分析.md)
- observer selection：`src/scout_apps/control/spmpc_local_planner/src/ros/spmpc_local_planner_ros.cpp:1335-1443`
- robot-only compose：`src/scout_apps/control/spmpc_local_planner/include/spmpc_local_planner/ros/delay_phase_types.h:260-285`
- 统一 0.22 s predictor：`src/scout_apps/control/spmpc_local_planner/src/ros/execution_state_predictor.cpp:33-40`
- stage-0 equality 与 warm start：`src/scout_apps/control/spmpc_local_planner/src/solvers/continuous_mpcc_solver_acados.cpp:191-199,466-510,833-919`
- 生成 stage-0 全状态索引：`src/scout_apps/control/spmpc_local_planner/generated/acados/spmpc_slosh/acados_solver_spmpc_slosh.c:624`
- IMU-driven observer：`src/scout_apps/control/spmpc_local_planner/src/estimation/slosh_observer_bank.cpp:77-119`
- 名义 MPC 液体模型：`src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_model.py:180-225`
- G3R2 权重混杂：`src/scout_apps/control/spmpc_local_planner/config/planner/variants.yaml:18-32,82-96`
- 当前无 slack hard cap：`src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_constraints.py:56-87`
- RGB L/C/R message：`src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg`
- G3 RGB 主指标实现：`src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_g3_online_rgb_trial.py:605`
- 本地 acados MHE 示例：`/home/a/acados/examples/acados_python/pendulum_on_cart/mhe/minimal_example_mhe.py`
- 本地 acados closed-loop MHE 示例：`/home/a/acados/examples/acados_python/pendulum_on_cart/mhe/closed_loop_mhe_ocp.py`
- 本地 acados nonlinear soft-constraint 示例：`/home/a/acados/examples/acados_python/tests/soft_constraint_test.py`
- 本地 acados slack formulation 示例：`/home/a/acados/examples/acados_python/pendulum_on_cart/ocp/slack_min_formulation.py`

最终推荐一句话：**继续使用每次 solve 的 observer/fused estimate 固定 solver `x0`；先做 matched-weight 因果隔离，再用通过可观测性证明的 source-stamped 液体测量产生 innovation，并以 bounded disturbance-augmented horizon 表示稳定残差；只有在 held-out joint error 可界定后，才从 audited-slack soft cap 走向概率 tightening 和可选 model-hard cap。**
