# spmpc_local_planner

> 当前论文目标方法见：[`Methods章节组织思路.md`](../../../../docs_for_offlineslosh/Methods/Methods章节组织思路.md)。
>
> 当前实验目标与放行边界见：[`实验章节组织思路.md`](../../../../docs_for_offlineslosh/Experiments/实验章节组织思路.md)。
>
> 2026-08-20 模块化实现与仿真记录见：[`20260820_PhaseRejoining模块化代码修改与仿真报告.md`](../../../../docs_for_offlineslosh/代码修改报告/20260820_PhaseRejoining模块化代码修改与仿真报告.md)。
>
> [`README_METHOD.md`](README_METHOD.md) 记录的是 2026-07 旧版 full-horizon online S-MPCC 口径，只作为历史参考，不再代表当前论文主线。

`spmpc_local_planner` 是 Scout Mini 开口液体运输实验中的局部规划与控制包。当前论文目标方法已经从“在线每周期重新规划整段防晃动作”调整为：

```text
Phase-Rejoining Residual S-MPCC

离线先生成完整防晃名义序列；
在线只做有限纠偏；
只有纠偏后仍能接回后续防晃尾段，才接受这次纠偏。
```

这里必须区分“论文目标”和“当前可运行版本”：

| 层级 | 当前口径 |
| --- | --- |
| 论文目标方法 | `OfflineSloshOCP + Phase-Rejoining Residual S-MPCC` |
| 在线算法基座 | 10 维 alpha-state slosh-aware continuous MPCC + acados SQP-RTI |
| 默认运行行为 | `phase_rejoin.mode=off`，完全保持旧 continuous MPCC 行为 |
| 当前 Phase-Rejoin 代码 | development simulation release：模块、合同、短求解器和监督分支已接通 |
| 当前实物状态 | **`enforce` 仍为 NO-GO；本文档不放行 formal trial，工程检查只能使用 `off/monitor` 并遵守既有实物 SOP** |

当前代码已经包含：

- 最终 published command history；
- 线速度、角速度双通道延迟和可选一阶惯性预测；
- source timestamp / common epoch 状态对齐；
- odom 与 processed-IMU 液体 observer；
- `NominalSequenceArtifact`、`PhaseClock`、有限相位候选和 9 维经验 gate；
- 专用 10 维、`N=3` 的 `spmpc_phase_rejoin` acados solver；
- residual 第一拍、保存命令和受控停车三种分支；
- `off / monitor / enforce` 三种模式；
- `/spmpc/debug/phase_rejoin` 和逐周期命令审计。

但以下内容尚未完成，不能在论文或实物记录中写成已经具备：

- 正式 `OfflineSloshOCP` 及其完整 formal artifact；
- Scout 线/角双通道执行模型的最终 held-out 冻结；
- 保留新决策依赖的 delay-augmented OCP 或严格等价 bridge；
- 9 维 gate 之外的执行 buffer / 惯性状态兼容集；
- held-out gate、false-accept 和真实恢复证据；
- Phase-Rejoin 专用实物 runner、recorder 和 postflight；
- 实物 `enforce` 和独立 RGB 防晃收益。

当前诊断固定为：

```text
state_complete_for_certificate=false
```

所以当前对象只能称为 **phase-indexed empirical recovery gate/set**，不能称为 funnel、certificate、反馈恢复策略、递归可行性或鲁棒安全保证。

---

## 1. 总体工作流

### 1.1 试验前：几何路径与离线名义序列

目标工作流为：

```text
静态地图
  ↓
MBF 调用 ROS1 global_planner/GlobalPlanner
  ↓
几何 Path
  ↓
检查、重采样、必要的几何平滑、碰撞复核
  ↓
冻结 path 点列、frame 和 SHA-256
  ↓
OfflineSloshOCP
  ↓
完整运动—减速—液体沉降—零命令保持 artifact
```

边界要点：

- MBF 只负责在 trial 前调用全局规划器，不进入本包的在线局部控制回路；
- `global_planner/GlobalPlanner` 输出的是几何路径，不是带速度和防晃相位的时序轨迹，也不自动保证满足本文需要的平滑程度；
- 当前目标运行协议只接受冻结后的 `nav_msgs/Path`，默认话题是 `/scout/global_path_fixed`；代码虽能收到路径更新，但 formal trial 禁止热切换；
- `reference/preprocess_enable` 当前默认是 `false`，不能假定 planner 会自动把任意 MBF 路径处理好；
- 每条新路径都必须重新检查、冻结并生成自己的 OfflineSlosh artifact，不能跨路径复用；
- 当前研究范围是冻结路径、冻结容器和小幅偏离，没有在线避障；trial 中出现新障碍时应终止并停车，下一次 trial 前重新规划。

现有全局规划服务入口可以继续使用：

```bash
roslaunch scout_global_planner mbf_global.launch
```

该命令启动 MBF/GlobalPlanner 的规划服务，本身不替代本包的局部控制器。正式 trial 只使用它预先生成并保存的路径，不能在运行中把 MBF 新结果热切换给 Phase-Rejoin。

当前仓库中的 `generate_phase_rejoin_development_nominal.py` 只会从冻结 Path 生成动力学一致的 development 名义序列。它不是 `OfflineSloshOCP`，不能用于实物 formal release 或论文主结果。

### 1.2 在线：执行前沿后的有限修正

当前代码的数据流是：

```text
/odom + selected slosh observer + /scout/global_path_fixed
                + nominal artifact
                + 最终 published /cmd_vel history
                           │
                           ▼
source-time 对齐到共同状态时刻
                           │
                           ▼
ExecutionStatePredictor
线/角通道分别按 delay 和可选 time constant 传播
                           │
                           ▼
预测的共同执行前沿状态
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
       PhaseClock                有界、单调相位候选
              └────────────┬────────────┘
                           ▼
        nominal-relative residual OCP
        enforce 时使用专用 N=3 solver
                           │
                           ▼
              9 维 terminal empirical gate
                           │
                           ▼
 solver 第一拍 / artifact[j_f] 保存命令 / 请求 (0,0)
                           │
                           ▼
 terminal、limiter、tracking safety 与系统外独立安全覆盖
                           │
                           ▼
              最终 u_pub → /cmd_vel
                           │
                           └── 写回 command history
```

最重要的命令区分是：

```text
u_sol   solver 或 phase supervisor 提出的候选速度命令
u_pub   经过 terminal、限幅和安全链后真正发布的速度命令
```

执行预测和下一周期 artifact 对齐只允许使用真实 `u_pub` 历史，不能使用尚未真正发给底盘的 OCP action 或 `u_sol`。

上图中的 `j_f` 是“共同执行前沿对应的名义序列索引”，完整索引关系见第 4.3 节。

### 1.3 三种 Phase-Rejoin 模式

Phase-Rejoin 是叠加在现有 variant 上的运行模式，不是新的 `B_*` variant。

| 模式 | Phase 模块做什么 | 是否改变 OCP | 是否改变命令 |
| --- | --- | ---: | ---: |
| `off` | 完全关闭 | 否 | 否 |
| `monitor` | 加载 artifact、预测执行前沿、选相位并发布诊断 | 否 | 否 |
| `enforce` | 启用名义相对短 OCP、residual 硬边界和 terminal gate | 是 | 可能 |

`monitor` 自身严格不干预；若要比较 `off` 与 `monitor` 的命令一致性，两边的 variant、delay mode、limiter 和安全参数也必须完全相同。

---

## 2. 目录职责

```text
include/spmpc_local_planner/
├── core/          通用类型、SpmpcProblem、terminal、安全与 fallback
├── reference/     ReferencePath、投影、预处理、spline 和路径进度
├── dynamics/      二维低阶 SloshDynamics
├── estimation/    processed-IMU pipeline、observer bank 和 source selector
├── warm_start/    差速/全向 warm start
├── phase_rejoin/  artifact、时钟、相位候选、经验 gate 和 coordinator
├── solvers/       continuous MPCC、Phase-Rejoin acados wrapper 和 legacy 后端
└── ros/           ROS I/O、命令历史、执行预测、诊断和 node wrapper

src/
├── core/
├── reference/
├── dynamics/
├── estimation/
├── warm_start/
├── phase_rejoin/
├── solvers/
└── ros/

msg/
├── ControlCycleAudit.msg
├── PhaseRejoinDebug.msg
├── PredictedHorizon.msg
├── PreSolveSnapshot.msg
├── SloshObserverDebug.msg
└── SloshObserverSelectionDebug.msg

config/
├── planner/       通用参数和历史 variant
├── platforms/     Scout Mini 运动边界
├── containers/    容器与液体模型
└── experiments/   fixed_path / point_to_point 开发入口

launch/            ROS 启动入口
scripts/           runner、recorder 和 artifact development 入口
tools/analysis/    只读离线分析与 postflight 工具
tools/codegen/acados/  acados 模型源、代价、约束与唯一 codegen 入口
test/python/       Python 工具链回归测试
generated/acados/  本机生成的 solver；禁止手工修改
```

依赖边界：

```text
phase_rejoin core  不依赖 roscpp、ROS message 或 parameter server
ROS adapter        负责时间戳、话题、参数、生命周期和安全链
tools/codegen/acados  是 generated solver 的源头
generated/acados   只保存生成结果，不作为手工编辑源
```

当前 CMake 仍以一个主要 planner library 组织这些模块；目录职责已经拆开，target 级拆分可在后续维护中进行。

### 2.1 Scout Mini 尺寸与当前运动范围

平台配置文件：

```text
config/platforms/scout_mini.yaml
```

越野版 Scout Mini 当前使用的说明书尺寸口径为：

```text
长×宽×高: 612 × 580 × 245 mm
轴距:     451 mm
前/后轮距: 490 mm
说明书最高速度: 1.5 m/s
最小转弯半径: 0 m
```

包内采用更保守的 formal comparison 边界：

```yaml
robot:
  v_max: 0.8
  omega_max: 1.2
  a_max: 0.6
  alpha_max: 1.2
  geometry:
    length: 0.612
    width: 0.580
    height: 0.245
    wheelbase: 0.451
    track_width: 0.490
    footprint:
      type: polygon
      vertices: [[0.31, 0.2925], [0.31, -0.2925], [-0.31, -0.2925], [-0.31, 0.2925]]
    circumscribed_radius: 0.426
    inscribed_radius: 0.2925
```

说明：

- footprint 使用约 `0.620 m × 0.585 m` 的保守矩形；
- continuous MPCC 当前直接约束速度、角速度和加速度，不等于已经加入完整多边形碰撞约束；
- `obstacle_enable`、`corridor_enable` 和 `homotopy_enable` 不属于当前 Phase-Rejoin 主线；
- 2026-06-25 的旧 `B_ours` fixed-path no-regression 只证明写入尺寸没有破坏旧方案，不是新 Phase-Rejoin 的性能证据。

---

## 3. 新旧方法共用的基础模型

### 3.1 当前实际状态与控制维度

当前 acados 代码中保留三种需要分清的状态口径。

普通 B0 continuous MPCC 为 6 维：

```text
x_b0 = [px, py, yaw, v, s, omega]
q    = [a, alpha, v_s]
```

slosh-aware continuous MPCC 和当前 Phase-Rejoin 短 OCP 都是 10 维：

```text
x_slosh = [px, py, yaw, v, s, omega,
           eta_x, eta_x_dot, eta_y, eta_y_dot]

q       = [a, alpha, v_s]
```

经验 gate 使用 9 维状态，不包含路径进度 `s`：

```text
chi = [px, py, yaw, v, omega,
       eta_x, eta_x_dot, eta_y, eta_y_dot]
```

因此当前不能写成“在线 MPC 已经是 14 维或更高维执行器增广 OCP”。命令 buffer、双通道 delay 和可选一阶惯性目前由 OCP 外的 `ExecutionStatePredictor` 维护；正式 delay-augmented OCP 的最终状态维度要等执行模型和 buffer 表达冻结后再确定。

变量含义：

```text
a       线加速度控制量
alpha   角加速度控制量，即 omega_dot
v_s     虚拟路径进度速度，只推进 s，不等于底盘线速度
omega   底盘角速度状态
eta_*   二维液体模态位移和模态速度
```

### 3.2 路径误差与路径进度

continuous MPCC 不跟踪固定时间索引，而是沿冻结几何路径优化 `s`：

```text
contour error       法向路径误差
lag error           切向路径误差
progress reward     路径推进奖励
v / v_s tracking    防止物理速度和虚拟进度停滞
```

历史 anti-creep 参数来自 variant：

```text
w_v     v 对 v_ref 的 tracking penalty
w_vs    v_s 对 v_ref 的 tracking penalty
v_ref   参考速度
```

Phase-Rejoin `enforce` 仍保留 contour/lag，但短 OCP 的主要任务变成跟随名义状态和名义输入，并在有限 residual 范围内修正误差。

### 3.3 液体模型与 observer

`SloshDynamics` 传播二维低阶液体模态：

```text
eta_x, eta_x_dot, eta_y, eta_y_dot
```

内部模型液面 proxy 为：

```text
h_model = c_h * sqrt(eta_x^2 + eta_y^2)
```

液体状态目前可由两路 observer 提供：

| source | 作用 |
| --- | --- |
| `odom` | 当前默认 release，利用底盘运动估计 realized excitation |
| `processed_imu` | 可选 IMU pipeline，需通过 READY、freshness、source-time 和 fallback 合同 |

当前二维水平/yaw 适用范围内，动捕辅助冻结的平面外参为：

```text
imu_to_base_yaw_rad = 0
IMU → container center XY = [-0.100, +0.045] m
```

该结论要求 IMU、支架、容器安装不变，容器水平、地面基本平整且只考虑平面运动。当前模型不使用 Z；这不等于已经完成完整 6DoF 标定。processed-IMU 的 accelerometer 内参、主频幅相和端到端时间链仍需在 G0 中完成验收。

内部 slosh proxy 用于控制和机制诊断；真实防晃效果必须由独立 RGB 液面测量判断。

### 3.4 第一拍与最终发布命令

在 alpha-state 模型中，solver 第一拍先形成候选速度：

```text
u_sol.v     = clamp(v_origin     + a_0     * dt, 0, v_max)
u_sol.omega = clamp(omega_origin + alpha_0 * dt, -omega_max, omega_max)
```

旧 continuous 模式的 origin 通常是当前 solver input；Phase-Rejoin `enforce` 的 origin 是当前实现预测的共同执行前沿。候选命令随后还要经过 phase supervisor、terminal、limiter 和安全覆盖，最终结果才是 `u_pub`。

---

## 4. Phase-Rejoining Residual S-MPCC

### 4.1 名义 artifact

正式目标 artifact 应至少包含：

```text
冻结路径的 frame、点列/hash 和长度
固定 dt 与 contract_id
执行模型、液体模型、容器与命令边界
逐相位机器人状态、路径进度和液体状态
逐相位 q=[a, alpha, v_s] 与预期 u_pub
完整运动—减速—沉降—零命令保持尾段
逐相位 empirical gate
逐相位执行状态兼容条件
恢复动作及其证据等级
```

当前 loader 支持：

| schema | 当前用途 |
| --- | --- |
| `phase_rejoin_empirical_v1` | development 接口/rolling proxy；没有完整终端尾段，不能进入当前 `enforce` |
| `phase_rejoin_empirical_v2` | 检查固定周期、动力学转移、路径几何、液体模型、命令边界和 stop–settle–zero-hold 尾段 |

当前 V2 runtime 会核对 frame、path length、逐点几何、液体模型系数和命令边界，但 mandatory metadata 还没有完整绑定 path SHA-256、Scout 执行模型 artifact、容器/装液量 hash 和执行状态兼容集，也不能独立证明 evidence 标签的来源真实性。formal schema、只读 freeze validator 和 hash 链仍需补齐，不能把“V2 loader 通过”等同于正式资产全部冻结，更不能靠改 metadata 把 development artifact 升级成 formal artifact。

当前 V2 development 生成器仍会永久写入：

```text
evidence_level=development_only
offline_slosh_ocp=false
paper_main_result_eligible=false
```

`allow_development_artifact_in_enforce=true` 只用于 proxy 仿真分支测试，禁止用于实物。

当前 V2 的：

```text
recovery_contract=nominal_command_v1
```

表示每个索引的保存命令等于该索引的名义 published command。在线需要恢复时，当前代码实际取执行前沿索引 `j_f=front_index` 的 `kappa`，不是候选当前索引 `j` 的命令。它不是根据当前误差计算的反馈 recovery policy。

### 4.2 双通道共同执行前沿

当前候选配置为：

```text
control_frequency = 30 Hz
dt                 = 0.0333333333 s
linear_delay       = 0.15 s
angular_delay      = 0.22 s
linear_tau         = 0.0 s   # 仍是候选值，待辨识冻结
angular_tau        = 0.0 s   # 仍是候选值，待辨识冻结
```

共同栅格前沿按较慢通道计算：

```text
n_f = ceil(max(d_v, d_omega) / dt) = 7
```

当前 HEAD 必须区分两个时间基准：

```text
t_e     求解前调用 predictor 时的 delay_phase_now
t_pub   本周期求解、安全链结束后的真实命令发布时间
d_c     t_pub - t_e
```

当前实现的实际时间线是：

| 时间/索引 | 当前代码含义 |
| --- | --- |
| `t_e` | 用 source-stamped 状态和已有 `u_pub` history 开始预测；本周期新决策此时还不存在 |
| `t_e + 150 ms` | history-only 线通道采样位置；仍不含本周期新决策 |
| `t_e + 220 ms` | predictor 输出的物理共同执行前沿状态 |
| `j_f=j+7` | artifact 使用的共同前沿索引，对应 `7dt≈233.3 ms` |
| `t_e + 320 ms` | 当前短 OCP 从 220 ms predictor 前沿再走 3 步的模型 terminal epoch；尚不能等同于真实新命令的物理终端 |
| `j_e=j+10` | artifact terminal 索引，对应 `10dt≈333.3 ms` |
| `t_pub=t_e+d_c` | solver 和后续安全链结束后，才真正发布本周期新命令 |

因此当前物理前沿 `220 ms` 与名义索引前沿 `233.3 ms` 已有约 `13.3 ms` 错位，solve/publish delay `d_c` 也没有进入 predictor。正式目标应先对齐到预计/实际发布时间，再保留本周期新决策在双通道传播中的因果作用。

所以“液体短时域约 100 ms”只表示**共同执行前沿之后**的 3 步设计窗口，不是从原始传感器时刻或真实命令发布时间开始的完整预测时间。

### 4.3 PhaseClock 与相位候选

相位选择不是全局最近点，也不允许任意跳转。当前逻辑为：

1. `PhaseClock` 从本次序列起点建立绝对时钟；
2. 只在时钟索引附近搜索；
3. 不允许退到上一次已接受相位之前；
4. `max_clock_lead_steps` 限制相对绝对时钟的最大超前量；
5. 用位置、yaw、速度和液体 9 维误差选择候选；
6. 选定一次相位后，只求解一次 OCP。

代码中的索引含义为：

```text
j    候选当前相位
j_f  与共同执行前沿对应的名义相位
j_e  前沿后 N_l=3 的 terminal 相位
```

当前实现不使用自由时间缩放，也没有把 `tau` 或 `nu` 加入 OCP 状态/控制。

### 4.4 专用短时域 residual OCP

`enforce` 使用单独生成的：

```text
spmpc_phase_rejoin
```

它和 `spmpc_slosh` 使用相同的 10 维动力学和 3 维控制，但 horizon 固定为：

```text
N_s = N_l = N = phase_rejoin/liquid_horizon_steps = 3
3 个控制节点，4 个状态节点，约 100 ms
```

这样可以避免“只在前 3 步约束液体，后面又接一段不受重接条件保护的长几何尾巴”。`off/monitor` 仍使用原来的长时域 solver，默认 `N=60`、约 2 s。

短 OCP 保留路径跟踪，同时惩罚相对名义状态和输入的偏离。第一拍 residual 通过 OCP 内 stage-0 输入边界限制：

```text
|u_sol.v     - u_bar_pub(j_f).v|     <= max_residual_v
|u_sol.omega - u_bar_pub(j_f).omega| <= max_residual_omega
```

默认 development 边界为：

```text
max_residual_v     = 0.08 m/s
max_residual_omega = 0.20 rad/s
```

这些数值尚不是 formal 冻结结论。

### 4.5 9 维 terminal empirical gate

当前 gate 对 terminal 相位的 9 维相对误差使用对角椭球：

```text
m(e) = sum((e_i / r_i)^2)
m(e) <= 1  表示通过当前经验 gate
```

9 个分量为：

```text
x, y, yaw, v, omega,
eta_x, eta_x_dot, eta_y, eta_y_dot
```

它的正确含义是“在已有经验对象中，这个 terminal 误差还有接回尾段的可能”，不是“100 ms 后液面已经最低”。

当前 9 维 gate 没有显式包含 pending command buffer 和执行器惯性状态。即使 metric 通过，也不能称为完整可恢复证明；正式方法还需要独立的执行状态兼容条件以及 held-out false-accept 统计。

### 4.6 supervisor 与终端所有权

当前 `enforce` 分支为：

| 在线结果 | 提交给后续安全链的候选 |
| --- | --- |
| OCP 成功、terminal gate 通过且命令合同一致 | residual-bounded solver 第一拍 |
| OCP/terminal 失败，但当前 execution-front gate 通过 | `artifact[j_f].kappa`，当前 V2 中等于 `u_bar_pub(j_f)` |
| 状态、artifact、当前 gate 或命令合同无效 | 请求 `(0,0)` 受控停车 |

V2 artifact 只有在完整 slowdown–settle–zero-hold 尾段通过 loader 和 runtime contract 后，才可暂时接管普通 terminal clamp。最终 `GOAL_REACHED` 零命令锁存、包内 terminal/tracking safety 以及系统外独立安全覆盖始终拥有更高优先级；本包当前不提供在线人员或障碍物检测。

### 4.7 当前必须保留的 B0 缺口

当前 `ExecutionStatePredictor` 是 history-only：先用已经发布的旧命令预测共同执行前沿，再从该前沿启动 3 步 OCP。在线/实物 `enforce` 尚不能放行，原因是：

1. 当 `d_v=150 ms`、`d_omega=220 ms` 时，新线速度决策会在共同前沿前约 `70–83 ms` 已开始作用；
2. 当前固定前沿状态没有保留这段对“本周期新决策”的依赖；
3. 物理慢通道前沿 `220 ms` 与 `7dt≈233.3 ms` 之间还有约 `13.3 ms` 的 epoch/index 差；
4. 当前 predictor 的 evaluation epoch 还没有显式推进到“预计命令发布时刻”，实际 solve/publish 延迟及其估计误差仍需进入 G0；
5. 因此 proxy S0–S4 只能证明接口、时序和监督分支可运行，不能证明正式因果前沿正确，也不能证明防晃有效。

正式版本必须改成双通道 delay-augmented OCP，或使用严格保留相同决策依赖和时间索引的凝聚 bridge，然后重新完成 G0 总 lead 验证。

---

## 5. 保留的旧 continuous MPCC 与 legacy 后端

### 5.1 alpha-state continuous MPCC

`continuous_mpcc_acados` 仍是所有当前工作的在线求解基座：

```text
omega 是状态
alpha = d(omega)/dt 是控制
```

在 `phase_rejoin=off` 时，它按旧方法每个控制周期重新优化整个长时域中的局部运动、路径进度和第一帧命令。历史 `B0/B_smooth/B_slosh/B_ours` 都从这里运行。

这条路径必须保留，用于：

- no-regression；
- C0/旧 S-MPCC comparator 的开发；
- `monitor` 命令不干预验证；
- Phase-Rejoin 失效时的安全退回研究。

### 5.2 RouteB / direct-omega

`continuous_mpcc_direct_omega_legacy` 是早期定位 alpha-state stall/chatter 的诊断后端，不是当前论文目标方法。

B0 direct-omega：

```text
x = [px, py, yaw, v, s]       # 5D
u = [a, omega, v_s]
```

slosh direct-omega：

```text
x = [px, py, yaw, v, s,
     eta_x, eta_x_dot, eta_y, eta_y_dot]  # 9D
u = [a, omega, v_s]
```

这里 `alpha_max` 是输出 `cmd_omega` 的 rate clamp，不是 OCP 内的角加速度状态约束。RouteB 改变了 OCP 结构，不能混入普通 slosh 消融表。

### 5.3 primitive

`primitive` 后端基于候选控制序列 rollout 和 argmin，继续保留作：

- smoke/fallback；
- 早期 primitive 复现；
- 附录或工程对照。

它不是当前论文主线。

---

## 6. Solver、variant 与实验名字

### 6.1 generated solver

| generated model | 状态/控制 | horizon | 用途 |
| --- | --- | ---: | --- |
| `spmpc_b0` | 6D / 3D | 默认 60 | 普通 alpha-state MPCC |
| `spmpc_slosh` | 10D / 3D | 默认 60 | 旧 full-horizon slosh MPCC |
| `spmpc_phase_rejoin` | 10D / 3D | 固定 3 | Phase-Rejoin `enforce` 短 OCP |
| `spmpc_b0_direct_omega_legacy` | 5D / 3D | 默认 60 | RouteB B0 诊断 |
| `spmpc_slosh_direct_omega` | 9D / 3D | 默认 60 | RouteB slosh 生成入口，formal 未验证 |

对应编译宏：

```text
SPMPC_WITH_ACADOS
SPMPC_WITH_ACADOS_SLOSH
SPMPC_WITH_ACADOS_PHASE_REJOIN
SPMPC_WITH_ACADOS_B0_DIRECT_OMEGA_LEGACY
SPMPC_WITH_ACADOS_SLOSH_DIRECT_OMEGA
```

若专用短 solver 未生成或未链接，`enforce` 会在初始化时拒绝启动，不会退回到 60 步 solver 冒充短时域方法。

### 6.2 旧 `B_*` variant

旧 variant 继续保留，但名称含义不应升级：

| variant | generated model | 含义 | 当前角色 |
| --- | --- | --- | --- |
| `B0` | `spmpc_b0` | 普通 MPCC | legacy baseline / smoke |
| `B_smooth` | `spmpc_b0` | 只增加控制平滑 | legacy smooth baseline |
| `B_slosh` | `spmpc_slosh` | 在线液体状态/代价 | Phase-Rejoin 的 slosh-enabled 基座之一 |
| `B_ours` | `spmpc_slosh` | 旧在线 slosh + smooth | 旧完整方法，不再称当前完整方法 |
| `B_slosh_matched0/5` | `spmpc_slosh` | 旧 100 ms 液体代价匹配消融 | development only |

Phase-Rejoin `enforce` 要求 `slosh_enable=true`，但它仍是一个 mode overlay，不应创建“`B_ours` 就等于 Phase-Rejoin”的口径。

### 6.3 目标论文比较条件

新论文目标使用 `C0–C4`，而不是把旧 `B_*` 直接改名：

```text
C0 OrdinaryMPCC
C1 SmoothMatch
C2 OfflineReplay
C3 ResidualNoGate
C4 Full Phase-Rejoin
```

这些是目标实验协议，目前不是五个现成 runtime 开关。特别是 C2 的冻结时钟零 residual runner、C3 的 no-gate 确定性失败语义和 C4 的正式资产链都尚未完成。完整定义见实验章节组织思路。

---

## 7. 与 RGB 液面测量的边界

`spmpc_local_planner` 不依赖 RGB 液面结果闭环控制。

```text
spmpc_local_planner
  负责机器人/液体内部状态、OCP、phase decision、/cmd_vel 和诊断

realsense_liquid_measurement
  负责相机测量、有效性、标定和液面评价数据
```

必须区分：

```text
/spmpc/slosh_height       内部低阶模型 proxy
/spmpc/debug/slosh_*      observer / model 机制诊断
/liquid/measurement       独立相机测量结果
RGB max-LCR               论文物理主指标候选
```

内部模型不能作为“模型自身有效”的唯一证据。正式结果必须报告独立 RGB，同时报告有效帧、标定、source timestamp、延迟和失败 trial。

当前 recorder 默认不录原始图像流：

```text
RECORD_RGB=false
RECORD_ONLINE_LIQUID=true
```

是否保存原始 RGB 或只保存带 source timestamp 的派生 measurement，应由新的 Phase-Rejoin protocol/freeze 统一规定，不能沿用 README 的历史默认说法。

---

## 8. 配置文件与关键参数组

```text
config/planner/common.yaml             通用 MPCC、observer、delay 和 phase 参数
config/planner/variants.yaml           历史 B_* variant
config/platforms/scout_mini.yaml       Scout Mini 运动/几何边界
config/containers/tube_default.yaml    容器和液体模态参数
config/experiments/fixed_path.yaml     固定路径开发模板
config/experiments/point_to_point.yaml 点到点 smoke 模板
```

参数职责：

```text
platforms/         机器人硬边界和 footprint
containers/        容器、液体与模态参数
planner/variants   旧方法代价差异
planner/common     时间、observer、执行预测、phase 合同和 solver 通用设置
experiments/       单次实验入口，不作为物理参数 source-of-truth
```

当前关键组：

| 参数组 | 作用 | 默认状态 |
| --- | --- | --- |
| `slosh_observer` | 选择 odom 或 processed-IMU 液体状态 | `source=odom` |
| `imu_shadow` | processed-IMU 滤波、时间戳和杠杆臂 | 默认关闭 |
| `state_timing` | robot/slosh common epoch 对齐 | `require_common_epoch=true` |
| `delay_phase` | command history 与双通道执行前沿预测 | `mode=off` |
| `phase_rejoin` | artifact、候选、短时域、residual 和 mode | `mode=off` |
| `execution_contract` | post-solver limiter 是否改变已验证命令 | 默认 audit-only |
| `tracking_safety` | 路径投影和旋转安全门 | 开启 |

Phase-Rejoin development 默认值：

```text
phase_rejoin/mode                    = off
phase_rejoin/liquid_horizon_steps    = 3
phase_rejoin/max_residual_v          = 0.08
phase_rejoin/max_residual_omega      = 0.20
phase_rejoin/candidate/backward_radius      = 1
phase_rejoin/candidate/forward_radius       = 3
phase_rejoin/candidate/max_clock_lead_steps = 1
```

双通道默认值 `0.15/0.22 s` 只是当前候选，不是已冻结的 Scout 执行模型 artifact。正式实验必须由 published `/cmd_vel`、Nokov/odom 和 IMU 的完整 trial 辨识结果覆盖，并把物理执行延迟与 IMU 处理延迟分开。

`enforce` 初始化至少要求：

```text
continuous_mpcc_acados
slosh_enable=true
delay_phase/mode=fixed_closed_loop
delay_phase/require_complete_history=true
state_timing/require_common_epoch=true
专用 spmpc_phase_rejoin solver 可用且 N 匹配
V2 complete terminal tail 和 runtime contract 匹配
development artifact 默认禁止
post-solver limiter 开启时必须 fail closed on change
```

---

## 9. acados codegen 与构建

### 9.1 生成 solver

不要手工修改 `generated/acados/`。修改 `tools/codegen/acados/` 中的模型源后重新生成。

```bash
source /opt/ros/noetic/setup.bash
source /home/a/acados_venv/bin/activate
export ACADOS_SOURCE_DIR=/home/a/acados
export LD_LIBRARY_PATH=/home/a/acados/lib:${LD_LIBRARY_PATH:-}
cd /home/a/scout_ws/src/scout_apps/control/spmpc_local_planner/tools/codegen/acados

python3 generate_spmpc_acados.py --model b0
python3 generate_spmpc_acados.py --model slosh
python3 generate_spmpc_acados.py --model phase_rejoin
```

`--model phase_rejoin` 只生成在线 `N=3` 短 solver，不会生成 OfflineSloshOCP artifact。

RouteB 诊断需要时再生成：

```bash
python3 generate_spmpc_acados.py --model b0_direct_omega_legacy
python3 generate_spmpc_acados.py --model slosh_direct_omega
```

不生成 C code 时可先检查 CasADi 装配：

```bash
python3 generate_spmpc_acados.py --check --model b0
python3 generate_spmpc_acados.py --check --model slosh
python3 generate_spmpc_acados.py --check --model phase_rejoin
```

### 9.2 编译

```bash
cd /home/a/scout_ws
source /opt/ros/noetic/setup.bash
source /home/a/acados_venv/bin/activate
export ACADOS_SOURCE_DIR=/home/a/acados
export LD_LIBRARY_PATH=/home/a/acados/lib:${LD_LIBRARY_PATH:-}

catkin_make --force-cmake \
  -DCATKIN_WHITELIST_PACKAGES="spmpc_local_planner" \
  --pkg spmpc_local_planner
source devel/setup.bash
```

构建日志应明确显示主 B0/slosh solver 和 dedicated short-horizon Phase-Rejoin solver。缺少普通 acados 时后端会编译为 stub；缺少专用 Phase solver 时 `enforce` 会拒绝初始化。

---

## 10. 启动入口

### 10.1 旧 continuous MPCC / Phase off

这是当前默认、可回归的运行入口：

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh \
  solver_backend:=continuous_mpcc_acados \
  phase_rejoin_mode:=off
```

### 10.2 Phase-Rejoin monitor

`monitor` 只用于加载 development/formal-like artifact、核对合同、相位和诊断；它不改变 phase OCP 或命令：

```bash
PHASE_ARTIFACT_PATH=/abs/path/development_artifact.csv
PHASE_CONTRACT_ID=development_route_v2

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh \
  solver_backend:=continuous_mpcc_acados \
  reference_target_frame:=map \
  phase_rejoin_mode:=monitor \
  phase_rejoin_artifact_path:="${PHASE_ARTIFACT_PATH}" \
  phase_rejoin_required_contract_id:="${PHASE_CONTRACT_ID}" \
  phase_rejoin_required_frame_id:=map
```

### 10.3 `enforce` 接口示例：当前禁止实物使用

下面只记录代码接口，供 proxy 仿真和后续正式链开发。**当前不要在 Scout 实物上执行。**

```bash
PHASE_FORMAL_ARTIFACT_PATH=/abs/path/formal_artifact.csv
PHASE_FORMAL_CONTRACT_ID=formal_route_v2

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B_slosh \
  solver_backend:=continuous_mpcc_acados \
  reference_target_frame:=map \
  shared_linear_accel_limit_enable:=false \
  shared_angular_limit_enable:=false \
  delay_phase_mode:=fixed_closed_loop \
  delay_phase_require_complete_history:=true \
  phase_rejoin_mode:=enforce \
  phase_rejoin_liquid_horizon_steps:=3 \
  phase_rejoin_artifact_path:="${PHASE_FORMAL_ARTIFACT_PATH}" \
  phase_rejoin_allow_development_artifact_in_enforce:=false \
  phase_rejoin_required_contract_id:="${PHASE_FORMAL_CONTRACT_ID}" \
  phase_rejoin_required_frame_id:=map
```

development proxy 仿真曾显式使用 `phase_rejoin_allow_development_artifact_in_enforce:=true` 做分支测试；该 override 不能复制到实物或 formal 数据采集。

### 10.4 RouteB 与点到点 smoke

```bash
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_direct_omega_legacy \
  alpha_max:=3.5

roslaunch spmpc_local_planner spmpc_point_to_point.launch \
  planner_variant:=B0 \
  solver_backend:=continuous_mpcc_acados
```

### 10.5 实物脚本边界

`scripts/run_continuous_real.sh` 是旧 continuous MPCC 的历史/开发入口，不是 Phase-Rejoin formal runner。

当前 `run_spmpc_real_fixed_path_trial.sh`、`record_spmpc_full_rgb_bag.sh` 和旧 postflight 也尚未完整表达 C2/C3/C4、phase artifact、双通道模型、`/spmpc/debug/phase_rejoin` 和新 freeze contract。正式实物前必须新增或升级专用 runner/recorder/postflight，不能只靠手工补几个 roslaunch 参数。

---

## 11. 诊断话题

### 11.1 基本 solver 与状态

```text
/spmpc/status
/spmpc/solver_backend
/spmpc/controller_variant
/spmpc/experiment_mode
/spmpc/local_trajectory
/spmpc/solver_time_ms
/spmpc/cost_breakdown
/spmpc/debug/effective_config
/spmpc/debug/progress_s
/spmpc/debug/raw_state
/spmpc/debug/predicted_state
/spmpc/debug/solver_input_state
```

### 11.2 observer、时间和执行前沿

```text
/spmpc/debug/slosh_observer_odom
/spmpc/debug/slosh_observer_imu
/spmpc/debug/slosh_observer_selection
/spmpc/debug/delay_phase
/spmpc/debug/odom_timing
/spmpc/debug/execution_state
/spmpc/debug/execution_alignment_status
/spmpc/debug/delay_compensation
/spmpc/debug/cmd_odom_alignment
```

### 11.3 Phase-Rejoin 与最终命令

```text
/spmpc/debug/phase_rejoin
/spmpc/debug/control_cycle_audit
/spmpc/debug/command_intervention
/spmpc/debug/cmd_vel_output
/spmpc/debug/cmd_vel_output_status
```

`PhaseRejoinDebug` 用同一 `cycle_id` 记录：

- `mode/evidence_level`；
- artifact 和 runtime contract 是否有效；
- clock/current/front/terminal index；
- candidate count、phase lead 和 `front_steps/liquid_steps`；
- current/terminal gate metric；
- solver、nominal 和 phase coordinator 输出候选；
- residual、recovery、stop、terminal release 和 command contract；
- prediction origin/epoch 和 `state_complete_for_certificate`。

`ControlCycleAudit` 记录 solver 第一拍经过 terminal、phase、limiter 和 safety 后如何变成最终 published command，并保存 source/effective timestamp。分析时必须按 `cycle_id` 和显式时间戳 join，不能按 bag 到达顺序拼接。

`PhaseRejoinDebug.output_cmd_*` 仍可能被后续 terminal-spin、tracking safety、limiter 或 command contract 改写，不是最终 `u_pub`。最终发布值应读取同一 `cycle_id` 的 `ControlCycleAudit.published_cmd_*`，并与 `/spmpc/debug/cmd_vel_output` 和 `/cmd_vel` 交叉核对。

### 11.4 预测时域与 replay

```text
/spmpc/debug/predicted_horizon
/spmpc/debug/pre_solve_snapshot
```

不能再把这两个话题的尺寸固定写成“永远 61/60”：

| 运行路径 | 状态节点 | 控制节点 | generated `nx` | snapshot 统一状态宽度 | 控制宽度 |
| --- | ---: | ---: | ---: | ---: | ---: |
| B0 long horizon | 61 | 60 | 6 | 10 | 3 |
| slosh long horizon off/monitor | 61 | 60 | 10 | 10 | 3 |
| Phase-Rejoin enforce | 4 | 3 | 10 | 10 | 3 |

B0 的 replay message 仍使用统一 10 列状态布局，四个液体列填零；这不把 B0 generated OCP 变成 10 维。

当前 B0 参数宽度为 23；当前 slosh/Phase 参数布局为 55。generated 模型发生变化后，应以消息中的 `horizon_steps/state_width/control_width/parameter_width` 和 codegen 静态合同为准，不要在分析脚本中继续硬编码旧的 32 参数口径。

`PreSolveSnapshot.primal_guess_only=true` 表示仍未记录对偶变量和 acados 内部 SQP memory。消息存在不等于可以逐位重放；正式 replay 还需验证 status、第一拍和 raw command 在冻结容差内一致。

### 11.5 液体与 RGB

```text
/spmpc/debug/slosh_state
/spmpc/slosh_height
/spmpc/slosh_horizon_summary
/spmpc/debug/slosh_cost_monitor
/liquid/measurement
```

不要复用 `/mpc/cost_breakdown`，以免污染 `scout_local_planner` 的历史分析链。

---

## 12. 录包、测试与当前证据

### 12.1 Phase-Rejoin 最小证据集

Phase-Rejoin development bag 至少需要：

```text
/cmd_vel
/odom
/imu/data 或 processed observer diagnostics
/scout/global_path_fixed
/tf
/spmpc/status
/spmpc/debug/effective_config
/spmpc/debug/raw_state
/spmpc/debug/predicted_state
/spmpc/debug/solver_input_state
/spmpc/debug/execution_state
/spmpc/debug/slosh_observer_odom
/spmpc/debug/slosh_observer_imu
/spmpc/debug/slosh_observer_selection
/spmpc/debug/control_cycle_audit
/spmpc/debug/phase_rejoin
/spmpc/debug/predicted_horizon
/spmpc/debug/pre_solve_snapshot
/spmpc/debug/command_intervention
/liquid/measurement 或冻结协议规定的独立 RGB 证据
```

当前 `record_spmpc_full_rgb_bag.sh` 已包含大量 observer、execution 和 control-audit 话题，但尚未强制记录 `/spmpc/debug/phase_rejoin`，因此不能称为 Phase-Rejoin formal recorder。

### 12.2 自动测试入口

C++ 回归：

```bash
cd /home/a/scout_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=spmpc_local_planner -j1
catkin_make run_tests_spmpc_local_planner -j1
catkin_test_results build
```

Phase-Rejoin 重点测试文件：

```text
test/test_command_history_buffer.cpp
test/test_execution_state_predictor.cpp
test/test_nominal_sequence_artifact.cpp
test/test_phase_rejoin.cpp
test/test_replay_diagnostics.cpp
test/test_terminal_controller.cpp
```

Python development 工具测试：

```bash
python3 -m unittest \
  src/scout_apps/control/spmpc_local_planner/test/python/test_phase_rejoin_development_artifact.py \
  src/scout_apps/control/spmpc_local_planner/test/python/test_phase_rejoin_development_nominal.py \
  src/scout_apps/control/spmpc_local_planner/test/python/test_analyze_phase_rejoin_paired_bags.py
```

2026-08-20 的 proxy 仿真和分支测试只能说明：

```text
artifact/clock/candidate/short OCP/gate/supervisor 接线可运行
off/monitor/enforce 的开发语义可检查
```

不能据此说明：

```text
正式 B0 因果前沿已经闭合
held-out gate 已经安全
真实 Scout 液体晃动已经改善
实物 enforce 已经放行
```

---

## 13. 后续维护优先级

后续开发应按以下顺序推进，不要直接从 development `enforce` 跳到实物：

1. 用 Nokov、published `/cmd_vel`、odom 和 IMU 完成 Scout 线/角双通道执行模型辨识与 held-out 冻结；
2. 将 history-only 前沿改成 delay-augmented OCP 或严格等价 bridge，闭合快通道新决策依赖和 epoch/index；
3. 完成 G0 总 lead、首拍灵敏度和 processed-IMU 主频幅相验证；
4. 实现正式 `OfflineSloshOCP`，输出完整尾段和可审计 artifact/hash；
5. 构建执行状态兼容集、held-out 9 维 gate，并报告 false-accept；
6. 实现 C0–C4/A0–A3 所需的独立 runtime 条件和确定性失败语义；
7. 完成 Phase-Rejoin 专用 runner、recorder、postflight 和 formal freeze；
8. 先做 proxy/仿真和小规模 development 配对，通过后再申请实物 `enforce`。

长期工程原则：

```text
旧 solver 和旧 variant 保持可回归
新功能通过独立 mode/config 启用
算法核心尽量与 ROS 解耦
所有时间链使用 source timestamp
只把最终 u_pub 写回执行历史
generated solver 只由 tools/codegen/acados 重新生成
RGB 作为独立物理评价，不进入控制闭环
未通过放行门时 fail closed，并保留 NO-GO 证据
```
