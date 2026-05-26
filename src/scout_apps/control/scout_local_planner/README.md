# scout_local_planner 技术说明

> 写作立场：严格技术审稿人兼代码审计员。代码是最高真相，文档是注释，论文是待验证的描述。
> 所有结论以 `src/` 为准，与 `docs/` 或注释不一致之处均已标注。

---

## 关键代码文件索引

快速理解当前 MPC，请优先看以下文件：

| 文件 | 职责 |
|---|---|
| `src/local_planner_ros.cpp` / `include/scout_local_planner/local_planner_ros.h` | ROS 节点主逻辑、状态机、参数读取、速度治理、slosh debug topic 发布 |
| `src/mpc_solver.cpp` / `include/scout_local_planner/mpc_solver.h` | OSQP QP 构建、求解、warm start、解提取 |
| `src/cost_function.cpp` / `include/scout_local_planner/cost_function.h` | MPC 二次代价，包括 tracking、速度、控制、slosh η/η̇、terminal slosh |
| `src/diff_drive_model.cpp` / `include/scout_local_planner/diff_drive_model.h` | 8 维增广状态的线性化动力学，含 Frenet tracking 与 slosh 激励耦合 |
| `src/constraint_manager.cpp` / `include/scout_local_planner/constraint_manager.h` | 速度、加速度、角速度、控制变化率、η 盒约束 |
| `src/path_handler.cpp` / `include/scout_local_planner/path_handler.h` | 全局路径清洗/平滑、参考点生成、速度剖面 |
| `src/slosh_integration.cpp` / `include/scout_local_planner/slosh_integration.h` | 运行时 slosh 状态传播与 `/slosh/height` 估计 |
| `config/mpc_params.yaml` | 实物默认参数 |
| `config/mpc_params_sim.yaml` | 仿真默认参数，当前 `R_domega` 已对齐实物 |
| `launch/slosh_experiment.launch` | 实物实验入口 |
| `launch/slosh_experiment_sim.launch` | 仿真实验入口 |
| `scripts/run_sim_fixed_path_bag.sh` | 固定终点/固定路径仿真录包脚本，支持当前主线 launch 可用的 `NOM / FAS_* / CUSTOM` 与 external speed profile |
| `scripts/extract_slosh_metrics.py` | 离线指标提取：height、energy、eta_dot、tracking error、odom ay、历史消融 active ratio、lag correlation |
| `scripts/template_fixed_path_generator.py` | 从当前位姿到终点生成模板路径 |
| `scripts/fixed_global_path_runner.py` | 固定路径采集/回放 |

---

## 当前状态快照（2026-04-29）

当前系统仍是**基于全局路径局部参考的 8 维增广 tracking MPC**，不是 MPCC，也不是 NMPC。核心 MPC 结构没有变：状态含 `e_l, e_c, e_theta, v, eta_x, eta_x_dot, eta_y, eta_y_dot`，控制为 `a, omega`。

当前主线保留的 anti-slosh 相关能力：

| 模块 | 当前状态 |
|---|---|
| `Q_slosh` | 保留，用于惩罚 `eta_x^2 + eta_y^2`，但历史实验显示单独调大不稳定 |
| `Q_slosh_eta_dot` | 保留，用于惩罚 `eta_dot`；历史实验显示不构成稳定主线 |
| `terminal_factor_slosh_eta / eta_dot` | 保留，用于 terminal slosh cost；历史实验显示不构成稳定主线 |
| `scripts/extract_slosh_metrics.py` | 保留并扩展，用于复盘历史 bag 和后续离线分析 |
| `scripts/run_sim_fixed_path_bag.sh` | 保留固定终点/固定路径录包流程，但已移除失败控制器入口的启动条件 |

截至 2026-04-29 的实验判断：

- MPC 内部 slosh cost、`PROFILE_*`、`OUTPUT_GUARD`、PMG 均未形成可作为主线的通用方案。
- `OUTPUT_GUARD` 只在 `P2_s_curve` 上出现“无显著减速 + 多指标下降”的局部正例，不能泛化到 `P3_mixed`。
- PMG 离线曾给出正信号，但仿真闭环暴露 `eta_dot` 明显上升，已不进入实物主线。
- 当前证据说明：**继续在 MPC 控制器输出层或 QP cost 内堆 anti-slosh 项，成功率低；下一阶段应转向轨迹/速度生成层的可验证方案。**

---

## §0 一句话定位

当前是一个**基于全局路径局部参考的8维增广跟踪MPC**，在差速机器人Frenet误差动力学上叠加了线性液体晃动模态子系统，用OSQP求解时变仿射QP，配合外层风险调度器和终点状态机。

不是MPCC，不是NMPC，不是流体观测器闭环。

---

## §1 数据流总览

```
/scout/global_path (nav_msgs/Path)
        │
        ▼
PathHandler::setGlobalPath()
  ├─ sanitizePolyline()           // 去重复点/极短段
  ├─ buildTrackingFriendlySmoothPath()  // 最多4轮迭代平滑
  │    └─ bspline → resample → sanitize → removeSinglePointSpikes
  ├─ fitGlobalSpline()            // cubic spline（曲率插值用）
  └─ updateSpeedProfile(v_plan)   // 速度剖面（geometry from smooth cache）
        │
        ▼
PathHandler::getReferencePoints(v_exec, v_plan)
  ├─ 找最近点, 取 N+1 个参考点
  ├─ 按 v_exec 裁剪 v_ref（执行层上界）
  └─ 输出 ref_points[0..N]
        │
        ▼
RiskScheduler::update(h_pred_max_prev, E_slosh_prev, d_goal,
                      a_y_imu, v_omega, imu_stamp, imu_bias_ready, v_ref)
  ├─ r_k = w_h*h_risk + w_e*e_risk + w_t*t_risk  (physical risk, frozen monitor)
  ├─ u_k = |a_y_imu - v*ω| / a_uncert_max        (IMU excitation uncertainty)
  ├─ ρ_k = w_r*r_k + w_u*u_k, s(ρ)=sigmoid, rate-limited
  └─ 输出 Q_eta_k, eta_bar_k, v_ref_eff_k
        │
        ▼
MPCSolver::solve(state, ref_points, risk_outputs)
  ├─ buildQP() — 时变仿射QP，OSQP求解
  ├─ solution extraction: v_cmd = v_0 + 0.5*a_0*dt
  └─ 输出 cmd_vel
        │
        ▼
StateMachine (IDLE→TRACKING→SETTLING→REACHED)
  └─ terminal recovery: ALIGN_TO_POINT / APPROACH_POINT / ALIGN_FINAL_YAW
     （这三段不走MPC，是显式几何控制律）
```

---

## §2 状态向量与控制向量

### 状态向量（8维）

| 索引 (StateIndex) | 符号 | 含义 | 单位 |
|---|---|---|---|
| 0 `IDX_EL` | e_l | 横向跟踪误差（法向偏差） | m |
| 1 `IDX_EC` | e_c | 横向速度（e_l导数，离散近似） | m/s |
| 2 `IDX_ETH` | e_θ | 航向误差 | rad |
| 3 `IDX_V` | v | 纵向速度 | m/s |
| 4 `IDX_ETA_X` | η_x | 纵向晃动模态广义坐标 | m（等效） |
| 5 `IDX_ETAD_X` | η̇_x | 纵向晃动模态广义速度 | m/s |
| 6 `IDX_ETA_Y` | η_y | 横向晃动模态广义坐标 | m |
| 7 `IDX_ETAD_Y` | η̇_y | 横向晃动模态广义速度 | m/s |

注意：**ω不是状态**，是直接控制输入。这意味着角速度约束是控制约束而非状态约束，Δω约束（角加速度代理）也是控制差分约束。

### 控制向量（2维）

| 索引 (ControlIndex) | 符号 | 含义 | 单位 |
|---|---|---|---|
| 0 `IDX_A` | a | 纵向加速度 | m/s² |
| 1 `IDX_OMEGA` | ω | 角速度 | rad/s |

---

## §3 QP结构

### 决策变量

```
z = [x_0, u_0, x_1, u_1, ..., x_N]
n_z = (n_x + n_u) * N + n_x = 10*N + 8
```

N为预测步数（通常20步），每步10个变量（8状态+2控制），末端多8个状态。

### 目标函数

```
J = Σ_{k=0}^{N-1} [
      Q_el * e_l_k²
    + Q_ec * e_c_k²
    + Q_eθ * e_θ_k²
    + Q_v  * (v_k - v_ref_k)²
    + Q_η  * (η_xk² + η_yk²)        // 由 RiskScheduler 调度
    + R_a  * a_k²
    + R_ω  * ω_k²
    + R_Δa * (a_k - a_{k-1})²
    + R_Δω * (ω_k - ω_{k-1})²
    ]
  + x_N^T * P_f * x_N               // 终端代价（若启用）
```

其中 Q_η = Q_eta_k 由风险调度器实时输出，不是固定常数。

### 约束

| 约束类型 | 表达式 | 说明 | 不可行风险 |
|---|---|---|---|
| 动力学等式 | x_{k+1} = A_k x_k + B_k u_k + d_k | 线性化Frenet+晃动 | 低 |
| 速度上下界 | 0 ≤ v_k ≤ v_cap | v_cap = min(v_des, v_geom) | 若v_cap<v_prev：高 |
| 加速度界 | a_min ≤ a_k ≤ a_max | 固定 | 低 |
| 角速度界 | \|ω_k\| ≤ ω_max | 固定 | 低 |
| 加速度差分 | \|Δa_k\| ≤ Δa_max | 依赖u_prev同步 | 若u_prev未更新：高 |
| 角速度差分 | \|Δω_k\| ≤ Δω_max | 同上 | 同上 |
| 晃动盒约束 | \|η_x\|,\|η_y\| ≤ η̄_k | 由RiskScheduler调度 | 中（可软化） |

**已修复问题**：MPC求解失败后，`last_control_`未及时更新，导致下一周期Δu约束相对错误基准计算，系统性不可行。现已在每次`solve()`后无论成功失败都更新`u_prev`。

### 解提取

```cpp
// 不是standard "apply first element"
// lead_time = cmd_vel_lead_time（>=0时直接用）或 0.5*dt（<0时退回midpoint）
// 实物 mpc_params.yaml: cmd_vel_lead_time=-1.0 → 0.5*dt
// 仿真 mpc_params_sim.yaml: cmd_vel_lead_time=0.15（补偿 Gazebo 速度反馈滞后）
v_cmd     = v0 + a0 * lead_time;
omega_cmd = 0.5 * (ω_0 + ω_1);    // 首两步均值（固定，不受 lead_time 影响）
```

这是可配置前瞻提取（`mpc_solver.cpp:424`），默认退回 midpoint extraction。

---

## §4 Frenet误差动力学

### 连续时间

```
ė_l = v * sin(e_θ)           ≈ v * e_θ（小角近似）
ė_c ≈ (e_l - e_l_prev) / dt  （离散近似，不是严格连续导数）
ė_θ = ω - κ(s) * v
v̇   = a
```

注意：e_c的处理是离散近似，不是真正的连续横向速度动力学。这在高曲率段有误差。

### 线性化矩阵（关键非零项）

```
A_k[IDX_EL][IDX_ETH] = v_ref * dt
A_k[IDX_ETH][IDX_V]  = -kappa_ref * dt
B_k[IDX_ETH][IDX_OMEGA] = dt
B_k[IDX_V][IDX_A]    = dt
d_k（偏置项）包含参考曲率贡献：d[IDX_ETH] += -kappa_ref * v_ref * dt
```

---

## §5 液体晃动子系统

### 模型来源

使用 `slosh_models` 库的**L模型**（线性等效摆），不是完整非线性晃动动力学。

### 连续时间ODE

```
η̈ + 2ζω_n η̇ + ω_n² η = -f_excitation
```

其中激励 f 为：
- 纵向：a（纵向加速度）
- 横向：v * ω（向心加速度）

### 离散状态矩阵（来自ZOH离散化）

```
[η_{k+1}  ]   [A_sl 2x2] [η_k  ]   [B_sl 2x2] [a_k]
[η̇_{k+1} ] = [        ] [η̇_k ] + [         ] [vω_k]
```

耦合项进入线性化的B_k（ω通过 v_ref * ω 进入横向激励），d_k包含围绕工作点的非线性余项。

### 液面高度估计

```
h = h_coeff * sqrt(η_x² + η_y²) + [R²*ω²/(4g)]
```

- `h_coeff`：从 `slosh_models` 的L模型几何推导（与容器尺寸相关）
- `use_linear_model=true` vs `false`：**只影响h_coeff的计算公式**（线性模型系数 vs 非线性近似系数，约18%差异），**不切换ODE**
- 方括号项（`parabola_term`）：旋转液面的工程附加项，**不是论文中的非线性流体动力学**，是独立开关，与ODE解耦

**`/slosh/height`不是测量值，是模型估计值**。它仅在ODE模型准确时可信，实物验证前不可直接用于论文图表。

### 晃动盒约束

```
|η_x| ≤ η̄_k,  |η_y| ≤ η̄_k
```

这是**模态坐标的代理约束**，不是直接约束液面高度h。η̄_k由RiskScheduler实时调度（越紧越保守）。

---

## §6 风险调度器

### 核心逻辑

```
// ── r_k：物理风险（来自上一周期冻结的 monitor rollout）──
h_risk = h_pred_max_prev / eta_bar_max              // 液面预测高度占比
e_risk = E_slosh_prev / (2 * eta_bar_max²)          // 模态能量占比（两轴满量程估计）
t_risk = max(0, 1 - d_goal / d_goal_thresh)         // 终点接近度（d_goal < d_goal_thresh 时激活）
r_k    = clip(w_h*h_risk + w_e*e_risk + w_t*t_risk, 0, 1)

// ── u_k：激励不确定性（独立 IMU 观测，与 r_k 解耦）──
u_k = clip(|a_y_imu - v*ω| / a_uncert_max, 0, 1)

// ── ρ_k 合成 ──
ρ_k = clip(w_r * r_k + w_u * u_k, 0, 1)
s_k = sigmoid(gamma * (ρ_k - rho_0))

// Rate-limited（每步最大变化量 = rate_limit_per_step × 满量程）
Q_eta_k   = Q_eta_min + s_k * (Q_eta_max - Q_eta_min)
eta_bar_k = eta_bar_max - s_k * delta_eta_bar
v_ref_eff = v_ref * (1 - beta * s_k)
```

### 参数（当前 `mpc_params.yaml` 值）

| 参数 | 当前值 | 含义 |
|---|---|---|
| gamma | 5.0 | sigmoid陡峭度 |
| rho_0 | 0.3 | sigmoid中点（低于此值不显著干预） |
| rate_limit_per_step | 0.05 | 每步最大变化量（5%/步） |
| beta | 0.3 | 速度压制上限（最多折减30%） |
| a_uncert_max | 0.5 | u_k归一化分母 (m/s²) |
| w_r / w_u | 0.7 / 0.3 | r_k 与 u_k 合成权重 |

### fallback 触发条件（任一触发即切固定保守参数）

```
1. IMU 最近消息距今 > imu_timeout_s=0.1s
2. imu_bias_ready=false（bias 估计窗口未完成）
3. u_k > u_threshold_high=0.8，连续 u_high_count_trigger=10 步
```

fallback 期间输出固定值：`Q_eta_fix=5.0, eta_bar_fix=0.04`，`v_ref_eff=v_ref`（不折减）。

`fallback_active=true` 是内部状态标志，不是 MPC 求解失败的 fallback，两者命名有歧义需注意。

---

## §7 速度剖面

### 三约束公式

```
v_geom = min(
    sqrt(a_lat_max / |κ|),          // 横向加速度约束（path_handler.cpp:1398）
    omega_max / |κ|,                 // 角速度约束（path_handler.cpp:1404）
    sqrt(alpha_max / |dκ/ds|)       // 角加速度代理约束（path_handler.cpp:1413）
)
```

注意：第三项是 **sqrt**（平方根），不是 cbrt。推导：`v² * |dκ/ds| ≤ alpha_max` → `v ≤ sqrt(alpha_max / |dκ/ds|)`。

### 几何来源（P1后）

- 曲率来自 `global_points_map_smooth_` 的**离散有限差分**，不是cubic spline的解析导数
- cubic spline导数对dκ/ds有约4倍放大效应（实测验证），已放弃
- 采样粗化：先重采样到 `profile_geom_spacing = max(0.10, 2×speed_profile_ds)`，再计算几何

### `speed_profile_omega_max` 桥接行为

`mpc_params.yaml` 未显式配置 `path_handler/speed_profile_omega_max` 时，代码自动桥接（`local_planner_ros.cpp:432`）：

```cpp
if (path_params_.speed_profile_omega_max <= 1e-6)
    path_params_.speed_profile_omega_max = vehicle_params_.omega_max;
```

实物 `vehicle/omega_max=1.0`，这意味着 κ≥1.0 的弯道速度上限由 `omega_max/κ` 主导（比 `sqrt(max_lat_accel/κ)` 更紧）。若要解耦，需在 yaml 里显式写 `speed_profile_omega_max: <值>`。`speed_profile_alpha_max` 同理桥接到 `vehicle/alpha_max`。

同样的桥接约束在 TRACKING 主循环里也有一份（`tracking_curvature_speed_cap`，`local_planner_ros.cpp:874`），形成双重几何限速。

### v_plan / v_exec解耦（P1）

```
getReferencePoints(v_exec, v_plan, ref_points)
├─ 只有 |v_plan - v_stored| > 1e-3 时才重算速度剖面（用v_plan）
└─ v_ref 最终被 v_exec 作为上界裁剪（执行层）
```

P1前：执行层限速 v_exec 会反向触发 updateSpeedProfile，使整条速度剖面按偏低速度重算，系统性压速。

### 诊断日志格式

```
[SpeedProfile] n=... ds=... kappa_max=... dkappa_max=...
  v_geom_min=... v_profile_min=... v_des=...
  dom(nom/lat/omega/alpha)=...   // 主导限速约束计数
  pass(accel/decel)=...          // 前向/后向动态规划通过数
```

---

## §8 终点状态机

```
IDLE
  │ goal received
  ▼
TRACKING
  │ dist_to_goal < enter_distance
  ▼
SETTLING  ← 这里仍走MPC，但参数变化：
  │           Q_tracking=0, Q_v=high, Q_eta=max, v_des=0
  │ |η| < eta_tol AND |η̇| < eta_dot_tol  [OR timeout]
  ▼
REACHED
```

**SETTLING是MPC，不是停车控制**。MPC在SETTLING阶段仍然求解，只是权重大幅调整，目标是让机器人减速停止同时等待晃动衰减。

### Terminal Recovery（不走MPC）

```
ALIGN_TO_POINT    → 原地旋转对准目标方向（纯几何）
APPROACH_POINT    → 直线趋近目标点（speed = terminal_v_max）
ALIGN_FINAL_YAW   → 原地旋转到目标朝向（纯几何）
```

这三段是**显式几何控制律**，完全绕开MPC。触发条件：dist_to_goal < terminal_enter_distance。

---

## §9 可行性保护机制

```
fail_streak >= 3: v_des_cmd → 0.5 m/s（降速，mild cap）
fail_streak >= 6: v_des_cmd → 0.3 m/s（进一步降速，strong cap）
5次连续成功:      恢复原始 v_des
```

失败时 `cmd_vel` 保持上一周期值（不发零速），避免急停。

### TRACKING 重入速度爬坡

每次进入 TRACKING（新路径、IDLE→TRACKING）时，触发 `tracking_reentry_ramp_steps=10` 步爬坡：

```
v_des_cmd = min(v_des_cmd, tracking_reentry_v_cap_ + α*(v_des_raw - tracking_reentry_v_cap_))
```

`tracking_reentry_v_cap_=0.6 m/s`，α 从 0 线性增到 1，10 步后恢复正常（共 0.5s @ 20Hz）。  
若弯道入口正好在路径起点，爬坡 + 曲率限速会叠加，前 0.5s 明显慢。

### 已修复的典型失败链

1. **路径几何病态** → fitLocalSpline发散 → 参考路径max_kappa=18，OSQP不可行
   - 修复：局部样条阈值放宽（kappa>30, dkappa>500），引入buildTrackingFriendlySmoothPath全局迭代平滑
2. **omega_max约束缺失** → 曲率段速度过高 → 角速度硬件饱和 → 实际轨迹与预测分歧
   - 修复：速度剖面加入 `v_cap_omega = omega_max / kappa`
3. **u_prev未同步** → Δu约束基准错误 → 系统性不可行
   - 修复：每次solve()后强制更新u_prev

---

## §10 防晃抑制：三层结构

### 第零层：速度剖面几何限速（静态，路径加载时计算）

- `v_geom = min(sqrt(a_lat/κ), omega_max/κ, sqrt(alpha_max/dκ))`
- 对整条路径的 v(s) 预先压速，非实时
- **与后续各层相互独立**，不能互相替代

### 第一层：外层参考整形（实时，每控制周期）

- TRACKING 曲率预览二次压速（`tracking_curvature_speed_cap`，与第零层使用相同公式但实时 preview）
- v_des rate limit：对执行层参考速度做加减速限幅，降低纵向 ax 脉冲

### 第二层：MPC内层

- Q_η惩罚η_x²+η_y²（软代价，让MPC主动选择减少晃动的控制）
- η̄约束（硬约束，盒约束，可能引发不可行）
- SETTLING阶段：Q_eta=max，强制晃动衰减后才允许状态转移

### 第三层：状态传播

- 晃动模态状态作为MPC状态，全预测窗口内传播
- 预测窗口内激励（a, v·ω）与控制输入耦合

### 当前主要矛盾

MPC的晃动抑制通过**软代价**实现（Q_η项），而不是刚性约束液面高度。这意味着：
- 若Q_η过小：MPC忽略晃动，激励自由
- 若Q_η过大：与跟踪代价冲突，MPC可能优先选择不精确跟踪以减小晃动

η̄硬约束收紧时若超过当前η，会直接导致QP不可行，需要与Q_η配合使用。

---

## §11 论文写作十条结论

**1. 系统本质是tracking MPC，不是MPCC**
- tracking MPC：有固定时间参数化参考，在时间域上跟踪
- MPCC（Model Predictive Contouring Control）：进度变量σ是决策变量之一，优化进度速度
- 本系统的参考点是按弧长固定采样的，σ不是优化变量

**2. 控制输入是(a, ω)，不是(v, ω)**
- ω是直接控制量，不是状态
- 这与很多文献（以v_dot/omega_dot为输入）不同，需要在论文中明确声明

**3. `/slosh/height`是模型估计，不是传感器测量**
- h = h_coeff·√(η²) + [parabola_term]，完全依赖ODE模型
- 若要论文中引用液面高度曲线，必须加注"model-estimated height"

**4. slosh软代价惩罚的是η，不是h**
- 目标函数中是 Q_η·(η_x²+η_y²)，不是 Q_h·h²
- 这在因果上是正确的（η是状态，h是派生量），但论文中需要说清楚

**5. `use_linear_model`不切换ODE**
- 只影响h_coeff的数值（约18%差异）
- ODE结构（ω_n, ζ, 激励形式）与此开关无关

**6. `parabola_term`不是论文中的非线性流体动力学**
- 是旋转液面的工程additive估计，独立开关
- 不与ODE耦合，不影响MPC内部的晃动传播

**7. terminal recovery完全不走MPC**
- ALIGN_TO_POINT / APPROACH_POINT / ALIGN_FINAL_YAW是显式几何控制
- 不受任何MPC参数影响，由terminal_enter_distance触发
- 论文中不能写"MPC handles terminal positioning"

**8. SETTLING仍是MPC，只是权重变化**
- Q_tracking→0，Q_v→high，Q_eta→max，v_des→0
- 不是停车，是带晃动约束的软制动
- 可以写"a settling phase of MPC with modified cost weights"

**9. 速度剖面是规划层计算，与MPC执行层解耦**
- updateSpeedProfile对应v_plan（nominal），不随每步v_exec重算
- P1修复后，执行层降速不再反向重写全局速度剖面

**10. 晃动约束是模态坐标代理约束，不是液面高度约束**
- |η_x|≤η̄，|η_y|≤η̄，不是h≤h_max
- 从η到h的映射是非线性的（开方加parabola），所以直接约束η比约束h在QP中更干净

---

## §12 改代码十条结论

**1. v_plan/v_exec必须分开传入getReferencePoints**
- 速度剖面更新只用v_plan（名义值），执行层用v_exec裁剪v_ref
- 混用会导致执行层降速反向重写整条速度剖面

**2. u_prev必须在每次solve()后立即同步**
- 无论求解成功还是失败
- 失败时用fallback值（如当前cmd_vel）更新
- 否则Δu约束基准偏离，下一步大概率不可行

**3. 速度剖面几何只用smooth cache的离散曲率**
- 不要用global_spline_.evaluateKappa()的解析导数
- cubic spline对dκ/ds有约4倍放大，会系统性压速

**4. omega_max约束必须同时体现在速度剖面**
- v_cap_omega = omega_max / |κ|
- 只有v_cap_kappa（从a_lat）而没有v_cap_omega，曲率段会硬件饱和

**5. fitLocalSpline阈值与buildTrackingFriendlySmoothPath要配套**
- smooth cache目标kappa≤3.8，dkappa≤45.0
- fitLocalSpline跳过阈值不能比smooth cache目标低太多（否则会频繁重做样条）
- 当前：skip if kappa>30||dkappa>500（已对齐）

**6. 局部B-spline平滑的条件应检查global_smooth_valid_**
- smooth cache有效时，局部窗口不需要重复做B-spline
- 否则在每次getReferencePoints时都会重做，计算冗余且可能引入不一致

**7. eta_bar和Q_eta的联动**
- eta_bar收紧时，若eta已接近或超过新bar：QP立即不可行
- 需要rate_limit保证bar的变化速率远小于eta的衰减速率

**8. SETTLING的timeout不能太短**
- 晃动衰减时间与ω_n/ζ相关，通常需要1-3个自然周期
- 过短的timeout会导致SETTLING→REACHED时液面仍在晃

**9. feasibility guard的阈值与应用场景**
- fail_streak≥3才降速，不是fail_streak≥1
- 这是有意的：单次fail可能是瞬态（路径曲率跳变），连续fail才是系统性问题

**10. success_ratio的统计口径**
- 全包统计的success_ratio会被SETTLING/REACHED/IDLE污染（这些阶段MPC有时故意失败）
- 论文中应只引用TRACKING阶段的success_ratio

---

## §13 文档与代码的已知不一致

| 不一致点 | 文档描述 | 代码实际 |
|---|---|---|
| 液面高度约束 | "hard constraint on h" | 软代价惩罚η，η̄是模态盒约束 |
| use_linear_model | "切换线性/非线性动力学" | 只改h_coeff，ODE不变 |
| parabola_term | "非线性流体项" | 工程附加修正，独立开关 |
| terminal recovery | "MPC终点控制" | 显式几何控制，完全绕开MPC |
| success_ratio | 全包统计 | 应按mpc_status分段 |
| 速度剖面几何来源 | 未明确说明 | smooth cache离散曲率（P1后） |

---

## §14 缺失信息

以下信息在代码中找不到具体数值，论文写作时需要补充：

1. **ω_n、ζ的实测值**：代码中从参数服务器读取，当前仿真和实物值未固定在代码里
2. **容器几何参数**：R（等效摆长）、液位高度等，影响h_coeff计算
3. **仿真到实物的gap**：当前仿真bag收敛（test7基线），实物验证尚未进行
4. **侧视相机标定**：RealSense的液面测量结果与模型估计h的对比，尚未完成

---

## §15 当前核心问题（2026-04-27）

当前失败更像是**控制结构问题为主，代价函数设置问题为辅**。

已经排除或弱化的方向：

| 方向 | 当前结论 |
|---|---|
| 单独增大 `Q_slosh` | 不稳定，容易改变相位而非稳定降晃 |
| 增加 `Q_slosh_eta_dot` | P1 失败，`eta_dot_rms` 可能反升 |
| terminal slosh cost | P2 不稳定，改善难以归因，激励不可比 |
| preview κ / dκ 前馈限速 | P3B/P3C 失败；实测 odom κ 与 reference κ 差异较大，前馈偏乐观 |
| 增大 `R_da/R_domega` | P4 失败；tracking 可改善，但实际 `odom_ay_abs_p95 / odom_kappa_abs_p95` 反向恶化 |

关键观察：

- `/slosh/height` 是模型估计，不是真值测量；实物红色液体视觉和 `/slosh/height` 曾给出不同判断。
- 当前 MPC 中 slosh 是状态和软代价，但车辆未来激励与液体相位之间没有被稳定约束。
- 只压某个单点指标（`eta`、`eta_dot`、`ay_p95`、`cmd_dwz`）可能把激励搬到更差的相位或更长持续时间。
- `odom_ay_abs_p95` 降低不等于 `h_rms / modal_energy` 降低，仍需 RGB 真值主指标判断。

当前推荐的下一步方向：

1. 先把诊断指标补齐并固定口径：`h_rms / h_p95 / h_peak / modal_energy_norm_rms / eta_dot_rms / odom_ay_abs_p95 / odom_kappa_abs_p95 / track_dist_p95 / task_time / solve_success_ratio`。
2. 若继续改控制结构，应从“预测式低激励速度规划”或“v 与 omega 联合限幅”入手，而不是继续只调 `Q_slosh*`、`R_domega` 或单个 `ay_threshold`。
3. 在没有稳定仿真证据前，不建议把当前 anti-slosh 改动推到实物主实验。

---

## 附录：14条常见论文写作错误

| # | 错误写法 | 正确写法/原因 |
|---|---|---|
| 1 | "We use MPCC to track the path" | tracking MPC with time-parameterized reference; σ is not a decision variable |
| 2 | "hard constraint on liquid height h ≤ h_max" | soft penalty on modal coordinates η; box constraint on |η| ≤ η̄ |
| 3 | "The slosh model captures nonlinear fluid dynamics" | linear MSD modal approximation (L-model); excitation is kinematic, not full CFD |
| 4 | "use_linear_model switches between linear and nonlinear dynamics" | only changes h_coeff coefficient; ODE structure is invariant |
| 5 | "parabola_term implements the nonlinear term from [ref]" | engineering additive rotation correction; independent of ODE |
| 6 | "control inputs are (v, ω)" | control inputs are (a, ω); v is a state, not a direct input |
| 7 | "/slosh/height is measured by sensor" | model-estimated: h = h_coeff·√(η²) + [parabola_term] |
| 8 | "MPC handles terminal positioning" | terminal recovery (ALIGN/APPROACH/ALIGN_FINAL_YAW) is explicit geometric control, outside MPC |
| 9 | "SETTLING phase stops the robot" | SETTLING is MPC with modified weights (Q_track=0, Q_v=high, Q_eta=max); not a stop command |
| 10 | "overall success_ratio = X%" | must report per-phase: TRACKING/SETTLING separately; overall rate is polluted by non-tracking phases |
| 11 | "the speed profile is updated every cycle" | speed profile updates only when |v_plan - v_stored| > 1e-3; v_exec clipping does not retrigger it |
| 12 | "curvature from the global spline" | curvature from discrete finite-difference on smooth-cache path; cubic spline has ~4× dκ amplification |
| 13 | "solution is the first element of the optimal sequence" | midpoint extraction: v_cmd = v_0 + 0.5·a_0·Δt; ω_cmd = 0.5·(ω_0+ω_1) |
| 14 | "the risk scheduler provides a hard safety guarantee" | RiskScheduler is a heuristic scheduler based on sigmoid; no formal guarantee; fallback_active is a flag, not a safety proof |

---

*最后更新：2026-04-27。以 `src/` 代码为准，如有冲突以代码为准。*
