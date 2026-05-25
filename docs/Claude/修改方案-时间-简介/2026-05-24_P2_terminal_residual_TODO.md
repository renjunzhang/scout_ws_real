# 2026-05-24 P2: Terminal Residual 治理 TODO List

> 对应 `2026-05-24_SloshPriorityMPC物理保真度与残振治理方案.md` 第 P2 节。
> 治理 20260524 实验报告局限 6.2（terminal approach 1s 仍有 jerk 脉冲污染液体）。
> 拆成 **A（轻量 ramp，先做）** 与 **B（free-decay 自适应，论文卖点）** 两阶段，
> A 通过后再决定是否上 B。所有改动默认 disable，向后兼容。

---

## 全局约束（CLAUDE.md 外科手术式修改）

- 不动 launch 33 arg / 60 publisher / RGB 视觉流程接口
- 不动 terminal_capture_stop_distance / terminal_slowdown_distance 等 gate
- 不引入新决策变量（不破 OSQP）
- 任何新 param 默认值必须等价旧行为（disable）
- 每个 phase 提交前必须做 sim S 曲线 byte-equal 回归（开关关闭）

---

## Phase A —— terminal_factor 沿 horizon 末多步线性 ramp（轻量）

**目的**：现 `terminal_factor_slosh_eta` 只对 k=N-1 那一步加权，等价于一个"末端尖峰"。
改成 k ∈ [N-K_term, N-1] 线性 ramp 从 1.0 升到 terminal_factor，让 MPC 把"末端别晃"
的引导意图分摊到末端几步，避免最后一步突然加重导致中间步剩余晃动估计有偏。

**预期效果**：terminal phase 内 cmd_v 衰减更平、ax 脉冲下降；模型 eta 末段更低。

### TODO

```text
[A.1] params
  - mpc_params.yaml / mpc_params_sim.yaml 新增:
      mpc:
        terminal_slosh_ramp_steps: 0       # 0 = 等价旧行为（仅 k=N-1 加权）
                                            # >0 时 k ∈ [N-ramp_steps, N-1] 线性 ramp
  - slosh_experiment.launch / _sim.launch 新增 arg:
      <arg name="mpc_terminal_slosh_ramp_steps" default="0"/>
      <param name="mpc/terminal_slosh_ramp_steps" value="$(arg mpc_terminal_slosh_ramp_steps)"/>

[A.2] types.h
  - MPCParams 新增 int terminal_slosh_ramp_steps = 0;
  - 不破现有 MPCParams 字段顺序（追加到末尾，避免 ABI 影响）

[A.3] cost_function.cpp (StateTrackingCost::getQuadraticCost + ::evaluate)
  - 把现有的 "if (k == N-1) factor = terminal_factor_slosh_eta" 改成:
      int rs = std::max(0, params_.terminal_slosh_ramp_steps);
      if (rs == 0 && k == N - 1) {
          // 旧行为分支
          factor = terminal_factor_slosh_eta;
      } else if (rs > 0 && k >= N - rs) {
          int j = k - (N - rs);          // j ∈ [0, rs-1]
          double alpha = (j + 1.0) / rs;
          factor = 1.0 + alpha * (terminal_factor_slosh_eta - 1.0);
      } else {
          factor = 1.0;
      }
  - 同时改 J_slosh_eta_dot 的 terminal_factor_slosh_eta_dot ramp（同样公式）

[A.4] local_planner_ros.cpp:computeCostBreakdown
  - 双份实现警告: 这里也得用同一个 ramp 公式
  - 把现行 "if (k == N-1) terminal_factor(...)" 改成与 A.3 完全一致的逻辑
  - 用一个 inline helper 函数避免重复:
      static inline double terminal_factor(int k, int N, int ramp_steps, double base);

[A.5] 单元 sanity check（不写 GoogleTest，shell 验证即可）
  - 启动 sim S 曲线 smoke，rosbag record /mpc/cost_breakdown:
      terminal_slosh_ramp_steps:=0  (default) → cost_breakdown 与 git HEAD~1 byte-equal
      terminal_slosh_ramp_steps:=5            → J_slosh_eta 末段变化可见

[A.6] sim 验证
  - cd /home/a/scout_ws
  - 用 run_s_curve_smoke_test.sh + Q_SLOSH=5 跑两次（ramp=0 vs ramp=5）
  - analyze_terminal_approach_1s.py + analyze_ferrari_indices.py（如果有 sim RGB CSV，
    没有 RGB 时只看 model 侧 slosh/* 趋势）
  - 期望：ramp=5 时 horizon 末段 model eta 平均更低；前段无差异

[A.7] 实物 F 配置对照（用 d200 同款命令，仅新增 mpc_terminal_slosh_ramp_steps:=5）
  - 录 1 包 P2_s_curve_F_d200_rampA_run01
  - analyze_terminal_approach_1s.py + analyze_ferrari_indices.py
  - 验收：
      * REACHED = 1, GOAL_PASSED = 0（与 d200 baseline 一致）
      * terminal_approach_1s 报告 ax_pulse / jerk_pulse 不增加（最好下降）
      * RGB peak 不显著上升（>5% 视为退步）
  - 任一项不过 → 回退 A，分析原因

[A.8] commit
  - 单文件粒度 commit:
      "P2-A: 加入 terminal_factor ramp 参数 + cost 公式 + breakdown 同步"
  - 注意保留 byte-equal 默认行为说明在 commit message
```

---

## Phase B —— horizon 末 free-decay 预测 + 自适应 terminal_factor（论文卖点）

**目的**：Ferrari 论文式 19f 强调"η ≤ 0.2·η_lim, t > t_end"，即运动结束后残振也要约束。
我们 online QP 加不了硬约束，但可以在 MPC solve 后 **post-process**：
取 horizon 末状态 (η_N, η̇_N)，假设此后 a_x = a_y = 0 free decay，
rollout T_decay 秒（如 1.5s）算 residual peak；若 peak > 阈值，
**下一周期动态放大 terminal_factor_runtime**。

不引入新决策变量 → OSQP 形式不破。

**预期效果**：MPC 自动惩罚"把高残振状态留到 horizon 尾部"的解 → 终点停车后 RGB 振幅更小。

### TODO

```text
[B.1] params
  - mpc_params.yaml / mpc_params_sim.yaml 新增:
      mpc:
        terminal_decay_predict_enable: false   # 默认关闭
        terminal_decay_horizon_s: 1.5          # free-decay rollout 时长
        terminal_decay_h_lim_factor: 0.2       # Ferrari 0.2·η_lim 阈值（h_lim 仍隐式 = slosh_height_ref）
        terminal_decay_amplify_factor: 2.0     # 超阈值时 terminal_factor 临时放大倍数
        terminal_decay_amplify_max_steps: 10   # 放大状态最多保持的控制周期数（防陷死）
  - launch 加对应 5 个 arg

[B.2] 新增 SloshFreeDecay 工具
  - 位置: include/scout_local_planner/slosh_free_decay.h + src/slosh_free_decay.cpp
    （或直接挂在 slosh_integration.h 上，避免新文件）
  - 接口:
      double predictResidualPeak(
          double eta_x, double eta_x_dot,
          double eta_y, double eta_y_dot,
          double horizon_s, double dt);
    内部用 slosh_integration_.getDiscreteMatrices(...) 的 (A_d, B_d)，
    强制 u = (a_x, a_y) = 0，连续乘 A_d 直到 horizon_s 截止，
    返回路径上的 max sqrt(eta_x² + eta_y²) * h_coeff（mm）

[B.3] local_planner_ros.cpp:controlLoop
  - 在 MPC solve 之后、publishCmdVel 之前:
      if (terminal_decay_predict_enable_) {
          double residual_peak_mm =
              slosh_free_decay_.predictResidualPeak(
                  solution.x_traj.back()[ETA_X], ...);
          double h_lim_mm = mpc_params_.slosh_height_ref * 1000.0;
          if (residual_peak_mm > terminal_decay_h_lim_factor_ * h_lim_mm) {
              terminal_factor_amplify_active_steps_ = terminal_decay_amplify_max_steps_;
          }
      }
      double terminal_factor_runtime =
          (terminal_factor_amplify_active_steps_ > 0)
            ? mpc_params_.terminal_factor_slosh_eta * terminal_decay_amplify_factor_
            : mpc_params_.terminal_factor_slosh_eta;
      if (terminal_factor_amplify_active_steps_ > 0)
          terminal_factor_amplify_active_steps_--;
  - terminal_factor_runtime 通过 MPCParams runtime copy 传给下一周期的 cost_function

[B.4] 调试 publisher（一个就够）
  - 新增 ros::Publisher slosh_predicted_residual_peak_pub_;
    advertise: /slosh/predicted_residual_peak (std_msgs/Float32, mm)
  - 每周期 publish，无论 enable 与否（disable 时 publish 0.0）

[B.5] 单元 sanity check
  - SloshFreeDecay::predictResidualPeak 单输入测试:
      eta=(0,0)/eta_dot=(0,0) → 返回 0
      eta=(η0,0)/eta_dot=(0,0) → 返回值应 ≈ η0 * h_coeff（无衰减时刻峰值就是初值）
      eta=(0,0)/eta_dot=(v0,0) → 返回值应非零（有动能转化为势能）

[B.6] sim 验证
  - terminal_decay_predict_enable:=false → byte-equal 旧行为
  - terminal_decay_predict_enable:=true + sim S 曲线:
      /slosh/predicted_residual_peak 时间序列可读
      MPC 在 horizon 末 model eta 自适应被压低

[B.7] 实物 F 配置对照
  - 录 1 包 P2_s_curve_F_d200_rampB_run01:
      mpc_terminal_slosh_ramp_steps:=5 (即 A+B 联用)
      mpc_terminal_decay_predict_enable:=true
  - analyze_terminal_approach_1s + analyze_ferrari_indices + 录 1 包 RGB
  - 验收（同 A.7 + 新增）:
      * residual_peak 实际下降可见
      * terminal_factor_amplify 触发次数与 RGB peak 下降量级有相关
  - 不过 → 回退 B 保留 A

[B.8] commit
  - "P2-B: 加入 free-decay 残振预测 + 自适应 terminal_factor 自动放大"

[B.9] 论文素材整理
  - 把 A + B 的对照（baseline / A only / A+B）做成 RGB peak 与 ax_pulse 的 2x3 figure
  - 写一段 method 描述（草稿放论文方法节 §III）:
      "Online soft surrogate of Ferrari's η ≤ 0.2·η_lim post-motion constraint:
       horizon-end free-decay prediction with adaptive terminal weight."
```

---

## 阶段间检查点（强制 review）

```text
A.7 通过 → 决定是否做 B
  - 如果 A 已经让 RGB peak 再降 ≥10%，且 ax_pulse 显著下降，B 可以不做（论文用 A 即可）
  - 如果 A 收益小（<5%），上 B

B.7 通过 → 决定 paper figure 排版
  - A+B 联用如果 RGB peak 比单 A 还高，说明 free-decay 触发过频，回退 amplify_factor 到 1.5

B.7 不通过 → 回退 B，论文只写 A，B 留作 future work
```

---

## 风险与回退

| 风险 | 触发条件 | 回退 |
|---|---|---|
| ramp_steps 太大侵蚀跟踪 | A.7 中 RGB peak 上升 | terminal_slosh_ramp_steps:=3 重试 |
| free-decay rollout 长 dt 不稳 | B.5 单测发散 | 缩 dt（用 MPC 的 0.05s），horizon 不变 |
| amplify_max_steps 卡死 | B.7 中 amplify 一直 active | 看 publisher 改 max_steps=5 |
| QP solve 时间上升 | B.7 中 /mpc/solve_ms > 25ms | free-decay 内移除 sqrt，用 eta_x² + eta_y² 直接比阈值² |

任一回退执行后必须 git commit revert，不残留半成品。

---

## 工期与里程碑

```text
Phase A: 0.5-1 天代码 + 0.5 天 sim 验证 + 半天实物 = 2 天上限
  里程碑 A: 实物 F + rampA 通过 d200 PASS 标准
Phase B: 1-2 天代码 + 0.5 天 sim 验证 + 半天实物 = 3 天上限
  里程碑 B: 实物 F + rampA + decayB 通过 PASS 标准 + 论文素材整理
合计 5 天上限
建议节奏:
  第 1 天: A.1-A.4 + A.5 sanity check
  第 2 天: A.6 sim + A.7 实物 + A.8 commit + A.9 review 决定是否上 B
  第 3 天: B.1-B.4 + B.5 sanity check
  第 4 天: B.6 sim + B.7 实物 + B.8 commit
  第 5 天: B.9 论文素材整理 + figure
```

---

## 关联文件

- 改动文件：
  - `src/scout_apps/control/scout_local_planner/config/mpc_params.yaml`
  - `src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml`
  - `src/scout_apps/control/scout_local_planner/launch/slosh_experiment.launch`
  - `src/scout_apps/control/scout_local_planner/launch/slosh_experiment_sim.launch`
  - `src/scout_apps/control/scout_local_planner/include/scout_local_planner/types.h`
  - `src/scout_apps/control/scout_local_planner/src/cost_function.cpp`
  - `src/scout_apps/control/scout_local_planner/src/local_planner_ros.cpp`
  - `src/scout_apps/control/scout_local_planner/include/scout_local_planner/local_planner_ros.h`（私有成员）
  - Phase B 新增：`include/scout_local_planner/slosh_free_decay.h` + `src/slosh_free_decay.cpp`（或合并到 slosh_integration）

- 验证脚本：
  - `scripts/analysis/analyze_terminal_approach_1s.py`
  - `scripts/analysis/analyze_ferrari_indices.py`
  - `scripts/run_s_curve_smoke_test.sh`

- 参考文档：
  - `docs/重要文档/红色液体视觉验证固定流程.md` §7.1.2 / §8.6
  - `docs/重要文档/论文参考总结/Ferrari2026_方法剖析.md` §4.1 式 19f
  - `docs/重要文档/20260518_MPC终点收敛与固定路径验证方案.md` §2.1 d200 配置
