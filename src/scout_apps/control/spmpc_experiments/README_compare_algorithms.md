# 对比算法文件分类索引

本文件是 SPMPC 对比实验的“先看这里”入口，用来回答：每个对比算法的配置、实现、运行入口、指标和表格角色分别放在哪里。

> 当前采取低风险组织方式：Hamaguchi/Lim 已从老控制器目录拆到独立 `scout_profile_baselines` 包；旧 `rosrun scout_local_planner generate_*.py` 入口仅作为兼容 wrapper 保留。LT-DWA 上游仍作为 source-only vendored code 放在 `third_party/LT_DWA`，Scout 自有可运行 adapter 单独放在 `src/scout_apps/control/lt_dwa_adapter/`，不把上游包放入 catkin 主构建。

## 1. 总体分层

```text
spmpc_experiments/
  config/benchmark/          fairness policy / common limits / freshness / table rules
  config/baselines/          TEB / DWA / mpc_local_planner / LT-DWA runtime configs
  config/profile_baselines/  Hamaguchi / Lim method configs
  launch/sim/                SPMPC / TEB / DWA / mpc_local_planner / LT-DWA sim launch wrappers
  scripts/                   suites / phase-0 gates / contract validators / evidence conversion / metrics extraction

scout_profile_baselines/
  scripts/                   stable wrappers + Hamaguchi / Lim method folders + common helpers

scout_local_planner/
  scripts/analysis/          legacy TOPPRA/Ruckig/Biagiotti helpers + Hamaguchi/Lim wrapper entrypoints
  launch/slosh_experiment_sim.launch
                              common tracker used by profile baselines

lt_dwa_adapter/
                              Scout-owned LT-DWA-style adapter, default shadow-only

third_party/LT_DWA/
                              upstream LT-DWA source-only reference, never symlinked into src
```

运行数据和 strict fresh 外层生命周期在隔离仿真根目录：

```text
/data/a/scout_sim_replacement/scripts/   strict fresh wrappers
/data/a/scout_sim_replacement/results/   metrics, manifest, comparison tables
/data/a/scout_sim_replacement/bags/      rosbag files
/data/a/scout_sim_replacement/logs/      sim/localization logs
```

Obsidian 只作为报告/镜像层，不作为 canonical raw source：

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/
```

## 2. 按算法找文件

| 类别                 | 方法                                             | 配置文件                                                                                   | 核心实现                                                                             | 运行入口                                                                                                     | 指标/表格                                                       | 表格角色                                 | slosh monitor 反馈                                 |
| -------------------- | ------------------------------------------------ | ------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------- | ---------------------------------------- | -------------------------------------------------- |
| online SPMPC         | `B0` / `B_slosh` / `B_smooth` / `B_ours` | SPMPC variant/config；benchmark common policy 在 `config/benchmark/`                     | `src/scout_apps/control/spmpc_local_planner/`                                      | `scripts/run_fixed_path_spmpc_suite.sh`；外层 strict wrapper 在 `/data/a/scout_sim_replacement/scripts/` | `scripts/extract_fixed_path_paper_metrics.py`                 | 主表 / 内部消融                          | 禁止作为控制输入；仅 rosbag/metrics/report         |
| legacy local planner | TEB                                              | `config/baselines/teb_local_planner_standalone_sim.yaml`                                 | `teb_local_planner` nav_core plugin + `baseline_local_planner_runner`            | `scripts/run_fixed_path_baseline_suite.sh`                                                                 | `scripts/extract_fixed_path_paper_metrics.py`                 | 同层 online baseline                     | 禁止作为控制输入；仅 rosbag/metrics/report         |
| legacy local planner | DWA                                              | `config/baselines/dwa_local_planner_standalone_sim.yaml`、`dwa_local_planner_sim.yaml` | `navigation/dwa_local_planner` nav_core plugin + `baseline_local_planner_runner` | `scripts/run_fixed_path_baseline_suite.sh`                                                                 | `scripts/extract_fixed_path_paper_metrics.py`                 | 同层 online baseline                     | 禁止作为控制输入；仅 rosbag/metrics/report         |
| fallback MPC         | `mpc_local_planner`                            | `config/baselines/mpc_local_planner_standalone_sim.yaml`                                 | `src/scout_apps/control/mpc_local_planner/`                                        | `scripts/run_fixed_path_baseline_suite.sh`，需显式 opt-in                                                  | `scripts/extract_fixed_path_paper_metrics.py`                 | fallback / supplement；按 readiness 决定 | 禁止作为控制输入；仅 rosbag/metrics/report         |
| profile baseline     | Hamaguchi-style                                  | `config/profile_baselines/hamaguchi_profile.yaml`                                        | wrapper: `scout_profile_baselines/scripts/generate_hamaguchi_profile.py`；impl: `scripts/hamaguchi/generate_profile.py` | `scripts/run_fixed_path_profile_baseline_suite.sh` + common tracker                                        | `profile_baseline_metrics_aggregate.csv`、combined comparison | supplementary profile baseline           | generator/tracker 不消费 monitor；monitor 只做评价 |
| profile baseline     | Lim-style                                        | `config/profile_baselines/lim_profile.yaml`                                              | wrapper: `scout_profile_baselines/scripts/generate_lim_style_profile.py`；impl: `scripts/lim/generate_profile.py` | `scripts/run_fixed_path_profile_baseline_suite.sh` + common tracker                                        | `profile_baseline_metrics_aggregate.csv`、combined comparison | supplementary profile baseline           | generator/tracker 不消费 monitor；monitor 只做评价 |
| modern baseline candidate | LT-DWA                                      | `config/baselines/lt_dwa_adapter_standalone_sim.yaml` + `config/benchmark/capability_matrix.yaml` | adapter: `src/scout_apps/control/lt_dwa_adapter/`；reference: `third_party/LT_DWA/` source-only vendor | `launch/sim/run_lt_dwa_fixed_path_sim.launch`；`scripts/run_fixed_path_baseline_suite.sh`；isolated smoke wrapper | `/baseline/lt_dwa/status`、local/global plan、metrics extractor | adapter 已可运行；formal 主表仍按 gate/证据审批 | 不消费 monitor；仅 rosbag/metrics/report |
| advanced candidate   | `src/mpc_planner`                              | `config/benchmark/capability_matrix.yaml`                                                | `src/mpc_planner/`，solver/deps readiness-gated                                    | `scripts/bench_check_advanced_baseline_readiness.py`                                                       | readiness report                                                | candidate；未通过前不进表                | 不适用                                             |

## 3. Hamaguchi/Lim profile baseline 的链路

Hamaguchi/Lim 不直接作为 online local planner；它们是补充的液体防晃 profile baseline：

```text
fixed path JSON
  -> offline profile generator
       generate_hamaguchi_profile.py
       generate_lim_style_profile.py
  -> profile CSV
       s_normalized, s_m, t_s, x, y, yaw,
       v_ref_m_s, a_ref_m_s2, jerk_ref_m_s3, method
  -> common tracker
       scout_local_planner/launch/slosh_experiment_sim.launch
       external_profile_mode=custom_csv
       Q_slosh=0
  -> /cmd_vel
  -> rosbag / metrics / report
```

禁止链路：

```text
/slosh/* 或 /benchmark/slosh_monitor/*
  -> profile generator during test
  -> command gate
  -> SPMPC OCP
  -> planner input
  -> /cmd_vel chain
```

允许链路：

```text
/slosh/* 或 /benchmark/slosh_monitor/*
  -> rosbag
  -> metrics extractor
  -> plots / report
```

## 4. 运行入口分类

### SPMPC fixed-path

```bash
VARIANTS=B_ours \
PATH_SOURCE_MODE=stable_goal \
PATH_ID=P2_s_curve_current_start \
OUT_ROOT=/data/a/spmpc_paper_compare/example_spmpc \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh
```

### TEB/DWA/mpc_local_planner/LT-DWA fixed-path

```bash
BASELINE=teb \
PATH_SOURCE_MODE=stable_goal \
PATH_ID=P2_s_curve_current_start \
OUT_ROOT=/data/a/spmpc_paper_compare/example_baseline \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_baseline_suite.sh
```

`mpc_local_planner` 需要显式 opt-in；未确认依赖与 readiness 前不要放入主表。`lt_dwa` 使用 Scout-owned adapter，默认 shadow-only；isolated closed-loop smoke 已验证通路，但 formal 主表仍需按 capability/main-table gate 和 freshness evidence 明确审批。

### Hamaguchi/Lim profile baseline

```bash
PROFILE_BASELINE=hamaguchi_profile \
PATH_SOURCE_MODE=stable_goal \
PATH_ID=P2_s_curve_current_start \
OUT_ROOT=/data/a/spmpc_paper_compare/profile_baseline_smoke \
bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_profile_baseline_suite.sh
```

该入口只做 current-sim/smoke 编排；正式结果必须由 `/data/a/scout_sim_replacement` 外层 strict wrapper 每 case 单独启停仿真并生成 freshness evidence。

### Phase-0 / contract gate

fixed-path suite 入口会先调用 `scripts/bench_run_phase0_preflight.sh`，该 wrapper 只运行 `bench_preflight.py --dry-run`，不启动 ROS/Gazebo、不修改控制链。需要检查整体对比方法 contract 时，可单独运行：

```bash
python3 src/scout_apps/control/spmpc_experiments/scripts/bench_validate_comparison_contracts.py --format yaml
```

如显式设置 `SKIP_PHASE0_PREFLIGHT=true`，该 run 只能作为 current-sim diagnostics，不应进入 formal/main-table evidence。

## 5. 结果文件分类

当前 Hamaguchi/Lim N=3 strict fresh 结果示例：

```text
/data/a/scout_sim_replacement/results/strict_fresh_profile_n1_20260619_005425/profile_baseline_metrics_per_case.csv
/data/a/scout_sim_replacement/results/strict_fresh_profile_n1_20260619_005425/profile_baseline_metrics_aggregate.csv
/data/a/scout_sim_replacement/results/strict_fresh_profile_n1_20260619_005425/profile_baseline_combined_comparison.csv
/data/a/scout_sim_replacement/results/strict_fresh_profile_n1_20260619_005425/profile_baseline_combined_comparison.md
```

总表镜像：

```text
/data/a/Obsidian/vaults/StudyVault/30-Projects/MPC/规控一体的实验记录/仿真实验/20260614_SPMPC仿真结果总表.md
```

## 6. 当前物理隔离状态

Hamaguchi/Lim generator 的真实实现已经放在独立包中：

```text
scout_profile_baselines/scripts/
  generate_hamaguchi_profile.py   # stable rosrun wrapper
  generate_lim_style_profile.py   # stable rosrun wrapper
  hamaguchi/generate_profile.py   # Hamaguchi-style implementation
  lim/generate_profile.py         # Lim-style implementation
  common/advanced_profile_common.py
  common/path_profile_utils.py
```

旧控制器目录只保留兼容 wrapper，避免旧命令和外层脚本立即失效：

```text
scout_local_planner/scripts/analysis/generate_hamaguchi_profile.py
scout_local_planner/scripts/analysis/generate_lim_style_profile.py
```

`scout_local_planner/scripts/analysis/path_profile_utils.py` 继续保留给 TOPPRA/Ruckig/Biagiotti 旧 profile 工具使用；profile-baseline 正式 schema source 以 `scout_profile_baselines/scripts/common/path_profile_utils.py` 为准。

LT-DWA 当前分成两层：

```text
src/scout_apps/control/lt_dwa_adapter/   # Scout-owned runnable adapter
third_party/LT_DWA/                      # upstream source-only reference
```

不要把 `third_party/LT_DWA` symlink 到 `src/`，因为上游包含 `local_planner`、`navigation` 等与本 workspace 冲突的 catkin package 名称。可运行对比只通过 `lt_dwa_adapter` 进入；adapter 默认 `publish_cmd_vel=false`，仿真闭环必须由隔离 suite 显式开启。适配边界见 `third_party/LT_DWA/SCOUT_ADAPTER_PLAN.md`。
