# Profile baseline 配置索引

本目录放 **对比实验视角的 profile baseline 方法配置**，不是控制器实现目录。

当前方法：

| 方法 | 配置文件 | stable generator 入口 | 真实实现 | 运行入口 | 表格角色 |
|---|---|---|---|---|---|
| Hamaguchi-style profile | `hamaguchi_profile.yaml` | `src/scout_apps/control/scout_profile_baselines/scripts/generate_hamaguchi_profile.py` | `src/scout_apps/control/scout_profile_baselines/scripts/hamaguchi/generate_profile.py` | `src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_profile_baseline_suite.sh` | supplementary profile baseline |
| Lim-style profile | `lim_profile.yaml` | `src/scout_apps/control/scout_profile_baselines/scripts/generate_lim_style_profile.py` | `src/scout_apps/control/scout_profile_baselines/scripts/lim/generate_profile.py` | `src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_profile_baseline_suite.sh` | supplementary profile baseline |

## 物理隔离

Hamaguchi/Lim 的真实实现已经从老控制器 analysis 目录拆出，并在 `scout_profile_baselines` 内按方法分目录：

```text
src/scout_apps/control/scout_profile_baselines/
  scripts/generate_hamaguchi_profile.py   # stable wrapper
  scripts/generate_lim_style_profile.py   # stable wrapper
  scripts/hamaguchi/generate_profile.py   # Hamaguchi-style implementation
  scripts/lim/generate_profile.py         # Lim-style implementation
  scripts/common/advanced_profile_common.py
  scripts/common/path_profile_utils.py
```

职责分工：

```text
scout_profile_baselines  = offline profile generator + generator-local helpers
scout_local_planner      = common external-profile tracker + legacy wrapper entrypoints
spmpc_experiments        = benchmark policy / suite / preflight / metrics
```

旧入口 `rosrun scout_local_planner generate_hamaguchi_profile.py` 和
`rosrun scout_local_planner generate_lim_style_profile.py` 仅作为薄 wrapper 保留，正式入口为：

```bash
rosrun scout_profile_baselines generate_hamaguchi_profile.py --help
rosrun scout_profile_baselines generate_lim_style_profile.py --help
```

## 统一链路

```text
fixed path JSON
  -> offline profile generator
       generate_hamaguchi_profile.py / generate_lim_style_profile.py
  -> profile CSV
  -> common tracker
       external_profile_mode=custom_csv
       Q_slosh=0
  -> /cmd_vel
  -> rosbag / metrics / report
```

Hamaguchi/Lim 不是 online local-planner 同层主表方法；论文中应标为 supplementary/slosh-specific profile baseline。

## monitor-only 边界

禁止：

```text
/slosh/* 或 /benchmark/slosh_monitor/*
  -> profile generator during test
  -> command gate
  -> SPMPC OCP
  -> planner input
  -> /cmd_vel chain
```

允许：

```text
/slosh/* 或 /benchmark/slosh_monitor/*
  -> rosbag
  -> metrics extractor
  -> plot/report
```
