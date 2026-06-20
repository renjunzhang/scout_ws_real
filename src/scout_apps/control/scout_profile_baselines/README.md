# scout_profile_baselines

本包集中存放固定路径对比实验用的 **offline profile baseline** 生成器。

## 职责边界

- 本包只负责离线生成 profile CSV：
  - `scripts/hamaguchi/generate_profile.py`：Hamaguchi/Taniguchi 风格 two-impulse input-shaped profile。
  - `scripts/lim/generate_profile.py`：Lim-style 离线 slosh-aware heuristic retiming profile。
- `scripts/generate_hamaguchi_profile.py` 和 `scripts/generate_lim_style_profile.py` 是稳定 `rosrun` 入口 wrapper。
- 生成器只读取 fixed path JSON 和冻结的模型/限幅参数，只写 profile CSV。
- 生成器不订阅 ROS topic，不读取 slosh monitor，不发布 `/cmd_vel`。
- runtime common tracker 仍在 `scout_local_planner` 中。
- suite、preflight、freshness、metrics 仍由 `spmpc_experiments` 管理。

## 目录结构

```text
scout_profile_baselines/scripts/
  generate_hamaguchi_profile.py   # stable rosrun wrapper
  generate_lim_style_profile.py   # stable rosrun wrapper

  hamaguchi/
    generate_profile.py           # Hamaguchi-style fixed-path implementation

  lim/
    generate_profile.py           # Lim-style heuristic retiming implementation

  common/
    advanced_profile_common.py    # shared retiming/slosh/profile helpers
    path_profile_utils.py         # shared path JSON / CSV / plot helpers
```

`common/` 中的 helper 会随包安装以保证 `rosrun`/install-space 可用，但不是单独对比算法入口。

## 典型链路

```text
scout_profile_baselines generator
  -> profile_csv
  -> scout_local_planner common external-profile tracker
  -> cmd_vel
  -> spmpc_experiments metrics/report
```

monitor 只允许进入 rosbag/metrics/report，不能进入 generator、tracker、OCP、command gate 或 `/cmd_vel` 控制链。

## CSV contract

生成器与 common tracker 的稳定接口是 profile CSV，而不是 ROS topic 或 monitor feedback。CSV columns 必须与 `spmpc_experiments/config/benchmark/profile_tracking_common.yaml` 保持一致：

```text
s_normalized, s_m, t_s, x, y, yaw, v_ref_m_s, a_ref_m_s2, jerk_ref_m_s3, method
```

## 入口

```bash
rosrun scout_profile_baselines generate_hamaguchi_profile.py --help
rosrun scout_profile_baselines generate_lim_style_profile.py --help
```

旧入口 `rosrun scout_local_planner generate_hamaguchi_profile.py` 和
`rosrun scout_local_planner generate_lim_style_profile.py` 仅作为兼容 wrapper 保留。
