# scout_profile_baselines

本包集中存放固定路径对比实验用的 **offline profile baseline** 生成器。

## 职责边界

- 本包只负责离线生成 profile CSV：
  - `generate_hamaguchi_profile.py`：Hamaguchi/Taniguchi 风格 two-impulse input-shaped profile。
  - `generate_lim_style_profile.py`：Lim-style 离线 slosh-aware retiming profile。
- 生成器只读取 fixed path JSON 和冻结的模型/限幅参数，只写 profile CSV。
- 生成器不订阅 ROS topic，不读取 slosh monitor，不发布 `/cmd_vel`。
- runtime common tracker 仍在 `scout_local_planner` 中。
- suite、preflight、freshness、metrics 仍由 `spmpc_experiments` 管理。

## 典型链路

```text
scout_profile_baselines generator
  -> profile_csv
  -> scout_local_planner common external-profile tracker
  -> cmd_vel
  -> spmpc_experiments metrics/report
```

monitor 只允许进入 rosbag/metrics/report，不能进入 generator、tracker、OCP、command gate 或 `/cmd_vel` 控制链。

## 入口

```bash
rosrun scout_profile_baselines generate_hamaguchi_profile.py --help
rosrun scout_profile_baselines generate_lim_style_profile.py --help
```

旧入口 `rosrun scout_local_planner generate_hamaguchi_profile.py` 和
`rosrun scout_local_planner generate_lim_style_profile.py` 仅作为兼容 wrapper 保留。
