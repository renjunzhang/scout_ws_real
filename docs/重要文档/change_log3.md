## 2026-04-14

- 仿真 Scout Mini 增加 `imu_link`，并通过 Gazebo IMU 插件发布 `/imu/data`，使仿真和实物 IMU 话题/坐标系口径更一致。
- 新增 `launch_sim_nav_stack.sh`，用于按顺序启动 Gazebo、NanoScan3 sim、Cartographer localization sim、MBF global planner sim，并在 MBF 启动后执行后退 + 原地自转刷新定位。
- `slosh_experiment.launch` 保留 `risk_scheduler_enable` 参数，撤销临时 `/odom -> /imu/data` 代理入口，避免和 Gazebo 原生 IMU 重复发布。
- `record_slosh_experiment.sh` 增加 `/risk_scheduler/*` 话题录制；实物 bag 路径保持 `/data/a/slosh_bags`，仿真通过 `SLOSH_BAG_MODE=sim` 分流到 `/data/a/slosh_bags/sim`。
- Day2 `rho_k` 调度器仿真验证通过：正常 IMU 下 `fallback=false`，缺失 IMU 下 `fallback=true` 且 `Q_eta_k=5.0`。
- 新增 `analyze_sim_speed_issue.py`，用于从 bag 中检查 `/cmd_vel`、`/odom`、`/slosh/v_des_eff`、`/risk_scheduler/*`、MPC 状态和终端模式，辅助定位仿真慢速问题。
- MPC 增加 `mpc/cmd_vel_lead_time`：实物配置保持 `-1.0` 沿用原半步输出，仿真配置设为 `0.25s` 作为 Gazebo 速度反馈滞后的专用补偿。
- 拆分 anti-slosh 实验入口：`slosh_experiment.launch` 固定加载实物 `mpc_params.yaml`，新增 `slosh_experiment_sim.launch` 固定加载仿真 `mpc_params_sim.yaml`。
- 根据 `slosh_Q5_20260414_213501_test1.bag` 初步分析，仿真 `/local_path` 与全局路径偏离偏大，收敛仿真配置：关闭 B-spline 平滑、提高 `Q_contour`，并降低 `cmd_vel_lead_time`、`omega_max`、`alpha_max`，避免高线速度配大角速度导致 Gazebo 行为过激。
- 继续收敛仿真贴线参数：降低 `lookahead_distance`，提高 `Q_contour/Q_etheta`，并降低 `max_lat_accel`，优先让仿真车贴全局路径后再处理 Gazebo 翘头问题。
