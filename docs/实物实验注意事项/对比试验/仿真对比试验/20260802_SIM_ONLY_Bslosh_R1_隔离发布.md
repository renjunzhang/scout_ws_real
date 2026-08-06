# SIM-ONLY Bslosh R1 隔离发布（2026-08-02）

## 结论

`SIM_Bslosh_R1` 是独立的仿真控制器 condition，协议为
`SMPCC-SIM-ONLY-BSLOSH-R1-v1`。它**不是**
`SMPCC-SIM-40-64-88-v0.1` 的实物对齐 Bslosh，也不能生成、消费或解释为原
40/64/88 矩阵数据。

它是新的 clean identity，不是 `W5_S10`，也不是历史 0705 `B_slosh` 的重新命名。

## 固定控制器身份

- 10-state `continuous_mpcc_acados`；`v_ref=0.20`。
- `w_control=0.3`，`w_smooth=w_alpha=w_du_a=w_du_vs=1.0`，`w_slosh=5.0`；其余
  tracking 权重为 `w_contour=1`、`w_lag=w_progress=.2`、`w_v=1`、`w_vs=.3`。
- nominal observer=`odom`，IMU shadow=false，delay=`off`，30 Hz、N=60。
- 仅允许两个 simulation-only、unvalidated 容器 condition：C1 37 mm / 58 mm，或
  C2 95 mm / 58 mm。C2 除半径外保持模型参数不变。

专用入口：

```bash
roslaunch spmpc_local_planner spmpc_sim_only_bslosh_r1.launch \
  sim_only_release_ack:=true \
  sim_only_container_condition:=C1  # 或 C2
```

该 launch 不会设置全局 `/use_sim_time`；它只能连接已经由仿真环境建立的 master。
运行时默认拒绝，且要求已有 `/use_sim_time=true`、显式 ACK、上述 observer/delay/
backend、完整 variant 参数块和已批准的 C1/C2 配对。任何漂移均在订阅或发布前
`ROS_FATAL`。

## 协议与数据隔离

发布物是只读的：

```text
/data/a/scout_sim_replacement/SMPCC-SIM-ONLY-BSLOSH-R1/releases/SIM-ONLY-BSLOSH-R1/
```

其中 `release_manifest.json` SHA-256 为
`d794d522b6d0b7e6f3ead1876e92848603bdd7f9ac88e850827169ae1dcbaf1a`。它 hash 绑定
controller/config/launch、ACADOS source/model、C1/C2、实际 proxy world/map，并且
不创建 results、bags、planned rows、随机表或 dataset ledger。

原 `smpcc_sim_toolchain.py formal-gate` 已实测以非零退出拒绝该 manifest；它同时
明确报告 protocol mismatch 和 SIM-ONLY isolation。新 release 的
`formal_simulation_only=true` 仅表示独立仿真协议的发布状态，通用 `formal=false`，
且 `physical_alignment=false`、`physical_primary_eligible=false`。

## liquid measurement 边界

`/slosh/height` 仍为 `H_proxy`，`/spmpc/slosh_height` 仍为 `H_modal`。二者不可被
称作独立 plant truth 或 physical primary；独立 plant 的 formal capability/fidelity/
firewall 仍须按原 40/64/88 协议单独满足。

本发布没有产生任何轨迹、bag 或实验数据。
