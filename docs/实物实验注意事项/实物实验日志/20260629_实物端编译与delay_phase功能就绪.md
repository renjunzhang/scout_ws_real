# 20260629 实物端编译与 delay phase 功能就绪

## 背景

`diag/lt-dwa-collision-tracking` 分支已包含最新 delay phase 诊断功能（cda1e82 → f3813d4）。实物机代码已同步、solver 已生成、已编译。

## 实物端状态确认

- 分支：`diag/lt-dwa-collision-tracking`，与远端一致
- acados：`/home/geist/acados`，b0 + slosh solver 均存在
- `libspmpc_local_planner.so` (629K)，delay phase 符号已链接（`CommandHistoryBuffer` 12 项）
- `spmpc_local_planner_node` (104K)
- 白名单编译可绕过 `local_map_generation` 缺 `pcl_ros` 问题

## delay phase 功能状态

| 模式 | 说明 | 状态 |
|---|---|---|
| `off` | 默认，零开销 | ✅ |
| `monitor` | 发布时序诊断 topic | ✅ 编译通过 |
| `shadow` | 前向预测执行状态（含 slosh dynamics） | ✅ 编译通过 |

配置入口：`common.yaml` 中 `delay_phase/mode: off`，可通过 launch arg 覆盖。
R0 采集时用 `delay_phase_mode:=shadow` 同步录包诊断。

## 新地图

- `map_carto_20260629_R0.pbstream` (5.3M) — 开门建图，包含门槛区域
- 已转换为 PGM+YAML，resolution=0.02
- 定位 launch 已指向新地图

## 相机参数（锁定值）

| 参数 | 值 |
|---|---|
| enable_auto_exposure | false |
| enable_auto_white_balance | false |
| exposure | 166 |
| gain | 64 |
| white_balance | 4600 |
| brightness | 0 |
| contrast | 50 |
| saturation | 64 |
| sharpness | 50 |

## 待办

- [x] 建新地图 `map_carto_20260629_R0`
- [ ] 三标尺标定
- [ ] R0-Pre：传感器/RGB/TF/录包链路检查
- [ ] R0-A：B_ours 保守速度 N=3，delay_phase shadow 录包
- [ ] R0-QC：离线检查 bag 可回放、RGB 可标注
