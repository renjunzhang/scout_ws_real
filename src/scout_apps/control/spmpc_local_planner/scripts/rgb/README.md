# 红色液体高度识别：脚本索引

[返回总索引](../README.md) · [权威固定流程](../../../../../../docs/重要文档/红色液体视觉验证固定流程.md)

这条流程是：**调好相机 → 录原始 RGB → 取帧、标三把尺和 HSV → 离线识别 → 检查识别质量 → 比较液面指标**。识别核心在传感器包，本目录负责相机检查；录包器在 [real/](../real/README.md)。

## 1. 相机与采集准备

| 脚本 | 作用与适用条件 |
| --- | --- |
| [launch_real_sensors_stack.sh](../../../scout_local_planner/scripts/launch_real_sensors_stack.sh) | 旧实物传感器启动入口，包含 RealSense；按本次部署选择传感器栈 |
| [tune_realsense_red_liquid_exposure.py](../../../../sensors/realsense_liquid_measurement/scripts/tune_realsense_red_liquid_exposure.py) | 扫曝光、增益、白平衡，输出图像和 CSV |
| [interactive_realsense_red_tuner.py](../../../../sensors/realsense_liquid_measurement/scripts/interactive_realsense_red_tuner.py) | 现场交互调相机、HSV 与检测参数 |
| [set_realsense_rgb_manual_params.sh](../../../scout_local_planner/scripts/set_realsense_rgb_manual_params.sh) | 固定手动相机参数 |
| [prepare_spmpc_g3_realsense.sh](prepare_spmpc_g3_realsense.sh) | **仅复现 G3 冻结批次**：硬编码现场资产及 1920×1080 参数；默认会应用设置，`VALIDATE_ONLY=true` 只校验文件/哈希 |
| [validate_realsense_timestamp_health.py](validate_realsense_timestamp_health.py) | 在线检查源/接收时间、间隔、时钟速率，输出 JSON |
| [diagnose_spmpc_rgb_recording.py](diagnose_spmpc_rgb_recording.py) | 静止对比不录制、只录 RGB、完整录制；检查帧率与丢帧发生在哪一段 |

G3 准备脚本不是新实验必经步骤；新批次按实际光照、视角和标定确定参数。静态诊断需要运行中的 ROS/相机，会做短时录制，但不发速度。

## 2. 录制原始 RGB

[record_spmpc_full_rgb_bag.sh](../real/record_spmpc_full_rgb_bag.sh) 名字虽含 `full_rgb`，**默认 `RECORD_RGB=false`**。做离线识别时显式打开图像，并核对 bag 中有 `/camera/color/image_raw` 和 `/camera/color/camera_info`。

从仓库根目录执行（只录制，控制器单独启动）：

```bash
SPMPC_SCRIPTS="$PWD/src/scout_apps/control/spmpc_local_planner/scripts"
RECORD_RGB=true RECORD_CAMERA_INFO=true RECORD_SEC=60 \
  OUT_DIR=/path/to/new_run RUN_LABEL=full_01 \
  bash "$SPMPC_SCRIPTS/real/record_spmpc_full_rgb_bag.sh"
```

保留 bag、实际相机参数、topic 清单和录制 sidecar。只录 `/liquid/measurement` 的协议没有可供重新识别的原图。

## 3. 取帧、标定、识别

以下脚本都属于 `realsense_liquid_measurement`，继续在传感器包维护。

| 顺序 | 脚本 | 输入 → 输出 |
| --- | --- | --- |
| 取标定帧和峰值帧 | [calibrate_liquid_roi.py](../../../../sensors/realsense_liquid_measurement/scripts/calibrate_liquid_roi.py) | RGB bag + 帧号 → PNG |
| 标三把尺 | [red_liquid_calibrate.py](../../../../sensors/realsense_liquid_measurement/scripts/red_liquid_calibrate.py) | 图像 → ROI、内管壁、左/中/右 0/2/4/6/8 mm 标尺 YAML 和预览 |
| 采 HSV | [red_liquid_sample_hsv.py](../../../../sensors/realsense_liquid_measurement/scripts/red_liquid_sample_hsv.py) | 静止和峰值帧 + 标定 → 红色液体阈值 |
| 离线高度识别 | [red_liquid_infer_from_bag.py](../../../../sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py) | 原始 RGB + 本 run 标定/HSV → `*_red_top.csv`、曲线、debug 图 |
| 可选在线监控 | [online_liquid_height_node.py](../../../../sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py)、[launch](../../../../sensors/realsense_liquid_measurement/launch/online_liquid_height.launch) | 复用红色检测核；发布带图像时间戳/质量信息的 `/liquid/measurement` 及高度 topic |

标定按固定流程逐 run 核对，HSV 要覆盖运动峰值帧。在线与离线复用同一红色检测核，但采样、零点、平滑和质量筛选仍须对齐；该固定流程的最终分析以离线结果为准。

```bash
LIQUID_SCRIPTS="$PWD/src/scout_apps/sensors/realsense_liquid_measurement/scripts"
python3 "$LIQUID_SCRIPTS/red_liquid_infer_from_bag.py" \
  --bag /path/run.bag --calibration /path/run_calibration.yaml \
  --out-dir /path/run_red_results --topic /camera/color/image_raw \
  --zero-correction-frames 30 --smooth-frames 5 --debug-every 30
```

按固定流程填写本 run 的 HSV 参数；逐帧预览应确认三列都贴合液面，且没有被 ROI 裁断或误识别反光。

## 4. 指标与分析入口

固定流程主指标为 `H_vis = abs(rolling_median(max(h_left, h_center, h_right), 5))`；`h_smooth_corr`/三列中位数用于质量核查。保留各 run 零点信息，不把不同高度列混为一个指标。

| 分析目的 | 入口 |
| --- | --- |
| 旧固定路径 MPC cost 与 RGB 对比 | [analyze_fixed_path_cost_effect.py](../../../scout_local_planner/scripts/analysis/analyze_fixed_path_cost_effect.py) |
| 旧 MPC cost 分解 | [extract_mpc_cost_breakdown.py](../../../scout_local_planner/scripts/analysis/extract_mpc_cost_breakdown.py) |
| G2S 三包原始 RGB 选择来源 | [分析 wrapper](../protocols/g_series/analyze_spmpc_g2s_raw_rgb_three_trial.sh)、[Python 分析](../analysis/analyze_g2s_raw_rgb_three_trial.py) |
| 历史 I0 ABBA 的 RGB 对比 | [analyze_i0_failclosed_fixed_abba_rgb.py](../analysis/analyze_i0_failclosed_fixed_abba_rgb.py) |
| G3 在线测量验收 | [validate_g3_online_rgb_trial.py](../analysis/validate_g3_online_rgb_trial.py) |

旧 fixed-path 的 tracking 到 terminal 前窗口只适用于原协议；新主线需覆盖约定的整趟运输与停车评价。`/slosh/height`、IMU/odom 驱动的液体观察器都是模型量。

另一套 [extract_liquid_height_v2_from_bag.py](../../../../sensors/realsense_liquid_measurement/scripts/extract_liquid_height_v2_from_bag.py) 使用 v2 多尺度标定，**不是本红色三标尺流程**。历史 ArUco/Hough `extract_visual_height.py` 也不能替代这里的 red-infer。

原始 bag、逐 run 标定、HSV、debug 图放实验数据目录；代码仓库只保留必要索引和少量报告。运行依赖 ROS Noetic、rosbag/cv_bridge、OpenCV、NumPy、Matplotlib、PyYAML；在线节点还需构建对应消息。
