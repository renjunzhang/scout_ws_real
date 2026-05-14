#!/bin/bash
# ============================================================================
# 液体晃动抑制实验 — rosbag 录制脚本
# ============================================================================
#
# 使用方式：
#   ./record_slosh_experiment.sh                # 默认 Q_slosh=0
#   ./record_slosh_experiment.sh 10             # Q_slosh=10
#   ./record_slosh_experiment.sh 10 trial_3     # Q_slosh=10, 自定义后缀
#
# 完整 MPC 启动模板:
#   roslaunch scout_local_planner slosh_experiment.launch \
#     global_path_topic:=/scout/global_path_anti_slosh \
#     Q_slosh:=0 ... (其他参数)
#
# 说明：
#   terminal recovery 默认开启；recovery 分支会持续发布 /slosh/*，
#   不需要为了录包额外关闭 terminal_recovery_enable。
#
# 录制内容：
#   - 晃动状态：/slosh/state, /slosh/height, /slosh/height_pred_max
#                /slosh/q_slosh_eta
#                /slosh/ax_est, /slosh/ay_est, /slosh/alpha_est
#                /slosh/omega_est_used, /slosh/imu_omega_z_filtered
#                /slosh/imu_ay_bias, /slosh/imu_ay_filtered, /slosh/imu_ay_bias_ready
#                /slosh/episode_id, /slosh/constraint_active
#                /slosh/v_des_eff, /slosh/speed_governor_active
#   - 风险调度器：/risk_scheduler/rho_k, /risk_scheduler/r_k, /risk_scheduler/u_k
#                /risk_scheduler/Q_eta_k, /risk_scheduler/fallback_active
#   - RealSense 原始输入：
#                /camera/color/image_raw
#                /camera/color/camera_info
#                /camera/depth/image_rect_raw
#                /camera/depth/camera_info
#   - 视觉液面测量：
#                /liquid_measurement/height_left_px
#                /liquid_measurement/height_right_px
#                /liquid_measurement/height_peak_px
#                /liquid_measurement/height_left_rel_px
#                /liquid_measurement/height_right_rel_px
#                /liquid_measurement/height_peak_rel_px
#                /liquid_measurement/height_left_rel_mm
#                /liquid_measurement/height_right_rel_mm
#                /liquid_measurement/height_peak_rel_mm
#                /liquid_measurement/meniscus_valid
#                /liquid_measurement/meniscus_confidence
#                /liquid_measurement/debug_image
#   - IMU 原始输入：/imu/data, /wit/mag
#   - MPC 性能：/mpc/solve_ms, /mpc/status_val, /mpc_status，/mpc/solve_time_ms
#   - MPC 代价占比：/mpc/cost_breakdown
#   - MPC horizon 晃动预测摘要：/mpc/slosh_horizon_summary
#   - 控制/底盘状态：/cmd_vel, /odom, /scout_status, /rs_status
#   - 目标/路径：/scout/goal, /scout/current_goal, /scout/global_path, /local_path
#                /scout/global_path_anti_slosh
#                /anti_slosh_path/metrics, /anti_slosh_path/candidate_report
#   - 避障实现/接口核查：
#                /scan_front
#                /map, /map_updates
#                /scout/mbf_costmap_nav/GlobalPlanner/plan
#                /scout/mbf_costmap_nav/GlobalPlanner/potential
#                /scout/mbf_costmap_nav/get_path/goal
#                /scout/mbf_costmap_nav/get_path/feedback
#                /scout/mbf_costmap_nav/get_path/result
#                /scout/mbf_costmap_nav/get_path/status
#                /scout/mbf_costmap_nav/local_costmap/costmap
#                /scout/mbf_costmap_nav/local_costmap/costmap_updates
#                /scout/mbf_costmap_nav/local_costmap/footprint
#                /scout/mbf_costmap_nav/global_costmap/costmap
#                /scout/mbf_costmap_nav/global_costmap/costmap_updates
#                /scout/mbf_costmap_nav/global_costmap/footprint
#                /scout/mbf_costmap_nav/local_costmap/parameter_updates
#                /scout/mbf_costmap_nav/local_costmap/inflation_layer/parameter_updates
#                /scout/mbf_costmap_nav/local_costmap/obstacle_layer/parameter_updates
#                /scout/mbf_costmap_nav/global_costmap/parameter_updates
#                /scout/mbf_costmap_nav/global_costmap/inflation_layer/parameter_updates
#   - 终点/摇摆根因排查：
#                /scout/move_base_cmd_vel
#                /scout/mbf_costmap_nav/current_goal
#                /scout/mbf_costmap_nav/exe_path/feedback
#                /scout/mbf_costmap_nav/exe_path/result
#                /scout/mbf_costmap_nav/exe_path/status
#                /scout/mbf_costmap_nav/move_base/feedback
#                /scout/mbf_costmap_nav/move_base/result
#                /scout/mbf_costmap_nav/move_base/status
#                /terminal/mode
#                /terminal/recovery_latched
#                /terminal/goal_info
#   - 定位漂移排查：
#                /amcl_pose, /particlecloud
#                /tracked_pose, /submap_list, /trajectory_node_list
#                /diagnostics, /rosout
#   - TF：      /tf, /tf_static
#   - 仿真时钟：/clock（仿真时存在，实物可忽略）
#
# 输出路径：
#   1. 若设置环境变量 SLOSH_BAG_DIR，则优先使用它
#   2. 若存在 /data/$USER，实物默认沿用 /data/$USER/slosh_bags
#   3. 仿真设置 SLOSH_BAG_MODE=sim 时使用 /data/$USER/slosh_bags/sim
#   4. 否则退回 ~/slosh_bags（适合实物）
#   SLOSH_BAG_MODE 默认 real
# ============================================================================

set -euo pipefail

# ---------- 参数 ----------
Q_SLOSH="${1:-0}"
SUFFIX="${2:-}"
DATE_STR=$(date +%Y%m%d_%H%M%S)
SLOSH_BAG_MODE="${SLOSH_BAG_MODE:-real}"

# 目录
if [[ -n "${SLOSH_BAG_DIR:-}" ]]; then
    BAG_DIR="${SLOSH_BAG_DIR}"
elif [[ -d "/data/${USER}" ]]; then
    if [[ "${SLOSH_BAG_MODE}" == "sim" ]]; then
        BAG_DIR="/data/${USER}/slosh_bags/sim"
    else
        BAG_DIR="/data/${USER}/slosh_bags"
    fi
else
    if [[ "${SLOSH_BAG_MODE}" == "sim" ]]; then
        BAG_DIR="${HOME}/slosh_bags/sim"
    else
        BAG_DIR="${HOME}/slosh_bags"
    fi
fi
mkdir -p "${BAG_DIR}"

# 文件名
if [[ -n "${SUFFIX}" ]]; then
    BAG_NAME="slosh_Q${Q_SLOSH}_${DATE_STR}_${SUFFIX}"
else
    BAG_NAME="slosh_Q${Q_SLOSH}_${DATE_STR}"
fi

BAG_PATH="${BAG_DIR}/${BAG_NAME}"

# ---------- 录制话题 ----------
TOPICS=(
    # 晃动观测
    /slosh/state
    /slosh/eta_norm
    /slosh/eta_dot_norm
    /slosh/modal_energy
    /slosh/modal_energy_norm
    /slosh/excitation_ay_abs
    /slosh/excitation_alpha_abs
    /slosh/height
    /slosh/height_pred_max
    /slosh/q_slosh_eta
    /slosh/ax_est
    /slosh/ay_est
    /slosh/alpha_est
    /slosh/omega_est_used
    /slosh/imu_omega_z_filtered
    /slosh/imu_ay_bias
    /slosh/imu_ay_filtered
    /slosh/imu_ay_bias_ready
    /slosh/episode_id
    /slosh/constraint_active
    /slosh/v_des_eff
    /slosh/speed_governor_active
    /slosh/speed_cap_active
    /slosh/speed_cap_v_limit
    /slosh/settling_time

    # 风险调度器
    /risk_scheduler/rho_k
    /risk_scheduler/r_k
    /risk_scheduler/u_k
    /risk_scheduler/Q_eta_k
    /risk_scheduler/fallback_active

    # RealSense 原始图像
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/depth/image_rect_raw
    /camera/depth/camera_info

    # 视觉液面测量（节点存在时会自动录到；节点未启动时不影响 rosbag record）
    /liquid_measurement/height_left_px
    /liquid_measurement/height_right_px
    /liquid_measurement/height_peak_px
    /liquid_measurement/height_left_rel_px
    /liquid_measurement/height_right_rel_px
    /liquid_measurement/height_peak_rel_px
    /liquid_measurement/height_left_rel_mm
    /liquid_measurement/height_right_rel_mm
    /liquid_measurement/height_peak_rel_mm
    /liquid_measurement/meniscus_valid
    /liquid_measurement/meniscus_confidence
    /liquid_measurement/debug_image

    # RA-L D5 视觉 GT (extract_visual_height.py 输出；节点未启动时自动跳过)
    /slosh/h_visual
    /slosh/h_visual_quality

    # RA-L §4.1 SAFETY_ALARM topic (post-processor 在 hard gate 全失败时一次性发布)
    /anti_slosh_path/safety_alarm

    # IMU 原始输入
    /imu/data
    /wit/mag

    # MPC 性能
    /mpc/solve_ms
    /mpc/solve_time_ms
    /mpc/status_val
    /mpc/cost_breakdown
    /mpc/slosh_horizon_summary
    /mpc_status

    # 控制与状态
    /cmd_vel
    /odom
    /scout_status
    /rs_status

    # 避障实现/接口核查
    /scan_front
    /map
    /map_updates

    # 目标与路径
    /scout/goal
    /scout/current_goal
    /scout/global_path
    /scout/global_path_anti_slosh
    /scout/global_path_fixed
    /scout/global_path_smooth
    /anti_slosh_path/metrics
    /anti_slosh_path/candidate_report
    /anti_slosh_path/debug/original
    /anti_slosh_path/debug/mild
    /anti_slosh_path/debug/medium
    /anti_slosh_path/debug/mid
    /anti_slosh_path/debug/strong
    /mpc/reference_path
    /local_path
    /scout/mbf_costmap_nav/GlobalPlanner/plan
    /scout/mbf_costmap_nav/GlobalPlanner/potential
    /scout/mbf_costmap_nav/get_path/goal
    /scout/mbf_costmap_nav/get_path/feedback
    /scout/mbf_costmap_nav/get_path/result
    /scout/mbf_costmap_nav/get_path/status
    /scout/mbf_costmap_nav/global_costmap/costmap
    /scout/mbf_costmap_nav/global_costmap/costmap_updates
    /scout/mbf_costmap_nav/global_costmap/footprint
    /scout/mbf_costmap_nav/global_costmap/parameter_updates
    /scout/mbf_costmap_nav/global_costmap/inflation_layer/parameter_updates
    /scout/mbf_costmap_nav/local_costmap/costmap
    /scout/mbf_costmap_nav/local_costmap/costmap_updates
    /scout/mbf_costmap_nav/local_costmap/footprint
    /scout/mbf_costmap_nav/local_costmap/parameter_updates
    /scout/mbf_costmap_nav/local_costmap/inflation_layer/parameter_updates
    /scout/mbf_costmap_nav/local_costmap/obstacle_layer/parameter_updates
    /scout/move_base_cmd_vel
    /scout/mbf_costmap_nav/current_goal
    /scout/mbf_costmap_nav/exe_path/feedback
    /scout/mbf_costmap_nav/exe_path/result
    /scout/mbf_costmap_nav/exe_path/status
    /scout/mbf_costmap_nav/move_base/feedback
    /scout/mbf_costmap_nav/move_base/result
    /scout/mbf_costmap_nav/move_base/status
    /terminal/mode
    /terminal/recovery_latched
    /terminal/goal_info

    # 定位漂移 / scan-map mismatch 排查
    # AMCL 常用输出
    /amcl_pose
    /particlecloud
    # Cartographer 常用输出（不存在时 rosbag 自动跳过）
    /tracked_pose
    /submap_list
    /trajectory_node_list
    # ROS warning / driver diagnostics
    /diagnostics
    /diagnostics_agg
    /rosout

    # 仿真时钟（实物环境无此话题也不影响录制）
    /clock

    # TF (用于回放可视化)
    /tf
    /tf_static
)

echo "============================================"
echo "  液体晃动抑制实验录制"
echo "============================================"
echo "  Q_slosh  = ${Q_SLOSH}"
echo "  mode     = ${SLOSH_BAG_MODE}"
echo "  输出文件 = ${BAG_PATH}.bag"
echo "  话题数   = ${#TOPICS[@]}"
echo "============================================"
echo "  Ctrl+C 停止录制"
echo "============================================"

rosbag record -O "${BAG_PATH}" "${TOPICS[@]}"
