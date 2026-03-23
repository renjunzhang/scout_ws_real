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
# 录制内容：
#   - 晃动状态：/slosh/state, /slosh/height, /slosh/height_pred_max
#                /slosh/q_slosh_eta
#                /slosh/ax_est, /slosh/ay_est, /slosh/alpha_est
#                /slosh/omega_est_used, /slosh/imu_omega_z_filtered
#                /slosh/imu_ay_bias, /slosh/imu_ay_filtered, /slosh/imu_ay_bias_ready
#                /slosh/episode_id, /slosh/constraint_active
#                /slosh/v_des_eff, /slosh/speed_governor_active
#   - RealSense 原始输入：
#                /camera/color/image_raw
#                /camera/color/camera_info
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
#   - MPC 性能：/mpc/solve_ms, /mpc/status_val, /mpc_status
#   - 控制/底盘状态：/cmd_vel, /odom, /scout_status, /rs_status
#   - 目标/路径：/scout/goal, /scout/current_goal, /scout/global_path, /local_path
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
#   - TF：      /tf, /tf_static
#   - 仿真时钟：/clock（仿真时存在，实物可忽略）
#
# 输出路径：
#   1. 若设置环境变量 SLOSH_BAG_DIR，则优先使用它
#   2. 若存在 /data/$USER，则使用 /data/$USER/slosh_bags
#   3. 否则退回 ~/slosh_bags（适合实物）
# ============================================================================

set -euo pipefail

# ---------- 参数 ----------
Q_SLOSH="${1:-0}"
SUFFIX="${2:-}"
DATE_STR=$(date +%Y%m%d_%H%M%S)

# 目录
if [[ -n "${SLOSH_BAG_DIR:-}" ]]; then
    BAG_DIR="${SLOSH_BAG_DIR}"
elif [[ -d "/data/${USER}" ]]; then
    BAG_DIR="/data/${USER}/slosh_bags"
else
    BAG_DIR="${HOME}/slosh_bags"
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

    # RealSense 原始图像
    /camera/color/image_raw
    /camera/color/camera_info

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

    # IMU 原始输入
    /imu/data
    /wit/mag

    # MPC 性能
    /mpc/solve_ms
    /mpc/status_val
    /mpc_status

    # 控制与状态
    /cmd_vel
    /odom
    /scout_status
    /rs_status

    # 目标与路径
    /scout/goal
    /scout/current_goal
    /scout/global_path
    /scout/global_path_smooth
    /local_path
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
echo "  输出文件 = ${BAG_PATH}.bag"
echo "  话题数   = ${#TOPICS[@]}"
echo "============================================"
echo "  Ctrl+C 停止录制"
echo "============================================"

rosbag record -O "${BAG_PATH}" "${TOPICS[@]}"
