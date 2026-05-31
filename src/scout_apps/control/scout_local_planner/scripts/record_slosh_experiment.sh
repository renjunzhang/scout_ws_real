#!/bin/bash
# ============================================================================
# 液体晃动抑制实验 — rosbag 录制脚本
# ============================================================================
#
# 使用方式：
#   ./record_slosh_experiment.sh                # 默认 Q_slosh=0
#   ./record_slosh_experiment.sh 10             # Q_slosh=10
#   ./record_slosh_experiment.sh 10 trial_3     # Q_slosh=10, 自定义后缀
#   SLOSH_RECORD_ALL=false ./record_slosh_experiment.sh 10 trial_3
#                                               # 仅录脚本白名单话题
#
# 完整 MPC 启动模板:
#   roslaunch scout_local_planner slosh_experiment.launch \
#     global_path_topic:=/scout/global_path_fixed \
#     Q_slosh:=0 ... (其他参数)
#
# 说明：
#   terminal recovery 当前实物主实验默认关闭；如需调头/回点兜底，可在 launch 中显式开启。
#   TERMINAL_MPC_STOP / REACHED 阶段仍会持续发布 /slosh/* 调试话题。
#   默认 SLOSH_RECORD_ALL=true，使用 rosbag record -a 全量录制所有当前存在的话题。
#   下方 TOPICS 是白名单 fallback，用于磁盘压力过大时关闭全量录制。
#
# 录制内容：
#   - 录制前自动保存 sidecar 元数据：
#                *_rosparam.yaml, *_topics.txt, *_nodes.txt, *_info.txt
#   - 晃动状态：/slosh/state, /slosh/height, /slosh/height_pred_max
#                /slosh/q_slosh_eta
#                /slosh/eta_norm, /slosh/eta_dot_norm, /slosh/modal_energy*
#                /slosh/ax_est, /slosh/ay_est, /slosh/alpha_est
#                /slosh/omega_est_used, /slosh/imu_omega_z_filtered
#                /slosh/imu_ay_bias, /slosh/imu_ay_filtered, /slosh/imu_ay_bias_ready
#                /slosh/episode_id, /slosh/constraint_active
#                /slosh/v_des_eff
#   - RealSense 原始输入：
#                /camera/color/image_raw
#                /camera/color/camera_info
#                /camera/depth/image_rect_raw
#                /camera/depth/camera_info
#                /camera/aligned_depth_to_color/image_raw
#                /camera/aligned_depth_to_color/camera_info
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
#   - 控制/底盘状态：/cmd_vel, /odom, /scout/cmd_vel, /scout/odom, /scout_status, /rs_status
#   - 参考速度链路：/reference/v_des_raw, /reference/v_des_target,
#                /reference/v_des_eff, /reference/v_des_rate_limited,
#                /reference/v_ref_horizon, /reference/implied_ax
#   - 外部 profile 执行 cap：/profile_cap/*
#   - 目标/路径：/scout/goal, /scout/current_goal, /scout/global_path,
#                /scout/global_path_fixed, /local_path
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
#                /terminal/v_envelope
#                /terminal/envelope_active
#                /terminal/phase_active
#                /terminal/cmd_v_pre_clamp
#                /terminal/cmd_v_post_clamp
#   - 定位漂移排查：
#                /amcl_pose, /particlecloud
#                /tracked_pose, /submap_list, /trajectory_node_list
#                /diagnostics, /rosout
#   - 机器人通用状态：
#                /joint_states, /battery_state, /tf, /tf_static
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
SLOSH_RECORD_ALL="${SLOSH_RECORD_ALL:-true}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || pwd)"

read_rosparam_or_default() {
    local name="$1"
    local default="$2"
    rosparam get "${name}" 2>/dev/null || echo "${default}"
}

EXPERIMENT_GROUP="${EXPERIMENT_GROUP:-$(read_rosparam_or_default /scout_local_planner/experiment_group LEGACY)}"
CONTROLLER_VARIANT="${CONTROLLER_VARIANT:-$(read_rosparam_or_default /scout_local_planner/controller_variant mpc)}"
EXTERNAL_PROFILE_MODE="${EXTERNAL_PROFILE_MODE:-$(read_rosparam_or_default /scout_local_planner/external_profile_mode none)}"
MPC_R_A_META="${MPC_R_A_META:-$(read_rosparam_or_default /scout_local_planner/mpc/R_a unknown)}"
MPC_R_DA_META="${MPC_R_DA_META:-$(read_rosparam_or_default /scout_local_planner/mpc/R_da unknown)}"

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

# 文件名。LEGACY 保持旧命名；正式 group 使用统一前缀，便于 analysis 分组。
if [[ "${EXPERIMENT_GROUP}" != "LEGACY" ]]; then
    BAG_NAME="slosh_${EXPERIMENT_GROUP}_qs${Q_SLOSH}_ra${MPC_R_A_META}_rda${MPC_R_DA_META}_${DATE_STR}"
    if [[ -n "${SUFFIX}" ]]; then
        BAG_NAME="${BAG_NAME}_${SUFFIX}"
    fi
elif [[ -n "${SUFFIX}" ]]; then
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

    # RealSense 原始图像
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/color/metadata
    /camera/color/image_raw/compressed
    /camera/color/image_raw/compressed/parameter_descriptions
    /camera/color/image_raw/compressed/parameter_updates
    /camera/depth/image_rect_raw
    /camera/depth/camera_info
    /camera/depth/metadata
    /camera/depth/image_rect_raw/compressed
    /camera/depth/image_rect_raw/compressed/parameter_descriptions
    /camera/depth/image_rect_raw/compressed/parameter_updates
    /camera/aligned_depth_to_color/image_raw
    /camera/aligned_depth_to_color/camera_info
    /camera/aligned_depth_to_color/image_raw/compressed
    /camera/aligned_depth_to_color/image_raw/compressed/parameter_descriptions
    /camera/aligned_depth_to_color/image_raw/compressed/parameter_updates
    /camera/extrinsics/depth_to_color
    /camera/extrinsics/color_to_depth

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

    # IMU 原始输入
    /imu/data
    /wit/mag
    /camera/gyro/sample
    /camera/accel/sample
    /camera/imu

    # MPC 性能
    /mpc/solve_ms
    /mpc/solve_time_ms
    /mpc/status_val
    /mpc/cost_breakdown
    /mpc/slosh_horizon_summary
    /mpc_status
    /diagnostics/experiment_group
    /diagnostics/controller_variant
    /diagnostics/external_profile_mode
    /diagnostics/mpc_cost_variant

    # 控制与状态
    /cmd_vel
    /odom
    /scout/cmd_vel
    /scout/odom
    /scout_status
    /rs_status
    /joint_states
    /battery_state

    # 避障实现/接口核查
    /scan_front
    /map
    /map_updates

    # 目标与路径
    /scout/goal
    /scout/current_goal
    /scout/global_path
    /scout/global_path_fixed
    /scout/global_path_smooth
    /mpc/reference_path
    /reference/v_des_raw
    /reference/v_des_target
    /reference/v_des_eff
    /reference/v_des_rate_limited
    /reference/v_ref
    /reference/v_ref_horizon
    /reference/s_horizon
    /reference/v_path
    /reference/kappa
    /reference/s
    /reference/implied_ax
    /reference/implied_ay
    /reference/implied_jerk
    /reference/implied_ax_abs_p95
    /reference/implied_ay_abs_p95
    /reference/implied_jerk_abs_p95
    /profile_cap/active
    /profile_cap/v_profile
    /profile_cap/cmd_v_pre_cap
    /profile_cap/cmd_v_post_cap
    /profile_cap/implied_ax
    /profile_cap/implied_jerk
    /rpp_speed_reg/active
    /rpp_speed_reg/curvature
    /rpp_speed_reg/curvature_active
    /rpp_speed_reg/approach_active
    /rpp_speed_reg/v_raw
    /rpp_speed_reg/v_curvature_cap
    /rpp_speed_reg/v_approach_cap
    /rpp_speed_reg/v_out
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
    /terminal/v_envelope
    /terminal/envelope_active
    /terminal/phase_active
    /terminal/cmd_v_pre_clamp
    /terminal/cmd_v_post_clamp

    # 定位漂移 / scan-map mismatch 排查
    # AMCL 常用输出
    /amcl_pose
    /particlecloud
    # Cartographer 常用输出（不存在时 rosbag 自动跳过）
    /tracked_pose
    /submap_list
    /trajectory_node_list
    /scan_matched_points2
    /constraint_list
    /landmark_poses_list
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
echo "  group    = ${EXPERIMENT_GROUP}"
echo "  variant  = ${CONTROLLER_VARIANT}"
echo "  profile  = ${EXTERNAL_PROFILE_MODE}"
echo "  mode     = ${SLOSH_BAG_MODE}"
echo "  record_a = ${SLOSH_RECORD_ALL}"
echo "  输出文件 = ${BAG_PATH}.bag"
echo "  白名单数 = ${#TOPICS[@]}"
echo "============================================"
echo "  Ctrl+C 停止录制"
echo "============================================"

INFO_PATH="${BAG_PATH}_info.txt"
PARAM_PATH="${BAG_PATH}_rosparam.yaml"
TOPIC_PATH="${BAG_PATH}_topics.txt"
NODE_PATH="${BAG_PATH}_nodes.txt"

{
    echo "bag=${BAG_PATH}.bag"
    echo "date=${DATE_STR}"
    echo "user=${USER}"
    echo "host=$(hostname)"
    echo "pwd=$(pwd)"
    echo "q_slosh=${Q_SLOSH}"
    echo "experiment_group=${EXPERIMENT_GROUP}"
    echo "controller_variant=${CONTROLLER_VARIANT}"
    echo "external_profile_mode=${EXTERNAL_PROFILE_MODE}"
    echo "mpc_R_a=${MPC_R_A_META}"
    echo "mpc_R_da=${MPC_R_DA_META}"
    echo "suffix=${SUFFIX}"
    echo "mode=${SLOSH_BAG_MODE}"
    echo "record_all=${SLOSH_RECORD_ALL}"
    echo "whitelist_topics=${#TOPICS[@]}"
    echo "repo_root=${REPO_ROOT}"
    echo "git_branch=$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
    echo "git_commit=$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || true)"
    echo "git_dirty=$(git -C "${REPO_ROOT}" status --short 2>/dev/null | wc -l || true)"
} > "${INFO_PATH}"

rosparam dump "${PARAM_PATH}" || true
rostopic list | sort > "${TOPIC_PATH}" || true
rosnode list | sort > "${NODE_PATH}" || true

echo "  元数据   = ${BAG_PATH}_{info.txt,rosparam.yaml,topics.txt,nodes.txt}"
echo "============================================"

if [[ "${SLOSH_RECORD_ALL}" == "1" || "${SLOSH_RECORD_ALL}" == "true" || "${SLOSH_RECORD_ALL}" == "TRUE" ]]; then
    rosbag record -a -O "${BAG_PATH}"
else
    rosbag record -O "${BAG_PATH}" "${TOPICS[@]}"
fi
