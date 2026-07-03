#!/usr/bin/env bash
# Full rosbag recorder for real SPMPC smoke / RGB offline analysis.
# Safety: this script only records topics and metadata. It never sends velocity commands,
# never sends goals, never starts/stops planners, and never changes rosparams.

set -euo pipefail

DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VARIANT="${VARIANT:-B_ours}"
RUN_LABEL="${RUN_LABEL:-P0_or_P2}"
RECORD_SEC="${RECORD_SEC:-0}"  # 0 means record until Ctrl+C.
OUT_DIR="${OUT_DIR:-${HOME}/slosh_bags/real/${DATE}_spmpc_full_rgb}"
NAME="${NAME:-spmpc_full_${RUN_LABEL}_${VARIANT}_${STAMP}}"

# Full whitelist by default. Use RECORD_ALL_EXISTING_TOPICS=true only for short
# diagnostic runs when disk space is known to be sufficient.
RECORD_ALL_EXISTING_TOPICS="${RECORD_ALL_EXISTING_TOPICS:-false}"
# RECORD_RGB is the preferred switch. RECORD_CAMERA is kept as a backward-compatible alias.
RECORD_RGB="${RECORD_RGB:-${RECORD_CAMERA:-true}}"
RECORD_CAMERA="${RECORD_CAMERA:-${RECORD_RGB}}"
RECORD_CAMERA_COMPRESSED="${RECORD_CAMERA_COMPRESSED:-false}"
RECORD_DEPTH="${RECORD_DEPTH:-false}"
RECORD_SCAN="${RECORD_SCAN:-true}"
RECORD_STANDALONE_SLOSH="${RECORD_STANDALONE_SLOSH:-true}"
RECORD_ONLINE_LIQUID="${RECORD_ONLINE_LIQUID:-false}"
RECORD_MOCAP="${RECORD_MOCAP:-false}"
RECORD_ROSOUT="${RECORD_ROSOUT:-true}"
RECORD_TOPIC_INFO="${RECORD_TOPIC_INFO:-true}"

# Optional post-record liquid export. Disabled by default: recording remains
# subscribe-only and never launches perception/control unless explicitly asked.
LIQUID_EXPORT_AFTER_RECORD="${LIQUID_EXPORT_AFTER_RECORD:-false}"
LIQUID_EXPORT_SOURCE="${LIQUID_EXPORT_SOURCE:-rgb}"
LIQUID_CALIBRATION="${LIQUID_CALIBRATION:-}"
LIQUID_EXPORT_OUT_DIR="${LIQUID_EXPORT_OUT_DIR:-${OUT_DIR}/${NAME}_liquid_variation}"
LIQUID_IMAGE_TOPIC="${LIQUID_IMAGE_TOPIC:-/camera/color/image_raw}"
LIQUID_ZERO_CORRECTION_FRAMES="${LIQUID_ZERO_CORRECTION_FRAMES:-30}"
LIQUID_SMOOTH_FRAMES="${LIQUID_SMOOTH_FRAMES:-5}"
LIQUID_DEBUG_EVERY="${LIQUID_DEBUG_EVERY:-30}"
LIQUID_HUE1_LOW="${LIQUID_HUE1_LOW:-0}"
LIQUID_HUE1_HIGH="${LIQUID_HUE1_HIGH:-11}"
LIQUID_HUE2_LOW="${LIQUID_HUE2_LOW:-173}"
LIQUID_HUE2_HIGH="${LIQUID_HUE2_HIGH:-179}"
LIQUID_SAT_MIN="${LIQUID_SAT_MIN:-80}"
LIQUID_VAL_MIN="${LIQUID_VAL_MIN:-162}"

MOCAP_TRACKER="${MOCAP_TRACKER:-Scout}"

# Optional run-intent metadata. These values do not affect recording; they make
# the bag self-describing when the launch command is run in another terminal.
SOLVER_BACKEND="${SOLVER_BACKEND:-}"
V_REF="${V_REF:-}"
W_SLOSH="${W_SLOSH:-}"
SLOSH_HEIGHT_MAX="${SLOSH_HEIGHT_MAX:-}"
DELAY_PHASE_MODE="${DELAY_PHASE_MODE:-}"
DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC:-}"
DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC:-}"
CONTROL_FREQUENCY="${CONTROL_FREQUENCY:-}"
MAP_FILE="${MAP_FILE:-}"
GOAL_X="${GOAL_X:-}"
GOAL_Y="${GOAL_Y:-}"
GOAL_YAW="${GOAL_YAW:-}"
LAUNCH_COMMAND="${LAUNCH_COMMAND:-}"
OPERATOR_NOTE="${OPERATOR_NOTE:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || pwd)"
mkdir -p "${OUT_DIR}"

BAG_PATH="${OUT_DIR}/${NAME}.bag"
INFO_PATH="${OUT_DIR}/${NAME}_info.txt"
PARAM_PATH="${OUT_DIR}/${NAME}_rosparam.yaml"
TOPIC_PATH="${OUT_DIR}/${NAME}_topics.txt"
NODE_PATH="${OUT_DIR}/${NAME}_nodes.txt"
CMD_INFO_PATH="${OUT_DIR}/${NAME}_cmd_vel_info.txt"
SELECTED_TOPIC_PATH="${OUT_DIR}/${NAME}_selected_topics.txt"
EXISTING_SELECTED_TOPIC_PATH="${OUT_DIR}/${NAME}_existing_selected_topics_at_start.txt"
MISSING_TOPIC_PATH="${OUT_DIR}/${NAME}_missing_selected_topics_at_start.txt"
BAG_INFO_PATH="${OUT_DIR}/${NAME}_bag_info.txt"
RECORDED_TOPIC_PATH="${OUT_DIR}/${NAME}_recorded_topics.txt"
NOT_RECORDED_TOPIC_PATH="${OUT_DIR}/${NAME}_selected_topics_not_recorded.txt"
ENV_PATH="${OUT_DIR}/${NAME}_env.txt"
TOPIC_INFO_DIR="${OUT_DIR}/${NAME}_topic_info"

if ! rostopic list >/dev/null 2>&1; then
  echo "[record_spmpc_full_rgb_bag] ERROR: roscore not reachable. Start the real sensor stack first." >&2
  exit 1
fi

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

write_selected_topic_snapshot() {
  local existing_count=0
  local missing_count=0
  : > "${EXISTING_SELECTED_TOPIC_PATH}"
  : > "${MISSING_TOPIC_PATH}"
  for topic in "${record_topics[@]}"; do
    if grep -Fxq -- "${topic}" "${TOPIC_PATH}"; then
      printf '%s\n' "${topic}" >> "${EXISTING_SELECTED_TOPIC_PATH}"
      existing_count=$((existing_count + 1))
    else
      printf '%s\n' "${topic}" >> "${MISSING_TOPIC_PATH}"
      missing_count=$((missing_count + 1))
    fi
  done
  SELECTED_EXISTING_COUNT="${existing_count}"
  SELECTED_MISSING_COUNT="${missing_count}"
}

write_topic_info_snapshot() {
  truthy "${RECORD_TOPIC_INFO}" || return 0
  mkdir -p "${TOPIC_INFO_DIR}"
  local topic safe
  for topic in "${record_topics[@]}"; do
    if grep -Fxq -- "${topic}" "${TOPIC_PATH}"; then
      safe="${topic#/}"
      safe="${safe//\//__}"
      rostopic info "${topic}" > "${TOPIC_INFO_DIR}/${safe}.txt" 2>&1 || true
    fi
  done
}

record_topics=(
  # Control / robot state. Recording /cmd_vel is required for analysis; this
  # script never sends it.
  /cmd_vel
  /spmpc_shadow_cmd_vel
  /cmd_vel_drive
  /odom
  /scout/odom
  /scout/cmd_vel
  /scout_status
  /rs_status
  /joint_states
  /battery_state
  /tf
  /tf_static

  # Localization / map / environment evidence.
  /map
  /map_updates
  /tracked_pose
  /submap_list
  /trajectory_node_list
  /scan_matched_points2
  /constraint_list
  /landmark_poses_list
  /amcl_pose
  /particlecloud

  # Fixed-path / goal evidence.
  /scout/goal
  /scout/current_goal
  /scout/global_path
  /scout/global_path_fixed
  /scout/global_path_smooth
  /local_path
  /mpc/reference_path

  # SPMPC mainline status / solver / trajectory.
  /spmpc/status
  /spmpc/solver_backend
  /spmpc/controller_variant
  /spmpc/experiment_mode
  /spmpc/solver_time_ms
  /spmpc/solve_ms
  /spmpc/solve_time_ms
  /spmpc/debug/effective_config
  /spmpc/cost_breakdown
  /spmpc/local_trajectory
  /spmpc/corridor
  /spmpc/guidance
  /spmpc/primitive

  # SPMPC slosh model and debug topics.
  /spmpc/slosh_height
  /spmpc/slosh_horizon_summary
  /spmpc/debug/slosh_state
  /spmpc/debug/slosh_cost_monitor
  /spmpc/debug/slosh_hard_constraint
  /spmpc/debug/slosh_hard_constraint_effective
  /spmpc/debug/progress_s
  /spmpc/debug/v_ref_current
  /spmpc/debug/map_vref_status
  /spmpc/debug/runtime_bounds
  /spmpc/debug/generated_bounds
  /spmpc/debug/first_shot_summary
  /spmpc/debug/projector
  /spmpc/debug/stage0_reference
  /spmpc/debug/local_traj_head
  /spmpc/debug/warm_start_head
  /spmpc/debug/cmd_vel_output
  /spmpc/debug/cmd_vel_output_status
  # Future/optional black-box diagnostics. Recording missing topics is harmless;
  # the post-record report lists which ones were actually present in the bag.
  /spmpc/debug/timing_budget
  /spmpc/debug/command_intervention
  /spmpc/debug/slosh_model_consistency
  /spmpc/debug/solver_input_state
  /spmpc/debug/raw_state
  /spmpc/debug/predicted_state
  /spmpc/debug/delay_phase
  /spmpc/debug/odom_timing
  /spmpc/debug/execution_state
  /spmpc/debug/execution_alignment_status
  /spmpc/debug/delay_compensation
  /spmpc/debug/cmd_odom_alignment

  # SPMPC safety modes.
  /spmpc/start_lock/active
  /spmpc/start_lock/mode
  /spmpc/start_lock/debug
  /spmpc/terminal/mode
  /spmpc/terminal/debug

  # IMU / inertial inputs used for post-analysis and fault diagnosis.
  /imu/data
  /container_imu
  /wit/mag
  /camera/gyro/sample
  /camera/accel/sample
  /camera/imu

  # Reference/profile cap diagnostics from the older slosh analysis chain.
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

  # Legacy MPC/slosh topics: useful when comparing against older Route A bags.
  /mpc/solve_ms
  /mpc/solve_time_ms
  /mpc/status_val
  /mpc_status
  /mpc/cost_breakdown
  /mpc/slosh_horizon_summary

  # Terminal / MBF topics are recorded if present; recorder does not invoke them.
  /terminal/mode
  /terminal/recovery_latched
  /terminal/goal_info
  /terminal/v_envelope
  /terminal/envelope_active
  /terminal/phase_active
  /terminal/cmd_v_pre_clamp
  /terminal/cmd_v_post_clamp
  /scout/move_base_cmd_vel
  /scout/mbf_costmap_nav/current_goal
  /scout/mbf_costmap_nav/GlobalPlanner/plan
  /scout/mbf_costmap_nav/GlobalPlanner/potential
  /scout/mbf_costmap_nav/get_path/goal
  /scout/mbf_costmap_nav/get_path/feedback
  /scout/mbf_costmap_nav/get_path/result
  /scout/mbf_costmap_nav/get_path/status
  /scout/mbf_costmap_nav/exe_path/feedback
  /scout/mbf_costmap_nav/exe_path/result
  /scout/mbf_costmap_nav/exe_path/status
  /scout/mbf_costmap_nav/move_base/feedback
  /scout/mbf_costmap_nav/move_base/result
  /scout/mbf_costmap_nav/move_base/status
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
)

if truthy "${RECORD_SCAN}"; then
  record_topics+=(/scan_front)
fi

if truthy "${RECORD_STANDALONE_SLOSH}"; then
  record_topics+=(
    /slosh/state
    /slosh/height
    /slosh/height_pred_max
    /slosh/q_slosh_eta
    /slosh/eta_norm
    /slosh/eta_dot_norm
    /slosh/modal_energy
    /slosh/modal_energy_norm
    /slosh/excitation_ay_abs
    /slosh/excitation_alpha_abs
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
    /slosh/debug
    /slosh/h_visual
    /slosh/h_visual_quality
  )
fi

if truthy "${RECORD_CAMERA}"; then
  record_topics+=(
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/color/metadata
  )
fi

if truthy "${RECORD_CAMERA_COMPRESSED}"; then
  record_topics+=(
    /camera/color/image_raw/compressed
    /camera/color/image_raw/compressed/parameter_descriptions
    /camera/color/image_raw/compressed/parameter_updates
  )
fi

if truthy "${RECORD_DEPTH}"; then
  record_topics+=(
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
  )
fi

if truthy "${RECORD_ONLINE_LIQUID}"; then
  record_topics+=(
    /liquid/height
    /liquid/height_lcr
    /liquid/height_median
    /liquid/debug_image
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
  )
fi

if truthy "${RECORD_MOCAP}"; then
  record_topics+=(
    "/vrpn_client_node/${MOCAP_TRACKER}/pose"
    /mocap/scout_pose
    /mocap/scout_odom
    /mocap/scout_path
    /mocap/status
  )
fi

if truthy "${RECORD_ROSOUT}"; then
  record_topics+=(/rosout /diagnostics /diagnostics_agg)
fi

# Keep deterministic order and remove duplicates without relying on associative arrays.
mapfile -t record_topics < <(printf '%s\n' "${record_topics[@]}" | awk 'NF && !seen[$0]++')
printf '%s\n' "${record_topics[@]}" > "${SELECTED_TOPIC_PATH}"

# Snapshot the system before recording. Missing selected topics are not fatal: this
# recorder is often started before the planner, and post-record bag_info below is
# the final source of truth for what was actually captured.
env | sort > "${ENV_PATH}" || true
rosparam dump "${PARAM_PATH}" >/dev/null 2>&1 || true
rostopic list | sort > "${TOPIC_PATH}" || :
touch "${TOPIC_PATH}"
rosnode list | sort > "${NODE_PATH}" || true
rostopic info /cmd_vel > "${CMD_INFO_PATH}" 2>&1 || true
write_selected_topic_snapshot
write_topic_info_snapshot

{
  echo "bag=${BAG_PATH}"
  echo "date=${DATE}"
  echo "stamp=${STAMP}"
  echo "variant=${VARIANT}"
  echo "run_label=${RUN_LABEL}"
  echo "record_sec=${RECORD_SEC}"
  echo "record_all_existing_topics=${RECORD_ALL_EXISTING_TOPICS}"
  echo "record_rgb=${RECORD_RGB}"
  echo "record_camera=${RECORD_CAMERA}"
  echo "record_camera_compressed=${RECORD_CAMERA_COMPRESSED}"
  echo "record_depth=${RECORD_DEPTH}"
  echo "record_scan=${RECORD_SCAN}"
  echo "record_standalone_slosh=${RECORD_STANDALONE_SLOSH}"
  echo "record_online_liquid=${RECORD_ONLINE_LIQUID}"
  echo "record_mocap=${RECORD_MOCAP}"
  echo "record_rosout=${RECORD_ROSOUT}"
  echo "record_topic_info=${RECORD_TOPIC_INFO}"
  echo "liquid_export_after_record=${LIQUID_EXPORT_AFTER_RECORD}"
  echo "liquid_export_source=${LIQUID_EXPORT_SOURCE}"
  echo "liquid_calibration=${LIQUID_CALIBRATION}"
  echo "liquid_export_out_dir=${LIQUID_EXPORT_OUT_DIR}"
  echo "liquid_image_topic=${LIQUID_IMAGE_TOPIC}"
  echo "liquid_zero_correction_frames=${LIQUID_ZERO_CORRECTION_FRAMES}"
  echo "liquid_smooth_frames=${LIQUID_SMOOTH_FRAMES}"
  echo "mocap_tracker=${MOCAP_TRACKER}"
  echo "topic_count=${#record_topics[@]}"
  echo "selected_existing_at_start=${SELECTED_EXISTING_COUNT}"
  echo "selected_missing_at_start=${SELECTED_MISSING_COUNT}"
  echo "solver_backend=${SOLVER_BACKEND}"
  echo "v_ref=${V_REF}"
  echo "w_slosh=${W_SLOSH}"
  echo "slosh_height_max=${SLOSH_HEIGHT_MAX}"
  echo "delay_phase_mode=${DELAY_PHASE_MODE}"
  echo "delay_phase_linear_delay_sec=${DELAY_PHASE_LINEAR_DELAY_SEC}"
  echo "delay_phase_angular_delay_sec=${DELAY_PHASE_ANGULAR_DELAY_SEC}"
  echo "control_frequency=${CONTROL_FREQUENCY}"
  echo "map_file=${MAP_FILE}"
  echo "goal_x=${GOAL_X}"
  echo "goal_y=${GOAL_Y}"
  echo "goal_yaw=${GOAL_YAW}"
  echo "launch_command=${LAUNCH_COMMAND}"
  echo "operator_note=${OPERATOR_NOTE}"
  echo "repo_root=${REPO_ROOT}"
  echo "git_branch=$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
  echo "git_commit=$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || true)"
  echo "git_dirty=$(git -C "${REPO_ROOT}" status --short 2>/dev/null | wc -l || true)"
} > "${INFO_PATH}"

cat <<EOF
============================================
  SPMPC full RGB rosbag recorder
============================================
  output       = ${BAG_PATH}
  variant      = ${VARIANT}
  run_label    = ${RUN_LABEL}
  record_sec   = ${RECORD_SEC} (0 means Ctrl+C)
  record_all   = ${RECORD_ALL_EXISTING_TOPICS}
  topics       = ${#record_topics[@]} whitelist topics
  at start     = ${SELECTED_EXISTING_COUNT} selected topics exist, ${SELECTED_MISSING_COUNT} missing/not yet advertised
  camera RGB   = ${RECORD_CAMERA}
  scan_front   = ${RECORD_SCAN}
  depth        = ${RECORD_DEPTH}
  liquid proxy = ${RECORD_ONLINE_LIQUID}
  liquid export= ${LIQUID_EXPORT_AFTER_RECORD} (${LIQUID_EXPORT_SOURCE})
  mocap        = ${RECORD_MOCAP}
  metadata     = ${OUT_DIR}/${NAME}_{info.txt,rosparam.yaml,topics.txt,nodes.txt,selected_topics.txt,*topics*.txt,bag_info.txt}
============================================
  Start this recorder BEFORE launching SPMPC to capture startup transients.
  This script records /cmd_vel but never sends it.
============================================
EOF

if truthy "${RECORD_ALL_EXISTING_TOPICS}"; then
  record_cmd=(rosbag record -a -O "${BAG_PATH}")
else
  record_cmd=(rosbag record -O "${BAG_PATH}" "${record_topics[@]}")
fi

set +e
if [[ "${RECORD_SEC}" == "0" ]]; then
  "${record_cmd[@]}"
  code=$?
else
  timeout --signal=INT --kill-after=5s "${RECORD_SEC}s" "${record_cmd[@]}"
  code=$?
fi
set -e
if [[ ${code} -ne 0 && ${code} -ne 124 && ${code} -ne 130 ]]; then
  echo "[record_spmpc_full_rgb_bag] ERROR: rosbag record failed with exit code ${code}" >&2
  exit "${code}"
fi

echo "[record_spmpc_full_rgb_bag] OK: bag saved: ${BAG_PATH}"

if [[ -f "${BAG_PATH}" ]]; then
  rosbag info "${BAG_PATH}" > "${BAG_INFO_PATH}" 2>&1 || true
  python3 - "${BAG_PATH}" "${RECORDED_TOPIC_PATH}" "${SELECTED_TOPIC_PATH}" "${NOT_RECORDED_TOPIC_PATH}" <<'PY' || true
import sys
try:
    import rosbag
except Exception as exc:
    print(f"rosbag import failed: {exc}", file=sys.stderr)
    sys.exit(0)

bag_path, recorded_path, selected_path, missing_path = sys.argv[1:5]
with rosbag.Bag(bag_path) as bag:
    recorded = sorted(bag.get_type_and_topic_info().topics.keys())
with open(recorded_path, "w", encoding="utf-8") as f:
    for topic in recorded:
        f.write(topic + "\n")
recorded_set = set(recorded)
with open(selected_path, "r", encoding="utf-8") as f:
    selected = [line.strip() for line in f if line.strip()]
with open(missing_path, "w", encoding="utf-8") as f:
    for topic in selected:
        if topic not in recorded_set:
            f.write(topic + "\n")
PY
  echo "[record_spmpc_full_rgb_bag] metadata: ${BAG_INFO_PATH}"
  echo "[record_spmpc_full_rgb_bag] recorded topics: ${RECORDED_TOPIC_PATH}"
  echo "[record_spmpc_full_rgb_bag] selected-but-not-recorded topics: ${NOT_RECORDED_TOPIC_PATH}"
fi

if truthy "${LIQUID_EXPORT_AFTER_RECORD}"; then
  LIQUID_EXPORT_SCRIPT="${LIQUID_EXPORT_SCRIPT:-${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/export_liquid_variation_from_bags.py}"
  if [[ ! -x "${LIQUID_EXPORT_SCRIPT}" ]]; then
    echo "[record_spmpc_full_rgb_bag] ERROR: liquid export script not executable: ${LIQUID_EXPORT_SCRIPT}" >&2
    exit 20
  fi
  if [[ "${LIQUID_EXPORT_SOURCE}" != "online" && -z "${LIQUID_CALIBRATION}" ]]; then
    echo "[record_spmpc_full_rgb_bag] ERROR: LIQUID_CALIBRATION is required when LIQUID_EXPORT_SOURCE=${LIQUID_EXPORT_SOURCE}" >&2
    exit 21
  fi
  echo "[record_spmpc_full_rgb_bag] running liquid export -> ${LIQUID_EXPORT_OUT_DIR}"
  python3 "${LIQUID_EXPORT_SCRIPT}" "${BAG_PATH}" \
    --source "${LIQUID_EXPORT_SOURCE}" \
    --calibration "${LIQUID_CALIBRATION}" \
    --topic "${LIQUID_IMAGE_TOPIC}" \
    --out-dir "${LIQUID_EXPORT_OUT_DIR}" \
    --zero-correction-frames "${LIQUID_ZERO_CORRECTION_FRAMES}" \
    --smooth-frames "${LIQUID_SMOOTH_FRAMES}" \
    --debug-every "${LIQUID_DEBUG_EVERY}" \
    --hue1-low "${LIQUID_HUE1_LOW}" \
    --hue1-high "${LIQUID_HUE1_HIGH}" \
    --hue2-low "${LIQUID_HUE2_LOW}" \
    --hue2-high "${LIQUID_HUE2_HIGH}" \
    --sat-min "${LIQUID_SAT_MIN}" \
    --val-min "${LIQUID_VAL_MIN}"
  echo "[record_spmpc_full_rgb_bag] OK: liquid export saved under ${LIQUID_EXPORT_OUT_DIR}"
fi
