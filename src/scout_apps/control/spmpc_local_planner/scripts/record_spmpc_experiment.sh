#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${OUT_DIR:-/tmp/spmpc_bags}"
NAME="${NAME:-spmpc_smoke}"
RECORD_CAMERA="${RECORD_CAMERA:-true}"                 # RGB 原始图像只用于离线真值/复查，不是 planner 输入
RECORD_ONLINE_LIQUID="${RECORD_ONLINE_LIQUID:-false}"  # 在线 /liquid/* 是 realsense_liquid_measurement 调试 proxy，默认不录
RECORD_MOCAP="${RECORD_MOCAP:-false}"                  # Nokov 动捕只作为监控/真值记录，不是 planner 输入
MOCAP_TRACKER="${MOCAP_TRACKER:-Scout}"
mkdir -p "$OUT_DIR"

record_topics=(
  /spmpc/status
  /spmpc/controller_variant
  /spmpc/experiment_mode
  /spmpc/local_trajectory
  /spmpc/debug/progress_s
  /spmpc/debug/slosh_state
  /spmpc/slosh_horizon_summary
  /spmpc/corridor
  /spmpc/guidance
  /spmpc/primitive
  /spmpc/solver_time_ms
  /spmpc/cost_breakdown
  /cmd_vel
  /odom
  /scout/global_path
  /scout/global_path_fixed
  /map
  /slosh/height
  /slosh/height_pred_max
  /slosh/eta_x
  /slosh/eta_y
  /slosh/eta_x_dot
  /slosh/eta_y_dot
  /tf
  /tf_static
)

if [[ "${RECORD_CAMERA}" == "true" ]]; then
  record_topics+=(/camera/color/image_raw /camera/color/camera_info)
fi
if [[ "${RECORD_ONLINE_LIQUID}" == "true" ]]; then
  record_topics+=(/liquid/height /liquid/height_lcr /liquid/height_median)
fi
if [[ "${RECORD_MOCAP}" == "true" || "${RECORD_MOCAP}" == "1" ]]; then
  record_topics+=(
    "/vrpn_client_node/${MOCAP_TRACKER}/pose"
    /mocap/scout_pose
    /mocap/scout_odom
    /mocap/scout_path
    /mocap/status
  )
fi

rosbag record -O "${OUT_DIR}/${NAME}.bag" "${record_topics[@]}"
