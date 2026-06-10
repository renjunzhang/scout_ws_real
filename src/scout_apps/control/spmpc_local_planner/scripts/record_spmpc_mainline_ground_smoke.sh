#!/usr/bin/env bash
# Lightweight rosbag recorder for SPMPC slosh-aware mainline ground smoke.
# This script only records selected topics; it does not publish /cmd_vel or start/stop SPMPC.

set -euo pipefail

DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VARIANT="${VARIANT:-B_ours}"
SOLVER_BACKEND="${SOLVER_BACKEND:-continuous_mpcc_acados}"
RECORD_SEC="${RECORD_SEC:-20}"  # 0 means record until Ctrl+C.
OUT_DIR="${OUT_DIR:-${HOME}/slosh_bags/real/${DATE}_spmpc_mainline_ground}"
NAME="${NAME:-spmpc_${VARIANT}_acados_ground_${STAMP}}"

# Fixed target used by the 2026-06-10 operator-override smoke.
GOAL_X="${GOAL_X:-7.164488315582275}"
GOAL_Y="${GOAL_Y:-9.307367324829102}"
GOAL_YAW="${GOAL_YAW:-1.0808}"

# Keep the default bag small. Enable these only when that signal is needed.
RECORD_CAMERA="${RECORD_CAMERA:-false}"
RECORD_SCAN="${RECORD_SCAN:-false}"
RECORD_ROSOUT="${RECORD_ROSOUT:-true}"
RECORD_STANDALONE_SLOSH="${RECORD_STANDALONE_SLOSH:-true}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || pwd)"
mkdir -p "${OUT_DIR}"

BAG_PATH="${OUT_DIR}/${NAME}.bag"
INFO_PATH="${OUT_DIR}/${NAME}_info.txt"
PARAM_PATH="${OUT_DIR}/${NAME}_rosparam.yaml"
TOPIC_PATH="${OUT_DIR}/${NAME}_topics.txt"
NODE_PATH="${OUT_DIR}/${NAME}_nodes.txt"
CMD_INFO_PATH="${OUT_DIR}/${NAME}_cmd_vel_info.txt"

if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] 未检测到 roscore。先启动实物传感器栈，再运行本脚本。" >&2
  exit 1
fi

record_topics=(
  /cmd_vel
  /odom
  /tf
  /tf_static
  /map
  /scout/goal
  /scout/global_path_fixed

  /spmpc/status
  /spmpc/solver_backend
  /spmpc/controller_variant
  /spmpc/experiment_mode
  /spmpc/solver_time_ms
  /spmpc/cost_breakdown
  /spmpc/local_trajectory

  /spmpc/slosh_height
  /spmpc/slosh_horizon_summary
  /spmpc/debug/slosh_state
  /spmpc/debug/progress_s
  /spmpc/debug/runtime_bounds
  /spmpc/debug/generated_bounds
  /spmpc/debug/first_shot_summary
  /spmpc/debug/projector
  /spmpc/debug/stage0_reference
  /spmpc/debug/local_traj_head
  /spmpc/debug/warm_start_head

  /spmpc/start_lock/active
  /spmpc/start_lock/mode
  /spmpc/start_lock/debug
  /spmpc/terminal/mode
  /spmpc/terminal/debug
)

if [[ "${RECORD_STANDALONE_SLOSH}" == "true" || "${RECORD_STANDALONE_SLOSH}" == "1" ]]; then
  record_topics+=(
    /slosh/height
    /slosh/state
    /slosh/debug
  )
fi

if [[ "${RECORD_CAMERA}" == "true" || "${RECORD_CAMERA}" == "1" ]]; then
  record_topics+=(
    /camera/color/image_raw
    /camera/color/camera_info
  )
fi

if [[ "${RECORD_SCAN}" == "true" || "${RECORD_SCAN}" == "1" ]]; then
  record_topics+=(/scan_front)
fi

if [[ "${RECORD_ROSOUT}" == "true" || "${RECORD_ROSOUT}" == "1" ]]; then
  record_topics+=(/rosout /diagnostics /diagnostics_agg)
fi

{
  echo "bag=${BAG_PATH}"
  echo "date=${DATE}"
  echo "stamp=${STAMP}"
  echo "variant=${VARIANT}"
  echo "solver_backend=${SOLVER_BACKEND}"
  echo "record_sec=${RECORD_SEC}"
  echo "goal_x=${GOAL_X}"
  echo "goal_y=${GOAL_Y}"
  echo "goal_yaw=${GOAL_YAW}"
  echo "record_camera=${RECORD_CAMERA}"
  echo "record_scan=${RECORD_SCAN}"
  echo "record_rosout=${RECORD_ROSOUT}"
  echo "record_standalone_slosh=${RECORD_STANDALONE_SLOSH}"
  echo "topic_count=${#record_topics[@]}"
  echo "repo_root=${REPO_ROOT}"
  echo "git_branch=$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
  echo "git_commit=$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || true)"
  echo "git_dirty=$(git -C "${REPO_ROOT}" status --short 2>/dev/null | wc -l || true)"
} > "${INFO_PATH}"

rosparam dump "${PARAM_PATH}" >/dev/null 2>&1 || true
rostopic list | sort > "${TOPIC_PATH}" || true
rosnode list | sort > "${NODE_PATH}" || true
rostopic info /cmd_vel > "${CMD_INFO_PATH}" 2>&1 || true

cat <<EOF
============================================
  SPMPC mainline ground smoke recorder
============================================
  variant        = ${VARIANT}
  solver_backend = ${SOLVER_BACKEND}
  record_sec     = ${RECORD_SEC} (0 means Ctrl+C)
  output         = ${BAG_PATH}
  metadata       = ${OUT_DIR}/${NAME}_{info.txt,rosparam.yaml,topics.txt,nodes.txt,cmd_vel_info.txt}
  topics         = ${#record_topics[@]}
  camera         = ${RECORD_CAMERA}
  scan           = ${RECORD_SCAN}
============================================
  建议：先启动本脚本，再启动 spmpc_fixed_path.launch，以捕捉起步瞬间。
  本脚本不发 /cmd_vel，不启动 planner，只录关键 /spmpc/* 与闭环状态。
============================================
EOF

printf '  - %s\n' "${record_topics[@]}"

if [[ "${RECORD_SEC}" == "0" ]]; then
  rosbag record -O "${BAG_PATH}" "${record_topics[@]}"
else
  set +e
  timeout --signal=INT --kill-after=5s "${RECORD_SEC}s" \
    rosbag record -O "${BAG_PATH}" "${record_topics[@]}"
  code=$?
  set -e
  if [[ ${code} -ne 0 && ${code} -ne 124 && ${code} -ne 130 ]]; then
    echo "[ERR] rosbag record failed with exit code ${code}" >&2
    exit "${code}"
  fi
fi

echo "[OK] bag saved: ${BAG_PATH}"
