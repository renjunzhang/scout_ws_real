#!/usr/bin/env bash
# Phase 4 fixed-path experiment runner.
#
# 目的:
#   用同一固定路径和同一录包口径跑 B0 / B_slosh / B_smooth / B_ours
#   以及 B_slosh_linear / B_slosh_anti / B_ours_anti 等消融。
#
# 前提:
#   1. 实物传感器、定位、底盘和相机已经启动；
#   2. 本脚本只负责固定路径生成/目标发送、启动 spmpc 和录包；
#   3. 每次切换 variant 前应人工复位到同一起点，并等待液体状态稳定。
#
# 示例:
#   source /opt/ros/noetic/setup.bash
#   source /home/geist/scout_ws/devel/setup.bash
#   cd /home/geist/scout_ws
#   VARIANT=B_slosh_anti OUT_DIR=/home/geist/slosh_bags/real/20260602_spmpc_phase4 \
#     GOAL_X=7.164488315582275 GOAL_Y=9.307367324829102 GOAL_YAW=1.0808 \
#     bash src/scout_apps/control/spmpc_local_planner/scripts/phase4_fixed_path_run.sh

set -euo pipefail

VARIANT="${VARIANT:-B0}"
OUT_DIR="${OUT_DIR:-/data/${USER}/spmpc_phase4_fixed_path}"
RECORD_SEC="${RECORD_SEC:-45}"
REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
GOAL_X="${GOAL_X:--1.2}"
GOAL_Y="${GOAL_Y:-2.6}"
GOAL_YAW="${GOAL_YAW:-1.0}"
PATH_TEMPLATE="${PATH_TEMPLATE:-s_curve}"
PATH_FILE="${PATH_FILE:-${OUT_DIR}/paths/P2_s_curve_spmpc_phase4.json}"
PATH_SPACING="${PATH_SPACING:-0.05}"
PATH_AMPLITUDE_RATIO="${PATH_AMPLITUDE_RATIO:-0.18}"
PATH_MIN_AMPLITUDE="${PATH_MIN_AMPLITUDE:-0.25}"
PATH_MAX_AMPLITUDE="${PATH_MAX_AMPLITUDE:-1.20}"
START_HEADING="${START_HEADING:-current}"
RECORD_CAMERA="${RECORD_CAMERA:-true}"
RECORD_MOCAP="${RECORD_MOCAP:-false}"       # Nokov 动捕只作为监控/真值记录，不是 planner 输入
MOCAP_TRACKER="${MOCAP_TRACKER:-Scout}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)_${VARIANT}}"

generator_pid=""
planner_pid=""
rec_pid=""

cleanup() {
  if [[ -n "${rec_pid}" ]]; then
    kill -INT "$rec_pid" 2>/dev/null || true
    wait "$rec_pid" 2>/dev/null || true
  fi
  if [[ -n "${planner_pid}" ]]; then
    kill -INT "$planner_pid" 2>/dev/null || true
    wait "$planner_pid" 2>/dev/null || true
  fi
  if [[ -n "${generator_pid}" ]]; then
    kill -INT "$generator_pid" 2>/dev/null || true
    wait "$generator_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

wait_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  if ! timeout "${timeout_sec}s" rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "[ERR] ${timeout_sec}s 内没有收到 ${topic}" >&2
    return 1
  fi
}

wait_topic_header() {
  local topic="$1"
  local timeout_sec="$2"
  if ! timeout "${timeout_sec}s" rostopic echo -n 1 "${topic}/header" >/dev/null 2>&1; then
    echo "[ERR] ${timeout_sec}s 内没有收到 ${topic}" >&2
    return 1
  fi
}

first_status() {
  timeout 3s rostopic echo -n 1 /spmpc/status 2>/dev/null | awk -F'"' '/data:/ {print $2; exit}'
}

first_cmd_v() {
  timeout 3s rostopic echo -n 1 /cmd_vel 2>/dev/null | awk '/x:/ {print $2; exit}'
}

wait_planner_ready() {
  local timeout_sec="$1"
  local start
  start="$(date +%s)"
  while true; do
    local status
    status="$(first_status || true)"
    case "${status:-}" in
      ""|INITIALIZED|WAITING_FOR_ODOM|WAITING_FOR_REFERENCE_PATH|WAITING_FOR_TF_POSE)
        ;;
      *)
        echo "$status"
        return 0
        ;;
    esac
    if (( $(date +%s) - start >= timeout_sec )); then
      echo "${status:-NO_STATUS}"
      return 1
    fi
    sleep 0.5
  done
}

is_near_zero() {
  awk -v x="${1:-999}" 'BEGIN { if (x < 0) x = -x; exit !(x < 0.001) }'
}

mkdir -p "$OUT_DIR" "$(dirname "$PATH_FILE")"

if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/实物栈未检测到。请先启动 sensors/localization/base。" >&2
  exit 1
fi

if [[ "${RECORD_CAMERA}" == "true" ]]; then
  echo "[camera] 等待 /camera/color/image_raw ..."
  wait_topic_once /camera/color/image_raw 10
fi

echo "================ Phase4 fixed_path variant=${VARIANT} run=${RUN_ID} ================"
echo "[path] 启动固定路径生成器: output=${REF_TOPIC}, file=${PATH_FILE}"
rosrun scout_local_planner template_fixed_path_generator.py \
  --template "${PATH_TEMPLATE}" \
  --goal-topic "${GOAL_TOPIC}" \
  --output-topic "${REF_TOPIC}" \
  --path-file "${PATH_FILE}" \
  --start-heading "${START_HEADING}" \
  --spacing "${PATH_SPACING}" \
  --amplitude-ratio "${PATH_AMPLITUDE_RATIO}" \
  --min-amplitude "${PATH_MIN_AMPLITUDE}" \
  --max-amplitude "${PATH_MAX_AMPLITUDE}" \
  --publish-count 0 \
  >"${OUT_DIR}/${RUN_ID}_path_generator.log" 2>&1 &
generator_pid=$!
sleep 2

echo "[goal] 发送目标: x=${GOAL_X}, y=${GOAL_Y}, yaw=${GOAL_YAW}"
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic "${GOAL_TOPIC}" \
  --x "${GOAL_X}" \
  --y "${GOAL_Y}" \
  --yaw "${GOAL_YAW}" \
  --repeat-count 1 \
  --repeat-rate 1 \
  >"${OUT_DIR}/${RUN_ID}_send_goal.log" 2>&1

echo "[path] 等待 ${REF_TOPIC} ..."
wait_topic_header "${REF_TOPIC}" 15

echo "[planner] 启动 spmpc_fixed_path.launch"
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:="${VARIANT}" \
  reference_path_topic:="${REF_TOPIC}" \
  >"${OUT_DIR}/${RUN_ID}_planner.log" 2>&1 &
planner_pid=$!
sleep 3

if ! status="$(wait_planner_ready 12)"; then
  echo "[ERR] planner 12s 内没有进入有效求解状态, 最后 status=${status:-NA}" >&2
  echo "      查看 ${OUT_DIR}/${RUN_ID}_planner.log" >&2
  exit 3
fi
cmd_v="$(first_cmd_v || true)"
echo "[preflight] status=${status:-NA}, cmd_v=${cmd_v:-NA}"
if [[ "${status}" == "GOAL_REACHED" ]] && is_near_zero "${cmd_v:-0}"; then
  echo "[ERR] planner 启动后立即 GOAL_REACHED 且 cmd_v≈0。" >&2
  echo "      通常表示车辆未复位到固定起点，或当前已在目标附近。" >&2
  exit 2
fi

meta="${OUT_DIR}/${RUN_ID}_meta.yaml"
git_hash="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
cat >"$meta" <<EOF
run_id: ${RUN_ID}
variant: ${VARIANT}
git_hash: ${git_hash}
record_sec: ${RECORD_SEC}
reference_topic: ${REF_TOPIC}
goal:
  x: ${GOAL_X}
  y: ${GOAL_Y}
  yaw: ${GOAL_YAW}
path:
  template: ${PATH_TEMPLATE}
  file: ${PATH_FILE}
  spacing: ${PATH_SPACING}
  amplitude_ratio: ${PATH_AMPLITUDE_RATIO}
  min_amplitude: ${PATH_MIN_AMPLITUDE}
  max_amplitude: ${PATH_MAX_AMPLITUDE}
  start_heading: ${START_HEADING}
record_camera: ${RECORD_CAMERA}
record_mocap: ${RECORD_MOCAP}
mocap_tracker: ${MOCAP_TRACKER}
mocap_raw_pose_topic: /vrpn_client_node/${MOCAP_TRACKER}/pose
mocap_monitor_topics:
  pose: /mocap/scout_pose
  odom: /mocap/scout_odom
  path: /mocap/scout_path
  status: /mocap/status
EOF

record_topics=(
  /spmpc/status
  /spmpc/controller_variant
  /spmpc/experiment_mode
  /spmpc/local_trajectory
  /spmpc/cost_breakdown
  /spmpc/slosh_horizon_summary
  /spmpc/corridor
  /spmpc/guidance
  /spmpc/primitive
  /spmpc/debug/slosh_state
  /spmpc/debug/progress_s
  /spmpc/solver_time_ms
  /cmd_vel
  /odom
  /scout/goal
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
if [[ "${RECORD_MOCAP}" == "true" || "${RECORD_MOCAP}" == "1" ]]; then
  record_topics+=(
    "/vrpn_client_node/${MOCAP_TRACKER}/pose"
    /mocap/scout_pose
    /mocap/scout_odom
    /mocap/scout_path
    /mocap/status
  )
fi

bag="${OUT_DIR}/${RUN_ID}.bag"
echo "[rec] 录包 ${RECORD_SEC}s -> ${bag}"
rosbag record -O "$bag" "${record_topics[@]}" \
  >"${OUT_DIR}/${RUN_ID}_rosbag.log" 2>&1 &
rec_pid=$!

sleep "$RECORD_SEC"

cleanup
trap - EXIT
sleep 1

echo "[done] ${VARIANT} -> ${bag}"
echo "[meta] ${meta}"
