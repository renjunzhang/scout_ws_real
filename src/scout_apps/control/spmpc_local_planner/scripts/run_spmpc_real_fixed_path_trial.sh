#!/usr/bin/env bash
# One-click SPMPC fixed-path real trial wrapper.
# Assumes the real sensor/base/localization stack is already running.
# This script starts the fixed-path generator, sends the goal, starts the
# black-box bag recorder, then launches the selected SPMPC variant. The recorder
# always has a bounded duration; Ctrl+C stops the run earlier.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VARIANT="${VARIANT:-${ALG:-B_ours}}"
ALG="${ALG:-${VARIANT}}"
RUN_LABEL="${RUN_LABEL:-real_fixed_${VARIANT}_${STAMP}}"
NAME="${NAME:-${RUN_LABEL}}"

MAX_RECORD_SEC="${MAX_RECORD_SEC:-60}"
RECORD_SEC="${RECORD_SEC:-60}"
case "${MAX_RECORD_SEC}" in
  ''|*[!0-9]*) echo "[ERR] MAX_RECORD_SEC must be a positive integer, got '${MAX_RECORD_SEC}'" >&2; exit 2 ;;
esac
case "${RECORD_SEC}" in
  ''|*[!0-9]*) echo "[ERR] RECORD_SEC must be a positive integer, got '${RECORD_SEC}'" >&2; exit 2 ;;
esac
if (( MAX_RECORD_SEC <= 0 )); then
  echo "[ERR] MAX_RECORD_SEC must be > 0, got '${MAX_RECORD_SEC}'" >&2
  exit 2
fi
if (( RECORD_SEC <= 0 || RECORD_SEC > MAX_RECORD_SEC )); then
  echo "[WARN] RECORD_SEC=${RECORD_SEC} is outside (0, ${MAX_RECORD_SEC}], forcing RECORD_SEC=${MAX_RECORD_SEC}." >&2
  RECORD_SEC="${MAX_RECORD_SEC}"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || cd "${SCRIPT_DIR}/../../../../.." && pwd)"
RECORDER_SCRIPT="${RECORDER_SCRIPT:-${SCRIPT_DIR}/record_spmpc_full_rgb_bag.sh}"

BAG_ROOT="${BAG_ROOT:-${HOME}/slosh_bags/real/${DATE}_fixed_path_compare}"
RUN_OUT_DIR="${RUN_OUT_DIR:-${BAG_ROOT}/${VARIANT}}"
PATH_ROOT="${PATH_ROOT:-${HOME}/fixed_paths/real/${DATE}_fixed_path_compare}"
PATH_FILE="${PATH_FILE:-${PATH_ROOT}/fixed_s_curve_compare.json}"
REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
GOAL_FRAME="${GOAL_FRAME:-map}"
# Defaults recovered from the 2026-07-02 real internal-slosh bags:
# /scout/global_path_fixed consistently ended at (-5.424, -4.736) in map.
# Goal yaw was not present in the bags and is not used by the template generator.
GOAL_X="${GOAL_X:--5.424}"
GOAL_Y="${GOAL_Y:--4.736}"
GOAL_YAW="${GOAL_YAW:-0.0}"

PATH_TEMPLATE="${PATH_TEMPLATE:-s_curve}"
PATH_SPACING="${PATH_SPACING:-0.05}"
PATH_AMPLITUDE_RATIO="${PATH_AMPLITUDE_RATIO:-0.18}"
PATH_MIN_AMPLITUDE="${PATH_MIN_AMPLITUDE:-0.25}"
PATH_MAX_AMPLITUDE="${PATH_MAX_AMPLITUDE:-1.20}"
PATH_SIDE="${PATH_SIDE:-left}"
PATH_SMOOTH_ITERATIONS="${PATH_SMOOTH_ITERATIONS:-3}"
GOAL_REPEAT_COUNT="${GOAL_REPEAT_COUNT:-5}"
GOAL_REPEAT_RATE="${GOAL_REPEAT_RATE:-5}"

CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
COSTMAP_TOPIC="${COSTMAP_TOPIC:-/map}"
REFERENCE_TARGET_FRAME="${REFERENCE_TARGET_FRAME:-map}"
SOLVER_BACKEND="${SOLVER_BACKEND:-continuous_mpcc_acados}"
V_REF="${V_REF:-0.20}"
W_SLOSH="${W_SLOSH:--1.0}"
SLOSH_HEIGHT_MAX="${SLOSH_HEIGHT_MAX:--1.0}"
DELAY_PHASE_MODE="${DELAY_PHASE_MODE:-fixed_closed_loop}"
DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC:-0.08}"
DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC:-0.05}"
ALPHA_MAX="${ALPHA_MAX:-1.2}"
SHARED_LINEAR_ACCEL_LIMIT_ENABLE="${SHARED_LINEAR_ACCEL_LIMIT_ENABLE:-true}"
SHARED_LINEAR_ACCEL_MAX="${SHARED_LINEAR_ACCEL_MAX:-0.6}"
SHARED_ANGULAR_LIMIT_ENABLE="${SHARED_ANGULAR_LIMIT_ENABLE:-true}"
SHARED_ANGULAR_RATE_MAX="${SHARED_ANGULAR_RATE_MAX:-1.2}"
SHARED_ANGULAR_ACCEL_MAX="${SHARED_ANGULAR_ACCEL_MAX:-1.2}"

RECORD_RGB="${RECORD_RGB:-false}"
RECORD_SCAN="${RECORD_SCAN:-true}"
RECORD_DEPTH="${RECORD_DEPTH:-false}"
RECORD_STANDALONE_SLOSH="${RECORD_STANDALONE_SLOSH:-true}"
RECORD_ONLINE_LIQUID="${RECORD_ONLINE_LIQUID:-false}"
RECORD_ALL_EXISTING_TOPICS="${RECORD_ALL_EXISTING_TOPICS:-false}"
RECORD_TOPIC_INFO="${RECORD_TOPIC_INFO:-true}"
RECORDER_STARTUP_SEC="${RECORDER_STARTUP_SEC:-2}"
SEND_ZERO_ON_EXIT="${SEND_ZERO_ON_EXIT:-true}"
OPERATOR_NOTE="${OPERATOR_NOTE:-one_click_spmpc_real_fixed_path_trial}"

mkdir -p "${RUN_OUT_DIR}" "$(dirname "${PATH_FILE}")"

path_generator_pid=""
recorder_pid=""
planner_pid=""
cleaned_up=false

kill_child() {
  local pid="$1"
  local label="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    echo "[cleanup] stopping ${label} (pid=${pid})"
    kill -INT "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

publish_zero_cmd() {
  if [[ "${CMD_TOPIC}" != "/cmd_vel" ]] || ! truthy "${SEND_ZERO_ON_EXIT}"; then
    return 0
  fi
  timeout 2s rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
}

cleanup() {
  ${cleaned_up} && return 0
  cleaned_up=true
  kill_child "${planner_pid}" "planner"
  publish_zero_cmd
  kill_child "${recorder_pid}" "recorder"
  kill_child "${path_generator_pid}" "path generator"
}

on_interrupt() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_interrupt INT TERM

planner_cmd=(
  roslaunch spmpc_local_planner spmpc_fixed_path.launch
  "planner_variant:=${VARIANT}"
  "solver_backend:=${SOLVER_BACKEND}"
  "reference_path_topic:=${REF_TOPIC}"
  "cmd_vel_topic:=${CMD_TOPIC}"
  "costmap_topic:=${COSTMAP_TOPIC}"
  "reference_target_frame:=${REFERENCE_TARGET_FRAME}"
  "delay_phase_mode:=${DELAY_PHASE_MODE}"
  "delay_phase_linear_delay_sec:=${DELAY_PHASE_LINEAR_DELAY_SEC}"
  "delay_phase_angular_delay_sec:=${DELAY_PHASE_ANGULAR_DELAY_SEC}"
  "v_ref:=${V_REF}"
  "w_slosh:=${W_SLOSH}"
  "slosh_height_max:=${SLOSH_HEIGHT_MAX}"
  "alpha_max:=${ALPHA_MAX}"
  "shared_linear_accel_limit_enable:=${SHARED_LINEAR_ACCEL_LIMIT_ENABLE}"
  "shared_linear_accel_max:=${SHARED_LINEAR_ACCEL_MAX}"
  "shared_angular_limit_enable:=${SHARED_ANGULAR_LIMIT_ENABLE}"
  "shared_angular_rate_max:=${SHARED_ANGULAR_RATE_MAX}"
  "shared_angular_accel_max:=${SHARED_ANGULAR_ACCEL_MAX}"
)
planner_command_string="$(printf '%q ' "${planner_cmd[@]}")"

run_meta="${RUN_OUT_DIR}/${NAME}_one_click_meta.env"
{
  echo "date=${DATE}"
  echo "stamp=${STAMP}"
  echo "variant=${VARIANT}"
  echo "run_label=${RUN_LABEL}"
  echo "name=${NAME}"
  echo "record_sec=${RECORD_SEC}"
  echo "max_record_sec=${MAX_RECORD_SEC}"
  echo "run_out_dir=${RUN_OUT_DIR}"
  echo "path_file=${PATH_FILE}"
  echo "ref_topic=${REF_TOPIC}"
  echo "goal_topic=${GOAL_TOPIC}"
  echo "goal_frame=${GOAL_FRAME}"
  echo "goal_x=${GOAL_X}"
  echo "goal_y=${GOAL_Y}"
  echo "goal_yaw=${GOAL_YAW}"
  echo "cmd_topic=${CMD_TOPIC}"
  echo "solver_backend=${SOLVER_BACKEND}"
  echo "v_ref=${V_REF}"
  echo "w_slosh=${W_SLOSH}"
  echo "slosh_height_max=${SLOSH_HEIGHT_MAX}"
  echo "delay_phase_mode=${DELAY_PHASE_MODE}"
  echo "delay_phase_linear_delay_sec=${DELAY_PHASE_LINEAR_DELAY_SEC}"
  echo "delay_phase_angular_delay_sec=${DELAY_PHASE_ANGULAR_DELAY_SEC}"
  echo "record_rgb=${RECORD_RGB}"
  echo "record_online_liquid=${RECORD_ONLINE_LIQUID}"
  echo "planner_command=${planner_command_string}"
} > "${run_meta}"

echo "================ SPMPC real fixed-path trial ================"
echo "  variant       = ${VARIANT}"
echo "  run_label     = ${RUN_LABEL}"
echo "  cmd_topic     = ${CMD_TOPIC}"
echo "  recorder      = ${RECORD_SEC}s max (Ctrl+C stops earlier)"
echo "  out_dir       = ${RUN_OUT_DIR}"
echo "  path_file     = ${PATH_FILE}"
echo "  goal          = (${GOAL_X}, ${GOAL_Y}, ${GOAL_YAW}) in ${GOAL_FRAME}"
echo "============================================================="

echo "[path] starting generator -> ${REF_TOPIC}"
rosrun scout_local_planner template_fixed_path_generator.py \
  --template "${PATH_TEMPLATE}" \
  --goal-topic "${GOAL_TOPIC}" \
  --output-topic "${REF_TOPIC}" \
  --path-file "${PATH_FILE}" \
  --start-heading current \
  --spacing "${PATH_SPACING}" \
  --amplitude-ratio "${PATH_AMPLITUDE_RATIO}" \
  --min-amplitude "${PATH_MIN_AMPLITUDE}" \
  --max-amplitude "${PATH_MAX_AMPLITUDE}" \
  --side "${PATH_SIDE}" \
  --smooth-iterations "${PATH_SMOOTH_ITERATIONS}" \
  --publish-count 0 \
  > "${RUN_OUT_DIR}/${NAME}_path_generator.log" 2>&1 &
path_generator_pid=$!
sleep 2

echo "[goal] sending fixed goal"
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic "${GOAL_TOPIC}" \
  --frame "${GOAL_FRAME}" \
  --x "${GOAL_X}" \
  --y "${GOAL_Y}" \
  --yaw "${GOAL_YAW}" \
  --repeat-count "${GOAL_REPEAT_COUNT}" \
  --repeat-rate "${GOAL_REPEAT_RATE}" \
  > "${RUN_OUT_DIR}/${NAME}_send_goal.log" 2>&1

echo "[path] waiting for ${REF_TOPIC}"
timeout 10s rostopic echo -n 1 "${REF_TOPIC}" >/dev/null

echo "[record] starting black-box recorder before planner"
(
  cd "${REPO_ROOT}"
  DATE="${DATE}" \
  STAMP="${STAMP}" \
  VARIANT="${VARIANT}" \
  RUN_LABEL="${RUN_LABEL}" \
  RECORD_SEC="${RECORD_SEC}" \
  OUT_DIR="${RUN_OUT_DIR}" \
  NAME="${NAME}" \
  RECORD_RGB="${RECORD_RGB}" \
  RECORD_SCAN="${RECORD_SCAN}" \
  RECORD_DEPTH="${RECORD_DEPTH}" \
  RECORD_STANDALONE_SLOSH="${RECORD_STANDALONE_SLOSH}" \
  RECORD_ONLINE_LIQUID="${RECORD_ONLINE_LIQUID}" \
  RECORD_ALL_EXISTING_TOPICS="${RECORD_ALL_EXISTING_TOPICS}" \
  RECORD_TOPIC_INFO="${RECORD_TOPIC_INFO}" \
  SOLVER_BACKEND="${SOLVER_BACKEND}" \
  V_REF="${V_REF}" \
  W_SLOSH="${W_SLOSH}" \
  SLOSH_HEIGHT_MAX="${SLOSH_HEIGHT_MAX}" \
  DELAY_PHASE_MODE="${DELAY_PHASE_MODE}" \
  DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
  DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
  GOAL_X="${GOAL_X}" \
  GOAL_Y="${GOAL_Y}" \
  GOAL_YAW="${GOAL_YAW}" \
  LAUNCH_COMMAND="${planner_command_string}" \
  OPERATOR_NOTE="${OPERATOR_NOTE}" \
  bash "${RECORDER_SCRIPT}"
) > "${RUN_OUT_DIR}/${NAME}_recorder.log" 2>&1 &
recorder_pid=$!
sleep "${RECORDER_STARTUP_SEC}"
if ! kill -0 "${recorder_pid}" 2>/dev/null; then
  wait "${recorder_pid}"
fi

echo "[launch] starting planner"
"${planner_cmd[@]}" > "${RUN_OUT_DIR}/${NAME}_planner.log" 2>&1 &
planner_pid=$!

echo "[run] recording until Ctrl+C or ${RECORD_SEC}s recorder timeout"
set +e
wait "${recorder_pid}"
recorder_code=$?
set -e

echo "[run] recorder exited with code ${recorder_code}; stopping planner/generator"
cleanup
trap - EXIT INT TERM

echo "================ trial finished ================"
echo "  bag/meta dir = ${RUN_OUT_DIR}"
echo "  run meta     = ${run_meta}"
echo "  recorder log = ${RUN_OUT_DIR}/${NAME}_recorder.log"
echo "  planner log  = ${RUN_OUT_DIR}/${NAME}_planner.log"
echo "  path log     = ${RUN_OUT_DIR}/${NAME}_path_generator.log"
echo "================================================"
exit "${recorder_code}"
