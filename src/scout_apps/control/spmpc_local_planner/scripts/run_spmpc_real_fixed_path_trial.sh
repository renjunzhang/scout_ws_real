#!/usr/bin/env bash
# One-click SPMPC fixed-path real trial wrapper.
# Assumes the real sensor/base/localization stack is already running.
# This script starts the fixed-path generator, starts the black-box bag recorder,
# sends the goal, waits for the fixed path, then launches the selected SPMPC
# variant. The recorder always has a bounded duration; Ctrl+C stops the run earlier.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[ERR] $*" >&2
  exit 2
}

show_log_tail() {
  local file="$1"
  local label="$2"
  if [[ -s "${file}" ]]; then
    echo "[debug] last lines from ${label}: ${file}" >&2
    tail -n 40 "${file}" >&2 || true
  fi
}

require_cmd() {
  local cmd="$1"
  command -v "${cmd}" >/dev/null 2>&1 || fail "Required command not found: ${cmd}"
}

is_number() {
  [[ "$1" =~ ^-?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]
}

require_number() {
  local name="$1"
  local value="$2"
  is_number "${value}" || fail "${name} must be numeric, got '${value}'"
}

child_running() {
  local pid="$1"
  [[ -n "${pid}" ]] && jobs -pr | grep -qx "${pid}"
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
DELAY_PHASE_MODE="${DELAY_PHASE_MODE:-off}"
DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC:--1.0}"
DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC:--1.0}"
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
PATH_GENERATOR_STARTUP_SEC="${PATH_GENERATOR_STARTUP_SEC:-2}"
RECORDER_STARTUP_SEC="${RECORDER_STARTUP_SEC:-2}"
PLANNER_STARTUP_SEC="${PLANNER_STARTUP_SEC:-2}"
SEND_ZERO_ON_EXIT="${SEND_ZERO_ON_EXIT:-true}"
OPERATOR_NOTE="${OPERATOR_NOTE:-one_click_spmpc_real_fixed_path_trial}"

require_cmd timeout
require_cmd rostopic
require_cmd rosrun
require_cmd roslaunch
[[ -f "${RECORDER_SCRIPT}" ]] || fail "Recorder script not found: ${RECORDER_SCRIPT}"
[[ -r "${RECORDER_SCRIPT}" ]] || fail "Recorder script is not readable: ${RECORDER_SCRIPT}"
case "${DELAY_PHASE_MODE}" in
  off|monitor|shadow|fixed_closed_loop) ;;
  *) fail "DELAY_PHASE_MODE must be off|monitor|shadow|fixed_closed_loop, got '${DELAY_PHASE_MODE}'" ;;
esac
for kv in \
  "GOAL_X=${GOAL_X}" \
  "GOAL_Y=${GOAL_Y}" \
  "GOAL_YAW=${GOAL_YAW}" \
  "PATH_SPACING=${PATH_SPACING}" \
  "PATH_AMPLITUDE_RATIO=${PATH_AMPLITUDE_RATIO}" \
  "PATH_MIN_AMPLITUDE=${PATH_MIN_AMPLITUDE}" \
  "PATH_MAX_AMPLITUDE=${PATH_MAX_AMPLITUDE}" \
  "GOAL_REPEAT_RATE=${GOAL_REPEAT_RATE}" \
  "V_REF=${V_REF}" \
  "W_SLOSH=${W_SLOSH}" \
  "SLOSH_HEIGHT_MAX=${SLOSH_HEIGHT_MAX}" \
  "DELAY_PHASE_LINEAR_DELAY_SEC=${DELAY_PHASE_LINEAR_DELAY_SEC}" \
  "DELAY_PHASE_ANGULAR_DELAY_SEC=${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
  "ALPHA_MAX=${ALPHA_MAX}" \
  "SHARED_LINEAR_ACCEL_MAX=${SHARED_LINEAR_ACCEL_MAX}" \
  "SHARED_ANGULAR_RATE_MAX=${SHARED_ANGULAR_RATE_MAX}" \
  "SHARED_ANGULAR_ACCEL_MAX=${SHARED_ANGULAR_ACCEL_MAX}" \
  "PATH_GENERATOR_STARTUP_SEC=${PATH_GENERATOR_STARTUP_SEC}" \
  "RECORDER_STARTUP_SEC=${RECORDER_STARTUP_SEC}" \
  "PLANNER_STARTUP_SEC=${PLANNER_STARTUP_SEC}"; do
  require_number "${kv%%=*}" "${kv#*=}"
done
for kv in \
  "GOAL_REPEAT_COUNT=${GOAL_REPEAT_COUNT}" \
  "PATH_SMOOTH_ITERATIONS=${PATH_SMOOTH_ITERATIONS}"; do
  case "${kv#*=}" in
    ''|*[!0-9]*) fail "${kv%%=*} must be a non-negative integer, got '${kv#*=}'" ;;
  esac
done
if ! timeout 5s rostopic list >/dev/null 2>&1; then
  fail "ROS master is not reachable; source the workspace and start the real stack/roscore first"
fi

mkdir -p "${RUN_OUT_DIR}" "$(dirname "${PATH_FILE}")"

path_generator_log="${RUN_OUT_DIR}/${NAME}_path_generator.log"
send_goal_log="${RUN_OUT_DIR}/${NAME}_send_goal.log"
recorder_log="${RUN_OUT_DIR}/${NAME}_recorder.log"
planner_log="${RUN_OUT_DIR}/${NAME}_planner.log"

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
  > "${path_generator_log}" 2>&1 &
path_generator_pid=$!
sleep "${PATH_GENERATOR_STARTUP_SEC}"
if ! child_running "${path_generator_pid}"; then
  set +e
  wait "${path_generator_pid}"
  path_code=$?
  set -e
  show_log_tail "${path_generator_log}" "path generator"
  fail "Path generator exited before receiving the fixed goal (code=${path_code})"
fi

echo "[record] starting black-box recorder before goal/planner"
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
) > "${recorder_log}" 2>&1 &
recorder_pid=$!
sleep "${RECORDER_STARTUP_SEC}"
if ! child_running "${recorder_pid}"; then
  set +e
  wait "${recorder_pid}"
  recorder_code=$?
  set -e
  show_log_tail "${recorder_log}" "recorder"
  fail "Recorder exited during startup (code=${recorder_code})"
fi

echo "[goal] sending fixed goal"
if ! rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic "${GOAL_TOPIC}" \
  --frame "${GOAL_FRAME}" \
  --x "${GOAL_X}" \
  --y "${GOAL_Y}" \
  --yaw "${GOAL_YAW}" \
  --repeat-count "${GOAL_REPEAT_COUNT}" \
  --repeat-rate "${GOAL_REPEAT_RATE}" \
  > "${send_goal_log}" 2>&1; then
  show_log_tail "${send_goal_log}" "send fixed goal"
  fail "Failed to send fixed goal"
fi

echo "[path] waiting for ${REF_TOPIC}"
if ! timeout 10s rostopic echo -n 1 "${REF_TOPIC}" >/dev/null; then
  show_log_tail "${path_generator_log}" "path generator"
  fail "Timed out waiting for fixed path on ${REF_TOPIC}"
fi
if ! child_running "${path_generator_pid}"; then
  show_log_tail "${path_generator_log}" "path generator"
  fail "Path generator stopped after goal; fixed path may not remain available"
fi

echo "[launch] starting planner"
"${planner_cmd[@]}" > "${planner_log}" 2>&1 &
planner_pid=$!
sleep "${PLANNER_STARTUP_SEC}"
if ! child_running "${planner_pid}"; then
  set +e
  wait "${planner_pid}"
  planner_code=$?
  set -e
  show_log_tail "${planner_log}" "planner"
  fail "Planner exited during startup (code=${planner_code})"
fi

echo "[run] recording until Ctrl+C, ${RECORD_SEC}s recorder timeout, or planner exit"
set +e
wait -n "${recorder_pid}" "${planner_pid}"
first_exit_code=$?
set -e
if child_running "${recorder_pid}"; then
  show_log_tail "${planner_log}" "planner"
  echo "[ERR] planner exited before recorder timeout (code=${first_exit_code}); stopping recorder/generator" >&2
  recorder_code=1
else
  recorder_code=${first_exit_code}
  if (( recorder_code != 0 )); then
    show_log_tail "${recorder_log}" "recorder"
  fi
  echo "[run] recorder exited with code ${recorder_code}; stopping planner/generator"
fi
cleanup
trap - EXIT INT TERM

echo "================ trial finished ================"
echo "  bag/meta dir = ${RUN_OUT_DIR}"
echo "  run meta     = ${run_meta}"
echo "  recorder log = ${recorder_log}"
echo "  planner log  = ${planner_log}"
echo "  path log     = ${path_generator_log}"
echo "================================================"
exit "${recorder_code}"
