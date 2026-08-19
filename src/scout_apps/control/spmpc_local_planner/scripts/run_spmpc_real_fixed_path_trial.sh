#!/usr/bin/env bash
# One-click SPMPC fixed-path real trial wrapper.
# Assumes the real sensor/base/localization stack is already running.
# The path source can either generate a path from the current pose or replay a
# frozen JSON path with a configurable start-pose gate. The script also starts
# the black-box recorder and selected SPMPC variant. The recorder always has a
# bounded duration; Ctrl+C stops the run earlier.
# With IMU_SHADOW_ENABLE=true it records the complete stationary bias/filter
# transient, waits for a valid READY diagnostic, and only then releases the
# path/goal stage.  The default false path preserves the established odom run.

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

wait_for_operator() {
  local message="$1"
  if [[ -t 0 ]]; then
    echo ""
    echo "[matrix] ${message}"
    echo "[matrix] Press Enter to continue, or Ctrl+C to stop."
    read -r _
  else
    echo "[matrix] ${message}"
    echo "[matrix] stdin is not a terminal; continuing without interactive wait."
  fi
}

run_matrix_preset() {
  local preset="$1"
  local script_path
  script_path="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

  local label_tag="${MATRIX_LABEL_TAG:-0706}"
  local matrix_date="${DATE:-$(date +%Y%m%d)}"
  local include_gate="${MATRIX_INCLUDE_GATE:-true}"
  local wait_between="${MATRIX_WAIT_BETWEEN_RUNS:-true}"
  local continue_on_fail="${MATRIX_CONTINUE_ON_FAIL:-false}"
  local runs=()

  case "${preset}" in
    0706_bsmooth_bours|0706_smooth_bridge)
      if truthy "${include_gate}"; then
        runs+=("B0|B0_fixed_150_220_${label_tag}_gate01|false")
      fi
      runs+=("B_smooth|Bsmooth_fixed_150_220_${label_tag}_r1|true")
      runs+=("B_ours|Bours_fixed_150_220_${label_tag}_r1|true")
      runs+=("B_ours|Bours_fixed_150_220_${label_tag}_r2|true")
      runs+=("B_smooth|Bsmooth_fixed_150_220_${label_tag}_r2|true")
      runs+=("B_smooth|Bsmooth_fixed_150_220_${label_tag}_r3|true")
      runs+=("B_ours|Bours_fixed_150_220_${label_tag}_r3|true")
      ;;
    *)
      fail "Unknown MATRIX_PRESET='${preset}'. Supported: 0706_bsmooth_bours"
      ;;
  esac

  echo "================ SPMPC real fixed-path matrix ================"
  echo "  preset       = ${preset}"
  echo "  date         = ${matrix_date}"
  echo "  label_tag    = ${label_tag}"
  echo "  include_gate = ${include_gate}"
  echo "  runs         = ${#runs[@]}"
  echo "  control      = fixed_closed_loop 0.15 / 0.22, v_ref=0.20"
  echo "=============================================================="

  local index=0
  local item variant label record_rgb rc
  for item in "${runs[@]}"; do
    index=$((index + 1))
    IFS='|' read -r variant label record_rgb <<< "${item}"
    echo ""
    echo "[matrix] next ${index}/${#runs[@]}: ${label} (${variant}, RECORD_RGB=${record_rgb})"
    if truthy "${wait_between}"; then
      wait_for_operator "Return the robot to the start mark, align heading, let liquid settle 60-90s, and confirm RGB/TF/odom are healthy."
    fi

    set +e
    env \
      MATRIX_PRESET= \
      DATE="${matrix_date}" \
      STAMP= \
      ALG="${variant}" \
      VARIANT="${variant}" \
      RUN_LABEL="${label}" \
      NAME="${label}" \
      RUN_OUT_DIR= \
      PATH_FILE= \
      PILOT_MODE=false \
      PILOT_METHOD= \
      PILOT_RECORD_RGB=false \
      PATH_SOURCE_MODE=generate \
      CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}" \
      V_REF="${V_REF:-0.20}" \
      ALPHA_MAX="${ALPHA_MAX:-1.2}" \
      SHARED_LINEAR_ACCEL_LIMIT_ENABLE="${SHARED_LINEAR_ACCEL_LIMIT_ENABLE:-true}" \
      SHARED_LINEAR_ACCEL_MAX="${SHARED_LINEAR_ACCEL_MAX:-0.6}" \
      SHARED_ANGULAR_LIMIT_ENABLE="${SHARED_ANGULAR_LIMIT_ENABLE:-true}" \
      SHARED_ANGULAR_RATE_MAX="${SHARED_ANGULAR_RATE_MAX:-1.2}" \
      SHARED_ANGULAR_ACCEL_MAX="${SHARED_ANGULAR_ACCEL_MAX:-1.2}" \
      DELAY_PHASE_MODE="${DELAY_PHASE_MODE:-fixed_closed_loop}" \
      DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC:-0.15}" \
      DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC:-0.22}" \
      RECORD_TOPIC_INFO="${RECORD_TOPIC_INFO:-false}" \
      RECORDER_STARTUP_SEC="${RECORDER_STARTUP_SEC:-8}" \
      RECORD_RGB="${record_rgb}" \
      RECORD_SEC="${RECORD_SEC:-60}" \
      MAX_RECORD_SEC="${MAX_RECORD_SEC:-60}" \
      bash "${script_path}"
    rc=$?
    set -e

    if (( rc != 0 )); then
      echo "[matrix] run failed: ${label} rc=${rc}" >&2
      if ! truthy "${continue_on_fail}"; then
        echo "[matrix] stop remaining runs. Set MATRIX_CONTINUE_ON_FAIL=true to continue after failures." >&2
        return "${rc}"
      fi
    fi
  done

  echo "================ matrix finished ================"
  echo "  preset = ${preset}"
  echo "  date   = ${matrix_date}"
  echo "================================================="
}

if [[ -n "${MATRIX_PRESET:-}" ]]; then
  run_matrix_preset "${MATRIX_PRESET}"
  exit $?
fi

DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
PILOT_MODE="${PILOT_MODE:-false}"
PILOT_METHOD="${PILOT_METHOD:-}"
PILOT_RECORD_RGB="${PILOT_RECORD_RGB:-false}"
PILOT_RECORD_ONLINE_LIQUID="${PILOT_RECORD_ONLINE_LIQUID:-false}"
ALLOW_PILOT_PATH_OVERWRITE="${ALLOW_PILOT_PATH_OVERWRITE:-false}"

if [[ -z "${PATH_SOURCE_MODE:-}" ]]; then
  if truthy "${PILOT_MODE}"; then
    PATH_SOURCE_MODE=replay
  else
    PATH_SOURCE_MODE=generate
  fi
fi

if [[ -n "${PILOT_METHOD}" ]]; then
  case "${PILOT_METHOD}" in
    B0)
      VARIANT=B0
      W_SLOSH=0.0
      ;;
    Bsmooth|B_smooth)
      VARIANT=B_smooth
      W_SLOSH=0.0
      ;;
    W*)
      pilot_weight="${PILOT_METHOD#W}"
      require_number "PILOT_METHOD weight" "${pilot_weight}"
      [[ "${pilot_weight}" != -* ]] || fail "PILOT_METHOD weight must be non-negative, got '${pilot_weight}'"
      VARIANT=B_slosh
      W_SLOSH="${pilot_weight}"
      ;;
    *)
      fail "Unknown PILOT_METHOD='${PILOT_METHOD}'. Use B0, Bsmooth, or W<number> such as W1/W2/W5/W10."
      ;;
  esac
  ALG="${VARIANT}"
else
  VARIANT="${VARIANT:-${ALG:-B_ours}}"
  ALG="${ALG:-${VARIANT}}"
fi

if [[ -z "${RUN_LABEL:-}" ]]; then
  if truthy "${PILOT_MODE}"; then
    RUN_LABEL="pilot_${PILOT_METHOD:-${VARIANT}}_${STAMP}"
  else
    RUN_LABEL="real_fixed_${VARIANT}_${STAMP}"
  fi
fi
NAME="${NAME:-${RUN_LABEL}}"
if truthy "${PILOT_MODE}"; then
  RUN_CLASS=pilot
else
  RUN_CLASS=trial
fi

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
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || { cd "${SCRIPT_DIR}/../../../../.." && pwd; })"
RECORDER_SCRIPT="${RECORDER_SCRIPT:-${SCRIPT_DIR}/record_spmpc_full_rgb_bag.sh}"

if truthy "${PILOT_MODE}"; then
  BAG_ROOT="${BAG_ROOT:-${HOME}/slosh_bags/real/${DATE}_spmpc_parameter_pilot}"
  RUN_OUT_DIR="${RUN_OUT_DIR:-${BAG_ROOT}/${PILOT_METHOD:-${VARIANT}}}"
else
  BAG_ROOT="${BAG_ROOT:-${HOME}/slosh_bags/real/${DATE}_fixed_path_compare}"
  RUN_OUT_DIR="${RUN_OUT_DIR:-${BAG_ROOT}/${VARIANT}}"
fi
if truthy "${PILOT_MODE}"; then
  PATH_ROOT="${PATH_ROOT:-${HOME}/fixed_paths/real/${DATE}_spmpc_parameter_pilot}"
  PATH_FILE="${PATH_FILE:-${PATH_ROOT}/H0_weight_pilot.json}"
else
  PATH_ROOT="${PATH_ROOT:-${HOME}/fixed_paths/real/${DATE}_fixed_path_compare}"
  if [[ "${PATH_SOURCE_MODE}" == "replay" ]]; then
    PATH_FILE="${PATH_FILE:-}"
  else
    PATH_FILE="${PATH_FILE:-${PATH_ROOT}/fixed_s_curve_compare.json}"
  fi
fi
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
BASE_FRAME="${BASE_FRAME:-base_link}"
if truthy "${PILOT_MODE}"; then
  START_POS_TOL="${START_POS_TOL:-0.08}"
  START_YAW_TOL="${START_YAW_TOL:-0.15}"
else
  START_POS_TOL="${START_POS_TOL:-0.05}"
  START_YAW_TOL="${START_YAW_TOL:-0.10}"
fi
START_HOLD_SEC="${START_HOLD_SEC:-0.5}"
START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC:-120}"
PATH_PUBLISH_RATE="${PATH_PUBLISH_RATE:-2.0}"
GENERATED_PATH_WAIT_SEC="${GENERATED_PATH_WAIT_SEC:-10}"

CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
COSTMAP_TOPIC="${COSTMAP_TOPIC:-/map}"
REFERENCE_TARGET_FRAME="${REFERENCE_TARGET_FRAME:-map}"
SOLVER_BACKEND="${SOLVER_BACKEND:-continuous_mpcc_acados}"
V_REF="${V_REF:-0.20}"
W_SLOSH="${W_SLOSH:--1.0}"
W_SMOOTH="${W_SMOOTH:--1.0}"
W_ALPHA="${W_ALPHA:--1.0}"
W_DU_A="${W_DU_A:--1.0}"
W_DU_VS="${W_DU_VS:--1.0}"
SLOSH_HEIGHT_MAX="${SLOSH_HEIGHT_MAX:--1.0}"
IMU_SHADOW_ENABLE="${IMU_SHADOW_ENABLE:-false}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
IMU_SUBSCRIBER_QUEUE_SIZE="${IMU_SUBSCRIBER_QUEUE_SIZE:-10}"
CURRENT_OBSERVER_SOURCE="${CURRENT_OBSERVER_SOURCE:-${OBSERVER_SOURCE:-odom}}"
OBSERVER_FALLBACK_POLICY="${OBSERVER_FALLBACK_POLICY:-odom}"
OBSERVER_LATCH_FALLBACK="${OBSERVER_LATCH_FALLBACK:-true}"
OBSERVER_MAX_IMU_STATE_AGE_SEC="${OBSERVER_MAX_IMU_STATE_AGE_SEC:-0.10}"
OBSERVER_MAX_ODOM_STATE_AGE_SEC="${OBSERVER_MAX_ODOM_STATE_AGE_SEC:-0.50}"
OBSERVER_MAX_FUTURE_SKEW_SEC="${OBSERVER_MAX_FUTURE_SKEW_SEC:-0.005}"
STATE_TIMING_REQUIRE_COMMON_EPOCH="${STATE_TIMING_REQUIRE_COMMON_EPOCH:-true}"
STATE_TIMING_MAX_RAW_SKEW_SEC="${STATE_TIMING_MAX_RAW_SKEW_SEC:-0.080}"
STATE_TIMING_MAX_INTERPOLATION_GAP_SEC="${STATE_TIMING_MAX_INTERPOLATION_GAP_SEC:-0.050}"
STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC="${STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC:-0.010}"
EXECUTION_CONTRACT_FAIL_CLOSED="${EXECUTION_CONTRACT_FAIL_CLOSED:-false}"
EXECUTION_CONTRACT_MAX_DELTA_V="${EXECUTION_CONTRACT_MAX_DELTA_V:-0.0001}"
EXECUTION_CONTRACT_MAX_DELTA_OMEGA="${EXECUTION_CONTRACT_MAX_DELTA_OMEGA:-0.0001}"
case "${CURRENT_OBSERVER_SOURCE}" in
  odom) ;;
  processed_imu|imu) CURRENT_OBSERVER_SOURCE=processed_imu ;;
  *) fail "CURRENT_OBSERVER_SOURCE must be odom|processed_imu, got '${CURRENT_OBSERVER_SOURCE}'" ;;
esac
case "${OBSERVER_FALLBACK_POLICY}" in
  odom|fail_closed) ;;
  *) fail "OBSERVER_FALLBACK_POLICY must be odom|fail_closed, got '${OBSERVER_FALLBACK_POLICY}'" ;;
esac
if [[ "${CURRENT_OBSERVER_SOURCE}" == "processed_imu" ]]; then
  # A nominal IMU source cannot be launched without its input pipeline and
  # startup READY gate, regardless of a stale shell export.
  IMU_SHADOW_ENABLE=true
fi
if truthy "${IMU_SHADOW_ENABLE}"; then
  # roslaunch XML bool arguments are most reliable as literal true/false.
  IMU_SHADOW_ENABLE=true
else
  IMU_SHADOW_ENABLE=false
fi
IMU_SHADOW_READY_TOPIC="${IMU_SHADOW_READY_TOPIC:-/spmpc/debug/slosh_observer_imu}"
OBSERVER_SELECTION_TOPIC="${OBSERVER_SELECTION_TOPIC:-/spmpc/debug/slosh_observer_selection}"
IMU_SHADOW_READY_TIMEOUT_SEC="${IMU_SHADOW_READY_TIMEOUT_SEC:-20}"
if truthy "${PILOT_MODE}"; then
  DELAY_PHASE_MODE="${DELAY_PHASE_MODE:-fixed_closed_loop}"
  DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC:-0.15}"
  DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC:-0.22}"
else
  DELAY_PHASE_MODE="${DELAY_PHASE_MODE:-off}"
  DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC:--1.0}"
  DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC:--1.0}"
fi
ALPHA_MAX="${ALPHA_MAX:-1.2}"
SHARED_LINEAR_ACCEL_LIMIT_ENABLE="${SHARED_LINEAR_ACCEL_LIMIT_ENABLE:-true}"
SHARED_LINEAR_ACCEL_MAX="${SHARED_LINEAR_ACCEL_MAX:-0.6}"
SHARED_ANGULAR_LIMIT_ENABLE="${SHARED_ANGULAR_LIMIT_ENABLE:-true}"
SHARED_ANGULAR_RATE_MAX="${SHARED_ANGULAR_RATE_MAX:-1.2}"
SHARED_ANGULAR_ACCEL_MAX="${SHARED_ANGULAR_ACCEL_MAX:-1.2}"

if truthy "${PILOT_MODE}"; then
  # Pilot image and online-scalar policies are explicit so stale shell exports
  # cannot silently change the evidence stream.
  RECORD_RGB="${PILOT_RECORD_RGB}"
  RECORD_CAMERA="${PILOT_RECORD_RGB}"
  RECORD_CAMERA_INFO="${RECORD_CAMERA_INFO:-true}"
  RECORD_CAMERA_COMPRESSED=false
  RECORD_DEPTH=false
  RECORD_ONLINE_LIQUID="${PILOT_RECORD_ONLINE_LIQUID}"
else
  RECORD_RGB="${RECORD_RGB:-false}"
  RECORD_CAMERA="${RECORD_CAMERA:-${RECORD_RGB}}"
  RECORD_CAMERA_INFO="${RECORD_CAMERA_INFO:-true}"
  RECORD_CAMERA_COMPRESSED="${RECORD_CAMERA_COMPRESSED:-false}"
  RECORD_DEPTH="${RECORD_DEPTH:-false}"
  RECORD_ONLINE_LIQUID="${RECORD_ONLINE_LIQUID:-true}"
fi
RECORD_ONLINE_LIQUID_DEBUG_IMAGES="${RECORD_ONLINE_LIQUID_DEBUG_IMAGES:-false}"
FORBID_IMAGE_STREAMS="${FORBID_IMAGE_STREAMS:-false}"
RECORD_SCAN="${RECORD_SCAN:-true}"
RECORD_STANDALONE_SLOSH="${RECORD_STANDALONE_SLOSH:-true}"
RECORD_ALL_EXISTING_TOPICS="${RECORD_ALL_EXISTING_TOPICS:-false}"
RGB_CALIBRATION_FILE="${RGB_CALIBRATION_FILE:-${LIQUID_CALIBRATION:-}}"
RGB_CALIBRATION_EXPECTED_SHA256="${RGB_CALIBRATION_EXPECTED_SHA256:-}"
RGB_CALIBRATION_ACTUAL_SHA256=""
RGB_EXPECTED_WIDTH="${RGB_EXPECTED_WIDTH:-}"
RGB_EXPECTED_HEIGHT="${RGB_EXPECTED_HEIGHT:-}"
RGB_EXPECTED_FPS="${RGB_EXPECTED_FPS:-}"
ONLINE_LIQUID_MEASUREMENT_TOPIC="${ONLINE_LIQUID_MEASUREMENT_TOPIC:-/liquid/measurement}"
ONLINE_LIQUID_PROTOCOL="${ONLINE_LIQUID_PROTOCOL:-}"
ONLINE_LIQUID_CALIBRATION_SHA256="${ONLINE_LIQUID_CALIBRATION_SHA256:-}"
ONLINE_LIQUID_DETECTOR_SHA256="${ONLINE_LIQUID_DETECTOR_SHA256:-}"
ONLINE_LIQUID_NODE_SHA256="${ONLINE_LIQUID_NODE_SHA256:-}"
ONLINE_LIQUID_MSG_SHA256="${ONLINE_LIQUID_MSG_SHA256:-}"
ONLINE_LIQUID_CONFIG_SHA256="${ONLINE_LIQUID_CONFIG_SHA256:-}"
PATH_GENERATOR_STARTUP_SEC="${PATH_GENERATOR_STARTUP_SEC:-2}"
if truthy "${PILOT_MODE}"; then
  RECORD_TOPIC_INFO="${RECORD_TOPIC_INFO:-false}"
  RECORDER_STARTUP_SEC="${RECORDER_STARTUP_SEC:-8}"
else
  RECORD_TOPIC_INFO="${RECORD_TOPIC_INFO:-true}"
  RECORDER_STARTUP_SEC="${RECORDER_STARTUP_SEC:-2}"
fi
RECORDER_ACTIVE_TIMEOUT_SEC="${RECORDER_ACTIVE_TIMEOUT_SEC:-15}"
PLANNER_STARTUP_SEC="${PLANNER_STARTUP_SEC:-2}"
SEND_ZERO_ON_EXIT="${SEND_ZERO_ON_EXIT:-true}"
OPERATOR_NOTE="${OPERATOR_NOTE:-one_click_spmpc_real_fixed_path_trial}"
PILOT_CONDITION="${PILOT_CONDITION:-}"
BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID:-}"
SPLIT_BLOCK="${SPLIT_BLOCK:-false}"
ORDER_POSITION="${ORDER_POSITION:-}"
ACQUISITION_RETRY="${ACQUISITION_RETRY:-false}"
RETRY_REASON_FILE="${RETRY_REASON_FILE:-}"
PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256:-}"
PATH_ACTUAL_SHA256=""
REQUIRE_PATH_HASH="${REQUIRE_PATH_HASH:-false}"

require_cmd timeout
require_cmd rostopic
require_cmd rosrun
require_cmd roslaunch
require_cmd sha256sum
[[ -f "${RECORDER_SCRIPT}" ]] || fail "Recorder script not found: ${RECORDER_SCRIPT}"
[[ -r "${RECORDER_SCRIPT}" ]] || fail "Recorder script is not readable: ${RECORDER_SCRIPT}"
case "${PATH_SOURCE_MODE}" in
  generate|replay) ;;
  *) fail "PATH_SOURCE_MODE must be generate|replay, got '${PATH_SOURCE_MODE}'" ;;
esac
if [[ "${PATH_SOURCE_MODE}" == "replay" ]]; then
  [[ -n "${PATH_FILE}" ]] || fail "PATH_FILE is required when PATH_SOURCE_MODE=replay"
  [[ -s "${PATH_FILE}" ]] || fail "Frozen replay path is missing or empty: ${PATH_FILE}"
  if truthy "${REQUIRE_PATH_HASH}" || [[ -n "${PATH_EXPECTED_SHA256}" ]]; then
    [[ "${PATH_EXPECTED_SHA256}" =~ ^[0-9a-fA-F]{64}$ ]] || \
      fail "PATH_EXPECTED_SHA256 must be the frozen 64-hex digest when path hash checking is enabled"
  fi
fi
if truthy "${PILOT_MODE}" && [[ "${PATH_SOURCE_MODE}" == "generate" && -e "${PATH_FILE}" ]] && ! truthy "${ALLOW_PILOT_PATH_OVERWRITE}"; then
  fail "Pilot path already exists and will not be overwritten: ${PATH_FILE}. Use replay, choose a new PATH_FILE, or explicitly set ALLOW_PILOT_PATH_OVERWRITE=true."
fi
if [[ -n "${RGB_CALIBRATION_FILE}" ]]; then
  [[ -s "${RGB_CALIBRATION_FILE}" ]] || fail "RGB calibration is missing or empty: ${RGB_CALIBRATION_FILE}"
  RGB_CALIBRATION_ACTUAL_SHA256="$(sha256sum "${RGB_CALIBRATION_FILE}" | awk '{print $1}')"
  if [[ -n "${RGB_CALIBRATION_EXPECTED_SHA256}" ]]; then
    [[ "${RGB_CALIBRATION_EXPECTED_SHA256}" =~ ^[0-9a-fA-F]{64}$ ]] || \
      fail "RGB_CALIBRATION_EXPECTED_SHA256 must be a 64-hex digest"
    [[ "${RGB_CALIBRATION_ACTUAL_SHA256,,}" == "${RGB_CALIBRATION_EXPECTED_SHA256,,}" ]] || \
      fail "RGB calibration SHA-256 mismatch: expected=${RGB_CALIBRATION_EXPECTED_SHA256}, actual=${RGB_CALIBRATION_ACTUAL_SHA256}"
  fi
fi
case "${DELAY_PHASE_MODE}" in
  off|monitor|shadow|fixed_closed_loop|fixed_robot_only) ;;
  *) fail "DELAY_PHASE_MODE must be off|monitor|shadow|fixed_closed_loop|fixed_robot_only, got '${DELAY_PHASE_MODE}'" ;;
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
  "START_POS_TOL=${START_POS_TOL}" \
  "START_YAW_TOL=${START_YAW_TOL}" \
  "START_HOLD_SEC=${START_HOLD_SEC}" \
  "START_GATE_TIMEOUT_SEC=${START_GATE_TIMEOUT_SEC}" \
  "PATH_PUBLISH_RATE=${PATH_PUBLISH_RATE}" \
  "GENERATED_PATH_WAIT_SEC=${GENERATED_PATH_WAIT_SEC}" \
  "V_REF=${V_REF}" \
  "W_SLOSH=${W_SLOSH}" \
  "W_SMOOTH=${W_SMOOTH}" \
  "W_ALPHA=${W_ALPHA}" \
  "W_DU_A=${W_DU_A}" \
  "W_DU_VS=${W_DU_VS}" \
  "SLOSH_HEIGHT_MAX=${SLOSH_HEIGHT_MAX}" \
  "OBSERVER_MAX_IMU_STATE_AGE_SEC=${OBSERVER_MAX_IMU_STATE_AGE_SEC}" \
  "OBSERVER_MAX_ODOM_STATE_AGE_SEC=${OBSERVER_MAX_ODOM_STATE_AGE_SEC}" \
  "OBSERVER_MAX_FUTURE_SKEW_SEC=${OBSERVER_MAX_FUTURE_SKEW_SEC}" \
  "STATE_TIMING_MAX_RAW_SKEW_SEC=${STATE_TIMING_MAX_RAW_SKEW_SEC}" \
  "STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=${STATE_TIMING_MAX_INTERPOLATION_GAP_SEC}" \
  "STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC=${STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC}" \
  "EXECUTION_CONTRACT_MAX_DELTA_V=${EXECUTION_CONTRACT_MAX_DELTA_V}" \
  "EXECUTION_CONTRACT_MAX_DELTA_OMEGA=${EXECUTION_CONTRACT_MAX_DELTA_OMEGA}" \
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
case "${IMU_SUBSCRIBER_QUEUE_SIZE}" in
  ''|*[!0-9]*) fail "IMU_SUBSCRIBER_QUEUE_SIZE must be an integer in [1,1000], got '${IMU_SUBSCRIBER_QUEUE_SIZE}'" ;;
esac
if (( IMU_SUBSCRIBER_QUEUE_SIZE < 1 || IMU_SUBSCRIBER_QUEUE_SIZE > 1000 )); then
  fail "IMU_SUBSCRIBER_QUEUE_SIZE must be in [1,1000], got '${IMU_SUBSCRIBER_QUEUE_SIZE}'"
fi
if truthy "${OBSERVER_LATCH_FALLBACK}"; then
  OBSERVER_LATCH_FALLBACK=true
else
  OBSERVER_LATCH_FALLBACK=false
fi
if truthy "${IMU_SHADOW_ENABLE}"; then
  require_number "IMU_SHADOW_READY_TIMEOUT_SEC" "${IMU_SHADOW_READY_TIMEOUT_SEC}"
  require_number "RECORDER_ACTIVE_TIMEOUT_SEC" "${RECORDER_ACTIVE_TIMEOUT_SEC}"
  if ! awk -v value="${IMU_SHADOW_READY_TIMEOUT_SEC}" 'BEGIN { exit !(value > 0.0) }'; then
    fail "IMU_SHADOW_READY_TIMEOUT_SEC must be > 0, got '${IMU_SHADOW_READY_TIMEOUT_SEC}'"
  fi
  if ! awk -v value="${RECORDER_ACTIVE_TIMEOUT_SEC}" 'BEGIN { exit !(value > 0.0) }'; then
    fail "RECORDER_ACTIVE_TIMEOUT_SEC must be > 0, got '${RECORDER_ACTIVE_TIMEOUT_SEC}'"
  fi
fi
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
if truthy "${IMU_SHADOW_ENABLE}" && \
   ! timeout 5s rostopic echo -n 1 "${IMU_TOPIC}" >/dev/null 2>&1; then
  fail "IMU shadow is enabled but no message arrived on ${IMU_TOPIC} within 5s"
fi

mkdir -p "${RUN_OUT_DIR}"
[[ ! -e "${RUN_OUT_DIR}/${NAME}.bag" && ! -e "${RUN_OUT_DIR}/${NAME}.bag.active" ]] || \
  fail "Run output already exists for NAME=${NAME}; use the protocol-authorized next repeat label"
if [[ "${PATH_SOURCE_MODE}" == "generate" ]]; then
  mkdir -p "$(dirname "${PATH_FILE}")"
else
  PATH_ACTUAL_SHA256="$(sha256sum "${PATH_FILE}" | awk '{print $1}')"
  path_hash_pass=false
  if [[ -n "${PATH_EXPECTED_SHA256}" ]]; then
    [[ "${PATH_ACTUAL_SHA256,,}" == "${PATH_EXPECTED_SHA256,,}" ]] || \
      fail "Frozen path SHA-256 mismatch: expected=${PATH_EXPECTED_SHA256}, actual=${PATH_ACTUAL_SHA256}"
    path_hash_pass=true
  elif truthy "${REQUIRE_PATH_HASH}"; then
    fail "REQUIRE_PATH_HASH=true but PATH_EXPECTED_SHA256 is empty"
  fi
  {
    echo "pass=${path_hash_pass}"
    echo "path_file=${PATH_FILE}"
    echo "path_expected_sha256=${PATH_EXPECTED_SHA256,,}"
    echo "path_actual_sha256=${PATH_ACTUAL_SHA256,,}"
  } > "${RUN_OUT_DIR}/${NAME}_path_sha256.txt"
fi

path_generator_log="${RUN_OUT_DIR}/${NAME}_path_generator.log"
send_goal_log="${RUN_OUT_DIR}/${NAME}_send_goal.log"
recorder_log="${RUN_OUT_DIR}/${NAME}_recorder.log"
planner_log="${RUN_OUT_DIR}/${NAME}_planner.log"
imu_shadow_ready_log="${RUN_OUT_DIR}/${NAME}_imu_shadow_ready.log"
recorder_active_bag="${RUN_OUT_DIR}/${NAME}.bag.active"

path_generator_pid=""
recorder_pid=""
planner_pid=""
gate_wait_pid=""
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

signal_child() {
  local pid="$1"
  local label="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    echo "[cleanup] signaling ${label} (pid=${pid})"
    kill -INT "${pid}" 2>/dev/null || true
  fi
}

reap_child() {
  local pid="$1"
  if [[ -n "${pid}" ]]; then
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
  # Stop all possible command/path producers first, publish zero immediately,
  # then reap them and publish zero once more before closing the recorder.
  signal_child "${gate_wait_pid}" "topic wait"
  signal_child "${path_generator_pid}" "path source"
  signal_child "${planner_pid}" "planner"
  publish_zero_cmd
  reap_child "${gate_wait_pid}"
  reap_child "${path_generator_pid}"
  reap_child "${planner_pid}"
  publish_zero_cmd
  signal_child "${recorder_pid}" "recorder"
  reap_child "${recorder_pid}"
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
  "imu_topic:=${IMU_TOPIC}"
  "imu_shadow_enable:=${IMU_SHADOW_ENABLE}"
  "imu_subscriber_queue_size:=${IMU_SUBSCRIBER_QUEUE_SIZE}"
  "observer_source:=${CURRENT_OBSERVER_SOURCE}"
  "observer_fallback_policy:=${OBSERVER_FALLBACK_POLICY}"
  "observer_latch_fallback:=${OBSERVER_LATCH_FALLBACK}"
  "observer_max_imu_state_age_sec:=${OBSERVER_MAX_IMU_STATE_AGE_SEC}"
  "observer_max_odom_state_age_sec:=${OBSERVER_MAX_ODOM_STATE_AGE_SEC}"
  "observer_max_future_skew_sec:=${OBSERVER_MAX_FUTURE_SKEW_SEC}"
  "state_timing_require_common_epoch:=${STATE_TIMING_REQUIRE_COMMON_EPOCH}"
  "state_timing_max_raw_skew_sec:=${STATE_TIMING_MAX_RAW_SKEW_SEC}"
  "state_timing_max_interpolation_gap_sec:=${STATE_TIMING_MAX_INTERPOLATION_GAP_SEC}"
  "state_timing_max_robot_extrapolation_sec:=${STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC}"
  "execution_contract_fail_closed_on_post_limit_change:=${EXECUTION_CONTRACT_FAIL_CLOSED}"
  "execution_contract_max_post_limit_delta_v:=${EXECUTION_CONTRACT_MAX_DELTA_V}"
  "execution_contract_max_post_limit_delta_omega:=${EXECUTION_CONTRACT_MAX_DELTA_OMEGA}"
  "v_ref:=${V_REF}"
  "w_slosh:=${W_SLOSH}"
  "w_smooth:=${W_SMOOTH}"
  "w_alpha:=${W_ALPHA}"
  "w_du_a:=${W_DU_A}"
  "w_du_vs:=${W_DU_VS}"
  "slosh_height_max:=${SLOSH_HEIGHT_MAX}"
  "alpha_max:=${ALPHA_MAX}"
  "shared_linear_accel_limit_enable:=${SHARED_LINEAR_ACCEL_LIMIT_ENABLE}"
  "shared_linear_accel_max:=${SHARED_LINEAR_ACCEL_MAX}"
  "shared_angular_limit_enable:=${SHARED_ANGULAR_LIMIT_ENABLE}"
  "shared_angular_rate_max:=${SHARED_ANGULAR_RATE_MAX}"
  "shared_angular_accel_max:=${SHARED_ANGULAR_ACCEL_MAX}"
)
planner_command_string="$(printf '%q ' "${planner_cmd[@]}")"

if [[ "${PATH_SOURCE_MODE}" == "generate" ]]; then
  path_cmd=(
    rosrun scout_local_planner template_fixed_path_generator.py
    --template "${PATH_TEMPLATE}"
    --goal-topic "${GOAL_TOPIC}"
    --output-topic "${REF_TOPIC}"
    --path-file "${PATH_FILE}"
    --start-heading current
    --spacing "${PATH_SPACING}"
    --amplitude-ratio "${PATH_AMPLITUDE_RATIO}"
    --min-amplitude "${PATH_MIN_AMPLITUDE}"
    --max-amplitude "${PATH_MAX_AMPLITUDE}"
    --side "${PATH_SIDE}"
    --smooth-iterations "${PATH_SMOOTH_ITERATIONS}"
    --publish-count 0
  )
else
  path_cmd=(
    rosrun scout_local_planner fixed_global_path_runner.py
    --mode replay
    --path-file "${PATH_FILE}"
    --output-topic "${REF_TOPIC}"
    --base-frame "${BASE_FRAME}"
    --start-pos-tol "${START_POS_TOL}"
    --start-yaw-tol "${START_YAW_TOL}"
    --start-hold-sec "${START_HOLD_SEC}"
    --publish-rate "${PATH_PUBLISH_RATE}"
    --publish-count 0
  )
fi
path_command_string="$(printf '%q ' "${path_cmd[@]}")"

run_meta="${RUN_OUT_DIR}/${NAME}_one_click_meta.env"
{
  echo "date=${DATE}"
  echo "stamp=${STAMP}"
  echo "variant=${VARIANT}"
  echo "run_class=${RUN_CLASS}"
  echo "pilot_mode=${PILOT_MODE}"
  echo "pilot_method=${PILOT_METHOD}"
  echo "pilot_condition=${PILOT_CONDITION}"
  echo "block_segment_id=${BLOCK_SEGMENT_ID}"
  echo "split_block=${SPLIT_BLOCK}"
  echo "order_position=${ORDER_POSITION}"
  echo "acquisition_retry=${ACQUISITION_RETRY}"
  echo "retry_reason_file=${RETRY_REASON_FILE}"
  echo "allow_pilot_path_overwrite=${ALLOW_PILOT_PATH_OVERWRITE}"
  echo "run_label=${RUN_LABEL}"
  echo "name=${NAME}"
  echo "record_sec=${RECORD_SEC}"
  echo "max_record_sec=${MAX_RECORD_SEC}"
  echo "run_out_dir=${RUN_OUT_DIR}"
  echo "path_file=${PATH_FILE}"
  echo "path_expected_sha256=${PATH_EXPECTED_SHA256}"
  echo "path_actual_sha256=${PATH_ACTUAL_SHA256}"
  echo "require_path_hash=${REQUIRE_PATH_HASH}"
  echo "path_source_mode=${PATH_SOURCE_MODE}"
  echo "path_command=${path_command_string}"
  echo "ref_topic=${REF_TOPIC}"
  echo "base_frame=${BASE_FRAME}"
  echo "start_pos_tol=${START_POS_TOL}"
  echo "start_yaw_tol=${START_YAW_TOL}"
  echo "start_hold_sec=${START_HOLD_SEC}"
  echo "start_gate_timeout_sec=${START_GATE_TIMEOUT_SEC}"
  echo "goal_topic=${GOAL_TOPIC}"
  echo "goal_frame=${GOAL_FRAME}"
  echo "goal_x=${GOAL_X}"
  echo "goal_y=${GOAL_Y}"
  echo "goal_yaw=${GOAL_YAW}"
  echo "cmd_topic=${CMD_TOPIC}"
  echo "solver_backend=${SOLVER_BACKEND}"
  echo "v_ref=${V_REF}"
  echo "w_slosh=${W_SLOSH}"
  echo "w_smooth=${W_SMOOTH}"
  echo "w_alpha=${W_ALPHA}"
  echo "w_du_a=${W_DU_A}"
  echo "w_du_vs=${W_DU_VS}"
  echo "slosh_height_max=${SLOSH_HEIGHT_MAX}"
  echo "delay_phase_mode=${DELAY_PHASE_MODE}"
  echo "delay_phase_linear_delay_sec=${DELAY_PHASE_LINEAR_DELAY_SEC}"
  echo "delay_phase_angular_delay_sec=${DELAY_PHASE_ANGULAR_DELAY_SEC}"
  echo "imu_shadow_enable=${IMU_SHADOW_ENABLE}"
  echo "imu_topic=${IMU_TOPIC}"
  echo "imu_subscriber_queue_size=${IMU_SUBSCRIBER_QUEUE_SIZE}"
  echo "imu_shadow_ready_topic=${IMU_SHADOW_READY_TOPIC}"
  echo "imu_shadow_ready_timeout_sec=${IMU_SHADOW_READY_TIMEOUT_SEC}"
  echo "current_observer_source=${CURRENT_OBSERVER_SOURCE}"
  echo "observer_fallback_policy=${OBSERVER_FALLBACK_POLICY}"
  echo "observer_latch_fallback=${OBSERVER_LATCH_FALLBACK}"
  echo "observer_max_imu_state_age_sec=${OBSERVER_MAX_IMU_STATE_AGE_SEC}"
  echo "observer_max_odom_state_age_sec=${OBSERVER_MAX_ODOM_STATE_AGE_SEC}"
  echo "observer_max_future_skew_sec=${OBSERVER_MAX_FUTURE_SKEW_SEC}"
  echo "state_timing_require_common_epoch=${STATE_TIMING_REQUIRE_COMMON_EPOCH}"
  echo "state_timing_max_raw_skew_sec=${STATE_TIMING_MAX_RAW_SKEW_SEC}"
  echo "state_timing_max_interpolation_gap_sec=${STATE_TIMING_MAX_INTERPOLATION_GAP_SEC}"
  echo "state_timing_max_robot_extrapolation_sec=${STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC}"
  echo "execution_contract_fail_closed=${EXECUTION_CONTRACT_FAIL_CLOSED}"
  echo "execution_contract_max_delta_v=${EXECUTION_CONTRACT_MAX_DELTA_V}"
  echo "execution_contract_max_delta_omega=${EXECUTION_CONTRACT_MAX_DELTA_OMEGA}"
  echo "observer_selection_topic=${OBSERVER_SELECTION_TOPIC}"
  echo "recorder_active_timeout_sec=${RECORDER_ACTIVE_TIMEOUT_SEC}"
  echo "record_rgb=${RECORD_RGB}"
  echo "record_camera=${RECORD_CAMERA}"
  echo "record_camera_info=${RECORD_CAMERA_INFO}"
  echo "record_camera_compressed=${RECORD_CAMERA_COMPRESSED}"
  echo "record_depth=${RECORD_DEPTH}"
  echo "record_online_liquid=${RECORD_ONLINE_LIQUID}"
  echo "record_online_liquid_debug_images=${RECORD_ONLINE_LIQUID_DEBUG_IMAGES}"
  echo "forbid_image_streams=${FORBID_IMAGE_STREAMS}"
  echo "rgb_calibration_file=${RGB_CALIBRATION_FILE}"
  echo "rgb_calibration_expected_sha256=${RGB_CALIBRATION_EXPECTED_SHA256}"
  echo "rgb_calibration_actual_sha256=${RGB_CALIBRATION_ACTUAL_SHA256}"
  echo "rgb_expected_width=${RGB_EXPECTED_WIDTH}"
  echo "rgb_expected_height=${RGB_EXPECTED_HEIGHT}"
  echo "rgb_expected_fps=${RGB_EXPECTED_FPS}"
  echo "online_liquid_measurement_topic=${ONLINE_LIQUID_MEASUREMENT_TOPIC}"
  echo "online_liquid_protocol=${ONLINE_LIQUID_PROTOCOL}"
  echo "online_liquid_calibration_sha256=${ONLINE_LIQUID_CALIBRATION_SHA256}"
  echo "online_liquid_detector_sha256=${ONLINE_LIQUID_DETECTOR_SHA256}"
  echo "online_liquid_node_sha256=${ONLINE_LIQUID_NODE_SHA256}"
  echo "online_liquid_msg_sha256=${ONLINE_LIQUID_MSG_SHA256}"
  echo "online_liquid_config_sha256=${ONLINE_LIQUID_CONFIG_SHA256}"
  echo "planner_command=${planner_command_string}"
} > "${run_meta}"

echo "================ SPMPC real fixed-path trial ================"
echo "  variant       = ${VARIANT}"
echo "  pilot         = ${PILOT_MODE} (${PILOT_METHOD:-direct parameters})"
echo "  condition     = ${PILOT_CONDITION:-n/a}"
echo "  run_label     = ${RUN_LABEL}"
echo "  block_segment = ${BLOCK_SEGMENT_ID:-n/a} split=${SPLIT_BLOCK} position=${ORDER_POSITION:-n/a} retry=${ACQUISITION_RETRY}"
echo "  cmd_topic     = ${CMD_TOPIC}"
echo "  recorder      = ${RECORD_SEC}s max (Ctrl+C stops earlier)"
echo "  out_dir       = ${RUN_OUT_DIR}"
echo "  record_rgb    = ${RECORD_RGB}"
echo "  online_liquid = ${RECORD_ONLINE_LIQUID} (debug images ${RECORD_ONLINE_LIQUID_DEBUG_IMAGES})"
echo "  forbid_images = ${FORBID_IMAGE_STREAMS}"
echo "  path_source   = ${PATH_SOURCE_MODE}"
echo "  path_file     = ${PATH_FILE}"
if [[ "${PATH_SOURCE_MODE}" == "replay" ]]; then
  echo "  start_gate    = ${START_POS_TOL} m / ${START_YAW_TOL} rad, hold ${START_HOLD_SEC}s"
else
  echo "  goal          = (${GOAL_X}, ${GOAL_Y}, ${GOAL_YAW}) in ${GOAL_FRAME}"
fi
echo "  v_ref/w_slosh = ${V_REF} / ${W_SLOSH}"
echo "  imu_shadow    = ${IMU_SHADOW_ENABLE} (${IMU_TOPIC})"
echo "  observer      = nominal ${CURRENT_OBSERVER_SOURCE}, fallback ${OBSERVER_FALLBACK_POLICY}, latch ${OBSERVER_LATCH_FALLBACK}"
if truthy "${IMU_SHADOW_ENABLE}"; then
  echo "  shadow gate   = ${IMU_SHADOW_READY_TOPIC}, timeout ${IMU_SHADOW_READY_TIMEOUT_SEC}s"
fi
echo "============================================================="

require_shadow_topics_idle() {
  local published_topics
  if ! published_topics="$(timeout --foreground 5s rostopic list -p)"; then
    fail "Could not query the ROS master publisher state for the IMU shadow safety gate"
  fi

  local topic purpose
  local guarded_topics=("${REF_TOPIC}" "${CMD_TOPIC}" "${IMU_SHADOW_READY_TOPIC}" "${OBSERVER_SELECTION_TOPIC}")
  local guarded_purposes=("Reference topic" "Command topic" "IMU READY topic" "Observer selection topic")
  if [[ "${PATH_SOURCE_MODE}" == "generate" ]]; then
    guarded_topics+=("${GOAL_TOPIC}")
    guarded_purposes+=("Goal topic")
  fi
  local index
  for index in "${!guarded_topics[@]}"; do
    topic="${guarded_topics[index]}"
    purpose="${guarded_purposes[index]}"
    if grep -Fxq -- "${topic}" <<< "${published_topics}"; then
      fail "${purpose} ${topic} already has a publisher; stop the stale/conflicting node before enabling the IMU shadow startup gate"
    fi
  done
}

recorder_actively_recording() {
  child_running "${recorder_pid}" && [[ -e "${recorder_active_bag}" ]]
}

require_shadow_run_active() {
  local context="$1"
  if ! recorder_actively_recording; then
    show_log_tail "${recorder_log}" "recorder"
    fail "Rosbag stopped actively recording during ${context}; stopping before an unrecorded motion trial"
  fi
  if ! child_running "${planner_pid}"; then
    show_log_tail "${planner_log}" "planner"
    fail "Planner stopped during ${context}; no motion trial was continued"
  fi
}

start_path_source() {
  local monitor_active_run="$1"
  echo "[path] starting ${PATH_SOURCE_MODE} source -> ${REF_TOPIC}"
  "${path_cmd[@]}" > "${path_generator_log}" 2>&1 &
  path_generator_pid=$!
  if truthy "${monitor_active_run}"; then
    sleep "${PATH_GENERATOR_STARTUP_SEC}" &
    gate_wait_pid=$!
    while child_running "${gate_wait_pid}"; do
      require_shadow_run_active "path-source startup"
      if ! child_running "${path_generator_pid}"; then
        kill_child "${gate_wait_pid}" "path-source startup wait"
        gate_wait_pid=""
        show_log_tail "${path_generator_log}" "path source"
        fail "Path source exited during startup (mode=${PATH_SOURCE_MODE})"
      fi
      sleep 0.1
    done
    reap_child "${gate_wait_pid}"
    gate_wait_pid=""
    require_shadow_run_active "path-source startup"
  else
    sleep "${PATH_GENERATOR_STARTUP_SEC}"
  fi
  if ! child_running "${path_generator_pid}"; then
    local path_code
    set +e
    wait "${path_generator_pid}"
    path_code=$?
    set -e
    show_log_tail "${path_generator_log}" "path source"
    fail "Path source exited during startup (mode=${PATH_SOURCE_MODE}, code=${path_code})"
  fi
}

wait_for_reference() {
  local timeout_sec="$1"
  local description="$2"
  local monitor_active_run="$3"
  local wait_code=0

  if ! truthy "${monitor_active_run}"; then
    if ! timeout --foreground "${timeout_sec}s" rostopic echo -n 1 "${REF_TOPIC}" >/dev/null; then
      return 1
    fi
    return 0
  fi

  timeout --foreground "${timeout_sec}s" rostopic echo -n 1 "${REF_TOPIC}" >/dev/null 2>&1 &
  gate_wait_pid=$!
  while child_running "${gate_wait_pid}"; do
    if ! recorder_actively_recording || ! child_running "${planner_pid}"; then
      # Let the EXIT cleanup signal the topic wait, path source, and planner
      # together; do not synchronously reap this low-priority waiter first.
      require_shadow_run_active "${description}"
    fi
    sleep 0.1
  done
  set +e
  wait "${gate_wait_pid}"
  wait_code=$?
  set -e
  gate_wait_pid=""
  if (( wait_code != 0 )); then
    return 1
  fi
  require_shadow_run_active "completion of ${description}"
  if ! child_running "${path_generator_pid}"; then
    show_log_tail "${path_generator_log}" "path source"
    fail "Path source stopped at completion of ${description}"
  fi
  return 0
}

prepare_reference() {
  local monitor_active_run="$1"
  if [[ "${PATH_SOURCE_MODE}" == "replay" ]]; then
    echo "[path] waiting up to ${START_GATE_TIMEOUT_SEC}s for the relaxed start gate and ${REF_TOPIC}"
    if ! wait_for_reference "${START_GATE_TIMEOUT_SEC}" "replay start gate/path" "${monitor_active_run}"; then
      show_log_tail "${path_generator_log}" "fixed-path replay"
      fail "Timed out waiting for replay start gate/path on ${REF_TOPIC}"
    fi
    if ! child_running "${path_generator_pid}"; then
      show_log_tail "${path_generator_log}" "fixed-path replay"
      fail "Fixed-path replay stopped after publishing the path"
    fi
    return 0
  fi

  local goal_cmd=(
    rosrun scout_local_planner send_fixed_goal.py
    --goal-topic "${GOAL_TOPIC}"
    --frame "${GOAL_FRAME}"
    --x "${GOAL_X}"
    --y "${GOAL_Y}"
    --yaw "${GOAL_YAW}"
    --repeat-count "${GOAL_REPEAT_COUNT}"
    --repeat-rate "${GOAL_REPEAT_RATE}"
  )
  echo "[goal] sending fixed goal"
  local goal_code=0
  if truthy "${monitor_active_run}"; then
    "${goal_cmd[@]}" > "${send_goal_log}" 2>&1 &
    gate_wait_pid=$!
    while child_running "${gate_wait_pid}"; do
      require_shadow_run_active "fixed-goal publication"
      if ! child_running "${path_generator_pid}"; then
        kill_child "${gate_wait_pid}" "fixed-goal publisher"
        gate_wait_pid=""
        show_log_tail "${path_generator_log}" "path generator"
        fail "Path generator stopped during fixed-goal publication"
      fi
      sleep 0.1
    done
    set +e
    wait "${gate_wait_pid}"
    goal_code=$?
    set -e
    gate_wait_pid=""
    require_shadow_run_active "completion of fixed-goal publication"
  else
    set +e
    "${goal_cmd[@]}" > "${send_goal_log}" 2>&1
    goal_code=$?
    set -e
  fi
  if (( goal_code != 0 )); then
    show_log_tail "${send_goal_log}" "send fixed goal"
    fail "Failed to send fixed goal"
  fi

  echo "[path] waiting for ${REF_TOPIC}"
  if ! wait_for_reference "${GENERATED_PATH_WAIT_SEC}" "generated fixed path" "${monitor_active_run}"; then
    show_log_tail "${path_generator_log}" "path generator"
    fail "Timed out waiting for generated fixed path on ${REF_TOPIC}"
  fi
  if ! child_running "${path_generator_pid}"; then
    show_log_tail "${path_generator_log}" "path generator"
    fail "Path generator stopped after goal; fixed path may not remain available"
  fi
}

start_recorder() {
  echo "[record] starting black-box recorder"
  (
  cd "${REPO_ROOT}"
  DATE="${DATE}" \
  STAMP="${STAMP}" \
  VARIANT="${VARIANT}" \
  RUN_CLASS="${RUN_CLASS}" \
  PILOT_MODE="${PILOT_MODE}" \
  PILOT_METHOD="${PILOT_METHOD}" \
  PILOT_CONDITION="${PILOT_CONDITION}" \
  BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID}" \
  SPLIT_BLOCK="${SPLIT_BLOCK}" \
  ORDER_POSITION="${ORDER_POSITION}" \
  ACQUISITION_RETRY="${ACQUISITION_RETRY}" \
  RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
  RUN_LABEL="${RUN_LABEL}" \
  RECORD_SEC="${RECORD_SEC}" \
  OUT_DIR="${RUN_OUT_DIR}" \
  NAME="${NAME}" \
  RECORD_RGB="${RECORD_RGB}" \
  RECORD_CAMERA="${RECORD_CAMERA}" \
  RECORD_CAMERA_INFO="${RECORD_CAMERA_INFO}" \
  RECORD_CAMERA_COMPRESSED="${RECORD_CAMERA_COMPRESSED}" \
  RECORD_SCAN="${RECORD_SCAN}" \
  RECORD_DEPTH="${RECORD_DEPTH}" \
  RECORD_STANDALONE_SLOSH="${RECORD_STANDALONE_SLOSH}" \
  RECORD_ONLINE_LIQUID="${RECORD_ONLINE_LIQUID}" \
  RECORD_ONLINE_LIQUID_DEBUG_IMAGES="${RECORD_ONLINE_LIQUID_DEBUG_IMAGES}" \
  FORBID_IMAGE_STREAMS="${FORBID_IMAGE_STREAMS}" \
  RECORD_ALL_EXISTING_TOPICS="${RECORD_ALL_EXISTING_TOPICS}" \
  RECORD_TOPIC_INFO="${RECORD_TOPIC_INFO}" \
  LIQUID_CALIBRATION="${RGB_CALIBRATION_FILE}" \
  RGB_CALIBRATION_EXPECTED_SHA256="${RGB_CALIBRATION_EXPECTED_SHA256}" \
  RGB_CALIBRATION_ACTUAL_SHA256="${RGB_CALIBRATION_ACTUAL_SHA256}" \
  RGB_EXPECTED_WIDTH="${RGB_EXPECTED_WIDTH}" \
  RGB_EXPECTED_HEIGHT="${RGB_EXPECTED_HEIGHT}" \
  RGB_EXPECTED_FPS="${RGB_EXPECTED_FPS}" \
  ONLINE_LIQUID_MEASUREMENT_TOPIC="${ONLINE_LIQUID_MEASUREMENT_TOPIC}" \
  ONLINE_LIQUID_PROTOCOL="${ONLINE_LIQUID_PROTOCOL}" \
  ONLINE_LIQUID_CALIBRATION_SHA256="${ONLINE_LIQUID_CALIBRATION_SHA256}" \
  ONLINE_LIQUID_DETECTOR_SHA256="${ONLINE_LIQUID_DETECTOR_SHA256}" \
  ONLINE_LIQUID_NODE_SHA256="${ONLINE_LIQUID_NODE_SHA256}" \
  ONLINE_LIQUID_MSG_SHA256="${ONLINE_LIQUID_MSG_SHA256}" \
  ONLINE_LIQUID_CONFIG_SHA256="${ONLINE_LIQUID_CONFIG_SHA256}" \
  SOLVER_BACKEND="${SOLVER_BACKEND}" \
  V_REF="${V_REF}" \
  W_SLOSH="${W_SLOSH}" \
  W_SMOOTH="${W_SMOOTH}" \
  W_ALPHA="${W_ALPHA}" \
  W_DU_A="${W_DU_A}" \
  W_DU_VS="${W_DU_VS}" \
  SLOSH_HEIGHT_MAX="${SLOSH_HEIGHT_MAX}" \
  DELAY_PHASE_MODE="${DELAY_PHASE_MODE}" \
  DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
  DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
  IMU_SHADOW_ENABLE="${IMU_SHADOW_ENABLE}" \
  IMU_TOPIC="${IMU_TOPIC}" \
  IMU_SUBSCRIBER_QUEUE_SIZE="${IMU_SUBSCRIBER_QUEUE_SIZE}" \
  CURRENT_OBSERVER_SOURCE="${CURRENT_OBSERVER_SOURCE}" \
  OBSERVER_FALLBACK_POLICY="${OBSERVER_FALLBACK_POLICY}" \
  OBSERVER_LATCH_FALLBACK="${OBSERVER_LATCH_FALLBACK}" \
  OBSERVER_MAX_IMU_STATE_AGE_SEC="${OBSERVER_MAX_IMU_STATE_AGE_SEC}" \
  OBSERVER_MAX_ODOM_STATE_AGE_SEC="${OBSERVER_MAX_ODOM_STATE_AGE_SEC}" \
  OBSERVER_MAX_FUTURE_SKEW_SEC="${OBSERVER_MAX_FUTURE_SKEW_SEC}" \
  GOAL_X="${GOAL_X}" \
  GOAL_Y="${GOAL_Y}" \
  GOAL_YAW="${GOAL_YAW}" \
  PATH_SOURCE_MODE="${PATH_SOURCE_MODE}" \
  PATH_FILE="${PATH_FILE}" \
  PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256}" \
  PATH_ACTUAL_SHA256="${PATH_ACTUAL_SHA256}" \
  REQUIRE_PATH_HASH="${REQUIRE_PATH_HASH}" \
  START_POS_TOL="${START_POS_TOL}" \
  START_YAW_TOL="${START_YAW_TOL}" \
  START_HOLD_SEC="${START_HOLD_SEC}" \
  LAUNCH_COMMAND="${planner_command_string}" \
  OPERATOR_NOTE="${OPERATOR_NOTE}" \
  bash "${RECORDER_SCRIPT}"
  ) > "${recorder_log}" 2>&1 &
  recorder_pid=$!
  sleep "${RECORDER_STARTUP_SEC}"
  if ! child_running "${recorder_pid}"; then
    local recorder_code
    set +e
    wait "${recorder_pid}"
    recorder_code=$?
    set -e
    show_log_tail "${recorder_log}" "recorder"
    fail "Recorder exited during startup (code=${recorder_code})"
  fi
  if truthy "${IMU_SHADOW_ENABLE}"; then
    if ! timeout --foreground "${RECORDER_ACTIVE_TIMEOUT_SEC}s" \
      bash -c 'while [[ ! -e "$1" ]]; do sleep 0.1; done' _ "${recorder_active_bag}"; then
      show_log_tail "${recorder_log}" "recorder"
      fail "Recorder did not create ${recorder_active_bag} within ${RECORDER_ACTIVE_TIMEOUT_SEC}s; refusing to start the IMU bias window"
    fi
    if ! recorder_actively_recording; then
      show_log_tail "${recorder_log}" "recorder"
      fail "Recorder stopped immediately after creating ${recorder_active_bag}; no motion trial was started"
    fi
  fi
}

start_planner() {
  echo "[launch] starting planner"
  "${planner_cmd[@]}" > "${planner_log}" 2>&1 &
  planner_pid=$!
  sleep "${PLANNER_STARTUP_SEC}"
  if ! child_running "${planner_pid}"; then
    local planner_code
    set +e
    wait "${planner_pid}"
    planner_code=$?
    set -e
    show_log_tail "${planner_log}" "planner"
    fail "Planner exited during startup (code=${planner_code})"
  fi
}

wait_for_imu_shadow_ready() {
  echo "[imu-shadow] keep the robot stationary; waiting for calibrated and filtered READY"
  if ! timeout --foreground "${IMU_SHADOW_READY_TIMEOUT_SEC}s" \
    rostopic echo -n 1 \
      --filter "m.input_status == 'READY' and m.valid and m.bias_ready and m.filter_ready" \
      "${IMU_SHADOW_READY_TOPIC}" > "${imu_shadow_ready_log}" 2>&1; then
    show_log_tail "${imu_shadow_ready_log}" "IMU shadow READY gate"
    echo "[debug] latest ${IMU_SHADOW_READY_TOPIC} sample:" >&2
    timeout --foreground 2s rostopic echo -n 1 "${IMU_SHADOW_READY_TOPIC}" >&2 || true
    show_log_tail "${planner_log}" "planner"
    fail "IMU shadow did not become READY within ${IMU_SHADOW_READY_TIMEOUT_SEC}s; the robot may have moved during the bias window"
  fi
  require_shadow_run_active "completion of the IMU shadow READY gate"
  echo "imu_shadow_ready_wall_time=$(date --iso-8601=seconds)" >> "${run_meta}"
  echo "[imu-shadow] READY; enabling the path/goal stage"
}

if truthy "${IMU_SHADOW_ENABLE}"; then
  # A pre-existing reference could make the planner move before calibration;
  # a competing command publisher or stale debug publisher is equally unsafe.
  require_shadow_topics_idle

  # Keep the entire bias/filter transient in the bag.  With no reference path,
  # the planner's control loop explicitly publishes a zero command.
  start_recorder
  # Close the recorder-startup TOCTOU window immediately before planner launch.
  require_shadow_topics_idle
  start_planner
  wait_for_imu_shadow_ready
  require_shadow_run_active "release of the path/goal stage"
  start_path_source true
  prepare_reference true
else
  # Preserve the established odom-only experiment order byte-for-byte in
  # behavior: path/gate -> recorder -> goal (generate only) -> planner.
  start_path_source false
  if [[ "${PATH_SOURCE_MODE}" == "replay" ]]; then
    prepare_reference false
  fi
  start_recorder
  if [[ "${PATH_SOURCE_MODE}" == "generate" ]]; then
    prepare_reference false
  fi
  start_planner
fi

if [[ "${PATH_SOURCE_MODE}" == "replay" ]]; then
  echo "[goal] replay mode uses the frozen path directly; fixed-goal generation is skipped"
fi

echo "[run] recording until Ctrl+C, ${RECORD_SEC}s recorder timeout, or planner exit"
if truthy "${IMU_SHADOW_ENABLE}"; then
  # The recorder wrapper performs metadata work after rosbag closes, so its PID
  # can remain alive after the .bag.active file disappears.  Stop motion at the
  # actual recording boundary, then allow metadata finalization to finish.
  while recorder_actively_recording && child_running "${planner_pid}"; do
    sleep 0.1
  done
  if recorder_actively_recording && ! child_running "${planner_pid}"; then
    show_log_tail "${planner_log}" "planner"
    echo "[ERR] planner exited while rosbag was active; stopping recorder/path source" >&2
    recorder_code=1
  else
    echo "[run] rosbag active recording ended; stopping planner/path source before recorder post-processing"
    signal_child "${path_generator_pid}" "path source"
    signal_child "${planner_pid}" "planner"
    publish_zero_cmd
    reap_child "${path_generator_pid}"
    reap_child "${planner_pid}"
    path_generator_pid=""
    planner_pid=""
    publish_zero_cmd
    set +e
    wait "${recorder_pid}"
    recorder_code=$?
    set -e
    recorder_pid=""
    if (( recorder_code != 0 )); then
      show_log_tail "${recorder_log}" "recorder"
    fi
  fi
else
  set +e
  wait -n "${recorder_pid}" "${planner_pid}"
  first_exit_code=$?
  set -e
  if child_running "${recorder_pid}"; then
    show_log_tail "${planner_log}" "planner"
    echo "[ERR] planner exited before recorder timeout (code=${first_exit_code}); stopping recorder/path source" >&2
    recorder_code=1
  else
    recorder_code=${first_exit_code}
    if (( recorder_code != 0 )); then
      show_log_tail "${recorder_log}" "recorder"
    fi
    echo "[run] recorder exited with code ${recorder_code}; stopping planner/path source"
  fi
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
