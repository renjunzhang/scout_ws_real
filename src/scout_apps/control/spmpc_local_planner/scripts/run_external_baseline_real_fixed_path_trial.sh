#!/usr/bin/env bash
# One-click external-baseline fixed-path real trial wrapper.
# Assumes the real sensor/base/localization stack is already running.
# This script starts the standalone slosh evaluator, starts the fixed-path
# generator and black-box bag recorder, sends the goal, waits for the fixed
# path, then launches one external baseline in shadow or actuated mode. The
# recorder bounds run duration; Ctrl+C stops earlier.

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

cmd_vel_publishers_present() {
  local info="$1"
  local in_publishers=false
  local line
  while IFS= read -r line; do
    if [[ "${line}" == Publishers:* ]]; then
      [[ "${line}" == *None* ]] && return 1
      in_publishers=true
      continue
    fi
    if [[ "${line}" == Subscribers:* ]]; then
      break
    fi
    if [[ "${in_publishers}" == true && "${line}" =~ ^[[:space:]]*\* ]]; then
      return 0
    fi
  done <<< "${info}"
  return 1
}

check_cmd_vel_clear_for_actuated() {
  [[ "${STAGE}" == "actuated" ]] || return 0
  local info
  info="$(timeout 5s rostopic info /cmd_vel 2>&1 || true)"
  if cmd_vel_publishers_present "${info}"; then
    echo "[ERR] /cmd_vel already has publisher(s); stop old planners before actuated run:" >&2
    printf '%s\n' "${info}" >&2
    exit 2
  fi
}

ros_node_exists() {
  local node_name="$1"
  timeout 5s rosnode list 2>/dev/null | grep -Fxq "${node_name}"
}

wait_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  timeout "${timeout_sec}s" rostopic echo -n 1 "${topic}" >/dev/null 2>&1
}

reset_slosh_monitor() {
  if ! timeout "${SLOSH_MONITOR_RESET_TIMEOUT_SEC}s" \
      rosservice call "${SLOSH_MONITOR_RESET_SERVICE}" >/dev/null 2>&1; then
    show_log_tail "${slosh_monitor_log}" "standalone slosh monitor"
    fail "Failed to reset standalone slosh monitor via ${SLOSH_MONITOR_RESET_SERVICE}"
  fi
  if ! wait_topic_once "${SLOSH_MONITOR_HEIGHT_TOPIC}" "${SLOSH_MONITOR_RESET_TIMEOUT_SEC}"; then
    show_log_tail "${slosh_monitor_log}" "standalone slosh monitor"
    fail "No ${SLOSH_MONITOR_HEIGHT_TOPIC} sample received after reset"
  fi
  echo "[slosh_monitor] reset ${SLOSH_MONITOR_RESET_SERVICE}"
}

prepare_slosh_monitor() {
  if ! truthy "${START_STANDALONE_SLOSH}" && ! truthy "${RECORD_STANDALONE_SLOSH}"; then
    echo "[slosh_monitor] disabled: START_STANDALONE_SLOSH=false and RECORD_STANDALONE_SLOSH=false"
    return 0
  fi

  if truthy "${START_STANDALONE_SLOSH}"; then
    if ros_node_exists "${SLOSH_MONITOR_NODE}"; then
      fail "${SLOSH_MONITOR_NODE} already exists; stop the old monitor or set START_STANDALONE_SLOSH=false to reuse it explicitly"
    fi

    echo "[slosh_monitor] starting ${SLOSH_MONITOR_NODE} from ${SLOSH_MONITOR_ODOM_TOPIC}"
    roslaunch slosh_models slosh_monitor.launch \
      "odom_topic:=${SLOSH_MONITOR_ODOM_TOPIC}" \
      "cmd_vel_topic:=${SLOSH_MONITOR_CMD_VEL_TOPIC}" \
      "output_namespace:=${SLOSH_MONITOR_OUTPUT_NAMESPACE}" \
      "container_radius:=${SLOSH_MONITOR_CONTAINER_RADIUS}" \
      "liquid_height:=${SLOSH_MONITOR_LIQUID_HEIGHT}" \
      "damping_ratio:=${SLOSH_MONITOR_DAMPING_RATIO}" \
      "use_parabola_term:=${SLOSH_MONITOR_USE_PARABOLA_TERM}" \
      "model_dt:=${SLOSH_MONITOR_MODEL_DT}" \
      "accel_filter_alpha:=${SLOSH_MONITOR_ACCEL_FILTER_ALPHA}" \
      "min_dt:=${SLOSH_MONITOR_MIN_DT}" \
      "max_dt:=${SLOSH_MONITOR_MAX_DT}" \
      > "${slosh_monitor_log}" 2>&1 &
    slosh_monitor_pid=$!
  else
    echo "[slosh_monitor] START_STANDALONE_SLOSH=false; requiring an externally managed monitor"
  fi

  if ! wait_topic_once "${SLOSH_MONITOR_HEIGHT_TOPIC}" "${SLOSH_MONITOR_STARTUP_TIMEOUT_SEC}"; then
    show_log_tail "${slosh_monitor_log}" "standalone slosh monitor"
    fail "Timed out waiting for ${SLOSH_MONITOR_HEIGHT_TOPIC}; check ${SLOSH_MONITOR_ODOM_TOPIC} and slosh_models build/source"
  fi
  if [[ -n "${slosh_monitor_pid}" ]] && ! child_running "${slosh_monitor_pid}"; then
    show_log_tail "${slosh_monitor_log}" "standalone slosh monitor"
    fail "Standalone slosh monitor exited during startup"
  fi

  reset_slosh_monitor
}

DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
METHOD="${METHOD:-${BASELINE:-}}"
STAGE="${STAGE:-${MODE:-shadow}}"

case "${METHOD}" in
  lt_dwa|ltdwa|official_lt_dwa|lt_dwa_official) METHOD="lt_dwa_official" ;;
  teb|TEB) METHOD="teb" ;;
  mpc|mpc_local|mpc_local_planner) METHOD="mpc_local_planner" ;;
  '') fail "METHOD is required: lt_dwa_official|teb|mpc_local_planner" ;;
  *) fail "METHOD must be lt_dwa_official|teb|mpc_local_planner, got '${METHOD}'" ;;
esac
case "${STAGE}" in
  shadow|actuated) ;;
  *) fail "STAGE must be shadow|actuated, got '${STAGE}'" ;;
esac

ALG="${ALG:-${METHOD}}"
RUN_LABEL="${RUN_LABEL:-real_fixed_${METHOD}_${STAGE}_${STAMP}}"
NAME="${NAME:-${RUN_LABEL}}"

MAX_RECORD_SEC="${MAX_RECORD_SEC:-60}"
RECORD_SEC="${RECORD_SEC:-60}"
case "${MAX_RECORD_SEC}" in
  ''|*[!0-9]*) fail "MAX_RECORD_SEC must be a positive integer, got '${MAX_RECORD_SEC}'" ;;
esac
case "${RECORD_SEC}" in
  ''|*[!0-9]*) fail "RECORD_SEC must be a positive integer, got '${RECORD_SEC}'" ;;
esac
if (( MAX_RECORD_SEC <= 0 )); then
  fail "MAX_RECORD_SEC must be > 0, got '${MAX_RECORD_SEC}'"
fi
if (( RECORD_SEC <= 0 || RECORD_SEC > MAX_RECORD_SEC )); then
  echo "[WARN] RECORD_SEC=${RECORD_SEC} is outside (0, ${MAX_RECORD_SEC}], forcing RECORD_SEC=${MAX_RECORD_SEC}." >&2
  RECORD_SEC="${MAX_RECORD_SEC}"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || { cd "${SCRIPT_DIR}/../../../../.." && pwd; })"
RECORDER_SCRIPT="${RECORDER_SCRIPT:-${SCRIPT_DIR}/record_spmpc_full_rgb_bag.sh}"

BAG_ROOT="${BAG_ROOT:-${HOME}/slosh_bags/real/${DATE}_fixed_path_compare}"
RUN_OUT_DIR="${RUN_OUT_DIR:-${BAG_ROOT}/${METHOD}}"
PATH_ROOT="${PATH_ROOT:-${HOME}/fixed_paths/real/${DATE}_fixed_path_compare}"
PATH_FILE="${PATH_FILE:-${PATH_ROOT}/fixed_s_curve_compare.json}"
REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
GOAL_FRAME="${GOAL_FRAME:-map}"
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

SHADOW_CMD_TOPIC="${SHADOW_CMD_TOPIC:-/spmpc_shadow_cmd_vel}"
ACTUATED_CMD_TOPIC="${ACTUATED_CMD_TOPIC:-/cmd_vel}"
LT_DWA_SHADOW_CMD_TOPIC="${LT_DWA_SHADOW_CMD_TOPIC:-/baseline/official_lt_dwa/shadow_cmd_vel}"
if [[ "${STAGE}" == "actuated" ]]; then
  CMD_TOPIC="${CMD_TOPIC:-${ACTUATED_CMD_TOPIC}}"
elif [[ "${METHOD}" == "lt_dwa_official" ]]; then
  CMD_TOPIC="${CMD_TOPIC:-${LT_DWA_SHADOW_CMD_TOPIC}}"
else
  CMD_TOPIC="${CMD_TOPIC:-${SHADOW_CMD_TOPIC}}"
fi

ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
BASE_FRAME="${BASE_FRAME:-base_link}"
PLAN_TARGET_FRAME="${PLAN_TARGET_FRAME:-map}"
CONTROLLER_FREQUENCY="${CONTROLLER_FREQUENCY:-10.0}"
PLANNER_RATE_HZ="${PLANNER_RATE_HZ:-5.0}"
COMMAND_PUBLISH_RATE_HZ="${COMMAND_PUBLISH_RATE_HZ:-30.0}"
case "${METHOD}" in
  teb) DEFAULT_MAX_V="0.30" ;;
  *) DEFAULT_MAX_V="0.50" ;;
esac
MAX_V="${MAX_V:-${DEFAULT_MAX_V}}"
MAX_W="${MAX_W:-1.2}"
MAX_ACC="${MAX_ACC:-0.6}"
MAX_ANGULAR_ACC="${MAX_ANGULAR_ACC:-1.2}"
ROBOT_RADIUS="${ROBOT_RADIUS:-0.426}"
XY_GOAL_TOL="${XY_GOAL_TOL:-0.20}"
YAW_GOAL_TOL="${YAW_GOAL_TOL:-0.30}"

RECORD_RGB="${RECORD_RGB:-true}"
RECORD_SCAN="${RECORD_SCAN:-true}"
RECORD_DEPTH="${RECORD_DEPTH:-false}"
RECORD_STANDALONE_SLOSH="${RECORD_STANDALONE_SLOSH:-true}"
RECORD_ONLINE_LIQUID="${RECORD_ONLINE_LIQUID:-false}"
START_STANDALONE_SLOSH="${START_STANDALONE_SLOSH:-true}"
SLOSH_MONITOR_ODOM_TOPIC="${SLOSH_MONITOR_ODOM_TOPIC:-${ODOM_TOPIC}}"
SLOSH_MONITOR_CMD_VEL_TOPIC="${SLOSH_MONITOR_CMD_VEL_TOPIC:-/cmd_vel}"
SLOSH_MONITOR_OUTPUT_NAMESPACE="${SLOSH_MONITOR_OUTPUT_NAMESPACE:-/slosh}"
SLOSH_MONITOR_CONTAINER_RADIUS="${SLOSH_MONITOR_CONTAINER_RADIUS:-0.0185}"
SLOSH_MONITOR_LIQUID_HEIGHT="${SLOSH_MONITOR_LIQUID_HEIGHT:-0.058}"
SLOSH_MONITOR_DAMPING_RATIO="${SLOSH_MONITOR_DAMPING_RATIO:-0.05}"
SLOSH_MONITOR_MODEL_DT="${SLOSH_MONITOR_MODEL_DT:-0.02}"
SLOSH_MONITOR_ACCEL_FILTER_ALPHA="${SLOSH_MONITOR_ACCEL_FILTER_ALPHA:-0.3}"
SLOSH_MONITOR_MIN_DT="${SLOSH_MONITOR_MIN_DT:-0.001}"
SLOSH_MONITOR_MAX_DT="${SLOSH_MONITOR_MAX_DT:-0.1}"
SLOSH_MONITOR_USE_PARABOLA_TERM="${SLOSH_MONITOR_USE_PARABOLA_TERM:-false}"
SLOSH_MONITOR_STARTUP_TIMEOUT_SEC="${SLOSH_MONITOR_STARTUP_TIMEOUT_SEC:-10}"
SLOSH_MONITOR_RESET_TIMEOUT_SEC="${SLOSH_MONITOR_RESET_TIMEOUT_SEC:-5}"
SLOSH_MONITOR_NAMESPACE_ROOT="${SLOSH_MONITOR_OUTPUT_NAMESPACE%/}"
if [[ -n "${SLOSH_MONITOR_NAMESPACE_ROOT}" && "${SLOSH_MONITOR_NAMESPACE_ROOT}" != /* ]]; then
  SLOSH_MONITOR_NAMESPACE_ROOT="/${SLOSH_MONITOR_NAMESPACE_ROOT}"
fi
SLOSH_MONITOR_NODE="${SLOSH_MONITOR_NAMESPACE_ROOT}/slosh_monitor"
SLOSH_MONITOR_RESET_SERVICE="${SLOSH_MONITOR_NAMESPACE_ROOT}/reset"
SLOSH_MONITOR_HEIGHT_TOPIC="${SLOSH_MONITOR_NAMESPACE_ROOT}/height"
# Deterministic whitelist for formal runs. Set true only for short diagnostics
# when disk space is known to be sufficient.
RECORD_ALL_EXISTING_TOPICS="${RECORD_ALL_EXISTING_TOPICS:-false}"
RECORD_TOPIC_INFO="${RECORD_TOPIC_INFO:-true}"
PATH_GENERATOR_STARTUP_SEC="${PATH_GENERATOR_STARTUP_SEC:-2}"
RECORDER_STARTUP_SEC="${RECORDER_STARTUP_SEC:-2}"
PLANNER_STARTUP_SEC="${PLANNER_STARTUP_SEC:-2}"
SEND_ZERO_ON_EXIT="${SEND_ZERO_ON_EXIT:-true}"
OPERATOR_NOTE="${OPERATOR_NOTE:-one_click_external_baseline_real_fixed_path_trial}"

require_cmd timeout
require_cmd rostopic
require_cmd rosrun
require_cmd roslaunch
require_cmd rospack
require_cmd rosnode
require_cmd rosservice
[[ -f "${RECORDER_SCRIPT}" ]] || fail "Recorder script not found: ${RECORDER_SCRIPT}"
[[ -r "${RECORDER_SCRIPT}" ]] || fail "Recorder script is not readable: ${RECORDER_SCRIPT}"

for kv in \
  "GOAL_X=${GOAL_X}" \
  "GOAL_Y=${GOAL_Y}" \
  "GOAL_YAW=${GOAL_YAW}" \
  "PATH_SPACING=${PATH_SPACING}" \
  "PATH_AMPLITUDE_RATIO=${PATH_AMPLITUDE_RATIO}" \
  "PATH_MIN_AMPLITUDE=${PATH_MIN_AMPLITUDE}" \
  "PATH_MAX_AMPLITUDE=${PATH_MAX_AMPLITUDE}" \
  "GOAL_REPEAT_RATE=${GOAL_REPEAT_RATE}" \
  "MAX_V=${MAX_V}" \
  "MAX_W=${MAX_W}" \
  "MAX_ACC=${MAX_ACC}" \
  "MAX_ANGULAR_ACC=${MAX_ANGULAR_ACC}" \
  "ROBOT_RADIUS=${ROBOT_RADIUS}" \
  "XY_GOAL_TOL=${XY_GOAL_TOL}" \
  "YAW_GOAL_TOL=${YAW_GOAL_TOL}" \
  "CONTROLLER_FREQUENCY=${CONTROLLER_FREQUENCY}" \
  "PLANNER_RATE_HZ=${PLANNER_RATE_HZ}" \
  "COMMAND_PUBLISH_RATE_HZ=${COMMAND_PUBLISH_RATE_HZ}" \
  "SLOSH_MONITOR_CONTAINER_RADIUS=${SLOSH_MONITOR_CONTAINER_RADIUS}" \
  "SLOSH_MONITOR_LIQUID_HEIGHT=${SLOSH_MONITOR_LIQUID_HEIGHT}" \
  "SLOSH_MONITOR_DAMPING_RATIO=${SLOSH_MONITOR_DAMPING_RATIO}" \
  "SLOSH_MONITOR_MODEL_DT=${SLOSH_MONITOR_MODEL_DT}" \
  "SLOSH_MONITOR_ACCEL_FILTER_ALPHA=${SLOSH_MONITOR_ACCEL_FILTER_ALPHA}" \
  "SLOSH_MONITOR_MIN_DT=${SLOSH_MONITOR_MIN_DT}" \
  "SLOSH_MONITOR_MAX_DT=${SLOSH_MONITOR_MAX_DT}" \
  "SLOSH_MONITOR_STARTUP_TIMEOUT_SEC=${SLOSH_MONITOR_STARTUP_TIMEOUT_SEC}" \
  "SLOSH_MONITOR_RESET_TIMEOUT_SEC=${SLOSH_MONITOR_RESET_TIMEOUT_SEC}" \
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

BASELINE_RUNNER_DIR="${BASELINE_RUNNER_DIR:-}"
SPMPC_EXP_DIR="${SPMPC_EXP_DIR:-}"
STATUS_TOPIC="${STATUS_TOPIC:-}"
GLOBAL_PLAN_TOPIC="${GLOBAL_PLAN_TOPIC:-}"

case "${METHOD}" in
  lt_dwa_official)
    SCOUT_WS_ROOT="${SCOUT_WS_ROOT:-/home/geist/scout_ws}"
    export SCOUT_WS_ROOT
    export ROS_PACKAGE_PATH="${SCOUT_WS_ROOT}/tools/lt_dwa/local_planner_runtime:${ROS_PACKAGE_PATH:-}"
    if [[ "${STAGE}" == "actuated" ]]; then
      LT_DWA_LAUNCH="scout_sop_cmd_vel_benchmark.launch"
      LT_DWA_ENABLE_ACTUATED_OUTPUT="true"
      LT_DWA_PUBLISH_CMD_VEL="true"
    else
      LT_DWA_LAUNCH="scout_sop_shadow_integration.launch"
      LT_DWA_ENABLE_ACTUATED_OUTPUT="false"
      LT_DWA_PUBLISH_CMD_VEL="false"
    fi
    STATUS_TOPIC="${STATUS_TOPIC:-/baseline/official_lt_dwa/status}"
    GLOBAL_PLAN_TOPIC="${GLOBAL_PLAN_TOPIC:-/baseline/official_lt_dwa/global_plan}"
    LT_DWA_SHADOW_OUTPUT_TOPIC="${LT_DWA_SHADOW_CMD_TOPIC}"
    if [[ "${STAGE}" == "shadow" ]]; then
      LT_DWA_SHADOW_OUTPUT_TOPIC="${CMD_TOPIC}"
    fi
    planner_cmd=(
      roslaunch lt_dwa_official_wrapper "${LT_DWA_LAUNCH}"
      "start_local_map_service:=${START_LOCAL_MAP_SERVICE:-true}"
      "enable_actuated_output:=${LT_DWA_ENABLE_ACTUATED_OUTPUT}"
      "publish_cmd_vel:=${LT_DWA_PUBLISH_CMD_VEL}"
      "planner_execution_mode:=${PLANNER_EXECUTION_MODE:-in_process}"
      "planner_rate_hz:=${PLANNER_RATE_HZ}"
      "command_publish_rate_hz:=${COMMAND_PUBLISH_RATE_HZ}"
      "max_v:=${MAX_V}"
      "max_w:=${MAX_W}"
      "max_acc:=${MAX_ACC}"
      "max_angular_acc:=${MAX_ANGULAR_ACC}"
      "robot_radius:=${ROBOT_RADIUS}"
      "goal_xy_tolerance:=${XY_GOAL_TOL}"
      "goal_yaw_tolerance:=${YAW_GOAL_TOL}"
      "input_odom_topic:=${ODOM_TOPIC}"
      "map_topic:=${MAP_TOPIC}"
      "path_topic:=${REF_TOPIC}"
      "goal_topic:=${GOAL_TOPIC}"
      "raw_cmd_topic:=${LT_DWA_RAW_CMD_TOPIC:-/baseline/official_lt_dwa/raw_cmd_vel}"
      "shadow_cmd_topic:=${LT_DWA_SHADOW_OUTPUT_TOPIC}"
      "status_topic:=${STATUS_TOPIC}"
      "diagnostics_topic:=${LT_DWA_DIAGNOSTICS_TOPIC:-/baseline/official_lt_dwa/diagnostics}"
      "global_plan_topic:=${GLOBAL_PLAN_TOPIC}"
      "local_plan_topic:=${LT_DWA_LOCAL_PLAN_TOPIC:-/baseline/official_lt_dwa/local_plan}"
      "worker_result_topic:=${LT_DWA_WORKER_RESULT_TOPIC:-/baseline/official_lt_dwa/worker_result}"
      "cmd_vel_topic:=${CMD_TOPIC}"
    )
    ;;
  teb|mpc_local_planner)
    if [[ -z "${BASELINE_RUNNER_DIR}" ]]; then
      BASELINE_RUNNER_DIR="$(rospack find baseline_local_planner_runner 2>/dev/null || true)"
    fi
    if [[ -z "${SPMPC_EXP_DIR}" ]]; then
      SPMPC_EXP_DIR="$(rospack find spmpc_experiments 2>/dev/null || true)"
    fi
    [[ -n "${BASELINE_RUNNER_DIR}" ]] || fail "rospack cannot find baseline_local_planner_runner"
    [[ -n "${SPMPC_EXP_DIR}" ]] || fail "rospack cannot find spmpc_experiments"
    if [[ "${METHOD}" == "teb" ]]; then
      PLUGIN_TYPE="teb_local_planner/TebLocalPlannerROS"
      PLUGIN_NAME="TebLocalPlannerROS"
      STATUS_TOPIC="${STATUS_TOPIC:-/baseline/teb/status}"
      GLOBAL_PLAN_TOPIC="${GLOBAL_PLAN_TOPIC:-/baseline/teb/global_plan}"
      RAW_CMD_TOPIC="${RAW_CMD_TOPIC:-/baseline/teb/raw_cmd_vel}"
      COMMAND_INTERVENTION_TOPIC="${COMMAND_INTERVENTION_TOPIC:-/baseline/teb/command_intervention}"
      TRACKING_DIAGNOSTICS_TOPIC="${TRACKING_DIAGNOSTICS_TOPIC:-/baseline/teb/tracking_error}"
      COSTMAP_CONFIG="${COSTMAP_CONFIG:-${BASELINE_RUNNER_DIR}/config/local_costmap_real_no_obstacles.yaml}"
      PLANNER_CONFIG="${PLANNER_CONFIG:-${SPMPC_EXP_DIR}/config/baselines/teb_local_planner_fixed_path_real_noobs.yaml}"
      OVERRIDE_TEB_LIMITS="${OVERRIDE_TEB_LIMITS:-true}"
    else
      MPC_OVERLAY_SETUP="${MPC_OVERLAY_SETUP:-/home/geist/scout_ws/install_isolated_mpc/setup.bash}"
      [[ -r "${MPC_OVERLAY_SETUP}" ]] || fail "mpc_local_planner overlay setup is missing: ${MPC_OVERLAY_SETUP}; source/build isolated MPC overlay before this baseline"
      if [[ ":${CMAKE_PREFIX_PATH:-}:" != *"install_isolated_mpc"* && ":${ROS_PACKAGE_PATH:-}:" != *"install_isolated_mpc"* ]]; then
        echo "[WARN] install_isolated_mpc is not visible in CMAKE_PREFIX_PATH/ROS_PACKAGE_PATH; source ${MPC_OVERLAY_SETUP} before running if plugin loading fails." >&2
      fi
      PLUGIN_TYPE="mpc_local_planner/MpcLocalPlannerROS"
      PLUGIN_NAME="MpcLocalPlannerROS"
      STATUS_TOPIC="${STATUS_TOPIC:-/baseline/mpc_local_planner/status}"
      GLOBAL_PLAN_TOPIC="${GLOBAL_PLAN_TOPIC:-/baseline/mpc_local_planner/global_plan}"
      RAW_CMD_TOPIC="${RAW_CMD_TOPIC:-/baseline/mpc_local_planner/raw_cmd_vel}"
      COMMAND_INTERVENTION_TOPIC="${COMMAND_INTERVENTION_TOPIC:-/baseline/mpc_local_planner/command_intervention}"
      TRACKING_DIAGNOSTICS_TOPIC="${TRACKING_DIAGNOSTICS_TOPIC:-/baseline/mpc_local_planner/tracking_error}"
      COSTMAP_CONFIG="${COSTMAP_CONFIG:-${BASELINE_RUNNER_DIR}/config/local_costmap_real.yaml}"
      PLANNER_CONFIG="${PLANNER_CONFIG:-${SPMPC_EXP_DIR}/config/baselines/mpc_local_planner_fixed_path_tuned_sim.yaml}"
      OVERRIDE_TEB_LIMITS="false"
    fi
    [[ -r "${COSTMAP_CONFIG}" ]] || fail "Costmap config is not readable: ${COSTMAP_CONFIG}"
    [[ -r "${PLANNER_CONFIG}" ]] || fail "Planner config is not readable: ${PLANNER_CONFIG}"
    planner_cmd=(
      roslaunch baseline_local_planner_runner nav_core_runner.launch
      "plugin_type:=${PLUGIN_TYPE}"
      "plugin_name:=${PLUGIN_NAME}"
      "global_path_topic:=${REF_TOPIC}"
      "goal_topic:=${GOAL_TOPIC}"
      "cmd_vel_topic:=${CMD_TOPIC}"
      "status_topic:=${STATUS_TOPIC}"
      "global_plan_topic:=${GLOBAL_PLAN_TOPIC}"
      "controller_frequency:=${CONTROLLER_FREQUENCY}"
      "base_frame:=${BASE_FRAME}"
      "plan_target_frame:=${PLAN_TARGET_FRAME}"
      "max_cmd_vel_x:=${MAX_V}"
      "max_cmd_vel_theta:=${MAX_W}"
      "raw_cmd_vel_topic:=${RAW_CMD_TOPIC}"
      "command_intervention_topic:=${COMMAND_INTERVENTION_TOPIC}"
      "tracking_diagnostics_topic:=${TRACKING_DIAGNOSTICS_TOPIC}"
      "override_teb_limits:=${OVERRIDE_TEB_LIMITS}"
      "teb_max_vel_x:=${MAX_V}"
      "teb_max_vel_trans:=${MAX_V}"
      "teb_max_vel_theta:=${MAX_W}"
      "teb_acc_lim_x:=${MAX_ACC}"
      "teb_acc_lim_theta:=${MAX_ANGULAR_ACC}"
      "xy_goal_tolerance:=${XY_GOAL_TOL}"
      "yaw_goal_tolerance:=${YAW_GOAL_TOL}"
      "costmap_config:=${COSTMAP_CONFIG}"
      "planner_config:=${PLANNER_CONFIG}"
    )
    ;;
esac
planner_command_string="$(printf '%q ' "${planner_cmd[@]}")"

check_cmd_vel_clear_for_actuated

mkdir -p "${RUN_OUT_DIR}" "$(dirname "${PATH_FILE}")"

path_generator_log="${RUN_OUT_DIR}/${NAME}_path_generator.log"
send_goal_log="${RUN_OUT_DIR}/${NAME}_send_goal.log"
recorder_log="${RUN_OUT_DIR}/${NAME}_recorder.log"
planner_log="${RUN_OUT_DIR}/${NAME}_planner.log"
slosh_monitor_log="${RUN_OUT_DIR}/${NAME}_slosh_monitor.log"
run_meta="${RUN_OUT_DIR}/${NAME}_external_baseline_meta.env"

path_generator_pid=""
recorder_pid=""
planner_pid=""
slosh_monitor_pid=""
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
  if [[ "${STAGE}" != "actuated" ]] || [[ "${CMD_TOPIC}" != "/cmd_vel" ]] || ! truthy "${SEND_ZERO_ON_EXIT}"; then
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
  kill_child "${slosh_monitor_pid}" "standalone slosh monitor"
}

on_interrupt() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap on_interrupt INT TERM

{
  echo "date=${DATE}"
  echo "stamp=${STAMP}"
  echo "method=${METHOD}"
  echo "stage=${STAGE}"
  echo "alg=${ALG}"
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
  echo "status_topic=${STATUS_TOPIC}"
  echo "global_plan_topic=${GLOBAL_PLAN_TOPIC}"
  echo "raw_cmd_topic=${RAW_CMD_TOPIC:-}"
  echo "command_intervention_topic=${COMMAND_INTERVENTION_TOPIC:-}"
  echo "tracking_diagnostics_topic=${TRACKING_DIAGNOSTICS_TOPIC:-}"
  echo "planner_config=${PLANNER_CONFIG:-}"
  echo "costmap_config=${COSTMAP_CONFIG:-}"
  echo "max_v=${MAX_V}"
  echo "max_w=${MAX_W}"
  echo "max_acc=${MAX_ACC}"
  echo "max_angular_acc=${MAX_ANGULAR_ACC}"
  echo "xy_goal_tol=${XY_GOAL_TOL}"
  echo "yaw_goal_tol=${YAW_GOAL_TOL}"
  echo "record_rgb=${RECORD_RGB}"
  echo "record_standalone_slosh=${RECORD_STANDALONE_SLOSH}"
  echo "start_standalone_slosh=${START_STANDALONE_SLOSH}"
  echo "slosh_monitor_odom_topic=${SLOSH_MONITOR_ODOM_TOPIC}"
  echo "slosh_monitor_cmd_vel_topic=${SLOSH_MONITOR_CMD_VEL_TOPIC}"
  echo "slosh_monitor_output_namespace=${SLOSH_MONITOR_OUTPUT_NAMESPACE}"
  echo "slosh_monitor_node=${SLOSH_MONITOR_NODE}"
  echo "slosh_monitor_height_topic=${SLOSH_MONITOR_HEIGHT_TOPIC}"
  echo "slosh_monitor_reset_service=${SLOSH_MONITOR_RESET_SERVICE}"
  echo "slosh_monitor_log=${slosh_monitor_log}"
  echo "slosh_monitor_container_radius=${SLOSH_MONITOR_CONTAINER_RADIUS}"
  echo "slosh_monitor_liquid_height=${SLOSH_MONITOR_LIQUID_HEIGHT}"
  echo "slosh_monitor_damping_ratio=${SLOSH_MONITOR_DAMPING_RATIO}"
  echo "slosh_monitor_model_dt=${SLOSH_MONITOR_MODEL_DT}"
  echo "slosh_monitor_accel_filter_alpha=${SLOSH_MONITOR_ACCEL_FILTER_ALPHA}"
  echo "slosh_monitor_min_dt=${SLOSH_MONITOR_MIN_DT}"
  echo "slosh_monitor_max_dt=${SLOSH_MONITOR_MAX_DT}"
  echo "slosh_monitor_use_parabola_term=${SLOSH_MONITOR_USE_PARABOLA_TERM}"
  echo "slosh_monitor_startup_timeout_sec=${SLOSH_MONITOR_STARTUP_TIMEOUT_SEC}"
  echo "slosh_monitor_reset_timeout_sec=${SLOSH_MONITOR_RESET_TIMEOUT_SEC}"
  echo "slosh_eval_only=true"
  echo "external_baseline_uses_slosh=false"
  echo "record_online_liquid=${RECORD_ONLINE_LIQUID}"
  echo "record_all_existing_topics=${RECORD_ALL_EXISTING_TOPICS}"
  echo "planner_command=${planner_command_string}"
} > "${run_meta}"

cat <<EOF
================ external baseline real fixed-path trial ================
  method       = ${METHOD}
  stage        = ${STAGE}
  run_label    = ${RUN_LABEL}
  cmd_topic    = ${CMD_TOPIC}
  status_topic = ${STATUS_TOPIC}
  planner_cfg  = ${PLANNER_CONFIG:-NA}
  costmap_cfg  = ${COSTMAP_CONFIG:-NA}
  slosh monitor= start:${START_STANDALONE_SLOSH} record:${RECORD_STANDALONE_SLOSH} topic:${SLOSH_MONITOR_HEIGHT_TOPIC}
  recorder     = ${RECORD_SEC}s max (Ctrl+C stops earlier)
  out_dir      = ${RUN_OUT_DIR}
  path_file    = ${PATH_FILE}
  goal         = (${GOAL_X}, ${GOAL_Y}, ${GOAL_YAW}) in ${GOAL_FRAME}
=========================================================================
EOF

if [[ "${STAGE}" == "actuated" ]]; then
  cat <<EOF
[SAFETY] Actuated run requested.
[SAFETY] Confirm before continuing that shadow/smoke passed, E-stop/remote operator is ready,
[SAFETY] robot is at the start mark, liquid settled 60-90s, and the area is clear.
EOF
fi

prepare_slosh_monitor

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
  VARIANT="${METHOD}" \
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
  GOAL_X="${GOAL_X}" \
  GOAL_Y="${GOAL_Y}" \
  GOAL_YAW="${GOAL_YAW}" \
  LAUNCH_COMMAND="${planner_command_string}" \
  OPERATOR_NOTE="${OPERATOR_NOTE}; method=${METHOD}; stage=${STAGE}" \
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

echo "[launch] starting external baseline"
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

echo "[run] recording until Ctrl+C, ${RECORD_SEC}s recorder timeout, planner exit, or slosh monitor exit"
wait_pids=("${recorder_pid}" "${planner_pid}")
if [[ -n "${slosh_monitor_pid}" ]]; then
  wait_pids+=("${slosh_monitor_pid}")
fi
set +e
wait -n "${wait_pids[@]}"
first_exit_code=$?
set -e
if [[ -n "${slosh_monitor_pid}" ]] && ! child_running "${slosh_monitor_pid}" && child_running "${recorder_pid}"; then
  show_log_tail "${slosh_monitor_log}" "standalone slosh monitor"
  echo "[ERR] standalone slosh monitor exited before recorder timeout (code=${first_exit_code}); invalidating run" >&2
  recorder_code=1
elif child_running "${recorder_pid}"; then
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
echo "  slosh log    = ${slosh_monitor_log}"
echo "  path log     = ${path_generator_log}"
echo "================================================"
exit "${recorder_code}"
