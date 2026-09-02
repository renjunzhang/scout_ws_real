#!/usr/bin/env bash
# Record one stationary velocity step and immediately measure response performance.

set -euo pipefail

SCRIPT_NAME="run_spmpc_mocap_velocity_step_trial"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
PUBLISHER="${SCRIPT_DIR}/analysis/publish_mocap_velocity_step.py"
ANALYZER="${SCRIPT_DIR}/analysis/analyze_mocap_velocity_step.py"

fail() {
  echo "[${SCRIPT_NAME}][ERR] $*" >&2
  exit 2
}

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

is_number() {
  [[ "${1:-}" =~ ^[0-9]+([.][0-9]+)?$ ]]
}

DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
TRIAL_CONTRACT="${TRIAL_CONTRACT:-low_speed_identification}"
case "${TRIAL_CONTRACT}" in
  low_speed_identification)
    DEFAULT_TEST_AXIS=angular
    DEFAULT_PRE_SEC=3.0
    DEFAULT_STEP_SEC=4.0
    DEFAULT_POST_SEC=3.0
    ;;
  hardware_accel_limit)
    DEFAULT_TEST_AXIS=linear
    DEFAULT_PRE_SEC=3.0
    DEFAULT_STEP_SEC=2.5
    DEFAULT_POST_SEC=4.0
    ;;
  *) fail "TRIAL_CONTRACT must be low_speed_identification or hardware_accel_limit" ;;
esac

TEST_AXIS="${TEST_AXIS:-${DEFAULT_TEST_AXIS}}"
case "${TEST_AXIS}" in
  angular)
    DEFAULT_STEP_DIRECTION=left
    DEFAULT_STEP_MAGNITUDE=0.20
    STEP_UNIT=rad/s
    MAX_STEP_MAGNITUDE=0.30
    ;;
  linear)
    DEFAULT_STEP_DIRECTION=forward
    if [[ "${TRIAL_CONTRACT}" == hardware_accel_limit ]]; then
      DEFAULT_STEP_MAGNITUDE=0.80
      MAX_STEP_MAGNITUDE=0.80
    else
      DEFAULT_STEP_MAGNITUDE=0.10
      MAX_STEP_MAGNITUDE=0.15
    fi
    STEP_UNIT=m/s
    ;;
  *) fail "TEST_AXIS must be linear or angular" ;;
esac
STEP_DIRECTION="${STEP_DIRECTION:-${DEFAULT_STEP_DIRECTION}}"
STEP_MAGNITUDE="${STEP_MAGNITUDE:-${DEFAULT_STEP_MAGNITUDE}}"
PRE_SEC="${PRE_SEC:-${DEFAULT_PRE_SEC}}"
STEP_SEC="${STEP_SEC:-${DEFAULT_STEP_SEC}}"
POST_SEC="${POST_SEC:-${DEFAULT_POST_SEC}}"
PUBLISH_RATE_HZ="${PUBLISH_RATE_HZ:-50.0}"
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_HARDWARE_ACCEL_LIMIT="${CONFIRM_HARDWARE_ACCEL_LIMIT:-NO}"
DATA_SPLIT="${DATA_SPLIT:-development}"
MATRIX_ROW="${MATRIX_ROW:-single}"
ATTEMPT="${ATTEMPT:-01}"

MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
STAMPED_CMD_TOPIC="${STAMPED_CMD_TOPIC:-/mocap_velocity_step/cmd_vel_stamped}"

RUN_LABEL="${RUN_LABEL:-MOCAP_${TEST_AXIS^^}_STEP_${STEP_DIRECTION}_${STAMP}}"
RUN_OUT_DIR="${RUN_OUT_DIR:-${HOME}/slosh_bags/real/${DATE}_mocap_velocity_step}"
BAG_PATH="${BAG_PATH:-${RUN_OUT_DIR}/${RUN_LABEL}.bag}"
METADATA_PATH="${METADATA_PATH:-${RUN_OUT_DIR}/${RUN_LABEL}_command.json}"
REPORT_PATH="${REPORT_PATH:-${RUN_OUT_DIR}/${RUN_LABEL}_response.json}"
PLOT_DIR="${PLOT_DIR:-${RUN_OUT_DIR}/${RUN_LABEL}_plots}"

[[ -s "${PUBLISHER}" ]] || fail "missing publisher: ${PUBLISHER}"
[[ -s "${ANALYZER}" ]] || fail "missing analyzer: ${ANALYZER}"
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"
case "${DATA_SPLIT}" in
  development|validation|final_test) ;;
  *) fail "DATA_SPLIT must be development, validation, or final_test" ;;
esac
[[ "${MATRIX_ROW}" == "single" || "${MATRIX_ROW}" =~ ^(0[1-9]|1[0-2])$ ]] || \
  fail "MATRIX_ROW must be single or 01..12"
[[ "${ATTEMPT}" =~ ^[0-9][0-9]$ ]] || fail "ATTEMPT must be two digits"
for value in "${STEP_MAGNITUDE}" "${PRE_SEC}" "${STEP_SEC}" "${POST_SEC}" "${PUBLISH_RATE_HZ}"; do
  is_number "${value}" || fail "all numeric settings must be non-negative decimal numbers"
done
awk -v value="${STEP_MAGNITUDE}" -v maximum="${MAX_STEP_MAGNITUDE}" \
  'BEGIN {exit !(value > 0.0 && value <= maximum)}' || \
  fail "require 0 < STEP_MAGNITUDE <= ${MAX_STEP_MAGNITUDE} ${STEP_UNIT} for ${TEST_AXIS}"
awk -v value="${PRE_SEC}" 'BEGIN {exit !(value >= 2.0 && value <= 10.0)}' || \
  fail "require 2 <= PRE_SEC <= 10"
awk -v value="${STEP_SEC}" 'BEGIN {exit !(value >= 2.5 && value <= 6.0)}' || \
  fail "require 2.5 <= STEP_SEC <= 6"
awk -v value="${POST_SEC}" 'BEGIN {exit !(value >= 2.0 && value <= 10.0)}' || \
  fail "require 2 <= POST_SEC <= 10"
awk -v value="${PUBLISH_RATE_HZ}" 'BEGIN {exit !(value >= 20.0 && value <= 100.0)}' || \
  fail "require 20 <= PUBLISH_RATE_HZ <= 100"

HARDWARE_ACCEL_LIMIT_ARM_TOKEN=""
REPORT_PROTOCOL_ID=MOCAP_VELOCITY_STEP_V2
if [[ "${TRIAL_CONTRACT}" == hardware_accel_limit ]]; then
  [[ "${TEST_AXIS}" == linear ]] || fail "hardware_accel_limit requires TEST_AXIS=linear"
  [[ "${STEP_DIRECTION}" == forward ]] || \
    fail "hardware_accel_limit requires STEP_DIRECTION=forward"
  awk -v value="${STEP_MAGNITUDE}" 'BEGIN {exit !(value == 0.80)}' || \
    fail "hardware_accel_limit requires STEP_MAGNITUDE=0.80"
  awk -v value="${PRE_SEC}" 'BEGIN {exit !(value == 3.0)}' || \
    fail "hardware_accel_limit requires PRE_SEC=3.0"
  awk -v value="${STEP_SEC}" 'BEGIN {exit !(value == 2.5)}' || \
    fail "hardware_accel_limit requires STEP_SEC=2.5"
  awk -v value="${POST_SEC}" 'BEGIN {exit !(value == 4.0)}' || \
    fail "hardware_accel_limit requires POST_SEC=4.0"
  awk -v value="${PUBLISH_RATE_HZ}" 'BEGIN {exit !(value == 50.0)}' || \
    fail "hardware_accel_limit requires PUBLISH_RATE_HZ=50.0"
  HARDWARE_ACCEL_LIMIT_ARM_TOKEN=MOCAP_HARDWARE_ACCEL_LIMIT_ARMED
  REPORT_PROTOCOL_ID=MOCAP_HARDWARE_ACCEL_LIMIT_V1
fi

case "${TEST_AXIS}:${STEP_DIRECTION}" in
  angular:left|linear:forward) SIGNED_COMMAND="${STEP_MAGNITUDE}" ;;
  angular:right|linear:reverse) SIGNED_COMMAND="-${STEP_MAGNITUDE}" ;;
  angular:*) fail "angular STEP_DIRECTION must be left or right" ;;
  linear:*) fail "linear STEP_DIRECTION must be forward or reverse" ;;
esac

RAW_MOCAP_TOPIC="/vrpn_client_node/${MOCAP_TRACKER}/pose"

echo "================ stationary velocity step ================"
echo "  trial contract  = ${TRIAL_CONTRACT}"
echo "  axis/direction  = ${TEST_AXIS} / ${STEP_DIRECTION}"
echo "  command         = ${SIGNED_COMMAND} ${STEP_UNIT}"
echo "  phases          = zero ${PRE_SEC}s -> step ${STEP_SEC}s -> zero ${POST_SEC}s"
echo "  command rate    = ${PUBLISH_RATE_HZ} Hz"
echo "  split / row     = ${DATA_SPLIT} / ${MATRIX_ROW}"
echo "  mocap / IMU     = ${RAW_MOCAP_TOPIC} / ${IMU_TOPIC}"
echo "  odometry        = ${ODOM_TOPIC}"
echo "  output bag      = ${BAG_PATH}"
echo "  separated plots = ${PLOT_DIR}"
echo "==========================================================="

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS"
  echo "To move the robot, set VALIDATE_ONLY=false ARM_MOTION=YES after checking clearance and E-stop."
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "set ARM_MOTION=YES only after confirming level ground, clear motion space, and E-stop"
if [[ "${TRIAL_CONTRACT}" == hardware_accel_limit ]]; then
  [[ "${CONFIRM_HARDWARE_ACCEL_LIMIT}" == YES ]] || \
    fail "hardware_accel_limit requires CONFIRM_HARDWARE_ACCEL_LIMIT=YES"
fi
for path in "${BAG_PATH}" "${BAG_PATH}.active" "${METADATA_PATH}" "${REPORT_PATH}" "${PLOT_DIR}"; do
  [[ ! -e "${path}" ]] || fail "refusing to overwrite existing output: ${path}"
done

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

use_sim_time="$(rosparam get /use_sim_time 2>/dev/null || echo false)"
[[ "${use_sim_time}" != "true" ]] || fail "real test refuses /use_sim_time=true"
timeout 5s rostopic echo -n 1 "${RAW_MOCAP_TOPIC}" >/dev/null 2>&1 || \
  fail "no NOKOV pose on ${RAW_MOCAP_TOPIC}"
timeout 5s rostopic echo -n 1 "${IMU_TOPIC}" >/dev/null 2>&1 || \
  fail "no IMU on ${IMU_TOPIC}"
timeout 5s rostopic echo -n 1 "${ODOM_TOPIC}" >/dev/null 2>&1 || \
  fail "no odometry on ${ODOM_TOPIC}"

published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
if grep -Fxq -- "${CMD_TOPIC}" <<< "${published_topics}"; then
  fail "${CMD_TOPIC} already has a publisher; stop planner and teleop first"
fi
cmd_topic_info="$(timeout 5s rostopic info "${CMD_TOPIC}" 2>/dev/null || true)"
if ! sed -n '/^Subscribers:/,$p' <<< "${cmd_topic_info}" | grep -qE '^[[:space:]]*\* '; then
  fail "${CMD_TOPIC} has no base subscriber; start the chassis driver first"
fi

mkdir -p "${RUN_OUT_DIR}"
recorder_pid=""
publisher_pid=""
motion_started=false

publish_zero() {
  timeout 1s rostopic pub -r 20 "${CMD_TOPIC}" geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
}

stop_recorder() {
  if [[ -n "${recorder_pid}" ]]; then
    kill -INT "${recorder_pid}" 2>/dev/null || true
    wait "${recorder_pid}" 2>/dev/null || true
    recorder_pid=""
  fi
}

cleanup() {
  local status=$?
  trap - EXIT
  if [[ -n "${publisher_pid}" ]]; then
    kill -INT "${publisher_pid}" 2>/dev/null || true
    wait "${publisher_pid}" 2>/dev/null || true
    publisher_pid=""
  fi
  if [[ "${motion_started}" == true ]]; then
    publish_zero
  fi
  stop_recorder
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

rosbag record --buffsize=256 --chunksize=768 -O "${BAG_PATH}" \
  "${CMD_TOPIC}" \
  "${STAMPED_CMD_TOPIC}" \
  "${RAW_MOCAP_TOPIC}" \
  "${IMU_TOPIC}" \
  "${ODOM_TOPIC}" \
  /scout_status &
recorder_pid=$!

for _ in $(seq 1 50); do
  [[ -e "${BAG_PATH}.active" ]] && break
  kill -0 "${recorder_pid}" 2>/dev/null || fail "rosbag recorder exited early"
  sleep 0.1
done
[[ -e "${BAG_PATH}.active" ]] || fail "rosbag recorder did not create ${BAG_PATH}.active"
sleep 0.5

motion_started=true
python3 "${PUBLISHER}" \
  --cmd-topic "${CMD_TOPIC}" \
  --stamped-topic "${STAMPED_CMD_TOPIC}" \
  --odom-topic "${ODOM_TOPIC}" \
  --imu-topic "${IMU_TOPIC}" \
  --axis "${TEST_AXIS}" \
  --trial-contract "${TRIAL_CONTRACT}" \
  --hardware-accel-limit-arm-token "${HARDWARE_ACCEL_LIMIT_ARM_TOKEN}" \
  --command-value "${SIGNED_COMMAND}" \
  --pre-sec "${PRE_SEC}" \
  --step-sec "${STEP_SEC}" \
  --post-sec "${POST_SEC}" \
  --rate-hz "${PUBLISH_RATE_HZ}" \
  --run-label "${RUN_LABEL}" \
  --data-split "${DATA_SPLIT}" \
  --matrix-row "${MATRIX_ROW}" \
  --attempt "${ATTEMPT}" \
  --arm-token MOCAP_VELOCITY_STEP_ARMED \
  --metadata "${METADATA_PATH}" &
publisher_pid=$!
publisher_status=0
wait "${publisher_pid}" || publisher_status=$?
publisher_pid=""
(( publisher_status == 0 )) || fail "step publisher exited with status ${publisher_status}"
publish_zero
motion_started=false
sleep 0.3
stop_recorder

[[ -s "${BAG_PATH}" ]] || fail "bag is missing or empty after recording: ${BAG_PATH}"
python3 "${ANALYZER}" "${BAG_PATH}" \
  --axis "${TEST_AXIS}" \
  --mocap-tracker "${MOCAP_TRACKER}" \
  --cmd-topic "${CMD_TOPIC}" \
  --imu-topic "${IMU_TOPIC}" \
  --odom-topic "${ODOM_TOPIC}" \
  --stamped-command-topic "${STAMPED_CMD_TOPIC}" \
  --protocol-id "${REPORT_PROTOCOL_ID}" \
  --run-label "${RUN_LABEL}" \
  --data-split "${DATA_SPLIT}" \
  --matrix-row "${MATRIX_ROW}" \
  --attempt "${ATTEMPT}" \
  --output-json "${REPORT_PATH}" \
  --plot-dir "${PLOT_DIR}"

trap - EXIT INT TERM
echo "[${SCRIPT_NAME}] PASS"
echo "  bag    : ${BAG_PATH}"
echo "  report : ${REPORT_PATH}"
echo "  plots  : ${PLOT_DIR}"
