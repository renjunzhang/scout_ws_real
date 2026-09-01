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
TEST_AXIS="${TEST_AXIS:-angular}"
case "${TEST_AXIS}" in
  angular)
    DEFAULT_STEP_DIRECTION=left
    DEFAULT_STEP_MAGNITUDE=0.20
    STEP_UNIT=rad/s
    MAX_STEP_MAGNITUDE=0.30
    ;;
  linear)
    DEFAULT_STEP_DIRECTION=forward
    DEFAULT_STEP_MAGNITUDE=0.10
    STEP_UNIT=m/s
    MAX_STEP_MAGNITUDE=0.15
    ;;
  *) fail "TEST_AXIS must be linear or angular" ;;
esac
STEP_DIRECTION="${STEP_DIRECTION:-${DEFAULT_STEP_DIRECTION}}"
STEP_MAGNITUDE="${STEP_MAGNITUDE:-${DEFAULT_STEP_MAGNITUDE}}"
PRE_SEC="${PRE_SEC:-3.0}"
STEP_SEC="${STEP_SEC:-4.0}"
POST_SEC="${POST_SEC:-3.0}"
PUBLISH_RATE_HZ="${PUBLISH_RATE_HZ:-50.0}"
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"

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
PLOT_PATH="${PLOT_PATH:-${RUN_OUT_DIR}/${RUN_LABEL}_response.png}"

[[ -s "${PUBLISHER}" ]] || fail "missing publisher: ${PUBLISHER}"
[[ -s "${ANALYZER}" ]] || fail "missing analyzer: ${ANALYZER}"
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"
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

case "${TEST_AXIS}:${STEP_DIRECTION}" in
  angular:left|linear:forward) SIGNED_COMMAND="${STEP_MAGNITUDE}" ;;
  angular:right|linear:reverse) SIGNED_COMMAND="-${STEP_MAGNITUDE}" ;;
  angular:*) fail "angular STEP_DIRECTION must be left or right" ;;
  linear:*) fail "linear STEP_DIRECTION must be forward or reverse" ;;
esac

RAW_MOCAP_TOPIC="/vrpn_client_node/${MOCAP_TRACKER}/pose"

echo "================ stationary velocity step ================"
echo "  axis/direction  = ${TEST_AXIS} / ${STEP_DIRECTION}"
echo "  command         = ${SIGNED_COMMAND} ${STEP_UNIT}"
echo "  phases          = zero ${PRE_SEC}s -> step ${STEP_SEC}s -> zero ${POST_SEC}s"
echo "  command rate    = ${PUBLISH_RATE_HZ} Hz"
echo "  mocap / IMU     = ${RAW_MOCAP_TOPIC} / ${IMU_TOPIC}"
echo "  odometry        = ${ODOM_TOPIC}"
echo "  output bag      = ${BAG_PATH}"
echo "==========================================================="

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS"
  echo "To move the robot, set VALIDATE_ONLY=false ARM_MOTION=YES after checking clearance and E-stop."
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "set ARM_MOTION=YES only after confirming level ground, clear motion space, and E-stop"
for path in "${BAG_PATH}" "${BAG_PATH}.active" "${METADATA_PATH}" "${REPORT_PATH}" "${PLOT_PATH}"; do
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
  --command-value "${SIGNED_COMMAND}" \
  --pre-sec "${PRE_SEC}" \
  --step-sec "${STEP_SEC}" \
  --post-sec "${POST_SEC}" \
  --rate-hz "${PUBLISH_RATE_HZ}" \
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
  --output-json "${REPORT_PATH}" \
  --plot "${PLOT_PATH}"

trap - EXIT INT TERM
echo "[${SCRIPT_NAME}] PASS"
echo "  bag    : ${BAG_PATH}"
echo "  report : ${REPORT_PATH}"
echo "  plot   : ${PLOT_PATH}"
