#!/usr/bin/env bash
# Record one continuous velocity profile and immediately analyze speed continuity.

set -euo pipefail

SCRIPT_NAME="run_spmpc_mocap_velocity_continuity_trial"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
PUBLISHER="${SCRIPT_DIR}/analysis/publish_mocap_velocity_continuity.py"
ANALYZER="${SCRIPT_DIR}/analysis/analyze_mocap_velocity_continuity.py"

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
TEST_AXIS="${TEST_AXIS:-linear}"
PROFILE="${PROFILE:-trapezoidal_velocity}"
case "${TEST_AXIS}" in
  linear)
    DEFAULT_DIRECTION=forward
    DEFAULT_TARGET=0.80
    SPEED_UNIT=m/s
    ACCEL_UNIT=m/s2
    RATED_MAX_SPEED=3.0
    MAX_TARGET=0.80
    MAX_PROFILE_ACCEL=0.60
    MAX_PROFILE_MOTION=5.0
    AXIS_TAG=LIN
    ;;
  angular)
    DEFAULT_DIRECTION=left
    DEFAULT_TARGET=0.20
    SPEED_UNIT=rad/s
    ACCEL_UNIT=rad/s2
    RATED_MAX_SPEED=not_applicable
    MAX_TARGET=0.30
    MAX_PROFILE_ACCEL=0.40
    MAX_PROFILE_MOTION=3.0
    AXIS_TAG=ANG
    ;;
  *) fail "TEST_AXIS must be linear or angular" ;;
esac

case "${PROFILE}" in
  constant_accel) PROFILE_TAG=CA ;;
  trapezoidal_velocity) PROFILE_TAG=TRAP ;;
  linear_accel) PROFILE_TAG=LA ;;
  *) fail "PROFILE must be constant_accel, trapezoidal_velocity, or linear_accel" ;;
esac

DIRECTION="${DIRECTION:-${DEFAULT_DIRECTION}}"
TARGET_MAGNITUDE="${TARGET_MAGNITUDE:-${DEFAULT_TARGET}}"
PRE_SEC="${PRE_SEC:-3.0}"
RAMP_UP_SEC="${RAMP_UP_SEC:-3.0}"
HOLD_SEC="${HOLD_SEC:-2.0}"
RAMP_DOWN_SEC="${RAMP_DOWN_SEC:-3.0}"
POST_SEC="${POST_SEC:-3.0}"
PUBLISH_RATE_HZ="${PUBLISH_RATE_HZ:-50.0}"
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
DATA_SPLIT="${DATA_SPLIT:-development}"
ATTEMPT="${ATTEMPT:-01}"

MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
STAMPED_CMD_TOPIC="${STAMPED_CMD_TOPIC:-/mocap_velocity_continuity/cmd_vel_stamped}"

[[ -s "${PUBLISHER}" ]] || fail "missing publisher: ${PUBLISHER}"
[[ -s "${ANALYZER}" ]] || fail "missing analyzer: ${ANALYZER}"
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${ATTEMPT}" =~ ^[0-9][0-9]$ ]] || fail "ATTEMPT must be two digits"
case "${DATA_SPLIT}" in
  development) SPLIT_TAG=DEV ;;
  validation) SPLIT_TAG=VAL ;;
  final_test) SPLIT_TAG=FINAL ;;
  *) fail "DATA_SPLIT must be development, validation, or final_test" ;;
esac

for value in \
  "${TARGET_MAGNITUDE}" "${PRE_SEC}" "${RAMP_UP_SEC}" "${HOLD_SEC}" \
  "${RAMP_DOWN_SEC}" "${POST_SEC}" "${PUBLISH_RATE_HZ}"; do
  is_number "${value}" || fail "all numeric settings must be non-negative decimal numbers"
done
awk -v value="${TARGET_MAGNITUDE}" -v maximum="${MAX_TARGET}" \
  'BEGIN {exit !(value > 0.0 && value <= maximum)}' || \
  fail "require 0 < TARGET_MAGNITUDE <= ${MAX_TARGET} ${SPEED_UNIT} for ${TEST_AXIS}"
for pair in \
  "PRE_SEC:${PRE_SEC}:2.0:10.0" \
  "RAMP_UP_SEC:${RAMP_UP_SEC}:2.0:10.0" \
  "HOLD_SEC:${HOLD_SEC}:1.0:6.0" \
  "RAMP_DOWN_SEC:${RAMP_DOWN_SEC}:2.0:10.0" \
  "POST_SEC:${POST_SEC}:2.0:10.0" \
  "PUBLISH_RATE_HZ:${PUBLISH_RATE_HZ}:20.0:100.0"; do
  IFS=: read -r name value minimum maximum <<< "${pair}"
  awk -v value="${value}" -v minimum="${minimum}" -v maximum="${maximum}" \
    'BEGIN {exit !(value >= minimum && value <= maximum)}' || \
    fail "require ${minimum} <= ${name} <= ${maximum}"
done

case "${TEST_AXIS}:${DIRECTION}" in
  linear:forward) SIGNED_TARGET="${TARGET_MAGNITUDE}"; DIRECTION_TAG=F ;;
  linear:reverse) SIGNED_TARGET="-${TARGET_MAGNITUDE}"; DIRECTION_TAG=R ;;
  angular:left) SIGNED_TARGET="${TARGET_MAGNITUDE}"; DIRECTION_TAG=L ;;
  angular:right) SIGNED_TARGET="-${TARGET_MAGNITUDE}"; DIRECTION_TAG=R ;;
  linear:*) fail "linear DIRECTION must be forward or reverse" ;;
  angular:*) fail "angular DIRECTION must be left or right" ;;
esac

MAG_TAG="$(awk -v value="${TARGET_MAGNITUDE}" 'BEGIN {printf "%03d", value * 100.0 + 0.5}')"
if [[ "${TEST_AXIS}" == linear ]]; then
  VALUE_TAG="V${MAG_TAG}"
else
  VALUE_TAG="W${MAG_TAG}"
fi
RUN_LABEL="${RUN_LABEL:-${SPLIT_TAG}_CHASSIS_CONT_${PROFILE_TAG}_${AXIS_TAG}_${DIRECTION_TAG}_${VALUE_TAG}_a${ATTEMPT}}"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"

RUN_OUT_DIR="${RUN_OUT_DIR:-${HOME}/slosh_bags/real/${DATE}_mocap_velocity_continuity_v1/${DATA_SPLIT}}"
BAG_PATH="${BAG_PATH:-${RUN_OUT_DIR}/${RUN_LABEL}.bag}"
METADATA_PATH="${METADATA_PATH:-${RUN_OUT_DIR}/${RUN_LABEL}_command.json}"
REPORT_PATH="${REPORT_PATH:-${RUN_OUT_DIR}/${RUN_LABEL}_continuity.json}"
PLOT_DIR="${PLOT_DIR:-${RUN_OUT_DIR}/${RUN_LABEL}_plots}"
RAW_MOCAP_TOPIC="/vrpn_client_node/${MOCAP_TRACKER}/pose"

if [[ "${PROFILE}" == constant_accel || "${PROFILE}" == trapezoidal_velocity ]]; then
  PROFILE_DESCRIPTION="constant acceleration; linear velocity ramp"
  PROFILE_FACTOR=1.0
  MOTION_ESTIMATE="$(awk \
    -v target="${TARGET_MAGNITUDE}" -v up="${RAMP_UP_SEC}" \
    -v hold="${HOLD_SEC}" -v down="${RAMP_DOWN_SEC}" \
    'BEGIN {printf "%.3f", target * (0.5 * up + hold + 0.5 * down)}')"
else
  PROFILE_DESCRIPTION="linearly increasing acceleration; quadratic velocity ramp"
  PROFILE_FACTOR=2.0
  MOTION_ESTIMATE="$(awk \
    -v target="${TARGET_MAGNITUDE}" -v up="${RAMP_UP_SEC}" \
    -v hold="${HOLD_SEC}" -v down="${RAMP_DOWN_SEC}" \
    'BEGIN {printf "%.3f", target * (up / 3.0 + hold + 2.0 * down / 3.0)}')"
fi
PEAK_ACCEL="$(awk \
  -v factor="${PROFILE_FACTOR}" -v target="${TARGET_MAGNITUDE}" \
  -v up="${RAMP_UP_SEC}" -v down="${RAMP_DOWN_SEC}" \
  'BEGIN {a=target/up; b=target/down; if (b>a) a=b; printf "%.4f", factor*a}')"
awk -v value="${PEAK_ACCEL}" -v maximum="${MAX_PROFILE_ACCEL}" \
  'BEGIN {exit !(value <= maximum)}' || \
  fail "profile peak acceleration ${PEAK_ACCEL} exceeds safe test limit ${MAX_PROFILE_ACCEL} ${ACCEL_UNIT}"
awk -v value="${MOTION_ESTIMATE}" -v maximum="${MAX_PROFILE_MOTION}" \
  'BEGIN {exit !(value <= maximum)}' || \
  fail "expected profile motion ${MOTION_ESTIMATE} exceeds site-safety limit ${MAX_PROFILE_MOTION}"

echo "================ chassis velocity continuity ================"
echo "  profile         = ${PROFILE}: ${PROFILE_DESCRIPTION}"
echo "  axis/direction  = ${TEST_AXIS} / ${DIRECTION}"
echo "  target          = ${SIGNED_TARGET} ${SPEED_UNIT}"
if [[ "${TEST_AXIS}" == linear ]]; then
  echo "  chassis rated   = high-speed mode max ${RATED_MAX_SPEED} m/s; not exercised here"
fi
echo "  phases          = zero ${PRE_SEC}s -> ramp ${RAMP_UP_SEC}s -> hold ${HOLD_SEC}s -> ramp-down ${RAMP_DOWN_SEC}s -> zero ${POST_SEC}s"
echo "  peak accel      = ${PEAK_ACCEL} ${ACCEL_UNIT}"
if [[ "${TEST_AXIS}" == linear ]]; then
  echo "  expected travel = about ${MOTION_ESTIMATE} m"
  echo "  clear distance  = require at least 5.0 m straight free space"
else
  echo "  expected turn   = about ${MOTION_ESTIMATE} rad"
fi
echo "  command rate    = ${PUBLISH_RATE_HZ} Hz"
echo "  mocap / IMU     = ${RAW_MOCAP_TOPIC} / ${IMU_TOPIC}"
echo "  output bag      = ${BAG_PATH}"
echo "  separated plots = ${PLOT_DIR}"
echo "=============================================================="

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS; motion NOT started"
  echo "Set VALIDATE_ONLY=false ARM_MOTION=YES only after checking clearance and E-stop."
  exit 0
fi

[[ "${ARM_MOTION}" == YES ]] || \
  fail "set ARM_MOTION=YES only after confirming level ground, clear motion space, and E-stop"
for path in "${BAG_PATH}" "${BAG_PATH}.active" "${METADATA_PATH}" "${REPORT_PATH}" "${PLOT_DIR}"; do
  [[ ! -e "${path}" ]] || fail "refusing to overwrite existing output: ${path}"
done

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

use_sim_time="$(rosparam get /use_sim_time 2>/dev/null || echo false)"
[[ "${use_sim_time}" != true ]] || fail "real test refuses /use_sim_time=true"
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
  --profile "${PROFILE}" \
  --target-value "${SIGNED_TARGET}" \
  --pre-sec "${PRE_SEC}" \
  --ramp-up-sec "${RAMP_UP_SEC}" \
  --hold-sec "${HOLD_SEC}" \
  --ramp-down-sec "${RAMP_DOWN_SEC}" \
  --post-sec "${POST_SEC}" \
  --rate-hz "${PUBLISH_RATE_HZ}" \
  --run-label "${RUN_LABEL}" \
  --data-split "${DATA_SPLIT}" \
  --attempt "${ATTEMPT}" \
  --arm-token MOCAP_VELOCITY_CONTINUITY_ARMED \
  --metadata "${METADATA_PATH}" &
publisher_pid=$!
publisher_status=0
wait "${publisher_pid}" || publisher_status=$?
publisher_pid=""
(( publisher_status == 0 )) || fail "continuity publisher exited with status ${publisher_status}"
publish_zero
motion_started=false
sleep 0.3
stop_recorder

[[ -s "${BAG_PATH}" ]] || fail "bag is missing or empty after recording: ${BAG_PATH}"
python3 "${ANALYZER}" "${BAG_PATH}" \
  --metadata "${METADATA_PATH}" \
  --axis "${TEST_AXIS}" \
  --profile "${PROFILE}" \
  --mocap-tracker "${MOCAP_TRACKER}" \
  --cmd-topic "${CMD_TOPIC}" \
  --imu-topic "${IMU_TOPIC}" \
  --odom-topic "${ODOM_TOPIC}" \
  --stamped-command-topic "${STAMPED_CMD_TOPIC}" \
  --run-label "${RUN_LABEL}" \
  --data-split "${DATA_SPLIT}" \
  --attempt "${ATTEMPT}" \
  --output-json "${REPORT_PATH}" \
  --plot-dir "${PLOT_DIR}"

trap - EXIT INT TERM
echo "[${SCRIPT_NAME}] PASS"
echo "  bag    : ${BAG_PATH}"
echo "  report : ${REPORT_PATH}"
echo "  plots  : ${PLOT_DIR}"
