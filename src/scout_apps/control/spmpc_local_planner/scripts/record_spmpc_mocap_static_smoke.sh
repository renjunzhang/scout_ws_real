#!/usr/bin/env bash
# Record a stationary NOKOV/odom/IMU timing smoke. This script never starts a
# planner, publishes a goal, or publishes /cmd_vel.

set -euo pipefail

SCRIPT_NAME="record_spmpc_mocap_static_smoke"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RECORDER="${SCRIPT_DIR}/record_spmpc_full_rgb_bag.sh"
POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_mocap_execution_chain_bag.py"

fail() {
  echo "[${SCRIPT_NAME}][ERR] $*" >&2
  exit 2
}

DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
SMOKE_SEC="${SMOKE_SEC:-15}"
MOCAP_HOST="${MOCAP_HOST:-192.168.203.85}"
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
OUT_DIR="${OUT_DIR:-${HOME}/slosh_bags/real/${DATE}_spmpc_mocap_execution_chain/static_smoke}"
NAME="${NAME:-MOCAP_STATIC_${STAMP}}"
BAG_PATH="${OUT_DIR}/${NAME}.bag"
REPORT_PATH="${REPORT_PATH:-${OUT_DIR}/${NAME}_mocap_static_smoke_postflight.json}"
PROTOCOL_META="${PROTOCOL_META:-${OUT_DIR}/${NAME}_protocol.env}"

MIN_MOCAP_RATE_HZ="${MIN_MOCAP_RATE_HZ:-60.0}"
MAX_MOCAP_GAP_SEC="${MAX_MOCAP_GAP_SEC:-0.10}"
MAX_STATIC_POSITION_P95_M="${MAX_STATIC_POSITION_P95_M:-0.01}"
MAX_STATIC_YAW_P95_RAD="${MAX_STATIC_YAW_P95_RAD:-0.03}"

[[ -s "${RECORDER}" ]] || fail "missing recorder: ${RECORDER}"
[[ -s "${POSTFLIGHT}" ]] || fail "missing postflight: ${POSTFLIGHT}"
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${NAME}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe NAME"
case "${SMOKE_SEC}" in
  ''|*[!0-9]*) fail "SMOKE_SEC must be an integer >= 10" ;;
esac
(( SMOKE_SEC >= 10 )) || fail "SMOKE_SEC must be >= 10"
[[ ! -e "${BAG_PATH}" && ! -e "${BAG_PATH}.active" ]] || \
  fail "output already exists: ${BAG_PATH}"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

raw_mocap_topic="/vrpn_client_node/${MOCAP_TRACKER}/pose"
timeout 5s rostopic echo -n 1 "${raw_mocap_topic}" >/dev/null 2>&1 || \
  fail "no raw mocap pose on ${raw_mocap_topic}"
timeout 5s rostopic echo -n 1 /mocap/scout_pose >/dev/null 2>&1 || \
  fail "no bridged pose on /mocap/scout_pose"
mocap_status="$(timeout 5s rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
grep -q "OK.*tracker=${MOCAP_TRACKER}" <<< "${mocap_status}" || \
  fail "/mocap/status is not OK for ${MOCAP_TRACKER}"
timeout 5s rostopic echo -n 1 /odom >/dev/null 2>&1 || fail "no odometry on /odom"
timeout 5s rostopic echo -n 1 "${IMU_TOPIC}" >/dev/null 2>&1 || \
  fail "no IMU on ${IMU_TOPIC}"

published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
if grep -Fxq -- /cmd_vel <<< "${published_topics}"; then
  fail "/cmd_vel has a publisher; stop planner/teleop before the stationary smoke"
fi

mkdir -p "${OUT_DIR}"
{
  printf 'protocol_id=%s\n' 'SMPCC_mocap_execution_chain_v1'
  printf 'run_class=%s\n' 'STATIC_MOCAP_SMOKE'
  printf 'duration_sec=%s\n' "${SMOKE_SEC}"
  printf 'mocap_host=%s\n' "${MOCAP_HOST}"
  printf 'mocap_tracker=%s\n' "${MOCAP_TRACKER}"
  printf 'raw_mocap_topic=%s\n' "${raw_mocap_topic}"
  printf 'imu_topic=%s\n' "${IMU_TOPIC}"
  printf 'created_at=%s\n' "$(date --iso-8601=seconds)"
} > "${PROTOCOL_META}"

echo "[${SCRIPT_NAME}] keep the robot completely still for ${SMOKE_SEC}s"
DATE="${DATE}" STAMP="${STAMP}" VARIANT=MOCAP_STATIC_SMOKE \
RUN_LABEL="${NAME}" NAME="${NAME}" OUT_DIR="${OUT_DIR}" \
RECORD_SEC="${SMOKE_SEC}" RECORD_ALL_EXISTING_TOPICS=false \
RECORD_RGB=false RECORD_CAMERA=false RECORD_CAMERA_INFO=false \
RECORD_CAMERA_COMPRESSED=false RECORD_DEPTH=false RECORD_SCAN=false \
RECORD_STANDALONE_SLOSH=false RECORD_ONLINE_LIQUID=false \
RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false FORBID_IMAGE_STREAMS=true \
RECORD_MOCAP=true RECORD_MOCAP_PATH=false MOCAP_TRACKER="${MOCAP_TRACKER}" \
RECORD_ROSOUT=true RECORD_TOPIC_INFO=true IMU_TOPIC="${IMU_TOPIC}" \
LIQUID_EXPORT_AFTER_RECORD=false OPERATOR_NOTE="stationary mocap timing smoke; no motion" \
bash "${RECORDER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after recorder: ${BAG_PATH}"
python3 "${POSTFLIGHT}" "${BAG_PATH}" \
  --mode static \
  --mocap-tracker "${MOCAP_TRACKER}" \
  --imu-topic "${IMU_TOPIC}" \
  --min-duration-sec 10.0 \
  --min-mocap-rate-hz "${MIN_MOCAP_RATE_HZ}" \
  --max-mocap-gap-sec "${MAX_MOCAP_GAP_SEC}" \
  --max-static-position-p95-m "${MAX_STATIC_POSITION_P95_M}" \
  --max-static-yaw-p95-rad "${MAX_STATIC_YAW_P95_RAD}" \
  --report "${REPORT_PATH}"

echo "[${SCRIPT_NAME}] PASS: ${BAG_PATH}"
