#!/usr/bin/env bash
# Run one frozen-path S-MPCC execution-chain trial with passive NOKOV recording.

set -euo pipefail

SCRIPT_NAME="run_spmpc_mocap_execution_chain_trial"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"
POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_mocap_execution_chain_bag.py"

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[${SCRIPT_NAME}][ERR] $*" >&2
  exit 2
}

TRIAL_ID="${TRIAL_ID:-}"
ATTEMPT="${ATTEMPT:-01}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"

case "${TRIAL_ID}" in
  R01|R02|R03|R04|R05) ;;
  *) fail "set TRIAL_ID=R01..R05" ;;
esac
[[ "${ATTEMPT}" == "01" ]] || fail "only ATTEMPT=01 is preregistered"

PATH_FILE="${PATH_FILE:-}"
[[ -n "${PATH_FILE}" ]] || fail "PATH_FILE is required"
[[ -s "${PATH_FILE}" ]] || fail "frozen path is missing or empty: ${PATH_FILE}"
PATH_SHA256_FILE="${PATH_SHA256_FILE:-${PATH_FILE}.sha256}"
PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256:-}"
if [[ -z "${PATH_EXPECTED_SHA256}" && -s "${PATH_SHA256_FILE}" ]]; then
  PATH_EXPECTED_SHA256="$(awk 'NR == 1 {print $1}' "${PATH_SHA256_FILE}")"
fi
[[ "${PATH_EXPECTED_SHA256}" =~ ^[0-9a-fA-F]{64}$ ]] || \
  fail "set PATH_EXPECTED_SHA256 or provide ${PATH_SHA256_FILE}"

VARIANT="${VARIANT:-B_slosh_matched5}"
case "${VARIANT}" in
  B_slosh_matched0) DEFAULT_W_SLOSH=0.0 ;;
  B_slosh_matched5) DEFAULT_W_SLOSH=5.0 ;;
  *) DEFAULT_W_SLOSH="${W_SLOSH:-}" ;;
esac
W_SLOSH="${W_SLOSH:-${DEFAULT_W_SLOSH}}"
[[ -n "${W_SLOSH}" ]] || fail "W_SLOSH is required for custom VARIANT=${VARIANT}"

V_REF="${V_REF:-0.20}"
W_SMOOTH="${W_SMOOTH:-1.0}"
W_ALPHA="${W_ALPHA:-1.0}"
W_DU_A="${W_DU_A:-1.0}"
W_DU_VS="${W_DU_VS:-1.0}"
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
MOCAP_HOST="${MOCAP_HOST:-192.168.203.85}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
RECORD_SEC="${RECORD_SEC:-70}"
START_POS_TOL="${START_POS_TOL:-0.08}"
START_YAW_TOL="${START_YAW_TOL:-0.15}"
START_HOLD_SEC="${START_HOLD_SEC:-0.5}"
START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC:-120}"

RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_mocap_execution_chain/S_path}"
RUN_LABEL="${RUN_LABEL:-DEV_MOCAP_EXEC_S_${VARIANT}_${TRIAL_ID}_a${ATTEMPT}}"
NAME="${NAME:-${RUN_LABEL}}"
BAG_PATH="${RUN_OUT_DIR}/${NAME}.bag"
POSTFLIGHT_REPORT="${POSTFLIGHT_REPORT:-${RUN_OUT_DIR}/${NAME}_mocap_execution_postflight.json}"
PROTOCOL_META="${PROTOCOL_META:-${RUN_OUT_DIR}/${NAME}_protocol.env}"

for required_file in "${RUNNER}" "${PATH_VALIDATOR}" "${POSTFLIGHT}"; do
  [[ -s "${required_file}" ]] || fail "missing required script: ${required_file}"
done
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"

actual_path_sha256="$(sha256sum "${PATH_FILE}" | awk '{print $1}')"
[[ "${actual_path_sha256,,}" == "${PATH_EXPECTED_SHA256,,}" ]] || \
  fail "path SHA-256 mismatch: expected=${PATH_EXPECTED_SHA256}, actual=${actual_path_sha256}"
python3 "${PATH_VALIDATOR}" "${PATH_FILE}" \
  --expected-sha256 "${PATH_EXPECTED_SHA256}" >/dev/null

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS"
  echo "  trial=${TRIAL_ID} variant=${VARIANT} w_slosh=${W_SLOSH} v_ref=${V_REF}"
  echo "  path=${PATH_FILE}"
  echo "  path_sha256=${PATH_EXPECTED_SHA256}"
  echo "  mocap=${MOCAP_TRACKER}@${MOCAP_HOST}"
  echo "  output=${BAG_PATH}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "set ARM_MOTION=YES only after the S path, clearance, and E-stop are confirmed"
[[ ! -e "${BAG_PATH}" && ! -e "${BAG_PATH}.active" ]] || \
  fail "output bag already exists: ${BAG_PATH}"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

raw_mocap_topic="/vrpn_client_node/${MOCAP_TRACKER}/pose"
timeout 5s rostopic echo -n 1 "${raw_mocap_topic}" >/dev/null 2>&1 || \
  fail "no raw mocap pose on ${raw_mocap_topic}"
mocap_status="$(timeout 5s rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
grep -q "OK.*tracker=${MOCAP_TRACKER}" <<< "${mocap_status}" || \
  fail "/mocap/status is not OK for ${MOCAP_TRACKER}"
timeout 5s rostopic echo -n 1 "${ODOM_TOPIC}" >/dev/null 2>&1 || \
  fail "no odometry on ${ODOM_TOPIC}"
timeout 5s rostopic echo -n 1 "${IMU_TOPIC}" >/dev/null 2>&1 || \
  fail "no IMU on ${IMU_TOPIC}"

published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
if grep -Fxq -- /cmd_vel <<< "${published_topics}"; then
  fail "/cmd_vel already has a publisher; stop the stale planner/teleop before this trial"
fi

mkdir -p "${RUN_OUT_DIR}"
{
  printf 'protocol_id=%s\n' 'SMPCC_mocap_execution_chain_v1'
  printf 'trial_id=%s\n' "${TRIAL_ID}"
  printf 'attempt=%s\n' "${ATTEMPT}"
  printf 'variant=%s\n' "${VARIANT}"
  printf 'v_ref=%s\n' "${V_REF}"
  printf 'w_slosh=%s\n' "${W_SLOSH}"
  printf 'path_file=%s\n' "${PATH_FILE}"
  printf 'path_sha256=%s\n' "${PATH_EXPECTED_SHA256,,}"
  printf 'mocap_host=%s\n' "${MOCAP_HOST}"
  printf 'mocap_tracker=%s\n' "${MOCAP_TRACKER}"
  printf 'raw_mocap_topic=%s\n' "${raw_mocap_topic}"
  printf 'run_label=%s\n' "${RUN_LABEL}"
  printf 'created_at=%s\n' "$(date --iso-8601=seconds)"
} > "${PROTOCOL_META}"

echo "================ mocap execution-chain trial ================"
echo "  trial/attempt = ${TRIAL_ID}/${ATTEMPT}"
echo "  variant       = ${VARIANT}; w_slosh=${W_SLOSH}; v_ref=${V_REF}"
echo "  path          = ${PATH_FILE}"
echo "  path sha256   = ${PATH_EXPECTED_SHA256,,}"
echo "  mocap         = ${raw_mocap_topic}"
echo "  output        = ${BAG_PATH}"
echo "==============================================================="

DATE="${DATE}" STAMP="${STAMP}" PILOT_MODE=false \
VARIANT="${VARIANT}" PILOT_CONDITION=SMPCC_mocap_execution_chain_v1 \
RUN_LABEL="${RUN_LABEL}" NAME="${NAME}" RUN_OUT_DIR="${RUN_OUT_DIR}" \
PATH_SOURCE_MODE=replay PATH_FILE="${PATH_FILE}" \
PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256}" REQUIRE_PATH_HASH=true \
START_POS_TOL="${START_POS_TOL}" START_YAW_TOL="${START_YAW_TOL}" \
START_HOLD_SEC="${START_HOLD_SEC}" START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC}" \
V_REF="${V_REF}" W_SLOSH="${W_SLOSH}" W_SMOOTH="${W_SMOOTH}" \
W_ALPHA="${W_ALPHA}" W_DU_A="${W_DU_A}" W_DU_VS="${W_DU_VS}" \
DELAY_PHASE_MODE=shadow IMU_SHADOW_ENABLE=true IMU_TOPIC="${IMU_TOPIC}" \
IMU_SUBSCRIBER_QUEUE_SIZE=10 CURRENT_OBSERVER_SOURCE=processed_imu \
OBSERVER_FALLBACK_POLICY=fail_closed OBSERVER_LATCH_FALLBACK=false \
STATE_TIMING_REQUIRE_COMMON_EPOCH=true STATE_TIMING_MAX_RAW_SKEW_SEC=0.080 \
STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=0.050 \
STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC=0.010 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=false SHARED_ANGULAR_LIMIT_ENABLE=false \
EXECUTION_CONTRACT_FAIL_CLOSED=true EXECUTION_CONTRACT_MAX_DELTA_V=0.0001 \
EXECUTION_CONTRACT_MAX_DELTA_OMEGA=0.0001 \
RECORD_RGB=false RECORD_CAMERA=false RECORD_CAMERA_INFO=false \
RECORD_CAMERA_COMPRESSED=false RECORD_DEPTH=false \
RECORD_ONLINE_LIQUID=false RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false \
RECORD_STANDALONE_SLOSH=false RECORD_SCAN=false FORBID_IMAGE_STREAMS=true \
RECORD_ALL_EXISTING_TOPICS=false RECORD_TOPIC_INFO=true \
RECORD_MOCAP=true RECORD_MOCAP_PATH=false MOCAP_TRACKER="${MOCAP_TRACKER}" \
RECORD_SEC="${RECORD_SEC}" MAX_RECORD_SEC="${RECORD_SEC}" \
BLOCK_SEGMENT_ID="MOCAP_EXEC_${TRIAL_ID}" SPLIT_BLOCK=false \
ORDER_POSITION="${TRIAL_ID#R}" ACQUISITION_RETRY=false SEND_ZERO_ON_EXIT=true \
bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"
python3 "${POSTFLIGHT}" "${BAG_PATH}" \
  --variant "${VARIANT}" \
  --mocap-tracker "${MOCAP_TRACKER}" \
  --imu-topic "${IMU_TOPIC}" \
  --path-file "${PATH_FILE}" \
  --path-sha256 "${PATH_EXPECTED_SHA256}" \
  --report "${POSTFLIGHT_REPORT}"

echo "[${SCRIPT_NAME}] PASS: ${BAG_PATH}"
