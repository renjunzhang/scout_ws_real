#!/usr/bin/env bash
# Low-speed, explicitly armed PATH_SELECTION run. This is development evidence
# only and can never consume an R01--R05 execution-chain trial identifier.

set -euo pipefail

SCRIPT_NAME="run_spmpc_mocap_path_selection_trial"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"
MAP_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_field_map.py"
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

SELECTION_ID="${SELECTION_ID:-C01}"
ATTEMPT="${ATTEMPT:-01}"
ACQUISITION_RETRY="${ACQUISITION_RETRY:-false}"
RETRY_REASON_FILE="${RETRY_REASON_FILE:-}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"

[[ "${SELECTION_ID}" =~ ^C0[1-9]$ ]] || fail "SELECTION_ID must be C01..C09"
[[ "${ATTEMPT}" =~ ^0[1-9]$ ]] || fail "ATTEMPT must be 01..09"
if [[ "${ATTEMPT}" == "01" ]]; then
  truthy "${ACQUISITION_RETRY}" && \
    fail "ATTEMPT=01 cannot be marked as an acquisition retry"
else
  truthy "${ACQUISITION_RETRY}" || \
    fail "ATTEMPT=${ATTEMPT} requires ACQUISITION_RETRY=true"
  [[ -n "${RETRY_REASON_FILE}" && -s "${RETRY_REASON_FILE}" ]] || \
    fail "ATTEMPT=${ATTEMPT} requires a non-empty RETRY_REASON_FILE"
fi

PATH_FILE="${PATH_FILE:-}"
[[ -n "${PATH_FILE}" ]] || fail "PATH_FILE is required"
[[ -s "${PATH_FILE}" ]] || fail "candidate path is missing or empty: ${PATH_FILE}"
PATH_SHA256_FILE="${PATH_SHA256_FILE:-${PATH_FILE}.sha256}"
PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256:-}"
if [[ -z "${PATH_EXPECTED_SHA256}" && -s "${PATH_SHA256_FILE}" ]]; then
  PATH_EXPECTED_SHA256="$(awk 'NR == 1 {print $1}' "${PATH_SHA256_FILE}")"
fi
[[ "${PATH_EXPECTED_SHA256}" =~ ^[0-9a-fA-F]{64}$ ]] || \
  fail "set PATH_EXPECTED_SHA256 or provide ${PATH_SHA256_FILE}"

FIELD_MAP_FILE="${FIELD_MAP_FILE:-${LOCALIZATION_MAP_FILE:-}}"
FIELD_MAP_EXPECTED_SHA256="${FIELD_MAP_EXPECTED_SHA256:-${MAP_PBSTREAM_SHA256:-${LOCALIZATION_MAP_EXPECTED_SHA256:-}}}"
FIELD_MAP_RESOLUTION="${FIELD_MAP_RESOLUTION:-${MAP_RESOLUTION:-0.02}}"
[[ -n "${FIELD_MAP_FILE}" && -s "${FIELD_MAP_FILE}" ]] || \
  fail "source the small-field freeze.env so FIELD_MAP_FILE/LOCALIZATION_MAP_FILE is available"
[[ "${FIELD_MAP_EXPECTED_SHA256}" =~ ^[0-9a-fA-F]{64}$ ]] || \
  fail "small-field expected pbstream SHA-256 is required"
LARGE_FIELD_G3R3_MAP_SHA256="4663ae1406ca049f1070b4a84fdf48dbbc675d3fc632c8063345495166261d3b"
[[ "${FIELD_MAP_EXPECTED_SHA256,,}" != "${LARGE_FIELD_G3R3_MAP_SHA256}" ]] || \
  fail "PATH_SELECTION refuses the reserved large-field G3R3 map"

VARIANT="${VARIANT:-B_slosh_matched5}"
case "${VARIANT}" in
  B_slosh_matched0) DEFAULT_W_SLOSH=0.0 ;;
  B_slosh_matched5) DEFAULT_W_SLOSH=5.0 ;;
  *) DEFAULT_W_SLOSH="${W_SLOSH:-}" ;;
esac
W_SLOSH="${W_SLOSH:-${DEFAULT_W_SLOSH}}"
[[ -n "${W_SLOSH}" ]] || fail "W_SLOSH is required for custom VARIANT=${VARIANT}"

V_REF="${V_REF:-0.10}"
MAX_SELECTION_V_REF="${MAX_SELECTION_V_REF:-0.15}"
awk -v value="${V_REF}" -v maximum="${MAX_SELECTION_V_REF}" \
  'BEGIN {exit !(value > 0 && maximum > 0 && value <= maximum)}' || \
  fail "PATH_SELECTION requires 0 < V_REF <= MAX_SELECTION_V_REF (${MAX_SELECTION_V_REF})"

W_SMOOTH="${W_SMOOTH:-1.0}"
W_ALPHA="${W_ALPHA:-1.0}"
W_DU_A="${W_DU_A:-1.0}"
W_DU_VS="${W_DU_VS:-1.0}"
if [[ "${VARIANT}" == "B_slosh_matched0" ||
      "${VARIANT}" == "B_slosh_matched5" ]]; then
  expected_w_slosh=0.0
  [[ "${VARIANT}" == "B_slosh_matched5" ]] && expected_w_slosh=5.0
  awk -v value="${V_REF}" 'BEGIN {exit !(value > 0 && value <= 0.20)}' || \
    fail "matched release requires 0 < V_REF <= 0.20 m/s"
  for field in \
    "W_SLOSH:${W_SLOSH}:${expected_w_slosh}" \
    "W_SMOOTH:${W_SMOOTH}:1.0" \
    "W_ALPHA:${W_ALPHA}:1.0" \
    "W_DU_A:${W_DU_A}:1.0" \
    "W_DU_VS:${W_DU_VS}:1.0"; do
    IFS=: read -r label value expected <<< "${field}"
    awk -v value="${value}" -v expected="${expected}" \
      'BEGIN {delta=value-expected; if (delta < 0) delta=-delta; exit !(delta <= 1e-12)}' || \
      fail "matched release requires ${label}=${expected}, got ${value}"
  done
fi
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
MOCAP_HOST="${MOCAP_HOST:-192.168.203.85}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_front}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
RECORD_SEC="${RECORD_SEC:-70}"
START_POS_TOL="${START_POS_TOL:-0.08}"
START_YAW_TOL="${START_YAW_TOL:-0.15}"
START_HOLD_SEC="${START_HOLD_SEC:-0.5}"
START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC:-120}"

RUN_OUT_DIR="${PATH_SELECTION_OUT_DIR:-${HOME}/slosh_bags/real/${DATE}_spmpc_mocap_execution_chain/path_selection}"
RUN_LABEL="${RUN_LABEL:-DEV_MOCAP_PATH_SELECTION_${SELECTION_ID}_${VARIANT}_a${ATTEMPT}}"
NAME="${NAME:-${RUN_LABEL}}"
BAG_PATH="${RUN_OUT_DIR}/${NAME}.bag"
POSTFLIGHT_REPORT="${POSTFLIGHT_REPORT:-${RUN_OUT_DIR}/${NAME}_mocap_path_selection_postflight.json}"
PROTOCOL_META="${PROTOCOL_META:-${RUN_OUT_DIR}/${NAME}_protocol.env}"

for required_file in "${RUNNER}" "${PATH_VALIDATOR}" "${MAP_VALIDATOR}" "${POSTFLIGHT}"; do
  [[ -s "${required_file}" ]] || fail "missing required script: ${required_file}"
done
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"

actual_path_sha256="$(sha256sum "${PATH_FILE}" | awk '{print $1}')"
[[ "${actual_path_sha256,,}" == "${PATH_EXPECTED_SHA256,,}" ]] || \
  fail "path SHA-256 mismatch: expected=${PATH_EXPECTED_SHA256}, actual=${actual_path_sha256}"
python3 "${PATH_VALIDATOR}" "${PATH_FILE}" \
  --expected-sha256 "${PATH_EXPECTED_SHA256}" >/dev/null
python3 "${MAP_VALIDATOR}" "${FIELD_MAP_FILE}" \
  --expected-resolution "${FIELD_MAP_RESOLUTION}" \
  --expected-pbstream-sha256 "${FIELD_MAP_EXPECTED_SHA256}" >/dev/null

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS"
  echo "  run_class=PATH_SELECTION selection=${SELECTION_ID} attempt=${ATTEMPT} (does not consume R01--R05)"
  echo "  acquisition_retry=${ACQUISITION_RETRY} retry_reason_file=${RETRY_REASON_FILE:-none}"
  echo "  variant=${VARIANT} w_slosh=${W_SLOSH} v_ref=${V_REF} max_v_ref=${MAX_SELECTION_V_REF}"
  echo "  path=${PATH_FILE}"
  echo "  path_sha256=${PATH_EXPECTED_SHA256,,}"
  echo "  map=${FIELD_MAP_FILE}"
  echo "  map_sha256=${FIELD_MAP_EXPECTED_SHA256,,}"
  echo "  output=${BAG_PATH}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "set ARM_MOTION=YES only after localization PASS, path clearance and E-stop checks"
for output in "${BAG_PATH}" "${BAG_PATH}.active" "${POSTFLIGHT_REPORT}" "${PROTOCOL_META}"; do
  [[ ! -e "${output}" ]] || \
    fail "output already exists; preserve it and choose a documented new ATTEMPT: ${output}"
done

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

for topic in "${MAP_TOPIC}" "${SCAN_TOPIC}" "${ODOM_TOPIC}" "${IMU_TOPIC}"; do
  timeout 5s rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1 || \
    fail "required runtime topic is unavailable: ${topic}"
done
raw_mocap_topic="/vrpn_client_node/${MOCAP_TRACKER}/pose"
timeout 5s rostopic echo -n 1 "${raw_mocap_topic}" >/dev/null 2>&1 || \
  fail "no raw mocap pose on ${raw_mocap_topic}"
mocap_status="$(timeout 5s rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
grep -q "OK.*tracker=${MOCAP_TRACKER}" <<< "${mocap_status}" || \
  fail "/mocap/status is not OK for ${MOCAP_TRACKER}"

runtime_map_file="$(rosparam get /cartographer_node/frozen_map_file 2>/dev/null || true)"
runtime_map_sha256="$(rosparam get /cartographer_node/frozen_map_expected_sha256 2>/dev/null || true)"
[[ -n "${runtime_map_file}" && -e "${runtime_map_file}" ]] || \
  fail "Cartographer runtime frozen_map_file is missing"
[[ "$(readlink -f "${runtime_map_file}")" == "$(readlink -f "${FIELD_MAP_FILE}")" ]] || \
  fail "Cartographer runtime map does not match the selected small-field map"
[[ "${runtime_map_sha256,,}" == "${FIELD_MAP_EXPECTED_SHA256,,}" ]] || \
  fail "Cartographer runtime expected map SHA-256 does not match freeze.env"

published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
if grep -Fxq -- /cmd_vel <<< "${published_topics}"; then
  fail "/cmd_vel already has a publisher; stop stale planner and teleop"
fi

mkdir -p "${RUN_OUT_DIR}"
{
  printf 'protocol_id=%s\n' 'SMPCC_mocap_path_selection_v1'
  printf 'run_class=%s\n' 'PATH_SELECTION'
  printf 'selection_id=%s\n' "${SELECTION_ID}"
  printf 'attempt=%s\n' "${ATTEMPT}"
  printf 'acquisition_retry=%s\n' "${ACQUISITION_RETRY}"
  printf 'retry_reason_file=%s\n' "${RETRY_REASON_FILE}"
  if [[ -n "${RETRY_REASON_FILE}" ]]; then
    printf 'retry_reason_sha256=%s\n' "$(sha256sum "${RETRY_REASON_FILE}" | awk '{print $1}')"
  fi
  printf 'formal_trial_consumed=%s\n' 'false'
  printf 'variant=%s\n' "${VARIANT}"
  printf 'v_ref=%s\n' "${V_REF}"
  printf 'max_selection_v_ref=%s\n' "${MAX_SELECTION_V_REF}"
  printf 'w_slosh=%s\n' "${W_SLOSH}"
  printf 'path_file=%s\n' "${PATH_FILE}"
  printf 'path_sha256=%s\n' "${PATH_EXPECTED_SHA256,,}"
  printf 'map_file=%s\n' "$(readlink -f "${FIELD_MAP_FILE}")"
  printf 'map_sha256=%s\n' "${FIELD_MAP_EXPECTED_SHA256,,}"
  printf 'mocap_host=%s\n' "${MOCAP_HOST}"
  printf 'mocap_tracker=%s\n' "${MOCAP_TRACKER}"
  printf 'run_label=%s\n' "${RUN_LABEL}"
  printf 'created_at=%s\n' "$(date --iso-8601=seconds)"
} > "${PROTOCOL_META}"

echo "================ mocap PATH_SELECTION ================"
echo "  selection     = ${SELECTION_ID}/${ATTEMPT}; retry=${ACQUISITION_RETRY}; formal_trial_consumed=false"
echo "  variant       = ${VARIANT}; w_slosh=${W_SLOSH}; v_ref=${V_REF}"
echo "  path          = ${PATH_FILE}"
echo "  map           = ${FIELD_MAP_FILE}"
echo "  output        = ${BAG_PATH}"
echo "========================================================"

DATE="${DATE}" STAMP="${STAMP}" PILOT_MODE=true PILOT_METHOD= \
VARIANT="${VARIANT}" PILOT_CONDITION=SMPCC_mocap_path_selection_v1 \
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
RECORD_STANDALONE_SLOSH=false RECORD_SCAN=true FORBID_IMAGE_STREAMS=true \
RECORD_ALL_EXISTING_TOPICS=false RECORD_TOPIC_INFO=true \
RECORD_MOCAP=true RECORD_MOCAP_PATH=false MOCAP_TRACKER="${MOCAP_TRACKER}" \
RECORD_SEC="${RECORD_SEC}" MAX_RECORD_SEC="${RECORD_SEC}" \
BLOCK_SEGMENT_ID="MOCAP_PATH_SELECTION_${SELECTION_ID}_A${ATTEMPT}" SPLIT_BLOCK=false \
ORDER_POSITION="${SELECTION_ID#C}" SEND_ZERO_ON_EXIT=true \
ACQUISITION_RETRY="${ACQUISITION_RETRY}" RETRY_REASON_FILE="${RETRY_REASON_FILE}" \
OPERATOR_NOTE="low-speed PATH_SELECTION development run; never R01-R05" \
bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"
python3 "${POSTFLIGHT}" "${BAG_PATH}" \
  --variant "${VARIANT}" \
  --mocap-tracker "${MOCAP_TRACKER}" \
  --imu-topic "${IMU_TOPIC}" \
  --path-file "${PATH_FILE}" \
  --path-sha256 "${PATH_EXPECTED_SHA256}" \
  --report "${POSTFLIGHT_REPORT}"

echo "[${SCRIPT_NAME}] PASS: PATH_SELECTION ${SELECTION_ID}/${ATTEMPT}; R01--R05 remain unused"
