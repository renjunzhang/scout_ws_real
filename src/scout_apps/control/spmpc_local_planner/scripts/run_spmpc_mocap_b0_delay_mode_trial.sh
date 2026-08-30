#!/usr/bin/env bash
# Development-only B0 delay-mode A/B trial on the frozen C02_v2 mocap path.
# This wrapper never consumes the formal R01--R05 identifiers.  It defaults to
# validate-only and supports exactly the two modes needed for delay diagnosis:
# shadow (prediction is diagnostic only) and fixed_robot_only (robot prediction
# replaces SolverInput.robot; liquid state remains diagnostic and unconsumed).

set -euo pipefail

SCRIPT_NAME="run_spmpc_mocap_b0_delay_mode_trial"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"
MAP_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_field_map.py"
POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_mocap_execution_chain_bag.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
DELAY_GATE_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_b0_delay_mode_summary.py"

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

DELAY_TEST_ID="${DELAY_TEST_ID:-D01}"
ATTEMPT="${ATTEMPT:-01}"
DELAY_PHASE_MODE="${DELAY_PHASE_MODE:-fixed_robot_only}"
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"

[[ "${DELAY_TEST_ID}" =~ ^D0[1-9]$ ]] || fail "DELAY_TEST_ID must be D01..D09"
[[ "${ATTEMPT}" =~ ^0[1-9]$ ]] || fail "ATTEMPT must be 01..09"
case "${DELAY_PHASE_MODE}" in
  shadow) EXPECTED_MODE_CODE=2 ;;
  fixed_robot_only) EXPECTED_MODE_CODE=4 ;;
  *) fail "DELAY_PHASE_MODE must be shadow|fixed_robot_only for this diagnostic" ;;
esac

VARIANT="${VARIANT:-B0}"
[[ "${VARIANT}" == "B0" ]] || fail "this diagnostic is B0-only; got VARIANT=${VARIANT}"
W_SLOSH="${W_SLOSH:-0.0}"
awk -v value="${W_SLOSH}" 'BEGIN {exit !(value == 0.0)}' || \
  fail "B0 delay diagnostic requires W_SLOSH=0.0"

V_REF="${V_REF:-0.10}"
awk -v value="${V_REF}" \
  'BEGIN {delta=value-0.10; if (delta < 0) delta=-delta; exit !(delta <= 1e-12)}' || \
  fail "B0 delay diagnostic requires V_REF=0.10 m/s"

SPEED_SAFETY_ENABLE="${SPEED_SAFETY_ENABLE:-true}"
truthy "${SPEED_SAFETY_ENABLE}" || \
  fail "B0 delay diagnostic requires SPEED_SAFETY_ENABLE=true"
SPEED_SAFETY_ENABLE=true
V_SAFE_MAX="${V_SAFE_MAX:-0.15}"
SPEED_SAFETY_TOLERANCE="${SPEED_SAFETY_TOLERANCE:-0.0001}"
awk -v value="${V_SAFE_MAX}" \
  'BEGIN {delta=value-0.15; if (delta < 0) delta=-delta; exit !(delta <= 1e-12)}' || \
  fail "B0 delay diagnostic requires V_SAFE_MAX=0.15 m/s"
awk -v value="${SPEED_SAFETY_TOLERANCE}" \
  'BEGIN {delta=value-0.0001; if (delta < 0) delta=-delta; exit !(delta <= 1e-12)}' || \
  fail "B0 delay diagnostic requires SPEED_SAFETY_TOLERANCE=0.0001 m/s"

W_SMOOTH="${W_SMOOTH:-1.0}"
W_ALPHA="${W_ALPHA:-1.0}"
W_DU_A="${W_DU_A:-1.0}"
W_DU_VS="${W_DU_VS:-1.0}"
for field in \
  "W_SMOOTH:${W_SMOOTH}" \
  "W_ALPHA:${W_ALPHA}" \
  "W_DU_A:${W_DU_A}" \
  "W_DU_VS:${W_DU_VS}"; do
  IFS=: read -r label value <<< "${field}"
  awk -v value="${value}" 'BEGIN {delta=value-1.0; if (delta < 0) delta=-delta; exit !(delta <= 1e-12)}' || \
    fail "${label} must remain 1.0 for the paired delay diagnostic"
done

DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC:-0.15}"
DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC:-0.22}"
awk -v value="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
  'BEGIN {delta=value-0.15; if (delta < 0) delta=-delta; exit !(delta <= 1e-12)}' || \
  fail "DELAY_PHASE_LINEAR_DELAY_SEC must remain 0.15"
awk -v value="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
  'BEGIN {delta=value-0.22; if (delta < 0) delta=-delta; exit !(delta <= 1e-12)}' || \
  fail "DELAY_PHASE_ANGULAR_DELAY_SEC must remain 0.22"

FROZEN_PATH_FILE="/home/geist/fixed_paths/real/20260829_spmpc_mocap_execution_chain/candidates/mocap_compact_s_C02.json"
FROZEN_PATH_SHA256="1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164"
PATH_FILE="${PATH_FILE:-${FROZEN_PATH_FILE}}"
PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256:-${FROZEN_PATH_SHA256}}"
[[ "${PATH_EXPECTED_SHA256,,}" == "${FROZEN_PATH_SHA256}" ]] || \
  fail "this diagnostic is frozen to C02_v2 SHA-256 ${FROZEN_PATH_SHA256}"

FROZEN_MAP_FILE="/home/geist/scout_maps/real/20260829_mocap_exec/map_carto_20260829_mocap_exec_v1.pbstream"
FROZEN_MAP_SHA256="34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595"
FIELD_MAP_FILE="${FIELD_MAP_FILE:-${LOCALIZATION_MAP_FILE:-${FROZEN_MAP_FILE}}}"
FIELD_MAP_EXPECTED_SHA256="${FIELD_MAP_EXPECTED_SHA256:-${MAP_PBSTREAM_SHA256:-${LOCALIZATION_MAP_EXPECTED_SHA256:-${FROZEN_MAP_SHA256}}}}"
FIELD_MAP_RESOLUTION="${FIELD_MAP_RESOLUTION:-${MAP_RESOLUTION:-0.02}}"
[[ "${FIELD_MAP_EXPECTED_SHA256,,}" == "${FROZEN_MAP_SHA256}" ]] || \
  fail "this diagnostic is frozen to the 20260829 mocap map SHA-256 ${FROZEN_MAP_SHA256}"

MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
MOCAP_HOST="${MOCAP_HOST:-192.168.203.85}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_front}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
RECORD_SEC="${RECORD_SEC:-90}"
START_POS_TOL="${START_POS_TOL:-0.08}"
START_YAW_TOL="${START_YAW_TOL:-0.15}"
START_HOLD_SEC="${START_HOLD_SEC:-0.5}"
START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC:-120}"

case "${RECORD_SEC}" in
  ''|*[!0-9]*) fail "RECORD_SEC must be an integer in [20,90]" ;;
esac
(( RECORD_SEC >= 20 && RECORD_SEC <= 90 )) || fail "RECORD_SEC must be in [20,90]"

RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_mocap_execution_chain/delay_diagnostic}"
RUN_LABEL="${RUN_LABEL:-DEV_MOCAP_B0_DELAY_${DELAY_TEST_ID}_${DELAY_PHASE_MODE}_a${ATTEMPT}}"
NAME="${NAME:-${RUN_LABEL}}"
BAG_PATH="${RUN_OUT_DIR}/${NAME}.bag"
POSTFLIGHT_REPORT="${POSTFLIGHT_REPORT:-${RUN_OUT_DIR}/${NAME}_mocap_delay_postflight.json}"
SUMMARY_JSON="${RUN_OUT_DIR}/${NAME}_summary.json"
DELAY_GATE_REPORT="${RUN_OUT_DIR}/${NAME}_delay_mode_gate.json"
PROTOCOL_META="${PROTOCOL_META:-${RUN_OUT_DIR}/${NAME}_protocol.env}"

for required_file in \
  "${RUNNER}" "${PATH_VALIDATOR}" "${MAP_VALIDATOR}" "${POSTFLIGHT}" "${SUMMARIZER}" \
  "${DELAY_GATE_VALIDATOR}" \
  "${PATH_FILE}" "${FIELD_MAP_FILE}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"

actual_path_sha256="$(sha256sum "${PATH_FILE}" | awk '{print $1}')"
[[ "${actual_path_sha256,,}" == "${FROZEN_PATH_SHA256}" ]] || \
  fail "C02_v2 path SHA-256 mismatch: expected=${FROZEN_PATH_SHA256}, actual=${actual_path_sha256}"
actual_map_sha256="$(sha256sum "${FIELD_MAP_FILE}" | awk '{print $1}')"
[[ "${actual_map_sha256,,}" == "${FROZEN_MAP_SHA256}" ]] || \
  fail "mocap map SHA-256 mismatch: expected=${FROZEN_MAP_SHA256}, actual=${actual_map_sha256}"

python3 "${PATH_VALIDATOR}" "${PATH_FILE}" \
  --expected-sha256 "${FROZEN_PATH_SHA256}" >/dev/null
python3 "${MAP_VALIDATOR}" "${FIELD_MAP_FILE}" \
  --expected-resolution "${FIELD_MAP_RESOLUTION}" \
  --expected-pbstream-sha256 "${FROZEN_MAP_SHA256}" >/dev/null

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

validate_launch_contract() {
  local launch_dump
  launch_dump="$(roslaunch --dump-params spmpc_local_planner spmpc_fixed_path.launch \
    planner_variant:=B0 \
    delay_phase_mode:="${DELAY_PHASE_MODE}" \
    delay_phase_linear_delay_sec:="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
    delay_phase_angular_delay_sec:="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
    imu_shadow_enable:=true \
    imu_subscriber_queue_size:=10 \
    observer_source:=processed_imu \
    observer_fallback_policy:=fail_closed \
    observer_latch_fallback:=false \
    speed_safety_enable:="${SPEED_SAFETY_ENABLE}" \
    v_safe_max:="${V_SAFE_MAX}" \
    speed_safety_tolerance:="${SPEED_SAFETY_TOLERANCE}" \
    v_ref:="${V_REF}" \
    w_slosh:=0.0 \
    w_smooth:=1.0 \
    w_alpha:=1.0 \
    w_du_a:=1.0 \
    w_du_vs:=1.0)" || fail "could not dump the B0 delay diagnostic launch parameters"

  local expected_lines=(
    "/spmpc_local_planner/planner_variant: B0"
    "/spmpc_local_planner/delay_phase/mode: ${DELAY_PHASE_MODE}"
    "/spmpc_local_planner/delay_phase/linear_delay_sec: 0.15"
    "/spmpc_local_planner/delay_phase/angular_delay_sec: 0.22"
    "/spmpc_local_planner/imu_shadow/enable: true"
    "/spmpc_local_planner/slosh_observer/source: processed_imu"
    "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
    "/spmpc_local_planner/speed_safety/enable: true"
    "/spmpc_local_planner/speed_safety/v_safe_max: 0.15"
    "/spmpc_local_planner/speed_safety/tolerance: 0.0001"
    "/spmpc_local_planner/variants/B0/slosh_enable: false"
    "/spmpc_local_planner/variants/B0/w_slosh: 0.0"
    "/spmpc_local_planner/variants/B0/v_ref: 0.1"
  )
  local expected_line
  for expected_line in "${expected_lines[@]}"; do
    grep -Fqx -- "${expected_line}" <<< "${launch_dump}" || \
      fail "launch contract missing: ${expected_line}"
  done
  echo "[${SCRIPT_NAME}] launch contract PASS (mode=${DELAY_PHASE_MODE}, code=${EXPECTED_MODE_CODE})"
}

validate_launch_contract

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS"
  echo "  run_class      = B0_DELAY_DIAGNOSTIC; formal_trial_consumed=false"
  echo "  diagnostic     = ${DELAY_TEST_ID}/${ATTEMPT}"
  echo "  mode/code      = ${DELAY_PHASE_MODE}/${EXPECTED_MODE_CODE}"
  echo "  robot state    = $([[ "${DELAY_PHASE_MODE}" == "fixed_robot_only" ]] && echo predicted || echo measured)"
  echo "  liquid state   = diagnostic only; B0 does not consume it"
  echo "  speed contract = v_ref=0.10; hard ceiling=0.15 m/s; fail-closed=true"
  echo "  path sha256    = ${FROZEN_PATH_SHA256}"
  echo "  map sha256     = ${FROZEN_MAP_SHA256}"
  echo "  output         = ${BAG_PATH}"
  echo "  motion         = NOT STARTED"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "real motion is disarmed; keep VALIDATE_ONLY=true until ready, then set ARM_MOTION=YES"

for output in \
  "${BAG_PATH}" "${BAG_PATH}.active" "${POSTFLIGHT_REPORT}" \
  "${SUMMARY_JSON}" "${DELAY_GATE_REPORT}" "${PROTOCOL_META}"; do
  [[ ! -e "${output}" ]] || \
    fail "output already exists; preserve it and choose a new ATTEMPT: ${output}"
done

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
  fail "Cartographer runtime map does not match the frozen mocap map"
[[ "${runtime_map_sha256,,}" == "${FROZEN_MAP_SHA256}" ]] || \
  fail "Cartographer runtime map SHA-256 does not match the frozen contract"

published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
if grep -Fxq -- /cmd_vel <<< "${published_topics}"; then
  fail "/cmd_vel already has a publisher; stop stale planner and teleop"
fi

mkdir -p "${RUN_OUT_DIR}"
{
  printf 'protocol_id=%s\n' 'SMPCC_mocap_b0_delay_mode_diagnostic_v2'
  printf 'run_class=%s\n' 'B0_DELAY_DIAGNOSTIC'
  printf 'formal_trial_consumed=%s\n' 'false'
  printf 'delay_test_id=%s\n' "${DELAY_TEST_ID}"
  printf 'attempt=%s\n' "${ATTEMPT}"
  printf 'variant=%s\n' 'B0'
  printf 'slosh_enabled=%s\n' 'false'
  printf 'w_slosh=%s\n' '0.0'
  printf 'v_ref=%s\n' "${V_REF}"
  printf 'delay_phase_mode=%s\n' "${DELAY_PHASE_MODE}"
  printf 'delay_phase_mode_code=%s\n' "${EXPECTED_MODE_CODE}"
  printf 'delay_phase_linear_delay_sec=%s\n' "${DELAY_PHASE_LINEAR_DELAY_SEC}"
  printf 'delay_phase_angular_delay_sec=%s\n' "${DELAY_PHASE_ANGULAR_DELAY_SEC}"
  printf 'speed_safety_enable=%s\n' "${SPEED_SAFETY_ENABLE}"
  printf 'v_safe_max=%s\n' "${V_SAFE_MAX}"
  printf 'speed_safety_tolerance=%s\n' "${SPEED_SAFETY_TOLERANCE}"
  printf 'path_file=%s\n' "${PATH_FILE}"
  printf 'path_sha256=%s\n' "${FROZEN_PATH_SHA256}"
  printf 'map_file=%s\n' "$(readlink -f "${FIELD_MAP_FILE}")"
  printf 'map_sha256=%s\n' "${FROZEN_MAP_SHA256}"
  printf 'mocap_host=%s\n' "${MOCAP_HOST}"
  printf 'mocap_tracker=%s\n' "${MOCAP_TRACKER}"
  printf 'run_label=%s\n' "${RUN_LABEL}"
  printf 'created_at=%s\n' "$(date --iso-8601=seconds)"
} > "${PROTOCOL_META}"

echo "================ mocap B0 delay diagnostic ================"
echo "  diagnostic    = ${DELAY_TEST_ID}/${ATTEMPT}; formal_trial_consumed=false"
echo "  mode/code     = ${DELAY_PHASE_MODE}/${EXPECTED_MODE_CODE}"
echo "  variant       = B0; slosh=false; w_slosh=0; v_ref=${V_REF}"
echo "  delay         = ${DELAY_PHASE_LINEAR_DELAY_SEC}/${DELAY_PHASE_ANGULAR_DELAY_SEC} s"
echo "  speed safety  = enabled; v_ref=${V_REF}; v_safe_max=${V_SAFE_MAX} m/s"
echo "  path          = ${PATH_FILE}"
echo "  output        = ${BAG_PATH}"
echo "============================================================="

DATE="${DATE}" STAMP="${STAMP}" PILOT_MODE=true PILOT_METHOD=B0 \
PILOT_RECORD_RGB=false PILOT_RECORD_ONLINE_LIQUID=false \
PILOT_CONDITION=SMPCC_mocap_b0_delay_mode_diagnostic_v2 \
RUN_LABEL="${RUN_LABEL}" NAME="${NAME}" RUN_OUT_DIR="${RUN_OUT_DIR}" \
PATH_SOURCE_MODE=replay PATH_FILE="${PATH_FILE}" \
PATH_EXPECTED_SHA256="${FROZEN_PATH_SHA256}" REQUIRE_PATH_HASH=true \
START_POS_TOL="${START_POS_TOL}" START_YAW_TOL="${START_YAW_TOL}" \
START_HOLD_SEC="${START_HOLD_SEC}" START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC}" \
V_REF="${V_REF}" SPEED_SAFETY_ENABLE="${SPEED_SAFETY_ENABLE}" \
V_SAFE_MAX="${V_SAFE_MAX}" SPEED_SAFETY_TOLERANCE="${SPEED_SAFETY_TOLERANCE}" \
W_SLOSH=0.0 W_SMOOTH=1.0 W_ALPHA=1.0 W_DU_A=1.0 W_DU_VS=1.0 \
DELAY_PHASE_MODE="${DELAY_PHASE_MODE}" \
DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
IMU_SHADOW_ENABLE=true IMU_TOPIC="${IMU_TOPIC}" IMU_SUBSCRIBER_QUEUE_SIZE=10 \
CURRENT_OBSERVER_SOURCE=processed_imu OBSERVER_FALLBACK_POLICY=fail_closed \
OBSERVER_LATCH_FALLBACK=false STATE_TIMING_REQUIRE_COMMON_EPOCH=true \
STATE_TIMING_MAX_RAW_SKEW_SEC=0.080 STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=0.050 \
STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC=0.010 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=false SHARED_ANGULAR_LIMIT_ENABLE=false \
EXECUTION_CONTRACT_FAIL_CLOSED=true EXECUTION_CONTRACT_MAX_DELTA_V=0.0001 \
EXECUTION_CONTRACT_MAX_DELTA_OMEGA=0.0001 \
RECORD_RGB=false RECORD_CAMERA=false RECORD_CAMERA_INFO=false \
RECORD_CAMERA_COMPRESSED=false RECORD_DEPTH=false RECORD_ONLINE_LIQUID=false \
RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false RECORD_STANDALONE_SLOSH=false \
RECORD_SCAN=true FORBID_IMAGE_STREAMS=true RECORD_ALL_EXISTING_TOPICS=false \
RECORD_TOPIC_INFO=true RECORD_MOCAP=true RECORD_MOCAP_PATH=false \
MOCAP_TRACKER="${MOCAP_TRACKER}" RECORD_SEC="${RECORD_SEC}" MAX_RECORD_SEC="${RECORD_SEC}" \
BLOCK_SEGMENT_ID="MOCAP_B0_DELAY_${DELAY_TEST_ID}_${DELAY_PHASE_MODE}" SPLIT_BLOCK=false \
ORDER_POSITION="${DELAY_TEST_ID#D}" ACQUISITION_RETRY=false SEND_ZERO_ON_EXIT=true \
OPERATOR_NOTE="B0 delay-mode diagnostic only; never consumes R01-R05" \
bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"
python3 "${POSTFLIGHT}" "${BAG_PATH}" \
  --variant B0 \
  --mocap-tracker "${MOCAP_TRACKER}" \
  --imu-topic "${IMU_TOPIC}" \
  --path-file "${PATH_FILE}" \
  --path-sha256 "${FROZEN_PATH_SHA256}" \
  --report "${POSTFLIGHT_REPORT}"

python3 "${SUMMARIZER}" "${BAG_PATH}" --out-dir "${RUN_OUT_DIR}"
[[ -s "${SUMMARY_JSON}" ]] || fail "summary missing after postflight: ${SUMMARY_JSON}"

python3 "${DELAY_GATE_VALIDATOR}" "${SUMMARY_JSON}" \
  --expected-mode "${DELAY_PHASE_MODE}" \
  --expected-code "${EXPECTED_MODE_CODE}" \
  --report "${DELAY_GATE_REPORT}"

echo "[${SCRIPT_NAME}] PASS: ${DELAY_PHASE_MODE} ${DELAY_TEST_ID}/${ATTEMPT}; R01--R05 remain unused"
