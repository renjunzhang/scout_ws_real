#!/usr/bin/env bash
# Two-row G5 minimal comparator confirmation: SmoothMatch then FixedProfile.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G5][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
FIXED_RUNNER="${SCRIPT_DIR}/run_fixed_profile_real_trial.sh"
VALIDATOR="${SCRIPT_DIR}/../tools/analysis/validate_g5_minimal_trial.py"
ANALYZER="${SCRIPT_DIR}/../tools/analysis/analyze_g5_minimal.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"

DATE="${DATE:-20260801}"
G5_ROW="${G5_ROW:-}"
ARM_MOTION="${ARM_MOTION:-NO}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g5_minimal/H0}"
PATH_FILE="${PATH_FILE:-/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json}"
PREREG="${RUN_OUT_DIR}/G5_COMPARATOR_PREREG_REPORT.json"
SMOOTH_CONFIG="${RUN_OUT_DIR}/G5_SMOOTH_MATCH_CONFIG.yaml"
FIXED_CONFIG="${RUN_OUT_DIR}/G5_FIXED_PROFILE_CONFIG.yaml"

for artifact in "${RUNNER}" "${FIXED_RUNNER}" "${VALIDATOR}" "${ANALYZER}" \
  "${SUMMARIZER}" "${PATH_FILE}" "${PREREG}" "${SMOOTH_CONFIG}" "${FIXED_CONFIG}"; do
  [[ -s "${artifact}" ]] || fail "missing artifact: ${artifact}; run prepare_spmpc_g5_comparators.sh first"
done

readarray -t frozen < <(python3 - "${PREREG}" "${SMOOTH_CONFIG}" "${FIXED_CONFIG}" <<'PY'
import json
import sys
import yaml
prereg = json.load(open(sys.argv[1], encoding="utf-8"))
smooth = yaml.safe_load(open(sys.argv[2], encoding="utf-8"))
fixed = yaml.safe_load(open(sys.argv[3], encoding="utf-8"))
if prereg.get("status") != "READY_FOR_REAL_CONFIRMATION":
    raise SystemExit("G5 prereg is not READY_FOR_REAL_CONFIRMATION")
print(smooth["v_ref_m_s"])
print(fixed["profile_csv"])
print(fixed["profile_sha256"])
PY
)
(( ${#frozen[@]} == 3 )) || fail "failed to load G5 frozen settings"
SMOOTH_V_REF="${frozen[0]}"
PROFILE_FILE="${frozen[1]}"
PROFILE_SHA256="${frozen[2]}"
PATH_SHA256="$(sha256sum "${PATH_FILE}" | awk '{print $1}')"
PREREG_SHA256="$(sha256sum "${PREREG}" | awk '{print $1}')"
SMOOTH_CONFIG_SHA256="$(sha256sum "${SMOOTH_CONFIG}" | awk '{print $1}')"
FIXED_CONFIG_SHA256="$(sha256sum "${FIXED_CONFIG}" | awk '{print $1}')"

case "${G5_ROW}" in
  01)
    CONDITION=SmoothMatch
    RUN_LABEL=DEV_G5_H0_C1_SmoothMatch_b01_p01_a01
    CONFIG_FILE="${SMOOTH_CONFIG}"
    CONFIG_SHA256="${SMOOTH_CONFIG_SHA256}"
    ;;
  02)
    CONDITION=FixedProfile
    RUN_LABEL=DEV_G5_H0_C1_FixedProfile_b01_p02_a01
    CONFIG_FILE="${FIXED_CONFIG}"
    CONFIG_SHA256="${FIXED_CONFIG_SHA256}"
    previous="${RUN_OUT_DIR}/DEV_G5_H0_C1_SmoothMatch_b01_p01_a01_g5_postflight.json"
    [[ -s "${previous}" ]] || fail "Row 01 postflight is missing"
    python3 - "${previous}" <<'PY'
import json
import sys
if json.load(open(sys.argv[1], encoding="utf-8")).get("status") != "PASS":
    raise SystemExit("G5 Row 01 postflight is not PASS")
PY
    ;;
  *) fail "set G5_ROW=01 or G5_ROW=02" ;;
esac

bag_path="${RUN_OUT_DIR}/${RUN_LABEL}.bag"
postflight="${RUN_OUT_DIR}/${RUN_LABEL}_g5_postflight.json"
[[ ! -e "${bag_path}" && ! -e "${bag_path}.active" ]] || fail "output already exists: ${bag_path}"

echo "================ G5 minimal unit ================"
echo "  row/condition = ${G5_ROW}/${CONDITION}"
echo "  path          = ${PATH_FILE}"
echo "  output        = ${bag_path}"
echo "  images        = forbidden"
echo "  prereg SHA    = ${PREREG_SHA256}"
echo "================================================="

if [[ "${CONDITION}" == "SmoothMatch" ]]; then
  command=(
    env
    "DATE=${DATE}"
    PILOT_MODE=true
    PILOT_METHOD=Bsmooth
    PILOT_CONDITION=G5_comparator_fairness_minimal_v1
    PILOT_RECORD_RGB=false
    PILOT_RECORD_ONLINE_LIQUID=false
    "RUN_LABEL=${RUN_LABEL}"
    "NAME=${RUN_LABEL}"
    "RUN_OUT_DIR=${RUN_OUT_DIR}"
    PATH_SOURCE_MODE=replay
    "PATH_FILE=${PATH_FILE}"
    "PATH_EXPECTED_SHA256=${PATH_SHA256}"
    REQUIRE_PATH_HASH=true
    START_POS_TOL=0.08
    START_YAW_TOL=0.15
    START_HOLD_SEC=0.5
    START_GATE_TIMEOUT_SEC=10
    "V_REF=${SMOOTH_V_REF}"
    W_SLOSH=0.0
    DELAY_PHASE_MODE=fixed_closed_loop
    DELAY_PHASE_LINEAR_DELAY_SEC=0.15
    DELAY_PHASE_ANGULAR_DELAY_SEC=0.22
    IMU_SHADOW_ENABLE=true
    IMU_SHADOW_READY_TIMEOUT_SEC=12
    CURRENT_OBSERVER_SOURCE=processed_imu
    OBSERVER_FALLBACK_POLICY=odom
    OBSERVER_LATCH_FALLBACK=true
    RECORD_RGB=false
    RECORD_CAMERA=false
    RECORD_CAMERA_INFO=false
    RECORD_CAMERA_COMPRESSED=false
    RECORD_DEPTH=false
    RECORD_SCAN=false
    RECORD_ONLINE_LIQUID=false
    RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false
    FORBID_IMAGE_STREAMS=true
    RECORD_ALL_EXISTING_TOPICS=false
    RECORD_ROSOUT=false
    RECORD_SEC=70
    MAX_RECORD_SEC=70
    BLOCK_SEGMENT_ID=G5_b01_seg01
    ORDER_POSITION=01
    SEND_ZERO_ON_EXIT=true
    "OPERATOR_NOTE=G5_SmoothMatch_vref_${SMOOTH_V_REF}_prereg_${PREREG_SHA256}"
    bash "${RUNNER}"
  )
else
  command=(
    env
    "DATE=${DATE}"
    "RUN_LABEL=${RUN_LABEL}"
    "NAME=${RUN_LABEL}"
    "RUN_OUT_DIR=${RUN_OUT_DIR}"
    "PATH_FILE=${PATH_FILE}"
    "PATH_EXPECTED_SHA256=${PATH_SHA256}"
    "PROFILE_FILE=${PROFILE_FILE}"
    "PROFILE_EXPECTED_SHA256=${PROFILE_SHA256}"
    "PROFILE_CONFIG=${FIXED_CONFIG}"
    "PROFILE_CONFIG_EXPECTED_SHA256=${FIXED_CONFIG_SHA256}"
    "G5_PREREG_SHA256=${PREREG_SHA256}"
    RECORD_SEC=70
    bash "${FIXED_RUNNER}"
  )
fi

if truthy "${VALIDATE_ONLY}"; then
  printf '[G5] validate-only command:\n  '
  printf '%q ' "${command[@]}"
  printf '\n'
  exit 0
fi
[[ "${ARM_MOTION}" == "YES" ]] || fail "set ARM_MOTION=YES after clearing the area"

ARM_MOTION=YES "${command[@]}"

validator_args=(
  --bag "${bag_path}"
  --condition "${CONDITION}"
  --row "${G5_ROW}"
  --path-file "${PATH_FILE}"
  --path-sha256 "${PATH_SHA256}"
  --config-file "${CONFIG_FILE}"
  --config-sha256 "${CONFIG_SHA256}"
  --g5-prereg-sha256 "${PREREG_SHA256}"
  --out-json "${postflight}"
)
if [[ "${CONDITION}" == "SmoothMatch" ]]; then
  validator_args+=(--expected-v-ref "${SMOOTH_V_REF}")
else
  validator_args+=(--profile-sha256 "${PROFILE_SHA256}")
fi
python3 "${VALIDATOR}" "${validator_args[@]}"

if [[ "${CONDITION}" == "SmoothMatch" ]]; then
  python3 "${SUMMARIZER}" "${bag_path}"
fi
if [[ "${G5_ROW}" == "02" ]]; then
  python3 "${ANALYZER}" --root "${RUN_OUT_DIR}"
fi
echo "[G5] Row ${G5_ROW} PASS: ${bag_path}"
