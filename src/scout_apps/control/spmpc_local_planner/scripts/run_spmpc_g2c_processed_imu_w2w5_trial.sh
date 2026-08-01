#!/usr/bin/env bash
# One physical processed-IMU G2C W2/W5 development trial.
# Execute rows separately; return/alignment and liquid settling are mandatory.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G2C][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
source /opt/ros/noetic/setup.bash
source "${REPO_ROOT}/devel/setup.bash"

RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
VALIDATOR="${SCRIPT_DIR}/analysis/validate_g2c_processed_imu_trial.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"

G2C_ROW="${G2C_ROW:-}"
G2C_ATTEMPT="${G2C_ATTEMPT:-01}"
DATE="${DATE:-20260801}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
ALLOW_DIRTY_VALIDATE_ONLY="${ALLOW_DIRTY_VALIDATE_ONLY:-false}"

PATH_FILE="/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json"
PATH_SHA256="578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e"
SOURCE_REPORT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/analysis/G2S_RAW_RGB_THREE_TRIAL_DEVELOPMENT_REPORT.json"
SOURCE_REPORT_SHA256="36db4d12948f98e4ef3580bc52367cdef14d8440d80a8afbb32182b704ee5442"
SMOKE_REPORT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/analysis/PROCESSED_IMU_G2C_DEVELOPMENT_SMOKE_REPORT.json"
SMOKE_REPORT_SHA256="5635d31e0221bdc54a00ee9eb11043515112a3326eb7945d10d0ffb5e13cc5d3"
PLANNER_SMOKE_BASE_REVISION="01700fd1672c1a421b0a789b6e85077b1eb48fa4"

RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g2c_processed_imu_w2w5/H0}"
PREREG_FILE="${RUN_OUT_DIR}/G2C_processed_imu_W2W5_prereg.env"

case "${G2C_ROW}" in
  01) METHOD=W2; WEIGHT=2; BLOCK=01 ;;
  02) METHOD=W5; WEIGHT=5; BLOCK=01 ;;
  03) METHOD=W5; WEIGHT=5; BLOCK=02 ;;
  04) METHOD=W2; WEIGHT=2; BLOCK=02 ;;
  *) fail "set G2C_ROW=01|02|03|04; frozen order is W2,W5,W5,W2" ;;
esac
[[ "${G2C_ATTEMPT}" == "01" ]] || \
  fail "this field wrapper currently permits only first attempt G2C_ATTEMPT=01"
[[ -s "${RUNNER}" && -s "${VALIDATOR}" && -s "${SUMMARIZER}" ]] || \
  fail "missing G2C runner/validator/summarizer"
[[ -s "${PATH_FILE}" ]] || fail "missing frozen path: ${PATH_FILE}"
[[ "$(sha256sum "${PATH_FILE}" | awk '{print $1}')" == "${PATH_SHA256}" ]] || \
  fail "frozen path hash mismatch"
[[ -s "${SOURCE_REPORT}" ]] || fail "missing raw-RGB three-trial G2S source report"
[[ "$(sha256sum "${SOURCE_REPORT}" | awk '{print $1}')" == "${SOURCE_REPORT_SHA256}" ]] || \
  fail "G2S source report hash mismatch"
[[ -s "${SMOKE_REPORT}" ]] || fail "missing processed-IMU smoke report"
[[ "$(sha256sum "${SMOKE_REPORT}" | awk '{print $1}')" == "${SMOKE_REPORT_SHA256}" ]] || \
  fail "processed-IMU smoke report hash mismatch"

release_revision="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
if ! git -C "${REPO_ROOT}" diff --quiet || \
   ! git -C "${REPO_ROOT}" diff --cached --quiet; then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G2C][WARN] allowing dirty tracked worktree for developer validate-only" >&2
  else
    fail "tracked worktree differs from ${release_revision}; commit/freeze before G2C"
  fi
fi

smoke_frozen_paths=(
  src/scout_apps/control/spmpc_local_planner/CMakeLists.txt
  src/scout_apps/control/spmpc_local_planner/config
  src/scout_apps/control/spmpc_local_planner/include
  src/scout_apps/control/spmpc_local_planner/launch
  src/scout_apps/control/spmpc_local_planner/msg
  src/scout_apps/control/spmpc_local_planner/src
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/summarize_spmpc_real_trial.py
)
git -C "${REPO_ROOT}" diff --quiet \
  "${PLANNER_SMOKE_BASE_REVISION}..${release_revision}" -- \
  "${smoke_frozen_paths[@]}" || \
  fail "planner runtime differs from processed-IMU smoke base ${PLANNER_SMOKE_BASE_REVISION}"

python3 - "${SOURCE_REPORT}" "${SMOKE_REPORT}" <<'PY'
import json
import sys

source_path, smoke_path = sys.argv[1:]
with open(source_path, encoding="utf-8") as stream:
    source = json.load(stream)
with open(smoke_path, encoding="utf-8") as stream:
    smoke = json.load(stream)
if source.get("decision") != "processed_imu":
    raise SystemExit("source report does not select processed_imu")
if source.get("decision_scope") != "G2C_DEVELOPMENT_ONLY":
    raise SystemExit("source report scope is not G2C_DEVELOPMENT_ONLY")
if source.get("status") != "PASS_FOR_G2C_DEVELOPMENT":
    raise SystemExit("raw-RGB three-trial source report is not PASS_FOR_G2C_DEVELOPMENT")
if source.get("formal_gate_status") != "NOT_RUN_THREE_OF_FOUR_AND_NO_FORMAL_POSTFLIGHT":
    raise SystemExit("source report formal-gate boundary mismatch")
if source.get("operator_stop_after_completed_units") != 3:
    raise SystemExit("source report does not bind the operator-approved three-unit stop")
if not source.get("bag_hashes_complete"):
    raise SystemExit("source report does not contain all three source-bag hashes")
aggregate = source.get("aggregate", {})
if aggregate.get("processed_imu_relative_improvement", 0.0) < 0.10:
    raise SystemExit("source report improvement is below the frozen 10% gate")
if aggregate.get("directional_trial_count") != 3:
    raise SystemExit("source report is not directionally consistent in all three trials")
if not aggregate.get("coverage_pass") or not aggregate.get("not_single_trial_dominated"):
    raise SystemExit("source report coverage/dominance gate failed")
if smoke.get("status") != "PASS_FOR_G2C_DEVELOPMENT":
    raise SystemExit("processed-IMU smoke is not PASS_FOR_G2C_DEVELOPMENT")
if smoke.get("nominal_source") != "processed_imu" or smoke.get("effective_source") != "processed_imu":
    raise SystemExit("processed-IMU smoke source mismatch")
if smoke.get("fallback_active") or smoke.get("cmd_vel_messages_observed") != 0:
    raise SystemExit("processed-IMU smoke fallback/no-command invariant failed")
PY

RUN_LABEL="DEV_G2C_H0_C1_${METHOD}_b${BLOCK}_p${G2C_ROW}_a${G2C_ATTEMPT}"
BAG_PATH="${RUN_OUT_DIR}/${RUN_LABEL}.bag"

prereg_contents() {
  printf '%s\n' \
    "protocol=G2C_processed_imu_W2W5_development_v1" \
    "scope=development_only" \
    "planned_units=4" \
    "planned_order=01:W2,02:W5,03:W5,04:W2" \
    "paired_blocks=2" \
    "candidate_weights=2,5" \
    "nominal_observer_source=processed_imu" \
    "fallback_policy=odom_latched" \
    "fallback_during_motion=trial_failure" \
    "source_report=${SOURCE_REPORT}" \
    "source_report_sha256=${SOURCE_REPORT_SHA256}" \
    "source_report_gate_status=THREE_OF_FOUR_DEVELOPMENT_ONLY" \
    "source_decision_scope=G2C_DEVELOPMENT_ONLY" \
    "processed_imu_smoke_report=${SMOKE_REPORT}" \
    "processed_imu_smoke_report_sha256=${SMOKE_REPORT_SHA256}" \
    "planner_smoke_base_revision=${PLANNER_SMOKE_BASE_REVISION}" \
    "release_revision=${release_revision}" \
    "path_file=${PATH_FILE}" \
    "path_sha256=${PATH_SHA256}" \
    "v_ref=0.20" \
    "delay_phase=fixed_closed_loop:0.15,0.22" \
    "record_rgb=false" \
    "record_online_liquid=false" \
    "forbid_image_streams=true" \
    "record_sec=90" \
    "no_early_stop=true"
}

runner_env=(
  "DATE=${DATE}"
  "PILOT_MODE=true"
  "PILOT_METHOD=${METHOD}"
  "PILOT_CONDITION=G2C_processed_imu_W2W5"
  "PILOT_RECORD_RGB=false"
  "PILOT_RECORD_ONLINE_LIQUID=false"
  "RUN_LABEL=${RUN_LABEL}"
  "NAME=${RUN_LABEL}"
  "RUN_OUT_DIR=${RUN_OUT_DIR}"
  "PATH_SOURCE_MODE=replay"
  "PATH_FILE=${PATH_FILE}"
  "PATH_EXPECTED_SHA256=${PATH_SHA256}"
  "REQUIRE_PATH_HASH=true"
  "START_POS_TOL=0.08"
  "START_YAW_TOL=0.15"
  "START_HOLD_SEC=0.5"
  "V_REF=0.20"
  "DELAY_PHASE_MODE=fixed_closed_loop"
  "DELAY_PHASE_LINEAR_DELAY_SEC=0.15"
  "DELAY_PHASE_ANGULAR_DELAY_SEC=0.22"
  "IMU_SHADOW_ENABLE=true"
  "CURRENT_OBSERVER_SOURCE=processed_imu"
  "OBSERVER_FALLBACK_POLICY=odom"
  "OBSERVER_LATCH_FALLBACK=true"
  "RECORD_RGB=false"
  "RECORD_CAMERA=false"
  "RECORD_CAMERA_INFO=true"
  "RECORD_CAMERA_COMPRESSED=false"
  "RECORD_DEPTH=false"
  "RECORD_ONLINE_LIQUID=false"
  "RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false"
  "FORBID_IMAGE_STREAMS=true"
  "RECORD_ALL_EXISTING_TOPICS=false"
  "RGB_CALIBRATION_FILE="
  "LIQUID_CALIBRATION="
  "SEND_ZERO_ON_EXIT=true"
  "RECORD_SEC=90"
  "MAX_RECORD_SEC=90"
  "BLOCK_SEGMENT_ID=G2C_b${BLOCK}_seg01"
  "ORDER_POSITION=${G2C_ROW}"
  "OPERATOR_NOTE=G2C_development_processed_imu_${METHOD}_source_report_${SOURCE_REPORT_SHA256}"
)

echo "================ G2C processed-IMU unit ================"
echo "  row/method   = ${G2C_ROW}/${METHOD}"
echo "  block/order  = ${BLOCK}/${G2C_ROW}"
echo "  source       = processed_imu (fallback during motion fails postflight)"
echo "  source proof = raw-RGB 3-trial development (${SOURCE_REPORT_SHA256})"
echo "  release      = ${release_revision} (planner smoke base ${PLANNER_SMOKE_BASE_REVISION})"
echo "  path         = ${PATH_FILE}"
echo "  output       = ${BAG_PATH}"
echo "  RGB          = not recorded and not used in G2C"
echo "  scope        = development only; formal remains NO-GO"
echo "========================================================="

if truthy "${VALIDATE_ONLY}"; then
  printf '[G2C] validate-only command:\n  env '
  printf '%q ' "${runner_env[@]}"
  printf 'bash %q\n' "${RUNNER}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "real motion is disarmed; rerun with ARM_MOTION=YES after clearing the path"
[[ ! -e "${BAG_PATH}" && ! -e "${BAG_PATH}.active" ]] || \
  fail "attempt output already exists: ${BAG_PATH}"

if [[ "${G2C_ROW}" != "01" ]]; then
  previous_row=$(printf '%02d' "$((10#${G2C_ROW} - 1))")
  case "${previous_row}" in
    01) previous_label="DEV_G2C_H0_C1_W2_b01_p01_a01" ;;
    02) previous_label="DEV_G2C_H0_C1_W5_b01_p02_a01" ;;
    03) previous_label="DEV_G2C_H0_C1_W5_b02_p03_a01" ;;
  esac
  previous_report="${RUN_OUT_DIR}/${previous_label}_g2c_postflight.json"
  [[ -s "${previous_report}" ]] || fail "previous row postflight is missing: ${previous_report}"
  python3 - "${previous_report}" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("status") != "PASS":
    raise SystemExit("previous G2C row postflight is not PASS")
PY
fi

mkdir -p "${RUN_OUT_DIR}"
expected_prereg="$(prereg_contents)"
if [[ -e "${PREREG_FILE}" ]]; then
  [[ "$(<"${PREREG_FILE}")" == "${expected_prereg}" ]] || \
    fail "existing G2C prereg differs: ${PREREG_FILE}"
else
  [[ "${G2C_ROW}" == "01" ]] || fail "row 01 must create the G2C prereg"
  printf '%s\n' "${expected_prereg}" > "${PREREG_FILE}"
  sha256sum "${PREREG_FILE}" > "${PREREG_FILE}.sha256"
fi

printf '%s\n' \
  "source_report=${SOURCE_REPORT}" \
  "source_report_sha256=${SOURCE_REPORT_SHA256}" \
  "processed_imu_smoke_report=${SMOKE_REPORT}" \
  "processed_imu_smoke_report_sha256=${SMOKE_REPORT_SHA256}" \
  "g2c_row=${G2C_ROW}" \
  "method=${METHOD}" \
  "weight=${WEIGHT}" \
  > "${RUN_OUT_DIR}/${RUN_LABEL}_g2c_binding.env"

env "${runner_env[@]}" bash "${RUNNER}"

python3 "${VALIDATOR}" \
  --bag "${BAG_PATH}" \
  --expected-weight "${WEIGHT}" \
  --expected-v-ref 0.20 \
  --hash-bag

python3 "${SUMMARIZER}" "${BAG_PATH}"

echo "[G2C] unit PASS: ${BAG_PATH}"
echo "[G2C] Return to the start mark, align, and let the liquid settle before the next row."
