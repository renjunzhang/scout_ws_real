#!/usr/bin/env bash
# Development G3R3 pair: same 10D solver and weights, only w_slosh=0/5.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[SHORT-HORIZON-MATCHED][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
POSTFLIGHT="${SCRIPT_DIR}/../tools/analysis/validate_short_horizon_matched_bag.py"
PREFLIGHT_EXECUTABLE="${SPMPC_SHORT_HORIZON_MATCHED_PREFLIGHT:-${REPO_ROOT}/devel/lib/spmpc_local_planner/spmpc_short_horizon_matched_preflight}"
PATH_FILE="/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json"
PATH_SHA256="578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e"
RGB_CALIBRATION_FILE="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml"
RGB_CALIBRATION_SHA256="7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01"

MATCHED_ROW="${MATCHED_ROW:-}"
MATCHED_ATTEMPT="${MATCHED_ATTEMPT:-01}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
RECORD_SEC="${RECORD_SEC:-70}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_short_horizon_matched/H0}"

[[ "${MATCHED_ATTEMPT}" == "01" ]] || fail "only acquisition attempt 01 is preregistered"
case "${MATCHED_ROW}" in
  01) BLOCK=01; POSITION=01; CONDITION=Matched0; VARIANT=B_slosh_matched0; WEIGHT=0.0 ;;
  02) BLOCK=01; POSITION=02; CONDITION=Matched5; VARIANT=B_slosh_matched5; WEIGHT=5.0 ;;
  03) BLOCK=02; POSITION=01; CONDITION=Matched5; VARIANT=B_slosh_matched5; WEIGHT=5.0 ;;
  04) BLOCK=02; POSITION=02; CONDITION=Matched0; VARIANT=B_slosh_matched0; WEIGHT=0.0 ;;
  05) BLOCK=03; POSITION=01; CONDITION=Matched0; VARIANT=B_slosh_matched0; WEIGHT=0.0 ;;
  06) BLOCK=03; POSITION=02; CONDITION=Matched5; VARIANT=B_slosh_matched5; WEIGHT=5.0 ;;
  *) fail "set MATCHED_ROW=01..06; order is M0,M5,M5,M0,M0,M5" ;;
esac

for path in "${RUNNER}" "${POSTFLIGHT}" "${PATH_FILE}" "${RGB_CALIBRATION_FILE}"; do
  [[ -s "${path}" ]] || fail "missing required file: ${path}"
done
[[ "$(sha256sum "${PATH_FILE}" | awk '{print $1}')" == "${PATH_SHA256}" ]] || \
  fail "H0 path hash changed"
[[ "$(sha256sum "${RGB_CALIBRATION_FILE}" | awk '{print $1}')" == "${RGB_CALIBRATION_SHA256}" ]] || \
  fail "RGB calibration hash changed"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"
[[ -x "${PREFLIGHT_EXECUTABLE}" ]] || \
  fail "missing C++ preflight executable; build spmpc_short_horizon_matched_preflight first"

validate_launch_contract() {
  local dump expected
  dump="$(roslaunch --dump-params spmpc_local_planner spmpc_fixed_path.launch \
    planner_variant:="${VARIANT}" delay_phase_mode:=shadow \
    imu_shadow_enable:=true observer_source:=processed_imu \
    observer_fallback_policy:=fail_closed observer_latch_fallback:=false \
    shared_linear_accel_limit_enable:=false shared_angular_limit_enable:=false \
    state_timing_require_common_epoch:=true \
    execution_contract_fail_closed_on_post_limit_change:=true \
    w_slosh:="${WEIGHT}" w_smooth:=1.0 w_alpha:=1.0 \
    w_du_a:=1.0 w_du_vs:=1.0 v_ref:=0.20)"
  expected=(
    "/spmpc_local_planner/planner_variant: ${VARIANT}"
    "/spmpc_local_planner/delay_phase/mode: shadow"
    "/spmpc_local_planner/slosh_observer/source: processed_imu"
    "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
    "/spmpc_local_planner/state_timing/require_common_epoch: true"
    "/spmpc_local_planner/execution_contract/fail_closed_on_post_limit_change: true"
    "/spmpc_local_planner/platform/shared_constraints/linear_accel_limit_enable: false"
    "/spmpc_local_planner/platform/shared_constraints/angular_limit_enable: false"
    "/spmpc_local_planner/variants/${VARIANT}/slosh_enable: true"
    "/spmpc_local_planner/variants/${VARIANT}/v_ref: 0.2"
    "/spmpc_local_planner/variants/${VARIANT}/w_contour: 1.0"
    "/spmpc_local_planner/variants/${VARIANT}/w_lag: 0.2"
    "/spmpc_local_planner/variants/${VARIANT}/w_progress: 0.2"
    "/spmpc_local_planner/variants/${VARIANT}/w_v: 1.0"
    "/spmpc_local_planner/variants/${VARIANT}/w_vs: 0.3"
    "/spmpc_local_planner/variants/${VARIANT}/w_control: 0.3"
    "/spmpc_local_planner/variants/${VARIANT}/w_accel: 0.0"
    "/spmpc_local_planner/variants/${VARIANT}/w_smooth: 1.0"
    "/spmpc_local_planner/variants/${VARIANT}/w_alpha: 1.0"
    "/spmpc_local_planner/variants/${VARIANT}/w_du_a: 1.0"
    "/spmpc_local_planner/variants/${VARIANT}/w_du_vs: 1.0"
    "/spmpc_local_planner/variants/${VARIANT}/w_slosh: ${WEIGHT}"
    "/spmpc_local_planner/variants/${VARIANT}/slosh_cost_horizon_steps: 3"
    "/spmpc_local_planner/variants/${VARIANT}/slosh_cost_tail_discount: 0.0"
  )
  for expected in "${expected[@]}"; do
    grep -Fqx -- "${expected}" <<< "${dump}" || fail "launch contract missing: ${expected}"
  done
  printf '%s\n' "${dump}" | "${PREFLIGHT_EXECUTABLE}" || \
    fail "C++ matched-pair preflight rejected the expanded launch contract"
}

validate_launch_contract
if truthy "${VALIDATE_ONLY}"; then
  echo "[SHORT-HORIZON-MATCHED] validate-only PASS: row=${MATCHED_ROW} ${VARIANT}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || fail "set ARM_MOTION=YES only after the robot is at the H0 start and E-stop is ready"
timeout 5s rostopic echo -n 1 /liquid/measurement >/dev/null 2>&1 || \
  fail "online /liquid/measurement is not active"

RUN_LABEL="DEV_G3R3_H0_C1_${CONDITION}_b${BLOCK}_p${POSITION}_r${MATCHED_ROW}_a${MATCHED_ATTEMPT}"
BAG_PATH="${RUN_OUT_DIR}/${RUN_LABEL}.bag"

echo "================ short-horizon matched trial ================"
echo "  row/block/pos = ${MATCHED_ROW}/${BLOCK}/${POSITION}"
echo "  condition     = ${CONDITION}; ${VARIANT}; w_slosh=${WEIGHT}"
echo "  common cost   = w_control=0.3; smooth split=1.0"
echo "  liquid window = state stages 0..3 (0..100 ms); tail=0"
echo "  state source  = processed_imu; robot interpolated to same epoch"
echo "  delay mode    = shadow only; no command-history state injection"
echo "  output        = ${BAG_PATH}"
echo "==============================================================="

DATE="${DATE}" STAMP="${STAMP}" PILOT_MODE=true PILOT_METHOD="" \
VARIANT="${VARIANT}" PILOT_CONDITION=G3R3_short_horizon_matched \
PILOT_RECORD_RGB=false PILOT_RECORD_ONLINE_LIQUID=true \
RUN_LABEL="${RUN_LABEL}" NAME="${RUN_LABEL}" RUN_OUT_DIR="${RUN_OUT_DIR}" \
PATH_SOURCE_MODE=replay PATH_FILE="${PATH_FILE}" PATH_EXPECTED_SHA256="${PATH_SHA256}" \
REQUIRE_PATH_HASH=true START_POS_TOL=0.08 START_YAW_TOL=0.15 START_HOLD_SEC=0.5 \
START_GATE_TIMEOUT_SEC=5 V_REF=0.20 W_SLOSH="${WEIGHT}" W_SMOOTH=1.0 \
W_ALPHA=1.0 W_DU_A=1.0 W_DU_VS=1.0 \
DELAY_PHASE_MODE=shadow IMU_SHADOW_ENABLE=true IMU_SUBSCRIBER_QUEUE_SIZE=10 \
CURRENT_OBSERVER_SOURCE=processed_imu OBSERVER_FALLBACK_POLICY=fail_closed \
OBSERVER_LATCH_FALLBACK=false STATE_TIMING_REQUIRE_COMMON_EPOCH=true \
STATE_TIMING_MAX_RAW_SKEW_SEC=0.080 STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=0.050 \
STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC=0.010 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=false SHARED_ANGULAR_LIMIT_ENABLE=false \
EXECUTION_CONTRACT_FAIL_CLOSED=true EXECUTION_CONTRACT_MAX_DELTA_V=0.0001 \
EXECUTION_CONTRACT_MAX_DELTA_OMEGA=0.0001 \
RECORD_RGB=false RECORD_CAMERA=false RECORD_CAMERA_INFO=true \
RECORD_CAMERA_COMPRESSED=false RECORD_DEPTH=false RECORD_ONLINE_LIQUID=true \
RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false FORBID_IMAGE_STREAMS=true \
RECORD_ALL_EXISTING_TOPICS=false RGB_CALIBRATION_FILE="${RGB_CALIBRATION_FILE}" \
RGB_CALIBRATION_EXPECTED_SHA256="${RGB_CALIBRATION_SHA256}" \
ONLINE_LIQUID_MEASUREMENT_TOPIC=/liquid/measurement \
ONLINE_LIQUID_PROTOCOL=G3R3_short_horizon_matched_v1 \
ONLINE_LIQUID_CALIBRATION_SHA256="${RGB_CALIBRATION_SHA256}" \
RECORD_SEC="${RECORD_SEC}" MAX_RECORD_SEC="${RECORD_SEC}" \
BLOCK_SEGMENT_ID="G3R3_b${BLOCK}" SPLIT_BLOCK=false ORDER_POSITION="${POSITION}" \
ACQUISITION_RETRY=false SEND_ZERO_ON_EXIT=true \
bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"
python3 "${POSTFLIGHT}" "${BAG_PATH}" --variant "${VARIANT}"
