#!/usr/bin/env bash
# Shared engine for the frozen legacy-v1 and short100-v2 I0/fixed ABBA profiles.
#
# This is a new 0.20/0.25 m/s development profile.  It reuses the frozen C02
# path/map/RGB artifacts, but it is NOT the released C02 0.10/0.15 profile and
# does not inherit C02's safety/evidence status.  Row 01 (B0) is the motion gate
# for this new profile.

set -euo pipefail

SCRIPT_NAME=run_spmpc_i0_failclosed_fixed_abba_engine
ENGINE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(cd "${ENGINE_DIR}/.." && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
PROFILE_TOOL="${SCRIPT_DIR}/analysis/i0_failclosed_fixed_abba_profile.py"
EXACT_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_i0_failclosed_fixed_abba_bag.py"
WINDOW_CONTRACT="${SCRIPT_DIR}/analysis/liquid_cost_window_contract.py"
OBSERVER_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_slosh_nowcast_shadow_bag.py"
RGB_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_g3_online_rgb_trial.py"
RGB_ANALYZER="${SCRIPT_DIR}/analysis/analyze_i0_failclosed_fixed_abba_rgb.py"
CHAIN_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_mocap_execution_chain_bag.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"
MAP_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_field_map.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"
CONTRACT_TEST="${SCRIPT_DIR}/tests/test_i0_failclosed_fixed_abba_contract.py"
SHORT_HORIZON_TEST="${SCRIPT_DIR}/tests/test_short_horizon_matched_release.py"
RUNTIME_GATE_TEST="${SCRIPT_DIR}/tests/test_i0_failclosed_fixed_short100_runtime_gate.py"
RGB_ANALYZER_TEST="${SCRIPT_DIR}/tests/test_i0_failclosed_fixed_abba_rgb_analysis.py"
ONLINE_LIQUID_LAUNCH="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/launch/online_liquid_height.launch"
ONLINE_LIQUID_NODE="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py"
ONLINE_LIQUID_DETECTOR="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py"
ONLINE_LIQUID_MSG="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg"

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

require_rgb_analysis_decision() {
  local expected_phase="$1"
  local expected_status="$2"
  local expected_decision="$3"
  local expected_rows="$4"
  [[ -s "${RGB_ANALYSIS_REPORT}" ]] || fail "RGB analysis report missing: ${RGB_ANALYSIS_REPORT}"
  if ! python3 - "${RGB_ANALYSIS_REPORT}" "${RUN_OUT_DIR}" "${PROTOCOL_ID}" \
      "${RGB_ANALYSIS_REPORT_TYPE}" "${PROFILE_ID}" "${RGB_REPORT_SUFFIX}" \
      "${expected_phase}" "${expected_status}" "${expected_decision}" "${expected_rows}" <<'PY'
import json
import sys
from pathlib import Path

(
    report_path,
    root,
    protocol,
    report_type,
    profile_id,
    postflight_suffix,
    phase,
    status,
    decisions_csv,
    rows_csv,
) = sys.argv[1:]
try:
    report = json.loads(Path(report_path).read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as exc:
    raise SystemExit("could not read RGB analysis JSON: {}".format(exc))

expected_rows = set(rows_csv.split(","))
expected_decisions = set(decisions_csv.split(","))
checks = {
    "report_type": report.get("report_type") == report_type,
    "profile_id": report.get("profile_id") == profile_id,
    "protocol": report.get("protocol") == protocol,
    "scope": report.get("scope") == "DEVELOPMENT_ONLY",
    "root": Path(str(report.get("root", ""))).resolve() == Path(root).resolve(),
    "phase": report.get("phase") == phase,
    "status": report.get("status") == status,
    "decision": report.get("decision") in expected_decisions,
    "rows": set(report.get("rows", {})) == expected_rows,
    "postflights": set(report.get("postflights", {})) == expected_rows,
    "postflight_suffix": (
        report.get("input_contract", {}).get("postflight_suffix")
        == postflight_suffix
    ),
    "failures": report.get("failures") == [],
}
failed = [name for name, passed in checks.items() if not passed]
if failed:
    raise SystemExit(
        "RGB analysis contract mismatch ({}): phase={!r} status={!r} decision={!r}; expected={!r}".format(
            ",".join(failed),
            report.get("phase"),
            report.get("status"),
            report.get("decision"),
            sorted(expected_decisions),
        )
    )
PY
  then
    fail "RGB analysis decision contract failed"
  fi
}

run_rgb_analysis() {
  local expected_phase="$1"
  local positive_decision="$2"
  local negative_decision="$3"
  local expected_rows="$4"
  local analysis_rc=0
  python3 "${RGB_ANALYZER}" --root "${RUN_OUT_DIR}" \
    --report "${RGB_ANALYSIS_REPORT}" --protocol "${PROTOCOL_ID}" \
    --profile-id "${PROFILE_ID}" --postflight-suffix "${RGB_REPORT_SUFFIX}" \
    --unit-pass-suffix "${UNIT_PASS_SUFFIX}" \
    --report-type "${RGB_ANALYSIS_REPORT_TYPE}" \
    --minimum-p95-improvement-mm "${MINIMUM_P95_IMPROVEMENT_MM}" \
    --minimum-rms-improvement-mm "${MINIMUM_RMS_IMPROVEMENT_MM}" \
    --maximum-slowdown-ratio "${MAXIMUM_SLOWDOWN_RATIO}" || analysis_rc=$?
  case "${analysis_rc}" in
    0)
      require_rgb_analysis_decision \
        "${expected_phase}" PASS "${positive_decision}" "${expected_rows}"
      echo "[${SCRIPT_NAME}] RGB decision=${positive_decision}; analysis=${RGB_ANALYSIS_REPORT}"
      ;;
    10)
      require_rgb_analysis_decision \
        "${expected_phase}" STOP "${negative_decision}" "${expected_rows}"
      echo "[${SCRIPT_NAME}] STOP: exact RGB decision is recorded in ${RGB_ANALYSIS_REPORT}" >&2
      return 10
      ;;
    *)
      fail "RGB analysis invalid/incomplete (rc=${analysis_rc}): ${RGB_ANALYSIS_REPORT}"
      ;;
  esac
}

PAIR_ROW="${PAIR_ROW:-}"
ATTEMPT="${ATTEMPT:-01}"
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"
CONFIRM_NEW_SPEED_PROFILE="${CONFIRM_NEW_SPEED_PROFILE:-NO}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"

ABBA_PROFILE="${I0FC_ABBA_PROFILE:-}"
[[ -n "${ABBA_PROFILE}" ]] || fail "profile wrapper did not set I0FC_ABBA_PROFILE"
[[ -s "${PROFILE_TOOL}" ]] || fail "missing ABBA profile module: ${PROFILE_TOOL}"
profile_assignments="$(python3 "${PROFILE_TOOL}" \
  --profile "${ABBA_PROFILE}" --row "${PAIR_ROW}" --format shell)" \
  || fail "could not resolve profile=${ABBA_PROFILE} row=${PAIR_ROW:-unset}"
eval "${profile_assignments}"

PROFILE_ID="${I0FC_PROFILE_ID}"
PROTOCOL_ID="${I0FC_PROTOCOL_ID}"
VERSION_LABEL="${I0FC_VERSION_LABEL}"
OUTPUT_TAG="${I0FC_OUTPUT_TAG}"
RUN_LABEL_PREFIX="${I0FC_RUN_LABEL_PREFIX}"
RUNNER_SELECTOR_MODE="${I0FC_RUNNER_SELECTOR_MODE}"
TREATMENT_VARIANT="${I0FC_TREATMENT_VARIANT}"
TREATMENT_COST_HORIZON_STEPS="${I0FC_TREATMENT_COST_HORIZON_STEPS}"
TREATMENT_COST_TAIL_DISCOUNT="${I0FC_TREATMENT_COST_TAIL_DISCOUNT}"
EXACT_REPORT_SUFFIX="${I0FC_EXACT_REPORT_SUFFIX}"
OBSERVER_REPORT_SUFFIX="${I0FC_OBSERVER_REPORT_SUFFIX}"
CHAIN_REPORT_SUFFIX="${I0FC_CHAIN_REPORT_SUFFIX}"
RGB_REPORT_SUFFIX="${I0FC_RGB_REPORT_SUFFIX}"
UNIT_PASS_SUFFIX="${I0FC_UNIT_PASS_SUFFIX}"
RGB_ANALYSIS_REPORT_NAME="${I0FC_RGB_ANALYSIS_REPORT_NAME}"
RGB_ANALYSIS_REPORT_TYPE="${I0FC_RGB_ANALYSIS_REPORT_TYPE}"
EXACT_REPORT_SCHEMA="${I0FC_EXACT_REPORT_SCHEMA}"
MINIMUM_P95_IMPROVEMENT_MM="${I0FC_MINIMUM_P95_IMPROVEMENT_MM}"
MINIMUM_RMS_IMPROVEMENT_MM="${I0FC_MINIMUM_RMS_IMPROVEMENT_MM}"
MAXIMUM_SLOWDOWN_RATIO="${I0FC_MAXIMUM_SLOWDOWN_RATIO}"
REQUIRE_FRESH_SESSION="${I0FC_REQUIRE_FRESH_SESSION}"
SESSION_MARKER_NAME="${I0FC_SESSION_MARKER_NAME}"
SUPERSEDES_PROTOCOL="${I0FC_SUPERSEDES_PROTOCOL}"
LEGACY_SOURCE_COMMIT="${I0FC_LEGACY_SOURCE_COMMIT}"
OPERATOR_NOTE_FROZEN="${I0FC_OPERATOR_NOTE_FROZEN}"
STRICT_RUNTIME_CONTRACT="${I0FC_STRICT_RUNTIME_CONTRACT}"
BLOCK="${I0FC_BLOCK}"
POSITION="${I0FC_POSITION}"
CONDITION="${I0FC_CONDITION}"
CONDITION_LABEL="${I0FC_CONDITION_LABEL}"
PILOT_METHOD="${I0FC_PILOT_METHOD}"
VARIANT="${I0FC_VARIANT}"
W_SLOSH="${I0FC_W_SLOSH}"
SLOSH_ENABLED="${I0FC_SLOSH_ENABLED}"
OBSERVER_APPLIED="${I0FC_OBSERVER_APPLIED}"
EXPECTED_COST_HORIZON_STEPS="${I0FC_EXPECTED_COST_HORIZON_STEPS}"
EXPECTED_COST_TAIL_DISCOUNT="${I0FC_EXPECTED_COST_TAIL_DISCOUNT}"
CANONICAL_RUN_LABEL="${I0FC_CANONICAL_RUN_LABEL}"
PREVIOUS_LABEL="${I0FC_PREVIOUS_RUN_LABEL}"
SCRIPT_NAME="run_spmpc_i0_failclosed_fixed_abba_engine:${PROFILE_ID}"

[[ "${ATTEMPT}" == "01" ]] || fail "development ABBA permits first attempt only (ATTEMPT=01)"

V_REF=0.20
# The C02 profile used 0.10/0.15.  This new development profile retains the
# same +0.05 m/s hard margin while raising the RGB-effect run to v_ref=0.20.
V_SAFE_MAX=0.25
SPEED_SAFETY_TOLERANCE=0.0001
DELAY_PHASE_MODE=fixed_closed_loop
DELAY_PHASE_LINEAR_DELAY_SEC=0.15
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22
RECORD_SEC=70
MIN_FREE_GIB="${MIN_FREE_GIB:-5}"
START_GATE_TIMEOUT_SEC=120
IMU_SHADOW_READY_TIMEOUT_SEC=20

FROZEN_PATH_FILE=/home/geist/fixed_paths/real/20260829_spmpc_mocap_execution_chain/candidates/mocap_compact_s_C02.json
FROZEN_PATH_SHA256=1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164
PATH_FILE="${PATH_FILE:-${FROZEN_PATH_FILE}}"
PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256:-${FROZEN_PATH_SHA256}}"
FROZEN_MAP_FILE=/home/geist/scout_maps/real/20260829_mocap_exec/map_carto_20260829_mocap_exec_v1.pbstream
FROZEN_MAP_SHA256=34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595
FIELD_MAP_FILE="${FIELD_MAP_FILE:-${FROZEN_MAP_FILE}}"
FIELD_MAP_EXPECTED_SHA256="${FIELD_MAP_EXPECTED_SHA256:-${FROZEN_MAP_SHA256}}"
FIELD_MAP_RESOLUTION=0.02

FROZEN_RGB_CALIBRATION_FILE=/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml
FROZEN_RGB_CALIBRATION_SHA256=7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01
RGB_CALIBRATION_FILE="${RGB_CALIBRATION_FILE:-${FROZEN_RGB_CALIBRATION_FILE}}"
RGB_EXPECTED_WIDTH=1920
RGB_EXPECTED_HEIGHT=1080
RGB_EXPECTED_FPS=30
RGB_IMAGE_TOPIC=/camera/color/image_raw
ONLINE_LIQUID_MEASUREMENT_TOPIC=/liquid/measurement
ONLINE_LIQUID_PROCESS_EVERY=1
ONLINE_LIQUID_ZERO_FRAMES=30
ONLINE_LIQUID_HUE1_LOW=0
ONLINE_LIQUID_HUE1_HIGH=12
ONLINE_LIQUID_HUE2_LOW=161
ONLINE_LIQUID_HUE2_HIGH=179
ONLINE_LIQUID_SAT_MIN=101
ONLINE_LIQUID_VAL_MIN=167
ONLINE_LIQUID_HEIGHT_BIAS_MM=0.0

if truthy "${STRICT_RUNTIME_CONTRACT}"; then
  MOCAP_TRACKER="${I0FC_RUNTIME_MOCAP_TRACKER}"
  IMU_TOPIC="${I0FC_RUNTIME_IMU_TOPIC}"
  ODOM_TOPIC="${I0FC_RUNTIME_ODOM_TOPIC}"
else
  MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
  IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
  ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
fi
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_${OUTPUT_TAG}/H0}"
requested_run_label="${RUN_LABEL:-}"
requested_name="${NAME:-}"
if truthy "${REQUIRE_FRESH_SESSION}"; then
  [[ -z "${requested_run_label}" || "${requested_run_label}" == "${CANONICAL_RUN_LABEL}" ]] \
    || fail "${PROFILE_ID} forbids RUN_LABEL override; expected ${CANONICAL_RUN_LABEL}"
  [[ -z "${requested_name}" || "${requested_name}" == "${CANONICAL_RUN_LABEL}" ]] \
    || fail "${PROFILE_ID} forbids NAME override; expected ${CANONICAL_RUN_LABEL}"
fi
RUN_LABEL="${requested_run_label:-${CANONICAL_RUN_LABEL}}"
NAME="${requested_name:-${RUN_LABEL}}"
BAG_PATH="${RUN_OUT_DIR}/${NAME}.bag"
EXACT_REPORT="${RUN_OUT_DIR}/${NAME}${EXACT_REPORT_SUFFIX}"
OBSERVER_REPORT="${RUN_OUT_DIR}/${NAME}${OBSERVER_REPORT_SUFFIX}"
CHAIN_REPORT="${RUN_OUT_DIR}/${NAME}${CHAIN_REPORT_SUFFIX}"
RGB_REPORT="${RUN_OUT_DIR}/${NAME}${RGB_REPORT_SUFFIX}"
RGB_ANALYSIS_REPORT="${RUN_OUT_DIR}/${RGB_ANALYSIS_REPORT_NAME}"
UNIT_PASS="${RUN_OUT_DIR}/${NAME}${UNIT_PASS_SUFFIX}"
PREREG_FILE="${RUN_OUT_DIR}/${PROTOCOL_ID}_prereg.env"
ORDER_FILE="${RUN_OUT_DIR}/${PROTOCOL_ID}_order.csv"
SESSION_MARKER=""
if [[ -n "${SESSION_MARKER_NAME}" ]]; then
  SESSION_MARKER="${RUN_OUT_DIR}/${SESSION_MARKER_NAME}"
fi

required_files=(
  "${RUNNER}" "${PROFILE_TOOL}" "${EXACT_POSTFLIGHT}" "${WINDOW_CONTRACT}" "${OBSERVER_POSTFLIGHT}" "${RGB_POSTFLIGHT}" "${RGB_ANALYZER}"
  "${CHAIN_POSTFLIGHT}" "${SUMMARIZER}" "${PATH_VALIDATOR}" "${MAP_VALIDATOR}"
  "${CAMERA_PREP}" "${CONTRACT_TEST}" "${ONLINE_LIQUID_LAUNCH}" "${ONLINE_LIQUID_NODE}"
  "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}" "${PATH_FILE}"
  "${FIELD_MAP_FILE}" "${RGB_CALIBRATION_FILE}"
)
if truthy "${STRICT_RUNTIME_CONTRACT}"; then
  required_files+=(
    "${SHORT_HORIZON_TEST}" "${RUNTIME_GATE_TEST}" "${RGB_ANALYZER_TEST}"
  )
fi
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"
[[ "${PATH_EXPECTED_SHA256,,}" == "${FROZEN_PATH_SHA256}" ]] || fail "C02 path hash contract changed"
[[ "${FIELD_MAP_EXPECTED_SHA256,,}" == "${FROZEN_MAP_SHA256}" ]] || fail "C02 map hash contract changed"
[[ "$(sha256sum "${PATH_FILE}" | awk '{print $1}')" == "${FROZEN_PATH_SHA256}" ]] || fail "C02 path SHA-256 mismatch"
[[ "$(sha256sum "${FIELD_MAP_FILE}" | awk '{print $1}')" == "${FROZEN_MAP_SHA256}" ]] || fail "C02 map SHA-256 mismatch"
[[ "$(sha256sum "${RGB_CALIBRATION_FILE}" | awk '{print $1}')" == "${FROZEN_RGB_CALIBRATION_SHA256}" ]] || fail "RGB calibration SHA-256 mismatch"

python3 "${PATH_VALIDATOR}" "${PATH_FILE}" --expected-sha256 "${FROZEN_PATH_SHA256}" >/dev/null
python3 "${MAP_VALIDATOR}" "${FIELD_MAP_FILE}" \
  --expected-resolution "${FIELD_MAP_RESOLUTION}" \
  --expected-pbstream-sha256 "${FROZEN_MAP_SHA256}" >/dev/null

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

launch_runtime_contract_args=()
if truthy "${STRICT_RUNTIME_CONTRACT}"; then
  launch_runtime_contract_args=(
    "cmd_vel_topic:=${I0FC_RUNTIME_CMD_TOPIC}"
    "w_smooth:=${I0FC_RUNTIME_W_SMOOTH}"
    "w_alpha:=${I0FC_RUNTIME_W_ALPHA}"
    "w_du_a:=${I0FC_RUNTIME_W_DU_A}"
    "w_du_vs:=${I0FC_RUNTIME_W_DU_VS}"
    "slosh_height_max:=${I0FC_RUNTIME_SLOSH_HEIGHT_MAX}"
    "alpha_max:=${I0FC_RUNTIME_ALPHA_MAX}"
    "observer_max_imu_state_age_sec:=${I0FC_RUNTIME_OBSERVER_MAX_IMU_STATE_AGE_SEC}"
    "observer_max_odom_state_age_sec:=${I0FC_RUNTIME_OBSERVER_MAX_ODOM_STATE_AGE_SEC}"
    "observer_max_future_skew_sec:=${I0FC_RUNTIME_OBSERVER_MAX_FUTURE_SKEW_SEC}"
    "liquid_nowcast_max_prediction_sec:=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_PREDICTION_SEC}"
    "liquid_nowcast_max_excitation_age_sec:=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC}"
    "liquid_nowcast_max_future_skew_sec:=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC}"
    "liquid_nowcast_max_state_excitation_skew_sec:=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_STATE_EXCITATION_SKEW_SEC}"
    "liquid_nowcast_max_integration_step_sec:=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_INTEGRATION_STEP_SEC}"
    "state_timing_max_raw_skew_sec:=${I0FC_RUNTIME_STATE_TIMING_MAX_RAW_SKEW_SEC}"
    "state_timing_max_interpolation_gap_sec:=${I0FC_RUNTIME_STATE_TIMING_MAX_INTERPOLATION_GAP_SEC}"
    "state_timing_max_robot_extrapolation_sec:=${I0FC_RUNTIME_STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC}"
    "execution_contract_max_post_limit_delta_v:=${I0FC_RUNTIME_EXECUTION_CONTRACT_MAX_DELTA_V}"
    "execution_contract_max_post_limit_delta_omega:=${I0FC_RUNTIME_EXECUTION_CONTRACT_MAX_DELTA_OMEGA}"
    "shared_linear_accel_max:=${I0FC_RUNTIME_SHARED_LINEAR_ACCEL_MAX}"
    "shared_angular_rate_max:=${I0FC_RUNTIME_SHARED_ANGULAR_RATE_MAX}"
    "shared_angular_accel_max:=${I0FC_RUNTIME_SHARED_ANGULAR_ACCEL_MAX}"
  )
fi

validate_launch_variant() {
  local variant="$1"
  local weight="$2"
  local slosh_enabled="$3"
  local cost_horizon_steps="$4"
  local cost_tail_discount="$5"
  local dump
  dump="$(roslaunch --dump-params spmpc_local_planner spmpc_fixed_path.launch \
    planner_variant:="${variant}" solver_backend:=continuous_mpcc_acados \
    imu_shadow_enable:=true observer_source:=processed_imu \
    observer_fallback_policy:=fail_closed observer_latch_fallback:=false \
    delay_phase_mode:=fixed_closed_loop delay_phase_linear_delay_sec:=0.15 \
    delay_phase_angular_delay_sec:=0.22 state_timing_require_common_epoch:=true \
    liquid_nowcast_enable:=true liquid_nowcast_publish_comparison:=true \
    shared_linear_accel_limit_enable:=false shared_angular_limit_enable:=false \
    execution_contract_fail_closed_on_post_limit_change:=true \
    speed_safety_enable:=true v_safe_max:=0.25 speed_safety_tolerance:=0.0001 \
    v_ref:=0.20 w_slosh:="${weight}" \
    "${launch_runtime_contract_args[@]}")" || fail "could not dump ${variant} launch parameters"
  local expected_lines=(
    "/spmpc_local_planner/planner_variant: ${variant}"
    "/spmpc_local_planner/solver_backend: continuous_mpcc_acados"
    "/spmpc_local_planner/variants/${variant}/slosh_enable: ${slosh_enabled}"
    "/spmpc_local_planner/variants/${variant}/slosh_constraint_enable: false"
    "/spmpc_local_planner/variants/${variant}/smooth_priority_enable: false"
    "/spmpc_local_planner/variants/${variant}/w_slosh: ${weight}"
    "/spmpc_local_planner/variants/${variant}/v_ref: 0.2"
    "/spmpc_local_planner/slosh_observer/source: processed_imu"
    "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
    "/spmpc_local_planner/slosh_observer/latch_fallback: false"
    "/spmpc_local_planner/delay_phase/mode: fixed_closed_loop"
    "/spmpc_local_planner/delay_phase/linear_delay_sec: 0.15"
    "/spmpc_local_planner/delay_phase/angular_delay_sec: 0.22"
    "/spmpc_local_planner/state_timing/require_common_epoch: true"
    "/spmpc_local_planner/platform/shared_constraints/linear_accel_limit_enable: false"
    "/spmpc_local_planner/platform/shared_constraints/angular_limit_enable: false"
    "/spmpc_local_planner/execution_contract/fail_closed_on_post_limit_change: true"
    "/spmpc_local_planner/speed_safety/enable: true"
    "/spmpc_local_planner/speed_safety/v_safe_max: 0.25"
  )
  if truthy "${STRICT_RUNTIME_CONTRACT}"; then
    expected_lines+=(
      "/spmpc_local_planner/topics/cmd_vel: ${I0FC_RUNTIME_CMD_TOPIC}"
      "/spmpc_local_planner/variants/${variant}/w_smooth: ${I0FC_RUNTIME_W_SMOOTH}"
      "/spmpc_local_planner/variants/${variant}/w_alpha: ${I0FC_RUNTIME_W_ALPHA}"
      "/spmpc_local_planner/variants/${variant}/w_du_a: ${I0FC_RUNTIME_W_DU_A}"
      "/spmpc_local_planner/variants/${variant}/w_du_vs: ${I0FC_RUNTIME_W_DU_VS}"
      "/spmpc_local_planner/slosh/slosh_height_max: ${I0FC_RUNTIME_SLOSH_HEIGHT_MAX}"
      "/spmpc_local_planner/robot/alpha_max: ${I0FC_RUNTIME_ALPHA_MAX}"
    )
  fi
  local line
  for line in "${expected_lines[@]}"; do
    grep -Fqx -- "${line}" <<< "${dump}" || fail "${variant} launch contract missing: ${line}"
  done
  if (( cost_horizon_steps >= 0 )); then
    grep -Fqx -- \
      "/spmpc_local_planner/variants/${variant}/slosh_cost_horizon_steps: ${cost_horizon_steps}" \
      <<< "${dump}" || fail "${variant} launch contract has the wrong liquid-cost horizon"
    grep -Fqx -- \
      "/spmpc_local_planner/variants/${variant}/slosh_cost_tail_discount: ${cost_tail_discount}" \
      <<< "${dump}" || fail "${variant} launch contract has the wrong liquid-cost tail discount"
  fi
}

validate_launch_variant B0 0.0 false -1 1.0
validate_launch_variant "${TREATMENT_VARIANT}" 5.0 true \
  "${TREATMENT_COST_HORIZON_STEPS}" "${TREATMENT_COST_TAIL_DISCOUNT}"
python3 "${CONTRACT_TEST}"
if truthy "${STRICT_RUNTIME_CONTRACT}"; then
  python3 "${SHORT_HORIZON_TEST}"
  python3 "${RUNTIME_GATE_TEST}"
  python3 "${RGB_ANALYZER_TEST}"
fi
VALIDATE_ONLY=true bash "${CAMERA_PREP}" >/dev/null

echo "================ I0 fail-closed fixed ABBA ================"
echo "  profile        = ${PROFILE_ID}; protocol=${PROTOCOL_ID}"
echo "  row/order      = ${PAIR_ROW}/04; B0,Bslosh,Bslosh,B0"
echo "  condition      = ${CONDITION_LABEL}; ${VARIANT}; w_slosh=${W_SLOSH}"
echo "  observer       = processed-IMU I0; fail_closed; common_epoch=true"
echo "  solver liquid  = ${OBSERVER_APPLIED}; cost steps=${EXPECTED_COST_HORIZON_STEPS}, tail=${EXPECTED_COST_TAIL_DISCOUNT}"
echo "  legacy delay   = fixed_closed_loop 0.15/0.22 s"
echo "  speed profile  = NEW DEVELOPMENT v_ref=0.20, hard v_safe=0.25 m/s"
echo "  evidence       = online RGB scalar + NOKOV + O0/I0/I1/L22 + solver audits"
echo "  output         = ${BAG_PATH}"
echo "============================================================="

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS; motion NOT started"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || fail "set ARM_MOTION=YES only after path clearance and E-stop check"
[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || fail "set CONFIRM_RGB_GEOMETRY=YES after checking camera/container/rulers"
[[ "${CONFIRM_NEW_SPEED_PROFILE}" == "YES" ]] || fail "set CONFIRM_NEW_SPEED_PROFILE=YES only after accepting the new 0.20/0.25 m/s development profile"

runtime_paths=(
  src/scout_apps/control/spmpc_local_planner/config
  src/scout_apps/control/spmpc_local_planner/include
  src/scout_apps/control/spmpc_local_planner/launch
  src/scout_apps/control/spmpc_local_planner/msg
  src/scout_apps/control/spmpc_local_planner/src
  src/scout_apps/control/spmpc_local_planner/generated
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_i0_failclosed_fixed_abba_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_i0_failclosed_fixed_short100_abba_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/lib/run_spmpc_i0_failclosed_fixed_abba_engine.sh
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/i0_failclosed_fixed_abba_profile.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/liquid_cost_window_contract.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_i0_failclosed_fixed_abba_bag.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_slosh_nowcast_shadow_bag.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_g3_online_rgb_trial.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/analyze_i0_failclosed_fixed_abba_rgb.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_mocap_execution_chain_bag.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_mocap_s_path.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_mocap_field_map.py
  src/scout_apps/control/spmpc_local_planner/scripts/summarize_spmpc_real_trial.py
  src/scout_apps/control/spmpc_local_planner/scripts/prepare_spmpc_g3_realsense.sh
  src/scout_apps/control/spmpc_local_planner/scripts/tests/test_i0_failclosed_fixed_abba_contract.py
  src/scout_apps/sensors/realsense_liquid_measurement
)
if truthy "${STRICT_RUNTIME_CONTRACT}"; then
  runtime_paths+=(
    src/scout_apps/control/spmpc_local_planner/scripts/tests/test_short_horizon_matched_release.py
    src/scout_apps/control/spmpc_local_planner/scripts/tests/test_i0_failclosed_fixed_short100_runtime_gate.py
    src/scout_apps/control/spmpc_local_planner/scripts/tests/test_i0_failclosed_fixed_abba_rgb_analysis.py
  )
fi
dirty_runtime="$(git -C "${REPO_ROOT}" status --porcelain --untracked-files=normal -- "${runtime_paths[@]}")"
[[ -z "${dirty_runtime}" ]] || fail "runtime/evidence paths are dirty; commit and rebuild before motion"

for output in "${BAG_PATH}" "${BAG_PATH}.active" "${EXACT_REPORT}" \
  "${OBSERVER_REPORT}" "${CHAIN_REPORT}" "${RGB_REPORT}" "${UNIT_PASS}"; do
  [[ ! -e "${output}" ]] || fail "preserve existing output: ${output}"
done

if truthy "${REQUIRE_FRESH_SESSION}" && [[ -d "${RUN_OUT_DIR}" ]]; then
  foreign_v1_artifact="$(find "${RUN_OUT_DIR}" -maxdepth 1 -type f \
    \( -name 'SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1_*' \
       -o -name 'I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS.json' \
       -o -name '*_i0_fixed_rgb_postflight.json' \
       -o -name 'DEV_I0FC_FIXED_[0-9]*' \) -print -quit)"
  [[ -z "${foreign_v1_artifact}" ]] \
    || fail "v1 artifact cannot enter ${PROFILE_ID} session: ${foreign_v1_artifact}"
fi
if truthy "${REQUIRE_FRESH_SESSION}" && [[ "${PAIR_ROW}" == "01" ]]; then
  for session_output in "${PREREG_FILE}" "${ORDER_FILE}" \
    "${SESSION_MARKER}" "${RGB_ANALYSIS_REPORT}"; do
    [[ -z "${session_output}" || ! -e "${session_output}" ]] \
      || fail "row 01 requires a fresh ${PROFILE_ID} session: ${session_output}"
  done
fi

if [[ -n "${PREVIOUS_LABEL}" ]]; then
  previous_pass="${RUN_OUT_DIR}/${PREVIOUS_LABEL}${UNIT_PASS_SUFFIX}"
  [[ -s "${previous_pass}" ]] || fail "previous frozen row has no complete PASS marker: ${previous_pass}"
fi
if [[ "${PAIR_ROW}" == "03" ]]; then
  require_rgb_analysis_decision \
    BLOCK1_RAPID_SCREEN PASS PROMOTE_BLOCK2 01,02
fi

available_kib="$(df -Pk /home/geist/slosh_bags/real | awk 'NR == 2 {print $4}')"
required_kib=$((MIN_FREE_GIB * 1024 * 1024))
(( available_kib >= required_kib )) || fail "require at least ${MIN_FREE_GIB} GiB free"

for topic in /map /scan_front "${ODOM_TOPIC}" "${IMU_TOPIC}" "${RGB_IMAGE_TOPIC}"; do
  timeout 5s rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1 || fail "runtime topic unavailable: ${topic}"
done
raw_mocap_topic="/vrpn_client_node/${MOCAP_TRACKER}/pose"
timeout 5s rostopic echo -n 1 "${raw_mocap_topic}" >/dev/null 2>&1 || fail "no NOKOV pose: ${raw_mocap_topic}"
mocap_status="$(timeout 5s rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
grep -q "OK.*tracker=${MOCAP_TRACKER}" <<< "${mocap_status}" || fail "/mocap/status is not OK"
runtime_map_file="$(rosparam get /cartographer_node/frozen_map_file 2>/dev/null || true)"
runtime_map_sha="$(rosparam get /cartographer_node/frozen_map_expected_sha256 2>/dev/null || true)"
[[ "$(readlink -f "${runtime_map_file}")" == "$(readlink -f "${FIELD_MAP_FILE}")" ]] || fail "runtime Cartographer map differs"
[[ "${runtime_map_sha,,}" == "${FROZEN_MAP_SHA256}" ]] || fail "runtime map hash differs"
published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
grep -Fxq -- /cmd_vel <<< "${published_topics}" && fail "/cmd_vel already has a publisher"

code_revision="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
order_contents="$(printf '%s\n' \
  'row,block,position,condition,variant,w_slosh' \
  '01,01,01,B0,B0,0.0' \
  "02,01,02,Bslosh,${TREATMENT_VARIANT},5.0" \
  "03,02,01,Bslosh,${TREATMENT_VARIANT},5.0" \
  '04,02,02,B0,B0,0.0')"
if [[ "${PROFILE_ID}" == "legacy_v1" ]]; then
  prereg_contents="$(printf '%s\n' \
    "protocol=${PROTOCOL_ID}" "scope=development_only" "not_c02_release=true" \
    "row_order=B0,Bslosh,Bslosh,B0" "v_ref=${V_REF}" "v_safe_max=${V_SAFE_MAX}" \
    "observer=processed_imu" "fallback=fail_closed" "common_epoch=true" \
    "delay=fixed_closed_loop:${DELAY_PHASE_LINEAR_DELAY_SEC},${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
    "path_sha256=${FROZEN_PATH_SHA256}" "map_sha256=${FROZEN_MAP_SHA256}" \
    "rgb_calibration_sha256=${FROZEN_RGB_CALIBRATION_SHA256}" "git_revision=${code_revision}")"
else
  prereg_contents="$(printf '%s\n' \
    "protocol=${PROTOCOL_ID}" "profile=${PROFILE_ID}" "version_label=${VERSION_LABEL}" \
    "scope=development_only" "not_c02_release=true" \
    "supersedes_protocol=${SUPERSEDES_PROTOCOL}" \
    "legacy_source_commit=${LEGACY_SOURCE_COMMIT}" \
    "row_order=B0,Bslosh-short100,Bslosh-short100,B0" \
    "baseline_variant=B0" "treatment_variant=${TREATMENT_VARIANT}" \
    "slosh_cost_state_stages=0..${TREATMENT_COST_HORIZON_STEPS}" \
    "slosh_cost_tail_discount=${TREATMENT_COST_TAIL_DISCOUNT}" \
    "robot_horizon_steps=60" "dt_sec=0.0333333333333333" "robot_horizon_sec=2.0" \
    "solver_backend=${I0FC_RUNTIME_SOLVER_BACKEND}" \
    "cmd_topic=${I0FC_RUNTIME_CMD_TOPIC}" \
    "w_smooth=${I0FC_RUNTIME_W_SMOOTH}" "w_alpha=${I0FC_RUNTIME_W_ALPHA}" \
    "w_du_a=${I0FC_RUNTIME_W_DU_A}" "w_du_vs=${I0FC_RUNTIME_W_DU_VS}" \
    "slosh_height_max=${I0FC_RUNTIME_SLOSH_HEIGHT_MAX}" \
    "alpha_max=${I0FC_RUNTIME_ALPHA_MAX}" \
    "v_ref=${V_REF}" "v_safe_max=${V_SAFE_MAX}" \
    "observer=processed_imu" "fallback=fail_closed" "common_epoch=true" \
    "delay=fixed_closed_loop:${DELAY_PHASE_LINEAR_DELAY_SEC},${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
    "minimum_p95_improvement_mm=${MINIMUM_P95_IMPROVEMENT_MM}" \
    "minimum_rms_improvement_mm=${MINIMUM_RMS_IMPROVEMENT_MM}" \
    "maximum_slowdown_ratio=${MAXIMUM_SLOWDOWN_RATIO}" \
    "path_sha256=${FROZEN_PATH_SHA256}" "map_sha256=${FROZEN_MAP_SHA256}" \
    "rgb_calibration_sha256=${FROZEN_RGB_CALIBRATION_SHA256}" "git_revision=${code_revision}")"
fi
session_contents="$(printf '%s\n' \
  "protocol=${PROTOCOL_ID}" "profile=${PROFILE_ID}" "git_revision=${code_revision}" \
  "path_sha256=${FROZEN_PATH_SHA256}" "map_sha256=${FROZEN_MAP_SHA256}" \
  "rgb_calibration_sha256=${FROZEN_RGB_CALIBRATION_SHA256}")"
PREREG_SHA256="$(printf '%s\n' "${prereg_contents}" | sha256sum | awk '{print $1}')"
OUTCOME_RULE_SHA256="$(printf '%s\n' 'online_rgb_source_stamp_motion_plus_5s_tail_causal_median5' | sha256sum | awk '{print $1}')"
SOURCE_BINDING_SHA256="$(printf '%s\n' "${FROZEN_PATH_SHA256}" "${FROZEN_MAP_SHA256}" "${FROZEN_RGB_CALIBRATION_SHA256}" | sha256sum | awk '{print $1}')"
online_config_sha="$(printf '%s\n' \
  "process_every=${ONLINE_LIQUID_PROCESS_EVERY}" "zero_frames=${ONLINE_LIQUID_ZERO_FRAMES}" \
  "hue1=${ONLINE_LIQUID_HUE1_LOW}:${ONLINE_LIQUID_HUE1_HIGH}" \
  "hue2=${ONLINE_LIQUID_HUE2_LOW}:${ONLINE_LIQUID_HUE2_HIGH}" \
  "sat_min=${ONLINE_LIQUID_SAT_MIN}" "val_min=${ONLINE_LIQUID_VAL_MIN}" \
  "height_bias_mm=${ONLINE_LIQUID_HEIGHT_BIAS_MM}" | sha256sum | awk '{print $1}')"

mkdir -p "${RUN_OUT_DIR}"
if [[ -e "${PREREG_FILE}" ]]; then
  [[ "$(<"${PREREG_FILE}")" == "${prereg_contents}" ]] || fail "prereg changed after row 01"
  [[ "$(<"${ORDER_FILE}")" == "${order_contents}" ]] || fail "ABBA order changed after row 01"
  if [[ -n "${SESSION_MARKER}" ]]; then
    [[ -s "${SESSION_MARKER}" ]] || fail "session marker missing: ${SESSION_MARKER}"
    [[ "$(<"${SESSION_MARKER}")" == "${session_contents}" ]] \
      || fail "session marker changed after row 01"
  fi
else
  [[ "${PAIR_ROW}" == "01" ]] || fail "row 01 must create the prereg/order bundle"
  printf '%s\n' "${prereg_contents}" > "${PREREG_FILE}"
  printf '%s\n' "${order_contents}" > "${ORDER_FILE}"
  if [[ -n "${SESSION_MARKER}" ]]; then
    printf '%s\n' "${session_contents}" > "${SESSION_MARKER}"
  fi
fi

bash "${CAMERA_PREP}"
publisher_count() {
  rostopic info "$1" 2>/dev/null | awk '
    /^Publishers:/ {inside=1; next}
    /^Subscribers:/ {inside=0}
    inside && /^[[:space:]]+\*/ {count++}
    END {print count+0}'
}
[[ "$(publisher_count "${ONLINE_LIQUID_MEASUREMENT_TOPIC}")" == "0" ]] || fail "stale online-liquid publisher exists"

online_log="${RUN_OUT_DIR}/${NAME}_online_liquid.log"
online_pid=""
cleanup_online() {
  if [[ -n "${online_pid}" ]] && kill -0 "${online_pid}" 2>/dev/null; then
    kill -INT "${online_pid}" 2>/dev/null || true
    wait "${online_pid}" 2>/dev/null || true
  fi
}
trap cleanup_online EXIT INT TERM

roslaunch realsense_liquid_measurement online_liquid_height.launch \
  calibration:="${RGB_CALIBRATION_FILE}" image_topic:="${RGB_IMAGE_TOPIC}" \
  measurement_topic:="${ONLINE_LIQUID_MEASUREMENT_TOPIC}" \
  process_every:="${ONLINE_LIQUID_PROCESS_EVERY}" zero_frames:="${ONLINE_LIQUID_ZERO_FRAMES}" \
  publish_debug:=false height_bias_mm:="${ONLINE_LIQUID_HEIGHT_BIAS_MM}" \
  hue1_low:="${ONLINE_LIQUID_HUE1_LOW}" hue1_high:="${ONLINE_LIQUID_HUE1_HIGH}" \
  hue2_low:="${ONLINE_LIQUID_HUE2_LOW}" hue2_high:="${ONLINE_LIQUID_HUE2_HIGH}" \
  sat_min:="${ONLINE_LIQUID_SAT_MIN}" val_min:="${ONLINE_LIQUID_VAL_MIN}" \
  > "${online_log}" 2>&1 &
online_pid=$!
sleep 2
kill -0 "${online_pid}" 2>/dev/null || { tail -80 "${online_log}" >&2 || true; fail "online liquid node exited"; }
ready_log="${RUN_OUT_DIR}/${NAME}_online_liquid_ready.log"
timeout 20s rostopic echo -n 20 \
  --filter "m.valid and m.zero_locked and m.status_code == 0 and not m.any_clipped" \
  "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" > "${ready_log}" 2>&1 || fail "online RGB scalar did not become READY"

case "${RUNNER_SELECTOR_MODE}" in
  pilot_method)
    runner_selector_env=("PILOT_METHOD=${PILOT_METHOD}")
    ;;
  direct_variant)
    runner_selector_env=("PILOT_METHOD=" "VARIANT=${VARIANT}" "ALG=${VARIANT}")
    ;;
  *)
    fail "unknown runner selector mode: ${RUNNER_SELECTOR_MODE}"
    ;;
esac
runtime_contract_env=()
if truthy "${STRICT_RUNTIME_CONTRACT}"; then
  runtime_contract_env=(
    "MATRIX_PRESET="
    "SOLVER_BACKEND=${I0FC_RUNTIME_SOLVER_BACKEND}"
    "CMD_TOPIC=${I0FC_RUNTIME_CMD_TOPIC}"
    "REF_TOPIC=${I0FC_RUNTIME_REFERENCE_PATH_TOPIC}"
    "COSTMAP_TOPIC=${I0FC_RUNTIME_COSTMAP_TOPIC}"
    "REFERENCE_TARGET_FRAME=${I0FC_RUNTIME_REFERENCE_TARGET_FRAME}"
    "BASE_FRAME=${I0FC_RUNTIME_BASE_FRAME}"
    "IMU_SHADOW_READY_TOPIC=${I0FC_RUNTIME_IMU_READY_TOPIC}"
    "OBSERVER_SELECTION_TOPIC=${I0FC_RUNTIME_OBSERVER_SELECTION_TOPIC}"
    "W_SMOOTH=${I0FC_RUNTIME_W_SMOOTH}"
    "W_ALPHA=${I0FC_RUNTIME_W_ALPHA}"
    "W_DU_A=${I0FC_RUNTIME_W_DU_A}"
    "W_DU_VS=${I0FC_RUNTIME_W_DU_VS}"
    "SLOSH_HEIGHT_MAX=${I0FC_RUNTIME_SLOSH_HEIGHT_MAX}"
    "ALPHA_MAX=${I0FC_RUNTIME_ALPHA_MAX}"
    "OBSERVER_MAX_IMU_STATE_AGE_SEC=${I0FC_RUNTIME_OBSERVER_MAX_IMU_STATE_AGE_SEC}"
    "OBSERVER_MAX_ODOM_STATE_AGE_SEC=${I0FC_RUNTIME_OBSERVER_MAX_ODOM_STATE_AGE_SEC}"
    "OBSERVER_MAX_FUTURE_SKEW_SEC=${I0FC_RUNTIME_OBSERVER_MAX_FUTURE_SKEW_SEC}"
    "LIQUID_NOWCAST_MAX_PREDICTION_SEC=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_PREDICTION_SEC}"
    "LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC}"
    "LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC}"
    "LIQUID_NOWCAST_MAX_STATE_EXCITATION_SKEW_SEC=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_STATE_EXCITATION_SKEW_SEC}"
    "LIQUID_NOWCAST_MAX_INTEGRATION_STEP_SEC=${I0FC_RUNTIME_LIQUID_NOWCAST_MAX_INTEGRATION_STEP_SEC}"
    "STATE_TIMING_MAX_RAW_SKEW_SEC=${I0FC_RUNTIME_STATE_TIMING_MAX_RAW_SKEW_SEC}"
    "STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=${I0FC_RUNTIME_STATE_TIMING_MAX_INTERPOLATION_GAP_SEC}"
    "STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC=${I0FC_RUNTIME_STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC}"
    "EXECUTION_CONTRACT_MAX_DELTA_V=${I0FC_RUNTIME_EXECUTION_CONTRACT_MAX_DELTA_V}"
    "EXECUTION_CONTRACT_MAX_DELTA_OMEGA=${I0FC_RUNTIME_EXECUTION_CONTRACT_MAX_DELTA_OMEGA}"
    "SHARED_LINEAR_ACCEL_MAX=${I0FC_RUNTIME_SHARED_LINEAR_ACCEL_MAX}"
    "SHARED_ANGULAR_RATE_MAX=${I0FC_RUNTIME_SHARED_ANGULAR_RATE_MAX}"
    "SHARED_ANGULAR_ACCEL_MAX=${I0FC_RUNTIME_SHARED_ANGULAR_ACCEL_MAX}"
    "RECORDER_SCRIPT=${SCRIPT_DIR}/record_spmpc_full_rgb_bag.sh"
    "RECORDER_STARTUP_SEC=${I0FC_RUNTIME_RECORDER_STARTUP_SEC}"
    "RECORDER_ACTIVE_TIMEOUT_SEC=${I0FC_RUNTIME_RECORDER_ACTIVE_TIMEOUT_SEC}"
    "PLANNER_STARTUP_SEC=${I0FC_RUNTIME_PLANNER_STARTUP_SEC}"
    "PATH_GENERATOR_STARTUP_SEC=${I0FC_RUNTIME_PATH_SOURCE_STARTUP_SEC}"
    "PATH_PUBLISH_RATE=${I0FC_RUNTIME_PATH_PUBLISH_RATE_HZ}"
    "SPLIT_BLOCK=false"
    "ACQUISITION_RETRY=false"
    "RETRY_REASON_FILE="
    "EXPECTED_RUNTIME_VARIANT=${VARIANT}"
    "RUNTIME_VARIANT_TIMEOUT_SEC=${I0FC_RUNTIME_VARIANT_TIMEOUT_SEC}"
  )
fi
if [[ "${PROFILE_ID}" == "legacy_v1" ]]; then
  BLOCK_SEGMENT_ID_VALUE="I0FC_FIXED_b${BLOCK}"
else
  BLOCK_SEGMENT_ID_VALUE="I0FC_FIXED_S100_V2_b${BLOCK}"
fi

env "${runner_selector_env[@]}" "${runtime_contract_env[@]}" \
DATE="${DATE}" STAMP="${STAMP}" PILOT_MODE=true \
PILOT_CONDITION="${PROTOCOL_ID}" PILOT_RECORD_RGB=false PILOT_RECORD_ONLINE_LIQUID=true \
RUN_LABEL="${RUN_LABEL}" NAME="${NAME}" RUN_OUT_DIR="${RUN_OUT_DIR}" \
PATH_SOURCE_MODE=replay PATH_FILE="${PATH_FILE}" PATH_EXPECTED_SHA256="${FROZEN_PATH_SHA256}" \
REQUIRE_PATH_HASH=true START_POS_TOL=0.08 START_YAW_TOL=0.15 START_HOLD_SEC=0.5 \
START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC}" V_REF="${V_REF}" W_SLOSH="${W_SLOSH}" \
DELAY_PHASE_MODE="${DELAY_PHASE_MODE}" \
DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
IMU_SHADOW_ENABLE=true IMU_TOPIC="${IMU_TOPIC}" IMU_SUBSCRIBER_QUEUE_SIZE=10 \
IMU_SHADOW_READY_TIMEOUT_SEC="${IMU_SHADOW_READY_TIMEOUT_SEC}" CURRENT_OBSERVER_SOURCE=processed_imu \
OBSERVER_FALLBACK_POLICY=fail_closed OBSERVER_LATCH_FALLBACK=false \
LIQUID_NOWCAST_ENABLE=true LIQUID_NOWCAST_PUBLISH_COMPARISON=true \
STATE_TIMING_REQUIRE_COMMON_EPOCH=true SHARED_LINEAR_ACCEL_LIMIT_ENABLE=false \
SHARED_ANGULAR_LIMIT_ENABLE=false EXECUTION_CONTRACT_FAIL_CLOSED=true \
SPEED_SAFETY_ENABLE=true V_SAFE_MAX="${V_SAFE_MAX}" SPEED_SAFETY_TOLERANCE="${SPEED_SAFETY_TOLERANCE}" \
RECORD_RGB=false RECORD_CAMERA=false RECORD_CAMERA_INFO=true RECORD_CAMERA_COMPRESSED=false \
RECORD_DEPTH=false RECORD_ONLINE_LIQUID=true RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false \
RECORD_STANDALONE_SLOSH=false RECORD_SCAN=true FORBID_IMAGE_STREAMS=true \
RECORD_ALL_EXISTING_TOPICS=false RECORD_TOPIC_INFO=true RECORD_MOCAP=true \
RECORD_MOCAP_PATH=false MOCAP_TRACKER="${MOCAP_TRACKER}" \
RGB_CALIBRATION_FILE="${RGB_CALIBRATION_FILE}" \
RGB_CALIBRATION_EXPECTED_SHA256="${FROZEN_RGB_CALIBRATION_SHA256}" \
RGB_EXPECTED_WIDTH="${RGB_EXPECTED_WIDTH}" RGB_EXPECTED_HEIGHT="${RGB_EXPECTED_HEIGHT}" \
RGB_EXPECTED_FPS="${RGB_EXPECTED_FPS}" ONLINE_LIQUID_MEASUREMENT_TOPIC="${ONLINE_LIQUID_MEASUREMENT_TOPIC}" \
ONLINE_LIQUID_PROTOCOL="${PROTOCOL_ID}" ONLINE_LIQUID_CALIBRATION_SHA256="${FROZEN_RGB_CALIBRATION_SHA256}" \
ONLINE_LIQUID_DETECTOR_SHA256="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')" \
ONLINE_LIQUID_NODE_SHA256="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')" \
ONLINE_LIQUID_MSG_SHA256="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')" \
ONLINE_LIQUID_CONFIG_SHA256="${online_config_sha}" RECORD_SEC="${RECORD_SEC}" MAX_RECORD_SEC="${RECORD_SEC}" \
BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID_VALUE}" ORDER_POSITION="${POSITION}" SEND_ZERO_ON_EXIT=true \
OPERATOR_NOTE="${OPERATOR_NOTE_FROZEN}" \
bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"
exact_postflight_args=(
  "${BAG_PATH}" --condition "${CONDITION}"
  --report "${EXACT_REPORT}" --protocol "${PROTOCOL_ID}"
  --report-schema "${EXACT_REPORT_SCHEMA}"
  --expected-v-ref "${V_REF}" --expected-v-safe-max "${V_SAFE_MAX}"
  --minimum-application-fraction 1.0
)
if [[ "${PROFILE_ID}" == "short100_v2" ]]; then
  exact_postflight_args+=(
    --expected-variant "${VARIANT}"
    --expected-slosh-cost-horizon-steps "${EXPECTED_COST_HORIZON_STEPS}"
    --expected-slosh-cost-tail-discount "${EXPECTED_COST_TAIL_DISCOUNT}"
    --expected-slosh-eta-dot-ratio 0.3
    --expected-robot-horizon-steps 60
    --expected-dt-sec 0.0333333333333333
    --expected-control-frequency-hz 30.0
    --expected-config "w_smooth=${I0FC_RUNTIME_W_SMOOTH}"
    --expected-config "w_alpha=${I0FC_RUNTIME_W_ALPHA}"
    --expected-config "w_du_a=${I0FC_RUNTIME_W_DU_A}"
    --expected-config "w_du_vs=${I0FC_RUNTIME_W_DU_VS}"
    --expected-config "slosh_height_max=${I0FC_RUNTIME_SLOSH_HEIGHT_MAX}"
    --expected-config "alpha_max=${I0FC_RUNTIME_ALPHA_MAX}"
  )
fi
python3 "${EXACT_POSTFLIGHT}" "${exact_postflight_args[@]}"

observer_args=(
  "${BAG_PATH}" --report "${OBSERVER_REPORT}" --protocol "${PROTOCOL_ID}"
  --minimum-coverage 0.99 --minimum-rgb-clean-coverage 0.98
  --expected-applied-method "${OBSERVER_APPLIED}"
  --expected-v-safe-max "${V_SAFE_MAX}" --speed-tolerance "${SPEED_SAFETY_TOLERANCE}"
)
if [[ "${SLOSH_ENABLED}" == "true" ]]; then
  observer_args+=(--expected-solver-consumes-liquid)
fi
python3 "${OBSERVER_POSTFLIGHT}" "${observer_args[@]}"

python3 "${CHAIN_POSTFLIGHT}" "${BAG_PATH}" --variant "${VARIANT}" \
  --mocap-tracker "${MOCAP_TRACKER}" --imu-topic "${IMU_TOPIC}" \
  --path-file "${PATH_FILE}" --path-sha256 "${FROZEN_PATH_SHA256}" --report "${CHAIN_REPORT}"

python3 "${RGB_POSTFLIGHT}" --bag "${BAG_PATH}" --condition "${CONDITION}" \
  --slosh-enabled "${SLOSH_ENABLED}" --smooth-priority-enabled false \
  --protocol "${PROTOCOL_ID}" --report-suffix "${RGB_REPORT_SUFFIX}" \
  --row "${PAIR_ROW}" --block "${BLOCK}" --position "${POSITION}" \
  --expected-weight "${W_SLOSH}" --expected-delay-mode-code 3 \
  --expected-solver-source-code 2 --require-delay-compensation-applied true \
  --require-robot-delay-compensation-applied true \
  --require-liquid-delay-compensation-applied true --require-state-diagnostics \
  --expected-v-ref "${V_REF}" --min-duration-sec 65 --min-source-fraction 0.99 \
  --min-ready-fraction 0.99 --min-online-valid-fraction 0.98 \
  --max-zero-window-spread-mm 0.25 --initial-stability-sec 5.0 \
  --min-initial-stability-valid-fraction 0.98 --max-initial-h-vis-p95-mm 0.25 \
  --max-initial-abs-height-p95-mm 0.25 \
  --max-initial-half-median-drift-mm 0.05 \
  --rgb-calibration-sha256 "${FROZEN_RGB_CALIBRATION_SHA256}" \
  --online-config-sha256 "${online_config_sha}" --outcome-window-rule-sha256 "${OUTCOME_RULE_SHA256}" \
  --prereg-sha256 "${PREREG_SHA256}" --source-report-sha256 "${SOURCE_BINDING_SHA256}" --hash-bag

python3 "${SUMMARIZER}" "${BAG_PATH}" --out-dir "${RUN_OUT_DIR}"
if [[ "${PROFILE_ID}" == "legacy_v1" ]]; then
  printf '%s\n' "status=PASS" "protocol=${PROTOCOL_ID}" "row=${PAIR_ROW}" \
    "condition=${CONDITION}" "bag=${BAG_PATH}" "completed_at=$(date --iso-8601=seconds)" > "${UNIT_PASS}"
else
  printf '%s\n' \
    "status=PASS" "protocol=${PROTOCOL_ID}" "profile=${PROFILE_ID}" \
    "row=${PAIR_ROW}" "condition=${CONDITION}" "condition_label=${CONDITION_LABEL}" \
    "variant=${VARIANT}" "slosh_cost_horizon_steps=${EXPECTED_COST_HORIZON_STEPS}" \
    "slosh_cost_tail_discount=${EXPECTED_COST_TAIL_DISCOUNT}" \
    "bag=${BAG_PATH}" \
    "exact_postflight_sha256=$(sha256sum "${EXACT_REPORT}" | awk '{print $1}')" \
    "observer_postflight_sha256=$(sha256sum "${OBSERVER_REPORT}" | awk '{print $1}')" \
    "chain_postflight_sha256=$(sha256sum "${CHAIN_REPORT}" | awk '{print $1}')" \
    "rgb_postflight_sha256=$(sha256sum "${RGB_REPORT}" | awk '{print $1}')" \
    "completed_at=$(date --iso-8601=seconds)" > "${UNIT_PASS}"
fi

rgb_analysis_rc=0
case "${PAIR_ROW}" in
  02)
    run_rgb_analysis \
      BLOCK1_RAPID_SCREEN PROMOTE_BLOCK2 STOP_BLOCK1_FUTILITY 01,02 || rgb_analysis_rc=$?
    ;;
  04)
    run_rgb_analysis \
      COMPLETE_ABBA DEVELOPMENT_POSITIVE NO_DEVELOPMENT_POSITIVE,RGB_POSITIVE_SLOWDOWN_CONFOUNDED 01,02,03,04 || rgb_analysis_rc=$?
    ;;
esac
if (( rgb_analysis_rc == 10 )); then
  exit 10
fi
(( rgb_analysis_rc == 0 )) || fail "unexpected RGB analysis rc=${rgb_analysis_rc}"

echo "[${SCRIPT_NAME}] PASS row=${PAIR_ROW} condition=${CONDITION}"
echo "[${SCRIPT_NAME}] bag=${BAG_PATH}"
echo "[${SCRIPT_NAME}] return to start and wait for the liquid to settle before the next frozen row"
