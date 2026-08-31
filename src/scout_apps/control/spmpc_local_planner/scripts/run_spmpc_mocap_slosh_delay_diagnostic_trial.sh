#!/usr/bin/env bash
# Development-only C02 low-speed trial for direct slosh-delay evidence.
# matched0 is the default: the processed-IMU I0 state enters the 10D solver,
# but w_slosh=0 keeps the first acquisition focused on model prediction rather
# than simultaneously changing the vehicle action through a liquid cost.

set -euo pipefail

SCRIPT_NAME="run_spmpc_mocap_slosh_delay_diagnostic_trial"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"
MAP_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_field_map.py"
CHAIN_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_mocap_execution_chain_bag.py"
MATCHED_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_short_horizon_matched_bag.py"
OBSERVER_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_slosh_nowcast_shadow_bag.py"
SAME_BAG_ANALYZER="${SCRIPT_DIR}/analysis/analyze_slosh_nowcast_same_bag.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"
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

SLOSH_TEST_ID="${SLOSH_TEST_ID:-S01}"
ATTEMPT="${ATTEMPT:-01}"
SLOSH_CONDITION="${SLOSH_CONDITION:-matched0}"
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"

[[ "${SLOSH_TEST_ID}" =~ ^S[0-9]{2}$ ]] || fail "SLOSH_TEST_ID must look like S01"
[[ "${ATTEMPT}" =~ ^0[1-9]$ ]] || fail "ATTEMPT must be 01..09"
case "${SLOSH_CONDITION}" in
  matched0)
    VARIANT=B_slosh_matched0
    W_SLOSH=0.0
    CONDITION_TAG=M0
    ;;
  matched5)
    VARIANT=B_slosh_matched5
    W_SLOSH=5.0
    CONDITION_TAG=M5
    ;;
  *)
    fail "SLOSH_CONDITION must be matched0|matched5"
    ;;
esac

FROZEN_PATH_FILE="/home/geist/fixed_paths/real/20260829_spmpc_mocap_execution_chain/candidates/mocap_compact_s_C02.json"
FROZEN_PATH_SHA256="1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164"
PATH_FILE="${PATH_FILE:-${FROZEN_PATH_FILE}}"
PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256:-${FROZEN_PATH_SHA256}}"

FROZEN_MAP_FILE="/home/geist/scout_maps/real/20260829_mocap_exec/map_carto_20260829_mocap_exec_v1.pbstream"
FROZEN_MAP_SHA256="34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595"
FIELD_MAP_FILE="${FIELD_MAP_FILE:-${FROZEN_MAP_FILE}}"
FIELD_MAP_EXPECTED_SHA256="${FIELD_MAP_EXPECTED_SHA256:-${FROZEN_MAP_SHA256}}"
FIELD_MAP_RESOLUTION="${FIELD_MAP_RESOLUTION:-0.02}"

RGB_CALIBRATION_FILE="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml"
RGB_CALIBRATION_SHA256="7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01"
RGB_IMAGE_TOPIC="/camera/color/image_raw"
ONLINE_LIQUID_MEASUREMENT_TOPIC="/liquid/measurement"
ONLINE_LIQUID_PROCESS_EVERY=1
ONLINE_LIQUID_ZERO_FRAMES=30
ONLINE_LIQUID_HUE1_LOW=0
ONLINE_LIQUID_HUE1_HIGH=12
ONLINE_LIQUID_HUE2_LOW=161
ONLINE_LIQUID_HUE2_HIGH=179
ONLINE_LIQUID_SAT_MIN=101
ONLINE_LIQUID_VAL_MIN=167
ONLINE_LIQUID_HEIGHT_BIAS_MM=0.0

V_REF=0.10
V_SAFE_MAX=0.15
SPEED_SAFETY_TOLERANCE=0.0001
DELAY_PHASE_MODE=shadow
# These legacy windows generate L22 diagnostics only.  Shadow mode guarantees
# that neither value replaces the common-epoch solver input.
DELAY_PHASE_LINEAR_DELAY_SEC=0.15
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22
LIQUID_NOWCAST_MAX_PREDICTION_SEC=0.050
LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC=0.060
LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC=0.005
LIQUID_NOWCAST_MAX_STATE_EXCITATION_SKEW_SEC=0.001
LIQUID_NOWCAST_MAX_INTEGRATION_STEP_SEC=0.020

ACTUATOR_CANDIDATE_PRESET_ID=actuator_fopdt_20260831_same_bag_v1
ACTUATOR_CANDIDATE_LINEAR="0.080,0.086,1.050"
ACTUATOR_CANDIDATE_ANGULAR="0.070,0.677,0.990"

MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
RECORD_SEC="${RECORD_SEC:-90}"
START_POS_TOL="${START_POS_TOL:-0.08}"
START_YAW_TOL="${START_YAW_TOL:-0.15}"
START_HOLD_SEC="${START_HOLD_SEC:-0.5}"
START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC:-120}"

case "${RECORD_SEC}" in
  ''|*[!0-9]*) fail "RECORD_SEC must be an integer in [30,120]" ;;
esac
(( RECORD_SEC >= 30 && RECORD_SEC <= 120 )) || fail "RECORD_SEC must be in [30,120]"

PROTOCOL_ID=SMPCC_C02_slosh_delay_diagnostic_dev_v1
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_mocap_slosh_delay_diagnostic}"
RUN_LABEL="${RUN_LABEL:-DEV_SLOSH_DELAY_${SLOSH_TEST_ID}_${CONDITION_TAG}_a${ATTEMPT}}"
NAME="${NAME:-${RUN_LABEL}}"
BAG_PATH="${RUN_OUT_DIR}/${NAME}.bag"
PROTOCOL_META="${RUN_OUT_DIR}/${NAME}_protocol.env"
CHAIN_REPORT="${RUN_OUT_DIR}/${NAME}_mocap_chain_postflight.json"
MATCHED_REPORT="${RUN_OUT_DIR}/${NAME}_matched_postflight.json"
OBSERVER_REPORT="${RUN_OUT_DIR}/${NAME}_observer_contract_postflight.json"
ANALYSIS_DIR="${RUN_OUT_DIR}/${NAME}_same_bag_observer_analysis"

required_files=(
  "${RUNNER}" "${PATH_VALIDATOR}" "${MAP_VALIDATOR}" "${CHAIN_POSTFLIGHT}"
  "${MATCHED_POSTFLIGHT}" "${OBSERVER_POSTFLIGHT}" "${SAME_BAG_ANALYZER}"
  "${SUMMARIZER}" "${CAMERA_PREP}" "${ONLINE_LIQUID_LAUNCH}"
  "${ONLINE_LIQUID_NODE}" "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}"
  "${PATH_FILE}" "${FIELD_MAP_FILE}" "${RGB_CALIBRATION_FILE}"
)
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"
[[ "${PATH_EXPECTED_SHA256,,}" == "${FROZEN_PATH_SHA256}" ]] || fail "C02 path hash contract changed"
[[ "${FIELD_MAP_EXPECTED_SHA256,,}" == "${FROZEN_MAP_SHA256}" ]] || fail "mocap map hash contract changed"
[[ "$(sha256sum "${PATH_FILE}" | awk '{print $1}')" == "${FROZEN_PATH_SHA256}" ]] || fail "C02 path SHA-256 mismatch"
[[ "$(sha256sum "${FIELD_MAP_FILE}" | awk '{print $1}')" == "${FROZEN_MAP_SHA256}" ]] || fail "mocap map SHA-256 mismatch"
[[ "$(sha256sum "${RGB_CALIBRATION_FILE}" | awk '{print $1}')" == "${RGB_CALIBRATION_SHA256}" ]] || fail "RGB calibration SHA-256 mismatch"

python3 "${PATH_VALIDATOR}" "${PATH_FILE}" --expected-sha256 "${FROZEN_PATH_SHA256}" >/dev/null
python3 "${MAP_VALIDATOR}" "${FIELD_MAP_FILE}" \
  --expected-resolution "${FIELD_MAP_RESOLUTION}" \
  --expected-pbstream-sha256 "${FROZEN_MAP_SHA256}" >/dev/null

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

validate_launch_contract() {
  local dump
  dump="$(roslaunch --dump-params spmpc_local_planner spmpc_fixed_path.launch \
    planner_variant:="${VARIANT}" delay_phase_mode:="${DELAY_PHASE_MODE}" \
    delay_phase_linear_delay_sec:="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
    delay_phase_angular_delay_sec:="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
    imu_shadow_enable:=true observer_source:=processed_imu \
    observer_fallback_policy:=fail_closed observer_latch_fallback:=false \
    liquid_nowcast_enable:=true liquid_nowcast_publish_comparison:=true \
    liquid_nowcast_max_prediction_sec:="${LIQUID_NOWCAST_MAX_PREDICTION_SEC}" \
    liquid_nowcast_max_excitation_age_sec:="${LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC}" \
    liquid_nowcast_max_future_skew_sec:="${LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC}" \
    liquid_nowcast_max_state_excitation_skew_sec:="${LIQUID_NOWCAST_MAX_STATE_EXCITATION_SKEW_SEC}" \
    liquid_nowcast_max_integration_step_sec:="${LIQUID_NOWCAST_MAX_INTEGRATION_STEP_SEC}" \
    shared_linear_accel_limit_enable:=false shared_angular_limit_enable:=false \
    state_timing_require_common_epoch:=true \
    execution_contract_fail_closed_on_post_limit_change:=true \
    speed_safety_enable:=true v_safe_max:="${V_SAFE_MAX}" \
    speed_safety_tolerance:="${SPEED_SAFETY_TOLERANCE}" \
    v_ref:="${V_REF}" w_slosh:="${W_SLOSH}" w_smooth:=1.0 \
    w_alpha:=1.0 w_du_a:=1.0 w_du_vs:=1.0)" || fail "could not dump launch parameters"
  local expected_lines=(
    "/spmpc_local_planner/planner_variant: ${VARIANT}"
    "/spmpc_local_planner/variants/${VARIANT}/slosh_enable: true"
    "/spmpc_local_planner/variants/${VARIANT}/w_slosh: ${W_SLOSH}"
    "/spmpc_local_planner/variants/${VARIANT}/v_ref: 0.1"
    "/spmpc_local_planner/variants/${VARIANT}/slosh_cost_horizon_steps: 3"
    "/spmpc_local_planner/variants/${VARIANT}/slosh_cost_tail_discount: 0.0"
    "/spmpc_local_planner/delay_phase/mode: shadow"
    "/spmpc_local_planner/slosh_observer/source: processed_imu"
    "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
    "/spmpc_local_planner/state_timing/require_common_epoch: true"
    "/spmpc_local_planner/execution_contract/fail_closed_on_post_limit_change: true"
    "/spmpc_local_planner/platform/shared_constraints/linear_accel_limit_enable: false"
    "/spmpc_local_planner/platform/shared_constraints/angular_limit_enable: false"
    "/spmpc_local_planner/liquid_nowcast/enable: true"
    "/spmpc_local_planner/liquid_nowcast/publish_comparison: true"
    "/spmpc_local_planner/speed_safety/enable: true"
    "/spmpc_local_planner/speed_safety/v_safe_max: 0.15"
  )
  local line
  for line in "${expected_lines[@]}"; do
    grep -Fqx -- "${line}" <<< "${dump}" || fail "launch contract missing: ${line}"
  done
}

validate_launch_contract
python3 "${SCRIPT_DIR}/tests/test_short_horizon_matched_release.py"
VALIDATE_ONLY=true bash "${CAMERA_PREP}" >/dev/null

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS"
  echo "  protocol       = ${PROTOCOL_ID}"
  echo "  test/attempt   = ${SLOSH_TEST_ID}/${ATTEMPT}; formal R01--R05 unused"
  echo "  solver         = ${VARIANT}; 10D slosh=true; w_slosh=${W_SLOSH}"
  echo "  state          = processed-IMU I0 -> solver; fail_closed; common epoch"
  echo "  delay          = shadow only; legacy 0.15/0.22 s is not applied"
  echo "  diagnostics    = O0/I0/I1/L22 + planned horizon + cmd/odom/IMU/NOKOV + RGB scalar"
  echo "  candidate      = ${ACTUATOR_CANDIDATE_PRESET_ID}; offline only"
  echo "  speed          = v_ref=0.10, v_safe_max=0.15 m/s"
  echo "  path/map       = frozen C02_v2 / 20260829 mocap field"
  echo "  output         = ${BAG_PATH}"
  echo "  motion         = NOT STARTED"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || fail "set ARM_MOTION=YES only after path clearance and E-stop check"
[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || fail "set CONFIRM_RGB_GEOMETRY=YES after checking camera/container/rulers"
for output in "${BAG_PATH}" "${BAG_PATH}.active" "${PROTOCOL_META}" \
  "${CHAIN_REPORT}" "${MATCHED_REPORT}" "${OBSERVER_REPORT}" "${ANALYSIS_DIR}"; do
  [[ ! -e "${output}" ]] || fail "preserve existing output and choose a new ATTEMPT: ${output}"
done

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

b0_solver="${REPO_ROOT}/src/scout_apps/control/spmpc_local_planner/generated/acados/spmpc_b0/libacados_ocp_solver_spmpc_b0.so"
slosh_solver="${REPO_ROOT}/src/scout_apps/control/spmpc_local_planner/generated/acados/spmpc_slosh/libacados_ocp_solver_spmpc_slosh.so"
[[ -s "${b0_solver}" && -s "${slosh_solver}" ]] || fail "generated solver artifacts are missing"
code_revision="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
dirty_count="$(git -C "${REPO_ROOT}" status --short | wc -l)"
online_config_sha="$(printf '%s\n' \
  "process_every=${ONLINE_LIQUID_PROCESS_EVERY}" "zero_frames=${ONLINE_LIQUID_ZERO_FRAMES}" \
  "hue1=${ONLINE_LIQUID_HUE1_LOW}:${ONLINE_LIQUID_HUE1_HIGH}" \
  "hue2=${ONLINE_LIQUID_HUE2_LOW}:${ONLINE_LIQUID_HUE2_HIGH}" \
  "sat_min=${ONLINE_LIQUID_SAT_MIN}" "val_min=${ONLINE_LIQUID_VAL_MIN}" \
  "height_bias_mm=${ONLINE_LIQUID_HEIGHT_BIAS_MM}" | sha256sum | awk '{print $1}')"

mkdir -p "${RUN_OUT_DIR}"
{
  echo "protocol_id=${PROTOCOL_ID}"
  echo "scope=development_slosh_delay_diagnostic"
  echo "formal_trial_consumed=false"
  echo "test_id=${SLOSH_TEST_ID}"
  echo "attempt=${ATTEMPT}"
  echo "git_revision=${code_revision}"
  echo "git_dirty_count=${dirty_count}"
  echo "condition=${SLOSH_CONDITION}"
  echo "variant=${VARIANT}"
  echo "w_slosh=${W_SLOSH}"
  echo "solver_consumes_liquid=true"
  echo "solver_liquid_method=I0"
  echo "delay_phase_mode=${DELAY_PHASE_MODE}"
  echo "legacy_delay_applied=false"
  echo "legacy_shadow_delay_sec=${DELAY_PHASE_LINEAR_DELAY_SEC},${DELAY_PHASE_ANGULAR_DELAY_SEC}"
  echo "i1_applied_to_solver=false"
  echo "l22_applied_to_solver=false"
  echo "actuator_candidate_preset_id=${ACTUATOR_CANDIDATE_PRESET_ID}"
  echo "actuator_candidate_applied=false"
  echo "actuator_candidate_linear_delay_tau_gain=${ACTUATOR_CANDIDATE_LINEAR}"
  echo "actuator_candidate_angular_delay_tau_gain=${ACTUATOR_CANDIDATE_ANGULAR}"
  echo "v_ref=${V_REF}"
  echo "v_safe_max=${V_SAFE_MAX}"
  echo "path_file=${PATH_FILE}"
  echo "path_sha256=${FROZEN_PATH_SHA256}"
  echo "map_file=${FIELD_MAP_FILE}"
  echo "map_sha256=${FROZEN_MAP_SHA256}"
  echo "rgb_calibration_sha256=${RGB_CALIBRATION_SHA256}"
  echo "online_config_sha256=${online_config_sha}"
  echo "b0_solver_sha256=$(sha256sum "${b0_solver}" | awk '{print $1}')"
  echo "slosh_solver_sha256=$(sha256sum "${slosh_solver}" | awk '{print $1}')"
  echo "created_at=$(date --iso-8601=seconds)"
} > "${PROTOCOL_META}"

bash "${CAMERA_PREP}"
publisher_count() {
  rostopic info "$1" 2>/dev/null | awk '
    /^Publishers:/ {inside=1; next}
    /^Subscribers:/ {inside=0}
    inside && /^[[:space:]]+\*/ {count++}
    END {print count+0}'
}
[[ "$(publisher_count "${ONLINE_LIQUID_MEASUREMENT_TOPIC}")" == "0" ]] || fail "stale /liquid/measurement publisher exists"

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
  --filter "m.valid and m.zero_locked and m.status_code == 0" \
  "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" > "${ready_log}" 2>&1 || fail "online RGB scalar did not become READY"

echo "================ C02 slosh-delay diagnostic ================"
echo "  test/attempt = ${SLOSH_TEST_ID}/${ATTEMPT}; R01--R05 unused"
echo "  solver       = ${VARIANT}; w_slosh=${W_SLOSH}; I0 applied"
echo "  delay        = shadow only; old and candidate compensation are not applied"
echo "  speed        = ${V_REF} m/s, hard maximum ${V_SAFE_MAX} m/s"
echo "  output       = ${BAG_PATH}"
echo "============================================================="

DATE="${DATE}" STAMP="${STAMP}" PILOT_MODE=true PILOT_METHOD="" \
VARIANT="${VARIANT}" ALG="${VARIANT}" PILOT_RECORD_RGB=false \
PILOT_RECORD_ONLINE_LIQUID=true PILOT_CONDITION="${PROTOCOL_ID}" \
RUN_LABEL="${RUN_LABEL}" NAME="${NAME}" RUN_OUT_DIR="${RUN_OUT_DIR}" \
PATH_SOURCE_MODE=replay PATH_FILE="${PATH_FILE}" \
PATH_EXPECTED_SHA256="${FROZEN_PATH_SHA256}" REQUIRE_PATH_HASH=true \
START_POS_TOL="${START_POS_TOL}" START_YAW_TOL="${START_YAW_TOL}" \
START_HOLD_SEC="${START_HOLD_SEC}" START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC}" \
V_REF="${V_REF}" W_SLOSH="${W_SLOSH}" W_SMOOTH=1.0 W_ALPHA=1.0 W_DU_A=1.0 W_DU_VS=1.0 \
SPEED_SAFETY_ENABLE=true V_SAFE_MAX="${V_SAFE_MAX}" SPEED_SAFETY_TOLERANCE="${SPEED_SAFETY_TOLERANCE}" \
DELAY_PHASE_MODE="${DELAY_PHASE_MODE}" \
DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
IMU_SHADOW_ENABLE=true IMU_TOPIC="${IMU_TOPIC}" IMU_SUBSCRIBER_QUEUE_SIZE=10 \
CURRENT_OBSERVER_SOURCE=processed_imu OBSERVER_FALLBACK_POLICY=fail_closed \
OBSERVER_LATCH_FALLBACK=false LIQUID_NOWCAST_ENABLE=true \
LIQUID_NOWCAST_PUBLISH_COMPARISON=true \
LIQUID_NOWCAST_MAX_PREDICTION_SEC="${LIQUID_NOWCAST_MAX_PREDICTION_SEC}" \
LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC="${LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC}" \
LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC="${LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC}" \
LIQUID_NOWCAST_MAX_STATE_EXCITATION_SKEW_SEC="${LIQUID_NOWCAST_MAX_STATE_EXCITATION_SKEW_SEC}" \
LIQUID_NOWCAST_MAX_INTEGRATION_STEP_SEC="${LIQUID_NOWCAST_MAX_INTEGRATION_STEP_SEC}" \
STATE_TIMING_REQUIRE_COMMON_EPOCH=true STATE_TIMING_MAX_RAW_SKEW_SEC=0.080 \
STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=0.050 STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC=0.010 \
EXECUTION_CONTRACT_FAIL_CLOSED=true EXECUTION_CONTRACT_MAX_DELTA_V=0.0001 \
EXECUTION_CONTRACT_MAX_DELTA_OMEGA=0.0001 \
SHARED_LINEAR_ACCEL_LIMIT_ENABLE=false SHARED_ANGULAR_LIMIT_ENABLE=false \
RECORD_RGB=false RECORD_CAMERA=false RECORD_CAMERA_INFO=false \
RECORD_CAMERA_COMPRESSED=false RECORD_DEPTH=false RECORD_ONLINE_LIQUID=true \
RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false RECORD_STANDALONE_SLOSH=false \
RECORD_SCAN=true FORBID_IMAGE_STREAMS=true RECORD_ALL_EXISTING_TOPICS=false \
RECORD_TOPIC_INFO=true RECORD_MOCAP=true RECORD_MOCAP_PATH=false MOCAP_TRACKER="${MOCAP_TRACKER}" \
RGB_CALIBRATION_FILE="${RGB_CALIBRATION_FILE}" \
RGB_CALIBRATION_EXPECTED_SHA256="${RGB_CALIBRATION_SHA256}" \
ONLINE_LIQUID_MEASUREMENT_TOPIC="${ONLINE_LIQUID_MEASUREMENT_TOPIC}" \
ONLINE_LIQUID_PROTOCOL="${PROTOCOL_ID}" \
ONLINE_LIQUID_CALIBRATION_SHA256="${RGB_CALIBRATION_SHA256}" \
ONLINE_LIQUID_DETECTOR_SHA256="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')" \
ONLINE_LIQUID_NODE_SHA256="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')" \
ONLINE_LIQUID_MSG_SHA256="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')" \
ONLINE_LIQUID_CONFIG_SHA256="${online_config_sha}" \
RECORD_SEC="${RECORD_SEC}" MAX_RECORD_SEC="${RECORD_SEC}" \
BLOCK_SEGMENT_ID="SLOSH_DELAY_${SLOSH_TEST_ID}" SPLIT_BLOCK=false \
ORDER_POSITION="${SLOSH_TEST_ID#S}" ACQUISITION_RETRY=false SEND_ZERO_ON_EXIT=true \
OPERATOR_NOTE="C02 slosh delay diagnostic; I0 applied; old/candidate delay compensation not applied" \
bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"
python3 "${CHAIN_POSTFLIGHT}" "${BAG_PATH}" \
  --variant "${VARIANT}" --mocap-tracker "${MOCAP_TRACKER}" --imu-topic "${IMU_TOPIC}" \
  --path-file "${PATH_FILE}" --path-sha256 "${FROZEN_PATH_SHA256}" --report "${CHAIN_REPORT}"
python3 "${MATCHED_POSTFLIGHT}" "${BAG_PATH}" --variant "${VARIANT}" --report "${MATCHED_REPORT}"
python3 "${OBSERVER_POSTFLIGHT}" "${BAG_PATH}" --report "${OBSERVER_REPORT}" \
  --protocol "${PROTOCOL_ID}" --expected-solver-consumes-liquid \
  --expected-applied-method I0 \
  --max-prediction-sec "${LIQUID_NOWCAST_MAX_PREDICTION_SEC}" \
  --max-excitation-age-sec "${LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC}" \
  --max-future-skew-sec "${LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC}" \
  --expected-v-safe-max "${V_SAFE_MAX}" --speed-tolerance "${SPEED_SAFETY_TOLERANCE}"
python3 "${SAME_BAG_ANALYZER}" --bag "${BAG_PATH}" --out-dir "${ANALYSIS_DIR}" \
  --evidence-class development
python3 "${SUMMARIZER}" "${BAG_PATH}" --out-dir "${RUN_OUT_DIR}"

echo "[${SCRIPT_NAME}] PASS: ${SLOSH_TEST_ID}/${ATTEMPT} ${SLOSH_CONDITION}"
echo "[${SCRIPT_NAME}] bag: ${BAG_PATH}"
echo "[${SCRIPT_NAME}] chain postflight: ${CHAIN_REPORT}"
echo "[${SCRIPT_NAME}] matched postflight: ${MATCHED_REPORT}"
echo "[${SCRIPT_NAME}] observer postflight: ${OBSERVER_REPORT}"
echo "[${SCRIPT_NAME}] observer analysis: ${ANALYSIS_DIR}/summary.md"
