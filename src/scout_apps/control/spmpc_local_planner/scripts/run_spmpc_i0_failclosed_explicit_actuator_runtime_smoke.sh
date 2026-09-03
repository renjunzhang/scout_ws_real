#!/usr/bin/env bash
# One independent B_slosh smoke for the explicit-actuator mainline.
# The default profile preserves the runtime acceptance run; waccel03 changes
# only w_accel for the first command-smoothing check. Neither is an ABBA sample.

set -euo pipefail

SCRIPT_NAME=run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
RECORDER="${SCRIPT_DIR}/record_spmpc_full_rgb_bag.sh"
EXACT_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_i0_failclosed_fixed_abba_bag.py"
RUNTIME_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_explicit_actuator_runtime_smoke.py"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"
MAP_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_field_map.py"
MODEL_TEST="${SCRIPT_DIR}/tests/test_explicit_actuator_model.py"
SMOKE_TEST="${SCRIPT_DIR}/tests/test_explicit_actuator_runtime_smoke.py"
ACADOS_B0_JSON="${REPO_ROOT}/src/scout_apps/control/spmpc_local_planner/generated/acados/spmpc_b0/acados_ocp_spmpc_b0.json"
ACADOS_SLOSH_JSON="${REPO_ROOT}/src/scout_apps/control/spmpc_local_planner/generated/acados/spmpc_slosh/acados_ocp_spmpc_slosh.json"

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

require_dump_line() {
  local line="$1"
  grep -Fqx -- "${line}" <<< "${launch_dump}" \
    || fail "launch contract missing: ${line}"
}

VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RUNTIME_SMOKE="${CONFIRM_RUNTIME_SMOKE:-NO}"
CONFIRM_PATH_CLEAR="${CONFIRM_PATH_CLEAR:-NO}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
MIN_FREE_GIB="${MIN_FREE_GIB:-5}"
SMOKE_PROFILE="${SMOKE_PROFILE:-runtime_baseline}"

case "${SMOKE_PROFILE}" in
  runtime_baseline)
    PROTOCOL_ID=SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_RUNTIME_SMOKE_DEV_V1
    OUTPUT_SERIES=spmpc_i0_failclosed_explicit_actuator_runtime_smoke_v1
    RUN_LABEL_PREFIX=DEV_I0FC_EXPACT_RUNTIME_SMOKE_V1
    SMOKE_SCOPE=development_runtime_smoke_only
    SMOKE_PURPOSE="runtime acceptance only; NOT B0/Bslosh efficacy"
    OPERATOR_NOTE="one B_slosh explicit-actuator runtime smoke; no RGB efficacy claim"
    BLOCK_SEGMENT_ID=I0FC_EXPACT_RUNTIME_SMOKE_V1
    W_ACCEL=0.0
    ;;
  waccel03)
    PROTOCOL_ID=SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_WACCEL03_SMOKE_DEV_V1
    OUTPUT_SERIES=spmpc_i0_failclosed_explicit_actuator_waccel03_smoke_v1
    RUN_LABEL_PREFIX=DEV_I0FC_EXPACT_WACCEL03_SMOKE_V1
    SMOKE_SCOPE=development_waccel03_smoke_only
    SMOKE_PURPOSE="single-variable w_accel=0.3 command-smoothing smoke"
    OPERATOR_NOTE="one B_slosh explicit-actuator w_accel=0.3 smoke; RGB disabled"
    BLOCK_SEGMENT_ID=I0FC_EXPACT_WACCEL03_SMOKE_V1
    W_ACCEL=0.3
    ;;
  *)
    fail "unsupported SMOKE_PROFILE=${SMOKE_PROFILE}; use runtime_baseline or waccel03"
    ;;
esac

VARIANT=B_slosh
W_SLOSH=5.0
V_REF=0.20
V_SAFE_MAX=0.25
SPEED_SAFETY_TOLERANCE=0.0001
W_SMOOTH=0.1
W_ALPHA=0.1
W_DU_A=0.1
W_DU_VS=0.1
SLOSH_HEIGHT_MAX=0.001
ALPHA_MAX=1.2

EXECUTION_MODEL_MODE=explicit_actuator
ACTUATOR_LINEAR_DELAY_SEC=0.1666666665
ACTUATOR_ANGULAR_DELAY_SEC=0.3333333330
ACTUATOR_LINEAR_TAU_SEC=0.112
ACTUATOR_ANGULAR_TAU_SEC=0.119
ACTUATOR_LINEAR_GAIN=1.018
ACTUATOR_ANGULAR_GAIN=1.096
DELAY_PHASE_MODE=off
DELAY_PHASE_LINEAR_DELAY_SEC=0.15
DELAY_PHASE_ANGULAR_DELAY_SEC=0.22

MOCAP_TRACKER=Tracker0
IMU_TOPIC=/imu/data
ODOM_TOPIC=/odom
CMD_TOPIC=/cmd_vel
REF_TOPIC=/scout/global_path_fixed
COSTMAP_TOPIC=/map
BASE_FRAME=base_link
REFERENCE_TARGET_FRAME=map

FROZEN_PATH_FILE=/home/geist/fixed_paths/real/20260829_spmpc_mocap_execution_chain/candidates/mocap_compact_s_C02.json
FROZEN_PATH_SHA256=1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164
FROZEN_MAP_FILE=/home/geist/scout_maps/real/20260829_mocap_exec/map_carto_20260829_mocap_exec_v1.pbstream
FROZEN_MAP_SHA256=34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595

RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_${OUTPUT_SERIES}/H0}"
RUN_LABEL="${RUN_LABEL_PREFIX}_${STAMP}_Bslosh"
NAME="${RUN_LABEL}"
BAG_PATH="${RUN_OUT_DIR}/${NAME}.bag"
EXACT_REPORT="${RUN_OUT_DIR}/${NAME}_i0_explicit_actuator_contract_postflight.json"
RUNTIME_REPORT="${RUN_OUT_DIR}/${NAME}_runtime_postflight.json"
PASS_MARKER="${RUN_OUT_DIR}/${NAME}_runtime_smoke_pass.env"
PREREG_FILE="${RUN_OUT_DIR}/${NAME}_runtime_smoke_prereg.env"

required_files=(
  "${RUNNER}"
  "${RECORDER}"
  "${EXACT_POSTFLIGHT}"
  "${RUNTIME_POSTFLIGHT}"
  "${PATH_VALIDATOR}"
  "${MAP_VALIDATOR}"
  "${MODEL_TEST}"
  "${SMOKE_TEST}"
  "${ACADOS_B0_JSON}"
  "${ACADOS_SLOSH_JSON}"
  "${FROZEN_PATH_FILE}"
  "${FROZEN_MAP_FILE}"
)
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe run label"
[[ "$(sha256sum "${FROZEN_PATH_FILE}" | awk '{print $1}')" == "${FROZEN_PATH_SHA256}" ]] \
  || fail "frozen path SHA-256 mismatch"
[[ "$(sha256sum "${FROZEN_MAP_FILE}" | awk '{print $1}')" == "${FROZEN_MAP_SHA256}" ]] \
  || fail "frozen map SHA-256 mismatch"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

python3 "${PATH_VALIDATOR}" "${FROZEN_PATH_FILE}" \
  --expected-sha256 "${FROZEN_PATH_SHA256}" >/dev/null
python3 "${MAP_VALIDATOR}" "${FROZEN_MAP_FILE}" \
  --expected-resolution 0.02 \
  --expected-pbstream-sha256 "${FROZEN_MAP_SHA256}" >/dev/null

python3 - "${ACADOS_B0_JSON}" "${ACADOS_SLOSH_JSON}" <<'PY'
import json
import sys
from pathlib import Path

for raw_path in sys.argv[1:]:
    path = Path(raw_path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    horizon = int(payload.get("dims", {}).get("N", -1))
    cond_n = int(payload.get("solver_options", {}).get("qp_solver_cond_N", -1))
    if horizon != 60 or cond_n != 10:
        raise SystemExit(
            "generated solver contract mismatch: {} N={} qp_solver_cond_N={}".format(
                path, horizon, cond_n
            )
        )
PY

launch_dump="$(roslaunch --dump-params \
  spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:="${VARIANT}" solver_backend:=continuous_mpcc_acados \
  reference_path_topic:="${REF_TOPIC}" cmd_vel_topic:="${CMD_TOPIC}" \
  costmap_topic:="${COSTMAP_TOPIC}" reference_target_frame:="${REFERENCE_TARGET_FRAME}" \
  odom_subscriber_queue_size:=10 \
  execution_model_mode:="${EXECUTION_MODEL_MODE}" \
  execution_model_linear_delay_sec:="${ACTUATOR_LINEAR_DELAY_SEC}" \
  execution_model_angular_delay_sec:="${ACTUATOR_ANGULAR_DELAY_SEC}" \
  execution_model_linear_tau_sec:="${ACTUATOR_LINEAR_TAU_SEC}" \
  execution_model_angular_tau_sec:="${ACTUATOR_ANGULAR_TAU_SEC}" \
  execution_model_linear_gain:="${ACTUATOR_LINEAR_GAIN}" \
  execution_model_angular_gain:="${ACTUATOR_ANGULAR_GAIN}" \
  execution_model_cmd_timeout_sec:=0.5 \
  execution_model_max_prefix_prediction_sec:=0.20 \
  execution_model_max_integration_step_sec:=0.01 \
  execution_model_require_complete_history:=true \
  delay_phase_mode:="${DELAY_PHASE_MODE}" \
  delay_phase_linear_delay_sec:="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
  delay_phase_angular_delay_sec:="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
  imu_shadow_enable:=true imu_topic:="${IMU_TOPIC}" imu_subscriber_queue_size:=10 \
  observer_source:=processed_imu observer_fallback_policy:=fail_closed \
  observer_latch_fallback:=false \
  observer_max_imu_state_age_sec:=0.10 observer_max_odom_state_age_sec:=0.50 \
  observer_max_future_skew_sec:=0.005 \
  liquid_nowcast_enable:=true liquid_nowcast_publish_comparison:=true \
  liquid_nowcast_max_prediction_sec:=0.050 \
  liquid_nowcast_max_excitation_age_sec:=0.060 \
  liquid_nowcast_max_future_skew_sec:=0.005 \
  liquid_nowcast_max_state_excitation_skew_sec:=0.001 \
  liquid_nowcast_max_integration_step_sec:=0.020 \
  state_timing_require_common_epoch:=true \
  state_timing_max_raw_skew_sec:=0.080 \
  state_timing_max_interpolation_gap_sec:=0.050 \
  state_timing_max_robot_extrapolation_sec:=0.010 \
  execution_contract_fail_closed_on_post_limit_change:=true \
  execution_contract_max_post_limit_delta_v:=0.0001 \
  execution_contract_max_post_limit_delta_omega:=0.0001 \
  shared_linear_accel_limit_enable:=false shared_angular_limit_enable:=false \
  speed_safety_enable:=true v_safe_max:="${V_SAFE_MAX}" \
  speed_safety_tolerance:="${SPEED_SAFETY_TOLERANCE}" \
  v_ref:="${V_REF}" w_slosh:="${W_SLOSH}" \
  w_accel:="${W_ACCEL}" \
  w_smooth:="${W_SMOOTH}" w_alpha:="${W_ALPHA}" \
  w_du_a:="${W_DU_A}" w_du_vs:="${W_DU_VS}" \
  slosh_height_max:="${SLOSH_HEIGHT_MAX}" alpha_max:="${ALPHA_MAX}")" \
  || fail "could not dump B_slosh launch parameters"

expected_launch_lines=(
  "/spmpc_local_planner/planner_variant: B_slosh"
  "/spmpc_local_planner/solver_backend: continuous_mpcc_acados"
  "/spmpc_local_planner/variants/B_slosh/slosh_enable: true"
  "/spmpc_local_planner/variants/B_slosh/w_slosh: 5.0"
  "/spmpc_local_planner/variants/B_slosh/v_ref: 0.2"
  "/spmpc_local_planner/variants/B_slosh/w_accel: ${W_ACCEL}"
  "/spmpc_local_planner/variants/B_slosh/w_smooth: 0.1"
  "/spmpc_local_planner/variants/B_slosh/w_alpha: 0.1"
  "/spmpc_local_planner/variants/B_slosh/w_du_a: 0.1"
  "/spmpc_local_planner/variants/B_slosh/w_du_vs: 0.1"
  "/spmpc_local_planner/slosh/slosh_height_max: 0.001"
  "/spmpc_local_planner/odom/subscriber_queue_size: 10"
  "/spmpc_local_planner/slosh_observer/source: processed_imu"
  "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
  "/spmpc_local_planner/slosh_observer/latch_fallback: false"
  "/spmpc_local_planner/state_timing/require_common_epoch: true"
  "/spmpc_local_planner/state_timing/max_interpolation_gap_sec: 0.05"
  "/spmpc_local_planner/execution_model/mode: explicit_actuator"
  "/spmpc_local_planner/execution_model/linear_delay_sec: 0.1666666665"
  "/spmpc_local_planner/execution_model/angular_delay_sec: 0.333333333"
  "/spmpc_local_planner/execution_model/linear_tau_sec: 0.112"
  "/spmpc_local_planner/execution_model/angular_tau_sec: 0.119"
  "/spmpc_local_planner/delay_phase/mode: 'off'"
  "/spmpc_local_planner/platform/shared_constraints/linear_accel_limit_enable: false"
  "/spmpc_local_planner/platform/shared_constraints/angular_limit_enable: false"
  "/spmpc_local_planner/execution_contract/fail_closed_on_post_limit_change: true"
  "/spmpc_local_planner/speed_safety/enable: true"
  "/spmpc_local_planner/speed_safety/v_safe_max: 0.25"
)
for expected_line in "${expected_launch_lines[@]}"; do
  require_dump_line "${expected_line}"
done

python3 "${MODEL_TEST}"
python3 "${SMOKE_TEST}"
bash -n "${BASH_SOURCE[0]}"

echo "================ explicit actuator runtime smoke ================"
echo "  profile        = ${SMOKE_PROFILE}"
echo "  protocol       = ${PROTOCOL_ID}"
echo "  purpose        = ${SMOKE_PURPOSE}"
echo "  condition      = B_slosh; w_slosh=5.0; w_accel=${W_ACCEL}; one bag only"
echo "  observer       = processed-IMU I0; fail_closed; common_epoch=true"
echo "  execution      = explicit_actuator; legacy delay=off"
echo "  solver runtime = N=60; qp_solver_cond_N=10; odom private queue=10"
echo "  frozen weights = w_accel=${W_ACCEL}; w_smooth/w_alpha/w_du_a/w_du_vs=0.1"
echo "  speed          = v_ref=0.20; hard v_safe=0.25 m/s"
echo "  RGB            = disabled; no efficacy conclusion from this bag"
echo "  output         = ${BAG_PATH}"
echo "  acceptance     = epoch/solver/fault-zero=0; odom gaps>50ms=0; callback P95<30ms; consecutive overruns<=1"
echo "================================================================="

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS; motion NOT started"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] \
  || fail "set ARM_MOTION=YES only after E-stop and operator check"
[[ "${CONFIRM_RUNTIME_SMOKE}" == "YES" ]] \
  || fail "set CONFIRM_RUNTIME_SMOKE=YES to confirm this is the one-bag runtime test"
[[ "${CONFIRM_PATH_CLEAR}" == "YES" ]] \
  || fail "set CONFIRM_PATH_CLEAR=YES after returning to the frozen-path start and clearing the path"

runtime_paths=(
  src/scout_apps/control/spmpc_local_planner/config
  src/scout_apps/control/spmpc_local_planner/generated
  src/scout_apps/control/spmpc_local_planner/include
  src/scout_apps/control/spmpc_local_planner/launch
  src/scout_apps/control/spmpc_local_planner/msg
  src/scout_apps/control/spmpc_local_planner/src
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_i0_failclosed_fixed_abba_bag.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_explicit_actuator_runtime_smoke.py
  src/scout_apps/control/spmpc_local_planner/scripts/tests/test_explicit_actuator_model.py
  src/scout_apps/control/spmpc_local_planner/scripts/tests/test_explicit_actuator_runtime_smoke.py
)
dirty_runtime="$(git -C "${REPO_ROOT}" status --porcelain --untracked-files=normal -- "${runtime_paths[@]}")"
[[ -z "${dirty_runtime}" ]] \
  || fail "runtime/evidence paths are dirty; commit and rebuild before motion"

for output in "${BAG_PATH}" "${BAG_PATH}.active" "${EXACT_REPORT}" \
  "${RUNTIME_REPORT}" "${PASS_MARKER}" "${PREREG_FILE}"; do
  [[ ! -e "${output}" ]] || fail "preserve existing output: ${output}"
done

available_kib="$(df -Pk /home/geist/slosh_bags/real | awk 'NR == 2 {print $4}')"
required_kib=$((MIN_FREE_GIB * 1024 * 1024))
(( available_kib >= required_kib )) || fail "require at least ${MIN_FREE_GIB} GiB free"

for topic in /map /scan_front "${ODOM_TOPIC}" "${IMU_TOPIC}"; do
  timeout 5s rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1 \
    || fail "runtime topic unavailable: ${topic}"
done
raw_mocap_topic="/vrpn_client_node/${MOCAP_TRACKER}/pose"
timeout 5s rostopic echo -n 1 "${raw_mocap_topic}" >/dev/null 2>&1 \
  || fail "no NOKOV pose: ${raw_mocap_topic}"
mocap_status="$(timeout 5s rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
grep -q "OK.*tracker=${MOCAP_TRACKER}" <<< "${mocap_status}" \
  || fail "/mocap/status is not OK for ${MOCAP_TRACKER}"
runtime_map_file="$(rosparam get /cartographer_node/frozen_map_file 2>/dev/null || true)"
runtime_map_sha="$(rosparam get /cartographer_node/frozen_map_expected_sha256 2>/dev/null || true)"
[[ "$(readlink -f "${runtime_map_file}")" == "$(readlink -f "${FROZEN_MAP_FILE}")" ]] \
  || fail "runtime Cartographer map differs from the frozen map"
[[ "${runtime_map_sha,,}" == "${FROZEN_MAP_SHA256}" ]] \
  || fail "runtime Cartographer map SHA-256 differs"
published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
grep -Fxq -- "${CMD_TOPIC}" <<< "${published_topics}" \
  && fail "${CMD_TOPIC} already has a publisher"

current_git_revision="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
mkdir -p "${RUN_OUT_DIR}"
{
  echo "protocol=${PROTOCOL_ID}"
  echo "profile=${SMOKE_PROFILE}"
  echo "scope=${SMOKE_SCOPE}"
  echo "condition=B_slosh"
  echo "w_slosh=${W_SLOSH}"
  echo "w_accel=${W_ACCEL}"
  echo "v_ref=${V_REF}"
  echo "v_safe_max=${V_SAFE_MAX}"
  echo "observer=processed_imu"
  echo "fallback=fail_closed"
  echo "common_epoch=true"
  echo "execution_model=explicit_actuator"
  echo "legacy_delay=off"
  echo "qp_solver_cond_N=10"
  echo "odom_subscriber_queue_size=10"
  echo "max_interpolation_gap_sec=0.050"
  echo "path_sha256=${FROZEN_PATH_SHA256}"
  echo "map_sha256=${FROZEN_MAP_SHA256}"
  echo "git_revision=${current_git_revision}"
  echo "rgb_efficacy_claim_forbidden=true"
} > "${PREREG_FILE}"

env \
  MATRIX_PRESET= PILOT_METHOD= VARIANT="${VARIANT}" ALG="${VARIANT}" \
  DATE="${DATE}" STAMP="${STAMP}" PILOT_MODE=true \
  PILOT_CONDITION="${PROTOCOL_ID}" PILOT_RECORD_RGB=false \
  PILOT_RECORD_ONLINE_LIQUID=false \
  RUN_LABEL="${RUN_LABEL}" NAME="${NAME}" RUN_OUT_DIR="${RUN_OUT_DIR}" \
  PATH_SOURCE_MODE=replay PATH_FILE="${FROZEN_PATH_FILE}" \
  PATH_EXPECTED_SHA256="${FROZEN_PATH_SHA256}" REQUIRE_PATH_HASH=true \
  START_POS_TOL=0.08 START_YAW_TOL=0.15 START_HOLD_SEC=0.5 \
  START_GATE_TIMEOUT_SEC=120 PATH_PUBLISH_RATE=2.0 \
  SOLVER_BACKEND=continuous_mpcc_acados CMD_TOPIC="${CMD_TOPIC}" \
  REF_TOPIC="${REF_TOPIC}" COSTMAP_TOPIC="${COSTMAP_TOPIC}" \
  REFERENCE_TARGET_FRAME="${REFERENCE_TARGET_FRAME}" BASE_FRAME="${BASE_FRAME}" \
  V_REF="${V_REF}" W_SLOSH="${W_SLOSH}" W_SMOOTH="${W_SMOOTH}" \
  W_ACCEL="${W_ACCEL}" \
  W_ALPHA="${W_ALPHA}" W_DU_A="${W_DU_A}" W_DU_VS="${W_DU_VS}" \
  SLOSH_HEIGHT_MAX="${SLOSH_HEIGHT_MAX}" ALPHA_MAX="${ALPHA_MAX}" \
  EXECUTION_MODEL_MODE="${EXECUTION_MODEL_MODE}" \
  EXECUTION_MODEL_LINEAR_DELAY_SEC="${ACTUATOR_LINEAR_DELAY_SEC}" \
  EXECUTION_MODEL_ANGULAR_DELAY_SEC="${ACTUATOR_ANGULAR_DELAY_SEC}" \
  EXECUTION_MODEL_LINEAR_TAU_SEC="${ACTUATOR_LINEAR_TAU_SEC}" \
  EXECUTION_MODEL_ANGULAR_TAU_SEC="${ACTUATOR_ANGULAR_TAU_SEC}" \
  EXECUTION_MODEL_LINEAR_GAIN="${ACTUATOR_LINEAR_GAIN}" \
  EXECUTION_MODEL_ANGULAR_GAIN="${ACTUATOR_ANGULAR_GAIN}" \
  EXECUTION_MODEL_CMD_TIMEOUT_SEC=0.5 \
  EXECUTION_MODEL_MAX_PREFIX_PREDICTION_SEC=0.20 \
  EXECUTION_MODEL_MAX_INTEGRATION_STEP_SEC=0.01 \
  EXECUTION_MODEL_REQUIRE_COMPLETE_HISTORY=true \
  DELAY_PHASE_MODE="${DELAY_PHASE_MODE}" \
  DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
  DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
  IMU_SHADOW_ENABLE=true IMU_TOPIC="${IMU_TOPIC}" IMU_SUBSCRIBER_QUEUE_SIZE=10 \
  IMU_SHADOW_READY_TIMEOUT_SEC=20 \
  IMU_SHADOW_READY_TOPIC=/spmpc/debug/slosh_observer_imu \
  CURRENT_OBSERVER_SOURCE=processed_imu OBSERVER_FALLBACK_POLICY=fail_closed \
  OBSERVER_LATCH_FALLBACK=false OBSERVER_MAX_IMU_STATE_AGE_SEC=0.10 \
  OBSERVER_MAX_ODOM_STATE_AGE_SEC=0.50 OBSERVER_MAX_FUTURE_SKEW_SEC=0.005 \
  OBSERVER_SELECTION_TOPIC=/spmpc/debug/slosh_observer_selection \
  LIQUID_NOWCAST_ENABLE=true LIQUID_NOWCAST_PUBLISH_COMPARISON=true \
  LIQUID_NOWCAST_MAX_PREDICTION_SEC=0.050 \
  LIQUID_NOWCAST_MAX_EXCITATION_AGE_SEC=0.060 \
  LIQUID_NOWCAST_MAX_FUTURE_SKEW_SEC=0.005 \
  LIQUID_NOWCAST_MAX_STATE_EXCITATION_SKEW_SEC=0.001 \
  LIQUID_NOWCAST_MAX_INTEGRATION_STEP_SEC=0.020 \
  STATE_TIMING_REQUIRE_COMMON_EPOCH=true STATE_TIMING_MAX_RAW_SKEW_SEC=0.080 \
  STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=0.050 \
  STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC=0.010 \
  EXECUTION_CONTRACT_FAIL_CLOSED=true \
  EXECUTION_CONTRACT_MAX_DELTA_V=0.0001 \
  EXECUTION_CONTRACT_MAX_DELTA_OMEGA=0.0001 \
  SHARED_LINEAR_ACCEL_LIMIT_ENABLE=false SHARED_LINEAR_ACCEL_MAX=0.6 \
  SHARED_ANGULAR_LIMIT_ENABLE=false SHARED_ANGULAR_RATE_MAX=1.2 \
  SHARED_ANGULAR_ACCEL_MAX=1.2 SPEED_SAFETY_ENABLE=true \
  V_SAFE_MAX="${V_SAFE_MAX}" SPEED_SAFETY_TOLERANCE="${SPEED_SAFETY_TOLERANCE}" \
  RECORDER_SCRIPT="${RECORDER}" RECORDER_STARTUP_SEC=8 \
  RECORDER_ACTIVE_TIMEOUT_SEC=15 PLANNER_STARTUP_SEC=2 \
  PATH_GENERATOR_STARTUP_SEC=2 EXPECTED_RUNTIME_VARIANT="${VARIANT}" \
  RUNTIME_VARIANT_TIMEOUT_SEC=5 SPLIT_BLOCK=false ACQUISITION_RETRY=false \
  RETRY_REASON_FILE= BLOCK_SEGMENT_ID="${BLOCK_SEGMENT_ID}" \
  ORDER_POSITION=01 RECORD_RGB=false RECORD_CAMERA=false \
  RECORD_CAMERA_INFO=false RECORD_CAMERA_COMPRESSED=false RECORD_DEPTH=false \
  RECORD_ONLINE_LIQUID=false RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false \
  RECORD_STANDALONE_SLOSH=false RECORD_SCAN=true FORBID_IMAGE_STREAMS=true \
  RECORD_ALL_EXISTING_TOPICS=false RECORD_TOPIC_INFO=true RECORD_MOCAP=true \
  RECORD_MOCAP_PATH=false MOCAP_TRACKER="${MOCAP_TRACKER}" \
  RECORD_SEC=70 MAX_RECORD_SEC=70 SEND_ZERO_ON_EXIT=true \
  OPERATOR_NOTE="${OPERATOR_NOTE}" \
  bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"

exact_rc=0
python3 "${EXACT_POSTFLIGHT}" "${BAG_PATH}" \
  --condition Bslosh --report "${EXACT_REPORT}" --protocol "${PROTOCOL_ID}" \
  --report-schema spmpc_explicit_actuator_runtime_smoke_contract_postflight_v1 \
  --expected-variant B_slosh \
  --expected-slosh-cost-horizon-steps -1 \
  --expected-slosh-cost-tail-discount 1.0 \
  --expected-slosh-eta-dot-ratio 0.3 \
  --expected-robot-horizon-steps 60 \
  --expected-dt-sec 0.0333333333333333 \
  --expected-control-frequency-hz 30.0 \
  --expected-v-ref "${V_REF}" --expected-v-safe-max "${V_SAFE_MAX}" \
  --minimum-application-fraction 1.0 --expected-delay-mode-code 0 \
  --require-legacy-delay-application false \
  --expected-execution-model-code 1 --expected-state-width 27 \
  --minimum-solver-schema-version 3 \
  --expected-config "w_accel=${W_ACCEL}" \
  --expected-config "w_smooth=${W_SMOOTH}" \
  --expected-config "w_alpha=${W_ALPHA}" \
  --expected-config "w_du_a=${W_DU_A}" \
  --expected-config "w_du_vs=${W_DU_VS}" \
  --expected-config "slosh_height_max=${SLOSH_HEIGHT_MAX}" \
  --expected-config "alpha_max=${ALPHA_MAX}" \
  --expected-config "execution_model_mode_code=1" \
  --expected-config "actuator_linear_delay_sec=${ACTUATOR_LINEAR_DELAY_SEC}" \
  --expected-config "actuator_angular_delay_sec=${ACTUATOR_ANGULAR_DELAY_SEC}" \
  --expected-config "actuator_linear_tau_sec=${ACTUATOR_LINEAR_TAU_SEC}" \
  --expected-config "actuator_angular_tau_sec=${ACTUATOR_ANGULAR_TAU_SEC}" \
  --expected-config "actuator_linear_gain=${ACTUATOR_LINEAR_GAIN}" \
  --expected-config "actuator_angular_gain=${ACTUATOR_ANGULAR_GAIN}" \
  --expected-config "actuator_linear_delay_steps=5" \
  --expected-config "actuator_angular_delay_steps=10" || exact_rc=$?

runtime_rc=0
python3 "${RUNTIME_POSTFLIGHT}" "${BAG_PATH}" \
  --report "${RUNTIME_REPORT}" --protocol "${PROTOCOL_ID}" \
  --max-planner-odom-gap-ms 50.0 \
  --max-control-callback-p95-ms 30.0 \
  --callback-period-ms 33.3333333333333 \
  --max-consecutive-callback-overrun 1 || runtime_rc=$?

if (( exact_rc != 0 || runtime_rc != 0 )); then
  fail "postflight failed: contract_rc=${exact_rc}, runtime_rc=${runtime_rc}; preserve the bag and reports for diagnosis"
fi

printf '%s\n' \
  "status=PASS" \
  "protocol=${PROTOCOL_ID}" \
  "profile=${SMOKE_PROFILE}" \
  "condition=Bslosh" \
  "bag=${BAG_PATH}" \
  "git_revision=${current_git_revision}" \
  "exact_postflight_sha256=$(sha256sum "${EXACT_REPORT}" | awk '{print $1}')" \
  "runtime_postflight_sha256=$(sha256sum "${RUNTIME_REPORT}" | awk '{print $1}')" \
  "completed_at=$(date --iso-8601=seconds)" > "${PASS_MARKER}"

echo "[${SCRIPT_NAME}] PASS: one B_slosh runtime smoke completed"
echo "[${SCRIPT_NAME}] bag=${BAG_PATH}"
echo "[${SCRIPT_NAME}] runtime report=${RUNTIME_REPORT}"
