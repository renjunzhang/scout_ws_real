#!/usr/bin/env bash
# Development-only replay of the useful 2026-07-05 control core on the frozen
# C02 path.  One invocation records exactly one ABBA row:
#   01 B_smooth -> 02 B_ours -> 03 B_ours -> 04 B_smooth
# ROW=B0 records one supplemental B0 reference outside the ABBA decision.
# The dedicated B0/B_slosh wrapper selects the second supported pair profile:
#   01 B0 -> 02 B_slosh -> 03 B_slosh -> 04 B0
#
# "O0+L22" means the nominal liquid observer is the legacy odom observer and
# fixed_closed_loop rolls robot+liquid state through the 0.15/0.22 s command
# history.  The current common-epoch and speed-safety guards remain enabled.
# Online RGB is converted to a stamped scalar in real time; no image stream is
# written to the bag.  NOKOV, odom, IMU, planner and delay diagnostics are kept.

set -euo pipefail

SCRIPT_NAME="${SCRIPT_NAME_OVERRIDE:-run_spmpc_o0_l22_bsmooth_bours_trial}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"

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

[[ -r /opt/ros/noetic/setup.bash ]] || fail "missing /opt/ros/noetic/setup.bash"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]] || \
  fail "missing ${REPO_ROOT}/devel/setup.bash; build the workspace first"
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"
MAP_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_field_map.py"
CHAIN_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_mocap_execution_chain_bag.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"
ONLINE_LIQUID_LAUNCH="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/launch/online_liquid_height.launch"
ONLINE_LIQUID_NODE="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py"
ONLINE_LIQUID_DETECTOR="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py"
ONLINE_LIQUID_MSG="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg"

ROW="${ROW:-${O0L22_ROW:-}}"
PAIR_PROFILE="${PAIR_PROFILE:-bsmooth_bours}"
ATTEMPT="${ATTEMPT:-01}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"

W_SMOOTH=1.0
W_ALPHA=1.0
W_DU_A=1.0
W_DU_VS=1.0
EXPECTED_W_CONTROL=0.3
EXPECTED_SMOOTH_PRIORITY=true
SUPPLEMENTAL_B0=false

case "${PAIR_PROFILE}:${ROW}" in
  bsmooth_bours:B0)
    VARIANT=B0
    CONDITION=B0
    W_SLOSH=0.0
    SLOSH_ENABLE=false
    W_SMOOTH=0.1
    W_ALPHA=0.1
    W_DU_A=0.1
    W_DU_VS=0.1
    EXPECTED_W_CONTROL=0.1
    EXPECTED_SMOOTH_PRIORITY=false
    BLOCK=B0
    POSITION=00
    SUPPLEMENTAL_B0=true
    PAIR_ORDER_TEXT="supplemental B0"
    ;;
  bsmooth_bours:01)
    VARIANT=B_smooth
    CONDITION=Bsmooth
    W_SLOSH=0.0
    SLOSH_ENABLE=false
    BLOCK=01
    POSITION=01
    PAIR_ORDER_TEXT="B_smooth -> B_ours -> B_ours -> B_smooth"
    ;;
  bsmooth_bours:02)
    VARIANT=B_ours
    CONDITION=Bours
    W_SLOSH=5.0
    SLOSH_ENABLE=true
    BLOCK=01
    POSITION=02
    PAIR_ORDER_TEXT="B_smooth -> B_ours -> B_ours -> B_smooth"
    ;;
  bsmooth_bours:03)
    VARIANT=B_ours
    CONDITION=Bours
    W_SLOSH=5.0
    SLOSH_ENABLE=true
    BLOCK=02
    POSITION=01
    PAIR_ORDER_TEXT="B_smooth -> B_ours -> B_ours -> B_smooth"
    ;;
  bsmooth_bours:04)
    VARIANT=B_smooth
    CONDITION=Bsmooth
    W_SLOSH=0.0
    SLOSH_ENABLE=false
    BLOCK=02
    POSITION=02
    PAIR_ORDER_TEXT="B_smooth -> B_ours -> B_ours -> B_smooth"
    ;;
  b0_bslosh:01|b0_bslosh:04)
    VARIANT=B0
    CONDITION=B0
    W_SLOSH=0.0
    SLOSH_ENABLE=false
    W_SMOOTH=0.1
    W_ALPHA=0.1
    W_DU_A=0.1
    W_DU_VS=0.1
    EXPECTED_W_CONTROL=0.1
    EXPECTED_SMOOTH_PRIORITY=false
    PAIR_ORDER_TEXT="B0 -> B_slosh -> B_slosh -> B0"
    if [[ "${ROW}" == "01" ]]; then
      BLOCK=01
      POSITION=01
    else
      BLOCK=02
      POSITION=02
    fi
    ;;
  b0_bslosh:02|b0_bslosh:03)
    VARIANT=B_slosh
    CONDITION=Bslosh
    W_SLOSH=5.0
    SLOSH_ENABLE=true
    W_SMOOTH=0.1
    W_ALPHA=0.1
    W_DU_A=0.1
    W_DU_VS=0.1
    EXPECTED_W_CONTROL=0.1
    EXPECTED_SMOOTH_PRIORITY=false
    PAIR_ORDER_TEXT="B0 -> B_slosh -> B_slosh -> B0"
    if [[ "${ROW}" == "02" ]]; then
      BLOCK=01
      POSITION=02
    else
      BLOCK=02
      POSITION=01
    fi
    ;;
  bsmooth_bours:*)
    fail "set ROW=B0|01|02|03|04 (ABBA order: B_smooth, B_ours, B_ours, B_smooth)"
    ;;
  b0_bslosh:*)
    fail "set ROW=01|02|03|04 (ABBA order: B0, B_slosh, B_slosh, B0)"
    ;;
  *) fail "unsupported PAIR_PROFILE=${PAIR_PROFILE}; use bsmooth_bours or b0_bslosh" ;;
esac
[[ "${ATTEMPT}" =~ ^[0-9][0-9]$ ]] || fail "ATTEMPT must be two digits, for example 01"

FROZEN_PATH_FILE="/home/geist/fixed_paths/real/20260829_spmpc_mocap_execution_chain/candidates/mocap_compact_s_C02.json"
FROZEN_PATH_SHA256="1464ef37857bcb899d8b0e4867ff63ea06f017e1b871bed80e077f450be14164"
FROZEN_MAP_FILE="/home/geist/scout_maps/real/20260829_mocap_exec/map_carto_20260829_mocap_exec_v1.pbstream"
FROZEN_MAP_SHA256="34e45fd8205a766dbc6e3dcea667c5a0a618e26b331d48351c25645e31a19595"
FIELD_MAP_RESOLUTION=0.02
RGB_CALIBRATION_FILE="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml"
RGB_CALIBRATION_SHA256="7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01"

if ${SUPPLEMENTAL_B0}; then
  PROTOCOL_ID="SMPCC_C02_B0_FIXED_SUPPLEMENT_DEV_V1"
  default_run_out_dir="/home/geist/slosh_bags/real/${DATE}_spmpc_b0_fixed_c02_supplement"
  default_run_label="DEV_B0_FIXED_C02_a${ATTEMPT}"
  BLOCK_SEGMENT_ID="C02_B0_FIXED_SUPPLEMENT"
  OPERATOR_NOTE="Supplemental B0 on frozen C02; fixed_closed_loop predicts robot state only; liquid state is diagnostic and not consumed"
elif [[ "${PAIR_PROFILE}" == "b0_bslosh" ]]; then
  PROTOCOL_ID="SMPCC_C02_O0_L22_B0_BSLOSH_ABBA_DEV_V1"
  default_run_out_dir="/home/geist/slosh_bags/real/${DATE}_spmpc_o0_l22_b0_bslosh_c02"
  default_run_label="DEV_O0L22_B0BS_C02_R${ROW}_${CONDITION}_a${ATTEMPT}"
  BLOCK_SEGMENT_ID="O0L22_B0BS_C02_B${BLOCK}"
  OPERATOR_NOTE="Fair B0 versus B_slosh ABBA on frozen C02; O0 plus legacy L22; only slosh enable and cost differ"
else
  PROTOCOL_ID="SMPCC_C02_O0_L22_BSMOOTH_BOURS_ABBA_DEV_V1"
  default_run_out_dir="/home/geist/slosh_bags/real/${DATE}_spmpc_o0_l22_bsmooth_bours_c02"
  default_run_label="DEV_O0L22_C02_R${ROW}_${CONDITION}_a${ATTEMPT}"
  BLOCK_SEGMENT_ID="O0L22_C02_B${BLOCK}"
  OPERATOR_NOTE="0705-core O0 plus legacy L22 on frozen C02; current common-epoch and speed guard enabled"
fi
RUN_OUT_DIR="${RUN_OUT_DIR:-${default_run_out_dir}}"
RUN_LABEL="${RUN_LABEL:-${default_run_label}}"
NAME="${NAME:-${RUN_LABEL}}"
BAG_PATH="${RUN_OUT_DIR}/${NAME}.bag"
CHAIN_REPORT="${RUN_OUT_DIR}/${NAME}_mocap_chain_postflight.json"
SUMMARY_JSON="${RUN_OUT_DIR}/${NAME}_summary.json"
PROTOCOL_FILE="${RUN_OUT_DIR}/${PROTOCOL_ID}_protocol.env"
ORDER_FILE="${RUN_OUT_DIR}/${PROTOCOL_ID}_order.csv"
ROW_BINDING="${RUN_OUT_DIR}/${NAME}_row_binding.env"

MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
RGB_IMAGE_TOPIC="${RGB_IMAGE_TOPIC:-/camera/color/image_raw}"
RGB_CAMERA_INFO_TOPIC="${RGB_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
ONLINE_LIQUID_MEASUREMENT_TOPIC="${ONLINE_LIQUID_MEASUREMENT_TOPIC:-/liquid/measurement}"
RECORD_SEC="${RECORD_SEC:-90}"
MIN_FREE_GIB="${MIN_FREE_GIB:-5}"

for integer_setting in RECORD_SEC MIN_FREE_GIB; do
  [[ "${!integer_setting}" =~ ^[1-9][0-9]*$ ]] || \
    fail "${integer_setting} must be a positive integer, got '${!integer_setting}'"
done
(( RECORD_SEC >= 60 && RECORD_SEC <= 120 )) || fail "RECORD_SEC must be in [60,120]"
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe RUN_LABEL"

required_files=(
  "${RUNNER}" "${PATH_VALIDATOR}" "${MAP_VALIDATOR}" "${CHAIN_POSTFLIGHT}"
  "${SUMMARIZER}" "${CAMERA_PREP}" "${ONLINE_LIQUID_LAUNCH}"
  "${ONLINE_LIQUID_NODE}" "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}"
  "${FROZEN_PATH_FILE}" "${FROZEN_MAP_FILE}" "${RGB_CALIBRATION_FILE}"
)
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ "$(sha256sum "${FROZEN_PATH_FILE}" | awk '{print $1}')" == "${FROZEN_PATH_SHA256}" ]] || \
  fail "frozen C02 path SHA-256 mismatch"
[[ "$(sha256sum "${FROZEN_MAP_FILE}" | awk '{print $1}')" == "${FROZEN_MAP_SHA256}" ]] || \
  fail "frozen map SHA-256 mismatch"
[[ "$(sha256sum "${RGB_CALIBRATION_FILE}" | awk '{print $1}')" == "${RGB_CALIBRATION_SHA256}" ]] || \
  fail "RGB calibration SHA-256 mismatch"

python3 "${PATH_VALIDATOR}" "${FROZEN_PATH_FILE}" \
  --expected-sha256 "${FROZEN_PATH_SHA256}" >/dev/null
python3 "${MAP_VALIDATOR}" "${FROZEN_MAP_FILE}" \
  --expected-resolution "${FIELD_MAP_RESOLUTION}" \
  --expected-pbstream-sha256 "${FROZEN_MAP_SHA256}" >/dev/null
VALIDATE_ONLY=true bash "${CAMERA_PREP}" >/dev/null

ONLINE_LIQUID_PROCESS_EVERY=1
ONLINE_LIQUID_ZERO_FRAMES=30
ONLINE_LIQUID_HUE1_LOW=0
ONLINE_LIQUID_HUE1_HIGH=12
ONLINE_LIQUID_HUE2_LOW=161
ONLINE_LIQUID_HUE2_HIGH=179
ONLINE_LIQUID_SAT_MIN=101
ONLINE_LIQUID_VAL_MIN=167
ONLINE_LIQUID_HEIGHT_BIAS_MM=0.0
ONLINE_CONFIG_SHA256="$(printf '%s\n' \
  "process_every=${ONLINE_LIQUID_PROCESS_EVERY}" \
  "zero_frames=${ONLINE_LIQUID_ZERO_FRAMES}" \
  "hue1=${ONLINE_LIQUID_HUE1_LOW}:${ONLINE_LIQUID_HUE1_HIGH}" \
  "hue2=${ONLINE_LIQUID_HUE2_LOW}:${ONLINE_LIQUID_HUE2_HIGH}" \
  "sat_min=${ONLINE_LIQUID_SAT_MIN}" "val_min=${ONLINE_LIQUID_VAL_MIN}" \
  "height_bias_mm=${ONLINE_LIQUID_HEIGHT_BIAS_MM}" | sha256sum | awk '{print $1}')"
ONLINE_NODE_SHA256="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')"
ONLINE_DETECTOR_SHA256="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')"
ONLINE_MSG_SHA256="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')"

validate_launch_contract() {
  local dump
  dump="$(roslaunch --dump-params spmpc_local_planner spmpc_fixed_path.launch \
    planner_variant:="${VARIANT}" solver_backend:=continuous_mpcc_acados \
    delay_phase_mode:=fixed_closed_loop \
    delay_phase_linear_delay_sec:=0.15 delay_phase_angular_delay_sec:=0.22 \
    imu_shadow_enable:=true imu_subscriber_queue_size:=10 \
    observer_source:=odom observer_fallback_policy:=fail_closed \
    observer_latch_fallback:=false liquid_nowcast_enable:=true \
    liquid_nowcast_publish_comparison:=true state_timing_require_common_epoch:=true \
    execution_contract_fail_closed_on_post_limit_change:=false \
    speed_safety_enable:=true v_safe_max:=0.25 speed_safety_tolerance:=0.0001 \
    v_ref:=0.20 w_slosh:="${W_SLOSH}" w_smooth:="${W_SMOOTH}" \
    w_alpha:="${W_ALPHA}" w_du_a:="${W_DU_A}" w_du_vs:="${W_DU_VS}" alpha_max:=1.2 \
    shared_linear_accel_limit_enable:=true shared_linear_accel_max:=0.6 \
    shared_angular_limit_enable:=true shared_angular_rate_max:=1.2 \
    shared_angular_accel_max:=1.2)" || fail "could not dump launch parameters"

  local expected_lines=(
    "/spmpc_local_planner/planner_variant: ${VARIANT}"
    "/spmpc_local_planner/variants/${VARIANT}/slosh_enable: ${SLOSH_ENABLE}"
    "/spmpc_local_planner/variants/${VARIANT}/smooth_priority_enable: ${EXPECTED_SMOOTH_PRIORITY}"
    "/spmpc_local_planner/variants/${VARIANT}/w_control: ${EXPECTED_W_CONTROL}"
    "/spmpc_local_planner/variants/${VARIANT}/w_smooth: ${W_SMOOTH}"
    "/spmpc_local_planner/variants/${VARIANT}/w_slosh: ${W_SLOSH}"
    "/spmpc_local_planner/variants/${VARIANT}/v_ref: 0.2"
    "/spmpc_local_planner/delay_phase/mode: fixed_closed_loop"
    "/spmpc_local_planner/delay_phase/linear_delay_sec: 0.15"
    "/spmpc_local_planner/delay_phase/angular_delay_sec: 0.22"
    "/spmpc_local_planner/slosh_observer/source: odom"
    "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
    "/spmpc_local_planner/state_timing/require_common_epoch: true"
    "/spmpc_local_planner/liquid_nowcast/enable: true"
    "/spmpc_local_planner/liquid_nowcast/publish_comparison: true"
    "/spmpc_local_planner/speed_safety/enable: true"
    "/spmpc_local_planner/speed_safety/v_safe_max: 0.25"
    "/spmpc_local_planner/platform/shared_constraints/linear_accel_limit_enable: true"
    "/spmpc_local_planner/platform/shared_constraints/angular_limit_enable: true"
  )
  local line
  for line in "${expected_lines[@]}"; do
    grep -Fqx -- "${line}" <<< "${dump}" || fail "launch contract missing: ${line}"
  done
}

validate_launch_contract

runner_env=(
  "MATRIX_PRESET=" "DATE=${DATE}" "STAMP=${STAMP}"
  "PILOT_MODE=true" "PILOT_METHOD=" "PILOT_CONDITION=${PROTOCOL_ID}"
  "PILOT_RECORD_RGB=false" "PILOT_RECORD_ONLINE_LIQUID=true"
  "VARIANT=${VARIANT}" "ALG=${VARIANT}" "EXPECTED_RUNTIME_VARIANT=${VARIANT}"
  "RUN_LABEL=${RUN_LABEL}" "NAME=${NAME}" "RUN_OUT_DIR=${RUN_OUT_DIR}"
  "PATH_SOURCE_MODE=replay" "PATH_FILE=${FROZEN_PATH_FILE}"
  "PATH_EXPECTED_SHA256=${FROZEN_PATH_SHA256}" "REQUIRE_PATH_HASH=true"
  "START_POS_TOL=0.08" "START_YAW_TOL=0.15" "START_HOLD_SEC=0.5"
  "START_GATE_TIMEOUT_SEC=120" "PATH_PUBLISH_RATE=2.0"
  "SOLVER_BACKEND=continuous_mpcc_acados" "CMD_TOPIC=/cmd_vel"
  "REF_TOPIC=/scout/global_path_fixed" "COSTMAP_TOPIC=/map"
  "REFERENCE_TARGET_FRAME=map" "BASE_FRAME=base_link"
  "V_REF=0.20" "W_SLOSH=${W_SLOSH}" "W_SMOOTH=${W_SMOOTH}"
  "W_ALPHA=${W_ALPHA}" "W_DU_A=${W_DU_A}" "W_DU_VS=${W_DU_VS}" "ALPHA_MAX=1.2"
  "DELAY_PHASE_MODE=fixed_closed_loop" "DELAY_PHASE_LINEAR_DELAY_SEC=0.15"
  "DELAY_PHASE_ANGULAR_DELAY_SEC=0.22"
  "IMU_SHADOW_ENABLE=true" "IMU_TOPIC=${IMU_TOPIC}"
  "IMU_SUBSCRIBER_QUEUE_SIZE=10" "IMU_SHADOW_READY_TIMEOUT_SEC=30"
  "CURRENT_OBSERVER_SOURCE=odom" "OBSERVER_FALLBACK_POLICY=fail_closed"
  "OBSERVER_LATCH_FALLBACK=false"
  "LIQUID_NOWCAST_ENABLE=true" "LIQUID_NOWCAST_PUBLISH_COMPARISON=true"
  "STATE_TIMING_REQUIRE_COMMON_EPOCH=true"
  "STATE_TIMING_MAX_RAW_SKEW_SEC=0.080"
  "STATE_TIMING_MAX_INTERPOLATION_GAP_SEC=0.050"
  "STATE_TIMING_MAX_ROBOT_EXTRAPOLATION_SEC=0.010"
  "EXECUTION_CONTRACT_FAIL_CLOSED=false"
  "SHARED_LINEAR_ACCEL_LIMIT_ENABLE=true" "SHARED_LINEAR_ACCEL_MAX=0.6"
  "SHARED_ANGULAR_LIMIT_ENABLE=true" "SHARED_ANGULAR_RATE_MAX=1.2"
  "SHARED_ANGULAR_ACCEL_MAX=1.2"
  "SPEED_SAFETY_ENABLE=true" "V_SAFE_MAX=0.25" "SPEED_SAFETY_TOLERANCE=0.0001"
  "RECORD_RGB=false" "RECORD_CAMERA=false" "RECORD_CAMERA_INFO=true"
  "RECORD_CAMERA_COMPRESSED=false" "RECORD_DEPTH=false"
  "RECORD_ONLINE_LIQUID=true" "RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false"
  "RECORD_STANDALONE_SLOSH=false" "RECORD_SCAN=true"
  "FORBID_IMAGE_STREAMS=true" "RECORD_ALL_EXISTING_TOPICS=false"
  "RECORD_TOPIC_INFO=true" "RECORD_MOCAP=true" "RECORD_MOCAP_PATH=false"
  "MOCAP_TRACKER=${MOCAP_TRACKER}"
  "RGB_CALIBRATION_FILE=${RGB_CALIBRATION_FILE}"
  "RGB_CALIBRATION_EXPECTED_SHA256=${RGB_CALIBRATION_SHA256}"
  "RGB_EXPECTED_WIDTH=1920" "RGB_EXPECTED_HEIGHT=1080" "RGB_EXPECTED_FPS=30"
  "ONLINE_LIQUID_MEASUREMENT_TOPIC=${ONLINE_LIQUID_MEASUREMENT_TOPIC}"
  "ONLINE_LIQUID_PROTOCOL=${PROTOCOL_ID}"
  "ONLINE_LIQUID_CALIBRATION_SHA256=${RGB_CALIBRATION_SHA256}"
  "ONLINE_LIQUID_DETECTOR_SHA256=${ONLINE_DETECTOR_SHA256}"
  "ONLINE_LIQUID_NODE_SHA256=${ONLINE_NODE_SHA256}"
  "ONLINE_LIQUID_MSG_SHA256=${ONLINE_MSG_SHA256}"
  "ONLINE_LIQUID_CONFIG_SHA256=${ONLINE_CONFIG_SHA256}"
  "RECORD_SEC=${RECORD_SEC}" "MAX_RECORD_SEC=${RECORD_SEC}"
  "RECORDER_STARTUP_SEC=8" "RECORDER_ACTIVE_TIMEOUT_SEC=15"
  "PLANNER_STARTUP_SEC=2" "PATH_GENERATOR_STARTUP_SEC=2"
  "BLOCK_SEGMENT_ID=${BLOCK_SEGMENT_ID}" "ORDER_POSITION=${ROW}"
  "SPLIT_BLOCK=false" "ACQUISITION_RETRY=false" "SEND_ZERO_ON_EXIT=true"
  "OPERATOR_NOTE=${OPERATOR_NOTE}"
)

echo "================ O0 + L22 C02 ABBA row ================"
if ${SUPPLEMENTAL_B0}; then
  echo "  run          = supplemental B0 reference; outside the four-row ABBA decision"
  echo "  state        = fixed_closed_loop predicts robot state; liquid state is not consumed"
else
  echo "  row/order    = ${ROW}; ${PAIR_ORDER_TEXT}"
  echo "  state        = odom O0; common epoch required for slosh solver"
fi
echo "  condition    = ${CONDITION} (${VARIANT}, w_slosh=${W_SLOSH})"
echo "  delay        = fixed_closed_loop 0.15 / 0.22 s"
echo "  speed        = v_ref 0.20 m/s; hard maximum 0.25 m/s"
echo "  evidence     = online RGB scalar + NOKOV + planner/odom/IMU"
echo "  bag images   = forbidden"
echo "  output       = ${BAG_PATH}"
echo "========================================================="

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS; motion was NOT started"
  printf '[%s] armed runner command:\n  env ' "${SCRIPT_NAME}"
  printf '%q ' "${runner_env[@]}"
  printf 'bash %q\n' "${RUNNER}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "motion is disarmed; set ARM_MOTION=YES only after clearing C02 and checking the E-stop"
[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || \
  fail "set CONFIRM_RGB_GEOMETRY=YES only if camera/container/rulers still match the frozen calibration"
for output in "${BAG_PATH}" "${BAG_PATH}.active" "${CHAIN_REPORT}" \
  "${SUMMARY_JSON}" "${ROW_BINDING}"; do
  [[ ! -e "${output}" ]] || fail "preserve existing output and choose a new ATTEMPT: ${output}"
done

# The wrapper itself is intentionally allowed to be a new development file.
# Runtime controller/config/recorder changes still require an explicit review.
runtime_paths=(
  src/scout_apps/control/spmpc_local_planner/src
  src/scout_apps/control/spmpc_local_planner/include
  src/scout_apps/control/spmpc_local_planner/config
  src/scout_apps/control/spmpc_local_planner/launch
  src/scout_apps/control/spmpc_local_planner/generated
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
  src/scout_apps/sensors/realsense_liquid_measurement
)
dirty_runtime="$(git -C "${REPO_ROOT}" status --porcelain --untracked-files=normal -- "${runtime_paths[@]}")"
[[ -z "${dirty_runtime}" ]] || fail "runtime/config/evidence paths are dirty; review, commit and rebuild before motion"

for topic in /map /scan_front "${ODOM_TOPIC}" "${IMU_TOPIC}" \
  "${RGB_IMAGE_TOPIC}" "${RGB_CAMERA_INFO_TOPIC}"; do
  timeout 6s rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1 || \
    fail "runtime topic unavailable: ${topic}"
done
raw_mocap_topic="/vrpn_client_node/${MOCAP_TRACKER}/pose"
timeout 6s rostopic echo -n 1 "${raw_mocap_topic}" >/dev/null 2>&1 || \
  fail "no NOKOV pose: ${raw_mocap_topic}"
mocap_status="$(timeout 6s rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
grep -q "OK.*tracker=${MOCAP_TRACKER}" <<< "${mocap_status}" || fail "/mocap/status is not OK"
runtime_map_file="$(rosparam get /cartographer_node/frozen_map_file 2>/dev/null || true)"
runtime_map_sha="$(rosparam get /cartographer_node/frozen_map_expected_sha256 2>/dev/null || true)"
[[ "$(readlink -f "${runtime_map_file}")" == "$(readlink -f "${FROZEN_MAP_FILE}")" ]] || \
  fail "runtime Cartographer map differs from the frozen map"
[[ "${runtime_map_sha,,}" == "${FROZEN_MAP_SHA256}" ]] || fail "runtime map hash differs"
published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
grep -Fxq -- /cmd_vel <<< "${published_topics}" && fail "/cmd_vel already has a publisher"

available_kib="$(df -Pk /home/geist/slosh_bags/real | awk 'NR == 2 {print $4}')"
required_kib=$((MIN_FREE_GIB * 1024 * 1024))
(( available_kib >= required_kib )) || fail "require at least ${MIN_FREE_GIB} GiB free"

mkdir -p "${RUN_OUT_DIR}"
if ${SUPPLEMENTAL_B0}; then
  protocol_contents="$(printf '%s\n' \
    "protocol=${PROTOCOL_ID}" "scope=supplemental_reference_not_abba" \
    "row_order=B0:B0" \
    "path_file=${FROZEN_PATH_FILE}" "path_sha256=${FROZEN_PATH_SHA256}" \
    "map_file=${FROZEN_MAP_FILE}" "map_sha256=${FROZEN_MAP_SHA256}" \
    "rgb_calibration_sha256=${RGB_CALIBRATION_SHA256}" \
    "observer=odom_diagnostic_only" "observer_fallback=fail_closed" "common_epoch=true" \
    "delay=fixed_closed_loop:0.15,0.22" "liquid_state_not_consumed=true" \
    "v_ref=0.20" "v_safe_max=0.25" "w_control=0.1" "w_smooth=0.1" \
    "B0_w_slosh=0.0" "record_online_rgb_scalar=true" \
    "record_images=false" "record_mocap=true")"
elif [[ "${PAIR_PROFILE}" == "b0_bslosh" ]]; then
  protocol_contents="$(printf '%s\n' \
    "protocol=${PROTOCOL_ID}" "scope=development_effect_screen" \
    "row_order=01:B0,02:B_slosh,03:B_slosh,04:B0" \
    "path_file=${FROZEN_PATH_FILE}" "path_sha256=${FROZEN_PATH_SHA256}" \
    "map_file=${FROZEN_MAP_FILE}" "map_sha256=${FROZEN_MAP_SHA256}" \
    "rgb_calibration_sha256=${RGB_CALIBRATION_SHA256}" \
    "observer=odom_O0" "observer_fallback=fail_closed" "common_epoch=true" \
    "delay=fixed_closed_loop:0.15,0.22" "liquid_delay_state=L22_for_B_slosh_only" \
    "v_ref=0.20" "v_safe_max=0.25" "w_control=0.1" "w_smooth=0.1" \
    "B0_w_slosh=0.0" "B_slosh_w_slosh=5.0" \
    "only_intended_difference=slosh_enable_and_slosh_cost" \
    "record_online_rgb_scalar=true" "record_images=false" "record_mocap=true")"
else
  protocol_contents="$(printf '%s\n' \
    "protocol=${PROTOCOL_ID}" "scope=development_effect_screen" \
    "row_order=01:B_smooth,02:B_ours,03:B_ours,04:B_smooth" \
    "path_file=${FROZEN_PATH_FILE}" "path_sha256=${FROZEN_PATH_SHA256}" \
    "map_file=${FROZEN_MAP_FILE}" "map_sha256=${FROZEN_MAP_SHA256}" \
    "rgb_calibration_sha256=${RGB_CALIBRATION_SHA256}" \
    "observer=odom_O0" "observer_fallback=fail_closed" "common_epoch=true" \
    "delay=fixed_closed_loop:0.15,0.22" "liquid_delay_state=L22" \
    "v_ref=0.20" "v_safe_max=0.25" "w_control=0.3" "w_smooth=1.0" \
    "B_smooth_w_slosh=0.0" "B_ours_w_slosh=5.0" \
    "record_online_rgb_scalar=true" "record_images=false" "record_mocap=true")"
fi
if [[ -e "${PROTOCOL_FILE}" ]]; then
  [[ "$(<"${PROTOCOL_FILE}")" == "${protocol_contents}" ]] || \
    fail "existing protocol differs; use a new DATE/RUN_OUT_DIR"
else
  printf '%s\n' "${protocol_contents}" > "${PROTOCOL_FILE}"
  if ${SUPPLEMENTAL_B0}; then
    printf '%s\n' \
      'row,block,position,condition,variant,w_slosh' \
      'B0,B0,00,B0,B0,0.0' > "${ORDER_FILE}"
  elif [[ "${PAIR_PROFILE}" == "b0_bslosh" ]]; then
    printf '%s\n' \
      'row,block,position,condition,variant,w_slosh' \
      '01,01,01,B0,B0,0.0' \
      '02,01,02,Bslosh,B_slosh,5.0' \
      '03,02,01,Bslosh,B_slosh,5.0' \
      '04,02,02,B0,B0,0.0' > "${ORDER_FILE}"
  else
    printf '%s\n' \
      'row,block,position,condition,variant,w_slosh' \
      '01,01,01,Bsmooth,B_smooth,0.0' \
      '02,01,02,Bours,B_ours,5.0' \
      '03,02,01,Bours,B_ours,5.0' \
      '04,02,02,Bsmooth,B_smooth,0.0' > "${ORDER_FILE}"
  fi
fi
printf '%s\n' \
  "protocol=${PROTOCOL_ID}" "row=${ROW}" "attempt=${ATTEMPT}" \
  "condition=${CONDITION}" "variant=${VARIANT}" "w_slosh=${W_SLOSH}" \
  "git_revision=$(git -C "${REPO_ROOT}" rev-parse HEAD)" \
  "wrapper_sha256=$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}')" \
  "created_at=$(date --iso-8601=seconds)" > "${ROW_BINDING}"

bash "${CAMERA_PREP}"

publisher_count() {
  local topic_info
  if ! topic_info="$(rostopic info "$1" 2>/dev/null)"; then
    printf '0\n'
    return 0
  fi
  awk '/^Publishers:/ {inside=1; next} /^Subscribers:/ {inside=0} inside && /^[[:space:]]+\*/ {count++} END {print count+0}' \
    <<< "${topic_info}"
}
for output_topic in "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" /liquid/height \
  /liquid/height_lcr /liquid/height_median /liquid/debug_image; do
  count="$(publisher_count "${output_topic}")"
  [[ "${count}" == "0" ]] || fail "unexpected pre-existing publisher(s) on ${output_topic}: ${count}"
done

online_log="${RUN_OUT_DIR}/${NAME}_online_liquid.log"
online_pid=""
cleanup_online() {
  if [[ -n "${online_pid}" ]] && kill -0 "${online_pid}" 2>/dev/null; then
    kill -INT "${online_pid}" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "${online_pid}" 2>/dev/null || break
      sleep 0.1
    done
    kill -0 "${online_pid}" 2>/dev/null && kill -TERM "${online_pid}" 2>/dev/null || true
    wait "${online_pid}" 2>/dev/null || true
  fi
}
trap cleanup_online EXIT

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
timeout 25s rostopic echo -n 20 \
  --filter "m.valid and m.zero_locked and m.status_code == 0 and not m.any_clipped" \
  "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" > "${ready_log}" 2>&1 || \
  fail "online RGB scalar did not reach 20 clean READY samples"

echo "[${SCRIPT_NAME}] RGB scalar READY. Keep the robot still until the runner releases C02."
env "${runner_env[@]}" bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"
postflight_rc=0
python3 "${CHAIN_POSTFLIGHT}" "${BAG_PATH}" \
  --variant "${VARIANT}" --mocap-tracker "${MOCAP_TRACKER}" \
  --imu-topic "${IMU_TOPIC}" --path-file "${FROZEN_PATH_FILE}" \
  --path-sha256 "${FROZEN_PATH_SHA256}" --report "${CHAIN_REPORT}" || postflight_rc=$?
summary_rc=0
python3 "${SUMMARIZER}" "${BAG_PATH}" --out-dir "${RUN_OUT_DIR}" || summary_rc=$?
(( summary_rc == 0 )) || fail "bag was preserved, but summary generation failed (rc=${summary_rc})"
(( postflight_rc == 0 )) || fail "bag was preserved, but execution-chain postflight failed; inspect ${CHAIN_REPORT}"

echo "[${SCRIPT_NAME}] PASS row ${ROW}: ${BAG_PATH}"
echo "[${SCRIPT_NAME}] Return to the C02 start, align heading, and let the liquid settle 60-90 s before the next row."
