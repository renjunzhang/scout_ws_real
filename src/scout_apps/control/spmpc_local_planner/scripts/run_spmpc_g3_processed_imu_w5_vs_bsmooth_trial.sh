#!/usr/bin/env bash
# One image-free online-RGB G3 development trial: processed-IMU W5 vs Bsmooth.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G3][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
G3_WRAPPER="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
[[ -r /opt/ros/noetic/setup.bash ]] || fail "missing ROS Noetic setup"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]] || fail "missing ${REPO_ROOT}/devel/setup.bash"
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
VALIDATOR="${SCRIPT_DIR}/../tools/analysis/validate_g3_online_rgb_trial.py"
ANALYZER="${SCRIPT_DIR}/../tools/analysis/analyze_g3_w5_vs_bsmooth.py"
SUMMARIZER="${SCRIPT_DIR}/../tools/analysis/summarize_spmpc_real_trial.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"
ONLINE_LIQUID_LAUNCH="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/launch/online_liquid_height.launch"
ONLINE_LIQUID_NODE="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py"
ONLINE_LIQUID_DETECTOR="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py"
ONLINE_LIQUID_MSG="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg"

G3_ROW="${G3_ROW:-}"
G3_ATTEMPT="${G3_ATTEMPT:-01}"
DATE="${DATE:-20260801}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ALLOW_DIRTY_VALIDATE_ONLY="${ALLOW_DIRTY_VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"

PATH_FILE="/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json"
PATH_SHA256="578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e"
SOURCE_REPORT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/analysis/G2S_RAW_RGB_THREE_TRIAL_DEVELOPMENT_REPORT.json"
SOURCE_REPORT_SHA256="36db4d12948f98e4ef3580bc52367cdef14d8440d80a8afbb32182b704ee5442"
SMOKE_REPORT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/analysis/PROCESSED_IMU_G2C_DEVELOPMENT_SMOKE_REPORT.json"
SMOKE_REPORT_SHA256="5635d31e0221bdc54a00ee9eb11043515112a3326eb7945d10d0ffb5e13cc5d3"
G2C_ROOT="/home/geist/slosh_bags/real/20260801_spmpc_g2c_processed_imu_w2w5/H0"
G2C_RELEASE_REVISION="5e0e12611de35e9869835a70336f0162e42da875"

RGB_CALIBRATION_FILE="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml"
RGB_CALIBRATION_SHA256="7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01"
RGB_CAMERA_PARAMS_FILE="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/camera_params/realsense_rgb_fixed_params_20260731_203322.yaml"
RGB_CAMERA_PARAMS_SHA256="0fac946203c4e7592a3fd4e5302d92e4339a074d9cbb4094ecf309b69f84c4d9"
RGB_EXPECTED_WIDTH=1920
RGB_EXPECTED_HEIGHT=1080
RGB_EXPECTED_FPS=30
RGB_IMAGE_TOPIC="/camera/color/image_raw"
RGB_CAMERA_INFO_TOPIC="/camera/color/camera_info"
ONLINE_LIQUID_MEASUREMENT_TOPIC="/liquid/measurement"
ONLINE_LIQUID_PROCESS_EVERY=1
ONLINE_LIQUID_ZERO_FRAMES=30
ONLINE_LIQUID_MIN_RATE_HZ=10.0
ONLINE_LIQUID_READY_TIMEOUT_SEC=20
ONLINE_LIQUID_HUE1_LOW=0
ONLINE_LIQUID_HUE1_HIGH=12
ONLINE_LIQUID_HUE2_LOW=161
ONLINE_LIQUID_HUE2_HIGH=179
ONLINE_LIQUID_SAT_MIN=101
ONLINE_LIQUID_VAL_MIN=167
ONLINE_LIQUID_HEIGHT_BIAS_MM=0.0

T_HVIS_TAIL=5.0
T_MOTION_MAX=42
MAX_PRE_MOTION_SEC=23
DELTA_H_DEV_MM=0.10
RECORD_SEC=70
MIN_FREE_GIB="${MIN_FREE_GIB:-5}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g3_processed_imu_w5_vs_bsmooth/H0}"
PREREG_FILE="${RUN_OUT_DIR}/G3_processed_imu_W5_vs_Bsmooth_prereg.env"
ORDER_FILE="${RUN_OUT_DIR}/G3_processed_imu_W5_vs_Bsmooth_order.csv"
METRIC_FILE="${RUN_OUT_DIR}/G3_rgb_sync_and_metric.yaml"
ONLINE_CONFIG_FILE="${RUN_OUT_DIR}/G3_online_liquid_config.env"

case "${G3_ROW}" in
  01) BLOCK=01; POSITION=01; CONDITION=Bsmooth; METHOD=Bsmooth; WEIGHT=0 ;;
  02) BLOCK=01; POSITION=02; CONDITION=Bslosh;  METHOD=W5;      WEIGHT=5 ;;
  03) BLOCK=02; POSITION=01; CONDITION=Bslosh;  METHOD=W5;      WEIGHT=5 ;;
  04) BLOCK=02; POSITION=02; CONDITION=Bsmooth; METHOD=Bsmooth; WEIGHT=0 ;;
  05) BLOCK=03; POSITION=01; CONDITION=Bslosh;  METHOD=W5;      WEIGHT=5 ;;
  06) BLOCK=03; POSITION=02; CONDITION=Bsmooth; METHOD=Bsmooth; WEIGHT=0 ;;
  07) BLOCK=04; POSITION=01; CONDITION=Bsmooth; METHOD=Bsmooth; WEIGHT=0 ;;
  08) BLOCK=04; POSITION=02; CONDITION=Bslosh;  METHOD=W5;      WEIGHT=5 ;;
  *) fail "set G3_ROW=01..08; frozen order is Bsmooth,W5,W5,Bsmooth,W5,Bsmooth,Bsmooth,W5" ;;
esac
[[ "${G3_ATTEMPT}" == "01" ]] || \
  fail "the frozen G3 development wrapper currently permits only G3_ATTEMPT=01"

required_files=(
  "${RUNNER}" "${VALIDATOR}" "${ANALYZER}" "${SUMMARIZER}" "${CAMERA_PREP}"
  "${ONLINE_LIQUID_LAUNCH}" "${ONLINE_LIQUID_NODE}" "${ONLINE_LIQUID_DETECTOR}"
  "${ONLINE_LIQUID_MSG}" "${PATH_FILE}" "${SOURCE_REPORT}" "${SMOKE_REPORT}"
  "${RGB_CALIBRATION_FILE}" "${RGB_CAMERA_PARAMS_FILE}"
)
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ "$(sha256sum "${PATH_FILE}" | awk '{print $1}')" == "${PATH_SHA256}" ]] || \
  fail "frozen path hash mismatch"
[[ "$(sha256sum "${SOURCE_REPORT}" | awk '{print $1}')" == "${SOURCE_REPORT_SHA256}" ]] || \
  fail "G2S source-report hash mismatch"
[[ "$(sha256sum "${SMOKE_REPORT}" | awk '{print $1}')" == "${SMOKE_REPORT_SHA256}" ]] || \
  fail "processed-IMU smoke-report hash mismatch"
[[ "$(sha256sum "${RGB_CALIBRATION_FILE}" | awk '{print $1}')" == "${RGB_CALIBRATION_SHA256}" ]] || \
  fail "v2 RGB calibration hash mismatch"
[[ "$(sha256sum "${RGB_CAMERA_PARAMS_FILE}" | awk '{print $1}')" == "${RGB_CAMERA_PARAMS_SHA256}" ]] || \
  fail "RealSense frozen-parameter sidecar hash mismatch"

release_revision="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
runtime_paths=(
  src/scout_apps/control/spmpc_local_planner/CMakeLists.txt
  src/scout_apps/control/spmpc_local_planner/config
  src/scout_apps/control/spmpc_local_planner/include
  src/scout_apps/control/spmpc_local_planner/launch
  src/scout_apps/control/spmpc_local_planner/msg
  src/scout_apps/control/spmpc_local_planner/src
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_experiment.sh
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
  src/scout_apps/control/spmpc_local_planner/tools/analysis/summarize_spmpc_real_trial.py
  src/scout_apps/sensors/realsense_liquid_measurement/launch/online_liquid_height.launch
  src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg
  src/scout_apps/sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py
  src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py
)
git -C "${REPO_ROOT}" diff --quiet "${G2C_RELEASE_REVISION}..${release_revision}" -- \
  "${runtime_paths[@]}" || \
  fail "planner/RGB runtime differs from G2C release ${G2C_RELEASE_REVISION}; redo affected gates"

g3_code_paths=(
  "${G3_WRAPPER}" "${RUNNER}" "${VALIDATOR}" "${ANALYZER}" "${SUMMARIZER}" "${CAMERA_PREP}"
)
tracked_missing=()
for code_path in "${g3_code_paths[@]}"; do
  relative_path="${code_path#${REPO_ROOT}/}"
  if ! git -C "${REPO_ROOT}" ls-files --error-unmatch "${relative_path}" >/dev/null 2>&1; then
    tracked_missing+=("${relative_path}")
  fi
done
if (( ${#tracked_missing[@]} > 0 )); then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G3][WARN] allowing untracked G3 code for developer validate-only: ${tracked_missing[*]}" >&2
  else
    fail "G3 code is not part of frozen revision: ${tracked_missing[*]}"
  fi
fi
relevant_repo_paths=(
  "${runtime_paths[@]}"
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3_processed_imu_w5_vs_bsmooth_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/prepare_spmpc_g3_realsense.sh
  src/scout_apps/control/spmpc_local_planner/tools/analysis/validate_g3_online_rgb_trial.py
  src/scout_apps/control/spmpc_local_planner/tools/analysis/analyze_g3_w5_vs_bsmooth.py
)
if ! git -C "${REPO_ROOT}" diff --quiet -- "${relevant_repo_paths[@]}" || \
   ! git -C "${REPO_ROOT}" diff --cached --quiet -- "${relevant_repo_paths[@]}"; then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G3][WARN] allowing dirty G3/runtime paths for developer validate-only" >&2
  else
    fail "G3/runtime paths differ from release ${release_revision}; commit/freeze before motion"
  fi
fi

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
if source.get("status") != "PASS_FOR_G2C_DEVELOPMENT":
    raise SystemExit("source report status changed")
if source.get("decision_scope") != "G2C_DEVELOPMENT_ONLY":
    raise SystemExit("unexpected source-report scope")
aggregate = source.get("aggregate", {})
if aggregate.get("directional_trial_count") != 3:
    raise SystemExit("source decision is not 3/3 directionally consistent")
if aggregate.get("processed_imu_relative_improvement", 0.0) < 0.10:
    raise SystemExit("source improvement is below 10%")
if not aggregate.get("coverage_pass") or not aggregate.get("not_single_trial_dominated"):
    raise SystemExit("source coverage/dominance gate failed")
if smoke.get("status") != "PASS_FOR_G2C_DEVELOPMENT":
    raise SystemExit("processed-IMU implementation smoke failed")
if smoke.get("fallback_active") or smoke.get("cmd_vel_messages_observed") != 0:
    raise SystemExit("processed-IMU smoke fallback/no-command invariant failed")
PY

g2c_labels=(
  DEV_G2C_H0_C1_W2_b01_p01_a01
  DEV_G2C_H0_C1_W5_b01_p02_a01
  DEV_G2C_H0_C1_W5_b02_p03_a01
  DEV_G2C_H0_C1_W2_b02_p04_a01
)
G2C_EVIDENCE_LINES=""
for g2c_label in "${g2c_labels[@]}"; do
  g2c_report="${G2C_ROOT}/${g2c_label}_g2c_postflight.json"
  [[ -s "${g2c_report}" ]] || fail "missing G2C postflight: ${g2c_report}"
  g2c_report_sha="$(sha256sum "${g2c_report}" | awk '{print $1}')"
  python3 - "${g2c_report}" <<'PY'
import json
import os
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("status") != "PASS":
    raise SystemExit("G2C postflight is not PASS")
if report.get("selection", {}).get("nominal_processed_imu_fraction") != 1.0:
    raise SystemExit("G2C source coverage is not 100%")
if report.get("processed_imu", {}).get("ready_fraction") != 1.0:
    raise SystemExit("G2C IMU READY coverage is not 100%")
if report.get("selection", {}).get("fallback_samples") != 0:
    raise SystemExit("G2C fallback was observed")
bag = report.get("bag")
if not bag or not os.path.isfile(bag):
    raise SystemExit("G2C bag referenced by postflight is missing")
if os.path.getsize(bag) != report.get("bag_size_bytes"):
    raise SystemExit("G2C bag size differs from postflight")
PY
  G2C_EVIDENCE_LINES+="g2c_postflight_${g2c_label}=${g2c_report_sha}"$'\n'
done
G2C_EVIDENCE_SHA256="$(printf '%s' "${G2C_EVIDENCE_LINES}" | sha256sum | awk '{print $1}')"

online_launch_sha="$(sha256sum "${ONLINE_LIQUID_LAUNCH}" | awk '{print $1}')"
online_node_sha="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')"
online_detector_sha="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')"
online_msg_sha="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')"

order_contents() {
  printf '%s\n' \
    "row,block,position,condition,pilot_method,block_segment_id" \
    "01,01,01,Bsmooth,Bsmooth,G3_b01_seg01" \
    "02,01,02,Bslosh,W5,G3_b01_seg01" \
    "03,02,01,Bslosh,W5,G3_b02_seg01" \
    "04,02,02,Bsmooth,Bsmooth,G3_b02_seg01" \
    "05,03,01,Bslosh,W5,G3_b03_seg01" \
    "06,03,02,Bsmooth,Bsmooth,G3_b03_seg01" \
    "07,04,01,Bsmooth,Bsmooth,G3_b04_seg01" \
    "08,04,02,Bslosh,W5,G3_b04_seg01"
}

metric_contents() {
  printf '%s\n' \
    "schema_version: 1" \
    "primary_metric: h_vis_p95_motion_plus_tail_mm" \
    "source_field: /liquid/measurement.height_max_lcr_mm" \
    "nonnegative_floor_mm: 0.0" \
    "rolling_median_window: 5" \
    "rolling_median_semantics: causal_current_and_previous_valid_samples" \
    "rolling_window_resets_after_gap_sec: 0.35" \
    "motion_threshold_abs_linear_or_angular: 0.03" \
    "t_hvis_tail_sec: 5.0" \
    "t_motion_max_sec: ${T_MOTION_MAX}" \
    "maximum_record_to_motion_sec: ${MAX_PRE_MOTION_SEC}" \
    "minimum_online_rate_hz: 10.0" \
    "minimum_valid_fraction: 0.90" \
    "minimum_pre_motion_valid_sec: 2.0" \
    "maximum_publish_lag_p95_sec: 0.50" \
    "maximum_future_skew_sec: 0.05" \
    "image_stream_policy: forbid_all" \
    "failure_timeout_window: first_motion_plus_t_motion_max_plus_tail"
}
expected_metric="$(metric_contents)"
OUTCOME_WINDOW_RULE_SHA256="$(printf '%s\n' "${expected_metric}" | sha256sum | awk '{print $1}')"

online_config_contents() {
  printf '%s\n' \
    "measurement_topic=${ONLINE_LIQUID_MEASUREMENT_TOPIC}" \
    "image_topic=${RGB_IMAGE_TOPIC}" \
    "process_every=${ONLINE_LIQUID_PROCESS_EVERY}" \
    "zero_frames=${ONLINE_LIQUID_ZERO_FRAMES}" \
    "minimum_recorded_rate_hz=${ONLINE_LIQUID_MIN_RATE_HZ}" \
    "height_bias_mm=${ONLINE_LIQUID_HEIGHT_BIAS_MM}" \
    "publish_debug=false" \
    "hue1_low=${ONLINE_LIQUID_HUE1_LOW}" \
    "hue1_high=${ONLINE_LIQUID_HUE1_HIGH}" \
    "hue2_low=${ONLINE_LIQUID_HUE2_LOW}" \
    "hue2_high=${ONLINE_LIQUID_HUE2_HIGH}" \
    "sat_min=${ONLINE_LIQUID_SAT_MIN}" \
    "val_min=${ONLINE_LIQUID_VAL_MIN}"
}
expected_online_config="$(online_config_contents)"
ONLINE_CONFIG_SHA256="$(printf '%s\n' "${expected_online_config}" | sha256sum | awk '{print $1}')"

prereg_contents() {
  printf '%s\n' \
    "protocol=G3_processed_imu_W5_vs_Bsmooth_online_RGB_development_v1" \
    "scope=development_only" \
    "formal_stage_status=NO_GO" \
    "planned_units=8" \
    "paired_blocks=4" \
    "planned_order=01:Bsmooth,02:W5,03:W5,04:Bsmooth,05:W5,06:Bsmooth,07:Bsmooth,08:W5" \
    "minimum_eligible_pairs=4" \
    "candidate=W5" \
    "candidate_weight=5" \
    "comparator=Bsmooth" \
    "current_observer_source=processed_imu" \
    "source_report=${SOURCE_REPORT}" \
    "source_report_sha256=${SOURCE_REPORT_SHA256}" \
    "source_evidence_scope=THREE_TRIAL_G2C_DEVELOPMENT_ONLY" \
    "source_evidence_limitation=not_a_formal_four_unit_G2S_pass" \
    "processed_imu_smoke_report_sha256=${SMOKE_REPORT_SHA256}" \
    "g2c_evidence_sha256=${G2C_EVIDENCE_SHA256}" \
    "g2c_release_revision=${G2C_RELEASE_REVISION}" \
    "g3_release_revision=${release_revision}" \
    "path_file=${PATH_FILE}" \
    "path_sha256=${PATH_SHA256}" \
    "v_ref=0.20" \
    "delay_phase=fixed_closed_loop:0.15,0.22" \
    "rgb_calibration_file=${RGB_CALIBRATION_FILE}" \
    "rgb_calibration_sha256=${RGB_CALIBRATION_SHA256}" \
    "rgb_camera_params_sha256=${RGB_CAMERA_PARAMS_SHA256}" \
    "online_launch_sha256=${online_launch_sha}" \
    "online_node_sha256=${online_node_sha}" \
    "online_detector_sha256=${online_detector_sha}" \
    "online_message_sha256=${online_msg_sha}" \
    "online_config_sha256=${ONLINE_CONFIG_SHA256}" \
    "outcome_window_rule_sha256=${OUTCOME_WINDOW_RULE_SHA256}" \
    "t_hvis_tail_sec=${T_HVIS_TAIL}" \
    "t_motion_max_sec=${T_MOTION_MAX}" \
    "maximum_record_to_motion_sec=${MAX_PRE_MOTION_SEC}" \
    "delta_h_dev_mm=${DELTA_H_DEV_MM}" \
    "minimum_positive_blocks=3" \
    "single_block_dominance=forbidden" \
    "leave_one_block_out_direction_flip=forbidden" \
    "record_sec=${RECORD_SEC}" \
    "record_online_liquid=true" \
    "forbid_image_streams=true" \
    "no_early_stop=true" \
    "attempt_policy=first_attempt_only_no_manual_retry"
}
expected_prereg="$(prereg_contents)"
PREREG_SHA256="$(printf '%s\n' "${expected_prereg}" | sha256sum | awk '{print $1}')"
expected_order="$(order_contents)"
ORDER_SHA256="$(printf '%s\n' "${expected_order}" | sha256sum | awk '{print $1}')"

RUN_LABEL="DEV_G3_H0_C1_${METHOD}_b${BLOCK}_p${POSITION}_a${G3_ATTEMPT}"
BAG_PATH="${RUN_OUT_DIR}/${RUN_LABEL}.bag"

runner_env=(
  "DATE=${DATE}"
  "STAMP=${STAMP}"
  "PILOT_MODE=true"
  "PILOT_METHOD=${METHOD}"
  "PILOT_CONDITION=G3_processed_imu_W5_vs_Bsmooth_online_RGB"
  "PILOT_RECORD_RGB=false"
  "PILOT_RECORD_ONLINE_LIQUID=true"
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
  "START_GATE_TIMEOUT_SEC=3"
  "V_REF=0.20"
  "DELAY_PHASE_MODE=fixed_closed_loop"
  "DELAY_PHASE_LINEAR_DELAY_SEC=0.15"
  "DELAY_PHASE_ANGULAR_DELAY_SEC=0.22"
  "IMU_SHADOW_ENABLE=true"
  "IMU_SHADOW_READY_TIMEOUT_SEC=12"
  "CURRENT_OBSERVER_SOURCE=processed_imu"
  "OBSERVER_FALLBACK_POLICY=odom"
  "OBSERVER_LATCH_FALLBACK=true"
  "RECORD_RGB=false"
  "RECORD_CAMERA=false"
  "RECORD_CAMERA_INFO=true"
  "RECORD_CAMERA_COMPRESSED=false"
  "RECORD_DEPTH=false"
  "RECORD_ONLINE_LIQUID=true"
  "RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false"
  "FORBID_IMAGE_STREAMS=true"
  "RECORD_ALL_EXISTING_TOPICS=false"
  "RGB_CALIBRATION_FILE=${RGB_CALIBRATION_FILE}"
  "RGB_CALIBRATION_EXPECTED_SHA256=${RGB_CALIBRATION_SHA256}"
  "RGB_EXPECTED_WIDTH=${RGB_EXPECTED_WIDTH}"
  "RGB_EXPECTED_HEIGHT=${RGB_EXPECTED_HEIGHT}"
  "RGB_EXPECTED_FPS=${RGB_EXPECTED_FPS}"
  "ONLINE_LIQUID_MEASUREMENT_TOPIC=${ONLINE_LIQUID_MEASUREMENT_TOPIC}"
  "ONLINE_LIQUID_PROTOCOL=G3_processed_imu_W5_vs_Bsmooth_online_RGB_development_v1"
  "ONLINE_LIQUID_CALIBRATION_SHA256=${RGB_CALIBRATION_SHA256}"
  "ONLINE_LIQUID_DETECTOR_SHA256=${online_detector_sha}"
  "ONLINE_LIQUID_NODE_SHA256=${online_node_sha}"
  "ONLINE_LIQUID_MSG_SHA256=${online_msg_sha}"
  "ONLINE_LIQUID_CONFIG_SHA256=${ONLINE_CONFIG_SHA256}"
  "RECORD_SEC=${RECORD_SEC}"
  "MAX_RECORD_SEC=${RECORD_SEC}"
  "BLOCK_SEGMENT_ID=G3_b${BLOCK}_seg01"
  "ORDER_POSITION=${POSITION}"
  "SEND_ZERO_ON_EXIT=true"
  "OPERATOR_NOTE=G3_dev_${CONDITION}_processed_imu_source_${SOURCE_REPORT_SHA256}_g2c_${G2C_EVIDENCE_SHA256}_window_${OUTCOME_WINDOW_RULE_SHA256}"
)

echo "================ G3 online-RGB unit ================="
echo "  row/condition = ${G3_ROW}/${CONDITION} (${METHOD})"
echo "  block/position= ${BLOCK}/${POSITION}"
echo "  source        = processed_imu; W5 consumes, Bsmooth records only"
echo "  source proof  = 3-trial development (${SOURCE_REPORT_SHA256})"
echo "  source limit  = not a formal four-unit G2S PASS"
echo "  path          = ${PATH_FILE}"
echo "  RGB calib     = v2 ${RGB_CALIBRATION_SHA256}"
echo "  bag images    = forbidden; stamped scalar only"
echo "  output        = ${BAG_PATH}"
echo "  scope         = G3 development; formal Stage remains NO-GO"
echo "======================================================"

if truthy "${VALIDATE_ONLY}"; then
  printf '[G3] validate-only runner command:\n  env '
  printf '%q ' "${runner_env[@]}"
  printf 'bash %q\n' "${RUNNER}"
  echo "[G3] prereg SHA256  = ${PREREG_SHA256}"
  echo "[G3] order SHA256   = ${ORDER_SHA256}"
  echo "[G3] window SHA256  = ${OUTCOME_WINDOW_RULE_SHA256}"
  echo "[G3] online SHA256  = ${ONLINE_CONFIG_SHA256}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "real motion is disarmed; rerun with ARM_MOTION=YES after clearing the path"
[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || \
  fail "set CONFIRM_RGB_GEOMETRY=YES only after checking v2 ROI/rulers/container/liquid geometry"
[[ ! -e "${BAG_PATH}" && ! -e "${BAG_PATH}.active" ]] || \
  fail "attempt output already exists: ${BAG_PATH}"

if [[ "${G3_ROW}" != "01" ]]; then
  previous_row="$(printf '%02d' "$((10#${G3_ROW} - 1))")"
  case "${previous_row}" in
    01) previous_label=DEV_G3_H0_C1_Bsmooth_b01_p01_a01 ;;
    02) previous_label=DEV_G3_H0_C1_W5_b01_p02_a01 ;;
    03) previous_label=DEV_G3_H0_C1_W5_b02_p01_a01 ;;
    04) previous_label=DEV_G3_H0_C1_Bsmooth_b02_p02_a01 ;;
    05) previous_label=DEV_G3_H0_C1_W5_b03_p01_a01 ;;
    06) previous_label=DEV_G3_H0_C1_Bsmooth_b03_p02_a01 ;;
    07) previous_label=DEV_G3_H0_C1_Bsmooth_b04_p01_a01 ;;
  esac
  previous_report="${RUN_OUT_DIR}/${previous_label}_g3_postflight.json"
  [[ -s "${previous_report}" ]] || fail "previous row postflight is missing: ${previous_report}"
  python3 - "${previous_report}" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("status") != "PASS":
    raise SystemExit("previous G3 row postflight is not PASS")
PY
fi

available_kib="$(df -Pk /home/geist/slosh_bags/real | awk 'NR == 2 {print $4}')"
required_kib=$((MIN_FREE_GIB * 1024 * 1024))
(( available_kib >= required_kib )) || \
  fail "insufficient free space: require at least ${MIN_FREE_GIB} GiB"

mkdir -p "${RUN_OUT_DIR}"
if [[ -e "${PREREG_FILE}" ]]; then
  [[ "$(<"${PREREG_FILE}")" == "${expected_prereg}" ]] || fail "G3 prereg changed"
  [[ "$(<"${ORDER_FILE}")" == "${expected_order}" ]] || fail "G3 order table changed"
  [[ "$(<"${METRIC_FILE}")" == "${expected_metric}" ]] || fail "G3 metric/window rule changed"
  [[ "$(<"${ONLINE_CONFIG_FILE}")" == "${expected_online_config}" ]] || fail "G3 online RGB config changed"
else
  [[ "${G3_ROW}" == "01" ]] || fail "row 01 must create the G3 prereg bundle"
  printf '%s\n' "${expected_prereg}" > "${PREREG_FILE}"
  printf '%s\n' "${expected_order}" > "${ORDER_FILE}"
  printf '%s\n' "${expected_metric}" > "${METRIC_FILE}"
  printf '%s\n' "${expected_online_config}" > "${ONLINE_CONFIG_FILE}"
  sha256sum "${PREREG_FILE}" "${ORDER_FILE}" "${METRIC_FILE}" "${ONLINE_CONFIG_FILE}" \
    > "${RUN_OUT_DIR}/G3_prereg_bundle.sha256"
fi
[[ "$(sha256sum "${PREREG_FILE}" | awk '{print $1}')" == "${PREREG_SHA256}" ]] || fail "G3 prereg hash mismatch"
[[ "$(sha256sum "${ORDER_FILE}" | awk '{print $1}')" == "${ORDER_SHA256}" ]] || fail "G3 order hash mismatch"
[[ "$(sha256sum "${METRIC_FILE}" | awk '{print $1}')" == "${OUTCOME_WINDOW_RULE_SHA256}" ]] || fail "G3 window-rule hash mismatch"
[[ "$(sha256sum "${ONLINE_CONFIG_FILE}" | awk '{print $1}')" == "${ONLINE_CONFIG_SHA256}" ]] || fail "G3 online-config hash mismatch"

printf '%s\n' \
  "g3_row=${G3_ROW}" \
  "block=${BLOCK}" \
  "position=${POSITION}" \
  "condition=${CONDITION}" \
  "pilot_method=${METHOD}" \
  "source_report_sha256=${SOURCE_REPORT_SHA256}" \
  "g2c_evidence_sha256=${G2C_EVIDENCE_SHA256}" \
  "prereg_sha256=${PREREG_SHA256}" \
  "order_sha256=${ORDER_SHA256}" \
  "outcome_window_rule_sha256=${OUTCOME_WINDOW_RULE_SHA256}" \
  > "${RUN_OUT_DIR}/${RUN_LABEL}_g3_binding.env"

bash "${CAMERA_PREP}"

publisher_count() {
  local topic_info
  if ! topic_info="$(rostopic info "$1" 2>/dev/null)"; then
    printf '0\n'
    return 0
  fi
  awk '
    /^Publishers:/ {in_publishers=1; next}
    /^Subscribers:/ {in_publishers=0}
    in_publishers && /^[[:space:]]+\*/ {count++}
    END {print count+0}
  ' <<< "${topic_info}"
}
for output_topic in "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" /liquid/height \
  /liquid/height_lcr /liquid/height_median /liquid/debug_image; do
  count="$(publisher_count "${output_topic}")"
  [[ "${count}" == "0" ]] || \
    fail "unexpected pre-existing publisher(s) on ${output_topic}: ${count}"
done

online_log="${RUN_OUT_DIR}/${RUN_LABEL}_online_liquid.log"
online_pid=""
cleanup_online() {
  if [[ -n "${online_pid}" ]] && kill -0 "${online_pid}" 2>/dev/null; then
    kill -INT "${online_pid}" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "${online_pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${online_pid}" 2>/dev/null; then
      kill -TERM "${online_pid}" 2>/dev/null || true
    fi
    wait "${online_pid}" 2>/dev/null || true
  fi
}
trap cleanup_online EXIT

echo "[G3] Starting frozen v2 online RGB scalar node (publish_debug=false)."
roslaunch realsense_liquid_measurement online_liquid_height.launch \
  calibration:="${RGB_CALIBRATION_FILE}" \
  image_topic:="${RGB_IMAGE_TOPIC}" \
  measurement_topic:="${ONLINE_LIQUID_MEASUREMENT_TOPIC}" \
  process_every:="${ONLINE_LIQUID_PROCESS_EVERY}" \
  zero_frames:="${ONLINE_LIQUID_ZERO_FRAMES}" \
  publish_debug:=false \
  height_bias_mm:="${ONLINE_LIQUID_HEIGHT_BIAS_MM}" \
  hue1_low:="${ONLINE_LIQUID_HUE1_LOW}" \
  hue1_high:="${ONLINE_LIQUID_HUE1_HIGH}" \
  hue2_low:="${ONLINE_LIQUID_HUE2_LOW}" \
  hue2_high:="${ONLINE_LIQUID_HUE2_HIGH}" \
  sat_min:="${ONLINE_LIQUID_SAT_MIN}" \
  val_min:="${ONLINE_LIQUID_VAL_MIN}" \
  > "${online_log}" 2>&1 &
online_pid=$!
sleep 2
if ! kill -0 "${online_pid}" 2>/dev/null; then
  tail -80 "${online_log}" >&2 || true
  fail "online RGB scalar node exited during startup"
fi
online_type="$(rostopic type "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" 2>/dev/null || true)"
[[ "${online_type}" == "realsense_liquid_measurement/OnlineLiquidMeasurement" ]] || \
  fail "unexpected ${ONLINE_LIQUID_MEASUREMENT_TOPIC} type: ${online_type:-none}"

ready_log="${RUN_OUT_DIR}/${RUN_LABEL}_online_liquid_ready.log"
if ! timeout 20s rostopic echo -n 20 \
  --filter "m.valid and m.zero_locked and m.status_code == 0" \
  "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" > "${ready_log}" 2>&1; then
  tail -80 "${online_log}" >&2 || true
  fail "online RGB did not provide 20 clean zero-locked measurements within 20s"
fi
ready_count="$(grep -Ec '^valid: (True|true)$' "${ready_log}" || true)"
(( ready_count >= 20 )) || fail "online RGB clean-ready sample count is ${ready_count}, expected 20"
echo "[G3] Online RGB READY; keep the robot still while the recorder/IMU bias gate starts."

env "${runner_env[@]}" bash "${RUNNER}"

python3 "${VALIDATOR}" \
  --bag "${BAG_PATH}" \
  --condition "${METHOD}" \
  --row "${G3_ROW}" \
  --block "${BLOCK}" \
  --position "${POSITION}" \
  --expected-weight "${WEIGHT}" \
  --expected-v-ref 0.20 \
  --t-hvis-tail-sec "${T_HVIS_TAIL}" \
  --t-motion-max-sec "${T_MOTION_MAX}" \
  --min-duration-sec 65 \
  --max-pre-motion-sec "${MAX_PRE_MOTION_SEC}" \
  --rgb-calibration-sha256 "${RGB_CALIBRATION_SHA256}" \
  --online-config-sha256 "${ONLINE_CONFIG_SHA256}" \
  --outcome-window-rule-sha256 "${OUTCOME_WINDOW_RULE_SHA256}" \
  --prereg-sha256 "${PREREG_SHA256}" \
  --source-report-sha256 "${SOURCE_REPORT_SHA256}" \
  --hash-bag

python3 "${SUMMARIZER}" "${BAG_PATH}"

if [[ "${G3_ROW}" == "08" ]]; then
  echo "[G3] All planned rows are present; running the frozen four-block analyzer."
  python3 "${ANALYZER}" \
    --root "${RUN_OUT_DIR}" \
    --delta-h-dev-mm "${DELTA_H_DEV_MM}" \
    --minimum-positive-blocks 3 \
    --expected-pairs 4 \
    --prereg-sha256 "${PREREG_SHA256}" \
    --outcome-window-rule-sha256 "${OUTCOME_WINDOW_RULE_SHA256}" \
    --source-report-sha256 "${SOURCE_REPORT_SHA256}"
fi

echo "[G3] unit PASS: ${BAG_PATH}"
echo "[G3] Return, align, and let the liquid settle before the next row."
