#!/usr/bin/env bash
# One physical G2S paired source-selection unit on the frozen 20260727 G2 path.
# The Bsmooth controller does not consume liquid state. The same bag records
# odom observer, processed-IMU observer, and stamped online RGB-derived scalar
# measurements, so source comparison is paired on an identical motion/time axis.
# Raw/debug image streams are forbidden from the bag. This script never runs four
# units automatically; return/alignment/liquid settling and RGB checks are
# required between units.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G2S][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"

# Make the field command self-contained. The sensor stack runs in another
# terminal, but this wrapper must not depend on the operator's shell startup files.
[[ -r /opt/ros/noetic/setup.bash ]] || fail "missing /opt/ros/noetic/setup.bash"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]] || \
  fail "missing ${REPO_ROOT}/devel/setup.bash; build the workspace first"
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
POSTFLIGHT_VALIDATOR="${SCRIPT_DIR}/analysis/validate_g2s_paired_trial.py"
ONLINE_LIQUID_LAUNCH="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/launch/online_liquid_height.launch"
ONLINE_LIQUID_NODE="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py"
ONLINE_LIQUID_DETECTOR="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py"
ONLINE_LIQUID_MSG="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg"
PLANNER_COMMON_CONFIG="${REPO_ROOT}/src/scout_apps/control/spmpc_local_planner/config/planner/common.yaml"
IMU_PLANAR_EXTRINSIC_REPORT="${IMU_PLANAR_EXTRINSIC_REPORT:-/home/geist/slosh_bags/real/20260731_imu_planar_extrinsic_analysis/imu_planar_extrinsic_report.json}"
IMU_PLANAR_EXTRINSIC_EXPECTED_SHA256="18eb5e0602f451612fe973e06254135b148330213c666d448c93b13bdc506202"

G2S_ROW="${G2S_ROW:-}"
G2S_ATTEMPT="${G2S_ATTEMPT:-01}"
DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"

H0S_PATH="${H0S_PATH:-/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json}"
H0S_SHA256="578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e"
RGB_CALIBRATION_FILE="${RGB_CALIBRATION_FILE:-/home/geist/slosh_bags/real/20260629_calib/red_3ruler.yaml}"
RGB_EXPECTED_WIDTH="${RGB_EXPECTED_WIDTH:-1920}"
RGB_EXPECTED_HEIGHT="${RGB_EXPECTED_HEIGHT:-1080}"
RGB_EXPECTED_FPS="${RGB_EXPECTED_FPS:-30}"
RGB_CAMERA_INFO_TOPIC="${RGB_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
RGB_IMAGE_TOPIC="${RGB_IMAGE_TOPIC:-/camera/color/image_raw}"
RGB_DYNPARAM_NS="${RGB_DYNPARAM_NS:-/camera/rgb_camera}"
ONLINE_LIQUID_MEASUREMENT_TOPIC="${ONLINE_LIQUID_MEASUREMENT_TOPIC:-/liquid/measurement}"
ONLINE_LIQUID_PROCESS_EVERY="${ONLINE_LIQUID_PROCESS_EVERY:-1}"
ONLINE_LIQUID_ZERO_FRAMES="${ONLINE_LIQUID_ZERO_FRAMES:-30}"
ONLINE_LIQUID_MIN_RATE_HZ="${ONLINE_LIQUID_MIN_RATE_HZ:-10.0}"
ONLINE_LIQUID_HEIGHT_BIAS_MM="${ONLINE_LIQUID_HEIGHT_BIAS_MM:-0.0}"
ONLINE_LIQUID_READY_TIMEOUT_SEC="${ONLINE_LIQUID_READY_TIMEOUT_SEC:-20}"
ONLINE_LIQUID_HUE1_LOW="${ONLINE_LIQUID_HUE1_LOW:-0}"
ONLINE_LIQUID_HUE1_HIGH="${ONLINE_LIQUID_HUE1_HIGH:-11}"
ONLINE_LIQUID_HUE2_LOW="${ONLINE_LIQUID_HUE2_LOW:-173}"
ONLINE_LIQUID_HUE2_HIGH="${ONLINE_LIQUID_HUE2_HIGH:-179}"
ONLINE_LIQUID_SAT_MIN="${ONLINE_LIQUID_SAT_MIN:-80}"
ONLINE_LIQUID_VAL_MIN="${ONLINE_LIQUID_VAL_MIN:-162}"
MIN_FREE_GIB="${MIN_FREE_GIB:-5}"
POSTFLIGHT_HASH_BAG="${POSTFLIGHT_HASH_BAG:-true}"

N_SRC="${N_SRC:-4}"
DELTA_SRC="${DELTA_SRC:-0.10}"
MIN_DIRECTIONAL_TRIALS="${MIN_DIRECTIONAL_TRIALS:-3}"
RECORD_SEC="${RECORD_SEC:-90}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g2s_source_selection/H0s_Bsmooth}"
PREREG_FILE="${RUN_OUT_DIR}/G2S_H0s_prereg.env"
ONLINE_CONFIG_FILE="${RUN_OUT_DIR}/G2S_online_liquid_config.env"

[[ -f "${RUNNER}" ]] || fail "missing runner: ${RUNNER}"
[[ -f "${POSTFLIGHT_VALIDATOR}" ]] || \
  fail "missing postflight validator: ${POSTFLIGHT_VALIDATOR}"
for required_file in "${ONLINE_LIQUID_LAUNCH}" "${ONLINE_LIQUID_NODE}" \
  "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}"; do
  [[ -s "${required_file}" ]] || fail "missing online RGB artifact: ${required_file}"
done
[[ -s "${IMU_PLANAR_EXTRINSIC_REPORT}" ]] || \
  fail "missing frozen planar IMU extrinsic report: ${IMU_PLANAR_EXTRINSIC_REPORT}"
imu_planar_extrinsic_actual_sha="$(sha256sum "${IMU_PLANAR_EXTRINSIC_REPORT}" | awk '{print $1}')"
[[ "${imu_planar_extrinsic_actual_sha}" == "${IMU_PLANAR_EXTRINSIC_EXPECTED_SHA256}" ]] || \
  fail "planar IMU extrinsic report hash mismatch: expected=${IMU_PLANAR_EXTRINSIC_EXPECTED_SHA256}, actual=${imu_planar_extrinsic_actual_sha}"
if ! python3 - "${PLANNER_COMMON_CONFIG}" "${IMU_PLANAR_EXTRINSIC_REPORT}" <<'PY'
import json
import math
import sys

import yaml

common_path, report_path = sys.argv[1:]
with open(common_path, encoding="utf-8") as stream:
    imu = yaml.safe_load(stream)["imu_shadow"]
with open(report_path, encoding="utf-8") as stream:
    report = json.load(stream)

if report["decision"]["runtime_extrinsic_action"] != "KEEP_NOMINAL_PLANAR_EXTRINSIC":
    raise SystemExit("frozen report does not authorize the nominal runtime extrinsic")

nominal = report["nominal_runtime_extrinsic"]
fixed = report["fixed_processing_parameters"]
expected = {
    "imu_to_base_yaw_rad": math.radians(nominal["imu_to_base_yaw_deg"]),
    "lever_arm_imu_to_target_x_m": nominal["lever_arm_imu_to_tube_x_m"],
    "lever_arm_imu_to_target_y_m": nominal["lever_arm_imu_to_tube_y_m"],
    "sensor_delay_sec": fixed["imu_mocap_lag_sec"],
    "gyro_scale": fixed["gyro_scale"],
    "gyro_offset_radps": fixed["gyro_offset_radps"],
}
for key, expected_value in expected.items():
    actual_value = float(imu[key])
    if not math.isclose(actual_value, float(expected_value), rel_tol=0.0, abs_tol=1e-9):
        raise SystemExit(
            "{} mismatch: runtime={} frozen={}".format(
                key, actual_value, expected_value
            )
        )
if imu.get("expected_frame") != "imu_link":
    raise SystemExit("runtime expected_frame is not imu_link")
print("[G2S] Frozen planar IMU extrinsic/config matches common.yaml")
PY
then
  fail "runtime processed-IMU configuration does not match the frozen planar extrinsic report"
fi
case "${G2S_ROW}" in
  01|02|03|04) ;;
  *) fail "set G2S_ROW=01|02|03|04 (one paired physical unit per invocation)" ;;
esac
[[ "${G2S_ATTEMPT}" =~ ^[0-9][0-9]$ ]] || \
  fail "G2S_ATTEMPT must be two digits, for example 01"
[[ "${N_SRC}" == "4" ]] || fail "this preregistered development script fixes N_SRC=4"
[[ "${MIN_DIRECTIONAL_TRIALS}" == "3" ]] || \
  fail "this preregistered development script fixes MIN_DIRECTIONAL_TRIALS=3"
for integer_setting in ONLINE_LIQUID_PROCESS_EVERY ONLINE_LIQUID_ZERO_FRAMES \
  ONLINE_LIQUID_READY_TIMEOUT_SEC MIN_FREE_GIB; do
  [[ "${!integer_setting}" =~ ^[1-9][0-9]*$ ]] || \
    fail "${integer_setting} must be a positive integer, got ${!integer_setting}"
done
[[ -s "${H0S_PATH}" ]] || fail "frozen H0s path is missing: ${H0S_PATH}"
actual_path_sha="$(sha256sum "${H0S_PATH}" | awk '{print $1}')"
[[ "${actual_path_sha}" == "${H0S_SHA256}" ]] || \
  fail "H0s path hash mismatch: expected=${H0S_SHA256}, actual=${actual_path_sha}"
[[ -s "${RGB_CALIBRATION_FILE}" ]] || \
  fail "RGB calibration is missing: ${RGB_CALIBRATION_FILE}"
rgb_calibration_sha="$(sha256sum "${RGB_CALIBRATION_FILE}" | awk '{print $1}')"
online_launch_sha="$(sha256sum "${ONLINE_LIQUID_LAUNCH}" | awk '{print $1}')"
online_node_sha="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')"
online_detector_sha="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')"
online_msg_sha="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')"

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
online_config_sha="$(printf '%s\n' "${expected_online_config}" | sha256sum | awk '{print $1}')"

RGB_CAMERA_AUTO_EXPOSURE="UNRESOLVED"
RGB_CAMERA_EXPOSURE="UNRESOLVED"
RGB_CAMERA_GAIN="UNRESOLVED"
RGB_CAMERA_AUTO_WHITE_BALANCE="UNRESOLVED"
RGB_CAMERA_WHITE_BALANCE="UNRESOLVED"

RUN_LABEL="DEV_G2S_H0s_C1_Bsmooth_u${G2S_ROW}_a${G2S_ATTEMPT}"
BAG_PATH="${RUN_OUT_DIR}/${RUN_LABEL}.bag"

prereg_contents() {
  printf '%s\n' \
    "protocol=G2S_H0s_source_selection_development_v2_online_scalar" \
    "planned_paired_units=${N_SRC}" \
    "planned_execution_order=01,02,03,04" \
    "controller=Bsmooth" \
    "solver_consumes_liquid_state=false" \
    "paired_sources=odom,processed_imu" \
    "imu_planar_extrinsic_report=${IMU_PLANAR_EXTRINSIC_REPORT}" \
    "imu_planar_extrinsic_report_sha256=${imu_planar_extrinsic_actual_sha}" \
    "imu_planar_extrinsic_decision=KEEP_NOMINAL_PLANAR_EXTRINSIC" \
    "imu_to_base_yaw_rad=0.0" \
    "lever_arm_imu_to_tube_xy_m=-0.100,0.045" \
    "rgb_reference=online_stamped_scalar_with_quality" \
    "raw_or_debug_image_topics_in_bag=forbidden" \
    "path_alias=H0s_equals_H0_G2_geometry" \
    "path_file=${H0S_PATH}" \
    "path_sha256=${H0S_SHA256}" \
    "rgb_calibration_file=${RGB_CALIBRATION_FILE}" \
    "rgb_calibration_sha256=${rgb_calibration_sha}" \
    "rgb_expected_width=${RGB_EXPECTED_WIDTH}" \
    "rgb_expected_height=${RGB_EXPECTED_HEIGHT}" \
    "rgb_expected_fps=${RGB_EXPECTED_FPS}" \
    "rgb_dynparam_ns=${RGB_DYNPARAM_NS}" \
    "rgb_enable_auto_exposure=${RGB_CAMERA_AUTO_EXPOSURE}" \
    "rgb_exposure=${RGB_CAMERA_EXPOSURE}" \
    "rgb_gain=${RGB_CAMERA_GAIN}" \
    "rgb_enable_auto_white_balance=${RGB_CAMERA_AUTO_WHITE_BALANCE}" \
    "rgb_white_balance=${RGB_CAMERA_WHITE_BALANCE}" \
    "online_measurement_topic=${ONLINE_LIQUID_MEASUREMENT_TOPIC}" \
    "online_measurement_timestamp=image_header_stamp" \
    "online_minimum_recorded_rate_hz=${ONLINE_LIQUID_MIN_RATE_HZ}" \
    "online_measurement_protocol_sha256=${online_config_sha}" \
    "online_launch_sha256=${online_launch_sha}" \
    "online_node_sha256=${online_node_sha}" \
    "online_detector_sha256=${online_detector_sha}" \
    "online_message_sha256=${online_msg_sha}" \
    "primary_metric=trial_level_motion_window_mae_mm" \
    "observer_height_field=total_height_m" \
    "rgb_height_field=height_max_lcr_mm_centered_rolling_median_5" \
    "rgb_zero_correction_frames=${ONLINE_LIQUID_ZERO_FRAMES}" \
    "rgb_smooth_frames=5_clean_scalar_frames" \
    "rgb_invalid_clipped_or_zero_unlocked_frames=excluded" \
    "secondary_metrics=rmse_mm,h_vis_p95_abs_error_mm,peak_bias_mm,static_residual_mm,coverage" \
    "additional_odom_lag_sec=0.0" \
    "additional_processed_imu_lag_sec=0.0" \
    "per_trial_lag_scale_filter_tuning=forbidden" \
    "processed_imu_min_relative_improvement=${DELTA_SRC}" \
    "minimum_same_direction_trials=${MIN_DIRECTIONAL_TRIALS}" \
    "single_trial_dominance=forbidden" \
    "readiness_or_coverage_degradation=forbidden" \
    "tie_conflict_or_uncertainty_decision=odom"
}

echo "================ G2S H0s paired unit ================"
echo "  row/attempt = ${G2S_ROW}/${G2S_ATTEMPT}"
echo "  label       = ${RUN_LABEL}"
echo "  path        = ${H0S_PATH}"
echo "  path SHA    = ${H0S_SHA256}"
echo "  controller  = Bsmooth (liquid state not consumed)"
echo "  paired data = odom observer + processed-IMU observer + online RGB scalar"
echo "  IMU freeze  = KEEP_NOMINAL_PLANAR_EXTRINSIC (${imu_planar_extrinsic_actual_sha})"
echo "  bag images  = forbidden (raw/compressed/debug/depth all excluded)"
echo "  RGB freeze  = ${RGB_EXPECTED_WIDTH}x${RGB_EXPECTED_HEIGHT}@${RGB_EXPECTED_FPS}, ${RGB_CALIBRATION_FILE}"
echo "  calibration = ${rgb_calibration_sha}"
echo "  output      = ${BAG_PATH}"
echo "  decision    = IMU >10%, >=3/4 same direction, no coverage loss; otherwise odom"
echo "======================================================="

runner_env=(
  "DATE=${DATE}"
  "STAMP=${STAMP}"
  "PILOT_MODE=true"
  "PILOT_METHOD=Bsmooth"
  "PILOT_CONDITION=G2S_H0s_source_selection"
  "PILOT_RECORD_RGB=false"
  "PILOT_RECORD_ONLINE_LIQUID=true"
  "RUN_LABEL=${RUN_LABEL}"
  "NAME=${RUN_LABEL}"
  "RUN_OUT_DIR=${RUN_OUT_DIR}"
  "PATH_SOURCE_MODE=replay"
  "PATH_FILE=${H0S_PATH}"
  "PATH_EXPECTED_SHA256=${H0S_SHA256}"
  "REQUIRE_PATH_HASH=true"
  "START_POS_TOL=0.08"
  "START_YAW_TOL=0.15"
  "START_HOLD_SEC=0.5"
  "V_REF=0.20"
  "DELAY_PHASE_MODE=fixed_closed_loop"
  "DELAY_PHASE_LINEAR_DELAY_SEC=0.15"
  "DELAY_PHASE_ANGULAR_DELAY_SEC=0.22"
  "IMU_SHADOW_ENABLE=true"
  "CURRENT_OBSERVER_SOURCE=odom"
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
  "RGB_CALIBRATION_EXPECTED_SHA256=${rgb_calibration_sha}"
  "RGB_EXPECTED_WIDTH=${RGB_EXPECTED_WIDTH}"
  "RGB_EXPECTED_HEIGHT=${RGB_EXPECTED_HEIGHT}"
  "RGB_EXPECTED_FPS=${RGB_EXPECTED_FPS}"
  "ONLINE_LIQUID_MEASUREMENT_TOPIC=${ONLINE_LIQUID_MEASUREMENT_TOPIC}"
  "ONLINE_LIQUID_PROTOCOL=G2S_H0s_source_selection_development_v2_online_scalar"
  "ONLINE_LIQUID_CALIBRATION_SHA256=${rgb_calibration_sha}"
  "ONLINE_LIQUID_DETECTOR_SHA256=${online_detector_sha}"
  "ONLINE_LIQUID_NODE_SHA256=${online_node_sha}"
  "ONLINE_LIQUID_MSG_SHA256=${online_msg_sha}"
  "ONLINE_LIQUID_CONFIG_SHA256=${online_config_sha}"
  "RECORD_SEC=${RECORD_SEC}"
  "MAX_RECORD_SEC=${RECORD_SEC}"
  "BLOCK_SEGMENT_ID=G2S_u${G2S_ROW}"
  "ORDER_POSITION=${G2S_ROW}"
  "OPERATOR_NOTE=development_G2S_paired_odom_processed_imu_online_RGB_scalar_no_images"
)

if truthy "${VALIDATE_ONLY}"; then
  printf '[G2S] validate-only command:\n  env '
  printf '%q ' "${runner_env[@]}"
  printf 'bash %q\n' "${RUNNER}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || \
  fail "real motion is disarmed; rerun with ARM_MOTION=YES after clearing the path"
[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || \
  fail "set CONFIRM_RGB_GEOMETRY=YES only if the frozen calibration still matches camera/container pose and liquid geometry"
[[ ! -e "${BAG_PATH}" && ! -e "${BAG_PATH}.active" ]] || \
  fail "attempt output already exists: ${BAG_PATH}"

if (( 10#${G2S_ATTEMPT} > 1 )); then
  [[ -n "${RETRY_REASON_FILE:-}" && -s "${RETRY_REASON_FILE}" ]] || \
    fail "attempt >01 requires a non-empty RETRY_REASON_FILE"
  runner_env+=("ACQUISITION_RETRY=true" "RETRY_REASON_FILE=${RETRY_REASON_FILE}")
fi

command -v rostopic >/dev/null 2>&1 || fail "rostopic is unavailable"
command -v roslaunch >/dev/null 2>&1 || fail "roslaunch is unavailable"
command -v rosrun >/dev/null 2>&1 || fail "rosrun is unavailable"
camera_info="$(timeout 6s rostopic echo -n 1 "${RGB_CAMERA_INFO_TOPIC}" 2>/dev/null)" || \
  fail "no camera_info received on ${RGB_CAMERA_INFO_TOPIC}"
actual_width="$(awk '$1 == "width:" {print $2; exit}' <<< "${camera_info}")"
actual_height="$(awk '$1 == "height:" {print $2; exit}' <<< "${camera_info}")"
[[ "${actual_width}" == "${RGB_EXPECTED_WIDTH}" &&
   "${actual_height}" == "${RGB_EXPECTED_HEIGHT}" ]] || \
  fail "RGB resolution mismatch: expected ${RGB_EXPECTED_WIDTH}x${RGB_EXPECTED_HEIGHT}, got ${actual_width:-?}x${actual_height:-?}"
timeout 6s rostopic echo -n 1 "${RGB_IMAGE_TOPIC}" >/dev/null 2>&1 || \
  fail "no raw RGB frame received on ${RGB_IMAGE_TOPIC}"

camera_dynparam="$(rosrun dynamic_reconfigure dynparam get "${RGB_DYNPARAM_NS}" 2>/dev/null)" || \
  fail "cannot read RealSense RGB dynamic config: ${RGB_DYNPARAM_NS}"
get_dynparam() {
  local key="$1"
  awk -F': ' -v key="${key}" '$1 == key {print $2; found=1; exit} END {if (!found) exit 1}' \
    <<< "${camera_dynparam}"
}
RGB_CAMERA_AUTO_EXPOSURE="$(get_dynparam enable_auto_exposure)" || \
  fail "missing enable_auto_exposure in ${RGB_DYNPARAM_NS}"
RGB_CAMERA_EXPOSURE="$(get_dynparam exposure)" || fail "missing exposure in ${RGB_DYNPARAM_NS}"
RGB_CAMERA_GAIN="$(get_dynparam gain)" || fail "missing gain in ${RGB_DYNPARAM_NS}"
RGB_CAMERA_AUTO_WHITE_BALANCE="$(get_dynparam enable_auto_white_balance)" || \
  fail "missing enable_auto_white_balance in ${RGB_DYNPARAM_NS}"
RGB_CAMERA_WHITE_BALANCE="$(get_dynparam white_balance)" || \
  fail "missing white_balance in ${RGB_DYNPARAM_NS}"
case "${RGB_CAMERA_AUTO_EXPOSURE,,}" in false|0) ;; *)
  fail "RGB auto exposure must be disabled, got ${RGB_CAMERA_AUTO_EXPOSURE}" ;;
esac
case "${RGB_CAMERA_AUTO_WHITE_BALANCE,,}" in false|0) ;; *)
  fail "RGB auto white balance must be disabled, got ${RGB_CAMERA_AUTO_WHITE_BALANCE}" ;;
esac

available_kib="$(df -Pk /home/geist/slosh_bags/real | awk 'NR == 2 {print $4}')"
required_kib=$((MIN_FREE_GIB * 1024 * 1024))
(( available_kib >= required_kib )) || \
  fail "insufficient free space: require at least ${MIN_FREE_GIB} GiB before a G2S scalar unit"

mkdir -p "${RUN_OUT_DIR}"
expected_prereg="$(prereg_contents)"
if [[ -e "${PREREG_FILE}" ]]; then
  actual_prereg="$(<"${PREREG_FILE}")"
  [[ "${actual_prereg}" == "${expected_prereg}" ]] || \
    fail "existing prereg differs from current path/camera/online inputs: ${PREREG_FILE}"
elif [[ "${G2S_ROW}" != "01" ]]; then
  fail "row 01 must create the prereg before row ${G2S_ROW}: ${PREREG_FILE}"
fi
if [[ ! -e "${PREREG_FILE}" ]]; then
  printf '%s\n' "${expected_prereg}" > "${PREREG_FILE}"
  sha256sum "${PREREG_FILE}" > "${PREREG_FILE}.sha256"
fi
if [[ -e "${ONLINE_CONFIG_FILE}" ]]; then
  actual_online_config="$(<"${ONLINE_CONFIG_FILE}")"
  [[ "${actual_online_config}" == "${expected_online_config}" ]] || \
    fail "online RGB config changed since row 01: ${ONLINE_CONFIG_FILE}"
else
  printf '%s\n' "${expected_online_config}" > "${ONLINE_CONFIG_FILE}"
  sha256sum "${ONLINE_CONFIG_FILE}" > "${ONLINE_CONFIG_FILE}.sha256"
fi

publisher_count() {
  rostopic info "$1" 2>/dev/null | awk '
    /^Publishers:/ {in_publishers=1; next}
    /^Subscribers:/ {in_publishers=0}
    in_publishers && /^[[:space:]]+\*/ {count++}
    END {print count+0}
  ' || printf '0\n'
}
for output_topic in "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" /liquid/height \
  /liquid/height_lcr /liquid/height_median /liquid/debug_image; do
  count="$(publisher_count "${output_topic}")"
  [[ "${count}" == "0" ]] || \
    fail "unexpected pre-existing publisher(s) on ${output_topic}: ${count}; stop the old online RGB node"
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

echo "[G2S] Starting frozen online RGB scalar node (publish_debug=false)."
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
[[ "${online_type}" == "realsense_liquid_measurement/OnlineLiquidMeasurement" ]] || {
  tail -80 "${online_log}" >&2 || true
  fail "unexpected or missing ${ONLINE_LIQUID_MEASUREMENT_TOPIC} type: ${online_type:-none}"
}

deadline=$((SECONDS + ONLINE_LIQUID_READY_TIMEOUT_SEC))
measurement_sample=""
while (( SECONDS < deadline )); do
  measurement_sample="$(timeout 2s rostopic echo -n 1 "${ONLINE_LIQUID_MEASUREMENT_TOPIC}" 2>/dev/null || true)"
  if grep -Eq '^valid: (True|true)$' <<< "${measurement_sample}" && \
     grep -Eq '^zero_locked: (True|true)$' <<< "${measurement_sample}" && \
     grep -Eq '^status_code: 0$' <<< "${measurement_sample}"; then
    break
  fi
done
grep -Eq '^valid: (True|true)$' <<< "${measurement_sample}" || {
  tail -80 "${online_log}" >&2 || true
  fail "online RGB did not reach a clean, zero-locked valid measurement within ${ONLINE_LIQUID_READY_TIMEOUT_SEC}s"
}
measurement_width="$(awk '$1 == "image_width:" {print $2; exit}' <<< "${measurement_sample}")"
measurement_height="$(awk '$1 == "image_height:" {print $2; exit}' <<< "${measurement_sample}")"
[[ "${measurement_width}" == "${RGB_EXPECTED_WIDTH}" && \
   "${measurement_height}" == "${RGB_EXPECTED_HEIGHT}" ]] || \
  fail "online measurement resolution mismatch: ${measurement_width:-?}x${measurement_height:-?}"
echo "[G2S] Online RGB READY: stamped scalar is valid; no image topic will enter the bag."

echo "[G2S] Keep the robot still for the recorded IMU bias window."
echo "[G2S] The underlying runner releases H0_G2 only after IMU READY."
env "${runner_env[@]}" bash "${RUNNER}"

postflight_args=(
  "--bag" "${BAG_PATH}"
  "--expected-width" "${RGB_EXPECTED_WIDTH}"
  "--expected-height" "${RGB_EXPECTED_HEIGHT}"
  "--expected-fps" "${RGB_EXPECTED_FPS}"
  "--process-every" "${ONLINE_LIQUID_PROCESS_EVERY}"
  "--zero-frames" "${ONLINE_LIQUID_ZERO_FRAMES}"
  "--min-online-rate-hz" "${ONLINE_LIQUID_MIN_RATE_HZ}"
  "--measurement-topic" "${ONLINE_LIQUID_MEASUREMENT_TOPIC}"
)
if truthy "${POSTFLIGHT_HASH_BAG}"; then
  postflight_args+=("--hash-bag")
fi
python3 "${POSTFLIGHT_VALIDATOR}" "${postflight_args[@]}"

echo "[G2S] unit finished: ${BAG_PATH}"
echo "[G2S] Do not start the next row until the robot is back at the start mark, aligned, and the liquid is settled."
