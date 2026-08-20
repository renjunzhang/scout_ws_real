#!/usr/bin/env bash
# G3R2: one Bsmooth smoke for robot-only delay compensation.
#
# This release intentionally keeps the full G3R2 candidate screen closed.  It
# first proves that predicting only SolverInput.robot restores frozen-path
# tracking while SolverInput.slosh remains the current processed-IMU state.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G3R2][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
VALIDATOR="${SCRIPT_DIR}/../tools/analysis/validate_g3_online_rgb_trial.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"

[[ -r /opt/ros/noetic/setup.bash ]] || fail "missing ROS Noetic setup"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]] || fail "missing ${REPO_ROOT}/devel/setup.bash"
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

G3R2_ROW="${G3R2_ROW:-01}"
G3R2_ATTEMPT="${G3R2_ATTEMPT:-01}"
DATE="${DATE:-20260801}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ALLOW_DIRTY_VALIDATE_ONLY="${ALLOW_DIRTY_VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"

[[ "${G3R2_ROW}" == "01" ]] || fail "only G3R2_ROW=01 Bsmooth smoke is released"
[[ "${G3R2_ATTEMPT}" == "01" ]] || fail "G3R2 smoke permits only planned attempt 01"

PATH_FILE="/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json"
PATH_SHA256="578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e"
SOURCE_REPORT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/analysis/G2S_RAW_RGB_THREE_TRIAL_DEVELOPMENT_REPORT.json"
SOURCE_REPORT_SHA256="36db4d12948f98e4ef3580bc52367cdef14d8440d80a8afbb32182b704ee5442"

FAILED_G3R_ROOT="/home/geist/slosh_bags/real/20260801_spmpc_g3r_raw_imu_weight_screen/H0"
FAILED_G3R_BAG="${FAILED_G3R_ROOT}/DEV_G3R_H0_C1_Bsmooth_r01_a01.bag"
FAILED_G3R_BAG_SHA256="b23cbe647a31392d08ec5c743310baf8dd384feac080f03d5d36cbb4430550bd"
FAILED_G3R_REPORT="${FAILED_G3R_ROOT}/DEV_G3R_H0_C1_Bsmooth_r01_a01_g3r_postflight.json"
FAILED_G3R_REPORT_SHA256="10d2392b3cb35459840d105b3e062d1d369453674689500d6cdccd7e94cbdf37"

RGB_CALIBRATION_FILE="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml"
RGB_CALIBRATION_SHA256="7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01"
RGB_CAMERA_PARAMS_FILE="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/camera_params/realsense_rgb_fixed_params_20260731_203322.yaml"
RGB_CAMERA_PARAMS_SHA256="0fac946203c4e7592a3fd4e5302d92e4339a074d9cbb4094ecf309b69f84c4d9"
RGB_IMAGE_TOPIC="/camera/color/image_raw"
RGB_MEASUREMENT_TOPIC="/liquid/measurement"
ONLINE_LIQUID_LAUNCH="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/launch/online_liquid_height.launch"
ONLINE_LIQUID_NODE="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py"
ONLINE_LIQUID_DETECTOR="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py"
ONLINE_LIQUID_MSG="${REPO_ROOT}/src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg"

PLANNER_LIBRARY="${REPO_ROOT}/devel/lib/libspmpc_local_planner.so"
PLANNER_LIBRARY_SHA256="e98fe1e6146604d631ded2cdb80f8ba7601458b75d4f1b94f5c371ea8b24243f"
PLANNER_NODE="${REPO_ROOT}/devel/lib/spmpc_local_planner/spmpc_local_planner_node"
PLANNER_NODE_SHA256="6b99888aed987096bef28095a4c8cfc9fc25bf6ebc3680c4bf02ce4bc7892657"
SLOSH_MODELS_LIBRARY="${REPO_ROOT}/devel/lib/libslosh_models.so"
SLOSH_MODELS_LIBRARY_SHA256="0e148ece8eae0552504112530b7fe57c171ea7d2ec0f11d4096bac11e41ad15e"
ACADOS_B0_SOLVER="${REPO_ROOT}/src/scout_apps/control/spmpc_local_planner/generated/acados/spmpc_b0/libacados_ocp_solver_spmpc_b0.so"
ACADOS_B0_SOLVER_SHA256="5c6e00deeb39bd5e47951c12bc6a4f7385eac15504287b40c1eff27599edfb91"
ACADOS_SLOSH_SOLVER="${REPO_ROOT}/src/scout_apps/control/spmpc_local_planner/generated/acados/spmpc_slosh/libacados_ocp_solver_spmpc_slosh.so"
ACADOS_SLOSH_SOLVER_SHA256="156dc60556b64c184d830ee3b6f13ab373eb38947b1ad9866418a44589dba560"
ACADOS_LIBRARY="/home/geist/acados/lib/libacados.so"
ACADOS_LIBRARY_SHA256="4c706ad1d3be2a4a821a7fc2a56e034f813593929c3e06b5671f6214bb3f0b9d"
HPIPM_LIBRARY="/home/geist/acados/lib/libhpipm.so"
HPIPM_LIBRARY_SHA256="9ff19f602080dd309b6413fbf58944d07ab70d11966731edd48ee3cf1e70efa8"
BLASFEO_LIBRARY="/home/geist/acados/lib/libblasfeo.so.0"
BLASFEO_LIBRARY_SHA256="eac90d057d8c0ef42f662dd1afd1b190f663ba3c6c12aacacfea9dd2ab268212"

RECORD_SEC=70
T_HVIS_TAIL=5.0
T_MOTION_MAX=42
MAX_PRE_MOTION_SEC=23
MIN_FREE_GIB="${MIN_FREE_GIB:-5}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g3r2_robot_only_smoke/H0}"
PREREG_FILE="${RUN_OUT_DIR}/G3R2_robot_only_smoke_prereg.env"
METRIC_FILE="${RUN_OUT_DIR}/G3R2_robot_only_smoke_metric.yaml"
ONLINE_CONFIG_FILE="${RUN_OUT_DIR}/G3R2_online_liquid_config.env"

required_files=(
  "${RUNNER}" "${VALIDATOR}" "${SUMMARIZER}" "${CAMERA_PREP}"
  "${PATH_FILE}" "${SOURCE_REPORT}" "${FAILED_G3R_BAG}" "${FAILED_G3R_REPORT}"
  "${RGB_CALIBRATION_FILE}" "${RGB_CAMERA_PARAMS_FILE}"
  "${ONLINE_LIQUID_LAUNCH}" "${ONLINE_LIQUID_NODE}" "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}"
  "${PLANNER_LIBRARY}" "${PLANNER_NODE}" "${SLOSH_MODELS_LIBRARY}"
  "${ACADOS_B0_SOLVER}" "${ACADOS_SLOSH_SOLVER}" "${ACADOS_LIBRARY}" "${HPIPM_LIBRARY}" "${BLASFEO_LIBRARY}"
)
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done

verify_sha256() {
  local path="$1"
  local expected="$2"
  local label="$3"
  local actual
  actual="$(sha256sum "${path}" | awk '{print $1}')"
  [[ "${actual}" == "${expected}" ]] || \
    fail "${label} hash mismatch: expected=${expected}, actual=${actual}"
}

verify_sha256 "${PATH_FILE}" "${PATH_SHA256}" "path"
verify_sha256 "${SOURCE_REPORT}" "${SOURCE_REPORT_SHA256}" "source report"
verify_sha256 "${FAILED_G3R_BAG}" "${FAILED_G3R_BAG_SHA256}" "failed G3R bag"
verify_sha256 "${FAILED_G3R_REPORT}" "${FAILED_G3R_REPORT_SHA256}" "failed G3R postflight"
verify_sha256 "${RGB_CALIBRATION_FILE}" "${RGB_CALIBRATION_SHA256}" "RGB calibration"
verify_sha256 "${RGB_CAMERA_PARAMS_FILE}" "${RGB_CAMERA_PARAMS_SHA256}" "RealSense parameters"

runtime_artifacts=(
  "${PLANNER_LIBRARY}|${PLANNER_LIBRARY_SHA256}|planner library"
  "${PLANNER_NODE}|${PLANNER_NODE_SHA256}|planner node"
  "${SLOSH_MODELS_LIBRARY}|${SLOSH_MODELS_LIBRARY_SHA256}|slosh-models library"
  "${ACADOS_B0_SOLVER}|${ACADOS_B0_SOLVER_SHA256}|acados B0 solver"
  "${ACADOS_SLOSH_SOLVER}|${ACADOS_SLOSH_SOLVER_SHA256}|acados slosh solver"
  "${ACADOS_LIBRARY}|${ACADOS_LIBRARY_SHA256}|acados runtime"
  "${HPIPM_LIBRARY}|${HPIPM_LIBRARY_SHA256}|HPIPM runtime"
  "${BLASFEO_LIBRARY}|${BLASFEO_LIBRARY_SHA256}|BLASFEO runtime"
)
for artifact_binding in "${runtime_artifacts[@]}"; do
  IFS='|' read -r artifact_path expected_sha artifact_label <<< "${artifact_binding}"
  verify_sha256 "${artifact_path}" "${expected_sha}" "${artifact_label}"
done

python3 - "${FAILED_G3R_REPORT}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("status") != "FAIL":
    raise SystemExit("bound G3R evidence is not the failed smoke")
if report.get("protocol") != "G3R_raw_processed_imu_weight_screen_v1":
    raise SystemExit("bound G3R evidence has the wrong protocol")
if report.get("row") != "01" or report.get("condition") != "Bsmooth":
    raise SystemExit("bound G3R evidence is not Bsmooth row 01")
tracking = report.get("tracking", {})
if float(tracking.get("contour_p95_m", 0.0)) <= 0.05:
    raise SystemExit("bound failure does not prove contour regression")
if float(tracking.get("yaw_p95_rad", 0.0)) <= 0.15:
    raise SystemExit("bound failure does not prove yaw regression")
internal = report.get("internal_state", {})
if float(internal.get("delay_compensation_applied_fraction", 1.0)) > 0.02:
    raise SystemExit("bound failure unexpectedly used closed-loop delay compensation")
config = report.get("effective_config_last", {})
if float(config.get("delay_phase_mode_code", -1.0)) != 2.0:
    raise SystemExit("bound failure was not the delay-shadow release")
PY

release_revision="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
relevant_repo_paths=(
  src/scout_apps/control/spmpc_local_planner/CMakeLists.txt
  src/scout_apps/control/spmpc_local_planner/config
  src/scout_apps/control/spmpc_local_planner/include
  src/scout_apps/control/spmpc_local_planner/launch
  src/scout_apps/control/spmpc_local_planner/msg
  src/scout_apps/control/spmpc_local_planner/src
  src/scout_apps/control/slosh_models
  src/scout_apps/control/spmpc_local_planner/scripts/record_spmpc_full_rgb_bag.sh
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_real_fixed_path_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_robot_only_smoke_trial.sh
  src/scout_apps/control/spmpc_local_planner/tools/analysis/validate_g3_online_rgb_trial.py
  src/scout_apps/control/spmpc_local_planner/scripts/prepare_spmpc_g3_realsense.sh
  src/scout_apps/control/spmpc_local_planner/scripts/summarize_spmpc_real_trial.py
  src/scout_apps/sensors/realsense_liquid_measurement/launch/online_liquid_height.launch
  src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg
  src/scout_apps/sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py
  src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py
)
tracked_missing=()
for path in "${relevant_repo_paths[@]}"; do
  if [[ -f "${REPO_ROOT}/${path}" ]] && ! git -C "${REPO_ROOT}" ls-files --error-unmatch "${path}" >/dev/null 2>&1; then
    tracked_missing+=("${path}")
  fi
done
if (( ${#tracked_missing[@]} > 0 )); then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G3R2][WARN] developer validate-only with untracked code: ${tracked_missing[*]}" >&2
  else
    fail "G3R2 code is not frozen in revision ${release_revision}: ${tracked_missing[*]}"
  fi
fi
if ! git -C "${REPO_ROOT}" diff --quiet -- "${relevant_repo_paths[@]}" || \
   ! git -C "${REPO_ROOT}" diff --cached --quiet -- "${relevant_repo_paths[@]}"; then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G3R2][WARN] developer validate-only with dirty relevant paths" >&2
  else
    fail "G3R2 runtime differs from release ${release_revision}; commit/freeze before motion"
  fi
fi

online_node_sha="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')"
online_detector_sha="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')"
online_msg_sha="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')"

metric_contents() {
  printf '%s\n' \
    "schema_version: 1" \
    "purpose: restore_path_tracking_before_reopening_weight_screen" \
    "max_contour_p95_m: 0.05" \
    "max_yaw_p95_rad: 0.15" \
    "max_motion_to_arrival_sec: ${T_MOTION_MAX}" \
    "robot_delay_compensation_min_fraction: 0.98" \
    "liquid_delay_compensation_max_fraction: 0.02" \
    "processed_imu_source_min_fraction: 0.98" \
    "processed_imu_ready_min_fraction: 0.98" \
    "t_hvis_tail_sec: ${T_HVIS_TAIL}" \
    "maximum_record_to_motion_sec: ${MAX_PRE_MOTION_SEC}" \
    "image_stream_policy: forbid_all"
}

online_config_contents() {
  printf '%s\n' \
    "protocol=G3R2_robot_only_delay_smoke_v1" \
    "measurement_topic=${RGB_MEASUREMENT_TOPIC}" \
    "image_topic=${RGB_IMAGE_TOPIC}" \
    "process_every=1" \
    "zero_frames=30" \
    "publish_debug=false" \
    "height_bias_mm=0.0" \
    "hue1=0:12" \
    "hue2=161:179" \
    "sat_min=101" \
    "val_min=167"
}

expected_metric="$(metric_contents)"
expected_online_config="$(online_config_contents)"
OUTCOME_WINDOW_RULE_SHA256="$(printf '%s\n' "${expected_metric}" | sha256sum | awk '{print $1}')"
ONLINE_CONFIG_SHA256="$(printf '%s\n' "${expected_online_config}" | sha256sum | awk '{print $1}')"
RUNTIME_BINARY_BUNDLE_SHA256="$(printf '%s\n' \
  "planner_library=${PLANNER_LIBRARY_SHA256}" \
  "planner_node=${PLANNER_NODE_SHA256}" \
  "slosh_models_library=${SLOSH_MODELS_LIBRARY_SHA256}" \
  "acados_b0_solver=${ACADOS_B0_SOLVER_SHA256}" \
  "acados_slosh_solver=${ACADOS_SLOSH_SOLVER_SHA256}" \
  "acados_runtime=${ACADOS_LIBRARY_SHA256}" \
  "hpipm_runtime=${HPIPM_LIBRARY_SHA256}" \
  "blasfeo_runtime=${BLASFEO_LIBRARY_SHA256}" | sha256sum | awk '{print $1}')"

prereg_contents() {
  printf '%s\n' \
    "protocol=G3R2_robot_only_delay_smoke_v1" \
    "scope=development_tracking_smoke_only" \
    "formal_stage_status=NO_GO" \
    "planned_units=1" \
    "released_condition=Bsmooth" \
    "later_candidates_released=false" \
    "release_revision=${release_revision}" \
    "failed_g3r_bag_sha256=${FAILED_G3R_BAG_SHA256}" \
    "failed_g3r_report_sha256=${FAILED_G3R_REPORT_SHA256}" \
    "source_report_sha256=${SOURCE_REPORT_SHA256}" \
    "path_sha256=${PATH_SHA256}" \
    "delay_phase_mode=fixed_robot_only" \
    "delay_phase_mode_code=4" \
    "robot_state=predicted_from_command_history" \
    "liquid_state=current_processed_imu_measurement" \
    "robot_delay_compensation=true" \
    "liquid_delay_compensation=false" \
    "observer_fallback_policy=fail_closed" \
    "imu_subscriber_queue_size=10" \
    "v_ref=0.20" \
    "outcome_window_rule_sha256=${OUTCOME_WINDOW_RULE_SHA256}" \
    "online_config_sha256=${ONLINE_CONFIG_SHA256}" \
    "runtime_binary_bundle_sha256=${RUNTIME_BINARY_BUNDLE_SHA256}" \
    "record_sec=${RECORD_SEC}" \
    "pass_required_before_candidate_release=true"
}

expected_prereg="$(prereg_contents)"
PREREG_SHA256="$(printf '%s\n' "${expected_prereg}" | sha256sum | awk '{print $1}')"

RUN_LABEL="DEV_G3R2_H0_C1_Bsmooth_robot_only_r01_a${G3R2_ATTEMPT}"
BAG_PATH="${RUN_OUT_DIR}/${RUN_LABEL}.bag"
runner_env=(
  "DATE=${DATE}" "STAMP=${STAMP}" "PILOT_MODE=true" "PILOT_METHOD=Bsmooth"
  "PILOT_CONDITION=G3R2_robot_only_delay_smoke"
  "PILOT_RECORD_RGB=false" "PILOT_RECORD_ONLINE_LIQUID=true"
  "RUN_LABEL=${RUN_LABEL}" "NAME=${RUN_LABEL}" "RUN_OUT_DIR=${RUN_OUT_DIR}"
  "PATH_SOURCE_MODE=replay" "PATH_FILE=${PATH_FILE}" "PATH_EXPECTED_SHA256=${PATH_SHA256}"
  "REQUIRE_PATH_HASH=true" "START_POS_TOL=0.08" "START_YAW_TOL=0.15"
  "START_HOLD_SEC=0.5" "START_GATE_TIMEOUT_SEC=3" "V_REF=0.20"
  "W_SLOSH=0.0" "W_SMOOTH=1.0" "W_ALPHA=1.0" "W_DU_A=1.0" "W_DU_VS=1.0"
  "DELAY_PHASE_MODE=fixed_robot_only" "DELAY_PHASE_LINEAR_DELAY_SEC=0.15"
  "DELAY_PHASE_ANGULAR_DELAY_SEC=0.22"
  "IMU_SHADOW_ENABLE=true" "IMU_SUBSCRIBER_QUEUE_SIZE=10" "IMU_SHADOW_READY_TIMEOUT_SEC=12"
  "CURRENT_OBSERVER_SOURCE=processed_imu" "OBSERVER_FALLBACK_POLICY=fail_closed"
  "OBSERVER_LATCH_FALLBACK=false"
  "RECORD_RGB=false" "RECORD_CAMERA=false" "RECORD_CAMERA_INFO=true"
  "RECORD_CAMERA_COMPRESSED=false" "RECORD_DEPTH=false"
  "RECORD_ONLINE_LIQUID=true" "RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false"
  "FORBID_IMAGE_STREAMS=true" "RECORD_ALL_EXISTING_TOPICS=false"
  "RGB_CALIBRATION_FILE=${RGB_CALIBRATION_FILE}"
  "RGB_CALIBRATION_EXPECTED_SHA256=${RGB_CALIBRATION_SHA256}"
  "RGB_EXPECTED_WIDTH=1920" "RGB_EXPECTED_HEIGHT=1080" "RGB_EXPECTED_FPS=30"
  "ONLINE_LIQUID_MEASUREMENT_TOPIC=${RGB_MEASUREMENT_TOPIC}"
  "ONLINE_LIQUID_PROTOCOL=G3R2_robot_only_delay_smoke_v1"
  "ONLINE_LIQUID_CALIBRATION_SHA256=${RGB_CALIBRATION_SHA256}"
  "ONLINE_LIQUID_DETECTOR_SHA256=${online_detector_sha}"
  "ONLINE_LIQUID_NODE_SHA256=${online_node_sha}" "ONLINE_LIQUID_MSG_SHA256=${online_msg_sha}"
  "ONLINE_LIQUID_CONFIG_SHA256=${ONLINE_CONFIG_SHA256}"
  "RECORD_SEC=${RECORD_SEC}" "MAX_RECORD_SEC=${RECORD_SEC}"
  "BLOCK_SEGMENT_ID=G3R2_robot_only_smoke" "ORDER_POSITION=01" "SEND_ZERO_ON_EXIT=true"
  "OPERATOR_NOTE=G3R2_robot_only_delay_failed_G3R_${FAILED_G3R_REPORT_SHA256}"
)

echo "================ G3R2 robot-only delay smoke ================"
echo "  row/condition = 01/Bsmooth"
echo "  robot state   = fixed delay-predicted odom/TF"
echo "  liquid state  = current processed-IMU (no delay rollout)"
echo "  mode/code     = fixed_robot_only / 4"
echo "  output        = ${BAG_PATH}"
echo "  next gate     = PASS before any weight candidate is released"
echo "=============================================================="

validate_launch_contract() {
  local launch_dump
  launch_dump="$(roslaunch --dump-params spmpc_local_planner spmpc_fixed_path.launch \
    planner_variant:=B_smooth \
    delay_phase_mode:=fixed_robot_only \
    delay_phase_linear_delay_sec:=0.15 \
    delay_phase_angular_delay_sec:=0.22 \
    imu_shadow_enable:=true \
    imu_subscriber_queue_size:=10 \
    observer_source:=processed_imu \
    observer_fallback_policy:=fail_closed \
    observer_latch_fallback:=false \
    v_ref:=0.20 \
    w_slosh:=0.0 \
    w_smooth:=1.0 \
    w_alpha:=1.0 \
    w_du_a:=1.0 \
    w_du_vs:=1.0)" || fail "could not dump the frozen launch parameters"

  local expected_lines=(
    "/spmpc_local_planner/planner_variant: B_smooth"
    "/spmpc_local_planner/delay_phase/mode: fixed_robot_only"
    "/spmpc_local_planner/delay_phase/linear_delay_sec: 0.15"
    "/spmpc_local_planner/delay_phase/angular_delay_sec: 0.22"
    "/spmpc_local_planner/imu_shadow/enable: true"
    "/spmpc_local_planner/imu_shadow/subscriber_queue_size: 10"
    "/spmpc_local_planner/slosh_observer/source: processed_imu"
    "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
    "/spmpc_local_planner/slosh_observer/latch_fallback: false"
    "/spmpc_local_planner/variants/B_smooth/w_slosh: 0.0"
    "/spmpc_local_planner/variants/B_smooth/w_smooth: 1.0"
    "/spmpc_local_planner/variants/B_smooth/w_alpha: 1.0"
    "/spmpc_local_planner/variants/B_smooth/w_du_a: 1.0"
    "/spmpc_local_planner/variants/B_smooth/w_du_vs: 1.0"
  )
  local expected_line
  for expected_line in "${expected_lines[@]}"; do
    grep -Fqx -- "${expected_line}" <<< "${launch_dump}" || \
      fail "launch contract missing: ${expected_line}"
  done
  echo "[G3R2] launch contract PASS (no planner started)"
}

validate_launch_contract

if truthy "${VALIDATE_ONLY}"; then
  printf '[G3R2] validate-only runner command:\n  env '
  printf '%q ' "${runner_env[@]}"
  printf 'bash %q\n' "${RUNNER}"
  echo "[G3R2] failed evidence SHA256 = ${FAILED_G3R_REPORT_SHA256}"
  echo "[G3R2] prereg SHA256          = ${PREREG_SHA256}"
  echo "[G3R2] runtime SHA256         = ${RUNTIME_BINARY_BUNDLE_SHA256}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || fail "real motion is disarmed; set ARM_MOTION=YES after clearing the path"
[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || fail "set CONFIRM_RGB_GEOMETRY=YES after checking ROI/rulers/container"
[[ ! -e "${BAG_PATH}" && ! -e "${BAG_PATH}.active" ]] || fail "attempt output already exists: ${BAG_PATH}"

available_kib="$(df -Pk /home/geist/slosh_bags/real | awk 'NR == 2 {print $4}')"
required_kib=$((MIN_FREE_GIB * 1024 * 1024))
(( available_kib >= required_kib )) || fail "insufficient free space: require ${MIN_FREE_GIB} GiB"

mkdir -p "${RUN_OUT_DIR}"
if [[ -e "${PREREG_FILE}" ]]; then
  [[ "$(<"${PREREG_FILE}")" == "${expected_prereg}" ]] || fail "G3R2 prereg changed"
  [[ "$(<"${METRIC_FILE}")" == "${expected_metric}" ]] || fail "G3R2 metric changed"
  [[ "$(<"${ONLINE_CONFIG_FILE}")" == "${expected_online_config}" ]] || fail "G3R2 online config changed"
else
  printf '%s\n' "${expected_prereg}" > "${PREREG_FILE}"
  printf '%s\n' "${expected_metric}" > "${METRIC_FILE}"
  printf '%s\n' "${expected_online_config}" > "${ONLINE_CONFIG_FILE}"
  sha256sum "${PREREG_FILE}" "${METRIC_FILE}" "${ONLINE_CONFIG_FILE}" > "${RUN_OUT_DIR}/G3R2_prereg_bundle.sha256"
fi

printf '%s\n' \
  "row=01" "condition=Bsmooth" "delay_phase_mode=fixed_robot_only" \
  "failed_g3r_report_sha256=${FAILED_G3R_REPORT_SHA256}" \
  "prereg_sha256=${PREREG_SHA256}" \
  > "${RUN_OUT_DIR}/${RUN_LABEL}_g3r2_binding.env"

bash "${CAMERA_PREP}"

publisher_count() {
  local topic_info
  if ! topic_info="$(rostopic info "$1" 2>/dev/null)"; then printf '0\n'; return 0; fi
  awk '/^Publishers:/ {inside=1; next} /^Subscribers:/ {inside=0} inside && /^[[:space:]]+\*/ {count++} END {print count+0}' <<< "${topic_info}"
}
for output_topic in "${RGB_MEASUREMENT_TOPIC}" /liquid/height /liquid/height_lcr /liquid/height_median /liquid/debug_image; do
  count="$(publisher_count "${output_topic}")"
  [[ "${count}" == "0" ]] || fail "unexpected pre-existing publisher(s) on ${output_topic}: ${count}"
done

online_log="${RUN_OUT_DIR}/${RUN_LABEL}_online_liquid.log"
online_pid=""
cleanup_online() {
  if [[ -n "${online_pid}" ]] && kill -0 "${online_pid}" 2>/dev/null; then
    kill -INT "${online_pid}" 2>/dev/null || true
    for _ in {1..30}; do kill -0 "${online_pid}" 2>/dev/null || break; sleep 0.1; done
    kill -0 "${online_pid}" 2>/dev/null && kill -TERM "${online_pid}" 2>/dev/null || true
    wait "${online_pid}" 2>/dev/null || true
  fi
}
trap cleanup_online EXIT

roslaunch realsense_liquid_measurement online_liquid_height.launch \
  calibration:="${RGB_CALIBRATION_FILE}" image_topic:="${RGB_IMAGE_TOPIC}" \
  measurement_topic:="${RGB_MEASUREMENT_TOPIC}" process_every:=1 zero_frames:=30 \
  publish_debug:=false height_bias_mm:=0.0 hue1_low:=0 hue1_high:=12 \
  hue2_low:=161 hue2_high:=179 sat_min:=101 val_min:=167 \
  > "${online_log}" 2>&1 &
online_pid=$!
sleep 2
kill -0 "${online_pid}" 2>/dev/null || { tail -80 "${online_log}" >&2 || true; fail "online RGB exited"; }
ready_log="${RUN_OUT_DIR}/${RUN_LABEL}_online_liquid_ready.log"
timeout 20s rostopic echo -n 20 --filter "m.valid and m.zero_locked and m.status_code == 0" \
  "${RGB_MEASUREMENT_TOPIC}" > "${ready_log}" 2>&1 || fail "online RGB did not reach clean zero-lock"
ready_count="$(grep -Ec '^valid: (True|true)$' "${ready_log}" || true)"
(( ready_count >= 20 )) || fail "online RGB ready samples ${ready_count} < 20"

env "${runner_env[@]}" bash "${RUNNER}"

python3 "${VALIDATOR}" \
  --bag "${BAG_PATH}" --condition Bsmooth --row 01 --block smoke --position 01 \
  --slosh-enabled false --smooth-priority-enabled true \
  --protocol G3R2_robot_only_delay_smoke_v1 --report-suffix _g3r2_postflight.json \
  --expected-weight 0.0 --expected-w-smooth 1.0 --expected-w-alpha 1.0 \
  --expected-w-du-a 1.0 --expected-w-du-vs 1.0 \
  --expected-delay-mode-code 4 --require-delay-compensation-applied true \
  --require-robot-delay-compensation-applied true \
  --require-liquid-delay-compensation-applied false \
  --require-state-diagnostics --expected-solver-source-code 2 \
  --expected-v-ref 0.20 --t-hvis-tail-sec "${T_HVIS_TAIL}" \
  --t-motion-max-sec "${T_MOTION_MAX}" --min-duration-sec 65 \
  --max-pre-motion-sec "${MAX_PRE_MOTION_SEC}" \
  --max-contour-p95-m 0.05 --max-yaw-p95-rad 0.15 \
  --rgb-calibration-sha256 "${RGB_CALIBRATION_SHA256}" \
  --online-config-sha256 "${ONLINE_CONFIG_SHA256}" \
  --outcome-window-rule-sha256 "${OUTCOME_WINDOW_RULE_SHA256}" \
  --prereg-sha256 "${PREREG_SHA256}" --source-report-sha256 "${SOURCE_REPORT_SHA256}" --hash-bag

python3 "${SUMMARIZER}" "${BAG_PATH}"

echo "[G3R2] smoke PASS: ${BAG_PATH}"
echo "[G3R2] Stop here. Freeze this evidence before releasing weight candidates."
