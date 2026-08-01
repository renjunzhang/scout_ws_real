#!/usr/bin/env bash
# G3R: one-run-per-candidate raw-state/weight screen with online RGB scalar.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G3R][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
VALIDATOR="${SCRIPT_DIR}/analysis/validate_g3_online_rgb_trial.py"
ANALYZER="${SCRIPT_DIR}/analysis/analyze_g3r_weight_screen.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"

[[ -r /opt/ros/noetic/setup.bash ]] || fail "missing ROS Noetic setup"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]] || fail "missing ${REPO_ROOT}/devel/setup.bash"
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

G3R_ROW="${G3R_ROW:-}"
G3R_ATTEMPT="${G3R_ATTEMPT:-01}"
DATE="${DATE:-20260801}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ALLOW_DIRTY_VALIDATE_ONLY="${ALLOW_DIRTY_VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"

PATH_FILE="/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json"
PATH_SHA256="578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e"
LEGACY_G3_ROOT="/home/geist/slosh_bags/real/20260801_spmpc_g3_processed_imu_w5_vs_bsmooth/H0"
ALIGNMENT_REPORT="${ALIGNMENT_REPORT:-${LEGACY_G3_ROOT}/analysis/G3_DELAY_STATE_ALIGNMENT_REPORT.json}"
ALIGNMENT_REPORT_EXPECTED_SHA256="08dd1276a1111e1e4b7eb0cc769cf40f03b86be63c28b6aa9cfcccc64229adbf"
SOURCE_REPORT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/analysis/G2S_RAW_RGB_THREE_TRIAL_DEVELOPMENT_REPORT.json"
SOURCE_REPORT_SHA256="36db4d12948f98e4ef3580bc52367cdef14d8440d80a8afbb32182b704ee5442"

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
PLANNER_LIBRARY_SHA256="0ef1d4679ae141c8e6a3c23cad32d16e5e5dcb20d6314d872fae4928f1523c30"
PLANNER_NODE="${REPO_ROOT}/devel/lib/spmpc_local_planner/spmpc_local_planner_node"
PLANNER_NODE_SHA256="b1de0d1772689f76813fbd78ca779a3d938aa040e2c007d060bb0da7495a37e0"
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
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g3r_raw_imu_weight_screen/H0}"
PREREG_FILE="${RUN_OUT_DIR}/G3R_weight_screen_prereg.env"
ORDER_FILE="${RUN_OUT_DIR}/G3R_weight_screen_order.csv"
METRIC_FILE="${RUN_OUT_DIR}/G3R_rgb_metric.yaml"
ONLINE_CONFIG_FILE="${RUN_OUT_DIR}/G3R_online_liquid_config.env"

case "${G3R_ROW}" in
  01) CONDITION=Bsmooth; PILOT_METHOD=Bsmooth; W_SLOSH=0.0; SMOOTH=1.0; SLOSH_ENABLED=false; SMOOTH_PRIORITY=true ;;
  02) CONDITION=W2_S03;  PILOT_METHOD=W2;      W_SLOSH=2.0; SMOOTH=0.3; SLOSH_ENABLED=true;  SMOOTH_PRIORITY=false ;;
  03) CONDITION=W5_S10;  PILOT_METHOD=W5;      W_SLOSH=5.0; SMOOTH=1.0; SLOSH_ENABLED=true;  SMOOTH_PRIORITY=false ;;
  04) CONDITION=W2_S10;  PILOT_METHOD=W2;      W_SLOSH=2.0; SMOOTH=1.0; SLOSH_ENABLED=true;  SMOOTH_PRIORITY=false ;;
  05) CONDITION=W5_S03;  PILOT_METHOD=W5;      W_SLOSH=5.0; SMOOTH=0.3; SLOSH_ENABLED=true;  SMOOTH_PRIORITY=false ;;
  *) fail "set G3R_ROW=01..05 (one baseline plus frozen 2x2 weight screen)" ;;
esac
[[ "${G3R_ATTEMPT}" == "01" ]] || fail "G3R screening permits one planned attempt per condition"

required_files=(
  "${RUNNER}" "${VALIDATOR}" "${ANALYZER}" "${SUMMARIZER}" "${CAMERA_PREP}"
  "${PATH_FILE}" "${ALIGNMENT_REPORT}" "${SOURCE_REPORT}"
  "${RGB_CALIBRATION_FILE}" "${RGB_CAMERA_PARAMS_FILE}"
  "${ONLINE_LIQUID_LAUNCH}" "${ONLINE_LIQUID_NODE}" "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}"
  "${PLANNER_LIBRARY}" "${PLANNER_NODE}" "${SLOSH_MODELS_LIBRARY}"
  "${ACADOS_B0_SOLVER}" "${ACADOS_SLOSH_SOLVER}" "${ACADOS_LIBRARY}" "${HPIPM_LIBRARY}" "${BLASFEO_LIBRARY}"
)
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ "$(sha256sum "${PATH_FILE}" | awk '{print $1}')" == "${PATH_SHA256}" ]] || fail "path hash mismatch"
[[ "$(sha256sum "${SOURCE_REPORT}" | awk '{print $1}')" == "${SOURCE_REPORT_SHA256}" ]] || fail "source-report hash mismatch"
[[ "$(sha256sum "${RGB_CALIBRATION_FILE}" | awk '{print $1}')" == "${RGB_CALIBRATION_SHA256}" ]] || fail "RGB calibration hash mismatch"
[[ "$(sha256sum "${RGB_CAMERA_PARAMS_FILE}" | awk '{print $1}')" == "${RGB_CAMERA_PARAMS_SHA256}" ]] || fail "RealSense parameter hash mismatch"
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
  actual_sha="$(sha256sum "${artifact_path}" | awk '{print $1}')"
  [[ "${actual_sha}" == "${expected_sha}" ]] || \
    fail "${artifact_label} hash mismatch: expected=${expected_sha}, actual=${actual_sha}"
done

ALIGNMENT_REPORT_SHA256="$(sha256sum "${ALIGNMENT_REPORT}" | awk '{print $1}')"
[[ "${ALIGNMENT_REPORT_SHA256}" == "${ALIGNMENT_REPORT_EXPECTED_SHA256}" ]] || \
  fail "alignment-report hash mismatch: expected=${ALIGNMENT_REPORT_EXPECTED_SHA256}, actual=${ALIGNMENT_REPORT_SHA256}"
python3 - "${ALIGNMENT_REPORT}" "${SCRIPT_DIR}/analysis/analyze_g3_delay_state_alignment.py" <<'PY'
import hashlib
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
with open(sys.argv[2], "rb") as stream:
    analyzer_sha256 = hashlib.sha256(stream.read()).hexdigest()
if report.get("analysis_script_sha256") != analyzer_sha256:
    raise SystemExit("alignment report is not bound to the frozen analyzer")
if report.get("status") != "PASS_FOR_G3R_SCREENING":
    raise SystemExit("alignment report does not permit G3R screening")
if report.get("decision") != "USE_RAW_MEASUREMENT_STATE_WITH_DELAY_SHADOW":
    raise SystemExit("alignment report selected a different control contract")
contract = report.get("release_contract", {})
if contract.get("delay_phase_mode") != "shadow":
    raise SystemExit("alignment report did not freeze shadow mode")
if contract.get("solver_uses_predicted_robot_state") or contract.get("solver_uses_predicted_liquid_state"):
    raise SystemExit("alignment report unexpectedly permits predicted state in the solver")
if contract.get("solver_liquid_source") != "processed_imu":
    raise SystemExit("alignment report selected a different liquid-state source")
if int(contract.get("imu_subscriber_queue_size_minimum", 0)) > 10:
    raise SystemExit("G3R queue depth is below the alignment-report contract")
if contract.get("fallback_policy") != "fail_closed":
    raise SystemExit("alignment report selected a different fallback policy")
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
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r_weight_screen_trial.sh
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/validate_g3_online_rgb_trial.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/analyze_g3_delay_state_alignment.py
  src/scout_apps/control/spmpc_local_planner/scripts/analysis/analyze_g3r_weight_screen.py
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
    echo "[G3R][WARN] developer validate-only with untracked code: ${tracked_missing[*]}" >&2
  else
    fail "G3R code is not frozen in revision ${release_revision}: ${tracked_missing[*]}"
  fi
fi
if ! git -C "${REPO_ROOT}" diff --quiet -- "${relevant_repo_paths[@]}" || \
   ! git -C "${REPO_ROOT}" diff --cached --quiet -- "${relevant_repo_paths[@]}"; then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G3R][WARN] developer validate-only with dirty relevant paths" >&2
  else
    fail "G3R runtime differs from release ${release_revision}; commit/freeze before motion"
  fi
fi

online_node_sha="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')"
online_detector_sha="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')"
online_msg_sha="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')"

order_contents() {
  printf '%s\n' \
    "row,condition,pilot_method,w_slosh,w_smooth,w_alpha,w_du_a,w_du_vs,replicates" \
    "01,Bsmooth,Bsmooth,0.0,1.0,1.0,1.0,1.0,1" \
    "02,W2_S03,W2,2.0,0.3,0.3,0.3,0.3,1" \
    "03,W5_S10,W5,5.0,1.0,1.0,1.0,1.0,1" \
    "04,W2_S10,W2,2.0,1.0,1.0,1.0,1.0,1" \
    "05,W5_S03,W5,5.0,0.3,0.3,0.3,0.3,1"
}

metric_contents() {
  printf '%s\n' \
    "schema_version: 1" \
    "primary_metric: h_vis_p95_motion_plus_tail_mm" \
    "secondary_metric: h_vis_rms_motion_plus_tail_mm" \
    "observer_guard: raw_processed_imu_p95_must_not_regress_more_than_0.05_mm" \
    "screen_minimum_p95_improvement_mm: 0.05" \
    "screen_minimum_rms_improvement_mm: 0.0" \
    "screen_scope: single_run_candidate_selection_only" \
    "candidate_ranking_score: rgb_p95_improvement+0.5*rgb_rms_improvement+0.25*max(raw_imu_p95_improvement,0)" \
    "rolling_median_window: 5" \
    "t_hvis_tail_sec: ${T_HVIS_TAIL}" \
    "t_motion_max_sec: ${T_MOTION_MAX}" \
    "maximum_record_to_motion_sec: ${MAX_PRE_MOTION_SEC}" \
    "image_stream_policy: forbid_all"
}

online_config_contents() {
  printf '%s\n' \
    "protocol=G3R_raw_processed_imu_weight_screen_v1" \
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

expected_order="$(order_contents)"
expected_metric="$(metric_contents)"
expected_online_config="$(online_config_contents)"
ORDER_SHA256="$(printf '%s\n' "${expected_order}" | sha256sum | awk '{print $1}')"
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
    "protocol=G3R_raw_processed_imu_weight_screen_v1" \
    "scope=development_single_run_screen" \
    "formal_stage_status=NO_GO" \
    "planned_units=5" \
    "replicates_per_condition=1" \
    "selection_then_confirmation=true" \
    "release_revision=${release_revision}" \
    "alignment_report=${ALIGNMENT_REPORT}" \
    "alignment_report_sha256=${ALIGNMENT_REPORT_SHA256}" \
    "source_report_sha256=${SOURCE_REPORT_SHA256}" \
    "path_sha256=${PATH_SHA256}" \
    "delay_phase_mode=shadow" \
    "closed_loop_delay_compensation=false" \
    "observer_source=processed_imu" \
    "observer_fallback_policy=fail_closed" \
    "imu_subscriber_queue_size=10" \
    "v_ref=0.20" \
    "order_sha256=${ORDER_SHA256}" \
    "outcome_window_rule_sha256=${OUTCOME_WINDOW_RULE_SHA256}" \
    "online_config_sha256=${ONLINE_CONFIG_SHA256}" \
    "runtime_binary_bundle_sha256=${RUNTIME_BINARY_BUNDLE_SHA256}" \
    "record_sec=${RECORD_SEC}" \
    "positive_candidates_only_enter_paired_confirmation=true" \
    "single_run_is_not_efficacy_evidence=true"
}
expected_prereg="$(prereg_contents)"
PREREG_SHA256="$(printf '%s\n' "${expected_prereg}" | sha256sum | awk '{print $1}')"

RUN_LABEL="DEV_G3R_H0_C1_${CONDITION}_r${G3R_ROW}_a${G3R_ATTEMPT}"
BAG_PATH="${RUN_OUT_DIR}/${RUN_LABEL}.bag"
runner_env=(
  "DATE=${DATE}" "STAMP=${STAMP}" "PILOT_MODE=true" "PILOT_METHOD=${PILOT_METHOD}"
  "PILOT_CONDITION=G3R_raw_processed_imu_weight_screen"
  "PILOT_RECORD_RGB=false" "PILOT_RECORD_ONLINE_LIQUID=true"
  "RUN_LABEL=${RUN_LABEL}" "NAME=${RUN_LABEL}" "RUN_OUT_DIR=${RUN_OUT_DIR}"
  "PATH_SOURCE_MODE=replay" "PATH_FILE=${PATH_FILE}" "PATH_EXPECTED_SHA256=${PATH_SHA256}"
  "REQUIRE_PATH_HASH=true" "START_POS_TOL=0.08" "START_YAW_TOL=0.15"
  "START_HOLD_SEC=0.5" "START_GATE_TIMEOUT_SEC=3" "V_REF=0.20"
  "W_SLOSH=${W_SLOSH}" "W_SMOOTH=${SMOOTH}" "W_ALPHA=${SMOOTH}"
  "W_DU_A=${SMOOTH}" "W_DU_VS=${SMOOTH}"
  "DELAY_PHASE_MODE=shadow" "DELAY_PHASE_LINEAR_DELAY_SEC=0.15"
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
  "ONLINE_LIQUID_PROTOCOL=G3R_raw_processed_imu_weight_screen_v1"
  "ONLINE_LIQUID_CALIBRATION_SHA256=${RGB_CALIBRATION_SHA256}"
  "ONLINE_LIQUID_DETECTOR_SHA256=${online_detector_sha}"
  "ONLINE_LIQUID_NODE_SHA256=${online_node_sha}" "ONLINE_LIQUID_MSG_SHA256=${online_msg_sha}"
  "ONLINE_LIQUID_CONFIG_SHA256=${ONLINE_CONFIG_SHA256}"
  "RECORD_SEC=${RECORD_SEC}" "MAX_RECORD_SEC=${RECORD_SEC}"
  "BLOCK_SEGMENT_ID=G3R_screen" "ORDER_POSITION=${G3R_ROW}" "SEND_ZERO_ON_EXIT=true"
  "OPERATOR_NOTE=G3R_${CONDITION}_raw_state_shadow_alignment_${ALIGNMENT_REPORT_SHA256}"
)

echo "================ G3R one-run weight screen ================"
echo "  row/condition = ${G3R_ROW}/${CONDITION}"
echo "  weights       = w_slosh ${W_SLOSH}; smooth split ${SMOOTH}"
echo "  state input   = current processed-IMU; delay predictor shadow only"
echo "  IMU contract  = queue 10; fail-closed; no odom fallback"
echo "  output        = ${BAG_PATH}"
echo "  claim         = screening only; positive candidate must be replicated"
echo "============================================================"

if truthy "${VALIDATE_ONLY}"; then
  printf '[G3R] validate-only runner command:\n  env '
  printf '%q ' "${runner_env[@]}"
  printf 'bash %q\n' "${RUNNER}"
  echo "[G3R] alignment SHA256 = ${ALIGNMENT_REPORT_SHA256}"
  echo "[G3R] prereg SHA256    = ${PREREG_SHA256}"
  echo "[G3R] order SHA256     = ${ORDER_SHA256}"
  echo "[G3R] runtime SHA256   = ${RUNTIME_BINARY_BUNDLE_SHA256}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || fail "real motion is disarmed; set ARM_MOTION=YES after clearing the path"
[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || fail "set CONFIRM_RGB_GEOMETRY=YES after checking ROI/rulers/container"
[[ ! -e "${BAG_PATH}" && ! -e "${BAG_PATH}.active" ]] || fail "attempt output already exists: ${BAG_PATH}"

if [[ "${G3R_ROW}" != "01" ]]; then
  previous_row="$(printf '%02d' "$((10#${G3R_ROW} - 1))")"
  case "${previous_row}" in
    01) previous_condition=Bsmooth ;;
    02) previous_condition=W2_S03 ;;
    03) previous_condition=W5_S10 ;;
    04) previous_condition=W2_S10 ;;
  esac
  previous_report="${RUN_OUT_DIR}/DEV_G3R_H0_C1_${previous_condition}_r${previous_row}_a01_g3r_postflight.json"
  [[ -s "${previous_report}" ]] || fail "previous row postflight is missing: ${previous_report}"
  python3 - "${previous_report}" "${previous_row}" "${PREREG_SHA256}" "${SOURCE_REPORT_SHA256}" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("status") != "PASS":
    raise SystemExit("previous G3R row is not PASS")
if str(report.get("row")) != sys.argv[2]:
    raise SystemExit("previous G3R report row binding mismatch")
if report.get("protocol") != "G3R_raw_processed_imu_weight_screen_v1":
    raise SystemExit("previous G3R report protocol mismatch")
bindings = report.get("bindings", {})
if bindings.get("prereg_sha256") != sys.argv[3]:
    raise SystemExit("previous G3R report prereg binding mismatch")
if bindings.get("source_report_sha256") != sys.argv[4]:
    raise SystemExit("previous G3R report source binding mismatch")
PY
fi

available_kib="$(df -Pk /home/geist/slosh_bags/real | awk 'NR == 2 {print $4}')"
required_kib=$((MIN_FREE_GIB * 1024 * 1024))
(( available_kib >= required_kib )) || fail "insufficient free space: require ${MIN_FREE_GIB} GiB"

mkdir -p "${RUN_OUT_DIR}"
if [[ -e "${PREREG_FILE}" ]]; then
  [[ "$(<"${PREREG_FILE}")" == "${expected_prereg}" ]] || fail "G3R prereg changed"
  [[ "$(<"${ORDER_FILE}")" == "${expected_order}" ]] || fail "G3R order changed"
  [[ "$(<"${METRIC_FILE}")" == "${expected_metric}" ]] || fail "G3R metric changed"
  [[ "$(<"${ONLINE_CONFIG_FILE}")" == "${expected_online_config}" ]] || fail "G3R online config changed"
else
  [[ "${G3R_ROW}" == "01" ]] || fail "row 01 must create the prereg bundle"
  printf '%s\n' "${expected_prereg}" > "${PREREG_FILE}"
  printf '%s\n' "${expected_order}" > "${ORDER_FILE}"
  printf '%s\n' "${expected_metric}" > "${METRIC_FILE}"
  printf '%s\n' "${expected_online_config}" > "${ONLINE_CONFIG_FILE}"
  sha256sum "${PREREG_FILE}" "${ORDER_FILE}" "${METRIC_FILE}" "${ONLINE_CONFIG_FILE}" > "${RUN_OUT_DIR}/G3R_prereg_bundle.sha256"
fi

printf '%s\n' \
  "row=${G3R_ROW}" "condition=${CONDITION}" "w_slosh=${W_SLOSH}" "smooth=${SMOOTH}" \
  "alignment_report_sha256=${ALIGNMENT_REPORT_SHA256}" "prereg_sha256=${PREREG_SHA256}" \
  > "${RUN_OUT_DIR}/${RUN_LABEL}_g3r_binding.env"

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
  --bag "${BAG_PATH}" --condition "${CONDITION}" --row "${G3R_ROW}" \
  --block screen --position "${G3R_ROW}" --slosh-enabled "${SLOSH_ENABLED}" \
  --smooth-priority-enabled "${SMOOTH_PRIORITY}" \
  --protocol G3R_raw_processed_imu_weight_screen_v1 --report-suffix _g3r_postflight.json \
  --expected-weight "${W_SLOSH}" --expected-w-smooth "${SMOOTH}" \
  --expected-w-alpha "${SMOOTH}" --expected-w-du-a "${SMOOTH}" --expected-w-du-vs "${SMOOTH}" \
  --expected-delay-mode-code 2 --require-delay-compensation-applied false \
  --require-state-diagnostics --expected-solver-source-code 2 \
  --expected-v-ref 0.20 --t-hvis-tail-sec "${T_HVIS_TAIL}" \
  --t-motion-max-sec "${T_MOTION_MAX}" --min-duration-sec 65 \
  --max-pre-motion-sec "${MAX_PRE_MOTION_SEC}" \
  --rgb-calibration-sha256 "${RGB_CALIBRATION_SHA256}" \
  --online-config-sha256 "${ONLINE_CONFIG_SHA256}" \
  --outcome-window-rule-sha256 "${OUTCOME_WINDOW_RULE_SHA256}" \
  --prereg-sha256 "${PREREG_SHA256}" --source-report-sha256 "${SOURCE_REPORT_SHA256}" --hash-bag

python3 "${SUMMARIZER}" "${BAG_PATH}"

if [[ "${G3R_ROW}" == "05" ]]; then
  set +e
  python3 "${ANALYZER}" --root "${RUN_OUT_DIR}" \
    --minimum-rgb-p95-improvement-mm 0.05 --minimum-rgb-rms-improvement-mm 0.0 \
    --maximum-raw-imu-regression-mm 0.05 \
    --alignment-report-sha256 "${ALIGNMENT_REPORT_SHA256}" --prereg-sha256 "${PREREG_SHA256}" \
    --source-report-sha256 "${SOURCE_REPORT_SHA256}"
  analyzer_rc=$?
  set -e
  [[ -s "${RUN_OUT_DIR}/G3R_WEIGHT_SCREEN_REPORT.json" ]] || fail "G3R analyzer produced no report"
  [[ -s "${RUN_OUT_DIR}/G3R_WEIGHT_SCREEN_METRICS.csv" ]] || fail "G3R analyzer produced no metrics CSV"
  [[ -s "${RUN_OUT_DIR}/G3R_WEIGHT_SCREEN_REPORT.json.sha256" ]] || fail "G3R report hash sidecar is missing"
  [[ -s "${RUN_OUT_DIR}/G3R_WEIGHT_SCREEN_METRICS.csv.sha256" ]] || fail "G3R metrics hash sidecar is missing"
  (
    cd "${RUN_OUT_DIR}"
    sha256sum -c G3R_WEIGHT_SCREEN_REPORT.json.sha256 G3R_WEIGHT_SCREEN_METRICS.csv.sha256 >/dev/null
  ) || fail "G3R analyzer artifact hash verification failed"
  screen_status="$(python3 - "${RUN_OUT_DIR}/G3R_WEIGHT_SCREEN_REPORT.json" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    print(json.load(stream).get("status", ""))
PY
)"
  if [[ "${screen_status}" == "PROMOTE_FOR_PAIRED_CONFIRMATION" ]]; then
    (( analyzer_rc == 0 )) || fail "promotion report and analyzer exit code disagree"
    [[ -s "${RUN_OUT_DIR}/G3R_CONFIRMATION_PLAN.draft.json" ]] || fail "promotion has no confirmation-plan draft"
    [[ -s "${RUN_OUT_DIR}/G3R_CONFIRMATION_PLAN.draft.json.sha256" ]] || fail "confirmation-plan hash sidecar is missing"
    (
      cd "${RUN_OUT_DIR}"
      sha256sum -c G3R_CONFIRMATION_PLAN.draft.json.sha256 >/dev/null
    ) || fail "confirmation-plan hash verification failed"
    echo "[G3R] Positive candidate found; freeze the report before paired confirmation."
  elif [[ "${screen_status}" == "NO_PROMOTION" ]]; then
    (( analyzer_rc == 10 )) || fail "no-promotion report and analyzer exit code disagree"
    echo "[G3R] Screen completed with no positive candidate; do not collect repeats."
  elif [[ "${screen_status}" == "SCREEN_INVALID" ]]; then
    fail "G3R screen is invalid; inspect G3R_WEIGHT_SCREEN_REPORT.json and do not repeat selectively"
  else
    fail "unexpected screen status: ${screen_status:-missing}"
  fi
fi

echo "[G3R] acquisition PASS: ${BAG_PATH}"
