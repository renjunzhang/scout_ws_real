#!/usr/bin/env bash
# Three-block G3R2 development paired confirmation: W5_S10 vs Bsmooth.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G3R2-CONFIRM][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
[[ -r /opt/ros/noetic/setup.bash ]] || fail "missing ROS Noetic setup"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]] || fail "missing workspace setup"
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
VALIDATOR="${SCRIPT_DIR}/../tools/analysis/validate_g3_online_rgb_trial.py"
ANALYZER="${SCRIPT_DIR}/../tools/analysis/analyze_g3r2_paired_confirmation.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"
TIMESTAMP_GATE="${REPO_ROOT}/devel/lib/spmpc_local_planner/spmpc_realsense_timestamp_health_gate"

G3R2C_ROW="${G3R2C_ROW:-}"
G3R2C_ATTEMPT="${G3R2C_ATTEMPT:-01}"
DATE="${DATE:-20260801}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ALLOW_DIRTY_VALIDATE_ONLY="${ALLOW_DIRTY_VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"
[[ "${G3R2C_ATTEMPT}" == "01" ]] || fail "paired confirmation permits only attempt 01"

case "${G3R2C_ROW}" in
  01) BLOCK=01; POSITION=01; CONDITION=Bsmooth; METHOD=Bsmooth; WEIGHT=0.0; PLANNER_VARIANT=B_smooth; SLOSH_ENABLED=false; SMOOTH_PRIORITY=true ;;
  02) BLOCK=01; POSITION=02; CONDITION=W5_S10;  METHOD=W5;      WEIGHT=5.0; PLANNER_VARIANT=B_slosh;  SLOSH_ENABLED=true;  SMOOTH_PRIORITY=false ;;
  03) BLOCK=02; POSITION=01; CONDITION=W5_S10;  METHOD=W5;      WEIGHT=5.0; PLANNER_VARIANT=B_slosh;  SLOSH_ENABLED=true;  SMOOTH_PRIORITY=false ;;
  04) BLOCK=02; POSITION=02; CONDITION=Bsmooth; METHOD=Bsmooth; WEIGHT=0.0; PLANNER_VARIANT=B_smooth; SLOSH_ENABLED=false; SMOOTH_PRIORITY=true ;;
  05) BLOCK=03; POSITION=01; CONDITION=Bsmooth; METHOD=Bsmooth; WEIGHT=0.0; PLANNER_VARIANT=B_smooth; SLOSH_ENABLED=false; SMOOTH_PRIORITY=true ;;
  06) BLOCK=03; POSITION=02; CONDITION=W5_S10;  METHOD=W5;      WEIGHT=5.0; PLANNER_VARIANT=B_slosh;  SLOSH_ENABLED=true;  SMOOTH_PRIORITY=false ;;
  *) fail "set G3R2C_ROW=01..06; frozen order is Bsmooth,W5_S10,W5_S10,Bsmooth,Bsmooth,W5_S10" ;;
esac

PATH_FILE="/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json"
PATH_SHA256="578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e"
SOURCE_REPORT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/analysis/G2S_RAW_RGB_THREE_TRIAL_DEVELOPMENT_REPORT.json"
SOURCE_REPORT_SHA256="36db4d12948f98e4ef3580bc52367cdef14d8440d80a8afbb32182b704ee5442"

SCREEN_ROOT="/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0"
SCREEN_REPORT="${SCREEN_ROOT}/G3R2_WEIGHT_SCREEN_REPORT.json"
SCREEN_REPORT_SHA256="e6c3b030cac925ce29887ea7d76da51d86fae8d01f6a486e5c2773f407eb7258"
SCREEN_METRICS="${SCREEN_ROOT}/G3R2_WEIGHT_SCREEN_METRICS.csv"
SCREEN_METRICS_SHA256="aff8bdd451065f37ee5b547d20b5b361a01d764a64b88aa506a15ed4e5038d12"
SCREEN_DRAFT="${SCREEN_ROOT}/G3R2_CONFIRMATION_PLAN.draft.json"
SCREEN_DRAFT_SHA256="78cefa54bda491c24197b958203fe23a0bf1c41b574673721f39d989be99b03f"
SELECTED_BAG="${SCREEN_ROOT}/DEV_G3R2_H0_C1_W5_S10_r03_a02.bag"
SELECTED_BAG_SHA256="872d37a23e4d85488fe97c9b2eecc6ea75172389a13a6c429e37624220df67c9"
SELECTED_POSTFLIGHT="${SCREEN_ROOT}/DEV_G3R2_H0_C1_W5_S10_r03_a02_g3r2_screen_retry_postflight.json"
SELECTED_POSTFLIGHT_SHA256="212214a3e80e934bf74f38b6a46689af0aad521acff1513e0c1c52afe74c4713"

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
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g3r2_w5s10_paired_confirmation/H0}"
PREREG_FILE="${RUN_OUT_DIR}/G3R2_paired_confirmation_prereg.env"
ORDER_FILE="${RUN_OUT_DIR}/G3R2_paired_confirmation_order.csv"
METRIC_FILE="${RUN_OUT_DIR}/G3R2_paired_confirmation_metric.yaml"
ONLINE_CONFIG_FILE="${RUN_OUT_DIR}/G3R2_paired_confirmation_online_config.env"

verify_sha256() {
  local path="$1" expected="$2" label="$3" actual
  [[ -s "${path}" ]] || fail "missing ${label}: ${path}"
  actual="$(sha256sum "${path}" | awk '{print $1}')"
  [[ "${actual}" == "${expected}" ]] || fail "${label} hash mismatch: ${actual}"
}

required_files=(
  "${RUNNER}" "${VALIDATOR}" "${ANALYZER}" "${SUMMARIZER}" "${CAMERA_PREP}" "${TIMESTAMP_GATE}"
  "${PATH_FILE}" "${SOURCE_REPORT}" "${SCREEN_REPORT}" "${SCREEN_METRICS}" "${SCREEN_DRAFT}"
  "${SELECTED_BAG}" "${SELECTED_POSTFLIGHT}" "${RGB_CALIBRATION_FILE}" "${RGB_CAMERA_PARAMS_FILE}"
  "${ONLINE_LIQUID_LAUNCH}" "${ONLINE_LIQUID_NODE}" "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}"
  "${PLANNER_LIBRARY}" "${PLANNER_NODE}" "${SLOSH_MODELS_LIBRARY}" "${ACADOS_B0_SOLVER}"
  "${ACADOS_SLOSH_SOLVER}" "${ACADOS_LIBRARY}" "${HPIPM_LIBRARY}" "${BLASFEO_LIBRARY}"
)
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ -x "${TIMESTAMP_GATE}" ]] || fail "timestamp gate is not executable: ${TIMESTAMP_GATE}"

verify_sha256 "${PATH_FILE}" "${PATH_SHA256}" "path"
verify_sha256 "${SOURCE_REPORT}" "${SOURCE_REPORT_SHA256}" "source report"
verify_sha256 "${SCREEN_REPORT}" "${SCREEN_REPORT_SHA256}" "weight-screen report"
verify_sha256 "${SCREEN_METRICS}" "${SCREEN_METRICS_SHA256}" "weight-screen metrics"
verify_sha256 "${SCREEN_DRAFT}" "${SCREEN_DRAFT_SHA256}" "confirmation draft"
verify_sha256 "${SELECTED_BAG}" "${SELECTED_BAG_SHA256}" "selected-candidate bag"
verify_sha256 "${SELECTED_POSTFLIGHT}" "${SELECTED_POSTFLIGHT_SHA256}" "selected-candidate postflight"
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
for binding in "${runtime_artifacts[@]}"; do
  IFS='|' read -r artifact expected label <<< "${binding}"
  verify_sha256 "${artifact}" "${expected}" "${label}"
done

python3 - "${SCREEN_REPORT}" "${SCREEN_DRAFT}" "${SELECTED_POSTFLIGHT}" \
  "${SCREEN_REPORT_SHA256}" "${SELECTED_BAG_SHA256}" "${SELECTED_POSTFLIGHT_SHA256}" \
  "${SOURCE_REPORT_SHA256}" <<'PY'
import json
import math
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    screen = json.load(stream)
with open(sys.argv[2], encoding="utf-8") as stream:
    draft = json.load(stream)
with open(sys.argv[3], encoding="utf-8") as stream:
    selected_postflight = json.load(stream)
if screen.get("status") != "PROMOTE_FOR_PAIRED_CONFIRMATION" or screen.get("failures"):
    raise SystemExit("weight screen did not produce a clean promotion")
selected = screen.get("selected_candidate", {})
expected_weights = {
    "w_slosh": 5.0,
    "smooth": 1.0,
}
if selected.get("condition") != "W5_S10" or selected.get("row") != "03":
    raise SystemExit("weight-screen selected candidate changed")
for key, value in expected_weights.items():
    if not math.isclose(float(selected.get(key, -1.0)), value, abs_tol=1.0e-8):
        raise SystemExit("selected candidate {} changed".format(key))
if not selected.get("screen_positive") or not selected.get("eligible_for_selection"):
    raise SystemExit("selected W5_S10 is not eligible/positive")
if float(selected.get("rgb_p95_improvement_mm", 0.0)) < 0.05:
    raise SystemExit("selected W5_S10 screen effect is below threshold")
if screen.get("bindings", {}).get("source_report_sha256") != sys.argv[7]:
    raise SystemExit("weight-screen source binding changed")
dataset = {str(item.get("row")): item for item in screen.get("dataset", [])}
row03 = dataset.get("03", {})
if row03.get("bag_sha256") != sys.argv[5] or row03.get("postflight_sha256") != sys.argv[6]:
    raise SystemExit("selected Row 03 artifact binding mismatch")
if len(screen.get("excluded_acquisition_attempts", [])) != 1:
    raise SystemExit("screen acquisition exclusion changed")
method_negative = screen.get("method_negative_candidates", [])
if len(method_negative) != 1 or method_negative[0].get("row") != "04":
    raise SystemExit("screen method-negative record changed")
if draft.get("status") != "DRAFT_REQUIRES_NEW_RELEASE_FREEZE":
    raise SystemExit("confirmation draft status changed")
if draft.get("selected_condition") != "W5_S10" or draft.get("paired_blocks") != 3:
    raise SystemExit("confirmation draft selection/blocks changed")
if draft.get("bindings", {}).get("screen_report_sha256") != sys.argv[4]:
    raise SystemExit("confirmation draft screen binding mismatch")
expected_order = [
    ("01", "01", "01", "Bsmooth"),
    ("02", "01", "02", "W5_S10"),
    ("03", "02", "01", "W5_S10"),
    ("04", "02", "02", "Bsmooth"),
    ("05", "03", "01", "Bsmooth"),
    ("06", "03", "02", "W5_S10"),
]
actual_order = [
    (str(item.get("row")), str(item.get("block")), str(item.get("position")), item.get("condition"))
    for item in draft.get("planned_rows", [])
]
if actual_order != expected_order:
    raise SystemExit("confirmation draft order changed")
weights = draft.get("selected_weights", {})
for field in ("w_slosh", "w_smooth", "w_alpha", "w_du_a", "w_du_vs"):
    expected = 5.0 if field == "w_slosh" else 1.0
    if not math.isclose(float(weights.get(field, -1.0)), expected, abs_tol=1.0e-8):
        raise SystemExit("confirmation draft {} changed".format(field))
if selected_postflight.get("status") != "PASS" or selected_postflight.get("bag_sha256") != sys.argv[5]:
    raise SystemExit("selected-candidate postflight is not the frozen PASS")
PY

runtime_revision="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
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
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_paired_confirmation_trial.sh
  src/scout_apps/control/spmpc_local_planner/tools/analysis/validate_g3_online_rgb_trial.py
  src/scout_apps/control/spmpc_local_planner/tools/analysis/analyze_g3r2_paired_confirmation.py
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
    echo "[G3R2-CONFIRM][WARN] developer validate-only with untracked code: ${tracked_missing[*]}" >&2
  else
    fail "confirmation code is not frozen in revision ${runtime_revision}: ${tracked_missing[*]}"
  fi
fi
if ! git -C "${REPO_ROOT}" diff --quiet -- "${relevant_repo_paths[@]}" || \
   ! git -C "${REPO_ROOT}" diff --cached --quiet -- "${relevant_repo_paths[@]}"; then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G3R2-CONFIRM][WARN] developer validate-only with dirty relevant paths" >&2
  else
    fail "confirmation runtime differs from revision ${runtime_revision}; commit/freeze before motion"
  fi
fi

online_node_sha="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')"
online_detector_sha="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')"
online_msg_sha="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')"
timestamp_gate_binary_sha="$(sha256sum "${TIMESTAMP_GATE}" | awk '{print $1}')"

order_contents() {
  printf '%s\n' \
    "row,block,position,condition,pilot_method,w_slosh,w_smooth" \
    "01,01,01,Bsmooth,Bsmooth,0.0,1.0" \
    "02,01,02,W5_S10,W5,5.0,1.0" \
    "03,02,01,W5_S10,W5,5.0,1.0" \
    "04,02,02,Bsmooth,Bsmooth,0.0,1.0" \
    "05,03,01,Bsmooth,Bsmooth,0.0,1.0" \
    "06,03,02,W5_S10,W5,5.0,1.0"
}

metric_contents() {
  printf '%s\n' \
    "schema_version: 1" \
    "primary_metric: paired_mean_h_vis_p95_improvement_mm" \
    "secondary_metric: paired_mean_h_vis_rms_improvement_mm" \
    "observer_guard: paired_mean_raw_processed_imu_p95_regression_le_0.05_mm" \
    "minimum_mean_rgb_p95_improvement_mm: 0.05" \
    "minimum_mean_rgb_rms_improvement_mm: 0.0" \
    "maximum_mean_raw_imu_regression_mm: 0.05" \
    "minimum_positive_blocks: 2" \
    "median_direction_must_be_positive: true" \
    "single_block_dominance_forbidden: true" \
    "leave_one_block_out_direction_must_be_positive: true" \
    "rolling_median_window: 5" \
    "t_hvis_tail_sec: ${T_HVIS_TAIL}" \
    "t_motion_max_sec: ${T_MOTION_MAX}" \
    "maximum_record_to_motion_sec: ${MAX_PRE_MOTION_SEC}" \
    "image_stream_policy: forbid_all"
}

online_config_contents() {
  printf '%s\n' \
    "protocol=G3R2_robot_only_W5_S10_paired_confirmation_v1" \
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
  "blasfeo_runtime=${BLASFEO_LIBRARY_SHA256}" \
  "timestamp_gate_binary=${timestamp_gate_binary_sha}" | sha256sum | awk '{print $1}')"

prereg_contents() {
  printf '%s\n' \
    "protocol=G3R2_robot_only_W5_S10_paired_confirmation_v1" \
    "scope=development_paired_confirmation_only" \
    "formal_stage_status=NO_GO" \
    "release_revision=${runtime_revision}" \
    "screen_report_sha256=${SCREEN_REPORT_SHA256}" \
    "screen_metrics_sha256=${SCREEN_METRICS_SHA256}" \
    "screen_draft_sha256=${SCREEN_DRAFT_SHA256}" \
    "selected_condition=W5_S10" \
    "selected_bag_sha256=${SELECTED_BAG_SHA256}" \
    "selected_postflight_sha256=${SELECTED_POSTFLIGHT_SHA256}" \
    "comparator=Bsmooth" \
    "paired_blocks=3" \
    "planned_units=6" \
    "attempts_per_unit=1" \
    "order_sha256=${ORDER_SHA256}" \
    "path_sha256=${PATH_SHA256}" \
    "source_report_sha256=${SOURCE_REPORT_SHA256}" \
    "observer_source=processed_imu" \
    "observer_fallback_policy=fail_closed" \
    "delay_phase_mode=fixed_robot_only" \
    "robot_delay_compensation=true" \
    "liquid_delay_compensation=false" \
    "v_ref=0.20" \
    "w5_slosh=5.0" \
    "w5_smooth=1.0" \
    "bsmooth_slosh=0.0" \
    "bsmooth_smooth=1.0" \
    "rgb_calibration_sha256=${RGB_CALIBRATION_SHA256}" \
    "rgb_camera_params_sha256=${RGB_CAMERA_PARAMS_SHA256}" \
    "online_config_sha256=${ONLINE_CONFIG_SHA256}" \
    "outcome_window_rule_sha256=${OUTCOME_WINDOW_RULE_SHA256}" \
    "timestamp_gate_impl=cpp_executable" \
    "timestamp_gate_sha256=${timestamp_gate_binary_sha}" \
    "runtime_binary_bundle_sha256=${RUNTIME_BINARY_BUNDLE_SHA256}" \
    "minimum_mean_rgb_p95_improvement_mm=0.05" \
    "minimum_mean_rgb_rms_improvement_mm=0.0" \
    "maximum_mean_raw_imu_regression_mm=0.05" \
    "minimum_positive_blocks=2" \
    "record_sec=${RECORD_SEC}" \
    "forbid_image_streams=true" \
    "no_early_stop=true"
}

expected_prereg="$(prereg_contents)"
PREREG_SHA256="$(printf '%s\n' "${expected_prereg}" | sha256sum | awk '{print $1}')"
RUN_LABEL="DEV_G3R2C_H0_C1_${CONDITION}_b${BLOCK}_p${POSITION}_r${G3R2C_ROW}_a${G3R2C_ATTEMPT}"
BAG_PATH="${RUN_OUT_DIR}/${RUN_LABEL}.bag"

runner_env=(
  "DATE=${DATE}" "STAMP=${STAMP}" "PILOT_MODE=true" "PILOT_METHOD=${METHOD}"
  "PILOT_CONDITION=G3R2_robot_only_W5_S10_paired_confirmation"
  "PILOT_RECORD_RGB=false" "PILOT_RECORD_ONLINE_LIQUID=true"
  "RUN_LABEL=${RUN_LABEL}" "NAME=${RUN_LABEL}" "RUN_OUT_DIR=${RUN_OUT_DIR}"
  "PATH_SOURCE_MODE=replay" "PATH_FILE=${PATH_FILE}" "PATH_EXPECTED_SHA256=${PATH_SHA256}"
  "REQUIRE_PATH_HASH=true" "START_POS_TOL=0.08" "START_YAW_TOL=0.15"
  "START_HOLD_SEC=0.5" "START_GATE_TIMEOUT_SEC=3" "V_REF=0.20"
  "W_SLOSH=${WEIGHT}" "W_SMOOTH=1.0" "W_ALPHA=1.0" "W_DU_A=1.0" "W_DU_VS=1.0"
  "DELAY_PHASE_MODE=fixed_robot_only" "DELAY_PHASE_LINEAR_DELAY_SEC=0.15"
  "DELAY_PHASE_ANGULAR_DELAY_SEC=0.22" "IMU_SHADOW_ENABLE=true"
  "IMU_SUBSCRIBER_QUEUE_SIZE=10" "IMU_SHADOW_READY_TIMEOUT_SEC=12"
  "CURRENT_OBSERVER_SOURCE=processed_imu" "OBSERVER_FALLBACK_POLICY=fail_closed"
  "OBSERVER_LATCH_FALLBACK=false" "RECORD_RGB=false" "RECORD_CAMERA=false"
  "RECORD_CAMERA_INFO=true" "RECORD_CAMERA_COMPRESSED=false" "RECORD_DEPTH=false"
  "RECORD_ONLINE_LIQUID=true" "RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false"
  "FORBID_IMAGE_STREAMS=true" "RECORD_ALL_EXISTING_TOPICS=false"
  "RGB_CALIBRATION_FILE=${RGB_CALIBRATION_FILE}"
  "RGB_CALIBRATION_EXPECTED_SHA256=${RGB_CALIBRATION_SHA256}"
  "RGB_EXPECTED_WIDTH=1920" "RGB_EXPECTED_HEIGHT=1080" "RGB_EXPECTED_FPS=30"
  "ONLINE_LIQUID_MEASUREMENT_TOPIC=${RGB_MEASUREMENT_TOPIC}"
  "ONLINE_LIQUID_PROTOCOL=G3R2_robot_only_W5_S10_paired_confirmation_v1"
  "ONLINE_LIQUID_CALIBRATION_SHA256=${RGB_CALIBRATION_SHA256}"
  "ONLINE_LIQUID_DETECTOR_SHA256=${online_detector_sha}"
  "ONLINE_LIQUID_NODE_SHA256=${online_node_sha}" "ONLINE_LIQUID_MSG_SHA256=${online_msg_sha}"
  "ONLINE_LIQUID_CONFIG_SHA256=${ONLINE_CONFIG_SHA256}"
  "RECORD_SEC=${RECORD_SEC}" "MAX_RECORD_SEC=${RECORD_SEC}"
  "BLOCK_SEGMENT_ID=G3R2C_b${BLOCK}" "SPLIT_BLOCK=false" "ORDER_POSITION=${POSITION}"
  "ACQUISITION_RETRY=false" "SEND_ZERO_ON_EXIT=true"
  "OPERATOR_NOTE=G3R2C_${CONDITION}_screen_${SCREEN_REPORT_SHA256}_prereg_${PREREG_SHA256}"
)

echo "================ G3R2 paired confirmation ================"
echo "  row/block/pos = ${G3R2C_ROW}/${BLOCK}/${POSITION}"
echo "  condition     = ${CONDITION} (${METHOD})"
echo "  weights       = w_slosh ${WEIGHT}; smooth split 1.0"
echo "  state timing  = fixed_robot_only; processed-IMU; fail-closed"
echo "  selection     = ${SCREEN_REPORT_SHA256}"
echo "  output        = ${BAG_PATH}"
echo "  scope         = development confirmation; formal remains NO-GO"
echo "==========================================================="

validate_launch_contract() {
  local dump expected_line
  dump="$(roslaunch --dump-params spmpc_local_planner spmpc_fixed_path.launch \
    planner_variant:="${PLANNER_VARIANT}" delay_phase_mode:=fixed_robot_only \
    delay_phase_linear_delay_sec:=0.15 delay_phase_angular_delay_sec:=0.22 \
    imu_shadow_enable:=true imu_subscriber_queue_size:=10 \
    observer_source:=processed_imu observer_fallback_policy:=fail_closed \
    observer_latch_fallback:=false v_ref:=0.20 w_slosh:="${WEIGHT}" \
    w_smooth:=1.0 w_alpha:=1.0 w_du_a:=1.0 w_du_vs:=1.0)" || \
    fail "could not dump confirmation launch parameters"
  expected_lines=(
    "/spmpc_local_planner/planner_variant: ${PLANNER_VARIANT}"
    "/spmpc_local_planner/delay_phase/mode: fixed_robot_only"
    "/spmpc_local_planner/imu_shadow/enable: true"
    "/spmpc_local_planner/imu_shadow/subscriber_queue_size: 10"
    "/spmpc_local_planner/slosh_observer/source: processed_imu"
    "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
    "/spmpc_local_planner/slosh_observer/latch_fallback: false"
    "/spmpc_local_planner/variants/${PLANNER_VARIANT}/slosh_enable: ${SLOSH_ENABLED}"
    "/spmpc_local_planner/variants/${PLANNER_VARIANT}/smooth_priority_enable: ${SMOOTH_PRIORITY}"
    "/spmpc_local_planner/variants/${PLANNER_VARIANT}/w_slosh: ${WEIGHT}"
    "/spmpc_local_planner/variants/${PLANNER_VARIANT}/w_smooth: 1.0"
    "/spmpc_local_planner/variants/${PLANNER_VARIANT}/w_alpha: 1.0"
    "/spmpc_local_planner/variants/${PLANNER_VARIANT}/w_du_a: 1.0"
    "/spmpc_local_planner/variants/${PLANNER_VARIANT}/w_du_vs: 1.0"
  )
  for expected_line in "${expected_lines[@]}"; do
    grep -Fqx -- "${expected_line}" <<< "${dump}" || fail "launch contract missing: ${expected_line}"
  done
  echo "[G3R2-CONFIRM] launch contract PASS"
}

previous_label() {
  case "$1" in
    01) printf '%s\n' DEV_G3R2C_H0_C1_Bsmooth_b01_p01_r01_a01 ;;
    02) printf '%s\n' DEV_G3R2C_H0_C1_W5_S10_b01_p02_r02_a01 ;;
    03) printf '%s\n' DEV_G3R2C_H0_C1_W5_S10_b02_p01_r03_a01 ;;
    04) printf '%s\n' DEV_G3R2C_H0_C1_Bsmooth_b02_p02_r04_a01 ;;
    05) printf '%s\n' DEV_G3R2C_H0_C1_Bsmooth_b03_p01_r05_a01 ;;
    *) return 1 ;;
  esac
}

validate_previous_row() {
  [[ "${G3R2C_ROW}" != "01" ]] || return 0
  local row label report
  row="$(printf '%02d' "$((10#${G3R2C_ROW} - 1))")"
  label="$(previous_label "${row}")" || fail "cannot resolve previous confirmation row"
  report="${RUN_OUT_DIR}/${label}_g3r2c_postflight.json"
  [[ -s "${report}" ]] || fail "previous confirmation postflight is missing: ${report}"
  python3 - "${report}" "${row}" "${PREREG_SHA256}" "${SOURCE_REPORT_SHA256}" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("status") != "PASS":
    raise SystemExit("previous confirmation row is not PASS")
if str(report.get("row")) != sys.argv[2]:
    raise SystemExit("previous confirmation row binding mismatch")
if report.get("protocol") != "G3R2_robot_only_W5_S10_paired_confirmation_v1":
    raise SystemExit("previous confirmation protocol mismatch")
bindings = report.get("bindings", {})
if bindings.get("prereg_sha256") != sys.argv[3]:
    raise SystemExit("previous confirmation prereg mismatch")
if bindings.get("source_report_sha256") != sys.argv[4]:
    raise SystemExit("previous confirmation source binding mismatch")
PY
  echo "[G3R2-CONFIRM] previous Row ${row} PASS"
}

validate_launch_contract
validate_previous_row

if truthy "${VALIDATE_ONLY}"; then
  printf '[G3R2-CONFIRM] validate-only runner command:\n  env '
  printf '%q ' "${runner_env[@]}"
  printf 'bash %q\n' "${RUNNER}"
  echo "[G3R2-CONFIRM] prereg SHA256  = ${PREREG_SHA256}"
  echo "[G3R2-CONFIRM] order SHA256   = ${ORDER_SHA256}"
  echo "[G3R2-CONFIRM] window SHA256  = ${OUTCOME_WINDOW_RULE_SHA256}"
  echo "[G3R2-CONFIRM] runtime SHA256 = ${RUNTIME_BINARY_BUNDLE_SHA256}"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || fail "real motion is disarmed; set ARM_MOTION=YES"
[[ "${CONFIRM_RGB_GEOMETRY}" == "YES" ]] || fail "confirm RGB geometry before motion"
[[ ! -e "${BAG_PATH}" && ! -e "${BAG_PATH}.active" ]] || fail "attempt output already exists"
available_kib="$(df -Pk /home/geist/slosh_bags/real | awk 'NR == 2 {print $4}')"
required_kib=$((MIN_FREE_GIB * 1024 * 1024))
(( available_kib >= required_kib )) || fail "insufficient free space: require ${MIN_FREE_GIB} GiB"

mkdir -p "${RUN_OUT_DIR}"
if [[ -e "${PREREG_FILE}" ]]; then
  [[ "$(<"${PREREG_FILE}")" == "${expected_prereg}" ]] || fail "confirmation prereg changed"
  [[ "$(<"${ORDER_FILE}")" == "${expected_order}" ]] || fail "confirmation order changed"
  [[ "$(<"${METRIC_FILE}")" == "${expected_metric}" ]] || fail "confirmation metric changed"
  [[ "$(<"${ONLINE_CONFIG_FILE}")" == "${expected_online_config}" ]] || fail "confirmation online config changed"
else
  [[ "${G3R2C_ROW}" == "01" ]] || fail "Row 01 must create the confirmation prereg bundle"
  printf '%s\n' "${expected_prereg}" > "${PREREG_FILE}"
  printf '%s\n' "${expected_order}" > "${ORDER_FILE}"
  printf '%s\n' "${expected_metric}" > "${METRIC_FILE}"
  printf '%s\n' "${expected_online_config}" > "${ONLINE_CONFIG_FILE}"
  sha256sum "${PREREG_FILE}" "${ORDER_FILE}" "${METRIC_FILE}" "${ONLINE_CONFIG_FILE}" \
    > "${RUN_OUT_DIR}/G3R2_paired_confirmation_prereg_bundle.sha256"
fi

printf '%s\n' \
  "row=${G3R2C_ROW}" "block=${BLOCK}" "position=${POSITION}" "condition=${CONDITION}" \
  "screen_report_sha256=${SCREEN_REPORT_SHA256}" "prereg_sha256=${PREREG_SHA256}" \
  "order_sha256=${ORDER_SHA256}" "outcome_window_rule_sha256=${OUTCOME_WINDOW_RULE_SHA256}" \
  "timestamp_gate_impl=cpp_executable" \
  "timestamp_gate_sha256=${timestamp_gate_binary_sha}" \
  > "${RUN_OUT_DIR}/${RUN_LABEL}_g3r2c_binding.env"

bash "${CAMERA_PREP}"

run_timestamp_gate() {
  local phase="$1"
  local report="${RUN_OUT_DIR}/${RUN_LABEL}_realsense_timestamp_${phase}.json"
  "${TIMESTAMP_GATE}" --topic /camera/color/camera_info --samples 90 \
    --timeout-sec 50 --settle-until-pass --max-future-skew-sec 0.05 \
    --max-p95-lag-sec 0.20 --min-clock-rate-ratio 0.98 --max-clock-rate-ratio 1.02 \
    --max-gap-sec 0.20 --report "${report}" || \
    fail "RealSense timestamp ${phase} gate failed; motion was not started"
}

run_timestamp_gate pre_zero

publisher_count() {
  local info
  if ! info="$(rostopic info "$1" 2>/dev/null)"; then printf '0\n'; return 0; fi
  awk '/^Publishers:/ {inside=1; next} /^Subscribers:/ {inside=0} inside && /^[[:space:]]+\*/ {count++} END {print count+0}' <<< "${info}"
}
for topic in "${RGB_MEASUREMENT_TOPIC}" /liquid/height /liquid/height_lcr /liquid/height_median /liquid/debug_image; do
  count="$(publisher_count "${topic}")"
  [[ "${count}" == "0" ]] || fail "unexpected pre-existing publisher(s) on ${topic}: ${count}"
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
  hue2_low:=161 hue2_high:=179 sat_min:=101 val_min:=167 > "${online_log}" 2>&1 &
online_pid=$!
sleep 2
kill -0 "${online_pid}" 2>/dev/null || { tail -80 "${online_log}" >&2 || true; fail "online RGB exited"; }
ready_log="${RUN_OUT_DIR}/${RUN_LABEL}_online_liquid_ready.log"
timeout 20s rostopic echo -n 20 --filter "m.valid and m.zero_locked and m.status_code == 0" \
  "${RGB_MEASUREMENT_TOPIC}" > "${ready_log}" 2>&1 || fail "online RGB did not reach clean zero-lock"
ready_count="$(grep -Ec '^valid: (True|true)$' "${ready_log}" || true)"
(( ready_count >= 20 )) || fail "online RGB ready samples ${ready_count} < 20"
run_timestamp_gate pre_record

env "${runner_env[@]}" bash "${RUNNER}"

python3 "${VALIDATOR}" \
  --bag "${BAG_PATH}" --condition "${CONDITION}" --row "${G3R2C_ROW}" \
  --block "${BLOCK}" --position "${POSITION}" --slosh-enabled "${SLOSH_ENABLED}" \
  --smooth-priority-enabled "${SMOOTH_PRIORITY}" \
  --protocol G3R2_robot_only_W5_S10_paired_confirmation_v1 \
  --report-suffix _g3r2c_postflight.json --expected-weight "${WEIGHT}" \
  --expected-w-smooth 1.0 --expected-w-alpha 1.0 --expected-w-du-a 1.0 --expected-w-du-vs 1.0 \
  --expected-delay-mode-code 4 --require-delay-compensation-applied true \
  --require-robot-delay-compensation-applied true \
  --require-liquid-delay-compensation-applied false --require-state-diagnostics \
  --expected-solver-source-code 2 --expected-v-ref 0.20 --t-hvis-tail-sec "${T_HVIS_TAIL}" \
  --t-motion-max-sec "${T_MOTION_MAX}" --min-duration-sec 65 \
  --max-pre-motion-sec "${MAX_PRE_MOTION_SEC}" --max-contour-p95-m 0.05 \
  --max-yaw-p95-rad 0.15 --rgb-calibration-sha256 "${RGB_CALIBRATION_SHA256}" \
  --online-config-sha256 "${ONLINE_CONFIG_SHA256}" \
  --outcome-window-rule-sha256 "${OUTCOME_WINDOW_RULE_SHA256}" \
  --prereg-sha256 "${PREREG_SHA256}" --source-report-sha256 "${SOURCE_REPORT_SHA256}" --hash-bag

python3 "${SUMMARIZER}" "${BAG_PATH}"

if [[ "${G3R2C_ROW}" == "06" ]]; then
  python3 "${ANALYZER}" --root "${RUN_OUT_DIR}" --prereg-sha256 "${PREREG_SHA256}" \
    --outcome-window-rule-sha256 "${OUTCOME_WINDOW_RULE_SHA256}" \
    --source-report-sha256 "${SOURCE_REPORT_SHA256}" --screen-report-sha256 "${SCREEN_REPORT_SHA256}" \
    --minimum-mean-rgb-p95-improvement-mm 0.05 \
    --minimum-mean-rgb-rms-improvement-mm 0.0 \
    --maximum-mean-raw-imu-regression-mm 0.05 --minimum-positive-blocks 2
fi

echo "[G3R2-CONFIRM] acquisition PASS: ${BAG_PATH}"
echo "[G3R2-CONFIRM] Return to the start and let the liquid settle before the next row."
