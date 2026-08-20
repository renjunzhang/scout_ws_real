#!/usr/bin/env bash
# G3R2 continuation: four one-run weight candidates after the frozen Bsmooth smoke.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G3R2-SCREEN][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
VALIDATOR="${SCRIPT_DIR}/../tools/analysis/validate_g3_online_rgb_trial.py"
ANALYZER="${SCRIPT_DIR}/../tools/analysis/analyze_g3r2_weight_screen.py"
SUMMARIZER="${SCRIPT_DIR}/../tools/analysis/summarize_spmpc_real_trial.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"
TIMESTAMP_GATE="${REPO_ROOT}/devel/lib/spmpc_local_planner/spmpc_realsense_timestamp_health_gate"

[[ -r /opt/ros/noetic/setup.bash ]] || fail "missing ROS Noetic setup"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]] || fail "missing ${REPO_ROOT}/devel/setup.bash"
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

G3R2_ROW="${G3R2_ROW:-}"
G3R2_ATTEMPT="${G3R2_ATTEMPT:-01}"
DATE="${DATE:-20260801}"
STAMP="${STAMP:-$(date +%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ALLOW_DIRTY_VALIDATE_ONLY="${ALLOW_DIRTY_VALIDATE_ONLY:-false}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_RGB_GEOMETRY="${CONFIRM_RGB_GEOMETRY:-NO}"

case "${G3R2_ROW}" in
  02) CONDITION=W2_S03; PILOT_METHOD=W2; W_SLOSH=2.0; SMOOTH=0.3 ;;
  03) CONDITION=W5_S10; PILOT_METHOD=W5; W_SLOSH=5.0; SMOOTH=1.0 ;;
  04) CONDITION=W2_S10; PILOT_METHOD=W2; W_SLOSH=2.0; SMOOTH=1.0 ;;
  05) CONDITION=W5_S03; PILOT_METHOD=W5; W_SLOSH=5.0; SMOOTH=0.3 ;;
  *) fail "set G3R2_ROW=02..05; the frozen Bsmooth smoke already supplies Row 01" ;;
esac
if [[ "${G3R2_ATTEMPT}" == "01" ]]; then
  ACQUISITION_RETRY=false
elif [[ "${G3R2_ROW}" == "03" && "${G3R2_ATTEMPT}" == "02" ]]; then
  ACQUISITION_RETRY=true
else
  fail "only the frozen Row 03 acquisition retry permits G3R2_ATTEMPT=02"
fi

PATH_FILE="/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json"
PATH_SHA256="578a4dd7663c2f49b4270c37755a08b2b0dc70735fb6b818da35b60a60f3990e"
SOURCE_REPORT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/analysis/G2S_RAW_RGB_THREE_TRIAL_DEVELOPMENT_REPORT.json"
SOURCE_REPORT_SHA256="36db4d12948f98e4ef3580bc52367cdef14d8440d80a8afbb32182b704ee5442"

BASELINE_ROOT="/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_smoke/H0"
BASELINE_BAG="${BASELINE_ROOT}/DEV_G3R2_H0_C1_Bsmooth_robot_only_r01_a01.bag"
BASELINE_BAG_SHA256="63399f5f6e80c9afe438e5dae65942c43545c4b2c54bffa866579d6a105820ba"
BASELINE_REPORT="${BASELINE_ROOT}/DEV_G3R2_H0_C1_Bsmooth_robot_only_r01_a01_g3r2_postflight.json"
BASELINE_REPORT_SHA256="9f645ae70b385d15935207d8b980bdd5a6ec5d1bcab2109056c5b3513a99e784"

# Historical evidence below remains bound to the release that produced it.
# New runs bind their preregistration to runtime_revision after the C++ gate
# migration and must not reuse that historical release identity.
SCREEN_EVIDENCE_RELEASE_REVISION="795a0da1abfa0abe7044de16052cefaddaf34411"
FAILED_ROW03_BAG="/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/DEV_G3R2_H0_C1_W5_S10_r03_a01.bag"
FAILED_ROW03_BAG_SHA256="52518e6d66758afedf156fe01ab7e118f6f5e390135b8b5dc80030acff031a2d"
FAILED_ROW03_REPORT="/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/DEV_G3R2_H0_C1_W5_S10_r03_a01_g3r2_screen_postflight.json"
FAILED_ROW03_REPORT_SHA256="7a49e052af66fb2073cc01e0f3af3df7aeff1595c1d53553cfd5768292a8940e"
FAILURE_EVIDENCE="${REPO_ROOT}/docs/实物实验注意事项/对比试验/实物对比实验/正式论文实验/20260801_G3R2_Row03_a01_相机时间戳采集失败证据.env"
FAILURE_EVIDENCE_SHA256="a5209f0723d1c9f50b5d1511ac8d6b31be06fed91bd24c87e7f33e52095eb543"
RETRY_AUTHORIZATION="${REPO_ROOT}/docs/实物实验注意事项/对比试验/实物对比实验/正式论文实验/20260801_G3R2_Row03_a02_相机时间戳采集重试授权.env"
RETRY_AUTHORIZATION_SHA256="599a24a8d556f66b5e43a72cdd5d5f11652a2c7ee5edc107a447e7aab46ed552"
METHOD_NEGATIVE_ROW04_BAG="/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/DEV_G3R2_H0_C1_W2_S10_r04_a01.bag"
METHOD_NEGATIVE_ROW04_BAG_SHA256="5b6fa9af7fca5cc381c8d6ab06504162d8ebb54e8c77a74e85ee58f244effe60"
METHOD_NEGATIVE_ROW04_REPORT="/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/DEV_G3R2_H0_C1_W2_S10_r04_a01_g3r2_screen_postflight.json"
METHOD_NEGATIVE_ROW04_REPORT_SHA256="8f1c51266dc4edd6a51b455eeb098fcf685f037b335cbd6e142ac46af7bccedc"
METHOD_NEGATIVE_ROW04_PRE_ZERO="/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/DEV_G3R2_H0_C1_W2_S10_r04_a01_realsense_timestamp_pre_zero.json"
METHOD_NEGATIVE_ROW04_PRE_ZERO_SHA256="21328bd09b6a661129c0fc2d74d5294a8e7c5c923ec949ddab3d8a669bec2c98"
METHOD_NEGATIVE_ROW04_PRE_RECORD="/home/geist/slosh_bags/real/20260801_spmpc_g3r2_robot_only_weight_screen/H0/DEV_G3R2_H0_C1_W2_S10_r04_a01_realsense_timestamp_pre_record.json"
METHOD_NEGATIVE_ROW04_PRE_RECORD_SHA256="134932be3a74b44da77018eb9b29dfeac987b95a59fdd391dd9fb43790903c2c"
METHOD_OUTCOME_EVIDENCE="${REPO_ROOT}/docs/实物实验注意事项/对比试验/实物对比实验/正式论文实验/20260801_G3R2_Row04_a01_跟踪门槛方法失败证据.env"
METHOD_OUTCOME_EVIDENCE_SHA256="e1bc6e81a3e943ecf51a7cfd885ace45eb3cbd8f7c22e9eae573168367b12ade"
SCREEN_CONTINUATION_AUTHORIZATION="${REPO_ROOT}/docs/实物实验注意事项/对比试验/实物对比实验/正式论文实验/20260801_G3R2_Row05_方法失败后继续筛选授权.env"
SCREEN_CONTINUATION_AUTHORIZATION_SHA256="abcbb8d5dbfc043c8dd5ae8224e303fa90266edaf7b3a27e26d8b715a000adc9"
RETRY_OF_ATTEMPT_ID=""
RETRY_REASON_FILE=""
REPORT_SUFFIX="_g3r2_screen_postflight.json"
if truthy "${ACQUISITION_RETRY}"; then
  RETRY_OF_ATTEMPT_ID="DEV_G3R2_H0_C1_W5_S10_r03_a01"
  RETRY_REASON_FILE="${RETRY_AUTHORIZATION}"
  REPORT_SUFFIX="_g3r2_screen_retry_postflight.json"
fi

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
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g3r2_robot_only_weight_screen/H0}"
PREREG_FILE="${RUN_OUT_DIR}/G3R2_weight_screen_prereg.env"
ORDER_FILE="${RUN_OUT_DIR}/G3R2_weight_screen_order.csv"
METRIC_FILE="${RUN_OUT_DIR}/G3R2_weight_screen_metric.yaml"
ONLINE_CONFIG_FILE="${RUN_OUT_DIR}/G3R2_online_liquid_config.env"

required_files=(
  "${RUNNER}" "${VALIDATOR}" "${ANALYZER}" "${SUMMARIZER}" "${CAMERA_PREP}" "${TIMESTAMP_GATE}"
  "${PATH_FILE}" "${SOURCE_REPORT}" "${BASELINE_BAG}" "${BASELINE_REPORT}"
  "${FAILED_ROW03_BAG}" "${FAILED_ROW03_REPORT}" "${FAILURE_EVIDENCE}" "${RETRY_AUTHORIZATION}"
  "${METHOD_NEGATIVE_ROW04_BAG}" "${METHOD_NEGATIVE_ROW04_REPORT}"
  "${METHOD_NEGATIVE_ROW04_PRE_ZERO}" "${METHOD_NEGATIVE_ROW04_PRE_RECORD}"
  "${METHOD_OUTCOME_EVIDENCE}" "${SCREEN_CONTINUATION_AUTHORIZATION}"
  "${RGB_CALIBRATION_FILE}" "${RGB_CAMERA_PARAMS_FILE}"
  "${ONLINE_LIQUID_LAUNCH}" "${ONLINE_LIQUID_NODE}" "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}"
  "${PLANNER_LIBRARY}" "${PLANNER_NODE}" "${SLOSH_MODELS_LIBRARY}"
  "${ACADOS_B0_SOLVER}" "${ACADOS_SLOSH_SOLVER}" "${ACADOS_LIBRARY}" "${HPIPM_LIBRARY}" "${BLASFEO_LIBRARY}"
)
for required_file in "${required_files[@]}"; do
  [[ -s "${required_file}" ]] || fail "missing required artifact: ${required_file}"
done
[[ -x "${TIMESTAMP_GATE}" ]] || fail "timestamp gate is not executable: ${TIMESTAMP_GATE}"

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
verify_sha256 "${BASELINE_BAG}" "${BASELINE_BAG_SHA256}" "G3R2 baseline bag"
verify_sha256 "${BASELINE_REPORT}" "${BASELINE_REPORT_SHA256}" "G3R2 baseline postflight"
verify_sha256 "${FAILED_ROW03_BAG}" "${FAILED_ROW03_BAG_SHA256}" "failed Row 03 bag"
verify_sha256 "${FAILED_ROW03_REPORT}" "${FAILED_ROW03_REPORT_SHA256}" "failed Row 03 postflight"
verify_sha256 "${FAILURE_EVIDENCE}" "${FAILURE_EVIDENCE_SHA256}" "Row 03 failure evidence"
verify_sha256 "${RETRY_AUTHORIZATION}" "${RETRY_AUTHORIZATION_SHA256}" "Row 03 retry authorization"
verify_sha256 "${METHOD_NEGATIVE_ROW04_BAG}" "${METHOD_NEGATIVE_ROW04_BAG_SHA256}" "method-negative Row 04 bag"
verify_sha256 "${METHOD_NEGATIVE_ROW04_REPORT}" "${METHOD_NEGATIVE_ROW04_REPORT_SHA256}" "method-negative Row 04 postflight"
verify_sha256 "${METHOD_NEGATIVE_ROW04_PRE_ZERO}" "${METHOD_NEGATIVE_ROW04_PRE_ZERO_SHA256}" "Row 04 pre-zero timestamp gate"
verify_sha256 "${METHOD_NEGATIVE_ROW04_PRE_RECORD}" "${METHOD_NEGATIVE_ROW04_PRE_RECORD_SHA256}" "Row 04 pre-record timestamp gate"
verify_sha256 "${METHOD_OUTCOME_EVIDENCE}" "${METHOD_OUTCOME_EVIDENCE_SHA256}" "Row 04 method-outcome evidence"
verify_sha256 "${SCREEN_CONTINUATION_AUTHORIZATION}" "${SCREEN_CONTINUATION_AUTHORIZATION_SHA256}" "Row 05 continuation authorization"
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

python3 - "${BASELINE_REPORT}" "${BASELINE_BAG_SHA256}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("status") != "PASS" or report.get("protocol") != "G3R2_robot_only_delay_smoke_v1":
    raise SystemExit("frozen G3R2 baseline is not a PASS smoke")
if report.get("row") != "01" or report.get("condition") != "Bsmooth":
    raise SystemExit("frozen G3R2 baseline row binding mismatch")
if report.get("bag_sha256") != sys.argv[2]:
    raise SystemExit("frozen G3R2 baseline bag binding mismatch")
config = report.get("effective_config_last", {})
internal = report.get("internal_state", {})
if float(config.get("delay_phase_mode_code", -1.0)) != 4.0:
    raise SystemExit("baseline did not run fixed_robot_only")
if float(internal.get("robot_delay_compensation_applied_fraction", 0.0)) < 0.98:
    raise SystemExit("baseline lacks robot delay compensation")
if float(internal.get("liquid_delay_compensation_applied_fraction", 1.0)) > 0.02:
    raise SystemExit("baseline applied forbidden liquid delay rollout")
if float(internal.get("solver_source_code_fraction", 0.0)) < 0.98:
    raise SystemExit("baseline lacks processed-IMU source coverage")
PY

python3 - \
  "${FAILURE_EVIDENCE}" "${RETRY_AUTHORIZATION}" "${FAILED_ROW03_REPORT}" \
  "${FAILURE_EVIDENCE_SHA256}" "${FAILED_ROW03_BAG_SHA256}" \
  "${FAILED_ROW03_REPORT_SHA256}" "${SCREEN_EVIDENCE_RELEASE_REVISION}" <<'PY'
import json
import sys


def read_unique_env(path):
    result = {}
    with open(path, encoding="utf-8") as stream:
        for line_number, raw in enumerate(stream, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise SystemExit("invalid retry evidence line {}".format(line_number))
            key, value = line.split("=", 1)
            if key in result:
                raise SystemExit("duplicate retry evidence key {}".format(key))
            result[key] = value
    return result


evidence = read_unique_env(sys.argv[1])
authorization = read_unique_env(sys.argv[2])
with open(sys.argv[3], encoding="utf-8") as stream:
    failed = json.load(stream)

expected_authorization = {
    "report_type": "DEVELOPMENT_RETRY_AUTHORIZATION",
    "status": "PASS",
    "planned_row": "03",
    "condition": "W5_S10",
    "failed_attempt_id": "DEV_G3R2_H0_C1_W5_S10_r03_a01",
    "authorized_attempt_id": "DEV_G3R2_H0_C1_W5_S10_r03_a02",
    "retry_of_attempt_id": "DEV_G3R2_H0_C1_W5_S10_r03_a01",
    "block_segment_id": "G3R2_screen",
    "split_block": "false",
    "failure_class": "METHOD_INDEPENDENT_ACQUISITION",
    "failure_reason_code": "REALSENSE_SOURCE_TIMESTAMP_UNSTABLE_AT_VISUAL_START",
    "condition_independent": "true",
    "condition_specific": "false",
    "motion_induced": "false",
    "method_failure": "false",
    "retry_authorized": "true",
    "maximum_authorized_attempt": "02",
    "original_release_revision": sys.argv[7],
    "screen_prereg_sha256": "0a8f0af5395bc930d84ac4359ba4716e493acf007148781c45d3e2669993fe85",
    "failure_evidence_manifest_sha256": sys.argv[4],
    "failed_bag_sha256": sys.argv[5],
    "failed_postflight_sha256": sys.argv[6],
    "attempt_01_artifacts_must_be_preserved": "true",
    "attempt_02_must_keep_identical_method_configuration": "true",
}
for key, value in expected_authorization.items():
    if authorization.get(key) != value:
        raise SystemExit("retry authorization mismatch for {}".format(key))
if evidence.get("status") != "PASS" or evidence.get("failure_class") != "METHOD_INDEPENDENT_ACQUISITION":
    raise SystemExit("failure evidence did not classify a method-independent acquisition fault")
if evidence.get("failed_bag_sha256") != sys.argv[5] or evidence.get("failed_postflight_sha256") != sys.argv[6]:
    raise SystemExit("failure evidence artifact binding mismatch")
if failed.get("status") != "FAIL" or failed.get("row") != "03" or failed.get("condition") != "W5_S10":
    raise SystemExit("failed Row 03 postflight identity mismatch")
expected_failures = {
    "online publish-lag P95 0.587s > 0.500s",
    "online source stamp future skew 1.944s > 0.050s",
}
if set(failed.get("failures", [])) != expected_failures:
    raise SystemExit("Row 03 failure is not limited to the frozen timestamp acquisition faults")
internal = failed.get("internal_state", {})
tracking = failed.get("tracking", {})
if float(internal.get("robot_delay_compensation_applied_fraction", 0.0)) < 0.98:
    raise SystemExit("failed Row 03 also lacks robot delay compensation")
if float(internal.get("liquid_delay_compensation_applied_fraction", 1.0)) > 0.02:
    raise SystemExit("failed Row 03 also applied liquid delay rollout")
if float(internal.get("solver_source_code_fraction", 0.0)) < 0.98:
    raise SystemExit("failed Row 03 also lacks processed-IMU coverage")
if float(tracking.get("contour_p95_m", 1.0)) > 0.05 or float(tracking.get("yaw_p95_rad", 1.0)) > 0.15:
    raise SystemExit("failed Row 03 also violated tracking gates")
PY

python3 - \
  "${METHOD_OUTCOME_EVIDENCE}" "${SCREEN_CONTINUATION_AUTHORIZATION}" \
  "${METHOD_NEGATIVE_ROW04_REPORT}" "${METHOD_NEGATIVE_ROW04_PRE_ZERO}" \
  "${METHOD_NEGATIVE_ROW04_PRE_RECORD}" "${METHOD_OUTCOME_EVIDENCE_SHA256}" \
  "${METHOD_NEGATIVE_ROW04_BAG_SHA256}" "${METHOD_NEGATIVE_ROW04_REPORT_SHA256}" \
  "${METHOD_NEGATIVE_ROW04_PRE_ZERO_SHA256}" "${METHOD_NEGATIVE_ROW04_PRE_RECORD_SHA256}" \
  "${SCREEN_EVIDENCE_RELEASE_REVISION}" <<'PY'
import json
import math
import sys


def read_unique_env(path):
    result = {}
    with open(path, encoding="utf-8") as stream:
        for line_number, raw in enumerate(stream, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise SystemExit("invalid method-outcome evidence line {}".format(line_number))
            key, value = line.split("=", 1)
            if key in result:
                raise SystemExit("duplicate method-outcome evidence key {}".format(key))
            result[key] = value
    return result


evidence = read_unique_env(sys.argv[1])
authorization = read_unique_env(sys.argv[2])
with open(sys.argv[3], encoding="utf-8") as stream:
    failed = json.load(stream)
with open(sys.argv[4], encoding="utf-8") as stream:
    pre_zero = json.load(stream)
with open(sys.argv[5], encoding="utf-8") as stream:
    pre_record = json.load(stream)

expected_authorization = {
    "report_type": "DEVELOPMENT_SCREEN_CONTINUATION_AUTHORIZATION",
    "status": "PASS",
    "scope": "G3R2_DEVELOPMENT_SINGLE_RUN_SCREEN",
    "continuation_authorized": "true",
    "failed_method_row": "04",
    "failed_method_condition": "W2_S10",
    "failed_method_attempt_id": "DEV_G3R2_H0_C1_W2_S10_r04_a01",
    "next_authorized_row": "05",
    "next_authorized_condition": "W5_S03",
    "next_authorized_attempt_id": "DEV_G3R2_H0_C1_W5_S03_r05_a01",
    "outcome_class": "METHOD_PERFORMANCE_FAILURE",
    "failure_reason_code": "TRACKING_CONTOUR_P95_GATE_EXCEEDED",
    "method_failure": "true",
    "acquisition_failure": "false",
    "row04_retry_authorized": "false",
    "row04_candidate_eligible_for_promotion": "false",
    "row04_must_remain_in_dataset": "true",
    "row05_must_keep_original_configuration": "true",
    "row05_replicates": "1",
    "screen_prereg_is_unchanged": "true",
    "formal_efficacy_claim_authorized": "false",
    "original_release_revision": sys.argv[11],
    "screen_prereg_sha256": "0a8f0af5395bc930d84ac4359ba4716e493acf007148781c45d3e2669993fe85",
    "method_outcome_evidence_sha256": sys.argv[6],
    "failed_bag_sha256": sys.argv[7],
    "failed_postflight_sha256": sys.argv[8],
    "failed_artifacts_must_be_preserved": "true",
    "analysis_policy": "RETAIN_ROW04_AS_INELIGIBLE_METHOD_NEGATIVE",
}
for key, value in expected_authorization.items():
    if authorization.get(key) != value:
        raise SystemExit("screen continuation authorization mismatch for {}".format(key))

expected_evidence = {
    "report_type": "DEVELOPMENT_METHOD_OUTCOME_EVIDENCE",
    "status": "PASS",
    "outcome_class": "METHOD_PERFORMANCE_FAILURE",
    "failure_reason_code": "TRACKING_CONTOUR_P95_GATE_EXCEEDED",
    "method_failure": "true",
    "acquisition_failure": "false",
    "planned_row": "04",
    "condition": "W2_S10",
    "attempt_id": "DEV_G3R2_H0_C1_W2_S10_r04_a01",
    "original_release_revision": sys.argv[11],
    "screen_prereg_sha256": "0a8f0af5395bc930d84ac4359ba4716e493acf007148781c45d3e2669993fe85",
    "bag_sha256": sys.argv[7],
    "postflight_sha256": sys.argv[8],
    "timestamp_pre_zero_sha256": sys.argv[9],
    "timestamp_pre_record_sha256": sys.argv[10],
    "candidate_eligible_for_promotion": "false",
    "row04_retry_authorized": "false",
    "artifacts_must_be_preserved": "true",
}
for key, value in expected_evidence.items():
    if evidence.get(key) != value:
        raise SystemExit("method-outcome evidence mismatch for {}".format(key))

if failed.get("status") != "FAIL" or failed.get("row") != "04" or failed.get("condition") != "W2_S10":
    raise SystemExit("method-negative Row 04 identity mismatch")
if failed.get("protocol") != "G3R2_robot_only_weight_screen_v1":
    raise SystemExit("method-negative Row 04 protocol mismatch")
if failed.get("bag_sha256") != sys.argv[7]:
    raise SystemExit("method-negative Row 04 bag binding mismatch")
if failed.get("failures") != ["stage-0 contour P95 0.053296m > 0.050000m"]:
    raise SystemExit("Row 04 failure is not limited to the frozen tracking gate")
config = failed.get("effective_config_last", {})
if not math.isclose(float(config.get("w_slosh", -1.0)), 2.0, abs_tol=1.0e-6):
    raise SystemExit("method-negative Row 04 w_slosh mismatch")
for field in ("w_smooth", "w_alpha", "w_du_a", "w_du_vs"):
    if not math.isclose(float(config.get(field, -1.0)), 1.0, abs_tol=1.0e-6):
        raise SystemExit("method-negative Row 04 {} mismatch".format(field))
internal = failed.get("internal_state", {})
processed = failed.get("processed_imu", {})
tracking = failed.get("tracking", {})
online = failed.get("online_rgb", {})
if float(internal.get("robot_delay_compensation_applied_fraction", 0.0)) < 0.98:
    raise SystemExit("method-negative Row 04 lacks robot delay compensation")
if float(internal.get("liquid_delay_compensation_applied_fraction", 1.0)) > 0.02:
    raise SystemExit("method-negative Row 04 applied forbidden liquid rollout")
if float(internal.get("solver_source_code_fraction", 0.0)) < 0.98:
    raise SystemExit("method-negative Row 04 lacks processed-IMU source coverage")
if float(processed.get("ready_fraction", 0.0)) < 0.98 or int(processed.get("fallback_samples", -1)) != 0:
    raise SystemExit("method-negative Row 04 has an IMU readiness/fallback failure")
if processed.get("reset_epochs") != [0]:
    raise SystemExit("method-negative Row 04 has an observer reset")
if not math.isclose(float(tracking.get("contour_p95_m", 0.0)), 0.053296396508812904, abs_tol=1.0e-12):
    raise SystemExit("method-negative Row 04 contour metric changed")
if float(tracking.get("yaw_p95_rad", 1.0)) > 0.15:
    raise SystemExit("method-negative Row 04 also violated yaw tracking")
if float(online.get("motion_tail_valid_fraction", 0.0)) < 0.98:
    raise SystemExit("method-negative Row 04 lacks RGB coverage")
if failed.get("image_stream_audit", {}).get("count") != 0:
    raise SystemExit("method-negative Row 04 recorded a forbidden image stream")
for label, gate in (("pre-zero", pre_zero), ("pre-record", pre_record)):
    if gate.get("status") != "PASS" or gate.get("failures"):
        raise SystemExit("method-negative Row 04 {} timestamp gate failed".format(label))
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
  src/scout_apps/control/spmpc_local_planner/scripts/run_spmpc_g3r2_weight_screen_trial.sh
  src/scout_apps/control/spmpc_local_planner/tools/analysis/validate_g3_online_rgb_trial.py
  src/scout_apps/control/spmpc_local_planner/tools/analysis/analyze_g3r2_weight_screen.py
  src/scout_apps/control/spmpc_local_planner/scripts/prepare_spmpc_g3_realsense.sh
  src/scout_apps/control/spmpc_local_planner/tools/analysis/summarize_spmpc_real_trial.py
  src/scout_apps/sensors/realsense_liquid_measurement/launch/online_liquid_height.launch
  src/scout_apps/sensors/realsense_liquid_measurement/msg/OnlineLiquidMeasurement.msg
  src/scout_apps/sensors/realsense_liquid_measurement/scripts/online_liquid_height_node.py
  src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py
  docs/实物实验注意事项/对比试验/实物对比实验/正式论文实验/20260801_G3R2_Row03_a01_相机时间戳采集失败证据.env
  docs/实物实验注意事项/对比试验/实物对比实验/正式论文实验/20260801_G3R2_Row03_a02_相机时间戳采集重试授权.env
  docs/实物实验注意事项/对比试验/实物对比实验/正式论文实验/20260801_G3R2_Row04_a01_跟踪门槛方法失败证据.env
  docs/实物实验注意事项/对比试验/实物对比实验/正式论文实验/20260801_G3R2_Row05_方法失败后继续筛选授权.env
)
tracked_missing=()
for path in "${relevant_repo_paths[@]}"; do
  if [[ -f "${REPO_ROOT}/${path}" ]] && ! git -C "${REPO_ROOT}" ls-files --error-unmatch "${path}" >/dev/null 2>&1; then
    tracked_missing+=("${path}")
  fi
done
if (( ${#tracked_missing[@]} > 0 )); then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G3R2-SCREEN][WARN] developer validate-only with untracked code: ${tracked_missing[*]}" >&2
  else
    fail "G3R2 screen amendment is not frozen in revision ${runtime_revision}: ${tracked_missing[*]}"
  fi
fi
if ! git -C "${REPO_ROOT}" diff --quiet -- "${relevant_repo_paths[@]}" || \
   ! git -C "${REPO_ROOT}" diff --cached --quiet -- "${relevant_repo_paths[@]}"; then
  if truthy "${VALIDATE_ONLY}" && truthy "${ALLOW_DIRTY_VALIDATE_ONLY}"; then
    echo "[G3R2-SCREEN][WARN] developer validate-only with dirty relevant paths" >&2
  else
    fail "G3R2 screen runtime differs from amendment ${runtime_revision}; commit/freeze before motion"
  fi
fi

online_node_sha="$(sha256sum "${ONLINE_LIQUID_NODE}" | awk '{print $1}')"
online_detector_sha="$(sha256sum "${ONLINE_LIQUID_DETECTOR}" | awk '{print $1}')"
online_msg_sha="$(sha256sum "${ONLINE_LIQUID_MSG}" | awk '{print $1}')"
timestamp_gate_binary_sha="$(sha256sum "${TIMESTAMP_GATE}" | awk '{print $1}')"

order_contents() {
  printf '%s\n' \
    "row,condition,pilot_method,w_slosh,w_smooth,w_alpha,w_du_a,w_du_vs,replicates" \
    "01,Bsmooth,Bsmooth,0.0,1.0,1.0,1.0,1.0,FROZEN_SMOKE" \
    "02,W2_S03,W2,2.0,0.3,0.3,0.3,0.3,1" \
    "03,W5_S10,W5,5.0,1.0,1.0,1.0,1.0,1" \
    "04,W2_S10,W2,2.0,1.0,1.0,1.0,1.0,1" \
    "05,W5_S03,W5,5.0,0.3,0.3,0.3,0.3,1"
}

metric_contents() {
  printf '%s\n' \
    "schema_version: 1" \
    "baseline_report_sha256: ${BASELINE_REPORT_SHA256}" \
    "primary_metric: h_vis_p95_motion_plus_tail_mm" \
    "secondary_metric: h_vis_rms_motion_plus_tail_mm" \
    "observer_guard: raw_processed_imu_p95_must_not_regress_more_than_0.05_mm" \
    "screen_minimum_p95_improvement_mm: 0.05" \
    "screen_minimum_rms_improvement_mm: 0.0" \
    "rolling_median_window: 5" \
    "t_hvis_tail_sec: ${T_HVIS_TAIL}" \
    "t_motion_max_sec: ${T_MOTION_MAX}" \
    "maximum_record_to_motion_sec: ${MAX_PRE_MOTION_SEC}" \
    "image_stream_policy: forbid_all"
}

online_config_contents() {
  printf '%s\n' \
    "protocol=G3R2_robot_only_weight_screen_v1" \
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
    "protocol=G3R2_robot_only_weight_screen_v1" \
    "scope=development_single_run_screen" \
    "formal_stage_status=NO_GO" \
    "baseline_is_external_frozen_smoke=true" \
    "baseline_bag_sha256=${BASELINE_BAG_SHA256}" \
    "baseline_report_sha256=${BASELINE_REPORT_SHA256}" \
    "planned_candidate_units=4" \
    "replicates_per_candidate=1" \
    "release_revision=${runtime_revision}" \
    "source_report_sha256=${SOURCE_REPORT_SHA256}" \
    "path_sha256=${PATH_SHA256}" \
    "delay_phase_mode=fixed_robot_only" \
    "delay_phase_mode_code=4" \
    "robot_delay_compensation=true" \
    "liquid_delay_compensation=false" \
    "observer_source=processed_imu" \
    "observer_fallback_policy=fail_closed" \
    "imu_subscriber_queue_size=10" \
    "v_ref=0.20" \
    "order_sha256=${ORDER_SHA256}" \
    "outcome_window_rule_sha256=${OUTCOME_WINDOW_RULE_SHA256}" \
    "online_config_sha256=${ONLINE_CONFIG_SHA256}" \
    "timestamp_gate_impl=cpp_executable" \
    "timestamp_gate_sha256=${timestamp_gate_binary_sha}" \
    "runtime_binary_bundle_sha256=${RUNTIME_BINARY_BUNDLE_SHA256}" \
    "record_sec=${RECORD_SEC}" \
    "positive_candidates_only_enter_paired_confirmation=true" \
    "single_run_is_not_efficacy_evidence=true"
}

expected_prereg="$(prereg_contents)"
PREREG_SHA256="$(printf '%s\n' "${expected_prereg}" | sha256sum | awk '{print $1}')"

RUN_LABEL="DEV_G3R2_H0_C1_${CONDITION}_r${G3R2_ROW}_a${G3R2_ATTEMPT}"
BAG_PATH="${RUN_OUT_DIR}/${RUN_LABEL}.bag"
runner_env=(
  "DATE=${DATE}" "STAMP=${STAMP}" "PILOT_MODE=true" "PILOT_METHOD=${PILOT_METHOD}"
  "PILOT_CONDITION=G3R2_robot_only_weight_screen"
  "PILOT_RECORD_RGB=false" "PILOT_RECORD_ONLINE_LIQUID=true"
  "RUN_LABEL=${RUN_LABEL}" "NAME=${RUN_LABEL}" "RUN_OUT_DIR=${RUN_OUT_DIR}"
  "PATH_SOURCE_MODE=replay" "PATH_FILE=${PATH_FILE}" "PATH_EXPECTED_SHA256=${PATH_SHA256}"
  "REQUIRE_PATH_HASH=true" "START_POS_TOL=0.08" "START_YAW_TOL=0.15"
  "START_HOLD_SEC=0.5" "START_GATE_TIMEOUT_SEC=3" "V_REF=0.20"
  "W_SLOSH=${W_SLOSH}" "W_SMOOTH=${SMOOTH}" "W_ALPHA=${SMOOTH}"
  "W_DU_A=${SMOOTH}" "W_DU_VS=${SMOOTH}"
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
  "ONLINE_LIQUID_PROTOCOL=G3R2_robot_only_weight_screen_v1"
  "ONLINE_LIQUID_CALIBRATION_SHA256=${RGB_CALIBRATION_SHA256}"
  "ONLINE_LIQUID_DETECTOR_SHA256=${online_detector_sha}"
  "ONLINE_LIQUID_NODE_SHA256=${online_node_sha}" "ONLINE_LIQUID_MSG_SHA256=${online_msg_sha}"
  "ONLINE_LIQUID_CONFIG_SHA256=${ONLINE_CONFIG_SHA256}"
  "RECORD_SEC=${RECORD_SEC}" "MAX_RECORD_SEC=${RECORD_SEC}"
  "BLOCK_SEGMENT_ID=G3R2_screen" "SPLIT_BLOCK=false" "ORDER_POSITION=${G3R2_ROW}"
  "ACQUISITION_RETRY=${ACQUISITION_RETRY}" "RETRY_REASON_FILE=${RETRY_REASON_FILE}"
  "SEND_ZERO_ON_EXIT=true"
  "OPERATOR_NOTE=G3R2_${CONDITION}_robot_only_attempt_${G3R2_ATTEMPT}_retry_of_${RETRY_OF_ATTEMPT_ID:-none}_baseline_${BASELINE_REPORT_SHA256}"
)

echo "================ G3R2 robot-only weight screen ================"
echo "  row/attempt   = ${G3R2_ROW}/${G3R2_ATTEMPT}"
echo "  condition     = ${CONDITION}"
echo "  weights       = w_slosh ${W_SLOSH}; smooth split ${SMOOTH}"
echo "  robot state   = fixed delay prediction"
echo "  liquid state  = current processed-IMU; no delay rollout"
echo "  baseline      = ${BASELINE_REPORT_SHA256}"
echo "  output        = ${BAG_PATH}"
echo "  acq retry     = ${ACQUISITION_RETRY} (${RETRY_OF_ATTEMPT_ID:-none})"
echo "  claim         = one-run screen only; positive candidate needs repeats"
echo "==============================================================="

validate_launch_contract() {
  local launch_dump
  launch_dump="$(roslaunch --dump-params spmpc_local_planner spmpc_fixed_path.launch \
    planner_variant:=B_slosh \
    delay_phase_mode:=fixed_robot_only \
    delay_phase_linear_delay_sec:=0.15 \
    delay_phase_angular_delay_sec:=0.22 \
    imu_shadow_enable:=true \
    imu_subscriber_queue_size:=10 \
    observer_source:=processed_imu \
    observer_fallback_policy:=fail_closed \
    observer_latch_fallback:=false \
    v_ref:=0.20 \
    w_slosh:="${W_SLOSH}" \
    w_smooth:="${SMOOTH}" \
    w_alpha:="${SMOOTH}" \
    w_du_a:="${SMOOTH}" \
    w_du_vs:="${SMOOTH}")" || fail "could not dump frozen launch parameters"
  local expected_lines=(
    "/spmpc_local_planner/planner_variant: B_slosh"
    "/spmpc_local_planner/delay_phase/mode: fixed_robot_only"
    "/spmpc_local_planner/delay_phase/linear_delay_sec: 0.15"
    "/spmpc_local_planner/delay_phase/angular_delay_sec: 0.22"
    "/spmpc_local_planner/imu_shadow/enable: true"
    "/spmpc_local_planner/imu_shadow/subscriber_queue_size: 10"
    "/spmpc_local_planner/slosh_observer/source: processed_imu"
    "/spmpc_local_planner/slosh_observer/fallback_policy: fail_closed"
    "/spmpc_local_planner/slosh_observer/latch_fallback: false"
    "/spmpc_local_planner/variants/B_slosh/w_slosh: ${W_SLOSH}"
    "/spmpc_local_planner/variants/B_slosh/w_smooth: ${SMOOTH}"
    "/spmpc_local_planner/variants/B_slosh/w_alpha: ${SMOOTH}"
    "/spmpc_local_planner/variants/B_slosh/w_du_a: ${SMOOTH}"
    "/spmpc_local_planner/variants/B_slosh/w_du_vs: ${SMOOTH}"
  )
  local expected_line
  for expected_line in "${expected_lines[@]}"; do
    grep -Fqx -- "${expected_line}" <<< "${launch_dump}" || \
      fail "launch contract missing: ${expected_line}"
  done
  echo "[G3R2-SCREEN] launch contract PASS (no planner started)"
}

validate_previous_row() {
  [[ "${G3R2_ROW}" != "02" ]] || return 0
  local previous_row previous_condition previous_attempt previous_suffix previous_report
  previous_row="$(printf '%02d' "$((10#${G3R2_ROW} - 1))")"
  case "${previous_row}" in
    02) previous_condition=W2_S03 ;;
    03) previous_condition=W5_S10 ;;
    04) previous_condition=W2_S10 ;;
    *) fail "unsupported previous G3R2 row: ${previous_row}" ;;
  esac
  previous_attempt=01
  previous_suffix=_g3r2_screen_postflight.json
  if [[ "${previous_row}" == "03" ]]; then
    previous_attempt=02
    previous_suffix=_g3r2_screen_retry_postflight.json
  fi
  previous_report="${RUN_OUT_DIR}/DEV_G3R2_H0_C1_${previous_condition}_r${previous_row}_a${previous_attempt}${previous_suffix}"
  [[ -s "${previous_report}" ]] || fail "previous row postflight is missing: ${previous_report}"
  if [[ "${previous_row}" == "04" && "${previous_report}" != "${METHOD_NEGATIVE_ROW04_REPORT}" ]]; then
    fail "Row 05 continuation is bound to the frozen method-negative Row 04 postflight"
  fi
  python3 - \
    "${previous_report}" "${previous_row}" "${PREREG_SHA256}" "${SOURCE_REPORT_SHA256}" \
    "${METHOD_NEGATIVE_ROW04_BAG_SHA256}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)
row = sys.argv[2]
if str(report.get("row")) != row:
    raise SystemExit("previous row binding mismatch")
if report.get("protocol") != "G3R2_robot_only_weight_screen_v1":
    raise SystemExit("previous protocol mismatch")
bindings = report.get("bindings", {})
if bindings.get("prereg_sha256") != sys.argv[3]:
    raise SystemExit("previous prereg binding mismatch")
if bindings.get("source_report_sha256") != sys.argv[4]:
    raise SystemExit("previous source binding mismatch")
if row == "04":
    if report.get("status") != "FAIL":
        raise SystemExit("authorized method-negative Row 04 is not FAIL")
    if report.get("condition") != "W2_S10":
        raise SystemExit("authorized method-negative Row 04 condition mismatch")
    if report.get("bag_sha256") != sys.argv[5]:
        raise SystemExit("authorized method-negative Row 04 bag mismatch")
    if report.get("failures") != ["stage-0 contour P95 0.053296m > 0.050000m"]:
        raise SystemExit("Row 04 has a failure outside the continuation authorization")
else:
    if report.get("status") != "PASS":
        raise SystemExit("previous G3R2 candidate row is not PASS")
PY
  if [[ "${previous_row}" == "04" ]]; then
    echo "[G3R2-SCREEN] previous Row 04 retained as an ineligible method-negative; Row 05 continuation PASS"
  else
    echo "[G3R2-SCREEN] previous Row ${previous_row} PASS"
  fi
}

validate_launch_contract
validate_previous_row

if truthy "${VALIDATE_ONLY}"; then
  printf '[G3R2-SCREEN] validate-only runner command:\n  env '
  printf '%q ' "${runner_env[@]}"
  printf 'bash %q\n' "${RUNNER}"
  echo "[G3R2-SCREEN] prereg SHA256  = ${PREREG_SHA256}"
  echo "[G3R2-SCREEN] order SHA256   = ${ORDER_SHA256}"
  echo "[G3R2-SCREEN] runtime SHA256 = ${RUNTIME_BINARY_BUNDLE_SHA256}"
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
  [[ "$(<"${PREREG_FILE}")" == "${expected_prereg}" ]] || fail "G3R2 screen prereg changed"
  [[ "$(<"${ORDER_FILE}")" == "${expected_order}" ]] || fail "G3R2 screen order changed"
  [[ "$(<"${METRIC_FILE}")" == "${expected_metric}" ]] || fail "G3R2 screen metric changed"
  [[ "$(<"${ONLINE_CONFIG_FILE}")" == "${expected_online_config}" ]] || fail "G3R2 screen online config changed"
else
  [[ "${G3R2_ROW}" == "02" ]] || fail "Row 02 must create the screen prereg bundle"
  printf '%s\n' "${expected_prereg}" > "${PREREG_FILE}"
  printf '%s\n' "${expected_order}" > "${ORDER_FILE}"
  printf '%s\n' "${expected_metric}" > "${METRIC_FILE}"
  printf '%s\n' "${expected_online_config}" > "${ONLINE_CONFIG_FILE}"
  sha256sum "${PREREG_FILE}" "${ORDER_FILE}" "${METRIC_FILE}" "${ONLINE_CONFIG_FILE}" \
    > "${RUN_OUT_DIR}/G3R2_screen_prereg_bundle.sha256"
fi

printf '%s\n' \
  "row=${G3R2_ROW}" "attempt=${G3R2_ATTEMPT}" "condition=${CONDITION}" \
  "w_slosh=${W_SLOSH}" "smooth=${SMOOTH}" \
  "baseline_report_sha256=${BASELINE_REPORT_SHA256}" "prereg_sha256=${PREREG_SHA256}" \
  "acquisition_retry=${ACQUISITION_RETRY}" "retry_of_attempt_id=${RETRY_OF_ATTEMPT_ID}" \
  "retry_authorization_sha256=${RETRY_AUTHORIZATION_SHA256}" \
  "method_outcome_evidence_sha256=${METHOD_OUTCOME_EVIDENCE_SHA256}" \
  "screen_continuation_authorization_sha256=${SCREEN_CONTINUATION_AUTHORIZATION_SHA256}" \
  "timestamp_gate_impl=cpp_executable" \
  "timestamp_gate_sha256=${timestamp_gate_binary_sha}" \
  > "${RUN_OUT_DIR}/${RUN_LABEL}_g3r2_screen_binding.env"

bash "${CAMERA_PREP}"

run_timestamp_gate() {
  local phase="$1"
  local report="${RUN_OUT_DIR}/${RUN_LABEL}_realsense_timestamp_${phase}.json"
  "${TIMESTAMP_GATE}" \
    --topic /camera/color/camera_info --samples 90 --timeout-sec 50 --settle-until-pass \
    --max-future-skew-sec 0.05 --max-p95-lag-sec 0.20 \
    --min-clock-rate-ratio 0.98 --max-clock-rate-ratio 1.02 \
    --max-gap-sec 0.20 --report "${report}" || \
    fail "RealSense timestamp ${phase} gate failed; no motion was started"
}

run_timestamp_gate pre_zero

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
run_timestamp_gate pre_record

env "${runner_env[@]}" bash "${RUNNER}"

python3 "${VALIDATOR}" \
  --bag "${BAG_PATH}" --condition "${CONDITION}" --row "${G3R2_ROW}" \
  --block screen --position "${G3R2_ROW}" --slosh-enabled true \
  --smooth-priority-enabled false \
  --protocol G3R2_robot_only_weight_screen_v1 --report-suffix "${REPORT_SUFFIX}" \
  --expected-weight "${W_SLOSH}" --expected-w-smooth "${SMOOTH}" \
  --expected-w-alpha "${SMOOTH}" --expected-w-du-a "${SMOOTH}" --expected-w-du-vs "${SMOOTH}" \
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

if [[ "${G3R2_ROW}" == "05" ]]; then
  set +e
  python3 "${ANALYZER}" --root "${RUN_OUT_DIR}" \
    --baseline-report "${BASELINE_REPORT}" --baseline-report-sha256 "${BASELINE_REPORT_SHA256}" \
    --screen-prereg-sha256 "${PREREG_SHA256}" --source-report-sha256 "${SOURCE_REPORT_SHA256}" \
    --retry-authorization "${RETRY_AUTHORIZATION}" \
    --retry-authorization-sha256 "${RETRY_AUTHORIZATION_SHA256}" \
    --method-failure-authorization "${SCREEN_CONTINUATION_AUTHORIZATION}" \
    --method-failure-authorization-sha256 "${SCREEN_CONTINUATION_AUTHORIZATION_SHA256}" \
    --minimum-rgb-p95-improvement-mm 0.05 --minimum-rgb-rms-improvement-mm 0.0 \
    --maximum-raw-imu-regression-mm 0.05
  analyzer_rc=$?
  set -e
  [[ -s "${RUN_OUT_DIR}/G3R2_WEIGHT_SCREEN_REPORT.json" ]] || fail "screen analyzer produced no report"
  [[ -s "${RUN_OUT_DIR}/G3R2_WEIGHT_SCREEN_METRICS.csv" ]] || fail "screen analyzer produced no CSV"
  screen_status="$(python3 - "${RUN_OUT_DIR}/G3R2_WEIGHT_SCREEN_REPORT.json" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    print(json.load(stream).get("status", ""))
PY
)"
  if [[ "${screen_status}" == "PROMOTE_FOR_PAIRED_CONFIRMATION" ]]; then
    (( analyzer_rc == 0 )) || fail "promotion report and analyzer exit code disagree"
    [[ -s "${RUN_OUT_DIR}/G3R2_CONFIRMATION_PLAN.draft.json" ]] || fail "promotion has no draft plan"
    echo "[G3R2-SCREEN] Positive candidate found; freeze before paired confirmation."
  elif [[ "${screen_status}" == "NO_PROMOTION" ]]; then
    (( analyzer_rc == 10 )) || fail "no-promotion report and analyzer exit code disagree"
    echo "[G3R2-SCREEN] No positive candidate; stop without repeats."
  else
    fail "screen invalid; inspect G3R2_WEIGHT_SCREEN_REPORT.json"
  fi
fi

echo "[G3R2-SCREEN] acquisition PASS: ${BAG_PATH}"
