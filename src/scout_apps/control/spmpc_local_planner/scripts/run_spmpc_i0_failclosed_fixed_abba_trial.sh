#!/usr/bin/env bash
# Development-only literal B0/B_slosh ABBA trial.
#
# This is a new 0.20/0.25 m/s development profile.  It reuses the frozen C02
# path/map/RGB artifacts, but it is NOT the released C02 0.10/0.15 profile and
# does not inherit C02's safety/evidence status.  Row 01 (B0) is the motion gate
# for this new profile.

set -euo pipefail

SCRIPT_NAME=run_spmpc_i0_failclosed_fixed_abba_trial
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RUNNER="${SCRIPT_DIR}/run_spmpc_real_fixed_path_trial.sh"
EXACT_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_i0_failclosed_fixed_abba_bag.py"
OBSERVER_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_slosh_nowcast_shadow_bag.py"
RGB_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_g3_online_rgb_trial.py"
RGB_ANALYZER="${SCRIPT_DIR}/analysis/analyze_i0_failclosed_fixed_abba_rgb.py"
CHAIN_POSTFLIGHT="${SCRIPT_DIR}/analysis/validate_mocap_execution_chain_bag.py"
SUMMARIZER="${SCRIPT_DIR}/summarize_spmpc_real_trial.py"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"
MAP_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_field_map.py"
CAMERA_PREP="${SCRIPT_DIR}/prepare_spmpc_g3_realsense.sh"
CONTRACT_TEST="${SCRIPT_DIR}/tests/test_i0_failclosed_fixed_abba_contract.py"
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
      "${expected_phase}" "${expected_status}" "${expected_decision}" "${expected_rows}" <<'PY'
import json
import sys
from pathlib import Path

report_path, root, protocol, phase, status, decisions_csv, rows_csv = sys.argv[1:]
try:
    report = json.loads(Path(report_path).read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError) as exc:
    raise SystemExit("could not read RGB analysis JSON: {}".format(exc))

expected_rows = set(rows_csv.split(","))
expected_decisions = set(decisions_csv.split(","))
checks = {
    "report_type": report.get("report_type") == "I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS",
    "protocol": report.get("protocol") == protocol,
    "scope": report.get("scope") == "DEVELOPMENT_ONLY",
    "root": Path(str(report.get("root", ""))).resolve() == Path(root).resolve(),
    "phase": report.get("phase") == phase,
    "status": report.get("status") == status,
    "decision": report.get("decision") in expected_decisions,
    "rows": set(report.get("rows", {})) == expected_rows,
    "postflights": set(report.get("postflights", {})) == expected_rows,
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
    --maximum-slowdown-ratio 1.05 || analysis_rc=$?
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

case "${PAIR_ROW}" in
  01) BLOCK=01; POSITION=01; CONDITION=B0;     PILOT_METHOD=B0; VARIANT=B0;      W_SLOSH=0.0; SLOSH_ENABLED=false; OBSERVER_APPLIED=none ;;
  02) BLOCK=01; POSITION=02; CONDITION=Bslosh; PILOT_METHOD=W5; VARIANT=B_slosh; W_SLOSH=5.0; SLOSH_ENABLED=true;  OBSERVER_APPLIED=L22 ;;
  03) BLOCK=02; POSITION=01; CONDITION=Bslosh; PILOT_METHOD=W5; VARIANT=B_slosh; W_SLOSH=5.0; SLOSH_ENABLED=true;  OBSERVER_APPLIED=L22 ;;
  04) BLOCK=02; POSITION=02; CONDITION=B0;     PILOT_METHOD=B0; VARIANT=B0;      W_SLOSH=0.0; SLOSH_ENABLED=false; OBSERVER_APPLIED=none ;;
  *) fail "set PAIR_ROW=01..04; frozen ABBA order is B0,Bslosh,Bslosh,B0" ;;
esac
[[ "${ATTEMPT}" == "01" ]] || fail "development ABBA permits first attempt only (ATTEMPT=01)"

PROTOCOL_ID=SMPCC_I0_FAILCLOSED_FIXED_ABBA_DEV_V1
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

MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_i0_failclosed_fixed_abba/H0}"
RUN_LABEL="${RUN_LABEL:-DEV_I0FC_FIXED_${PAIR_ROW}_${CONDITION}_b${BLOCK}_p${POSITION}_a${ATTEMPT}}"
NAME="${NAME:-${RUN_LABEL}}"
BAG_PATH="${RUN_OUT_DIR}/${NAME}.bag"
EXACT_REPORT="${RUN_OUT_DIR}/${NAME}_i0_fixed_postflight.json"
OBSERVER_REPORT="${RUN_OUT_DIR}/${NAME}_observer_postflight.json"
CHAIN_REPORT="${RUN_OUT_DIR}/${NAME}_mocap_chain_postflight.json"
RGB_REPORT="${RUN_OUT_DIR}/${NAME}_i0_fixed_rgb_postflight.json"
RGB_ANALYSIS_REPORT="${RUN_OUT_DIR}/I0_FAILCLOSED_FIXED_ABBA_RGB_ANALYSIS.json"
UNIT_PASS="${RUN_OUT_DIR}/${NAME}_unit_pass.env"
PREREG_FILE="${RUN_OUT_DIR}/${PROTOCOL_ID}_prereg.env"
ORDER_FILE="${RUN_OUT_DIR}/${PROTOCOL_ID}_order.csv"

required_files=(
  "${RUNNER}" "${EXACT_POSTFLIGHT}" "${OBSERVER_POSTFLIGHT}" "${RGB_POSTFLIGHT}" "${RGB_ANALYZER}"
  "${CHAIN_POSTFLIGHT}" "${SUMMARIZER}" "${PATH_VALIDATOR}" "${MAP_VALIDATOR}"
  "${CAMERA_PREP}" "${CONTRACT_TEST}" "${ONLINE_LIQUID_LAUNCH}" "${ONLINE_LIQUID_NODE}"
  "${ONLINE_LIQUID_DETECTOR}" "${ONLINE_LIQUID_MSG}" "${PATH_FILE}"
  "${FIELD_MAP_FILE}" "${RGB_CALIBRATION_FILE}"
)
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

validate_launch_variant() {
  local variant="$1"
  local weight="$2"
  local slosh_enabled="$3"
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
    v_ref:=0.20 w_slosh:="${weight}")" || fail "could not dump ${variant} launch parameters"
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
  local line
  for line in "${expected_lines[@]}"; do
    grep -Fqx -- "${line}" <<< "${dump}" || fail "${variant} launch contract missing: ${line}"
  done
}

validate_launch_variant B0 0.0 false
validate_launch_variant B_slosh 5.0 true
python3 "${CONTRACT_TEST}"
VALIDATE_ONLY=true bash "${CAMERA_PREP}" >/dev/null

echo "================ I0 fail-closed fixed ABBA ================"
echo "  row/order      = ${PAIR_ROW}/04; B0,Bslosh,Bslosh,B0"
echo "  condition      = ${CONDITION}; ${VARIANT}; w_slosh=${W_SLOSH}"
echo "  observer       = processed-IMU I0; fail_closed; common_epoch=true"
echo "  solver liquid  = ${OBSERVER_APPLIED} (fixed L22 replaces I0 only for B_slosh)"
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
dirty_runtime="$(git -C "${REPO_ROOT}" status --porcelain --untracked-files=normal -- "${runtime_paths[@]}")"
[[ -z "${dirty_runtime}" ]] || fail "runtime/evidence paths are dirty; commit and rebuild before motion"

for output in "${BAG_PATH}" "${BAG_PATH}.active" "${EXACT_REPORT}" \
  "${OBSERVER_REPORT}" "${CHAIN_REPORT}" "${RGB_REPORT}" "${UNIT_PASS}"; do
  [[ ! -e "${output}" ]] || fail "preserve existing output: ${output}"
done

case "${PAIR_ROW}" in
  01) PREVIOUS_LABEL= ;;
  02) PREVIOUS_LABEL=DEV_I0FC_FIXED_01_B0_b01_p01_a01 ;;
  03) PREVIOUS_LABEL=DEV_I0FC_FIXED_02_Bslosh_b01_p02_a01 ;;
  04) PREVIOUS_LABEL=DEV_I0FC_FIXED_03_Bslosh_b02_p01_a01 ;;
esac
if [[ -n "${PREVIOUS_LABEL}" ]]; then
  previous_pass="${RUN_OUT_DIR}/${PREVIOUS_LABEL}_unit_pass.env"
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
order_contents=$'row,block,position,condition,variant,w_slosh\n01,01,01,B0,B0,0.0\n02,01,02,Bslosh,B_slosh,5.0\n03,02,01,Bslosh,B_slosh,5.0\n04,02,02,B0,B0,0.0'
prereg_contents="$(printf '%s\n' \
  "protocol=${PROTOCOL_ID}" "scope=development_only" "not_c02_release=true" \
  "row_order=B0,Bslosh,Bslosh,B0" "v_ref=${V_REF}" "v_safe_max=${V_SAFE_MAX}" \
  "observer=processed_imu" "fallback=fail_closed" "common_epoch=true" \
  "delay=fixed_closed_loop:${DELAY_PHASE_LINEAR_DELAY_SEC},${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
  "path_sha256=${FROZEN_PATH_SHA256}" "map_sha256=${FROZEN_MAP_SHA256}" \
  "rgb_calibration_sha256=${FROZEN_RGB_CALIBRATION_SHA256}" "git_revision=${code_revision}")"
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
else
  [[ "${PAIR_ROW}" == "01" ]] || fail "row 01 must create the prereg/order bundle"
  printf '%s\n' "${prereg_contents}" > "${PREREG_FILE}"
  printf '%s\n' "${order_contents}" > "${ORDER_FILE}"
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

DATE="${DATE}" STAMP="${STAMP}" PILOT_MODE=true PILOT_METHOD="${PILOT_METHOD}" \
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
BLOCK_SEGMENT_ID="I0FC_FIXED_b${BLOCK}" ORDER_POSITION="${POSITION}" SEND_ZERO_ON_EXIT=true \
OPERATOR_NOTE="new dev profile 0.20/0.25; literal B0/B_slosh; I0 source then legacy L22 fixed rollout" \
bash "${RUNNER}"

[[ -s "${BAG_PATH}" ]] || fail "bag missing after runner: ${BAG_PATH}"
python3 "${EXACT_POSTFLIGHT}" "${BAG_PATH}" --condition "${CONDITION}" \
  --report "${EXACT_REPORT}" --protocol "${PROTOCOL_ID}" \
  --expected-v-ref "${V_REF}" --expected-v-safe-max "${V_SAFE_MAX}" \
  --minimum-application-fraction 1.0

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
  --protocol "${PROTOCOL_ID}" --report-suffix _i0_fixed_rgb_postflight.json \
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
printf '%s\n' "status=PASS" "protocol=${PROTOCOL_ID}" "row=${PAIR_ROW}" \
  "condition=${CONDITION}" "bag=${BAG_PATH}" "completed_at=$(date --iso-8601=seconds)" > "${UNIT_PASS}"

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
