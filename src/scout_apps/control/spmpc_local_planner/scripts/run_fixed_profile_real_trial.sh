#!/usr/bin/env bash
# Image-free real FixedProfile development runner for G5/formal integration.
# The sensor/localization stack must already be running.  This script starts
# only recorder, frozen profile tracker, and frozen path replay.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[FixedProfile][ERR] $*" >&2
  exit 2
}

child_running() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
RECORDER="${SCRIPT_DIR}/record_spmpc_full_rgb_bag.sh"

DATE="${DATE:-$(date +%Y%m%d)}"
STAMP="${STAMP:-$(date +%H%M%S)}"
RUN_LABEL="${RUN_LABEL:-DEV_G5_H0_C1_FixedProfile_a01}"
NAME="${NAME:-${RUN_LABEL}}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_g5_minimal/H0}"
PATH_FILE="${PATH_FILE:?set PATH_FILE to the frozen path JSON}"
PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256:?set PATH_EXPECTED_SHA256}"
PROFILE_FILE="${PROFILE_FILE:?set PROFILE_FILE to the frozen profile CSV}"
PROFILE_EXPECTED_SHA256="${PROFILE_EXPECTED_SHA256:?set PROFILE_EXPECTED_SHA256}"
PROFILE_CONFIG="${PROFILE_CONFIG:?set PROFILE_CONFIG to the frozen FixedProfile config}"
PROFILE_CONFIG_EXPECTED_SHA256="${PROFILE_CONFIG_EXPECTED_SHA256:?set PROFILE_CONFIG_EXPECTED_SHA256}"
G5_PREREG_SHA256="${G5_PREREG_SHA256:?set G5_PREREG_SHA256}"
ARM_MOTION="${ARM_MOTION:-NO}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
RECORD_SEC="${RECORD_SEC:-70}"
REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
BASE_FRAME="${BASE_FRAME:-base_link}"
START_POS_TOL="${START_POS_TOL:-0.08}"
START_YAW_TOL="${START_YAW_TOL:-0.15}"
START_HOLD_SEC="${START_HOLD_SEC:-0.5}"
START_GATE_TIMEOUT_SEC="${START_GATE_TIMEOUT_SEC:-10}"

for command in roslaunch rosrun rostopic rosnode timeout sha256sum; do
  command -v "${command}" >/dev/null 2>&1 || fail "missing command: ${command}"
done
for artifact in "${RECORDER}" "${PATH_FILE}" "${PROFILE_FILE}" "${PROFILE_CONFIG}"; do
  [[ -s "${artifact}" ]] || fail "missing artifact: ${artifact}"
done
[[ "$(sha256sum "${PATH_FILE}" | awk '{print $1}')" == "${PATH_EXPECTED_SHA256}" ]] || fail "path hash mismatch"
[[ "$(sha256sum "${PROFILE_FILE}" | awk '{print $1}')" == "${PROFILE_EXPECTED_SHA256}" ]] || fail "profile hash mismatch"
[[ "$(sha256sum "${PROFILE_CONFIG}" | awk '{print $1}')" == "${PROFILE_CONFIG_EXPECTED_SHA256}" ]] || fail "profile config hash mismatch"
case "${RECORD_SEC}" in ''|*[!0-9]*) fail "RECORD_SEC must be an integer" ;; esac
(( RECORD_SEC > 0 && RECORD_SEC <= 70 )) || fail "require 0 < RECORD_SEC <= 70"

bag_path="${RUN_OUT_DIR}/${NAME}.bag"
meta_path="${RUN_OUT_DIR}/${NAME}_fixed_profile_meta.env"
recorder_log="${RUN_OUT_DIR}/${NAME}_recorder.log"
planner_log="${RUN_OUT_DIR}/${NAME}_planner.log"
path_log="${RUN_OUT_DIR}/${NAME}_path_runner.log"

planner_cmd=(
  roslaunch scout_local_planner slosh_experiment.launch
  experiment_group:=LEGACY
  controller_variant:=mpc
  external_profile_mode:=custom_csv
  external_speed_profile_csv:="${PROFILE_FILE}"
  Q_slosh:=0.0
  global_path_topic:="${REF_TOPIC}"
  cmd_vel_topic:="${CMD_TOPIC}"
  filter_alpha_v:=1.0
  filter_alpha_omega:=1.0
  filter_kappa_boost:=0.0
  external_profile_execution_cap_enable:=true
  external_profile_execution_accel_limit:=0.6
  external_profile_execution_decel_limit:=0.6
  external_profile_execution_jerk_limit:=0.0
  max_lat_accel_safety:=2.5
  slosh_use_imu_lateral_accel:=false
  slosh_use_imu_yaw_rate:=false
  slosh_use_imu_alpha_z:=false
)
path_cmd=(
  rosrun scout_local_planner fixed_global_path_runner.py
  --mode replay
  --path-file "${PATH_FILE}"
  --output-topic "${REF_TOPIC}"
  --base-frame "${BASE_FRAME}"
  --start-pos-tol "${START_POS_TOL}"
  --start-yaw-tol "${START_YAW_TOL}"
  --start-hold-sec "${START_HOLD_SEC}"
  --publish-rate 10
  --publish-count 0
)

echo "================ G5 FixedProfile minimal trial ================"
echo "  label   = ${RUN_LABEL}"
echo "  path    = ${PATH_FILE}"
echo "  profile = ${PROFILE_FILE}"
echo "  output  = ${bag_path}"
echo "  record  = ${RECORD_SEC}s; no image stream"
echo "==============================================================="
if truthy "${VALIDATE_ONLY}"; then
  printf '[FixedProfile] planner: '; printf '%q ' "${planner_cmd[@]}"; printf '\n'
  printf '[FixedProfile] path:    '; printf '%q ' "${path_cmd[@]}"; printf '\n'
  echo "[FixedProfile] validate-only PASS"
  exit 0
fi

[[ "${ARM_MOTION}" == "YES" ]] || fail "real motion disarmed; set ARM_MOTION=YES after clearing the area"
timeout 5s rostopic list >/dev/null 2>&1 || fail "ROS master is not reachable"
timeout 5s rostopic echo -n 1 /odom >/dev/null 2>&1 || fail "/odom is unavailable"
[[ ! -e "${bag_path}" && ! -e "${bag_path}.active" ]] || fail "output already exists: ${bag_path}"
if timeout 5s rosnode list 2>/dev/null | grep -Fxq /scout_local_planner; then
  fail "/scout_local_planner already exists"
fi
cmd_info="$(timeout 5s rostopic info "${CMD_TOPIC}" 2>/dev/null || true)"
if awk '
  /^Publishers:/ {inside=1; next}
  /^Subscribers:/ {inside=0}
  inside && /^[[:space:]]+\*/ {found=1}
  END {exit(found ? 0 : 1)}
' <<< "${cmd_info}"; then
  echo "${cmd_info}" >&2
  fail "${CMD_TOPIC} already has a publisher"
fi

mkdir -p "${RUN_OUT_DIR}"
printf '%s\n' \
  "protocol=G5_comparator_fairness_minimal_v1" \
  "condition=FixedProfile" \
  "run_label=${RUN_LABEL}" \
  "path_file=${PATH_FILE}" \
  "path_sha256=${PATH_EXPECTED_SHA256}" \
  "profile_file=${PROFILE_FILE}" \
  "profile_sha256=${PROFILE_EXPECTED_SHA256}" \
  "profile_config=${PROFILE_CONFIG}" \
  "profile_config_sha256=${PROFILE_CONFIG_EXPECTED_SHA256}" \
  "g5_prereg_sha256=${G5_PREREG_SHA256}" \
  "runtime_profile_regeneration=false" \
  "online_liquid_feedback=false" \
  > "${meta_path}"

recorder_pid=""
planner_pid=""
path_pid=""
cleaned=false

publish_zero() {
  timeout 2s rostopic pub -1 "${CMD_TOPIC}" geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
}

stop_child() {
  local pid="$1"
  if child_running "${pid}"; then
    kill -INT "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  ${cleaned} && return 0
  cleaned=true
  stop_child "${path_pid}"
  stop_child "${planner_pid}"
  publish_zero
  stop_child "${recorder_pid}"
}
trap cleanup EXIT INT TERM

echo "[FixedProfile] starting recorder"
(
  cd "${REPO_ROOT}"
  DATE="${DATE}" STAMP="${STAMP}" VARIANT=FixedProfile \
  RUN_CLASS=G5_MINIMAL PILOT_MODE=true PILOT_METHOD=FixedProfile \
  PILOT_CONDITION=G5_comparator_fairness_minimal_v1 \
  BLOCK_SEGMENT_ID=G5_b01_seg01 ORDER_POSITION=02 \
  RUN_LABEL="${RUN_LABEL}" NAME="${NAME}" OUT_DIR="${RUN_OUT_DIR}" \
  RECORD_SEC="${RECORD_SEC}" RECORD_RGB=false RECORD_CAMERA=false \
  RECORD_CAMERA_INFO=false RECORD_CAMERA_COMPRESSED=false RECORD_DEPTH=false \
  RECORD_SCAN=false RECORD_STANDALONE_SLOSH=false RECORD_ONLINE_LIQUID=false \
  RECORD_ONLINE_LIQUID_DEBUG_IMAGES=false FORBID_IMAGE_STREAMS=true \
  RECORD_ALL_EXISTING_TOPICS=false RECORD_ROSOUT=false \
  PATH_SOURCE_MODE=replay PATH_FILE="${PATH_FILE}" \
  PATH_EXPECTED_SHA256="${PATH_EXPECTED_SHA256}" PATH_ACTUAL_SHA256="${PATH_EXPECTED_SHA256}" \
  REQUIRE_PATH_HASH=true START_POS_TOL="${START_POS_TOL}" \
  START_YAW_TOL="${START_YAW_TOL}" START_HOLD_SEC="${START_HOLD_SEC}" \
  OPERATOR_NOTE="G5 FixedProfile profile_sha256=${PROFILE_EXPECTED_SHA256}" \
  bash "${RECORDER}"
) > "${recorder_log}" 2>&1 &
recorder_pid=$!
for _ in {1..100}; do
  [[ -e "${bag_path}.active" ]] && break
  child_running "${recorder_pid}" || fail "recorder exited before creating .bag.active"
  sleep 0.1
done
[[ -e "${bag_path}.active" ]] || fail "recorder did not create .bag.active"

echo "[FixedProfile] starting frozen external-profile tracker"
"${planner_cmd[@]}" > "${planner_log}" 2>&1 &
planner_pid=$!
sleep 2
child_running "${planner_pid}" || fail "FixedProfile tracker exited during startup"
mode="$(timeout 5s rostopic echo -n 1 /diagnostics/external_profile_mode 2>/dev/null | awk '/^data:/ {print $2; exit}' | tr -d '\"')"
[[ "${mode}" == "custom_csv" ]] || fail "tracker external_profile_mode=${mode:-missing}, expected custom_csv"

echo "[FixedProfile] releasing frozen path after start-pose gate"
"${path_cmd[@]}" > "${path_log}" 2>&1 &
path_pid=$!
if ! timeout "${START_GATE_TIMEOUT_SEC}s" rostopic echo -n 1 "${REF_TOPIC}" >/dev/null 2>&1; then
  tail -60 "${path_log}" >&2 || true
  fail "path replay/start gate timed out"
fi
child_running "${path_pid}" || fail "path runner exited after publishing"

echo "[FixedProfile] recording until the bounded ${RECORD_SEC}s recorder closes"
set +e
wait "${recorder_pid}"
recorder_code=$?
set -e
recorder_pid=""
(( recorder_code == 0 )) || { tail -80 "${recorder_log}" >&2 || true; fail "recorder failed (${recorder_code})"; }
child_running "${planner_pid}" || { tail -80 "${planner_log}" >&2 || true; fail "tracker exited before recorder closure"; }
cleanup
trap - EXIT INT TERM

[[ -s "${bag_path}" && ! -e "${bag_path}.active" ]] || fail "completed bag is missing or still active"
echo "[FixedProfile] bag complete: ${bag_path}"
