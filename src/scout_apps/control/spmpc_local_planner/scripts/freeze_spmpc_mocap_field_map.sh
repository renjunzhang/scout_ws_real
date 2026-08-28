#!/usr/bin/env bash
# Finish one Cartographer mapping trajectory and freeze a uniquely named map.
# No service is called and no file is written unless ARM_MAP_FREEZE=YES.

set -euo pipefail

SCRIPT_NAME="freeze_spmpc_mocap_field_map"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
MAP_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_field_map.py"

fail() {
  echo "[${SCRIPT_NAME}][ERR] $*" >&2
  exit 2
}

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

wait_for_topic() {
  local topic="$1"
  local label="$2"
  timeout "${TOPIC_TIMEOUT_SEC}s" rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1 || \
    fail "no ${label} on ${topic} within ${TOPIC_TIMEOUT_SEC}s"
}

service_status_code() {
  awk '/^[[:space:]]*code:/ {gsub(/[^0-9-]/, "", $2); print $2; exit}' <<< "$1"
}

write_text_once() {
  local output="$1"
  local content="$2"
  (
    set -o noclobber
    printf '%s\n' "${content}" > "${output}"
  ) || fail "refused to overwrite evidence file: ${output}"
}

MAP_ID="${MAP_ID:-}"
MAP_OUTPUT_ROOT="${MAP_OUTPUT_ROOT:-${HOME}/scout_maps/real/$(date +%Y%m%d)_mocap_exec}"
MAP_RESOLUTION="${MAP_RESOLUTION:-0.02}"
TRAJECTORY_ID="${TRAJECTORY_ID:-0}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ARM_MAP_FREEZE="${ARM_MAP_FREEZE:-NO}"
REQUIRE_MOCAP="${REQUIRE_MOCAP:-true}"
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan_front}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
TOPIC_TIMEOUT_SEC="${TOPIC_TIMEOUT_SEC:-5}"
FINISH_TRAJECTORY_SERVICE="${FINISH_TRAJECTORY_SERVICE:-/finish_trajectory}"
WRITE_STATE_SERVICE="${WRITE_STATE_SERVICE:-/write_state}"
CARTOGRAPHER_SETUP="${CARTOGRAPHER_SETUP:-${REPO_ROOT}/src/scout_apps/sensors/cartographer_ws/install_isolated/setup.bash}"

[[ -s "${MAP_VALIDATOR}" ]] || fail "missing map validator: ${MAP_VALIDATOR}"
[[ -n "${MAP_ID}" ]] || fail "MAP_ID is required (example: map_carto_20260828_mocap_exec_v1)"
[[ "${MAP_ID}" =~ ^map_carto_[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || \
  fail "MAP_ID must start with map_carto_ and contain only letters, digits, '.', '_' or '-'"
[[ "${MAP_ID}" != "map_carto_20260629_R0" ]] || \
  fail "MAP_ID is reserved by the large-field G3R3 map; choose a unique small-field ID"
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
[[ "${MAP_OUTPUT_ROOT}" =~ ^[/A-Za-z0-9_.-]+$ ]] || \
  fail "MAP_OUTPUT_ROOT must be an absolute path without spaces or shell metacharacters"
[[ "${MAP_OUTPUT_ROOT}" == /* ]] || fail "MAP_OUTPUT_ROOT must be absolute"
[[ "${MAP_RESOLUTION}" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail "MAP_RESOLUTION must be positive"
awk -v value="${MAP_RESOLUTION}" 'BEGIN {exit !(value > 0)}' || fail "MAP_RESOLUTION must be > 0"
[[ "${TRAJECTORY_ID}" =~ ^[0-9]+$ ]] || fail "TRAJECTORY_ID must be a non-negative integer"
[[ "${TOPIC_TIMEOUT_SEC}" =~ ^[0-9]+$ ]] || fail "TOPIC_TIMEOUT_SEC must be a positive integer"
(( TOPIC_TIMEOUT_SEC > 0 )) || fail "TOPIC_TIMEOUT_SEC must be > 0"

MAP_OUTPUT_ROOT="$(readlink -m "${MAP_OUTPUT_ROOT}")"
MAP_STEM="${MAP_OUTPUT_ROOT}/${MAP_ID}"
PBSTREAM_FILE="${MAP_STEM}.pbstream"
PGM_FILE="${MAP_STEM}.pgm"
YAML_FILE="${MAP_STEM}.yaml"
SHA256_MANIFEST="${MAP_STEM}.sha256"
VALIDATION_REPORT="${MAP_STEM}_map_validation.json"
FREEZE_META="${MAP_STEM}_freeze.env"
FINISH_RESPONSE="${MAP_STEM}_finish_trajectory_response.yaml"
WRITE_RESPONSE="${MAP_STEM}_write_state_response.yaml"
CONVERTER_LOG="${MAP_STEM}_pbstream_to_ros_map.log"

known_map="${REPO_ROOT}/src/scout_apps/scout_maps/maps/${MAP_ID}.pbstream"
if [[ -e "${known_map}" && "$(readlink -f "${known_map}")" != "${PBSTREAM_FILE}" ]]; then
  fail "MAP_ID already names another repository map: ${known_map}"
fi

outputs=(
  "${PBSTREAM_FILE}"
  "${PGM_FILE}"
  "${YAML_FILE}"
  "${SHA256_MANIFEST}"
  "${VALIDATION_REPORT}"
  "${FREEZE_META}"
  "${FINISH_RESPONSE}"
  "${WRITE_RESPONSE}"
  "${CONVERTER_LOG}"
)
for output in "${outputs[@]}"; do
  [[ ! -e "${output}" ]] || fail "target already exists; choose a new MAP_ID: ${output}"
done

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS (no ROS query, service call or file write)"
  echo "  map_stem=${MAP_STEM}"
  echo "  trajectory_id=${TRAJECTORY_ID}"
  echo "  resolution=${MAP_RESOLUTION}"
  echo "  real freeze requires: ARM_MAP_FREEZE=YES"
  exit 0
fi

[[ "${ARM_MAP_FREEZE}" == "YES" ]] || \
  fail "real freeze is disarmed; inspect VALIDATE_ONLY=true output, then type ARM_MAP_FREEZE=YES"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"
[[ -f "${CARTOGRAPHER_SETUP}" ]] || fail "missing Cartographer setup: ${CARTOGRAPHER_SETUP}"
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export CATKIN_SETUP_UTIL_ARGS="${CATKIN_SETUP_UTIL_ARGS:---extend}"
# shellcheck disable=SC1090
source "${CARTOGRAPHER_SETUP}"

mkdir -p "${MAP_OUTPUT_ROOT}"
for output in "${outputs[@]}"; do
  [[ ! -e "${output}" ]] || fail "target appeared during preflight; refusing overwrite: ${output}"
done

published_topics="$(timeout 5s rostopic list -p)" || fail "could not query ROS publishers"
if grep -Fxq -- /cmd_vel <<< "${published_topics}"; then
  fail "/cmd_vel has a publisher; stop planner and teleop before freezing the map"
fi

wait_for_topic "${SCAN_TOPIC}" "front LiDAR scan"
wait_for_topic "${ODOM_TOPIC}" "Scout odometry"
wait_for_topic "${MAP_TOPIC}" "Cartographer occupancy map"

if truthy "${REQUIRE_MOCAP}"; then
  raw_mocap_topic="/vrpn_client_node/${MOCAP_TRACKER}/pose"
  wait_for_topic "${raw_mocap_topic}" "raw mocap pose"
  mocap_status="$(timeout "${TOPIC_TIMEOUT_SEC}s" rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
  grep -q "OK.*tracker=${MOCAP_TRACKER}" <<< "${mocap_status}" || \
    fail "/mocap/status is not OK for ${MOCAP_TRACKER}"
fi

services="$(timeout 5s rosservice list)" || fail "could not query ROS services"
grep -Fxq -- "${FINISH_TRAJECTORY_SERVICE}" <<< "${services}" || \
  fail "missing service: ${FINISH_TRAJECTORY_SERVICE}"
grep -Fxq -- "${WRITE_STATE_SERVICE}" <<< "${services}" || \
  fail "missing service: ${WRITE_STATE_SERVICE}"

echo "[${SCRIPT_NAME}] finishing Cartographer trajectory ${TRAJECTORY_ID}"
if finish_payload="$(rosservice call "${FINISH_TRAJECTORY_SERVICE}" "{trajectory_id: ${TRAJECTORY_ID}}" 2>&1)"; then
  finish_rc=0
else
  finish_rc=$?
fi
write_text_once "${FINISH_RESPONSE}" "${finish_payload}"
(( finish_rc == 0 )) || fail "finish_trajectory command failed; evidence: ${FINISH_RESPONSE}"
finish_code="$(service_status_code "${finish_payload}")"
[[ "${finish_code}" == "0" ]] || \
  fail "finish_trajectory returned status.code=${finish_code:-missing}; evidence: ${FINISH_RESPONSE}"

echo "[${SCRIPT_NAME}] writing frozen pbstream"
if write_payload="$(rosservice call "${WRITE_STATE_SERVICE}" \
    "{filename: '${PBSTREAM_FILE}', include_unfinished_submaps: false}" 2>&1)"; then
  write_rc=0
else
  write_rc=$?
fi
write_text_once "${WRITE_RESPONSE}" "${write_payload}"
(( write_rc == 0 )) || fail "write_state command failed; evidence: ${WRITE_RESPONSE}"
write_code="$(service_status_code "${write_payload}")"
[[ "${write_code}" == "0" ]] || \
  fail "write_state returned status.code=${write_code:-missing}; evidence: ${WRITE_RESPONSE}"
[[ -s "${PBSTREAM_FILE}" ]] || fail "write_state reported success but pbstream is missing or empty"

converter="$(command -v cartographer_pbstream_to_ros_map || true)"
[[ -n "${converter}" ]] || fail "cartographer_pbstream_to_ros_map is not on PATH"
echo "[${SCRIPT_NAME}] exporting ${PGM_FILE} and ${YAML_FILE}"
if ! "${converter}" \
    --map_filestem="${MAP_STEM}" \
    --pbstream_filename="${PBSTREAM_FILE}" \
    --resolution="${MAP_RESOLUTION}" 2>&1 | tee "${CONVERTER_LOG}"; then
  fail "pbstream conversion failed; partial outputs and log were preserved"
fi

python3 "${MAP_VALIDATOR}" "${MAP_STEM}" \
  --expected-resolution "${MAP_RESOLUTION}" \
  --report "${VALIDATION_REPORT}" \
  --sha256-manifest "${SHA256_MANIFEST}"

pbstream_sha256="$(sha256sum "${PBSTREAM_FILE}" | awk '{print $1}')"
yaml_sha256="$(sha256sum "${YAML_FILE}" | awk '{print $1}')"
pgm_sha256="$(sha256sum "${PGM_FILE}" | awk '{print $1}')"
git_commit="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain --untracked-files=normal)" ]]; then
  git_dirty="true"
else
  git_dirty="false"
fi
mapping_launch_sha256="$(sha256sum "${REPO_ROOT}/src/scout_apps/sensors/nanoscan3_mapping/launch/scout_nanoscan3_cartographer.launch" | awk '{print $1}')"
mapping_config_sha256="$(sha256sum "${REPO_ROOT}/src/scout_apps/sensors/nanoscan3_mapping/config/scout_2d.lua" | awk '{print $1}')"
localization_launch_sha256="$(sha256sum "${REPO_ROOT}/src/scout_apps/sensors/nanoscan3_localization/launch/scout_nanoscan3_cartographer_localization.launch" | awk '{print $1}')"
localization_config_sha256="$(sha256sum "${REPO_ROOT}/src/scout_apps/sensors/nanoscan3_localization/config/scout_2d_localization.lua" | awk '{print $1}')"

(
  set -o noclobber
  {
    printf 'export MAP_PROTOCOL_ID=%q\n' 'SMPCC_mocap_field_map_v1'
    printf 'export MAP_ID=%q\n' "${MAP_ID}"
    printf 'export MAP_STEM=%q\n' "${MAP_STEM}"
    printf 'export MAP_RESOLUTION=%q\n' "${MAP_RESOLUTION}"
    printf 'export MAP_PBSTREAM_SHA256=%q\n' "${pbstream_sha256}"
    printf 'export MAP_YAML_SHA256=%q\n' "${yaml_sha256}"
    printf 'export MAP_PGM_SHA256=%q\n' "${pgm_sha256}"
    printf 'export LOCALIZATION_MAP_FILE=%q\n' "${PBSTREAM_FILE}"
    printf 'export LOCALIZATION_MAP_EXPECTED_SHA256=%q\n' "${pbstream_sha256}"
    printf 'export REQUIRE_LOCALIZATION_MAP_HASH=%q\n' 'true'
    printf 'export LOCALIZATION_OCCUPANCY_GRID_RESOLUTION=%q\n' "${MAP_RESOLUTION}"
    printf 'export MAP_VALIDATION_REPORT=%q\n' "${VALIDATION_REPORT}"
    printf 'export MAP_SHA256_MANIFEST=%q\n' "${SHA256_MANIFEST}"
    printf 'export MAP_FREEZE_GIT_COMMIT=%q\n' "${git_commit}"
    printf 'export MAP_FREEZE_GIT_DIRTY=%q\n' "${git_dirty}"
    printf 'export MAP_MAPPING_LAUNCH_SHA256=%q\n' "${mapping_launch_sha256}"
    printf 'export MAP_MAPPING_CONFIG_SHA256=%q\n' "${mapping_config_sha256}"
    printf 'export MAP_LOCALIZATION_LAUNCH_SHA256=%q\n' "${localization_launch_sha256}"
    printf 'export MAP_LOCALIZATION_CONFIG_SHA256=%q\n' "${localization_config_sha256}"
    printf 'export MAP_FROZEN_AT=%q\n' "$(date --iso-8601=seconds)"
  } > "${FREEZE_META}"
) || fail "refused to overwrite freeze metadata: ${FREEZE_META}"

echo "[${SCRIPT_NAME}] PASS: map frozen without overwriting another field asset"
echo "  LOCALIZATION_MAP_FILE=${PBSTREAM_FILE}"
echo "  LOCALIZATION_MAP_EXPECTED_SHA256=${pbstream_sha256}"
echo "  source ${FREEZE_META}"
