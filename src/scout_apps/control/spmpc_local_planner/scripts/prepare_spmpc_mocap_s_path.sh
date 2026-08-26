#!/usr/bin/env bash
# One-shot, no-motion helper: wait for a clicked goal, generate a compact S
# path, validate its geometry, and freeze its SHA-256 for later replay.

set -euo pipefail

SCRIPT_NAME="prepare_spmpc_mocap_s_path"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
PATH_VALIDATOR="${SCRIPT_DIR}/analysis/validate_mocap_s_path.py"

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

DATE="${DATE:-$(date +%Y%m%d)}"
PATH_ROOT="${PATH_ROOT:-/home/geist/fixed_paths/real/${DATE}_spmpc_mocap_execution_chain}"
PATH_FILE="${PATH_FILE:-${PATH_ROOT}/mocap_compact_s_path.json}"
PATH_REPORT="${PATH_REPORT:-${PATH_FILE%.json}_validation.json}"
PATH_SHA256_FILE="${PATH_SHA256_FILE:-${PATH_FILE}.sha256}"
FREEZE_META="${FREEZE_META:-${PATH_FILE%.json}_freeze.env}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ALLOW_PATH_OVERWRITE="${ALLOW_PATH_OVERWRITE:-false}"
REQUIRE_MOCAP="${REQUIRE_MOCAP:-true}"

GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
BASE_FRAME="${BASE_FRAME:-base_link}"
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
PATH_SPACING="${PATH_SPACING:-0.03}"
PATH_AMPLITUDE_RATIO="${PATH_AMPLITUDE_RATIO:-0.18}"
PATH_MIN_AMPLITUDE="${PATH_MIN_AMPLITUDE:-0.15}"
PATH_MAX_AMPLITUDE="${PATH_MAX_AMPLITUDE:-0.60}"
PATH_SIDE="${PATH_SIDE:-left}"
PATH_SMOOTH_ITERATIONS="${PATH_SMOOTH_ITERATIONS:-3}"
MAX_SPAN_X_M="${MAX_SPAN_X_M:-0}"
MAX_SPAN_Y_M="${MAX_SPAN_Y_M:-0}"

[[ -s "${PATH_VALIDATOR}" ]] || fail "missing path validator: ${PATH_VALIDATOR}"
[[ "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "unsafe MOCAP_TRACKER"
case "${PATH_SIDE}" in left|right) ;; *) fail "PATH_SIDE must be left|right" ;; esac

if [[ -e "${PATH_FILE}" ]] && ! truthy "${ALLOW_PATH_OVERWRITE}"; then
  fail "path already exists: ${PATH_FILE}; set ALLOW_PATH_OVERWRITE=true only for a new exploration asset"
fi

generator_cmd=(
  rosrun scout_local_planner template_fixed_path_generator.py
  --template s_curve
  --goal-topic "${GOAL_TOPIC}"
  --output-topic "${REF_TOPIC}"
  --path-file "${PATH_FILE}"
  --base-frame "${BASE_FRAME}"
  --start-heading current
  --spacing "${PATH_SPACING}"
  --amplitude-ratio "${PATH_AMPLITUDE_RATIO}"
  --min-amplitude "${PATH_MIN_AMPLITUDE}"
  --max-amplitude "${PATH_MAX_AMPLITUDE}"
  --side "${PATH_SIDE}"
  --smooth-iterations "${PATH_SMOOTH_ITERATIONS}"
  --publish-count 10
)

if truthy "${VALIDATE_ONLY}"; then
  echo "[${SCRIPT_NAME}] validate-only PASS"
  printf '  command: '
  printf '%q ' "${generator_cmd[@]}"
  printf '\n  output: %s\n' "${PATH_FILE}"
  exit 0
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

if truthy "${REQUIRE_MOCAP}"; then
  raw_pose="/vrpn_client_node/${MOCAP_TRACKER}/pose"
  timeout 5s rostopic echo -n 1 "${raw_pose}" >/dev/null 2>&1 || \
    fail "no raw mocap pose on ${raw_pose}"
  mocap_status="$(timeout 5s rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
  grep -q "OK.*tracker=${MOCAP_TRACKER}" <<< "${mocap_status}" || \
    fail "/mocap/status is not OK for ${MOCAP_TRACKER}"
fi

mkdir -p "$(dirname "${PATH_FILE}")"
echo "[${SCRIPT_NAME}] click one safe goal on ${GOAL_TOPIC}; this helper never publishes /cmd_vel"
"${generator_cmd[@]}"

validator_cmd=(
  python3 "${PATH_VALIDATOR}" "${PATH_FILE}"
  --report "${PATH_REPORT}"
  --max-span-x-m "${MAX_SPAN_X_M}"
  --max-span-y-m "${MAX_SPAN_Y_M}"
)
"${validator_cmd[@]}"

path_sha256="$(sha256sum "${PATH_FILE}" | awk '{print $1}')"
printf '%s  %s\n' "${path_sha256}" "${PATH_FILE}" > "${PATH_SHA256_FILE}"
{
  printf 'protocol_id=%s\n' 'SMPCC_mocap_execution_chain_v1'
  printf 'path_file=%s\n' "${PATH_FILE}"
  printf 'path_sha256=%s\n' "${path_sha256}"
  printf 'path_report=%s\n' "${PATH_REPORT}"
  printf 'goal_topic=%s\n' "${GOAL_TOPIC}"
  printf 'reference_topic=%s\n' "${REF_TOPIC}"
  printf 'mocap_tracker=%s\n' "${MOCAP_TRACKER}"
  printf 'generated_at=%s\n' "$(date --iso-8601=seconds)"
} > "${FREEZE_META}"

echo "[${SCRIPT_NAME}] path frozen"
echo "  PATH_FILE=${PATH_FILE}"
echo "  PATH_EXPECTED_SHA256=${path_sha256}"
echo "  validation=${PATH_REPORT}"
echo "  freeze_meta=${FREEZE_META}"
