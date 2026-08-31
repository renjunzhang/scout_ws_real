#!/usr/bin/env bash
# Replay a recorded motion bag through B0 + O0/I0/I1/L22 without touching the
# live ROS graph or the real /cmd_vel topic.

set -euo pipefail

SCRIPT_NAME="run_spmpc_slosh_nowcast_replay"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
VALIDATOR="${SCRIPT_DIR}/analysis/validate_slosh_nowcast_replay.py"

fail() {
  echo "[${SCRIPT_NAME}][ERR] $*" >&2
  exit 2
}

SOURCE_BAG="${SOURCE_BAG:-${1:-}}"
[[ -n "${SOURCE_BAG}" ]] || fail "usage: SOURCE_BAG=/path/input.bag bash $0"
SOURCE_BAG="$(readlink -f "${SOURCE_BAG}")"
[[ -s "${SOURCE_BAG}" ]] || fail "source bag is missing or empty: ${SOURCE_BAG}"
[[ -s "${VALIDATOR}" ]] || fail "missing validator: ${VALIDATOR}"

PRIVATE_MASTER_PORT="${PRIVATE_MASTER_PORT:-11391}"
case "${PRIVATE_MASTER_PORT}" in
  ''|*[!0-9]*) fail "PRIVATE_MASTER_PORT must be an integer" ;;
esac
(( PRIVATE_MASTER_PORT >= 1024 && PRIVATE_MASTER_PORT <= 65535 )) || fail "PRIVATE_MASTER_PORT outside [1024,65535]"

SOURCE_STEM="$(basename "${SOURCE_BAG}" .bag)"
OUT_DIR="${OUT_DIR:-$(dirname "${SOURCE_BAG}")/${SOURCE_STEM}_nowcast_replay}"
OUTPUT_BAG="${OUT_DIR}/${SOURCE_STEM}_nowcast_replay.bag"
REPORT="${OUT_DIR}/${SOURCE_STEM}_nowcast_replay_report.json"
META="${OUT_DIR}/${SOURCE_STEM}_nowcast_replay_meta.env"
ISOLATED_CMD_TOPIC="${ISOLATED_CMD_TOPIC:-/spmpc_nowcast_replay/cmd_vel}"
PATH_TOPIC="${PATH_TOPIC:-/scout/global_path_fixed}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
REPLAY_RATE="${REPLAY_RATE:-1.0}"
REPLAY_CLOCK_HZ="${REPLAY_CLOCK_HZ:-500}"
REPLAY_DURATION_SEC="${REPLAY_DURATION_SEC:-0}"

[[ "${REPLAY_RATE}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || fail "REPLAY_RATE must be numeric"
awk -v value="${REPLAY_RATE}" 'BEGIN {exit !(value >= 0.1 && value <= 2.0)}' || \
  fail "REPLAY_RATE must stay in [0.1,2.0] so callbacks remain meaningful"
case "${REPLAY_CLOCK_HZ}" in
  ''|*[!0-9]*) fail "REPLAY_CLOCK_HZ must be an integer" ;;
esac
(( REPLAY_CLOCK_HZ >= 200 && REPLAY_CLOCK_HZ <= 1000 )) || \
  fail "REPLAY_CLOCK_HZ must stay in [200,1000] to honor the 5 ms future-skew gate"
[[ "${REPLAY_DURATION_SEC}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || \
  fail "REPLAY_DURATION_SEC must be numeric"
awk -v value="${REPLAY_DURATION_SEC}" 'BEGIN {exit !(value >= 0.0)}' || \
  fail "REPLAY_DURATION_SEC must be >= 0 (0 means the complete bag)"

[[ "${ISOLATED_CMD_TOPIC}" == /spmpc_nowcast_replay/* ]] || fail "isolated command topic must stay under /spmpc_nowcast_replay/"
[[ "${ISOLATED_CMD_TOPIC}" != "/cmd_vel" ]] || fail "real /cmd_vel is forbidden"
[[ ! -e "${OUT_DIR}" ]] || fail "output directory already exists: ${OUT_DIR}"

# Read-only input audit before starting any ROS process.
python3 - "${SOURCE_BAG}" "${PATH_TOPIC}" "${IMU_TOPIC}" <<'PY'
import sys
import rosbag

bag_path, path_topic, imu_topic = sys.argv[1:]
required = {"/odom", "/tf", "/map", path_topic, imu_topic}
with rosbag.Bag(bag_path, "r") as bag:
    topics = bag.get_type_and_topic_info().topics
missing = sorted(required - set(topics))
if missing:
    raise SystemExit("source bag missing replay inputs: {}".format(",".join(missing)))
PY

mkdir -p "${OUT_DIR}"
TMP_DIR="$(mktemp -d "${OUT_DIR}/runtime.XXXXXX")"
ROS_MASTER_URI="http://127.0.0.1:${PRIVATE_MASTER_PORT}"
export ROS_MASTER_URI
unset ROS_HOSTNAME || true
export ROS_IP=127.0.0.1

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

roscore_log="${OUT_DIR}/roscore.log"
planner_log="${OUT_DIR}/planner.log"
player_log="${OUT_DIR}/player.log"
recorder_log="${OUT_DIR}/recorder.log"
roscore_pid=""
planner_pid=""
player_pid=""
recorder_pid=""
cleaned=false

signal_and_wait() {
  local pid="$1"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  ${cleaned} && return 0
  cleaned=true
  signal_and_wait "${player_pid}"
  signal_and_wait "${planner_pid}"
  signal_and_wait "${recorder_pid}"
  signal_and_wait "${roscore_pid}"
  rmdir "${TMP_DIR}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Refuse to join or disturb an existing master on the chosen private port.
if timeout 1s rosparam list >/dev/null 2>&1; then
  fail "a ROS master already exists at ${ROS_MASTER_URI}; choose another PRIVATE_MASTER_PORT"
fi

roscore -p "${PRIVATE_MASTER_PORT}" > "${roscore_log}" 2>&1 &
roscore_pid=$!
for _ in {1..50}; do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam list >/dev/null 2>&1 || fail "private roscore did not start"
rosparam set /use_sim_time true

source_sha="$(sha256sum "${SOURCE_BAG}" | awk '{print $1}')"
{
  echo "protocol=SMPCC_slosh_state_nowcast_isolated_replay_v1"
  echo "source_bag=${SOURCE_BAG}"
  echo "source_bag_sha256=${source_sha}"
  echo "output_bag=${OUTPUT_BAG}"
  echo "ros_master_uri=${ROS_MASTER_URI}"
  echo "isolated_cmd_topic=${ISOLATED_CMD_TOPIC}"
  echo "replay_rate=${REPLAY_RATE}"
  echo "replay_clock_hz=${REPLAY_CLOCK_HZ}"
  echo "replay_duration_sec=${REPLAY_DURATION_SEC}"
  echo "real_cmd_topic_forbidden=true"
  echo "planner_variant=B0"
  echo "delay_phase_mode=fixed_robot_only"
  echo "liquid_methods=O0,I0,I1,L22"
  echo "liquid_applied_to_solver=false"
  echo "git_revision=$(git -C "${REPO_ROOT}" rev-parse HEAD)"
  echo "created_at=$(date --iso-8601=seconds)"
} > "${META}"

rosbag record --buffsize=1024 -O "${OUTPUT_BAG}" \
  /clock "${ISOLATED_CMD_TOPIC}" \
  /spmpc/status /spmpc/debug/effective_config \
  /spmpc/debug/control_cycle_audit \
  /spmpc/debug/slosh_observer_imu /spmpc/debug/slosh_observer_odom \
  /spmpc/debug/slosh_observer_selection \
  /spmpc/debug/slosh_estimator_comparison \
  > "${recorder_log}" 2>&1 &
recorder_pid=$!
sleep 1
kill -0 "${recorder_pid}" 2>/dev/null || fail "replay recorder exited during startup"

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:=B0 solver_backend:=continuous_mpcc_acados \
  reference_path_topic:="${PATH_TOPIC}" cmd_vel_topic:="${ISOLATED_CMD_TOPIC}" \
  costmap_topic:=/map reference_target_frame:=map \
  delay_phase_mode:=fixed_robot_only delay_phase_linear_delay_sec:=0.15 \
  delay_phase_angular_delay_sec:=0.22 imu_topic:="${IMU_TOPIC}" \
  imu_shadow_enable:=true imu_subscriber_queue_size:=10 \
  observer_source:=processed_imu observer_fallback_policy:=fail_closed \
  observer_latch_fallback:=false liquid_nowcast_enable:=true \
  liquid_nowcast_publish_comparison:=true liquid_nowcast_max_prediction_sec:=0.050 \
  liquid_nowcast_max_excitation_age_sec:=0.060 liquid_nowcast_max_future_skew_sec:=0.005 \
  liquid_nowcast_max_state_excitation_skew_sec:=0.001 \
  liquid_nowcast_max_integration_step_sec:=0.020 \
  state_timing_require_common_epoch:=true speed_safety_enable:=true \
  v_safe_max:=0.15 speed_safety_tolerance:=0.0001 v_ref:=0.10 w_slosh:=0.0 \
  execution_contract_fail_closed_on_post_limit_change:=true \
  execution_contract_max_post_limit_delta_v:=0.0001 \
  execution_contract_max_post_limit_delta_omega:=0.0001 \
  > "${planner_log}" 2>&1 &
planner_pid=$!
sleep 1
kill -0 "${planner_pid}" 2>/dev/null || { tail -80 "${planner_log}" >&2 || true; fail "replay planner exited during startup"; }

echo "[${SCRIPT_NAME}] private master=${ROS_MASTER_URI}"
echo "[${SCRIPT_NAME}] real /cmd_vel is disconnected; output=${ISOLATED_CMD_TOPIC}"
echo "[${SCRIPT_NAME}] replaying ${SOURCE_BAG}"
player_cmd=(
  rosbag play --clock --hz "${REPLAY_CLOCK_HZ}" --rate "${REPLAY_RATE}"
)
if awk -v value="${REPLAY_DURATION_SEC}" 'BEGIN {exit !(value > 0.0)}'; then
  player_cmd+=(--duration "${REPLAY_DURATION_SEC}")
fi
player_cmd+=(
  "${SOURCE_BAG}" --topics
  /odom /tf /tf_static /map "${PATH_TOPIC}" "${IMU_TOPIC}"
)
"${player_cmd[@]}" > "${player_log}" 2>&1 &
player_pid=$!
set +e
wait "${player_pid}"
player_code=$?
set -e
player_pid=""
(( player_code == 0 )) || { tail -80 "${player_log}" >&2 || true; fail "rosbag play failed with code ${player_code}"; }

# Simulated time is now frozen. Stop producers and finalize the output bag.
signal_and_wait "${planner_pid}"
planner_pid=""
signal_and_wait "${recorder_pid}"
recorder_pid=""
[[ -s "${OUTPUT_BAG}" ]] || fail "replay output bag was not finalized"
[[ "$(sha256sum "${SOURCE_BAG}" | awk '{print $1}')" == "${source_sha}" ]] || fail "source bag changed during replay"

python3 "${VALIDATOR}" "${OUTPUT_BAG}" --report "${REPORT}" \
  --isolated-cmd-topic "${ISOLATED_CMD_TOPIC}"

echo "[${SCRIPT_NAME}] PASS: no live master and no real /cmd_vel were used"
echo "[${SCRIPT_NAME}] output=${OUTPUT_BAG}"
echo "[${SCRIPT_NAME}] report=${REPORT}"
