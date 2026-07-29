#!/usr/bin/env bash
# Record a real-robot in-place-spin calibration bag.
#
# This script is recording-only. It never publishes /cmd_vel and never starts
# or stops the robot, IMU, VRPN client, planner, or localization stack.

set -euo pipefail

SCRIPT_NAME="record_mocap_imu_spin"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

resolve_ws_root() {
    local candidate=""
    if [[ -n "${SCOUT_WS:-}" ]]; then
        readlink -f -- "${SCOUT_WS}"
        return
    fi
    candidate="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null || true)"
    if [[ -n "${candidate}" ]]; then
        printf '%s\n' "${candidate}"
    fi
}

WS_ROOT="$(resolve_ws_root)"
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
IMU_NODE="${IMU_NODE:-/imu}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
SEGMENT_TOPIC="${SEGMENT_TOPIC:-/mocap_imu_calib/segment}"
STATUS_TOPIC="${STATUS_TOPIC:-/mocap_imu_calib/status}"
PRECHECK_TIMEOUT="${PRECHECK_TIMEOUT:-5}"
RECORD_SEC="${RECORD_SEC:-0}"
ROSBAG_BUFFER_MB="${ROSBAG_BUFFER_MB:-1024}"
DATE_TAG="${DATE_TAG:-$(date +%Y%m%d)}"
TIME_TAG="${TIME_TAG:-$(date +%H%M%S)}"
RUN_LABEL="${RUN_LABEL:-mocap_imu_spin}"
OUT_DIR="${OUT_DIR:-/home/geist/slosh_bags/real/${DATE_TAG}_mocap_imu_spin}"
NAME="${NAME:-${RUN_LABEL}_${TIME_TAG}}"

if [[ ! "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "[${SCRIPT_NAME}] ERROR: unsafe MOCAP_TRACKER=${MOCAP_TRACKER}" >&2
    exit 2
fi
if [[ ! "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "[${SCRIPT_NAME}] ERROR: RUN_LABEL may contain only letters, digits, '.', '_' and '-'." >&2
    exit 2
fi
if [[ ! "${NAME}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "[${SCRIPT_NAME}] ERROR: NAME may contain only letters, digits, '.', '_' and '-'." >&2
    exit 2
fi
if [[ "${NAME}" == *.bag || "${NAME}" == *.active ]]; then
    echo "[${SCRIPT_NAME}] ERROR: NAME must not end in .bag or .active; rosbag adds the suffix." >&2
    exit 2
fi
if [[ ! "${RECORD_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "[${SCRIPT_NAME}] ERROR: RECORD_SEC must be a non-negative number." >&2
    exit 2
fi

if ! command -v rostopic >/dev/null 2>&1 && [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi
if [[ -n "${WS_ROOT}" && -f "${WS_ROOT}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_ROOT}/devel/setup.bash"
elif [[ -n "${SCOUT_WS:-}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: ${WS_ROOT}/devel/setup.bash not found for explicit SCOUT_WS." >&2
    exit 1
fi

if ! rostopic list >/dev/null 2>&1; then
    echo "[${SCRIPT_NAME}] ERROR: ROS master is not reachable." >&2
    exit 1
fi
if ! command -v setsid >/dev/null 2>&1; then
    echo "[${SCRIPT_NAME}] ERROR: setsid is required for deterministic rosbag shutdown." >&2
    exit 1
fi

RAW_MOCAP_PREFIX="/vrpn_client_node/${MOCAP_TRACKER}"
RAW_POSE_TOPIC="${RAW_MOCAP_PREFIX}/pose"

wait_for_message() {
    local topic="$1"
    local label="$2"
    echo "[${SCRIPT_NAME}] Checking ${label}: ${topic}"
    if ! timeout "${PRECHECK_TIMEOUT}" rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1; then
        echo "[${SCRIPT_NAME}] ERROR: no ${label} message within ${PRECHECK_TIMEOUT}s: ${topic}" >&2
        exit 1
    fi
}

wait_for_message "${IMU_TOPIC}" "IMU"
wait_for_message "${RAW_POSE_TOPIC}" "raw mocap pose"

mapfile -t LIVE_TOPICS < <(rostopic list)

topic_is_live() {
    local wanted="$1"
    local topic
    for topic in "${LIVE_TOPICS[@]}"; do
        if [[ "${topic}" == "${wanted}" ]]; then
            return 0
        fi
    done
    return 1
}

candidate_topics=(
    "${IMU_TOPIC}"
    /imu/data_raw
    /container_imu
    "${RAW_MOCAP_PREFIX}/pose"
    "${RAW_MOCAP_PREFIX}/twist"
    "${RAW_MOCAP_PREFIX}/accel"
    /mocap/scout_pose
    /mocap/scout_odom
    /mocap/status
    /cmd_vel_drive
    "${ODOM_TOPIC}"
    /scout/odom
    /scout/cmd_vel
    /scout_status
    /joint_states
    /battery_state
    /diagnostics
    /wit/mag
    /scan_front
    /scan
    /tf
    /tf_static
)

record_topics=()
missing_topics=()

append_record_topic_once() {
    local wanted="$1"
    local existing
    for existing in "${record_topics[@]}"; do
        if [[ "${existing}" == "${wanted}" ]]; then
            return
        fi
    done
    record_topics+=("${wanted}")
}

append_missing_topic_once() {
    local wanted="$1"
    local existing
    for existing in "${missing_topics[@]}"; do
        if [[ "${existing}" == "${wanted}" ]]; then
            return
        fi
    done
    missing_topics+=("${wanted}")
}

for topic in "${candidate_topics[@]}"; do
    if topic_is_live "${topic}"; then
        append_record_topic_once "${topic}"
    else
        append_missing_topic_once "${topic}"
    fi
done

# The persistent motion publisher starts after this recorder.  Subscribe to
# its command and event topics unconditionally so there is no startup race.
append_record_topic_once "${IMU_TOPIC}"
append_record_topic_once "${CMD_TOPIC}"
append_record_topic_once "${ODOM_TOPIC}"
append_record_topic_once "${SEGMENT_TOPIC}"
append_record_topic_once "${STATUS_TOPIC}"

mkdir -p "${OUT_DIR}"
BAG_PREFIX="${OUT_DIR}/${NAME}"
BAG_PATH="${BAG_PREFIX}.bag"
INFO_PATH="${OUT_DIR}/${NAME}_info.txt"
TOPICS_PATH="${OUT_DIR}/${NAME}_topics.txt"
MISSING_PATH="${OUT_DIR}/${NAME}_missing_topics.txt"

for artifact in \
    "${BAG_PATH}" \
    "${BAG_PATH}.active" \
    "${INFO_PATH}" \
    "${TOPICS_PATH}" \
    "${MISSING_PATH}" \
    "${OUT_DIR}/${NAME}_bag_info.txt"; do
    if [[ -e "${artifact}" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: refusing to overwrite existing artifact: ${artifact}" >&2
        exit 1
    fi
done

printf '%s\n' "${record_topics[@]}" >"${TOPICS_PATH}"
printf '%s\n' "${missing_topics[@]}" >"${MISSING_PATH}"

VRPN_SERVER="$(rosparam get /vrpn_client_node/server 2>/dev/null || printf 'unknown')"
VRPN_PORT="$(rosparam get /vrpn_client_node/port 2>/dev/null || printf 'unknown')"
MOCAP_STATUS="$(timeout "${PRECHECK_TIMEOUT}" rostopic echo -n 1 /mocap/status 2>/dev/null || true)"
IMU_TYPE="$(rostopic type "${IMU_TOPIC}" 2>/dev/null || printf 'unknown')"
IMU_PORT="$(rosparam get "${IMU_NODE}/port" 2>/dev/null || printf 'unknown')"
IMU_BAUD="$(rosparam get "${IMU_NODE}/baud" 2>/dev/null || printf 'unknown')"
IMU_FRAME_ID="$(rosparam get "${IMU_NODE}/frame_id" 2>/dev/null || printf 'unknown')"
GIT_HASH="unknown"
GIT_DIRTY="unknown"
if [[ -n "${WS_ROOT}" ]] && git -C "${WS_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    GIT_HASH="$(git -C "${WS_ROOT}" rev-parse --short HEAD 2>/dev/null || printf 'unknown')"
    if [[ -n "$(git -C "${WS_ROOT}" status --porcelain --untracked-files=normal 2>/dev/null)" ]]; then
        GIT_DIRTY="true"
    else
        GIT_DIRTY="false"
    fi
fi

{
    echo "script=${SCRIPT_NAME}"
    echo "created_at=$(date --iso-8601=seconds)"
    echo "workspace=${WS_ROOT:-unknown}"
    echo "run_label=${RUN_LABEL}"
    echo "bag_path=${BAG_PATH}"
    echo "record_sec=${RECORD_SEC}"
    echo "imu_topic=${IMU_TOPIC}"
    echo "imu_topic_type=${IMU_TYPE}"
    echo "imu_node=${IMU_NODE}"
    echo "imu_port=${IMU_PORT}"
    echo "imu_baud=${IMU_BAUD}"
    echo "imu_frame_id=${IMU_FRAME_ID}"
    echo "cmd_topic=${CMD_TOPIC}"
    echo "odom_topic=${ODOM_TOPIC}"
    echo "segment_topic=${SEGMENT_TOPIC}"
    echo "status_topic=${STATUS_TOPIC}"
    echo "mocap_tracker=${MOCAP_TRACKER}"
    echo "raw_pose_topic=${RAW_POSE_TOPIC}"
    echo "vrpn_server=${VRPN_SERVER}"
    echo "vrpn_port=${VRPN_PORT}"
    echo "topic_count=${#record_topics[@]}"
    echo "git_hash=${GIT_HASH}"
    echo "git_dirty=${GIT_DIRTY}"
    echo "mocap_status_begin=${MOCAP_STATUS//$'\n'/ }"
    echo "protocol=recording_only; IMU values are direct driver-decoded samples; offline filtering is allowed"
} >"${INFO_PATH}"

echo
echo "[${SCRIPT_NAME}] Ready. This script DOES NOT move the robot."
echo "[${SCRIPT_NAME}] Formal run: >=60s static -> planar motion -> >=60s static -> Ctrl+C."
echo "[${SCRIPT_NAME}] Prefer separate RUN_LABEL values for CCW and CW runs."
echo "[${SCRIPT_NAME}] Bag: ${BAG_PATH}"
echo "[${SCRIPT_NAME}] Topics (${#record_topics[@]}):"
printf '  %s\n' "${record_topics[@]}"
if (( ${#missing_topics[@]} > 0 )); then
    echo "[${SCRIPT_NAME}] Optional topics not present (not recorded):"
    printf '  %s\n' "${missing_topics[@]}"
fi
echo

record_args=(
    --buffsize="${ROSBAG_BUFFER_MB}"
    -O "${BAG_PREFIX}"
)
if [[ "${RECORD_SEC}" != "0" && "${RECORD_SEC}" != "0.0" ]]; then
    record_args+=(--duration="${RECORD_SEC}")
fi

rosbag_pid=""
signal_received=false

handle_recorder_signal() {
    signal_received=true
    if [[ -n "${rosbag_pid}" ]] && kill -0 -- "-${rosbag_pid}" 2>/dev/null; then
        # rosbag's Python launcher and native record child share this new
        # process group.  Both receive SIGINT so .bag.active is finalized.
        kill -INT -- "-${rosbag_pid}" 2>/dev/null || true
    fi
}

trap handle_recorder_signal INT TERM
setsid rosbag record "${record_args[@]}" "${record_topics[@]}" &
rosbag_pid=$!

set +e
wait "${rosbag_pid}"
record_rc=$?
if kill -0 "${rosbag_pid}" 2>/dev/null; then
    wait "${rosbag_pid}"
    record_rc=$?
fi
# The launcher may exit just before the native recorder finishes its final
# chunk.  Wait for the whole process group, not only the direct child.
group_finalized=false
for _idx in $(seq 1 200); do
    if ! kill -0 -- "-${rosbag_pid}" 2>/dev/null; then
        group_finalized=true
        break
    fi
    sleep 0.05
done
if [[ "${group_finalized}" != "true" ]]; then
    kill -TERM -- "-${rosbag_pid}" 2>/dev/null || true
    sleep 1
    if kill -0 -- "-${rosbag_pid}" 2>/dev/null; then
        kill -KILL -- "-${rosbag_pid}" 2>/dev/null || true
    fi
    echo "[${SCRIPT_NAME}] ERROR: rosbag process group did not finalize within 10s." >&2
    record_rc=1
fi
set -e
trap - INT TERM
rosbag_pid=""

if [[ "${signal_received}" == "true" && "${group_finalized}" == "true" && ${record_rc} -ne 0 ]]; then
    record_rc=130
fi

if [[ ${record_rc} -ne 0 && ${record_rc} -ne 130 ]]; then
    echo "[${SCRIPT_NAME}] ERROR: rosbag record exited with status ${record_rc}." >&2
    exit "${record_rc}"
fi

if [[ ! -f "${BAG_PATH}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: expected bag was not finalized: ${BAG_PATH}" >&2
    exit 1
fi

rosbag info "${BAG_PATH}" >"${OUT_DIR}/${NAME}_bag_info.txt"
echo "[${SCRIPT_NAME}] Done: ${BAG_PATH}"
echo "[${SCRIPT_NAME}] Metadata: ${INFO_PATH}"
