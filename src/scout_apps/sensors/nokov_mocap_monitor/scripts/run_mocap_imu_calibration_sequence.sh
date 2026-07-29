#!/usr/bin/env bash
# Run a bounded real-robot planar motion sequence for mocap/IMU recording.
#
# This wrapper performs online prechecks, starts the recorder, and then starts
# one long-lived Python ROS publisher which owns all command timing and event
# markers.  No full six-axis/tilt calibration is attempted.

set -euo pipefail

SCRIPT_NAME="run_mocap_imu_calibration_sequence"
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
RECORDER_SCRIPT="${RECORDER_SCRIPT:-${SCRIPT_DIR}/record_mocap_imu_spin.sh}"
MOTION_SEQUENCE_SCRIPT="${MOTION_SEQUENCE_SCRIPT:-${SCRIPT_DIR}/mocap_imu_motion_sequence.py}"
BAG_VALIDATOR="${BAG_VALIDATOR:-${SCRIPT_DIR}/validate_mocap_imu_bag.py}"

ARM_MOTION="${ARM_MOTION:-NO}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ALLOW_EXISTING_CMD_PUBLISHER="${ALLOW_EXISTING_CMD_PUBLISHER:-false}"
START_RECORDER="${START_RECORDER:-true}"
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
RAW_MOCAP_POSE_TOPIC="/vrpn_client_node/${MOCAP_TRACKER}/pose"
REQUIRED_CMD_SUBSCRIBER="${REQUIRED_CMD_SUBSCRIBER:-/scout_base_node}"
REQUIRED_RECORDER_PREFIX="${REQUIRED_RECORDER_PREFIX:-/record_}"
CMD_HZ="${CMD_HZ:-50}"
PRECHECK_TIMEOUT="${PRECHECK_TIMEOUT:-5}"
CONNECTION_TIMEOUT="${CONNECTION_TIMEOUT:-12}"
MOCAP_STATUS_MAX_AGE="${MOCAP_STATUS_MAX_AGE:-3}"
MOCAP_POSE_MAX_AGE="${MOCAP_POSE_MAX_AGE:-0.20}"
IMU_MAX_AGE="${IMU_MAX_AGE:-0.20}"
ODOM_MAX_AGE="${ODOM_MAX_AGE:-0.20}"
COMMAND_LEASE_SEC="${COMMAND_LEASE_SEC:-0.25}"
MAX_PUBLISH_GAP_SEC="${MAX_PUBLISH_GAP_SEC:-0.10}"
FINAL_ZERO_SEC="${FINAL_ZERO_SEC:-1.0}"
COUNTDOWN_SEC="${COUNTDOWN_SEC:-5}"

DATE_TAG="${DATE_TAG:-$(date +%Y%m%d)}"
TIME_TAG="${TIME_TAG:-$(date +%H%M%S)}"
RUN_LABEL="${RUN_LABEL:-imu_mocap_planar_calib}"
OUT_DIR="${OUT_DIR:-/home/geist/slosh_bags/real/${DATE_TAG}_mocap_imu_calib}"
NAME="${NAME:-${RUN_LABEL}_${TIME_TAG}}"

# Defaults selected from the 2026-07-29 real-robot bag.  The previous 0.06 m/s
# command moved the robot, but provided too little steady-state/lateral signal.
LINEAR_LOW="${LINEAR_LOW:-0.10}"
LINEAR_NOMINAL="${LINEAR_NOMINAL:-0.15}"
STRAIGHT_SEC="${STRAIGHT_SEC:-1.5}"
SPIN_OMEGA="${SPIN_OMEGA:-0.20}"
SPIN_HOLD_SEC="${SPIN_HOLD_SEC:-5.0}"
SPIN_REV_LEG_SEC="${SPIN_REV_LEG_SEC:-3.0}"
SPIN_REV_MIDDLE_SEC="${SPIN_REV_MIDDLE_SEC:-6.0}"
S_V="${S_V:-0.10}"
S_OMEGA="${S_OMEGA:-0.40}"
S_HOLD_SEC="${S_HOLD_SEC:-1.0}"
S_REPEATS="${S_REPEATS:-3}"
STATIC_PRE_SEC="${STATIC_PRE_SEC:-60}"
STATIC_POST_SEC="${STATIC_POST_SEC:-60}"
SETTLE_SEC="${SETTLE_SEC:-2.5}"

# These cannot be raised through inherited shell state.
MAX_ABS_LINEAR=0.25
MAX_ABS_ANGULAR=0.50
MAX_CMD_HZ=100
MIN_CMD_HZ=50
MIN_STATIC_SEC=60

SEGMENT_TOPIC="/mocap_imu_calib/segment"
STATUS_TOPIC="/mocap_imu_calib/status"

motion_pid=""
recorder_pid=""
sequence_complete=false

require_bool() {
    local name="$1"
    local value="$2"
    if [[ "${value}" != "true" && "${value}" != "false" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: ${name} must be true or false (got ${value})." >&2
        exit 2
    fi
}

require_nonnegative_number() {
    local name="$1"
    local value="$2"
    if ! awk -v value="${value}" 'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/) }'; then
        echo "[${SCRIPT_NAME}] ERROR: ${name}=${value} must be non-negative." >&2
        exit 2
    fi
}

require_positive_number() {
    local name="$1"
    local value="$2"
    require_nonnegative_number "${name}" "${value}"
    if ! awk -v value="${value}" 'BEGIN { exit !(value > 0) }'; then
        echo "[${SCRIPT_NAME}] ERROR: ${name}=${value} must be greater than zero." >&2
        exit 2
    fi
}

require_range() {
    local name="$1"
    local value="$2"
    local minimum="$3"
    local maximum="$4"
    if ! awk -v value="${value}" -v minimum="${minimum}" -v maximum="${maximum}" \
        'BEGIN { exit !(value >= minimum && value <= maximum) }'; then
        echo "[${SCRIPT_NAME}] ERROR: ${name}=${value} must be in [${minimum}, ${maximum}]." >&2
        exit 2
    fi
}

require_bool VALIDATE_ONLY "${VALIDATE_ONLY}"
require_bool ALLOW_EXISTING_CMD_PUBLISHER "${ALLOW_EXISTING_CMD_PUBLISHER}"
require_bool START_RECORDER "${START_RECORDER}"
if [[ "${START_RECORDER}" != "true" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: START_RECORDER=false is not supported for the real sequence." >&2
    echo "[${SCRIPT_NAME}] The controlled recorder is required for online data-integrity checks." >&2
    exit 2
fi
if [[ "${ALLOW_EXISTING_CMD_PUBLISHER}" == "true" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: ALLOW_EXISTING_CMD_PUBLISHER=true is not supported for real motion." >&2
    echo "[${SCRIPT_NAME}] The persistent publisher aborts on any planner/teleop conflict." >&2
    exit 2
fi

if [[ "${VALIDATE_ONLY}" != "true" && "${ARM_MOTION}" != "YES" ]]; then
    echo "[${SCRIPT_NAME}] REFUSING TO MOVE: set ARM_MOTION=YES after clearing the area." >&2
    echo "[${SCRIPT_NAME}] This script directly publishes ${CMD_TOPIC} and has no obstacle avoidance." >&2
    exit 2
fi
if [[ ! "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "[${SCRIPT_NAME}] ERROR: unsafe MOCAP_TRACKER=${MOCAP_TRACKER}" >&2
    exit 2
fi
if [[ ! "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ || ! "${NAME}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "[${SCRIPT_NAME}] ERROR: RUN_LABEL and NAME must be filesystem-safe." >&2
    exit 2
fi
if [[ ! "${COUNTDOWN_SEC}" =~ ^[0-9]+$ ]]; then
    echo "[${SCRIPT_NAME}] ERROR: COUNTDOWN_SEC must be a non-negative integer." >&2
    exit 2
fi
if [[ ! "${S_REPEATS}" =~ ^[0-9]+$ ]] || (( S_REPEATS < 3 || S_REPEATS > 5 )); then
    echo "[${SCRIPT_NAME}] ERROR: S_REPEATS must be an integer in [3, 5]." >&2
    exit 2
fi

for spec in \
    "CMD_HZ:${CMD_HZ}" \
    "PRECHECK_TIMEOUT:${PRECHECK_TIMEOUT}" \
    "CONNECTION_TIMEOUT:${CONNECTION_TIMEOUT}" \
    "MOCAP_STATUS_MAX_AGE:${MOCAP_STATUS_MAX_AGE}" \
    "MOCAP_POSE_MAX_AGE:${MOCAP_POSE_MAX_AGE}" \
    "IMU_MAX_AGE:${IMU_MAX_AGE}" \
    "ODOM_MAX_AGE:${ODOM_MAX_AGE}" \
    "COMMAND_LEASE_SEC:${COMMAND_LEASE_SEC}" \
    "MAX_PUBLISH_GAP_SEC:${MAX_PUBLISH_GAP_SEC}" \
    "FINAL_ZERO_SEC:${FINAL_ZERO_SEC}" \
    "LINEAR_LOW:${LINEAR_LOW}" \
    "LINEAR_NOMINAL:${LINEAR_NOMINAL}" \
    "STRAIGHT_SEC:${STRAIGHT_SEC}" \
    "SPIN_OMEGA:${SPIN_OMEGA}" \
    "SPIN_HOLD_SEC:${SPIN_HOLD_SEC}" \
    "SPIN_REV_LEG_SEC:${SPIN_REV_LEG_SEC}" \
    "SPIN_REV_MIDDLE_SEC:${SPIN_REV_MIDDLE_SEC}" \
    "S_V:${S_V}" \
    "S_OMEGA:${S_OMEGA}" \
    "S_HOLD_SEC:${S_HOLD_SEC}" \
    "STATIC_PRE_SEC:${STATIC_PRE_SEC}" \
    "STATIC_POST_SEC:${STATIC_POST_SEC}" \
    "SETTLE_SEC:${SETTLE_SEC}"; do
    require_positive_number "${spec%%:*}" "${spec#*:}"
done

require_range CMD_HZ "${CMD_HZ}" "${MIN_CMD_HZ}" "${MAX_CMD_HZ}"
require_range PRECHECK_TIMEOUT "${PRECHECK_TIMEOUT}" 1 30
require_range CONNECTION_TIMEOUT "${CONNECTION_TIMEOUT}" 1 30
require_range MOCAP_STATUS_MAX_AGE "${MOCAP_STATUS_MAX_AGE}" 0.5 10
require_range COMMAND_LEASE_SEC "${COMMAND_LEASE_SEC}" 0.05 0.49
require_range MAX_PUBLISH_GAP_SEC "${MAX_PUBLISH_GAP_SEC}" 0.02 0.25
require_range MOCAP_POSE_MAX_AGE "${MOCAP_POSE_MAX_AGE}" 0.05 1.0
require_range IMU_MAX_AGE "${IMU_MAX_AGE}" 0.05 1.0
require_range ODOM_MAX_AGE "${ODOM_MAX_AGE}" 0.05 1.0
if ! awk -v gap="${MAX_PUBLISH_GAP_SEC}" -v lease="${COMMAND_LEASE_SEC}" \
    'BEGIN { exit !(gap < lease) }'; then
    echo "[${SCRIPT_NAME}] ERROR: MAX_PUBLISH_GAP_SEC must be less than COMMAND_LEASE_SEC." >&2
    exit 2
fi
require_range LINEAR_LOW "${LINEAR_LOW}" 0.000001 "${MAX_ABS_LINEAR}"
require_range LINEAR_NOMINAL "${LINEAR_NOMINAL}" 0.000001 "${MAX_ABS_LINEAR}"
require_range S_V "${S_V}" 0.000001 "${MAX_ABS_LINEAR}"
require_range SPIN_OMEGA "${SPIN_OMEGA}" 0.000001 "${MAX_ABS_ANGULAR}"
require_range S_OMEGA "${S_OMEGA}" 0.000001 "${MAX_ABS_ANGULAR}"
require_range STRAIGHT_SEC "${STRAIGHT_SEC}" 0.1 2.0
require_range SPIN_HOLD_SEC "${SPIN_HOLD_SEC}" 0.1 10.0
require_range SPIN_REV_LEG_SEC "${SPIN_REV_LEG_SEC}" 0.1 5.0
require_range SPIN_REV_MIDDLE_SEC "${SPIN_REV_MIDDLE_SEC}" 0.1 10.0
require_range S_HOLD_SEC "${S_HOLD_SEC}" 0.1 1.5
require_range SETTLE_SEC "${SETTLE_SEC}" 0.1 10.0
require_range FINAL_ZERO_SEC "${FINAL_ZERO_SEC}" 0.2 5.0
require_range STATIC_PRE_SEC "${STATIC_PRE_SEC}" "${MIN_STATIC_SEC}" 300
require_range STATIC_POST_SEC "${STATIC_POST_SEC}" "${MIN_STATIC_SEC}" 300
if (( COUNTDOWN_SEC > 15 )); then
    echo "[${SCRIPT_NAME}] ERROR: COUNTDOWN_SEC must be <= 15." >&2
    exit 2
fi
if ! awk -v low="${LINEAR_LOW}" -v nominal="${LINEAR_NOMINAL}" \
    'BEGIN { exit !(nominal >= low) }'; then
    echo "[${SCRIPT_NAME}] ERROR: LINEAR_NOMINAL must be >= LINEAR_LOW." >&2
    exit 2
fi

if [[ -f /opt/ros/noetic/setup.bash ]]; then
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
if [[ ! -x "${MOTION_SEQUENCE_SCRIPT}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: motion sequence helper is not executable: ${MOTION_SEQUENCE_SCRIPT}" >&2
    exit 1
fi
if [[ ! -x "${BAG_VALIDATOR}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: bag validator is not executable: ${BAG_VALIDATOR}" >&2
    exit 1
fi
if [[ "${START_RECORDER}" == "true" && ! -x "${RECORDER_SCRIPT}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: recorder is not executable: ${RECORDER_SCRIPT}" >&2
    exit 1
fi

wait_for_message() {
    local topic="$1"
    local label="$2"
    echo "[${SCRIPT_NAME}] Checking ${label}: ${topic}"
    if ! timeout "${PRECHECK_TIMEOUT}" rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1; then
        echo "[${SCRIPT_NAME}] ERROR: no ${label} message within ${PRECHECK_TIMEOUT}s." >&2
        exit 1
    fi
}

check_mocap_ok() {
    local raw_status
    local status_line
    local state
    local tracker=""
    local token
    raw_status="$(timeout "${PRECHECK_TIMEOUT}" rostopic echo -n 1 "${MOCAP_STATUS_TOPIC:-/mocap/status}" 2>/dev/null || true)"
    status_line="$(printf '%s\n' "${raw_status}" | awk '/^data: / { sub(/^data: /, ""); print; exit }')"
    status_line="${status_line#\'}"
    status_line="${status_line%\'}"
    status_line="${status_line#\"}"
    status_line="${status_line%\"}"
    state="${status_line%% *}"
    for token in ${status_line}; do
        if [[ "${token}" == tracker=* ]]; then
            tracker="${token#tracker=}"
        fi
    done
    if [[ "${state}" != "OK" || "${tracker}" != "${MOCAP_TRACKER}" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: mocap is not exactly OK for ${MOCAP_TRACKER}: ${status_line:-missing}" >&2
        return 1
    fi
}

if ! rostopic list >/dev/null 2>&1; then
    echo "[${SCRIPT_NAME}] ERROR: ROS master is not reachable." >&2
    exit 1
fi
wait_for_message "${IMU_TOPIC}" "IMU"
wait_for_message "${RAW_MOCAP_POSE_TOPIC}" "raw mocap pose"
wait_for_message /mocap/scout_pose "mocap bridge pose"
wait_for_message "${ODOM_TOPIC}" "odometry"
check_mocap_ok

if [[ "${ALLOW_EXISTING_CMD_PUBLISHER}" != "true" ]]; then
    cmd_info="$(rostopic info "${CMD_TOPIC}" 2>/dev/null || true)"
    publisher_count="$(printf '%s\n' "${cmd_info}" | awk '
        /^Publishers:/ { in_publishers=1; next }
        /^Subscribers:/ { in_publishers=0 }
        in_publishers && /^[[:space:]]*\*/ { count++ }
        END { print count+0 }
    ')"
    if (( publisher_count > 0 )); then
        echo "[${SCRIPT_NAME}] ERROR: ${CMD_TOPIC} already has ${publisher_count} publisher(s)." >&2
        echo "[${SCRIPT_NAME}] Stop planner/teleop publishers before running this sequence." >&2
        exit 1
    fi
fi

STRAIGHT_LOW_DISTANCE="$(awk -v v="${LINEAR_LOW}" -v t="${STRAIGHT_SEC}" 'BEGIN { printf "%.3f", v*t }')"
STRAIGHT_NOMINAL_DISTANCE="$(awk -v v="${LINEAR_NOMINAL}" -v t="${STRAIGHT_SEC}" 'BEGIN { printf "%.3f", v*t }')"
S_PASS_DISTANCE="$(awk -v v="${S_V}" -v t="${S_HOLD_SEC}" 'BEGIN { printf "%.3f", 2*v*t }')"
S_RADIUS="$(awk -v v="${S_V}" -v w="${S_OMEGA}" 'BEGIN { printf "%.3f", v/w }')"
S_LATERAL_ACCEL="$(awk -v v="${S_V}" -v w="${S_OMEGA}" 'BEGIN { printf "%.4f", v*w }')"
TOTAL_TRANSLATION="$(awk -v vl="${LINEAR_LOW}" -v vn="${LINEAR_NOMINAL}" -v ts="${STRAIGHT_SEC}" -v sv="${S_V}" -v sh="${S_HOLD_SEC}" -v n="${S_REPEATS}" 'BEGIN { printf "%.3f", 2*(vl+vn)*ts + 8*sv*sh*n }')"
TOTAL_YAW="$(awk -v w="${SPIN_OMEGA}" -v hold="${SPIN_HOLD_SEC}" -v leg="${SPIN_REV_LEG_SEC}" -v middle="${SPIN_REV_MIDDLE_SEC}" -v sw="${S_OMEGA}" -v sh="${S_HOLD_SEC}" -v n="${S_REPEATS}" 'BEGIN { printf "%.3f", 2*w*hold + w*(2*leg+middle) + 8*sw*sh*n }')"
PLANNED_DURATION="$(awk -v cd="${COUNTDOWN_SEC}" -v pre="${STATIC_PRE_SEC}" -v post="${STATIC_POST_SEC}" -v st="${STRAIGHT_SEC}" -v settle="${SETTLE_SEC}" -v spin="${SPIN_HOLD_SEC}" -v leg="${SPIN_REV_LEG_SEC}" -v middle="${SPIN_REV_MIDDLE_SEC}" -v sh="${S_HOLD_SEC}" -v n="${S_REPEATS}" -v zero="${FINAL_ZERO_SEC}" 'BEGIN { printf "%.1f", cd+pre+post+4*st+7*settle+2*spin+2*leg+middle+n*(8*sh+4*settle)+zero }')"

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

for guard in \
    "STRAIGHT_NOMINAL_DISTANCE:${STRAIGHT_NOMINAL_DISTANCE}:0.40" \
    "S_PASS_DISTANCE:${S_PASS_DISTANCE}:0.40" \
    "TOTAL_TRANSLATION:${TOTAL_TRANSLATION}:6.0" \
    "TOTAL_YAW:${TOTAL_YAW}:30.0" \
    "PLANNED_DURATION:${PLANNED_DURATION}:600.0"; do
    guard_name="${guard%%:*}"
    guard_rest="${guard#*:}"
    guard_value="${guard_rest%%:*}"
    guard_limit="${guard_rest#*:}"
    if ! awk -v value="${guard_value}" -v limit="${guard_limit}" 'BEGIN { exit !(value <= limit) }'; then
        echo "[${SCRIPT_NAME}] ERROR: ${guard_name}=${guard_value} exceeds hard limit ${guard_limit}." >&2
        exit 2
    fi
done

if [[ "${VALIDATE_ONLY}" == "true" ]]; then
    echo "[${SCRIPT_NAME}] Validation passed; no recorder or motion publisher was started."
    echo "[${SCRIPT_NAME}] Planar profile: cmd=${CMD_HZ}Hz, straight=${STRAIGHT_LOW_DISTANCE}/${STRAIGHT_NOMINAL_DISTANCE}m."
    echo "[${SCRIPT_NAME}] S pass=${S_PASS_DISTANCE}m, radius=${S_RADIUS}m, nominal |v*omega|=${S_LATERAL_ACCEL}m/s^2."
    echo "[${SCRIPT_NAME}] Static pre/post=${STATIC_PRE_SEC}/${STATIC_POST_SEC}s; full six-axis tilt is not included."
    exit 0
fi

mkdir -p "${OUT_DIR}"
BAG_PATH="${OUT_DIR}/${NAME}.bag"
BAG_ACTIVE_PATH="${BAG_PATH}.active"
RECORDER_LOG="${OUT_DIR}/${NAME}_recorder.log"
TIMELINE_PATH="${OUT_DIR}/${NAME}_timeline.tsv"
CONFIG_PATH="${OUT_DIR}/${NAME}_sequence_config.txt"
MOTION_LOG="${OUT_DIR}/${NAME}_motion.log"
VALIDATION_REPORT="${OUT_DIR}/${NAME}_validation.json"
SHA256_PATH="${OUT_DIR}/${NAME}_sha256.txt"

artifacts=(
    "${BAG_PATH}"
    "${BAG_ACTIVE_PATH}"
    "${RECORDER_LOG}"
    "${TIMELINE_PATH}"
    "${CONFIG_PATH}"
    "${MOTION_LOG}"
    "${VALIDATION_REPORT}"
    "${SHA256_PATH}"
    "${OUT_DIR}/${NAME}_info.txt"
    "${OUT_DIR}/${NAME}_topics.txt"
    "${OUT_DIR}/${NAME}_missing_topics.txt"
    "${OUT_DIR}/${NAME}_bag_info.txt"
)
for artifact in "${artifacts[@]}"; do
    if [[ -e "${artifact}" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: refusing to overwrite existing artifact: ${artifact}" >&2
        echo "[${SCRIPT_NAME}] Choose a new RUN_LABEL or NAME." >&2
        exit 1
    fi
done

{
    echo "created_at=$(date --iso-8601=seconds)"
    echo "run_label=${RUN_LABEL}"
    echo "name=${NAME}"
    echo "profile=planar_imu_mocap_raw_recording"
    echo "publisher_mode=persistent_python"
    echo "motion_sequence_script=${MOTION_SEQUENCE_SCRIPT}"
    echo "mocap_tracker=${MOCAP_TRACKER}"
    echo "imu_topic=${IMU_TOPIC}"
    echo "imu_max_age=${IMU_MAX_AGE}"
    echo "cmd_topic=${CMD_TOPIC}"
    echo "required_cmd_subscriber=${REQUIRED_CMD_SUBSCRIBER}"
    echo "required_recorder_prefix=${REQUIRED_RECORDER_PREFIX}"
    echo "odom_topic=${ODOM_TOPIC}"
    echo "odom_max_age=${ODOM_MAX_AGE}"
    echo "raw_mocap_pose_topic=${RAW_MOCAP_POSE_TOPIC}"
    echo "mocap_pose_max_age=${MOCAP_POSE_MAX_AGE}"
    echo "cmd_hz=${CMD_HZ}"
    echo "precheck_timeout=${PRECHECK_TIMEOUT}"
    echo "connection_timeout=${CONNECTION_TIMEOUT}"
    echo "mocap_status_max_age=${MOCAP_STATUS_MAX_AGE}"
    echo "command_lease_sec=${COMMAND_LEASE_SEC}"
    echo "max_publish_gap_sec=${MAX_PUBLISH_GAP_SEC}"
    echo "linear_low=${LINEAR_LOW}"
    echo "linear_nominal=${LINEAR_NOMINAL}"
    echo "straight_sec=${STRAIGHT_SEC}"
    echo "straight_low_command_distance_m=${STRAIGHT_LOW_DISTANCE}"
    echo "straight_nominal_command_distance_m=${STRAIGHT_NOMINAL_DISTANCE}"
    echo "spin_omega=${SPIN_OMEGA}"
    echo "spin_hold_sec=${SPIN_HOLD_SEC}"
    echo "spin_rev_leg_sec=${SPIN_REV_LEG_SEC}"
    echo "spin_rev_middle_sec=${SPIN_REV_MIDDLE_SEC}"
    echo "s_v=${S_V}"
    echo "s_omega=${S_OMEGA}"
    echo "s_hold_sec=${S_HOLD_SEC}"
    echo "s_pass_command_distance_m=${S_PASS_DISTANCE}"
    echo "s_radius_m=${S_RADIUS}"
    echo "s_nominal_lateral_accel_mps2=${S_LATERAL_ACCEL}"
    echo "s_repeats=${S_REPEATS}"
    echo "static_pre_sec=${STATIC_PRE_SEC}"
    echo "static_post_sec=${STATIC_POST_SEC}"
    echo "settle_sec=${SETTLE_SEC}"
    echo "countdown_sec=${COUNTDOWN_SEC}"
    echo "final_zero_sec=${FINAL_ZERO_SEC}"
    echo "total_commanded_translation_m=${TOTAL_TRANSLATION}"
    echo "total_commanded_abs_yaw_rad=${TOTAL_YAW}"
    echo "planned_duration_sec=${PLANNED_DURATION}"
    echo "full_six_axis_calibration=false"
    echo "git_hash=${GIT_HASH}"
    echo "git_dirty=${GIT_DIRTY}"
} >"${CONFIG_PATH}"

backup_send_zero() {
    timeout 2s rostopic pub -r 50 "${CMD_TOPIC}" geometry_msgs/Twist \
        '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
        >/dev/null 2>&1 || true
}

stop_motion_runner() {
    if [[ -z "${motion_pid}" ]] || ! kill -0 "${motion_pid}" 2>/dev/null; then
        motion_pid=""
        return
    fi
    kill -INT "${motion_pid}" 2>/dev/null || true
    for _idx in $(seq 1 80); do
        if ! kill -0 "${motion_pid}" 2>/dev/null; then
            break
        fi
        sleep 0.05
    done
    if kill -0 "${motion_pid}" 2>/dev/null; then
        echo "[${SCRIPT_NAME}] WARN: motion helper did not stop after SIGINT; using backup zero." >&2
        backup_send_zero
        kill -TERM "${motion_pid}" 2>/dev/null || true
        sleep 1
    fi
    if kill -0 "${motion_pid}" 2>/dev/null; then
        kill -KILL "${motion_pid}" 2>/dev/null || true
    fi
    wait "${motion_pid}" 2>/dev/null || true
    motion_pid=""
}

stop_recorder() {
    if [[ -z "${recorder_pid}" ]] || ! kill -0 "${recorder_pid}" 2>/dev/null; then
        recorder_pid=""
        return
    fi
    # A non-interactive background shell inherits SIGINT=ignored.  SIGTERM is
    # trapped by the recorder wrapper, which forwards graceful SIGINT to the
    # isolated rosbag process group.
    kill -TERM "${recorder_pid}" 2>/dev/null || true
    for _idx in $(seq 1 300); do
        if ! kill -0 "${recorder_pid}" 2>/dev/null; then
            break
        fi
        sleep 0.05
    done
    local forced_kill=false
    local recorder_rc=0
    if kill -0 "${recorder_pid}" 2>/dev/null; then
        echo "[${SCRIPT_NAME}] ERROR: recorder wrapper did not stop within 15s." >&2
        kill -KILL "${recorder_pid}" 2>/dev/null || true
        forced_kill=true
    fi
    wait "${recorder_pid}" 2>/dev/null || recorder_rc=$?
    recorder_pid=""
    if [[ "${forced_kill}" == "true" || ${recorder_rc} -ne 0 ]]; then
        return 1
    fi
    return 0
}

cleanup() {
    local status=$?
    trap - INT TERM EXIT
    stop_motion_runner
    backup_send_zero
    stop_recorder || true
    exit "${status}"
}

handle_signal() {
    echo
    echo "[${SCRIPT_NAME}] Interrupted; zeroing the robot before finalizing the bag." >&2
    exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

start_recorder() {
    MOCAP_TRACKER="${MOCAP_TRACKER}" \
    IMU_TOPIC="${IMU_TOPIC}" \
    CMD_TOPIC="${CMD_TOPIC}" \
    ODOM_TOPIC="${ODOM_TOPIC}" \
    RUN_LABEL="${RUN_LABEL}" \
    NAME="${NAME}" \
    OUT_DIR="${OUT_DIR}" \
    RECORD_SEC=0 \
    bash "${RECORDER_SCRIPT}" >"${RECORDER_LOG}" 2>&1 &
    recorder_pid=$!

    local ready=false
    local idx
    for ((idx=0; idx<80; idx++)); do
        if [[ -f "${BAG_ACTIVE_PATH}" ]]; then
            ready=true
            break
        fi
        if ! kill -0 "${recorder_pid}" 2>/dev/null; then
            echo "[${SCRIPT_NAME}] ERROR: recorder exited during startup." >&2
            tail -n 80 "${RECORDER_LOG}" >&2 || true
            exit 1
        fi
        sleep 0.25
    done
    if [[ "${ready}" != "true" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: recorder did not create ${BAG_ACTIVE_PATH}." >&2
        tail -n 80 "${RECORDER_LOG}" >&2 || true
        exit 1
    fi
    echo "[${SCRIPT_NAME}] Recorder ready: ${BAG_PATH}"
}

echo
echo "================ Planar mocap/IMU recording sequence ================"
echo "  output bag       = ${BAG_PATH}"
echo "  cmd publisher    = persistent ${CMD_HZ} Hz -> ${CMD_TOPIC}"
echo "  straight v       = ${LINEAR_LOW} / ${LINEAR_NOMINAL} m/s"
echo "  S command        = v=${S_V} m/s, |omega|=${S_OMEGA} rad/s"
echo "  S geometry       = pass ${S_PASS_DISTANCE} m, radius ${S_RADIUS} m"
echo "  lateral stimulus = ${S_LATERAL_ACCEL} m/s^2 nominal"
echo "  static pre/post  = ${STATIC_PRE_SEC} / ${STATIC_POST_SEC} s"
echo "  scope             = planar raw-data recording; no six-axis tilt"
echo "======================================================================"
echo "Clear at least ~1 m around the robot and keep the emergency stop ready."
echo

start_recorder

motion_args=(
    --arm-motion YES
    --cmd-topic "${CMD_TOPIC}"
    --segment-topic "${SEGMENT_TOPIC}"
    --status-topic "${STATUS_TOPIC}"
    --mocap-status-topic "${MOCAP_STATUS_TOPIC:-/mocap/status}"
    --mocap-tracker "${MOCAP_TRACKER}"
    --raw-mocap-pose-topic "${RAW_MOCAP_POSE_TOPIC}"
    --imu-topic "${IMU_TOPIC}"
    --odom-topic "${ODOM_TOPIC}"
    --required-cmd-subscriber "${REQUIRED_CMD_SUBSCRIBER}"
    --required-recorder-prefix "${REQUIRED_RECORDER_PREFIX}"
    --timeline-path "${TIMELINE_PATH}"
    --log-path "${MOTION_LOG}"
    --recorder-pid "${recorder_pid:-0}"
    --connection-timeout "${CONNECTION_TIMEOUT}"
    --mocap-status-max-age "${MOCAP_STATUS_MAX_AGE}"
    --mocap-pose-max-age "${MOCAP_POSE_MAX_AGE}"
    --imu-max-age "${IMU_MAX_AGE}"
    --odom-max-age "${ODOM_MAX_AGE}"
    --command-lease-sec "${COMMAND_LEASE_SEC}"
    --max-publish-gap-sec "${MAX_PUBLISH_GAP_SEC}"
    --final-zero-sec "${FINAL_ZERO_SEC}"
    --cmd-hz "${CMD_HZ}"
    --countdown-sec "${COUNTDOWN_SEC}"
    --linear-low "${LINEAR_LOW}"
    --linear-nominal "${LINEAR_NOMINAL}"
    --straight-sec "${STRAIGHT_SEC}"
    --spin-omega "${SPIN_OMEGA}"
    --spin-hold-sec "${SPIN_HOLD_SEC}"
    --spin-rev-leg-sec "${SPIN_REV_LEG_SEC}"
    --spin-rev-middle-sec "${SPIN_REV_MIDDLE_SEC}"
    --s-v "${S_V}"
    --s-omega "${S_OMEGA}"
    --s-hold-sec "${S_HOLD_SEC}"
    --s-repeats "${S_REPEATS}"
    --static-pre-sec "${STATIC_PRE_SEC}"
    --static-post-sec "${STATIC_POST_SEC}"
    --settle-sec "${SETTLE_SEC}"
)

python3 "${MOTION_SEQUENCE_SCRIPT}" "${motion_args[@]}" &
motion_pid=$!
set +e
wait "${motion_pid}"
motion_rc=$?
set -e
motion_pid=""

if [[ ${motion_rc} -ne 0 ]]; then
    echo "[${SCRIPT_NAME}] ERROR: motion helper exited with status ${motion_rc}." >&2
    exit "${motion_rc}"
fi

sequence_complete=true
if ! stop_recorder; then
    echo "[${SCRIPT_NAME}] ERROR: recorder did not finalize cleanly." >&2
    exit 1
fi
trap - INT TERM EXIT

if [[ ! -f "${BAG_PATH}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: expected finalized bag is missing: ${BAG_PATH}" >&2
    exit 1
fi
if [[ -e "${BAG_ACTIVE_PATH}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: recorder left an active bag: ${BAG_ACTIVE_PATH}" >&2
    exit 1
fi

python3 "${BAG_VALIDATOR}" \
    --bag "${BAG_PATH}" \
    --config "${CONFIG_PATH}" \
    --report "${VALIDATION_REPORT}" \
    --cmd-topic "${CMD_TOPIC}" \
    --imu-topic "${IMU_TOPIC}" \
    --odom-topic "${ODOM_TOPIC}" \
    --mocap-pose-topic "${RAW_MOCAP_POSE_TOPIC}" \
    --segment-topic "${SEGMENT_TOPIC}" \
    --status-topic "${STATUS_TOPIC}"

sha256sum "${BAG_PATH}" >"${SHA256_PATH}"

echo
echo "[${SCRIPT_NAME}] Complete."
echo "[${SCRIPT_NAME}] Bag: ${BAG_PATH}"
echo "[${SCRIPT_NAME}] Timeline: ${TIMELINE_PATH}"
echo "[${SCRIPT_NAME}] Config: ${CONFIG_PATH}"
echo "[${SCRIPT_NAME}] Motion log: ${MOTION_LOG}"
echo "[${SCRIPT_NAME}] Validation: ${VALIDATION_REPORT}"
echo "[${SCRIPT_NAME}] SHA-256: ${SHA256_PATH}"
