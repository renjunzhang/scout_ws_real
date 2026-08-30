#!/usr/bin/env bash
# Record a short yaw-excitation bag and estimate NOKOV lag relative to IMU.
#
# Safety boundary: this script never publishes /cmd_vel.  During the marked
# motion phase, the operator creates safe left/right yaw motion with the Scout
# remote, a separately supervised teleop process, or manual rigid-body motion.

set -euo pipefail

SCRIPT_NAME="run_mocap_imu_relative_latency_trial"
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
ANALYZER_SCRIPT="${ANALYZER_SCRIPT:-${SCRIPT_DIR}/analyze_mocap_imu_relative_latency.py}"

VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
ALLOW_EXISTING_CMD_PUBLISHER="${ALLOW_EXISTING_CMD_PUBLISHER:-false}"
MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
CMD_TOPIC="${CMD_TOPIC:-/cmd_vel}"
MOCAP_STATUS_TOPIC="${MOCAP_STATUS_TOPIC:-/mocap/status}"
SEGMENT_TOPIC="${SEGMENT_TOPIC:-/mocap_imu_calib/segment}"
RAW_MOCAP_POSE_TOPIC="/vrpn_client_node/${MOCAP_TRACKER}/pose"
PRECHECK_TIMEOUT="${PRECHECK_TIMEOUT:-5}"

# Short development protocol: static baseline, repeated yaw reversals, static
# tail.  Integer seconds keep the operator countdown unambiguous.
STATIC_PRE_SEC="${STATIC_PRE_SEC:-5}"
MOTION_SEC="${MOTION_SEC:-30}"
STATIC_POST_SEC="${STATIC_POST_SEC:-5}"

LATENCY_TEST_ID="${LATENCY_TEST_ID:-N01}"
ATTEMPT="${ATTEMPT:-01}"
DATE_TAG="${DATE_TAG:-$(date +%Y%m%d)}"
RUN_LABEL="${RUN_LABEL:-nokov_imu_relative_latency}"
OUT_DIR="${OUT_DIR:-/home/geist/slosh_bags/real/${DATE_TAG}_mocap_imu_relative_latency}"
NAME="${NAME:-DEV_NOKOV_IMU_LATENCY_${LATENCY_TEST_ID}_a${ATTEMPT}}"

require_bool() {
    local name="$1"
    local value="$2"
    if [[ "${value}" != "true" && "${value}" != "false" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: ${name} must be true or false (got ${value})." >&2
        exit 2
    fi
}

require_integer_range() {
    local name="$1"
    local value="$2"
    local minimum="$3"
    local maximum="$4"
    if [[ ! "${value}" =~ ^[0-9]+$ ]] || (( value < minimum || value > maximum )); then
        echo "[${SCRIPT_NAME}] ERROR: ${name}=${value} must be an integer in [${minimum}, ${maximum}]." >&2
        exit 2
    fi
}

require_bool VALIDATE_ONLY "${VALIDATE_ONLY}"
require_bool ALLOW_EXISTING_CMD_PUBLISHER "${ALLOW_EXISTING_CMD_PUBLISHER}"
require_integer_range PRECHECK_TIMEOUT "${PRECHECK_TIMEOUT}" 1 30
require_integer_range STATIC_PRE_SEC "${STATIC_PRE_SEC}" 3 30
require_integer_range MOTION_SEC "${MOTION_SEC}" 15 90
require_integer_range STATIC_POST_SEC "${STATIC_POST_SEC}" 3 30

if [[ ! "${MOCAP_TRACKER}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
    echo "[${SCRIPT_NAME}] ERROR: unsafe MOCAP_TRACKER=${MOCAP_TRACKER}." >&2
    exit 2
fi
for identity in "${LATENCY_TEST_ID}" "${ATTEMPT}" "${RUN_LABEL}" "${NAME}"; do
    if [[ ! "${identity}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
        echo "[${SCRIPT_NAME}] ERROR: unsafe trial identity: ${identity}." >&2
        exit 2
    fi
done
if [[ "${NAME}" == *.bag || "${NAME}" == *.active ]]; then
    echo "[${SCRIPT_NAME}] ERROR: NAME must not end in .bag or .active." >&2
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
    echo "[${SCRIPT_NAME}] ERROR: ${WS_ROOT}/devel/setup.bash not found." >&2
    exit 1
fi

for required in rostopic rosbag timeout python3; do
    if ! command -v "${required}" >/dev/null 2>&1; then
        echo "[${SCRIPT_NAME}] ERROR: required command is unavailable: ${required}." >&2
        exit 1
    fi
done
if [[ ! -x "${RECORDER_SCRIPT}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: recorder is not executable: ${RECORDER_SCRIPT}." >&2
    exit 1
fi
if [[ ! -x "${ANALYZER_SCRIPT}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: analyzer is not executable: ${ANALYZER_SCRIPT}." >&2
    exit 1
fi
if ! rostopic list >/dev/null 2>&1; then
    echo "[${SCRIPT_NAME}] ERROR: ROS master is not reachable." >&2
    exit 1
fi

wait_for_message() {
    local topic="$1"
    local label="$2"
    echo "[${SCRIPT_NAME}] checking ${label}: ${topic}"
    if ! timeout "${PRECHECK_TIMEOUT}" rostopic echo --noarr -n 1 "${topic}" >/dev/null 2>&1; then
        echo "[${SCRIPT_NAME}] ERROR: no fresh ${label} message within ${PRECHECK_TIMEOUT}s: ${topic}." >&2
        exit 1
    fi
}

check_mocap_ok() {
    local raw_status=""
    local status_line=""
    local state=""
    local tracker=""
    local token=""
    raw_status="$(timeout "${PRECHECK_TIMEOUT}" rostopic echo -n 1 "${MOCAP_STATUS_TOPIC}" 2>/dev/null || true)"
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
        echo "[${SCRIPT_NAME}] ERROR: mocap is not exactly OK for ${MOCAP_TRACKER}: ${status_line:-missing}." >&2
        exit 1
    fi
    echo "[${SCRIPT_NAME}] mocap status: ${status_line}"
}

wait_for_message "${IMU_TOPIC}" "IMU"
wait_for_message "${RAW_MOCAP_POSE_TOPIC}" "raw NOKOV pose"
wait_for_message "${ODOM_TOPIC}" "odometry"
wait_for_message "${MOCAP_STATUS_TOPIC}" "mocap status"
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
        echo "[${SCRIPT_NAME}] Stop autonomous planners first. For a supervised teleop publisher, explicitly set ALLOW_EXISTING_CMD_PUBLISHER=true." >&2
        exit 1
    fi
fi

echo
echo "================ NOKOV relative-to-IMU latency trial ================"
echo "  tracker          = ${MOCAP_TRACKER}"
echo "  IMU topic        = ${IMU_TOPIC}"
echo "  raw pose topic   = ${RAW_MOCAP_POSE_TOPIC}"
echo "  static / motion  = ${STATIC_PRE_SEC} / ${MOTION_SEC} / ${STATIC_POST_SEC} s"
echo "  output           = ${OUT_DIR}/${NAME}.bag"
echo "  command policy   = RECORDING ONLY; this script never publishes /cmd_vel"
echo "======================================================================"
echo "During the motion phase, rotate left/right about every 1--2 seconds."
echo "Use at least 10 clear reversals, keep translation small, and keep the emergency stop ready."
echo

if [[ "${VALIDATE_ONLY}" == "true" ]]; then
    echo "[${SCRIPT_NAME}] validation passed; recorder was not started."
    exit 0
fi

mkdir -p "${OUT_DIR}"
BAG_PATH="${OUT_DIR}/${NAME}.bag"
BAG_ACTIVE_PATH="${BAG_PATH}.active"
RECORDER_LOG="${OUT_DIR}/${NAME}_recorder.log"
CONFIG_PATH="${OUT_DIR}/${NAME}_trial_config.txt"
ANALYSIS_LOG="${OUT_DIR}/${NAME}_analysis.log"
REPORT_JSON="${OUT_DIR}/${NAME}_relative_latency.json"
SUMMARY_MD="${OUT_DIR}/${NAME}_relative_latency.md"
SIGNALS_CSV="${OUT_DIR}/${NAME}_relative_latency_signals.csv"
PLOT_PATH="${OUT_DIR}/${NAME}_relative_latency.png"
SHA256_PATH="${OUT_DIR}/${NAME}_sha256.txt"

artifacts=(
    "${BAG_PATH}"
    "${BAG_ACTIVE_PATH}"
    "${RECORDER_LOG}"
    "${CONFIG_PATH}"
    "${ANALYSIS_LOG}"
    "${REPORT_JSON}"
    "${SUMMARY_MD}"
    "${SIGNALS_CSV}"
    "${PLOT_PATH}"
    "${SHA256_PATH}"
    "${OUT_DIR}/${NAME}_info.txt"
    "${OUT_DIR}/${NAME}_topics.txt"
    "${OUT_DIR}/${NAME}_missing_topics.txt"
    "${OUT_DIR}/${NAME}_bag_info.txt"
)
for artifact in "${artifacts[@]}"; do
    if [[ -e "${artifact}" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: refusing to overwrite existing artifact: ${artifact}." >&2
        exit 1
    fi
done

{
    echo "created_at=$(date --iso-8601=seconds)"
    echo "latency_test_id=${LATENCY_TEST_ID}"
    echo "attempt=${ATTEMPT}"
    echo "name=${NAME}"
    echo "mocap_tracker=${MOCAP_TRACKER}"
    echo "mocap_pose_topic=${RAW_MOCAP_POSE_TOPIC}"
    echo "imu_topic=${IMU_TOPIC}"
    echo "odom_topic=${ODOM_TOPIC}"
    echo "cmd_topic=${CMD_TOPIC}"
    echo "segment_topic=${SEGMENT_TOPIC}"
    echo "static_pre_sec=${STATIC_PRE_SEC}"
    echo "motion_sec=${MOTION_SEC}"
    echo "static_post_sec=${STATIC_POST_SEC}"
    echo "motion_source=external_operator"
    echo "script_publishes_cmd_vel=false"
    echo "claim_scope=NOKOV_relative_to_IMU_lag_only"
} >"${CONFIG_PATH}"

recorder_pid=""

stop_recorder() {
    if [[ -z "${recorder_pid}" ]] || ! kill -0 "${recorder_pid}" 2>/dev/null; then
        recorder_pid=""
        return 0
    fi
    # Background bash ignores SIGINT at process creation.  The recorder traps
    # SIGTERM and forwards a graceful SIGINT to the rosbag process group.
    kill -TERM "${recorder_pid}" 2>/dev/null || true
    for _idx in $(seq 1 300); do
        if ! kill -0 "${recorder_pid}" 2>/dev/null; then
            break
        fi
        sleep 0.05
    done
    local recorder_rc=0
    if kill -0 "${recorder_pid}" 2>/dev/null; then
        echo "[${SCRIPT_NAME}] ERROR: recorder did not stop within 15s." >&2
        kill -KILL "${recorder_pid}" 2>/dev/null || true
    fi
    wait "${recorder_pid}" 2>/dev/null || recorder_rc=$?
    recorder_pid=""
    return "${recorder_rc}"
}

cleanup() {
    local status=$?
    trap - INT TERM EXIT
    stop_recorder || true
    exit "${status}"
}

handle_signal() {
    echo
    echo "[${SCRIPT_NAME}] interrupted; finalizing the bag." >&2
    exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

MOCAP_TRACKER="${MOCAP_TRACKER}" \
IMU_TOPIC="${IMU_TOPIC}" \
CMD_TOPIC="${CMD_TOPIC}" \
ODOM_TOPIC="${ODOM_TOPIC}" \
SEGMENT_TOPIC="${SEGMENT_TOPIC}" \
RUN_LABEL="${RUN_LABEL}" \
NAME="${NAME}" \
OUT_DIR="${OUT_DIR}" \
RECORD_SEC=0 \
bash "${RECORDER_SCRIPT}" >"${RECORDER_LOG}" 2>&1 &
recorder_pid=$!

recorder_ready=false
for _idx in $(seq 1 80); do
    if [[ -f "${BAG_ACTIVE_PATH}" ]]; then
        recorder_ready=true
        break
    fi
    if ! kill -0 "${recorder_pid}" 2>/dev/null; then
        echo "[${SCRIPT_NAME}] ERROR: recorder exited during startup." >&2
        tail -n 80 "${RECORDER_LOG}" >&2 || true
        exit 1
    fi
    sleep 0.25
done
if [[ "${recorder_ready}" != "true" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: recorder did not create ${BAG_ACTIVE_PATH}." >&2
    tail -n 80 "${RECORDER_LOG}" >&2 || true
    exit 1
fi

publish_marker() {
    local value="$1"
    if ! timeout 5s python3 - "${SEGMENT_TOPIC}" "${value}" <<'PY'
import sys
import time

import rospy
from std_msgs.msg import String

topic, value = sys.argv[1:3]
rospy.init_node("nokov_imu_latency_marker", anonymous=True, disable_signals=True)
publisher = rospy.Publisher(topic, String, queue_size=1)
deadline = time.monotonic() + 2.0
while publisher.get_num_connections() < 1 and time.monotonic() < deadline:
    time.sleep(0.02)
if publisher.get_num_connections() < 1:
    raise RuntimeError("recorder did not subscribe to {}".format(topic))
publisher.publish(String(data=value))
time.sleep(0.10)
PY
    then
        echo "[${SCRIPT_NAME}] ERROR: failed to publish marker ${value}." >&2
        exit 1
    fi
}

echo "[${SCRIPT_NAME}] recorder ready: ${BAG_PATH}"
publish_marker "NOKOV_IMU_LATENCY_TRIAL_START|id=${LATENCY_TEST_ID}|attempt=${ATTEMPT}"

echo "[${SCRIPT_NAME}] STATIC BASELINE: keep the robot completely still for ${STATIC_PRE_SEC}s."
sleep "${STATIC_PRE_SEC}"

echo -e "\a[${SCRIPT_NAME}] MOTION START: alternate LEFT and RIGHT yaw now."
publish_marker "NOKOV_IMU_LATENCY_MOTION_START|id=${LATENCY_TEST_ID}|attempt=${ATTEMPT}"
for ((remaining=MOTION_SEC; remaining>0; remaining--)); do
    if (( remaining == MOTION_SEC || remaining % 5 == 0 )); then
        echo "[${SCRIPT_NAME}] motion time remaining: ${remaining}s; keep reversing every 1--2s."
    fi
    sleep 1
done
echo -e "\a[${SCRIPT_NAME}] MOTION END: keep the robot still for ${STATIC_POST_SEC}s."
publish_marker "NOKOV_IMU_LATENCY_MOTION_END|id=${LATENCY_TEST_ID}|attempt=${ATTEMPT}"
sleep "${STATIC_POST_SEC}"
publish_marker "NOKOV_IMU_LATENCY_TRIAL_END|id=${LATENCY_TEST_ID}|attempt=${ATTEMPT}"

if ! stop_recorder; then
    echo "[${SCRIPT_NAME}] ERROR: recorder did not finalize cleanly." >&2
    exit 1
fi
trap - INT TERM EXIT

if [[ ! -f "${BAG_PATH}" || -e "${BAG_ACTIVE_PATH}" ]]; then
    echo "[${SCRIPT_NAME}] ERROR: finalized bag is missing or still active." >&2
    exit 1
fi

set +e
python3 "${ANALYZER_SCRIPT}" \
    --bag "${BAG_PATH}" \
    --imu-topic "${IMU_TOPIC}" \
    --mocap-topic "${RAW_MOCAP_POSE_TOPIC}" \
    --segment-topic "${SEGMENT_TOPIC}" \
    --output-json "${REPORT_JSON}" \
    --output-summary "${SUMMARY_MD}" \
    --output-csv "${SIGNALS_CSV}" \
    --output-plot "${PLOT_PATH}" \
    2>&1 | tee "${ANALYSIS_LOG}"
analysis_rc=${PIPESTATUS[0]}
set -e

sha256sum "${BAG_PATH}" >"${SHA256_PATH}"

echo
echo "[${SCRIPT_NAME}] finished."
echo "[${SCRIPT_NAME}] bag: ${BAG_PATH}"
echo "[${SCRIPT_NAME}] summary: ${SUMMARY_MD}"
echo "[${SCRIPT_NAME}] report: ${REPORT_JSON}"
echo "[${SCRIPT_NAME}] plot: ${PLOT_PATH}"
echo "[${SCRIPT_NAME}] SHA-256: ${SHA256_PATH}"
if (( analysis_rc != 0 )); then
    echo "[${SCRIPT_NAME}] WARNING: analysis was inconclusive; inspect ${SUMMARY_MD}." >&2
    exit "${analysis_rc}"
fi
