#!/usr/bin/env bash
# Launch the real-robot sensor/localization stack and check camera/IMU rates.

set -euo pipefail

SCRIPT_NAME="launch_real_sensors_stack"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${SCOUT_WS:-$(readlink -f "${SCRIPT_DIR}/../../../../..")}"
LOG_DIR="${LOG_DIR:-/tmp/${SCRIPT_NAME}_$(date +%Y%m%d_%H%M%S)}"
START_DELAY="${START_DELAY:-6}"
POST_TOPIC_DELAY="${POST_TOPIC_DELAY:-3}"
SETTLE_DELAY="${SETTLE_DELAY:-12}"
READY_TIMEOUT="${READY_TIMEOUT:-45}"
START_BASE="${START_BASE:-true}"
WAIT_FOR_ODOM="${WAIT_FOR_ODOM:-true}"
WAIT_FOR_LOCALIZATION_MAP="${WAIT_FOR_LOCALIZATION_MAP:-false}"
HZ_WINDOW="${HZ_WINDOW:-10}"
REALSENSE_COLOR_WIDTH="${REALSENSE_COLOR_WIDTH:-1920}"
REALSENSE_COLOR_HEIGHT="${REALSENSE_COLOR_HEIGHT:-1080}"
REALSENSE_COLOR_FPS="${REALSENSE_COLOR_FPS:-30}"
REALSENSE_ENABLE_DEPTH="${REALSENSE_ENABLE_DEPTH:-false}"
REALSENSE_ENABLE_INFRA="${REALSENSE_ENABLE_INFRA:-false}"
REALSENSE_ENABLE_INFRA1="${REALSENSE_ENABLE_INFRA1:-false}"
REALSENSE_ENABLE_INFRA2="${REALSENSE_ENABLE_INFRA2:-false}"

pids=()
names=()

cleanup() {
    local status=$?
    trap - INT TERM EXIT

    if (( ${#pids[@]} > 0 )); then
        echo
        echo "[${SCRIPT_NAME}] Stopping launched processes..."
        for pid in "${pids[@]}"; do
            if kill -0 "${pid}" 2>/dev/null; then
                kill "${pid}" 2>/dev/null || true
            fi
        done
        wait || true
    fi

    exit "${status}"
}

trap cleanup INT TERM EXIT

source_ros() {
    if [[ -f /opt/ros/noetic/setup.bash ]]; then
        # shellcheck disable=SC1091
        source /opt/ros/noetic/setup.bash
    fi

    if [[ -f "${WS_ROOT}/devel/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${WS_ROOT}/devel/setup.bash"
    else
        echo "[${SCRIPT_NAME}] ERROR: ${WS_ROOT}/devel/setup.bash not found." >&2
        echo "[${SCRIPT_NAME}] Set SCOUT_WS=/path/to/scout_ws if needed." >&2
        exit 1
    fi
}

setup_can0() {
    echo "[${SCRIPT_NAME}] Requesting sudo once for CAN setup..."
    sudo -v

    if ! ip link show can0 >/dev/null 2>&1; then
        echo "[${SCRIPT_NAME}] ERROR: can0 does not exist." >&2
        exit 1
    fi

    echo "[${SCRIPT_NAME}] Setting can0 up at 500000 bitrate..."
    if sudo ip link set can0 up type can bitrate 500000; then
        return
    fi

    if ip -details link show can0 | grep -q "state UP"; then
        echo "[${SCRIPT_NAME}] WARN: can0 setup command failed, but can0 is already UP; continuing."
        return
    fi

    echo "[${SCRIPT_NAME}] ERROR: failed to bring can0 up." >&2
    exit 1
}

start_launch() {
    local name="$1"
    shift

    local log_file="${LOG_DIR}/${name}.log"
    echo "[${SCRIPT_NAME}] Starting ${name}: roslaunch $*"
    roslaunch "$@" >"${log_file}" 2>&1 &
    local pid=$!
    pids+=("${pid}")
    names+=("${name}")

    sleep "${START_DELAY}"
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "[${SCRIPT_NAME}] ERROR: ${name} exited during startup. Log: ${log_file}" >&2
        wait "${pid}" || true
        exit 1
    fi
}

wait_for_topic() {
    local topic="$1"
    local label="${2:-${topic}}"
    local required="${3:-true}"
    local safe_name="${topic#/}"
    safe_name="${safe_name//\//_}"
    local log_file="${LOG_DIR}/wait_${safe_name}.log"

    echo "[${SCRIPT_NAME}] Waiting for ${label} (${topic}), timeout=${READY_TIMEOUT}s..."
    # Readiness only needs one delivered message. Avoid expanding large arrays
    # such as 1920x1080 RGB data, which can outlive the timeout while formatting.
    if timeout "${READY_TIMEOUT}" rostopic echo --noarr -n 1 "${topic}" >"${log_file}" 2>&1; then
        echo "[${SCRIPT_NAME}] ${label} is publishing."
        sleep "${POST_TOPIC_DELAY}"
        return 0
    fi

    if [[ "${required}" == "true" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: timed out waiting for ${label} (${topic}). Log: ${log_file}" >&2
        echo "[${SCRIPT_NAME}] Recent launch logs:" >&2
        for idx in "${!pids[@]}"; do
            local name="${names[${idx}]}"
            local launch_log="${LOG_DIR}/${name}.log"
            if [[ -f "${launch_log}" ]]; then
                echo "----- ${name} (${launch_log}) -----" >&2
                tail -n 30 "${launch_log}" >&2 || true
            fi
        done
        exit 1
    fi

    echo "[${SCRIPT_NAME}] WARN: timed out waiting for optional ${label} (${topic}); continuing."
    return 0
}

check_hz() {
    local topic="$1"
    local safe_name="${topic#/}"
    safe_name="${safe_name//\//_}"
    local log_file="${LOG_DIR}/hz_${safe_name}.log"

    echo
    echo "[${SCRIPT_NAME}] Checking ${topic} for ${HZ_WINDOW}s..."
    timeout "${HZ_WINDOW}" rostopic hz "${topic}" 2>&1 | tee "${log_file}" || true
}

check_camera_info() {
    local topic="/camera/color/camera_info"
    local log_file="${LOG_DIR}/camera_color_camera_info.log"

    echo
    echo "[${SCRIPT_NAME}] Checking RealSense color camera_info (${topic})..."
    if ! timeout "${READY_TIMEOUT}" rostopic echo -n 1 "${topic}" >"${log_file}" 2>&1; then
        echo "[${SCRIPT_NAME}] WARN: failed to read ${topic}. Log: ${log_file}"
        return 0
    fi

    local width
    local height
    width="$(awk '/^width:/ {print $2; exit}' "${log_file}")"
    height="$(awk '/^height:/ {print $2; exit}' "${log_file}")"
    echo "[${SCRIPT_NAME}] RealSense color camera_info: width=${width:-unknown} height=${height:-unknown}"

    if [[ "${width:-}" != "${REALSENSE_COLOR_WIDTH}" || "${height:-}" != "${REALSENSE_COLOR_HEIGHT}" ]]; then
        echo "[${SCRIPT_NAME}] WARN: expected ${REALSENSE_COLOR_WIDTH}x${REALSENSE_COLOR_HEIGHT}, got ${width:-unknown}x${height:-unknown}."
    fi
}

mkdir -p "${LOG_DIR}"
source_ros

echo "[${SCRIPT_NAME}] Workspace: ${WS_ROOT}"
echo "[${SCRIPT_NAME}] Logs: ${LOG_DIR}"
echo "[${SCRIPT_NAME}] START_DELAY=${START_DELAY}s POST_TOPIC_DELAY=${POST_TOPIC_DELAY}s SETTLE_DELAY=${SETTLE_DELAY}s READY_TIMEOUT=${READY_TIMEOUT}s"
echo "[${SCRIPT_NAME}] START_BASE=${START_BASE} WAIT_FOR_ODOM=${WAIT_FOR_ODOM} WAIT_FOR_LOCALIZATION_MAP=${WAIT_FOR_LOCALIZATION_MAP}"
echo "[${SCRIPT_NAME}] RealSense color=${REALSENSE_COLOR_WIDTH}x${REALSENSE_COLOR_HEIGHT}@${REALSENSE_COLOR_FPS}Hz depth=${REALSENSE_ENABLE_DEPTH} infra=${REALSENSE_ENABLE_INFRA}/${REALSENSE_ENABLE_INFRA1}/${REALSENSE_ENABLE_INFRA2}"

setup_can0

if [[ "${START_BASE}" == "true" ]]; then
    start_launch "scout_mini_robot_base" \
        scout_bringup scout_mini_robot_base.launch
elif [[ "${WAIT_FOR_ODOM}" == "true" ]]; then
    echo "[${SCRIPT_NAME}] START_BASE=false; assuming robot base was launched externally."
fi
if [[ "${WAIT_FOR_ODOM}" == "true" ]]; then
    wait_for_topic /odom "Scout base odometry"
fi

start_launch "nanoscan3_front" \
    nanoscan3_bringup nanoscan3_front.launch use_rviz:=false
wait_for_topic /scan_front "front LiDAR scan"

start_launch "nanoscan3_localization" \
    nanoscan3_localization scout_nanoscan3_cartographer_localization.launch
if [[ "${WAIT_FOR_LOCALIZATION_MAP}" == "true" ]]; then
    wait_for_topic /map "Cartographer occupancy grid" false
else
    echo "[${SCRIPT_NAME}] Skipping /map wait. Set WAIT_FOR_LOCALIZATION_MAP=true to require localization map readiness."
fi

start_launch "scout_imu_with_tf" \
    scout_bringup scout_imu_with_tf.launch
wait_for_topic /imu/data "IMU data"

start_launch "realsense2_camera" \
    realsense2_camera rs_camera.launch \
    enable_color:=true \
    color_width:="${REALSENSE_COLOR_WIDTH}" \
    color_height:="${REALSENSE_COLOR_HEIGHT}" \
    color_fps:="${REALSENSE_COLOR_FPS}" \
    enable_depth:="${REALSENSE_ENABLE_DEPTH}" \
    enable_infra:="${REALSENSE_ENABLE_INFRA}" \
    enable_infra1:="${REALSENSE_ENABLE_INFRA1}" \
    enable_infra2:="${REALSENSE_ENABLE_INFRA2}"
wait_for_topic /camera/color/image_raw "RealSense color image"
check_camera_info

echo
echo "[${SCRIPT_NAME}] Waiting ${SETTLE_DELAY}s before hz checks..."
sleep "${SETTLE_DELAY}"

check_hz /camera/color/image_raw
check_hz /imu/data

echo
echo "[${SCRIPT_NAME}] Stack is running. Press Ctrl+C to stop all launched processes."
wait -n "${pids[@]}"
echo "[${SCRIPT_NAME}] One launch process exited; shutting down the rest."
