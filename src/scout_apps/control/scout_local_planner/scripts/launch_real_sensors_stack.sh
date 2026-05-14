#!/usr/bin/env bash
# Launch the real-robot sensor/localization stack and check camera/IMU rates.

set -euo pipefail

SCRIPT_NAME="launch_real_sensors_stack"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${SCOUT_WS:-$(readlink -f "${SCRIPT_DIR}/../../../../..")}"
LOG_DIR="${LOG_DIR:-/tmp/${SCRIPT_NAME}_$(date +%Y%m%d_%H%M%S)}"
START_DELAY="${START_DELAY:-4}"
SETTLE_DELAY="${SETTLE_DELAY:-8}"
HZ_WINDOW="${HZ_WINDOW:-10}"

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

check_hz() {
    local topic="$1"
    local safe_name="${topic#/}"
    safe_name="${safe_name//\//_}"
    local log_file="${LOG_DIR}/hz_${safe_name}.log"

    echo
    echo "[${SCRIPT_NAME}] Checking ${topic} for ${HZ_WINDOW}s..."
    timeout "${HZ_WINDOW}" rostopic hz "${topic}" 2>&1 | tee "${log_file}" || true
}

mkdir -p "${LOG_DIR}"
source_ros

echo "[${SCRIPT_NAME}] Workspace: ${WS_ROOT}"
echo "[${SCRIPT_NAME}] Logs: ${LOG_DIR}"

setup_can0

start_launch "nanoscan3_front" \
    nanoscan3_bringup nanoscan3_front.launch use_rviz:=false

start_launch "nanoscan3_localization" \
    nanoscan3_localization scout_nanoscan3_cartographer_localization.launch

start_launch "scout_imu_with_tf" \
    scout_bringup scout_imu_with_tf.launch

start_launch "realsense2_camera" \
    realsense2_camera rs_camera.launch

echo
echo "[${SCRIPT_NAME}] Waiting ${SETTLE_DELAY}s before hz checks..."
sleep "${SETTLE_DELAY}"

check_hz /camera/color/image_raw
check_hz /imu/data

echo
echo "[${SCRIPT_NAME}] Stack is running. Press Ctrl+C to stop all launched processes."
wait -n "${pids[@]}"
echo "[${SCRIPT_NAME}] One launch process exited; shutting down the rest."
