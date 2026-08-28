#!/usr/bin/env bash
# Launch the real-robot sensor stack, optionally start localization, and check
# camera/IMU rates.

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
START_LOCALIZATION="${START_LOCALIZATION:-true}"
WAIT_FOR_LOCALIZATION_MAP="${WAIT_FOR_LOCALIZATION_MAP:-false}"
WAIT_FOR_LOCALIZATION_TF="${WAIT_FOR_LOCALIZATION_TF:-false}"
LOCALIZATION_GLOBAL_FRAME="${LOCALIZATION_GLOBAL_FRAME:-map}"
LOCALIZATION_BASE_FRAME="${LOCALIZATION_BASE_FRAME:-base_link}"
LOCALIZATION_MAP_FILE="${LOCALIZATION_MAP_FILE:-}"
LOCALIZATION_MAP_EXPECTED_SHA256="${LOCALIZATION_MAP_EXPECTED_SHA256:-}"
REQUIRE_LOCALIZATION_MAP_HASH="${REQUIRE_LOCALIZATION_MAP_HASH:-}"
LOCALIZATION_OCCUPANCY_GRID_RESOLUTION="${LOCALIZATION_OCCUPANCY_GRID_RESOLUTION:-0.02}"
LOCALIZATION_USE_RVIZ="${LOCALIZATION_USE_RVIZ:-true}"
START_IMU="${START_IMU:-true}"
START_REALSENSE="${START_REALSENSE:-true}"
CARTOGRAPHER_SETUP="${CARTOGRAPHER_SETUP:-${WS_ROOT}/src/scout_apps/sensors/cartographer_ws/install_isolated/setup.bash}"
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
    export ROS_DISTRO="${ROS_DISTRO:-noetic}"
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

    if [[ "${START_LOCALIZATION}" == "true" ]]; then
        if [[ ! -f "${CARTOGRAPHER_SETUP}" ]]; then
            echo "[${SCRIPT_NAME}] ERROR: Cartographer setup not found: ${CARTOGRAPHER_SETUP}" >&2
            exit 1
        fi
        export CATKIN_SETUP_UTIL_ARGS="${CATKIN_SETUP_UTIL_ARGS:---extend}"
        # shellcheck disable=SC1090
        source "${CARTOGRAPHER_SETUP}"

        if ! rospack find cartographer_ros >/dev/null 2>&1; then
            echo "[${SCRIPT_NAME}] ERROR: cartographer_ros is not visible after sourcing ${CARTOGRAPHER_SETUP}." >&2
            exit 1
        fi
        if ! rospack find nanoscan3_localization >/dev/null 2>&1; then
            echo "[${SCRIPT_NAME}] ERROR: nanoscan3_localization is not visible after environment setup." >&2
            exit 1
        fi
    fi
}

is_true() {
    case "${1:-}" in
        1|true|TRUE|yes|YES|on|ON) return 0 ;;
        *) return 1 ;;
    esac
}

prepare_localization_map() {
    local explicit_map="false"
    local map_path="${LOCALIZATION_MAP_FILE}"

    if [[ -n "${map_path}" ]]; then
        explicit_map="true"
    else
        map_path="$(rospack find scout_maps)/maps/map_carto_20260629_R0.pbstream"
    fi

    [[ "${map_path}" == *.pbstream ]] || {
        echo "[${SCRIPT_NAME}] ERROR: localization map must be a .pbstream: ${map_path}" >&2
        exit 1
    }
    [[ -s "${map_path}" ]] || {
        echo "[${SCRIPT_NAME}] ERROR: localization map is missing or empty: ${map_path}" >&2
        exit 1
    }
    LOCALIZATION_MAP_FILE="$(readlink -f "${map_path}")"
    LOCALIZATION_MAP_ACTUAL_SHA256="$(sha256sum "${LOCALIZATION_MAP_FILE}" | awk '{print $1}')"

    if [[ -z "${REQUIRE_LOCALIZATION_MAP_HASH}" ]]; then
        if [[ "${explicit_map}" == "true" ]]; then
            REQUIRE_LOCALIZATION_MAP_HASH="true"
        else
            REQUIRE_LOCALIZATION_MAP_HASH="false"
        fi
    fi

    if [[ -n "${LOCALIZATION_MAP_EXPECTED_SHA256}" ]]; then
        if [[ ! "${LOCALIZATION_MAP_EXPECTED_SHA256}" =~ ^[0-9a-fA-F]{64}$ ]]; then
            echo "[${SCRIPT_NAME}] ERROR: LOCALIZATION_MAP_EXPECTED_SHA256 must contain 64 hexadecimal characters." >&2
            exit 1
        fi
        LOCALIZATION_MAP_EXPECTED_SHA256="${LOCALIZATION_MAP_EXPECTED_SHA256,,}"
        if [[ "${LOCALIZATION_MAP_ACTUAL_SHA256}" != "${LOCALIZATION_MAP_EXPECTED_SHA256}" ]]; then
            echo "[${SCRIPT_NAME}] ERROR: localization map SHA-256 mismatch." >&2
            echo "[${SCRIPT_NAME}] expected=${LOCALIZATION_MAP_EXPECTED_SHA256}" >&2
            echo "[${SCRIPT_NAME}] actual=${LOCALIZATION_MAP_ACTUAL_SHA256}" >&2
            exit 1
        fi
    elif is_true "${REQUIRE_LOCALIZATION_MAP_HASH}"; then
        echo "[${SCRIPT_NAME}] ERROR: map hash is required; set LOCALIZATION_MAP_EXPECTED_SHA256." >&2
        exit 1
    else
        echo "[${SCRIPT_NAME}] WARN: localization map is not bound to an operator-supplied SHA-256."
    fi

    if [[ "${explicit_map}" == "true" ]]; then
        WAIT_FOR_LOCALIZATION_MAP="true"
        WAIT_FOR_LOCALIZATION_TF="true"
    fi
    if [[ "${explicit_map}" == "true" ]]; then
        LOCALIZATION_MAP_MODE="explicit"
    else
        LOCALIZATION_MAP_MODE="legacy_default"
    fi
}

write_map_identity() {
    local output="${LOG_DIR}/localization_map_identity.env"
    local git_commit
    local git_dirty
    git_commit="$(git -C "${WS_ROOT}" rev-parse HEAD 2>/dev/null || printf unknown)"
    if [[ -n "$(git -C "${WS_ROOT}" status --porcelain --untracked-files=normal 2>/dev/null || true)" ]]; then
        git_dirty="true"
    else
        git_dirty="false"
    fi
    {
        printf 'schema_version=%q\n' '1'
        printf 'map_mode=%q\n' "${LOCALIZATION_MAP_MODE}"
        printf 'map_file=%q\n' "${LOCALIZATION_MAP_FILE}"
        printf 'map_expected_sha256=%q\n' "${LOCALIZATION_MAP_EXPECTED_SHA256}"
        printf 'map_actual_sha256=%q\n' "${LOCALIZATION_MAP_ACTUAL_SHA256}"
        printf 'map_hash_required=%q\n' "${REQUIRE_LOCALIZATION_MAP_HASH}"
        printf 'occupancy_grid_resolution=%q\n' "${LOCALIZATION_OCCUPANCY_GRID_RESOLUTION}"
        printf 'global_frame=%q\n' "${LOCALIZATION_GLOBAL_FRAME}"
        printf 'base_frame=%q\n' "${LOCALIZATION_BASE_FRAME}"
        printf 'git_commit=%q\n' "${git_commit}"
        printf 'git_dirty=%q\n' "${git_dirty}"
        printf 'created_at=%q\n' "$(date --iso-8601=seconds)"
    } > "${output}"
    echo "[${SCRIPT_NAME}] Map identity: ${output}"
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

wait_for_tf() {
    local parent_frame="$1"
    local child_frame="$2"
    local required="${3:-true}"
    local log_file="${LOG_DIR}/wait_tf_${parent_frame//\//_}_to_${child_frame//\//_}.log"

    echo "[${SCRIPT_NAME}] Waiting for TF ${parent_frame}->${child_frame}, timeout=${READY_TIMEOUT}s..."
    if timeout "${READY_TIMEOUT}" python3 - "${parent_frame}" "${child_frame}" >"${log_file}" 2>&1 <<'PY'
import sys
import time

import rospy
import tf2_ros

parent_frame, child_frame = sys.argv[1:3]
rospy.init_node("wait_for_localization_tf", anonymous=True, disable_signals=True)
buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
listener = tf2_ros.TransformListener(buffer)
deadline = time.monotonic() + 3600.0
while not rospy.is_shutdown() and time.monotonic() < deadline:
    try:
        transform = buffer.lookup_transform(
            parent_frame, child_frame, rospy.Time(0), rospy.Duration(0.2)
        )
        value = transform.transform.translation
        print("translation: {:.6f} {:.6f} {:.6f}".format(value.x, value.y, value.z))
        sys.exit(0)
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        time.sleep(0.1)
sys.exit(2)
PY
    then
        echo "[${SCRIPT_NAME}] TF ${parent_frame}->${child_frame} is available."
        return 0
    fi

    if [[ "${required}" == "true" ]]; then
        echo "[${SCRIPT_NAME}] ERROR: timed out waiting for TF ${parent_frame}->${child_frame}. Log: ${log_file}" >&2
        exit 1
    fi
    echo "[${SCRIPT_NAME}] WARN: optional TF ${parent_frame}->${child_frame} is unavailable."
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
if [[ "${START_LOCALIZATION}" == "true" ]]; then
    prepare_localization_map
    write_map_identity
fi

echo "[${SCRIPT_NAME}] Workspace: ${WS_ROOT}"
echo "[${SCRIPT_NAME}] Logs: ${LOG_DIR}"
echo "[${SCRIPT_NAME}] START_DELAY=${START_DELAY}s POST_TOPIC_DELAY=${POST_TOPIC_DELAY}s SETTLE_DELAY=${SETTLE_DELAY}s READY_TIMEOUT=${READY_TIMEOUT}s"
echo "[${SCRIPT_NAME}] START_BASE=${START_BASE} START_IMU=${START_IMU} START_REALSENSE=${START_REALSENSE} WAIT_FOR_ODOM=${WAIT_FOR_ODOM}"
echo "[${SCRIPT_NAME}] START_LOCALIZATION=${START_LOCALIZATION} WAIT_FOR_LOCALIZATION_MAP=${WAIT_FOR_LOCALIZATION_MAP} WAIT_FOR_LOCALIZATION_TF=${WAIT_FOR_LOCALIZATION_TF}"
if [[ "${START_LOCALIZATION}" == "true" ]]; then
    echo "[${SCRIPT_NAME}] localization map=${LOCALIZATION_MAP_FILE} sha256=${LOCALIZATION_MAP_ACTUAL_SHA256}"
fi
echo "[${SCRIPT_NAME}] RealSense color=${REALSENSE_COLOR_WIDTH}x${REALSENSE_COLOR_HEIGHT}@${REALSENSE_COLOR_FPS}Hz depth=${REALSENSE_ENABLE_DEPTH} infra=${REALSENSE_ENABLE_INFRA}/${REALSENSE_ENABLE_INFRA1}/${REALSENSE_ENABLE_INFRA2}"

if [[ "${START_BASE}" == "true" ]]; then
    setup_can0
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

if [[ "${START_LOCALIZATION}" == "true" ]]; then
    start_launch "nanoscan3_localization" \
        nanoscan3_localization scout_nanoscan3_cartographer_localization.launch \
        map_file:="${LOCALIZATION_MAP_FILE}" \
        map_expected_sha256:="${LOCALIZATION_MAP_EXPECTED_SHA256}" \
        occupancy_grid_resolution:="${LOCALIZATION_OCCUPANCY_GRID_RESOLUTION}" \
        use_rviz:="${LOCALIZATION_USE_RVIZ}"
    if [[ "${WAIT_FOR_LOCALIZATION_MAP}" == "true" ]]; then
        wait_for_topic /map "Cartographer occupancy grid" true
    else
        echo "[${SCRIPT_NAME}] Skipping /map wait. Set WAIT_FOR_LOCALIZATION_MAP=true to require localization map readiness."
    fi
    if [[ "${WAIT_FOR_LOCALIZATION_TF}" == "true" ]]; then
        wait_for_tf "${LOCALIZATION_GLOBAL_FRAME}" "${LOCALIZATION_BASE_FRAME}" true
    else
        echo "[${SCRIPT_NAME}] Skipping localization TF wait. Set WAIT_FOR_LOCALIZATION_TF=true to require it."
    fi
else
    echo "[${SCRIPT_NAME}] Localization is disabled; no Cartographer/AMCL/map server will be started."
fi

if [[ "${START_IMU}" == "true" ]]; then
    start_launch "scout_imu_with_tf" \
        scout_bringup scout_imu_with_tf.launch
    wait_for_topic /imu/data "IMU data"
else
    echo "[${SCRIPT_NAME}] START_IMU=false; skipping IMU launch and checks."
fi

if [[ "${START_REALSENSE}" == "true" ]]; then
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
else
    echo "[${SCRIPT_NAME}] START_REALSENSE=false; skipping RealSense launch and checks."
fi

echo
echo "[${SCRIPT_NAME}] Waiting ${SETTLE_DELAY}s before hz checks..."
sleep "${SETTLE_DELAY}"

if [[ "${START_REALSENSE}" == "true" ]]; then
    check_hz /camera/color/image_raw
fi
if [[ "${START_IMU}" == "true" ]]; then
    check_hz /imu/data
fi

echo
echo "[${SCRIPT_NAME}] Stack is running. Press Ctrl+C to stop all launched processes."
wait -n "${pids[@]}"
echo "[${SCRIPT_NAME}] One launch process exited; shutting down the rest."
