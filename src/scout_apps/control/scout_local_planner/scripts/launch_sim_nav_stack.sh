#!/usr/bin/env bash
# Launch the Scout Mini simulation navigation stack up to the global planner.

set -euo pipefail

USE_RVIZ="${USE_RVIZ:-false}"
SIM_ENV="${SIM_ENV:-open}"

case "${SIM_ENV}" in
    open)
        DEFAULT_WORLD_NAME="/home/a/scout_ws/src/scout_ros/scout_description/worlds/open_walled.world"
        DEFAULT_MAP_FILE="/home/a/scout_ws/src/scout_apps/scout_maps/maps/map_sim_empty.pbstream"
        ;;
    maze)
        DEFAULT_WORLD_NAME="/home/a/scout_ws/src/scout_ros/scout_description/worlds/maze_course.world"
        DEFAULT_MAP_FILE="/home/a/scout_ws/src/scout_apps/scout_maps/maps/map_carto.pbstream"
        ;;
    custom)
        DEFAULT_WORLD_NAME=""
        DEFAULT_MAP_FILE=""
        ;;
    *)
        echo "[launch_sim_nav_stack] ERROR: unsupported SIM_ENV='${SIM_ENV}' (use open, maze, or custom)." >&2
        exit 2
        ;;
esac

WORLD_NAME="${WORLD_NAME:-${DEFAULT_WORLD_NAME}}"
MAP_FILE="${MAP_FILE:-${DEFAULT_MAP_FILE}}"
if [[ -z "${WORLD_NAME}" || -z "${MAP_FILE}" ]]; then
    echo "[launch_sim_nav_stack] ERROR: SIM_ENV=custom requires WORLD_NAME and MAP_FILE." >&2
    exit 2
fi
SPAWN_X="${SPAWN_X:--1.0}"
SPAWN_Y="${SPAWN_Y:-0.2}"
SPAWN_Z="${SPAWN_Z:-0.1}"
SPAWN_YAW="${SPAWN_YAW:-0.0}"
SPAWN_WHEEL_CONTROLLERS="${SPAWN_WHEEL_CONTROLLERS:-false}"
ENABLE_ODOM_TF_BRIDGE="${ENABLE_ODOM_TF_BRIDGE:-false}"
GAZEBO_WAIT_S="${GAZEBO_WAIT_S:-8}"
SENSOR_WAIT_S="${SENSOR_WAIT_S:-3}"
LOCALIZATION_WAIT_S="${LOCALIZATION_WAIT_S:-3}"
LOCALIZATION_BACKUP_S="${LOCALIZATION_BACKUP_S:-4}"
LOCALIZATION_BACKUP_V="${LOCALIZATION_BACKUP_V:--0.12}"
LOCALIZATION_SPIN_S="${LOCALIZATION_SPIN_S:-2}"
LOCALIZATION_SPIN_OMEGA="${LOCALIZATION_SPIN_OMEGA:-0.8}"
OPEN_LOCALIZATION_FORWARD_S="${OPEN_LOCALIZATION_FORWARD_S:-3}"
OPEN_LOCALIZATION_FORWARD_V="${OPEN_LOCALIZATION_FORWARD_V:-0.15}"
OPEN_LOCALIZATION_TURN_S="${OPEN_LOCALIZATION_TURN_S:-1}"
OPEN_LOCALIZATION_TURN_OMEGA="${OPEN_LOCALIZATION_TURN_OMEGA:-0.5}"


if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi

if [[ -f /data/a/official_scout_ws/devel_isolated/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /data/a/official_scout_ws/devel_isolated/setup.bash
fi

if [[ -f /home/a/scout_ws/devel/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/a/scout_ws/devel/setup.bash
fi

pids=()

cleanup() {
    local status=$?
    trap - INT TERM EXIT

    if (( ${#pids[@]} > 0 )); then
        echo
        echo "[launch_sim_nav_stack] Stopping launched processes..."
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

start_launch() {
    local name="$1"
    shift

    echo "[launch_sim_nav_stack] Starting ${name}: roslaunch $*"
    roslaunch "$@" &
    local pid=$!
    pids+=("${pid}")

    sleep 2
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "[launch_sim_nav_stack] ERROR: ${name} exited during startup." >&2
        wait "${pid}" || true
        exit 1
    fi
}

publish_cmd_for_duration() {
    local label="$1"
    local duration_s="$2"
    local linear_x="$3"
    local angular_z="$4"

    if [[ "${duration_s}" == "0" || "${duration_s}" == "0.0" ]]; then
        return
    fi

    echo "[launch_sim_nav_stack] ${label} for ${duration_s}s: v=${linear_x}, omega=${angular_z}"
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
        "{linear: {x: ${linear_x}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${angular_z}}}" &
    local pub_pid=$!

    sleep "${duration_s}"
    if kill -0 "${pub_pid}" 2>/dev/null; then
        kill "${pub_pid}" 2>/dev/null || true
        wait "${pub_pid}" 2>/dev/null || true
    fi
}

refresh_localization_motion() {
    if [[ "${SIM_ENV}" == "open" ]]; then
        publish_cmd_for_duration "Moving forward to refresh open-world localization" \
            "${OPEN_LOCALIZATION_FORWARD_S}" "${OPEN_LOCALIZATION_FORWARD_V}" "0.0"
        publish_cmd_for_duration "Turning clockwise to refresh open-world localization" \
            "${OPEN_LOCALIZATION_TURN_S}" "0.0" "-${OPEN_LOCALIZATION_TURN_OMEGA}"
        publish_cmd_for_duration "Turning counter-clockwise to refresh open-world localization" \
            "${OPEN_LOCALIZATION_TURN_S}" "0.0" "${OPEN_LOCALIZATION_TURN_OMEGA}"
        rostopic pub -1 /cmd_vel geometry_msgs/Twist \
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
        return
    fi

    publish_cmd_for_duration "Backing up to refresh localization" \
        "${LOCALIZATION_BACKUP_S}" "${LOCALIZATION_BACKUP_V}" "0.0"
    publish_cmd_for_duration "Turning counter-clockwise to refresh localization" \
        "${LOCALIZATION_SPIN_S}" "0.0" "${LOCALIZATION_SPIN_OMEGA}"
    publish_cmd_for_duration "Turning clockwise to refresh localization" \
        "${LOCALIZATION_SPIN_S}" "0.0" "-${LOCALIZATION_SPIN_OMEGA}"

    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
}

echo "[launch_sim_nav_stack] USE_RVIZ=${USE_RVIZ}"
echo "[launch_sim_nav_stack] SIM_ENV=${SIM_ENV}"
echo "[launch_sim_nav_stack] world: ${WORLD_NAME}"
echo "[launch_sim_nav_stack] map: ${MAP_FILE}"
echo "[launch_sim_nav_stack] spawn pose: x=${SPAWN_X}, y=${SPAWN_Y}, z=${SPAWN_Z}, yaw=${SPAWN_YAW}"
echo "[launch_sim_nav_stack] spawn_wheel_controllers=${SPAWN_WHEEL_CONTROLLERS}"
echo "[launch_sim_nav_stack] enable_odom_tf_bridge=${ENABLE_ODOM_TF_BRIDGE}"

start_launch "Official Scout Mini bridge" \
    /home/a/scout_ws/src/scout_ros/scout_description/launch/scout_mini_true_empty_bridge.launch \
    gui:="${USE_RVIZ}" \
    enable_odom_tf_bridge:="${ENABLE_ODOM_TF_BRIDGE}" \
    world_name:="${WORLD_NAME}" \
    x:="${SPAWN_X}" \
    y:="${SPAWN_Y}" \
    z:="${SPAWN_Z}" \
    yaw:="${SPAWN_YAW}" \
    spawn_wheel_controllers:="${SPAWN_WHEEL_CONTROLLERS}"
sleep "${GAZEBO_WAIT_S}"

start_launch "Cartographer localization sim" \
    nanoscan3_localization scout_nanoscan3_cartographer_localization_sim.launch \
    map_file:="${MAP_FILE}"
sleep "${LOCALIZATION_WAIT_S}"

start_launch "MBF global planner sim" \
    scout_global_planner mbf_global_sim.launch

refresh_localization_motion

echo "[launch_sim_nav_stack] Simulation navigation stack started."
echo "[launch_sim_nav_stack] Start the local planner separately, for example:"
echo "  roslaunch scout_local_planner slosh_experiment_sim.launch Q_slosh:=5 risk_scheduler_enable:=true"
echo "[launch_sim_nav_stack] Press Ctrl+C to stop all launched processes."

wait -n "${pids[@]}"
echo "[launch_sim_nav_stack] One launch process exited; shutting down the rest."
