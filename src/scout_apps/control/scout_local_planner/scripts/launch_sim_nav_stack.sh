#!/usr/bin/env bash
# Launch the Scout Mini simulation navigation stack up to the global planner.

set -euo pipefail

USE_RVIZ="${USE_RVIZ:-false}"
SPAWN_X="${SPAWN_X:--1.0}"
SPAWN_Y="${SPAWN_Y:-0.2}"
SPAWN_Z="${SPAWN_Z:-0.1}"
GAZEBO_WAIT_S="${GAZEBO_WAIT_S:-8}"
SENSOR_WAIT_S="${SENSOR_WAIT_S:-3}"
LOCALIZATION_WAIT_S="${LOCALIZATION_WAIT_S:-5}"
LOCALIZATION_BACKUP_S="${LOCALIZATION_BACKUP_S:-4}"
LOCALIZATION_BACKUP_V="${LOCALIZATION_BACKUP_V:--0.42}"
LOCALIZATION_SPIN_S="${LOCALIZATION_SPIN_S:-10}"
LOCALIZATION_SPIN_OMEGA="${LOCALIZATION_SPIN_OMEGA:-0.45}"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
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
    publish_cmd_for_duration "Backing up to refresh localization" \
        "${LOCALIZATION_BACKUP_S}" "${LOCALIZATION_BACKUP_V}" "0.0"

    publish_cmd_for_duration "Spinning in place to refresh localization" \
        "${LOCALIZATION_SPIN_S}" "0.0" "${LOCALIZATION_SPIN_OMEGA}"

    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
}

echo "[launch_sim_nav_stack] USE_RVIZ=${USE_RVIZ}"
echo "[launch_sim_nav_stack] spawn pose: x=${SPAWN_X}, y=${SPAWN_Y}, z=${SPAWN_Z}"

start_launch "Gazebo Scout Mini" \
    scout_description scout_mini_gazebo.launch \
    use_rviz:="${USE_RVIZ}" \
    x:="${SPAWN_X}" \
    y:="${SPAWN_Y}" \
    z:="${SPAWN_Z}"
sleep "${GAZEBO_WAIT_S}"

start_launch "NanoScan3 front sim" \
    nanoscan3_bringup nanoscan3_front_sim.launch \
    use_rviz:="${USE_RVIZ}"
sleep "${SENSOR_WAIT_S}"

start_launch "Cartographer localization sim" \
    nanoscan3_localization scout_nanoscan3_cartographer_localization_sim.launch
sleep "${LOCALIZATION_WAIT_S}"

start_launch "MBF global planner sim" \
    scout_global_planner mbf_global_sim.launch

refresh_localization_motion

echo "[launch_sim_nav_stack] Simulation navigation stack started."
echo "[launch_sim_nav_stack] Start the local planner separately, for example:"
echo "  roslaunch scout_local_planner slosh_experiment.launch sim:=true Q_slosh:=5 enable_slosh_box_constraint:=true risk_scheduler_enable:=true"
echo "[launch_sim_nav_stack] Press Ctrl+C to stop all launched processes."

wait -n "${pids[@]}"
echo "[launch_sim_nav_stack] One launch process exited; shutting down the rest."
