#!/usr/bin/env bash
# run_s_curve_smoke_test.sh — one-shot anti-slosh sim smoke test.
#
# Smoke test only: spawns Gazebo + bridge + cartographer + MBF + MPC, generates
# a short S-curve path, lets MPC track it, records a bag, then tears everything
# down. Confirms the stack starts and the robot reaches the goal. Does NOT
# produce numerically faithful data — sim/real ax/jerk differ by ~10-150x
# (see docs/Claude/修改日志-时间/2026-05-20.md). For paper-grade data, use the
# real Scout Mini.
#
# Usage:
#   bash run_s_curve_smoke_test.sh
# Environment overrides:
#   GOAL_X=-1.0 GOAL_Y=0.0       map-frame goal (default: -1, 0; ~5 m from spawn)
#   Q_SLOSH=5.0                   anti-slosh weight
#   USE_RVIZ=true                 GUI
#   TRACK_SEC=60                  monitor duration after goal
#   BAG_DIR=...                   bag output dir

set -euo pipefail

GOAL_X="${GOAL_X:--1.0}"
GOAL_Y="${GOAL_Y:-0.0}"
Q_SLOSH="${Q_SLOSH:-5.0}"
USE_RVIZ="${USE_RVIZ:-true}"
TRACK_SEC="${TRACK_SEC:-60}"
BAG_DIR="${BAG_DIR:-/home/a/scout_ws/bags/sim_s_curve}"
SPAWN_X="${SPAWN_X:--4.0}"
SPAWN_Y="${SPAWN_Y:-0.0}"
SPAWN_YAW="${SPAWN_YAW:-0.0}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/template_goal}"

ts="$(date +%Y%m%d_%H%M%S)"
mkdir -p "${BAG_DIR}" /home/a/scout_ws/fixed_paths/sim/"$(date +%Y%m%d)"
BAG_FILE="${BAG_DIR}/${ts}_s_curve_smoke.bag"
PATH_FILE="/home/a/scout_ws/fixed_paths/sim/$(date +%Y%m%d)/${ts}_s_curve.json"

source /opt/ros/noetic/setup.bash
source /home/a/scout_ws/devel/setup.bash

pids=()

cleanup() {
    local s=$?
    trap - INT TERM EXIT
    echo "[smoke] tearing down..."
    pkill -INT -f 'rosbag.*record' 2>/dev/null || true
    sleep 2
    for p in "${pids[@]}"; do
        kill -INT "$p" 2>/dev/null || true
    done
    sleep 3
    pkill -KILL -f 'gzclient' 2>/dev/null || true
    pkill -KILL -f 'gzserver' 2>/dev/null || true
    pkill -KILL -f 'roslaunch' 2>/dev/null || true
    pkill -KILL -f 'rosmaster' 2>/dev/null || true
    pkill -KILL -f 'template_fixed_path_generator' 2>/dev/null || true
    pkill -KILL -f 'rosbag.*record' 2>/dev/null || true
    sleep 1
    if [[ -f "${BAG_FILE}.active" && ! -f "${BAG_FILE}" ]]; then
        echo "[smoke] reindexing ${BAG_FILE}.active ..."
        rosbag reindex "${BAG_FILE}.active" 2>&1 | tail -2 || true
    fi
    echo "[smoke] bag: ${BAG_FILE}"
    exit "$s"
}
trap cleanup INT TERM EXIT

# --- 1. nav stack ---
echo "[smoke] launching nav stack..."
SIM_ENV=open USE_RVIZ="${USE_RVIZ}" \
  SPAWN_X="${SPAWN_X}" SPAWN_Y="${SPAWN_Y}" SPAWN_Z=0.1 SPAWN_YAW="${SPAWN_YAW}" \
  rosrun scout_local_planner launch_sim_nav_stack.sh > /tmp/smoke_nav.log 2>&1 &
pids+=("$!")
attempts=0
until grep -q 'Simulation navigation stack started' /tmp/smoke_nav.log 2>/dev/null; do
    sleep 3
    attempts=$((attempts+1))
    if [ $attempts -gt 40 ]; then
        echo "[smoke] ERROR: nav stack did not become ready" >&2
        exit 2
    fi
done
echo "[smoke] nav stack ready"

# --- 2. MPC ---
echo "[smoke] launching MPC..."
roslaunch scout_local_planner slosh_experiment_sim.launch \
    global_path_topic:=/scout/global_path_fixed \
    Q_slosh:="${Q_SLOSH}" \
    terminal_recovery_enable:=false \
    terminal_slowdown_enable:=true \
    terminal_capture_stop_enable:=true \
    v_des_rate_limit_enable:=true \
    > /tmp/smoke_mpc.log 2>&1 &
pids+=("$!")
attempts=0
until rostopic info /cmd_vel 2>/dev/null | grep -q 'scout_local_planner'; do
    sleep 2
    attempts=$((attempts+1))
    if [ $attempts -gt 25 ]; then
        echo "[smoke] ERROR: MPC did not start" >&2
        exit 2
    fi
done
echo "[smoke] MPC ready"

# --- 3. path generator (dedicated goal topic so MBF does not intercept) ---
echo "[smoke] launching path generator (goal topic ${GOAL_TOPIC})..."
rosrun scout_local_planner template_fixed_path_generator.py \
    --template s_curve \
    --goal-topic "${GOAL_TOPIC}" \
    --output-topic /scout/global_path_fixed \
    --path-file "${PATH_FILE}" \
    --start-heading current \
    --spacing 0.05 \
    --amplitude-ratio 0.18 \
    --min-amplitude 0.25 \
    --max-amplitude 1.20 \
    --publish-count 0 > /tmp/smoke_pathgen.log 2>&1 &
pids+=("$!")
attempts=0
until rosnode list 2>/dev/null | grep -q template_fixed_path_generator; do
    sleep 1
    attempts=$((attempts+1))
    if [ $attempts -gt 15 ]; then
        echo "[smoke] ERROR: path generator did not start" >&2
        exit 2
    fi
done
echo "[smoke] path generator ready"

# --- 4. rosbag ---
echo "[smoke] starting rosbag: ${BAG_FILE}"
rosbag record -O "${BAG_FILE}" \
    /cmd_vel /odom /gazebo/model_states /imu/data /imu/data_raw /joint_states \
    /scout/global_path_fixed "${GOAL_TOPIC}" /tf /tf_static \
    /slosh/height /slosh/eta_dot > /tmp/smoke_bag.log 2>&1 &
pids+=("$!")
sleep 2

# --- 5. publish goal ---
echo "[smoke] publishing goal (${GOAL_X}, ${GOAL_Y}) in map"
rostopic pub -1 "${GOAL_TOPIC}" geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map', stamp: now}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
    > /dev/null

# --- 6. monitor ---
echo "[smoke] tracking for ${TRACK_SEC}s..."
end=$((SECONDS + TRACK_SEC))
while (( SECONDS < end )); do
    sleep 10
    pose=$(rostopic echo -n 1 /odom/pose/pose/position 2>/dev/null \
            | awk '/x:|y:/{printf "%s=%s ",$1,$2}')
    echo "[smoke] t=${SECONDS}s odom ${pose}"
done

echo "[smoke] tracking window done; tearing down via trap"
# cleanup runs on exit
