#!/usr/bin/env bash
# record_spin_test.sh - Record a fixed-omega in-place spin for A-step before/after comparison.
#
# Prerequisite: Gazebo + bridge already running (e.g. via launch_sim_nav_stack.sh).
# Resets the model to a fixed pose via /gazebo/set_model_state, then publishes
# cmd_vel(0, OMEGA) for DURATION_S seconds while recording the relevant topics
# to a bag. Writes a sidecar .meta.json with the run parameters.

set -euo pipefail

LABEL="${LABEL:?LABEL is required (e.g. before_fix_a, after_fix_a)}"
OMEGA="${OMEGA:-0.2}"
DURATION_S="${DURATION_S:-30}"
WARMUP_S="${WARMUP_S:-2}"
COOLDOWN_S="${COOLDOWN_S:-1}"
BAG_DIR="${BAG_DIR:-/home/a/scout_ws/bags/sim_skid_steer_fix}"
MODEL_NAME="${MODEL_NAME:-scout/}"
RESET_X="${RESET_X:--4.0}"
RESET_Y="${RESET_Y:-0.0}"
RESET_Z="${RESET_Z:-0.1}"
RESET_YAW="${RESET_YAW:-0.0}"

mkdir -p "${BAG_DIR}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PREFIX="${BAG_DIR}/${TIMESTAMP}_${LABEL}"
BAG_FILE="${BAG_PREFIX}.bag"
META_FILE="${BAG_PREFIX}.meta.json"

pub_pid=""
rec_pid=""

cleanup() {
    local s=$?
    trap - INT TERM EXIT
    if [[ -n "${pub_pid}" ]]; then
        kill "${pub_pid}" 2>/dev/null || true
        wait "${pub_pid}" 2>/dev/null || true
    fi
    if [[ -n "${rec_pid}" ]]; then
        kill -INT "${rec_pid}" 2>/dev/null || true
        wait "${rec_pid}" 2>/dev/null || true
    fi
    exit "${s}"
}
trap cleanup INT TERM EXIT

# --- reset model pose ---
SIN_HALF_YAW="$(python3 -c "import math; print(math.sin(${RESET_YAW}/2.0))")"
COS_HALF_YAW="$(python3 -c "import math; print(math.cos(${RESET_YAW}/2.0))")"

echo "[record_spin] reset '${MODEL_NAME}' to (${RESET_X}, ${RESET_Y}, ${RESET_Z}), yaw=${RESET_YAW}"
rosservice call /gazebo/set_model_state "{
  model_state: {
    model_name: '${MODEL_NAME}',
    pose: {
      position: {x: ${RESET_X}, y: ${RESET_Y}, z: ${RESET_Z}},
      orientation: {x: 0.0, y: 0.0, z: ${SIN_HALF_YAW}, w: ${COS_HALF_YAW}}
    },
    twist: {
      linear: {x: 0.0, y: 0.0, z: 0.0},
      angular: {x: 0.0, y: 0.0, z: 0.0}
    },
    reference_frame: ''
  }
}" >/dev/null
sleep 1

# --- start bag ---
echo "[record_spin] start bag: ${BAG_FILE}"
rosbag record -O "${BAG_FILE}" \
    /cmd_vel /imu/data_raw /imu/data /odom \
    /gazebo/model_states /joint_states /tf /tf_static &
rec_pid=$!
sleep "${WARMUP_S}"

# --- publish spin command ---
echo "[record_spin] cmd_vel(0, ${OMEGA}) for ${DURATION_S}s"
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${OMEGA}}}" >/dev/null &
pub_pid=$!
sleep "${DURATION_S}"
kill "${pub_pid}" 2>/dev/null || true
wait "${pub_pid}" 2>/dev/null || true
pub_pid=""

rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null
sleep "${COOLDOWN_S}"

# --- stop bag ---
echo "[record_spin] stop bag"
kill -INT "${rec_pid}" 2>/dev/null || true
wait "${rec_pid}" 2>/dev/null || true
rec_pid=""

# --- metadata ---
GIT_HASH="$(cd /home/a/scout_ws && git rev-parse --short HEAD 2>/dev/null || echo unknown)"
cat > "${META_FILE}" <<EOF
{
  "label": "${LABEL}",
  "timestamp": "${TIMESTAMP}",
  "omega_rad_s": ${OMEGA},
  "duration_s": ${DURATION_S},
  "warmup_s": ${WARMUP_S},
  "reset_pose": {"x": ${RESET_X}, "y": ${RESET_Y}, "z": ${RESET_Z}, "yaw": ${RESET_YAW}},
  "model_name": "${MODEL_NAME}",
  "git_hash": "${GIT_HASH}"
}
EOF

echo "[record_spin] done"
echo "  bag : ${BAG_FILE}"
echo "  meta: ${META_FILE}"
