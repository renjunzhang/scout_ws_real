#!/usr/bin/env bash
# Launch localization, global planner, and slosh-aware local planner for fixed-path experiments.

set -euo pipefail

Q_SLOSH="${1:-0}"
GLOBAL_PATH_TOPIC="${GLOBAL_PATH_TOPIC:-/scout/global_path_fixed}"
EXTERNAL_SPEED_PROFILE_CSV="${EXTERNAL_SPEED_PROFILE_CSV:-}"
EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE="${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE:-false}"
EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT="${EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT:-0.60}"
EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT="${EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT:-0.80}"
EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT="${EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT:-0.0}"
LOCALIZATION_ACCURACY_THRESHOLD="${LOCALIZATION_ACCURACY_THRESHOLD:-70}"
LOCALIZATION_ACCURACY_TOPIC="${LOCALIZATION_ACCURACY_TOPIC:-}"
LOCALIZATION_WAIT_TIMEOUT="${LOCALIZATION_WAIT_TIMEOUT:-0}"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi

if [[ -f /home/a/scout_ws/devel/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/a/scout_ws/devel/setup.bash
fi

pids=()
names=()

cleanup() {
    local status=$?
    trap - INT TERM EXIT

    if (( ${#pids[@]} > 0 )); then
        echo
        echo "[launch_fixed_path_slosh_stack] Stopping launched processes..."
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

    echo "[launch_fixed_path_slosh_stack] Starting ${name}: roslaunch $*"
    roslaunch "$@" &
    local pid=$!
    pids+=("${pid}")
    names+=("${name}")

    sleep 2
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "[launch_fixed_path_slosh_stack] ERROR: ${name} exited during startup." >&2
        wait "${pid}" || true
        exit 1
    fi
}

wait_for_localization_accuracy() {
    if [[ -z "${LOCALIZATION_ACCURACY_TOPIC}" ]]; then
        echo
        echo "[launch_fixed_path_slosh_stack] Wait until localization accuracy >= ${LOCALIZATION_ACCURACY_THRESHOLD}%."
        echo "[launch_fixed_path_slosh_stack] No LOCALIZATION_ACCURACY_TOPIC is configured, so this is a manual gate."
        read -r -p "[launch_fixed_path_slosh_stack] Press Enter to continue after localization accuracy reaches ${LOCALIZATION_ACCURACY_THRESHOLD}%..."
        return
    fi

    echo
    echo "[launch_fixed_path_slosh_stack] Waiting for ${LOCALIZATION_ACCURACY_TOPIC} >= ${LOCALIZATION_ACCURACY_THRESHOLD}%..."
    python3 - "${LOCALIZATION_ACCURACY_TOPIC}" "${LOCALIZATION_ACCURACY_THRESHOLD}" "${LOCALIZATION_WAIT_TIMEOUT}" <<'PY'
import re
import subprocess
import sys
import time

topic = sys.argv[1]
threshold = float(sys.argv[2])
timeout = float(sys.argv[3])
deadline = None if timeout <= 0.0 else time.time() + timeout

def extract_number(text):
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("---"):
            continue
        matches = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", line)
        if matches:
            return float(matches[-1])
    return None

while True:
    if deadline is not None and time.time() > deadline:
        print(f"timeout waiting for {topic} >= {threshold}", file=sys.stderr)
        sys.exit(1)

    proc = subprocess.run(
        ["rostopic", "echo", "-n", "1", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        timeout=5.0,
        check=False,
    )
    value = extract_number(proc.stdout)
    if value is not None:
        print(f"[localization gate] {topic}={value:.3f}, threshold={threshold:.3f}")
        if value >= threshold:
            sys.exit(0)

    time.sleep(1.0)
PY
}

echo "[launch_fixed_path_slosh_stack] Q_slosh=${Q_SLOSH}"
echo "[launch_fixed_path_slosh_stack] global_path_topic=${GLOBAL_PATH_TOPIC}"
echo "[launch_fixed_path_slosh_stack] external_speed_profile_csv=${EXTERNAL_SPEED_PROFILE_CSV:-<internal>}"
echo "[launch_fixed_path_slosh_stack] external_profile_execution_cap=${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE}"
echo "[launch_fixed_path_slosh_stack] localization_accuracy_threshold=${LOCALIZATION_ACCURACY_THRESHOLD}%"

start_launch "nanoscan3 localization" \
    nanoscan3_localization scout_nanoscan3_cartographer_localization.launch

wait_for_localization_accuracy

start_launch "MBF global planner" \
    scout_global_planner mbf_global.launch

start_launch "slosh local planner" \
    scout_local_planner slosh_experiment.launch \
    global_path_topic:="${GLOBAL_PATH_TOPIC}" \
    external_speed_profile_csv:="${EXTERNAL_SPEED_PROFILE_CSV}" \
    external_profile_execution_cap_enable:="${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE}" \
    external_profile_execution_accel_limit:="${EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT}" \
    external_profile_execution_decel_limit:="${EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT}" \
    external_profile_execution_jerk_limit:="${EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT}" \
    Q_slosh:="${Q_SLOSH}" \
    enable_slosh_box_constraint:=false \
    slosh_speed_governor_enable:=false \
    filter_alpha_v:=1.0 \
    filter_alpha_omega:=1.0 \
    filter_kappa_boost:=0.0 \
    slosh_use_imu_lateral_accel:=false \
    slosh_use_imu_yaw_rate:=true \
    slosh_use_imu_alpha_z:=false

echo "[launch_fixed_path_slosh_stack] All launches started."
echo "[launch_fixed_path_slosh_stack] Press Ctrl+C to stop all launched processes."

wait -n "${pids[@]}"
echo "[launch_fixed_path_slosh_stack] One launch process exited; shutting down the rest."
