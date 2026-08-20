#!/usr/bin/env bash
# Own only a fresh ROS master, the isolated Gazebo proxy world, and read-only
# localization for one simulation case.  Controller and path publishing are
# deliberately absent; they are a later simulation-owned step.  This script
# never invokes a real-robot controller package, a legacy proxy navigation
# adapter, or an ambient ROS/Gazebo master.
set -euo pipefail

fail() { echo "[launch_sim_environment] ERROR: $*" >&2; exit 1; }
log() { echo "[launch_sim_environment] $*"; }

SIM_ROOT="${SMPCC_SIM_ROOT:-/data/a/scout_sim_replacement}"
CLASSIC_WS="${SMPCC_SIM_CLASSIC_WS:-${SIM_ROOT}/classic_ws}"
SIM_BUILD_WORKSPACE="${SMPCC_SIM_BUILD_WORKSPACE:-${SIM_ROOT}/r8_controller_ws}"
SIM_SETUP="${SIM_BUILD_WORKSPACE}/devel/setup.bash"
CARTO_SETUP="${CARTOGRAPHER_WS_SETUP:-/home/a/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/setup.bash}"
WORLD_FILE="${WORLD_NAME:-${SIM_ROOT}/classic_ws/src/scout_mini_proxy_gazebo/worlds/open_walled_proxy.world}"
MAP_FILE="${MAP_FILE:-${SIM_ROOT}/maps/proxy_world_manual_saved_20260611_154348.pbstream}"
LOG_DIR="${LOG_DIR:-${SIM_ROOT}/logs/smpcc_sim_environment_$(date +%Y%m%d_%H%M%S)}"
ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_DIR}/ros}"

# Do not expose broad source roots to rospack.  In particular, the legacy
# proxy navigation adapter and the real controller are deliberately absent
# from this environment's ROS_PACKAGE_PATH.  The Gazebo world remains an
# immutable data asset under SIM_ROOT; all launch decisions live in this
# package's smpcc_sim_environment.launch.
SIM_PACKAGE_SOURCE="${SMPCC_SIM_PACKAGE_SOURCE:-/home/a/scout_ws/src/scout_apps/simulation/spmpc_sim_local_planner}"
NANOSCAN_BRINGUP_SOURCE="${SMPCC_SIM_NANOSCAN_BRINGUP_SOURCE:-/home/a/scout_ws/src/scout_apps/sensors/nanoscan3_bringup}"
NANOSCAN_LOCALIZATION_SOURCE="${SMPCC_SIM_NANOSCAN_LOCALIZATION_SOURCE:-/home/a/scout_ws/src/scout_apps/sensors/nanoscan3_localization}"
PROXY_DESCRIPTION_SOURCE="${SMPCC_SIM_PROXY_DESCRIPTION_SOURCE:-${CLASSIC_WS}/src/scout_mini_proxy_description}"
PROXY_BRINGUP_SOURCE="${SMPCC_SIM_PROXY_BRINGUP_SOURCE:-${CLASSIC_WS}/src/scout_mini_proxy_bringup}"

[[ -n "${ROS_MASTER_URI:-}" ]] || fail "ROS_MASTER_URI must be supplied by the fresh-case runner"
[[ -n "${GAZEBO_MASTER_URI:-}" ]] || fail "GAZEBO_MASTER_URI must be supplied by the fresh-case runner"
[[ -f "${WORLD_FILE}" ]] || fail "world file is missing: ${WORLD_FILE}"
[[ -f "${MAP_FILE}" ]] || fail "map file is missing: ${MAP_FILE}"
[[ "${SIM_ROOT}" == "/data/a/scout_sim_replacement" ]] || fail "SMPCC_SIM_ROOT must be /data/a/scout_sim_replacement"
[[ "${SIM_BUILD_WORKSPACE}" == "${SIM_ROOT}/r8_controller_ws" ]] || fail "SMPCC_SIM_BUILD_WORKSPACE must be ${SIM_ROOT}/r8_controller_ws"
[[ -f "${SIM_SETUP}" ]] || fail "isolated simulation setup is missing: ${SIM_SETUP}; run build_sim_controller_workspace.sh"
[[ -f "${CARTO_SETUP}" ]] || fail "Cartographer setup is required for isolated localization: ${CARTO_SETUP}"
[[ -d "${CLASSIC_WS}/devel" && -d "${CLASSIC_WS}/src" ]] || fail "classic simulation workspace is incomplete: ${CLASSIC_WS}"
for source_root in "${SIM_PACKAGE_SOURCE}" "${NANOSCAN_BRINGUP_SOURCE}" \
                   "${NANOSCAN_LOCALIZATION_SOURCE}" "${PROXY_DESCRIPTION_SOURCE}" \
                   "${PROXY_BRINGUP_SOURCE}"; do
  [[ -f "${source_root}/package.xml" ]] || fail "required isolated package source is missing: ${source_root}"
done

parse_loopback_port() {
  local uri="$1" label="$2" port
  if [[ ! "${uri}" =~ ^http://127\.0\.0\.1:([0-9]+)$ ]]; then
    fail "${label} must be an exact loopback URI (http://127.0.0.1:PORT): ${uri}"
  fi
  port="${BASH_REMATCH[1]}"
  (( 10#${port} >= 1024 && 10#${port} <= 65535 )) || fail "${label} port is out of range: ${port}"
  printf '%s\n' "${port}"
}

require_unused_loopback_port() {
  local port="$1" label="$2"
  python3 - "${port}" "${label}" <<'PY'
import socket
import sys

port = int(sys.argv[1])
label = sys.argv[2]
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sock.bind(("127.0.0.1", port))
except OSError as exc:
    raise SystemExit(f"{label} port {port} is already occupied; fresh case refused: {exc}")
finally:
    sock.close()
PY
}

ROS_PORT="$(parse_loopback_port "${ROS_MASTER_URI}" "ROS_MASTER_URI")"
GAZEBO_PORT="$(parse_loopback_port "${GAZEBO_MASTER_URI}" "GAZEBO_MASTER_URI")"
[[ "${ROS_PORT}" != "${GAZEBO_PORT}" ]] || fail "ROS and Gazebo must use distinct fresh ports"
require_unused_loopback_port "${ROS_PORT}" "ROS"
require_unused_loopback_port "${GAZEBO_PORT}" "Gazebo"

# ROS setup files tend to append the whole development source tree.  Replace
# that broad search path with this fixed simulation allowlist plus the
# Cartographer install share and system ROS share.  The command paths are
# fixed by this script, and rospack cannot resolve excluded
# controller/navigation packages from a shell launched by it.
# Noetic's setup hook reads ROS_DISTRO before it exports it.  Supply a benign
# empty value first so this launcher also works from a clean user-service
# environment under ``set -u``; source then fills the canonical value.
export ROS_DISTRO="${ROS_DISTRO:-}"
source /opt/ros/noetic/setup.bash
source "${SIM_SETUP}"
source "${CARTO_SETUP}"
SIM_DEVEL="${SIM_SETUP%/setup.bash}"
CARTO_PREFIX="${CARTO_SETUP%/setup.bash}"
CARTO_SHARE="${CARTO_PREFIX}/share"
[[ -f "${CARTO_SHARE}/cartographer_ros/package.xml" ]] || fail "Cartographer package is missing below isolated setup: ${CARTO_SHARE}"
export CMAKE_PREFIX_PATH="${CLASSIC_WS}/devel:${SIM_DEVEL}:${CARTO_PREFIX}:/opt/ros/noetic"
export ROS_PACKAGE_PATH="${PROXY_DESCRIPTION_SOURCE}:${PROXY_BRINGUP_SOURCE}:${SIM_PACKAGE_SOURCE}:${NANOSCAN_BRINGUP_SOURCE}:${NANOSCAN_LOCALIZATION_SOURCE}:${CARTO_SHARE}:/opt/ros/noetic/share"
export PATH="${CLASSIC_WS}/devel/bin:${SIM_DEVEL}/bin:${CARTO_PREFIX}/bin:/opt/ros/noetic/bin:${PATH}"
# Solver artifacts deliberately retain historical model names, so an inherited
# loader path must never choose a same-named artifact from the real controller
# tree.  The four source-separated copies are mandatory and come first.
SIM_ACADOS_DIRS=(
  "${SIM_PACKAGE_SOURCE}/generated/acados/spmpc_b0"
  "${SIM_PACKAGE_SOURCE}/generated/acados/spmpc_slosh"
  "${SIM_PACKAGE_SOURCE}/generated/acados/spmpc_b0_direct_omega_legacy"
  "${SIM_PACKAGE_SOURCE}/generated/acados/spmpc_slosh_direct_omega"
)
for solver_dir in "${SIM_ACADOS_DIRS[@]}"; do
  [[ -d "${solver_dir}" ]] || fail "simulation ACADOS directory is missing: ${solver_dir}"
done
case ":${LD_LIBRARY_PATH:-}:" in
  *":/home/a/scout_ws/src/scout_apps/control/spmpc_local_planner/"*)
    fail "inherited real-controller ACADOS loader path is forbidden"
    ;;
esac
SIM_ACADOS_LIBRARY_PATH="$(IFS=:; printf '%s' "${SIM_ACADOS_DIRS[*]}")"
ACADOS_ROOT="${ACADOS_SOURCE_DIR:-/home/a/acados}"
[[ -f "${ACADOS_ROOT}/lib/libacados.so" ]] || fail "ACADOS runtime is missing under ${ACADOS_ROOT}"
export LD_LIBRARY_PATH="${SIM_ACADOS_LIBRARY_PATH}:${CLASSIC_WS}/devel/lib:${SIM_DEVEL}/lib:${CARTO_PREFIX}/lib:${ACADOS_ROOT}/lib:/opt/ros/noetic/lib"
export PYTHONPATH="${CLASSIC_WS}/devel/lib/python3/dist-packages:${SIM_DEVEL}/lib/python3/dist-packages:${CARTO_PREFIX}/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"
export GAZEBO_PLUGIN_PATH="${CLASSIC_WS}/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export ROS_LOG_DIR

# Force all graph endpoints created here to remain local to this fresh case;
# do not inherit a workstation hostname or namespace from a real-robot shell.
export ROS_IP=127.0.0.1
export ROS_NAMESPACE=/
unset ROS_HOSTNAME

assert_forbidden_packages_hidden() {
  local package_name
  for package_name in spmpc_local_planner spmpc_experiments slosh_models scout_mini_proxy_nav_adapter; do
    if rospack find "${package_name}" >/dev/null 2>&1; then
      fail "forbidden package remains discoverable in environment-only launch: ${package_name}"
    fi
  done
}

assert_fresh_master() {
  local node
  while IFS= read -r node; do
    [[ -z "${node}" || "${node}" == "/rosout" ]] && continue
    fail "fresh ROS master already contains node ${node}"
  done < <(rosnode list)
}

assert_no_forbidden_nodes() {
  local node
  while IFS= read -r node; do
    case "${node}" in
      /spmpc_local_planner|/sim_spmpc_local_planner|/proxy_spmpc|/scout_mini_proxy_nav_adapter|/move_base)
        fail "environment-only graph unexpectedly started forbidden node ${node}"
        ;;
    esac
  done < <(rosnode list)
}

assert_no_topic_publishers() {
  local topic="$1" role="$2" info
  info="$(rostopic info "${topic}" 2>/dev/null || true)"
  if printf '%s\n' "${info}" | awk '
      /^Publishers:/ { in_publishers = 1; next }
      /^Subscribers:/ { in_publishers = 0 }
      in_publishers && /^[[:space:]]*\*/ { found = 1 }
      END { exit(found ? 0 : 1) }
    '; then
    fail "environment-only graph has a ${role} publisher on ${topic}"
  fi
}

PIDS=()
NAMES=()
cleanup() {
  local code=$?
  trap - EXIT INT TERM
  log "cleanup requested (status=${code}); stopping only owned children"
  local index pid
  for ((index=${#PIDS[@]} - 1; index >= 0; --index)); do
    pid="${PIDS[index]}"
    if kill -0 "${pid}" 2>/dev/null; then kill -TERM "${pid}" 2>/dev/null || true; fi
  done
  for ((index=${#PIDS[@]} - 1; index >= 0; --index)); do wait "${PIDS[index]}" 2>/dev/null || true; done
  exit "${code}"
}
trap cleanup EXIT
trap 'log "received SIGINT"; exit 130' INT
trap 'log "received SIGTERM"; exit 143' TERM

start() {
  local name="$1"
  shift
  log "start ${name}: $*"
  "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
  NAMES+=("${name}")
  log "started ${name} pid=$!"
}
wait_topic() {
  local topic="$1"
  local timeout_sec="$2"
  timeout "${timeout_sec}" rostopic echo -n 1 "${topic}" >"${LOG_DIR}/ready_${topic#/}.log" 2>&1
}
wait_tf() {
  local parent="$1" child="$2" timeout_sec="$3"
  # ``tf_echo`` is intentionally long-lived.  A shell pipe ending in grep can
  # consume a valid Translation line but still wait for its upstream process,
  # turning a ready TF graph into a later timeout (and tearing down the fresh
  # environment during motion).  Own one probe explicitly and terminate only
  # that probe as soon as a translation arrives.
  python3 - "${parent}" "${child}" "${timeout_sec}" <<'PY'
import os
import select
import signal
import subprocess
import sys
import time

parent, child, raw_timeout = sys.argv[1:]
deadline = time.monotonic() + float(raw_timeout)
probe = subprocess.Popen(
    ["rosrun", "tf", "tf_echo", parent, child],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    start_new_session=True,
    bufsize=1,
)
found = False
try:
    while time.monotonic() < deadline:
        assert probe.stdout is not None
        remaining = max(0.01, deadline - time.monotonic())
        readable, _, _ = select.select([probe.stdout], [], [], min(0.25, remaining))
        if readable:
            line = probe.stdout.readline()
            if "Translation:" in line:
                found = True
                break
        if probe.poll() is not None:
            break
finally:
    if probe.poll() is None:
        try:
            os.killpg(probe.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    try:
        probe.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(probe.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        probe.wait(timeout=2.0)
raise SystemExit(0 if found else 1)
PY
}

TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-45}"
MAP_TIMEOUT="${MAP_TIMEOUT:-60}"
# Cartographer's restored map-to-base transform can take longer than raw
# odom/scan readiness after a fresh Gazebo start.  This is a pre-motion
# infrastructure admission limit, not the 60 s trajectory timeout; failing it
# remains an acquisition failure before any controller is released.
TF_TIMEOUT="${TF_TIMEOUT:-75}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
HEADLESS="${HEADLESS:-true}"
PAUSED="${PAUSED:-false}"
VERBOSE="${VERBOSE:-false}"
SPAWN_X="${SPAWN_X:--4.0}"
SPAWN_Y="${SPAWN_Y:-0.0}"
SPAWN_Z="${SPAWN_Z:-0.0}"
SPAWN_YAW="${SPAWN_YAW:-0.0}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
CMD_VEL_DRIVE_TOPIC="${CMD_VEL_DRIVE_TOPIC:-/cmd_vel_drive}"
CMD_GUARD_ENABLE="${CMD_GUARD_ENABLE:-true}"
REFERENCE_PATH_TOPIC="${REFERENCE_PATH_TOPIC:-/scout/global_path_fixed}"
MAX_ANGULAR_ACCEL="${MAX_ANGULAR_ACCEL:-1.2}"
SMPCC_SIMULATOR_SEED="${SMPCC_SIMULATOR_SEED:-}"

for boolean_name in GAZEBO_GUI HEADLESS PAUSED VERBOSE CMD_GUARD_ENABLE; do
  case "${!boolean_name}" in true|false|0|1) ;; *) fail "${boolean_name} must be true, false, 0, or 1";; esac
done
[[ "${CMD_VEL_TOPIC}" == /* ]] || fail "CMD_VEL_TOPIC must be absolute"
[[ "${CMD_VEL_DRIVE_TOPIC}" == /* ]] || fail "CMD_VEL_DRIVE_TOPIC must be absolute"
if [[ "${CMD_GUARD_ENABLE}" =~ ^(false|0)$ && "${CMD_VEL_TOPIC}" != "${CMD_VEL_DRIVE_TOPIC}" ]]; then
  fail "CMD_GUARD_ENABLE=false requires CMD_VEL_TOPIC=${CMD_VEL_DRIVE_TOPIC}; the controller must publish directly to the robot plugin"
fi
[[ "${REFERENCE_PATH_TOPIC}" == /* ]] || fail "REFERENCE_PATH_TOPIC must be absolute"
if [[ -n "${SMPCC_SIMULATOR_SEED}" ]]; then
  [[ "${SMPCC_SIMULATOR_SEED}" =~ ^[0-9]+$ ]] || fail "SMPCC_SIMULATOR_SEED must be a non-negative integer"
fi

mkdir -p "${LOG_DIR}" "${ROS_LOG_DIR}"
assert_forbidden_packages_hidden
start roscore roscore -p "${ROS_PORT}"
timeout 20 bash -c 'until rostopic list >/dev/null 2>&1; do sleep 0.2; done'
assert_fresh_master

environment_command=(roslaunch spmpc_sim_local_planner smpcc_sim_environment.launch
  world_file:="${WORLD_FILE}" map_file:="${MAP_FILE}"
  gui:="${GAZEBO_GUI}" headless:="${HEADLESS}" paused:="${PAUSED}" verbose:="${VERBOSE}"
  x:="${SPAWN_X}" y:="${SPAWN_Y}" z:="${SPAWN_Z}" yaw:="${SPAWN_YAW}"
  cmd_vel_in:="${CMD_VEL_TOPIC}" cmd_vel_drive:="${CMD_VEL_DRIVE_TOPIC}"
  cmd_guard_enable:="${CMD_GUARD_ENABLE}" publish_odom_tf:=false
  max_angular_accel:="${MAX_ANGULAR_ACCEL}" localization_rviz:="${LOCALIZATION_RVIZ:-false}")
if [[ -n "${SMPCC_SIMULATOR_SEED}" ]]; then
  environment_command+=(gzserver_extra_args:="--seed ${SMPCC_SIMULATOR_SEED}")
fi
start environment "${environment_command[@]}"
wait_topic /odom "${TOPIC_TIMEOUT}"
wait_topic /scan_front "${TOPIC_TIMEOUT}"
wait_topic /map "${MAP_TIMEOUT}"
wait_tf odom base_link "${TF_TIMEOUT}"
wait_tf map base_link "${TF_TIMEOUT}"
assert_no_forbidden_nodes
assert_no_topic_publishers "${CMD_VEL_TOPIC}" "controller"
assert_no_topic_publishers "${REFERENCE_PATH_TOPIC}" "path"

log "simulation environment is ready; controller/path are intentionally absent"
set +e
wait -n "${PIDS[@]}"
code=$?
set -e
log "owned child exited with status ${code}"
for index in "${!PIDS[@]}"; do
  pid="${PIDS[index]}"
  if ! kill -0 "${pid}" 2>/dev/null; then
    set +e
    wait "${pid}"
    child_code=$?
    set -e
    log "owned child ${NAMES[index]} pid=${pid} exit=${child_code}"
  fi
done
exit "${code}"
