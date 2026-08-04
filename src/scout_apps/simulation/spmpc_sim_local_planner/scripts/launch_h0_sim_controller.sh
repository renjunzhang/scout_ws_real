#!/usr/bin/env bash
# Start only the simulation path publisher and separately linked simulation
# S-MPCC target on an already-owned fresh Gazebo/ROS environment.  This script
# never includes a real-robot controller launch and never changes an ambient
# ROS master URI.
set -euo pipefail

fail() {
  echo "[launch_h0_sim_controller] ERROR: $*" >&2
  exit 1
}

[[ -n "${ROS_MASTER_URI:-}" ]] || fail "ROS_MASTER_URI must be supplied by the fresh H0 runner"
[[ -n "${GAZEBO_MASTER_URI:-}" ]] || fail "GAZEBO_MASTER_URI must be supplied by the fresh H0 runner"

# Noetic's setup hook reads ROS_DISTRO before it exports it.  Set an empty
# default so this isolated attach entry also works from a clean service
# environment with nounset enabled.
export ROS_DISTRO="${ROS_DISTRO:-}"
source /opt/ros/noetic/setup.bash
SIM_ROOT="/data/a/scout_sim_replacement"
SIM_BUILD_WORKSPACE="${SMPCC_SIM_BUILD_WORKSPACE:-${SIM_ROOT}/r8_controller_ws}"
[[ "${SIM_BUILD_WORKSPACE}" == "${SIM_ROOT}/r8_controller_ws" ]] || \
  fail "SMPCC_SIM_BUILD_WORKSPACE must be ${SIM_ROOT}/r8_controller_ws"
SIM_SETUP="${SIM_BUILD_WORKSPACE}/devel/setup.bash"
[[ -f "${SIM_SETUP}" ]] || fail "isolated simulation setup missing: ${SIM_SETUP}; run build_sim_controller_workspace.sh"
source "${SIM_SETUP}"

# The simulation devel prefix is external to scout_ws.  Re-establish the exact
# package allowlist before resolving either launch file so a caller cannot use
# an inherited physical controller/experiment overlay.
SIM_PACKAGE_SOURCE="${SMPCC_SIM_PACKAGE_SOURCE:-/home/a/scout_ws/src/scout_apps/simulation/spmpc_sim_local_planner}"
SIM_DEVEL="${SIM_SETUP%/setup.bash}"
[[ -f "${SIM_PACKAGE_SOURCE}/package.xml" ]] || fail "simulation package source missing: ${SIM_PACKAGE_SOURCE}"
export CMAKE_PREFIX_PATH="${SIM_DEVEL}:/opt/ros/noetic"
export ROS_PACKAGE_PATH="${SIM_PACKAGE_SOURCE}:/opt/ros/noetic/share"
export ROS_IP=127.0.0.1
export ROS_NAMESPACE=/
unset ROS_HOSTNAME

# The fork owns copies of every ACADOS solver artifact.  Put those directories
# ahead of the shared devel lib directory so an inherited loader path can
# never substitute a real-controller solver with the same historical SONAME.
SIM_ACADOS_DIRS=(
  "${SIM_PACKAGE_SOURCE}/generated/acados/spmpc_b0"
  "${SIM_PACKAGE_SOURCE}/generated/acados/spmpc_slosh"
  "${SIM_PACKAGE_SOURCE}/generated/acados/spmpc_b0_direct_omega_legacy"
  "${SIM_PACKAGE_SOURCE}/generated/acados/spmpc_slosh_direct_omega"
)
for solver_dir in "${SIM_ACADOS_DIRS[@]}"; do
  [[ -d "${solver_dir}" ]] || fail "simulation ACADOS directory missing: ${solver_dir}"
done
case ":${LD_LIBRARY_PATH:-}:" in
  *":/home/a/scout_ws/src/scout_apps/control/spmpc_local_planner/"*)
    fail "inherited real-controller ACADOS loader path is forbidden"
    ;;
esac
ACADOS_ROOT="${ACADOS_SOURCE_DIR:-/home/a/acados}"
[[ -f "${ACADOS_ROOT}/lib/libacados.so" ]] || fail "ACADOS runtime is missing under ${ACADOS_ROOT}"
SIM_ACADOS_LIBRARY_PATH="$(IFS=:; printf '%s' "${SIM_ACADOS_DIRS[*]}")"
export LD_LIBRARY_PATH="${SIM_ACADOS_LIBRARY_PATH}:${SIM_DEVEL}/lib:${ACADOS_ROOT}/lib:/opt/ros/noetic/lib"
export PYTHONPATH="${SIM_DEVEL}/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"

assert_forbidden_packages_hidden() {
  local package_name
  for package_name in spmpc_local_planner spmpc_experiments slosh_models scout_mini_proxy_nav_adapter; do
    if rospack find "${package_name}" >/dev/null 2>&1; then
      fail "real/legacy package remains discoverable in simulation controller attach: ${package_name}"
    fi
  done
}

assert_forbidden_packages_hidden

PIDS=()
cleanup() {
  local code=$?
  trap - EXIT INT TERM
  local pid
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
  exit "${code}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_child() {
  "$@" &
  PIDS+=("$!")
}

if [[ "${START_PATH_PUBLISHER:-true}" == "true" || "${START_PATH_PUBLISHER:-true}" == "1" ]]; then
  # The H0 publisher is simulation-owned.  It samples /odom and only emits a
  # latched Path/optional PoseStamped goal; it cannot launch a proxy planner or
  # publish a velocity command.
  start_child roslaunch spmpc_sim_local_planner smpcc_sim_h0_fixed_path_publisher.launch \
    h0_development_ack:=true development_condition:=H0 \
    odom_topic:="${ODOM_TOPIC:-/odom}" expected_odom_frame:="${EXPECTED_ODOM_FRAME:-odom}" \
    output_topic:="${REFERENCE_PATH_TOPIC:-/scout/global_path_fixed}" \
    goal_topic:="${GOAL_TOPIC:-/scout/goal}" publish_goal:="${PUBLISH_GOAL:-true}" \
    goal_x:="${GOAL_X:-5.0}" goal_y:="${GOAL_Y:-0.0}" goal_yaw:="${GOAL_YAW:-0.0}" \
    path_template:="${PATH_TEMPLATE:-s_curve}" \
    path_start_heading:="${PATH_START_HEADING:-current}" \
    path_amplitude_ratio:="${PATH_AMPLITUDE_RATIO:-0.18}" \
    path_min_amplitude:="${PATH_MIN_AMPLITUDE:-0.25}" \
    path_max_amplitude:="${PATH_MAX_AMPLITUDE:-1.20}" \
    path_side:="${PATH_SIDE:-left}" \
    path_smooth_iterations:="${PATH_SMOOTH_ITERATIONS:-3}" \
    spacing_m:="${PATH_SPACING_M:-0.05}" odom_timeout_sec:="${ODOM_TIMEOUT_SEC:-15.0}"
fi

if [[ "${START_SPMPC:-true}" == "true" || "${START_SPMPC:-true}" == "1" ]]; then
  start_child roslaunch spmpc_sim_local_planner smpcc_sim_mechanism_r8.launch \
    sim_release_ack:=true sim_controller_variant:=SIM_H0_Bsmooth_R1 \
    sim_container_condition:=C1 \
    reference_path_topic:=/scout/global_path_fixed \
    costmap_topic:="${COSTMAP_TOPIC:-/map}" \
    cmd_vel_topic:="${CMD_VEL_TOPIC:-/cmd_vel}" \
    publish_cmd_vel:="${PUBLISH_CMD_VEL:-true}"
fi

(( ${#PIDS[@]} > 0 )) || fail "both START_PATH_PUBLISHER and START_SPMPC are false"
while true; do
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}"
      exit $?
    fi
  done
  sleep 0.2
done
