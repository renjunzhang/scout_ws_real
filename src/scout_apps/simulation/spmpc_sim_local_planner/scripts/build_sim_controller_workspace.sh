#!/usr/bin/env bash
# Build only the source-separated simulation controller into an external
# catkin prefix.  This deliberately never writes into scout_ws/build or
# scout_ws/devel, so preparing/running simulation cannot replace a real-robot
# executable that happens to share the checkout.
set -euo pipefail

fail() {
  echo "[build_sim_controller_workspace] ERROR: $*" >&2
  exit 64
}

SIM_ROOT="/data/a/scout_sim_replacement"
SIM_BUILD_WORKSPACE="${SMPCC_SIM_BUILD_WORKSPACE:-${SIM_ROOT}/r8_controller_ws}"
SOURCE_SPACE="/home/a/scout_ws/src"
PACKAGE="spmpc_sim_local_planner"

BUILD_TARGET=""
case "${1:-}" in
  "") ;;
  --node-admission-test)
    # This exact catkin target is emitted by catkin_add_gtest below.  It stays
    # in the external simulation workspace, never scout_ws/build.
    BUILD_TARGET="run_tests_spmpc_sim_local_planner_gtest_test_sim_node_admission"
    ;;
  *)
    fail "usage: $0 [--node-admission-test]"
    ;;
esac

# A formal source receipt binds this exact external build prefix.  Do not
# silently accept an arbitrary overlay supplied by a caller.
[[ "${SIM_BUILD_WORKSPACE}" == "${SIM_ROOT}/r8_controller_ws" ]] || \
  fail "SMPCC_SIM_BUILD_WORKSPACE must be ${SIM_ROOT}/r8_controller_ws"
[[ -d "${SOURCE_SPACE}" && -f "${SOURCE_SPACE}/CMakeLists.txt" ]] || \
  fail "source space is unavailable: ${SOURCE_SPACE}"
[[ -f "${SOURCE_SPACE}/scout_apps/simulation/${PACKAGE}/package.xml" ]] || \
  fail "source-separated package is unavailable"
[[ ! -L "${SIM_BUILD_WORKSPACE}" ]] || \
  fail "simulation build workspace may not be a symlink"

ACADOS_ROOT="${ACADOS_SOURCE_DIR:-/home/a/acados}"
[[ -d "${ACADOS_ROOT}" && -f "${ACADOS_ROOT}/lib/libacados.so" ]] || \
  fail "ACADOS_SOURCE_DIR must name an ACADOS runtime with lib/libacados.so"

mkdir -p "${SIM_BUILD_WORKSPACE}"

# Start from a deliberately small process environment.  In particular, no
# real-robot workspace setup, ROS package path, loader path, or catkin cache
# is inherited.  `--only-pkg-with-deps` makes the source tree read-only input:
# it cannot build an unrelated real-robot package.
env -i \
  HOME="${HOME:-/home/a}" \
  USER="${USER:-a}" \
  LANG="${LANG:-C.UTF-8}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  SHELL="/bin/bash" \
  ROS_DISTRO="${ROS_DISTRO:-}" \
  ROS_MASTER_URI="http://127.0.0.1:11311" \
  ACADOS_SOURCE_DIR="${ACADOS_ROOT}" \
  SMPCC_SIM_BUILD_WORKSPACE="${SIM_BUILD_WORKSPACE}" \
  SMPCC_SIM_SOURCE_SPACE="${SOURCE_SPACE}" \
  SMPCC_SIM_BUILD_TARGET="${BUILD_TARGET}" \
  /bin/bash --noprofile --norc -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    set -u
    command=(/opt/ros/noetic/bin/catkin_make \
      -C "${SMPCC_SIM_BUILD_WORKSPACE}" \
      --source "${SMPCC_SIM_SOURCE_SPACE}" \
      --build "${SMPCC_SIM_BUILD_WORKSPACE}/build" \
      --only-pkg-with-deps spmpc_sim_local_planner \
      --force-cmake -j2)
    if [[ -n "${SMPCC_SIM_BUILD_TARGET}" ]]; then
      command+=("${SMPCC_SIM_BUILD_TARGET}")
    fi
    exec "${command[@]}"
  '
