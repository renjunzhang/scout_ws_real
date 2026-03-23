#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "source this file instead of executing it:"
  echo "  source ${BASH_SOURCE[0]}"
  exit 1
fi

_realsense_env_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REALSENSE_WS_ROOT="$(cd "${_realsense_env_script_dir}/../../../../.." && pwd)"
export REALSENSE_ROS_DISTRO="${ROS_DISTRO:-noetic}"
export REALSENSE_LOCAL_ROS_ROOT="${REALSENSE_WS_ROOT}/.ros_deps/opt/ros/${REALSENSE_ROS_DISTRO}"

if [[ -f "/opt/ros/${REALSENSE_ROS_DISTRO}/setup.bash" ]]; then
  # Load the base ROS environment first, then prepend the local dependency overlay.
  source "/opt/ros/${REALSENSE_ROS_DISTRO}/setup.bash"
fi

export ROS_HOME="${ROS_HOME:-/tmp/ros_home}"
export CMAKE_PREFIX_PATH="${REALSENSE_LOCAL_ROS_ROOT}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export ROS_PACKAGE_PATH="${REALSENSE_WS_ROOT}/src:${REALSENSE_LOCAL_ROS_ROOT}/share${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
export LD_LIBRARY_PATH="${REALSENSE_LOCAL_ROS_ROOT}/lib:${REALSENSE_LOCAL_ROS_ROOT}/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PKG_CONFIG_PATH="${REALSENSE_LOCAL_ROS_ROOT}/lib/pkgconfig:${REALSENSE_LOCAL_ROS_ROOT}/lib/x86_64-linux-gnu/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"

unset _realsense_env_script_dir
