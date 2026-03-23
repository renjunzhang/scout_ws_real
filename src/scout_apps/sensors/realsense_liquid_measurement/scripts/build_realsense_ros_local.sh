#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws_root="$(cd "${script_dir}/../../../../.." && pwd)"
deps_root="${ws_root}/.ros_deps"
deb_cache_dir="${ws_root}/.ros_deps_cache"
ros_distro="${ROS_DISTRO:-noetic}"
local_ros_root="${deps_root}/opt/ros/${ros_distro}"

required_packages=(
  ros-noetic-ddynamic-reconfigure
  ros-noetic-librealsense2
)

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "missing required command: $1" >&2
    exit 1
  fi
}

download_pkg() {
  local pkg="$1"
  mkdir -p "${deb_cache_dir}"
  pushd "${deb_cache_dir}" >/dev/null

  if apt download "${pkg}"; then
    popd >/dev/null
    return 0
  fi

  echo "apt download failed for ${pkg}, retrying without proxy env" >&2
  env -u https_proxy -u http_proxy -u all_proxy -u HTTPS_PROXY -u HTTP_PROXY -u ALL_PROXY \
    apt download "${pkg}"
  popd >/dev/null
}

latest_deb_for_pkg() {
  local pkg="$1"
  ls -t "${deb_cache_dir}/${pkg}"_*.deb 2>/dev/null | head -n1
}

ensure_local_deps() {
  local ddyn_marker="${local_ros_root}/share/ddynamic_reconfigure/cmake/ddynamic_reconfigureConfig.cmake"
  local rs_marker="${local_ros_root}/lib/x86_64-linux-gnu/cmake/realsense2/realsense2Config.cmake"

  if [[ -f "${ddyn_marker}" && -f "${rs_marker}" ]]; then
    return 0
  fi

  mkdir -p "${deps_root}"

  local pkg deb_path
  for pkg in "${required_packages[@]}"; do
    download_pkg "${pkg}"
    deb_path="$(latest_deb_for_pkg "${pkg}")"
    if [[ -z "${deb_path}" ]]; then
      echo "failed to locate downloaded package for ${pkg}" >&2
      exit 1
    fi
    dpkg-deb -x "${deb_path}" "${deps_root}"
  done
}

need_cmd apt
need_cmd catkin_make
need_cmd dpkg-deb

ensure_local_deps

# shellcheck disable=SC1091
source "${script_dir}/realsense_ros_env_local.sh"

catkin_make --force-cmake --pkg realsense2_description realsense2_camera -C "${ws_root}"
