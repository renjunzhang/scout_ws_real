#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="stub"
cmake_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-acados) mode="real"; shift ;;
    --stub) mode="stub"; shift ;;
    *) cmake_args+=("$1"); shift ;;
  esac
done
if [[ "$mode" == real ]]; then
  : "${ACADOS_SOURCE_DIR:?--with-acados requires ACADOS_SOURCE_DIR to point to an acados install}"
  [[ -f "${ACADOS_SOURCE_DIR}/lib/libacados.so" ]] || {
    echo "ACADOS_SOURCE_DIR lacks lib/libacados.so: ${ACADOS_SOURCE_DIR}" >&2; exit 2;
  }
  build_dir="${SPMPC_NATIVE_REAL_BUILD_DIR:-/tmp/spmpc_native_build_real}"
  cmake_args+=("-DSPMPC_NATIVE_WITH_ACADOS=ON" "-DACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}")
  export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"
else
  build_dir="${SPMPC_NATIVE_STUB_BUILD_DIR:-/tmp/spmpc_native_build_stub}"
  cmake_args+=("-DSPMPC_NATIVE_WITH_ACADOS=OFF")
fi
cmake -S "${repo_root}/test/native" -B "${build_dir}" -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo "${cmake_args[@]}"
cmake --build "${build_dir}" --parallel "${SPMPC_NATIVE_JOBS:-2}"
# Ubuntu 20.04 / ROS Noetic ships CTest 3.16, before --test-dir was added.
# Older CTest silently ignores that option and can report success with no tests.
(
  cd "${build_dir}"
  ctest --output-on-failure
)
