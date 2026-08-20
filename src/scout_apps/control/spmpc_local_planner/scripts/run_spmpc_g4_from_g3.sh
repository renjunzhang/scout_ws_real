#!/usr/bin/env bash
# Pure offline G4 trajectory/replay gate.  It never contacts the live ROS master.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
G3_ROOT="${G3_ROOT:-/home/geist/slosh_bags/real/20260801_spmpc_g3_processed_imu_w5_vs_bsmooth/H0}"
G4_OUT_DIR="${G4_OUT_DIR:-/home/geist/slosh_bags/real/20260801_spmpc_g4_from_g3/H0}"
PATH_FILE="${PATH_FILE:-/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json}"
G4_CONFIG="${G4_CONFIG:-${SCRIPT_DIR}/../tools/analysis/g4_replay_config.yaml}"
CODEGEN_JSON="${CODEGEN_JSON:-${REPO_ROOT}/src/scout_apps/control/spmpc_local_planner/generated/acados/spmpc_slosh/acados_ocp_spmpc_slosh.json}"
REPLAY_EXECUTABLE="${SPMPC_G4_SNAPSHOT_REPLAY:-${REPO_ROOT}/devel/lib/spmpc_local_planner/spmpc_g4_snapshot_replay}"

for path in "${G3_ROOT}" "${PATH_FILE}" "${G4_CONFIG}" "${CODEGEN_JSON}"; do
  [[ -e "${path}" ]] || { echo "[G4][ERR] missing: ${path}" >&2; exit 2; }
done
[[ -x "${REPLAY_EXECUTABLE}" ]] || {
  echo "[G4][ERR] missing C++ replay executable: ${REPLAY_EXECUTABLE}" >&2
  echo "[G4][ERR] build target spmpc_g4_snapshot_replay first" >&2
  exit 2
}

args=(
  --g3-root "${G3_ROOT}"
  --path-file "${PATH_FILE}"
  --config "${G4_CONFIG}"
  --out-dir "${G4_OUT_DIR}"
  --codegen-json "${CODEGEN_JSON}"
  --replay-executable "${REPLAY_EXECUTABLE}"
)
if [[ "${ALLOW_INCOMPLETE_G3:-false}" == "true" ]]; then
  args+=(--allow-incomplete-g3)
fi

python3 "${SCRIPT_DIR}/../tools/analysis/g4_replay_from_g3.py" "${args[@]}"
