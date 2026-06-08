#!/usr/bin/env bash
# Point-to-point supplementary comparison runner.
#
# P2P 是论文补充实验：证明导航入口可用，不抢 fixed-path 主线。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OUT_ROOT="${OUT_ROOT:-/data/${USER}/spmpc_paper_compare/p2p_supplement_$(date +%Y%m%d_%H%M%S)}"
SPMPC_VARIANTS="${SPMPC_VARIANTS:-B0 B_ours}"
BASELINES="${BASELINES:-teb dwa}"
INCLUDE_MPC_LOCAL_PLANNER="${INCLUDE_MPC_LOCAL_PLANNER:-false}"
RUNS="${RUNS:-1}"
RECORD_SEC="${RECORD_SEC:-60}"
GOAL_X="${GOAL_X:--1.2}"
GOAL_Y="${GOAL_Y:-2.6}"
GOAL_YAW="${GOAL_YAW:-1.0}"
SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND:-continuous_mpcc_acados}"
SPMPC_W_SLOSH="${SPMPC_W_SLOSH:--1.0}"

mkdir -p "${OUT_ROOT}"

echo "================ P2P paper supplement ================"
echo "OUT_ROOT=${OUT_ROOT}"
echo "SPMPC_VARIANTS=${SPMPC_VARIANTS}"
echo "BASELINES=${BASELINES}"
echo "GOAL=(${GOAL_X}, ${GOAL_Y}, ${GOAL_YAW})"

run_one() {
  local baseline="$1"
  local variant="$2"
  local run_idx="$3"
  local run_dir="${OUT_ROOT}/${baseline}_${variant}_run${run_idx}"
  local run_id="$(date +%Y%m%d_%H%M%S)_p2p_${baseline}_${variant}_run${run_idx}"
  mkdir -p "${run_dir}"
  BASELINE="${baseline}" \
  VARIANT="${variant}" \
  OUT_DIR="${run_dir}" \
  RUN_ID="${run_id}" \
  RECORD_SEC="${RECORD_SEC}" \
  GOAL_X="${GOAL_X}" \
  GOAL_Y="${GOAL_Y}" \
  GOAL_YAW="${GOAL_YAW}" \
  SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND}" \
  SPMPC_W_SLOSH="${SPMPC_W_SLOSH}" \
  bash "${SCRIPT_DIR}/run_p2p_baseline_smoke.sh"
}

for run_idx in $(seq 1 "${RUNS}"); do
  for variant in ${SPMPC_VARIANTS}; do
    run_one spmpc "${variant}" "${run_idx}"
  done
  for baseline in ${BASELINES}; do
    if [[ "${baseline}" == "mpc" || "${baseline}" == "mpc_local_planner" ]]; then
      if [[ "${INCLUDE_MPC_LOCAL_PLANNER}" != "true" ]]; then
        echo "[skip] ${baseline}: INCLUDE_MPC_LOCAL_PLANNER=false"
        continue
      fi
    fi
    run_one "${baseline}" none "${run_idx}"
  done
done

echo "[done] P2P supplement -> ${OUT_ROOT}"
