#!/usr/bin/env bash
# Batch runner for fixed-path critical scenario sweeps.
#
# PATH_MATRIX format:
#   "P2_s_curve:/data/a/fixed_paths/sim/P2_s_curve.json P3_uturn:/data/a/fixed_paths/sim/P3_uturn.json"

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OUT_ROOT="${OUT_ROOT:-/data/${USER}/spmpc_paper_compare/critical_path_sweep_$(date +%Y%m%d_%H%M%S)}"
PATH_MATRIX="${PATH_MATRIX:-P2_s_curve:/data/a/fixed_paths/sim/P2_s_curve.json}"
INTERNAL_VARIANTS="${INTERNAL_VARIANTS:-B0 B_ours}"
EXTERNAL_ANCHOR_VARIANTS="${EXTERNAL_ANCHOR_VARIANTS:-B0 B_ours}"
EXTERNAL_BASELINES="${EXTERNAL_BASELINES:-teb dwa}"
RUNS="${RUNS:-1}"
RECORD_SEC="${RECORD_SEC:-60}"
PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC:-30}"
SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE:-true}"
SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND:-continuous_mpcc_acados}"
SPMPC_W_SLOSH="${SPMPC_W_SLOSH:--1.0}"
INCLUDE_MPC_LOCAL_PLANNER="${INCLUDE_MPC_LOCAL_PLANNER:-false}"
INCLUDE_B_ACCEL="${INCLUDE_B_ACCEL:-false}"

mkdir -p "${OUT_ROOT}"

for item in ${PATH_MATRIX}; do
  path_id="${item%%:*}"
  path_file="${item#*:}"
  if [[ "${path_id}" == "${path_file}" || -z "${path_id}" || -z "${path_file}" ]]; then
    echo "[WARN] 跳过非法 PATH_MATRIX 项: ${item}" >&2
    continue
  fi
  if [[ ! -f "${path_file}" ]]; then
    echo "[WARN] 跳过不存在路径: ${path_id} -> ${path_file}" >&2
    continue
  fi

  path_out="${OUT_ROOT}/${path_id}"
  echo "================ critical path: ${path_id} ================"
  OUT_ROOT="${path_out}" \
  PATH_ID="${path_id}" \
  PATH_FILE="${path_file}" \
  RUNS="${RUNS}" \
  RECORD_SEC="${RECORD_SEC}" \
  PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC}" \
  SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE}" \
  SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND}" \
  SPMPC_W_SLOSH="${SPMPC_W_SLOSH}" \
  INTERNAL_VARIANTS="${INTERNAL_VARIANTS}" \
  EXTERNAL_ANCHOR_VARIANTS="${EXTERNAL_ANCHOR_VARIANTS}" \
  EXTERNAL_BASELINES="${EXTERNAL_BASELINES}" \
  INCLUDE_MPC_LOCAL_PLANNER="${INCLUDE_MPC_LOCAL_PLANNER}" \
  INCLUDE_B_ACCEL="${INCLUDE_B_ACCEL}" \
  bash "${SCRIPT_DIR}/run_fixed_path_paper_matrix.sh"
done

python3 "${SCRIPT_DIR}/extract_fixed_path_paper_metrics.py" \
  "${OUT_ROOT}" \
  --csv "${OUT_ROOT}/fixed_path_critical_metrics.csv" \
  --phase both \
  --path-topic /scout/global_path_fixed

echo "[done] critical path sweep -> ${OUT_ROOT}"
