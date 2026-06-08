#!/usr/bin/env bash
# Paper-facing fixed-path matrix runner.
#
# 这个脚本只编排已有 suite，不负责启动/重启 Gazebo。正式数据仍建议：
# 一个方法/一组方法跑完后 fresh 启动仿真，避免定位、odom、costmap 或 planner 状态残留。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OUT_ROOT="${OUT_ROOT:-/data/${USER}/spmpc_paper_compare/fixed_path_matrix_$(date +%Y%m%d_%H%M%S)}"
PATH_FILE="${PATH_FILE:-}"
PATH_ID="${PATH_ID:-fixed_path}"
RUNS="${RUNS:-1}"
RECORD_SEC="${RECORD_SEC:-60}"
PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC:-30}"
PATH_TOPIC="${PATH_TOPIC:-/scout/global_path_fixed}"
COSTMAP_TOPIC="${COSTMAP_TOPIC:-/map}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE:-true}"
SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND:-continuous_mpcc_acados}"
SPMPC_W_SLOSH="${SPMPC_W_SLOSH:--1.0}"
INTERNAL_VARIANTS="${INTERNAL_VARIANTS:-B0 B_smooth B_slosh B_ours}"
EXTERNAL_ANCHOR_VARIANTS="${EXTERNAL_ANCHOR_VARIANTS:-B0 B_ours}"
EXTERNAL_BASELINES="${EXTERNAL_BASELINES:-teb dwa}"
INCLUDE_MPC_LOCAL_PLANNER="${INCLUDE_MPC_LOCAL_PLANNER:-false}"
INCLUDE_B_ACCEL="${INCLUDE_B_ACCEL:-false}"
B_ACCEL_VARIANTS="${B_ACCEL_VARIANTS:-B_accel}"
EXTRACT_METRICS="${EXTRACT_METRICS:-true}"

if [[ -z "${PATH_FILE}" ]]; then
  echo "[ERR] PATH_FILE 不能为空，例如 /data/a/fixed_paths/sim/P2_s_curve.json" >&2
  exit 2
fi
if [[ ! -f "${PATH_FILE}" ]]; then
  echo "[ERR] PATH_FILE 不存在: ${PATH_FILE}" >&2
  exit 2
fi
if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/仿真栈未检测到。请先启动仿真，并等待定位/TF/costmap 稳定。" >&2
  exit 1
fi

mkdir -p "${OUT_ROOT}"

cat <<EOF
================ fixed-path paper matrix ================
OUT_ROOT=${OUT_ROOT}
PATH_ID=${PATH_ID}
PATH_FILE=${PATH_FILE}
RUNS=${RUNS}
RECORD_SEC=${RECORD_SEC}
PRE_PATH_WAIT_SEC=${PRE_PATH_WAIT_SEC}
SLOSH_MONITOR_ENABLE=${SLOSH_MONITOR_ENABLE}
SPMPC_SOLVER_BACKEND=${SPMPC_SOLVER_BACKEND}
INTERNAL_VARIANTS=${INTERNAL_VARIANTS}
EXTERNAL_ANCHOR_VARIANTS=${EXTERNAL_ANCHOR_VARIANTS}
EXTERNAL_BASELINES=${EXTERNAL_BASELINES}
INCLUDE_MPC_LOCAL_PLANNER=${INCLUDE_MPC_LOCAL_PLANNER}
INCLUDE_B_ACCEL=${INCLUDE_B_ACCEL}

[注意] 正式/半正式对比建议每个方法或方法组 fresh 启动仿真；本脚本只做统一编排。
EOF

common_env=(
  OUT_ROOT="${OUT_ROOT}"
  PATH_FILE="${PATH_FILE}"
  PATH_ID="${PATH_ID}"
  RUNS="${RUNS}"
  RECORD_SEC="${RECORD_SEC}"
  PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC}"
  PATH_TOPIC="${PATH_TOPIC}"
  COSTMAP_TOPIC="${COSTMAP_TOPIC}"
  CMD_VEL_TOPIC="${CMD_VEL_TOPIC}"
  SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE}"
  EVIDENCE_CHAIN_VERSION="20260605"
)

run_spmpc_group() {
  local group="$1"
  local variants="$2"
  echo "[group] ${group}: ${variants}"
  env "${common_env[@]}" \
    EXPERIMENT_GROUP="${group}" \
    VARIANTS="${variants}" \
    SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND}" \
    SPMPC_W_SLOSH="${SPMPC_W_SLOSH}" \
    bash "${SCRIPT_DIR}/run_fixed_path_spmpc_suite.sh"
}

run_baseline_group() {
  local baseline="$1"
  echo "[baseline] ${baseline}"
  env "${common_env[@]}" \
    EXPERIMENT_GROUP="fixed_path_external_baseline" \
    BASELINE="${baseline}" \
    INCLUDE_MPC_LOCAL_PLANNER="${INCLUDE_MPC_LOCAL_PLANNER}" \
    bash "${SCRIPT_DIR}/run_fixed_path_baseline_suite.sh"
}

run_spmpc_group fixed_path_internal_ablation "${INTERNAL_VARIANTS}"
run_spmpc_group fixed_path_external_anchor "${EXTERNAL_ANCHOR_VARIANTS}"

for baseline in ${EXTERNAL_BASELINES}; do
  if [[ "${baseline}" == "mpc_local_planner" || "${baseline}" == "mpc" ]]; then
    if [[ "${INCLUDE_MPC_LOCAL_PLANNER}" != "true" ]]; then
      echo "[skip] ${baseline}: INCLUDE_MPC_LOCAL_PLANNER=false"
      continue
    fi
  fi
  run_baseline_group "${baseline}"
done

if [[ "${INCLUDE_B_ACCEL}" == "true" ]]; then
  run_spmpc_group fixed_path_accel_proxy_supplement "${B_ACCEL_VARIANTS}"
fi

if [[ "${EXTRACT_METRICS}" == "true" ]]; then
  echo "[metrics] extract_fixed_path_paper_metrics.py"
  python3 "${SCRIPT_DIR}/extract_fixed_path_paper_metrics.py" \
    "${OUT_ROOT}" \
    --csv "${OUT_ROOT}/fixed_path_metrics.csv" \
    --phase both \
    --path-topic "${PATH_TOPIC}"
fi

echo "[done] fixed-path paper matrix -> ${OUT_ROOT}"
