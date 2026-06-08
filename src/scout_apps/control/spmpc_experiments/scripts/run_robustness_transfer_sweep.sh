#!/usr/bin/env bash
# Small robustness / transfer sweep harness.
#
# 第一版保持轻量：脚本统一目录、meta 和调用入口；需要改变仿真 spawn pose
# 的 perturbation 会打印明确提示，由用户 fresh 启动仿真后再运行。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OUT_ROOT="${OUT_ROOT:-/data/${USER}/spmpc_paper_compare/robustness_transfer_$(date +%Y%m%d_%H%M%S)}"
PATH_FILE="${PATH_FILE:-/data/a/fixed_paths/sim/P2_s_curve.json}"
PATH_ID="${PATH_ID:-P2_s_curve}"
PERTURBATIONS="${PERTURBATIONS:-nominal yaw_plus_5deg yaw_minus_5deg w_slosh_low w_slosh_high}"
VARIANTS="${VARIANTS:-B0 B_ours}"
RUNS="${RUNS:-1}"
RECORD_SEC="${RECORD_SEC:-60}"
PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC:-30}"
SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE:-true}"
SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND:-continuous_mpcc_acados}"

mkdir -p "${OUT_ROOT}"

if [[ ! -f "${PATH_FILE}" ]]; then
  echo "[ERR] PATH_FILE 不存在: ${PATH_FILE}" >&2
  exit 2
fi

write_manifest() {
  local out_dir="$1"
  local perturb="$2"
  local note="$3"
  cat >"${out_dir}/sweep_meta.yaml" <<EOF
experiment_group: robustness_transfer
sweep_name: ${perturb}
sweep_value: ${perturb}
path_id: ${PATH_ID}
path_file: ${PATH_FILE}
variants: ${VARIANTS}
solver_backend: ${SPMPC_SOLVER_BACKEND}
slosh_monitor_enable: ${SLOSH_MONITOR_ENABLE}
slosh_eval_only: true
note: ${note}
EOF
}

for perturb in ${PERTURBATIONS}; do
  perturb_out="${OUT_ROOT}/${perturb}"
  mkdir -p "${perturb_out}"
  w_slosh="-1.0"
  note="nominal"

  case "${perturb}" in
    nominal)
      note="nominal fixed-path run"
      ;;
    yaw_plus_5deg)
      note="fresh sim should be started with start yaw +5deg before running this group"
      ;;
    yaw_minus_5deg)
      note="fresh sim should be started with start yaw -5deg before running this group"
      ;;
    w_slosh_low)
      w_slosh="2.0"
      note="lower slosh weight transfer point"
      ;;
    w_slosh_high)
      w_slosh="4.0"
      note="higher slosh weight transfer point"
      ;;
    *)
      note="custom perturbation; ensure sim/config matches this label"
      ;;
  esac

  write_manifest "${perturb_out}" "${perturb}" "${note}"
  echo "================ robustness perturbation: ${perturb} ================"
  echo "[note] ${note}"

  OUT_ROOT="${perturb_out}" \
  PATH_FILE="${PATH_FILE}" \
  PATH_ID="${PATH_ID}_${perturb}" \
  RUNS="${RUNS}" \
  RECORD_SEC="${RECORD_SEC}" \
  PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC}" \
  SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE}" \
  SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND}" \
  SPMPC_W_SLOSH="${w_slosh}" \
  VARIANTS="${VARIANTS}" \
  EXPERIMENT_GROUP="robustness_transfer" \
  bash "${SCRIPT_DIR}/run_fixed_path_spmpc_suite.sh"
done

python3 "${SCRIPT_DIR}/extract_fixed_path_paper_metrics.py" \
  "${OUT_ROOT}" \
  --csv "${OUT_ROOT}/robustness_transfer_metrics.csv" \
  --phase both \
  --path-topic /scout/global_path_fixed

echo "[done] robustness / transfer sweep -> ${OUT_ROOT}"
