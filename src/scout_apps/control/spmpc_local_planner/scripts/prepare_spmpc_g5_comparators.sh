#!/usr/bin/env bash
# Offline-only G5 comparator preparation.  No ROS master or robot motion.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
G3_ROOT="${G3_ROOT:-/home/geist/slosh_bags/real/20260801_spmpc_g3_processed_imu_w5_vs_bsmooth/H0}"
G3_REPORT="${G3_REPORT:-${G3_ROOT}/G3_EFFICACY_REPORT.json}"
PATH_FILE="${PATH_FILE:-/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json}"
G5_CONFIG="${G5_CONFIG:-${SCRIPT_DIR}/../tools/analysis/g5_comparator_config.yaml}"
G5_OUT_DIR="${G5_OUT_DIR:-/home/geist/slosh_bags/real/20260801_spmpc_g5_minimal/H0}"

for artifact in "${G3_REPORT}" "${PATH_FILE}" "${G5_CONFIG}"; do
  [[ -s "${artifact}" ]] || { echo "[G5][ERR] missing: ${artifact}" >&2; exit 2; }
done

python3 "${SCRIPT_DIR}/../tools/analysis/prepare_g5_comparators.py" \
  --g3-report "${G3_REPORT}" \
  --path-file "${PATH_FILE}" \
  --config "${G5_CONFIG}" \
  --repo-root "${REPO_ROOT}" \
  --out-dir "${G5_OUT_DIR}"
