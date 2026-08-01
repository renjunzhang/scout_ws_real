#!/usr/bin/env bash
# One-command, read-only analysis after all four G2S rows have PASS postflights.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"

[[ -r /opt/ros/noetic/setup.bash ]]
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]]
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

G2S_DATE="${G2S_DATE:-20260731}"
BAG_DIR="${BAG_DIR:-/home/geist/slosh_bags/real/${G2S_DATE}_spmpc_g2s_source_selection/H0s_Bsmooth}"
CALIBRATION="${CALIBRATION:-/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_frozen.yaml}"
OUT_DIR="${OUT_DIR:-/home/geist/slosh_bags/real/${G2S_DATE}_spmpc_g2s_source_selection/analysis}"

exec python3 "${SCRIPT_DIR}/analysis/analyze_g2s_source_selection.py" \
  --bag-dir "${BAG_DIR}" \
  --calibration "${CALIBRATION}" \
  --out-dir "${OUT_DIR}"
