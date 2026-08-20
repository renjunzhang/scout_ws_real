#!/usr/bin/env bash
# Reproduce the frozen three-trial raw-RGB development source decision.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
source /opt/ros/noetic/setup.bash
source "${REPO_ROOT}/devel/setup.bash"

BAG_DIR="${BAG_DIR:-/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/H0s_Bsmooth}"
RELABEL_DIR="${RELABEL_DIR:-/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/relabel_20260731}"
CALIBRATION="${CALIBRATION:-/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml}"
OUT_DIR="${OUT_DIR:-/home/geist/slosh_bags/real/20260731_spmpc_g2s_rgb_diagnostic/analysis}"
HASH_BAGS="${HASH_BAGS:-true}"

hash_args=()
case "${HASH_BAGS}" in
  1|true|TRUE|yes|YES|on|ON) hash_args=(--hash-bags) ;;
  0|false|FALSE|no|NO|off|OFF) ;;
  *) echo "[G2S raw-RGB analysis][ERR] HASH_BAGS must be true or false" >&2; exit 2 ;;
esac

exec python3 "${SCRIPT_DIR}/../tools/analysis/analyze_g2s_raw_rgb_three_trial.py" \
  --trial u01 \
    "${BAG_DIR}/DEV_G2S_H0s_C1_Bsmooth_u01_a01.bag" \
    "${RELABEL_DIR}/offline_u01_attempt03_bottom60/DEV_G2S_H0s_C1_Bsmooth_u01_a01_red_top.csv" \
  --trial u02 \
    "${BAG_DIR}/DEV_G2S_H0s_C1_Bsmooth_u02_a01.bag" \
    "${RELABEL_DIR}/offline_u02_attempt03_bottom60/DEV_G2S_H0s_C1_Bsmooth_u02_a01_red_top.csv" \
  --trial u03 \
    "${BAG_DIR}/DEV_G2S_H0s_C1_Bsmooth_u03_a01.bag" \
    "${RELABEL_DIR}/offline_u03_attempt03_bottom60/DEV_G2S_H0s_C1_Bsmooth_u03_a01_red_top.csv" \
  --calibration "${CALIBRATION}" \
  --prereg "${BAG_DIR}/G2S_H0s_prereg.env" \
  --out-dir "${OUT_DIR}" \
  "${hash_args[@]}"
