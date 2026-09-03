#!/usr/bin/env bash
# Dedicated fair B0/B_slosh ABBA entry on frozen C02.
# Row order: 01 B0 -> 02 B_slosh -> 03 B_slosh -> 04 B0.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGINE="${SCRIPT_DIR}/run_spmpc_o0_l22_bsmooth_bours_trial.sh"

[[ -s "${ENGINE}" ]] || {
  echo "[run_spmpc_o0_l22_b0_bslosh_trial][ERR] missing engine: ${ENGINE}" >&2
  exit 2
}

export PAIR_PROFILE=b0_bslosh
export SCRIPT_NAME_OVERRIDE=run_spmpc_o0_l22_b0_bslosh_trial
exec bash "${ENGINE}" "$@"
