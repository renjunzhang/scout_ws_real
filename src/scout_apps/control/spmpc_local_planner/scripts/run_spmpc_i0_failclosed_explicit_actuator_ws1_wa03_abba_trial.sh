#!/usr/bin/env bash
# Frozen RGB ABBA profile: explicit actuator, shared w_accel=0.3, treatment w_slosh=1.0.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export I0FC_ABBA_PROFILE=explicit_actuator_ws1_wa03_v2
exec bash "${SCRIPT_DIR}/lib/run_spmpc_i0_failclosed_fixed_abba_engine.sh" "$@"
