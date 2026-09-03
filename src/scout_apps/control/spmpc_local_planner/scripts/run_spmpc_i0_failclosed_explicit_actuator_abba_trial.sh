#!/usr/bin/env bash
# Development-only explicit-actuator B0/B_slosh ABBA entrypoint.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export I0FC_ABBA_PROFILE=explicit_actuator_v1
exec bash "${SCRIPT_DIR}/lib/run_spmpc_i0_failclosed_fixed_abba_engine.sh" "$@"
