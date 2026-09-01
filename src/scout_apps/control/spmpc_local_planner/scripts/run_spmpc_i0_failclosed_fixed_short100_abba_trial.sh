#!/usr/bin/env bash
# Development-only short100-v2 B0/Bslosh ABBA entrypoint.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export I0FC_ABBA_PROFILE=short100_v2
exec bash "${SCRIPT_DIR}/lib/run_spmpc_i0_failclosed_fixed_abba_engine.sh" "$@"
