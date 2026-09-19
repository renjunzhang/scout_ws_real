#!/usr/bin/env bash
# Development-only short100-v2 B0/Bslosh ABBA entrypoint.

set -euo pipefail

# Shared scripts/ root for cross-category calls.
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
export I0FC_ABBA_PROFILE=short100_v2
exec bash "${SCRIPT_DIR}/lib/run_spmpc_i0_failclosed_fixed_abba_engine.sh" "$@"
