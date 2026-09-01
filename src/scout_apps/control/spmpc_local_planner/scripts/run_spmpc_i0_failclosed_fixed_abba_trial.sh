#!/usr/bin/env bash
# Historical v1 entrypoint.  Its protocol identity and W5 selector semantics
# are frozen in the shared profile; new short100 experiments use a separate
# wrapper and output namespace.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export I0FC_ABBA_PROFILE=legacy_v1
exec bash "${SCRIPT_DIR}/lib/run_spmpc_i0_failclosed_fixed_abba_engine.sh" "$@"
