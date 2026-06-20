#!/usr/bin/env bash
# Shared read-only Phase-0 gate for fixed-path benchmark suites.
#
# This wrapper only runs bench_preflight.py in dry-run mode. It does not start
# ROS/Gazebo, launch planners, kill processes, write files, or change control
# behavior. Set SKIP_PHASE0_PREFLIGHT=true only for explicit current-sim
# diagnostics that must not be labeled as formal/main-table evidence.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Benchmark suites also use MODE for closed_loop/shadow execution. Keep the
# Phase-0 preflight mode separate so those runtime modes are not forwarded to
# bench_preflight.py, which only accepts sim/real.
if [[ -n "${PHASE0_PREFLIGHT_MODE+x}" ]]; then
  PREFLIGHT_MODE="${PHASE0_PREFLIGHT_MODE}"
elif [[ "${MODE:-}" == "sim" || "${MODE:-}" == "real" ]]; then
  PREFLIGHT_MODE="${MODE}"
else
  PREFLIGHT_MODE="sim"
fi
FORMAT="${PHASE0_PREFLIGHT_FORMAT:-yaml}"
STRICT_MAIN_TABLE="${STRICT_MAIN_TABLE:-false}"
SKIP_PHASE0_PREFLIGHT="${SKIP_PHASE0_PREFLIGHT:-false}"

if [[ "$#" -lt 1 ]]; then
  echo "[phase0][ERR] at least one planner/method id is required" >&2
  exit 2
fi

if [[ "${SKIP_PHASE0_PREFLIGHT}" == "true" ]]; then
  echo "[phase0][WARN] SKIP_PHASE0_PREFLIGHT=true; this run is diagnostics only and must not be used as formal/main-table evidence" >&2
  exit 0
fi

for planner in "$@"; do
  args=(--mode "${PREFLIGHT_MODE}" --planner "${planner}" --dry-run --format "${FORMAT}")
  if [[ "${STRICT_MAIN_TABLE}" == "true" ]]; then
    args+=(--strict-main-table)
  fi
  echo "[phase0] bench_preflight.py ${args[*]}"
  python3 "${SCRIPT_DIR}/bench_preflight.py" "${args[@]}"
done
