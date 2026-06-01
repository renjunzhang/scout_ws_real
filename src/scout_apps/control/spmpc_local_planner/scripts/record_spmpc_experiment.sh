#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${OUT_DIR:-/tmp/spmpc_bags}"
NAME="${NAME:-spmpc_smoke}"
mkdir -p "$OUT_DIR"

rosbag record -O "${OUT_DIR}/${NAME}.bag" \
  /spmpc/status \
  /spmpc/controller_variant \
  /spmpc/experiment_mode \
  /spmpc/local_trajectory \
  /spmpc/debug/progress_s \
  /spmpc/debug/slosh_state \
  /spmpc/slosh_horizon_summary \
  /spmpc/solver_time_ms \
  /spmpc/cost_breakdown \
  /cmd_vel \
  /odom \
  /scout/global_path \
  /scout/global_path_fixed \
  /slosh/height \
  /slosh/eta_x \
  /slosh/eta_y \
  /tf
