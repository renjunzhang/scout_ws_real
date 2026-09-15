#!/usr/bin/env bash
set -euo pipefail
PROFILE="${PROFILE:-raw_mpcc}"
REGION_CONFIG="${REGION_CONFIG:-}"
PLAN_FILE="${PLAN_FILE:-}"
OUT_DIR="${OUT_DIR:-/tmp/spmpc_trajectory_bags}"
NAME="${NAME:-trajectory_${PROFILE}_$(date +%Y%m%d_%H%M%S)}"
VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_CONFIG="${PROFILE_CONFIG:-${SCRIPT_DIR}/../config/experiments/trajectory_mpcc/${PROFILE}.yaml}"
TASK_CONFIG="${TASK_CONFIG:-${SCRIPT_DIR}/../config/experiments/trajectory_mpcc/trajectory_common.yaml}"
TASK_OVERLAY_FILE="${TASK_OVERLAY_FILE:-}"
PLANNER_OVERLAY_FILE="${PLANNER_OVERLAY_FILE:-}"
PLANNER_VARIANT="${PLANNER_VARIANT:-}"
PUBLISH_CMD_VEL="${PUBLISH_CMD_VEL:-true}"
LAUNCH_ARGS="${LAUNCH_ARGS:-}"
CONTRACT="${SCRIPT_DIR}/experiments/recording_contract.py"
case "$PROFILE" in raw_mpcc|geometry_mpcc|planned_mpcc|planned_slosh) ;; *) echo "invalid PROFILE=$PROFILE" >&2; exit 2;; esac
[[ -s "$PROFILE_CONFIG" ]] || { echo "PROFILE_CONFIG is required: $PROFILE_CONFIG" >&2; exit 2; }
[[ -s "$TASK_CONFIG" ]] || { echo "TASK_CONFIG is required: $TASK_CONFIG" >&2; exit 2; }
[[ -s "$REGION_CONFIG" ]] || { echo "REGION_CONFIG is required for all trajectory profiles" >&2; exit 2; }
if [[ "$PROFILE" == raw_mpcc ]]; then [[ -z "$PLAN_FILE" ]] || { echo "raw_mpcc cannot use a trajectory plan" >&2; exit 2; }; fi
# Plan presence is checked after merging; a task overlay may supply plan_file.
[[ -z "$TASK_OVERLAY_FILE" || -s "$TASK_OVERLAY_FILE" ]] || { echo "missing task overlay $TASK_OVERLAY_FILE" >&2; exit 2; }
[[ -z "$PLANNER_OVERLAY_FILE" || -s "$PLANNER_OVERLAY_FILE" ]] || { echo "missing planner overlay $PLANNER_OVERLAY_FILE" >&2; exit 2; }
mkdir -p "$OUT_DIR"
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || pwd)"
RECORDED_LAUNCH_ARGS="$LAUNCH_ARGS"
[[ -n "$TASK_OVERLAY_FILE" ]] && RECORDED_LAUNCH_ARGS="$RECORDED_LAUNCH_ARGS task_overlay_file:=$TASK_OVERLAY_FILE"
[[ -n "$PLANNER_OVERLAY_FILE" ]] && RECORDED_LAUNCH_ARGS="$RECORDED_LAUNCH_ARGS planner_overlay_file:=$PLANNER_OVERLAY_FILE"
python3 "$CONTRACT" prepare --profile "$PROFILE" --profile-config "$PROFILE_CONFIG" --task-config "$TASK_CONFIG" --task-overlay "$TASK_OVERLAY_FILE" --region "$REGION_CONFIG" --plan-file "$PLAN_FILE" --planner-overlay "$PLANNER_OVERLAY_FILE" --planner-variant "$PLANNER_VARIANT" --publish-cmd-vel "$PUBLISH_CMD_VEL" --launch-args "$RECORDED_LAUNCH_ARGS" --output-dir "$OUT_DIR" --name "$NAME" --repo "$REPO_ROOT"
MANIFEST="$OUT_DIR/${NAME}_manifest.json"
if [[ "$VALIDATE_ONLY" == true || "$VALIDATE_ONLY" == 1 ]]; then echo "validate-only PASS: $MANIFEST"; exit 0; fi
# The planner is launched separately; this script never starts ROS or motion.
python3 "$CONTRACT" verify-live --manifest "$MANIFEST"
command -v rosbag >/dev/null || { echo "rosbag not found" >&2; exit 2; }
rosbag record -O "$OUT_DIR/${NAME}.bag" \
  /spmpc/debug/effective_config /spmpc/debug/planning_config /spmpc/debug/pre_solve_snapshot /spmpc/debug/predicted_horizon /spmpc/debug/control_cycle_audit \
  /spmpc/status /spmpc/controller_variant /spmpc/experiment_mode /spmpc/solver_backend /spmpc/local_trajectory /scout/global_path /scout/global_path_fixed /odom /imu/data /cmd_vel /tf /tf_static \
  /spmpc/cost_breakdown /spmpc/solver_time_ms /spmpc/terminal/debug /spmpc/terminal/mode \
  /spmpc/debug/slosh_state /spmpc/slosh_height /spmpc/slosh_horizon_summary /spmpc/debug/slosh_cost_monitor \
  /spmpc/debug/slosh_observer_selection /spmpc/debug/slosh_observer_odom /spmpc/debug/slosh_observer_imu
