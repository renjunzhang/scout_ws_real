#!/usr/bin/env bash
# Fixed-path simulation suite for SPMPC internal ablations.
#
# 前提：先启动仿真与定位，例如：
#   source devel/setup.bash
#   SIM_ENV=open USE_RVIZ=true SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
#     rosrun scout_local_planner launch_sim_nav_stack.sh
#
# 用法：
#   OUT_ROOT=/data/a/spmpc_paper_compare/fixed_path_smoke \
#   PATH_FILE=/data/a/fixed_paths/sim/P2_s_curve.json \
#   PATH_ID=P2_s_curve RUNS=1 \
#   bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_spmpc_suite.sh

set -euo pipefail

VARIANTS="${VARIANTS:-B0 B_smooth B_slosh B_ours}"
SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND:-continuous_mpcc_acados}"
SPMPC_W_SLOSH="${SPMPC_W_SLOSH:--1.0}"
SPMPC_SHARED_LINEAR_ACCEL_LIMIT_ENABLE="${SPMPC_SHARED_LINEAR_ACCEL_LIMIT_ENABLE:-true}"
SPMPC_SHARED_LINEAR_ACCEL_MAX="${SPMPC_SHARED_LINEAR_ACCEL_MAX:--1.0}"
SPMPC_SHARED_LINEAR_ACCEL_MAX_DT="${SPMPC_SHARED_LINEAR_ACCEL_MAX_DT:--1.0}"
OUT_ROOT="${OUT_ROOT:-/data/${USER}/spmpc_paper_compare/fixed_path_smoke}"
PATH_FILE="${PATH_FILE:-}"
PATH_ID="${PATH_ID:-fixed_path}"
RUNS="${RUNS:-1}"
RECORD_SEC="${RECORD_SEC:-35}"
PATH_TOPIC="${PATH_TOPIC:-/scout/global_path_fixed}"
COSTMAP_TOPIC="${COSTMAP_TOPIC:-/map}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
REFERENCE_TARGET_FRAME="${REFERENCE_TARGET_FRAME:-}"
SKIP_START_WAIT="${SKIP_START_WAIT:-true}"
WAIT_READY_SEC="${WAIT_READY_SEC:-30}"
PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC:-0}"
SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE:-false}"
SLOSH_MONITOR_ODOM_TOPIC="${SLOSH_MONITOR_ODOM_TOPIC:-/odom}"
SLOSH_MONITOR_CMD_VEL_TOPIC="${SLOSH_MONITOR_CMD_VEL_TOPIC:-${CMD_VEL_TOPIC}}"
EXPERIMENT_GROUP="${EXPERIMENT_GROUP:-fixed_path_internal_ablation}"
EVIDENCE_CHAIN_VERSION="${EVIDENCE_CHAIN_VERSION:-20260605}"
SLOSH_RESET_BEFORE_RUN="${SLOSH_RESET_BEFORE_RUN:-true}"

planner_pid=""
path_pid=""
rec_pid=""
slosh_monitor_pid=""

cleanup_run() {
  if [[ -n "${rec_pid}" ]]; then
    kill -INT "${rec_pid}" 2>/dev/null || true
    wait "${rec_pid}" 2>/dev/null || true
    rec_pid=""
  fi
  if [[ -n "${planner_pid}" ]]; then
    kill -INT "${planner_pid}" 2>/dev/null || true
    wait "${planner_pid}" 2>/dev/null || true
    planner_pid=""
  fi
  if [[ -n "${path_pid}" ]]; then
    kill -INT "${path_pid}" 2>/dev/null || true
    wait "${path_pid}" 2>/dev/null || true
    path_pid=""
  fi
  if [[ -n "${slosh_monitor_pid}" ]]; then
    kill -INT "${slosh_monitor_pid}" 2>/dev/null || true
    wait "${slosh_monitor_pid}" 2>/dev/null || true
    slosh_monitor_pid=""
  fi
}
trap cleanup_run EXIT

wait_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  if ! timeout "${timeout_sec}s" rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "[ERR] ${timeout_sec}s 内没有收到 ${topic}" >&2
    return 1
  fi
}

wait_status_or_cmd() {
  local status_topic="$1"
  local timeout_sec="$2"
  local start
  start="$(date +%s)"
  while true; do
    if timeout 1s rostopic echo -n 1 "${status_topic}" >/dev/null 2>&1; then
      return 0
    fi
    if timeout 1s rostopic echo -n 1 "${CMD_VEL_TOPIC}" >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      return 1
    fi
    sleep 0.5
  done
}

reset_slosh_monitor() {
  if [[ "${SLOSH_MONITOR_ENABLE}" != "true" || "${SLOSH_RESET_BEFORE_RUN}" != "true" ]]; then
    return 0
  fi
  if timeout 2s rosservice call /slosh/reset >/dev/null 2>&1; then
    echo "[slosh_monitor] reset /slosh/reset"
  else
    echo "[WARN] /slosh/reset 不可用，跳过本次 slosh monitor reset" >&2
  fi
}

if [[ -z "${PATH_FILE}" ]]; then
  echo "[ERR] PATH_FILE 不能为空；请指定固定路径 JSON，例如 /data/a/fixed_paths/sim/P2_s_curve.json" >&2
  exit 2
fi
if [[ ! -f "${PATH_FILE}" ]]; then
  echo "[ERR] PATH_FILE 不存在: ${PATH_FILE}" >&2
  exit 2
fi
if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/仿真栈未检测到。请先启动仿真。" >&2
  exit 1
fi

mkdir -p "${OUT_ROOT}"

echo "================ SPMPC fixed-path suite ================"
echo "PATH_ID=${PATH_ID}"
echo "PATH_FILE=${PATH_FILE}"
echo "VARIANTS=${VARIANTS}"
echo "OUT_ROOT=${OUT_ROOT}"
echo "EXPERIMENT_GROUP=${EXPERIMENT_GROUP}"
echo "[preflight] 等待 /odom ${COSTMAP_TOPIC} ..."
wait_topic_once /odom 10
wait_topic_once "${COSTMAP_TOPIC}" 10

git_hash="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"

record_topics=(
  /clock
  "${CMD_VEL_TOPIC}"
  /odom
  "${COSTMAP_TOPIC}"
  /scan_front
  "${PATH_TOPIC}"
  /spmpc/status
  /spmpc/controller_variant
  /spmpc/experiment_mode
  /spmpc/solver_backend
  /spmpc/local_trajectory
  /spmpc/solver_time_ms
  /spmpc/cost_breakdown
  /spmpc/slosh_height
  /spmpc/slosh_horizon_summary
  /spmpc/debug/slosh_state
  /spmpc/debug/progress_s
  /spmpc/debug/warm_start
  /spmpc/debug/warm_start_status
  /spmpc/terminal/debug
  /spmpc/terminal/mode
  /slosh/height
  /slosh/state
  /slosh/debug
  /tf
  /tf_static
)

for run_idx in $(seq 1 "${RUNS}"); do
  for variant in ${VARIANTS}; do
    run_id="$(date +%Y%m%d_%H%M%S)_spmpc_${variant}_${PATH_ID}_run${run_idx}"
    run_dir="${OUT_ROOT}/${run_id}"
    mkdir -p "${run_dir}"
    bag="${run_dir}/${run_id}.bag"
    meta="${run_dir}/${run_id}_meta.yaml"

    echo "---------------- ${run_id} ----------------"
    cat >"${meta}" <<EOF
run_id: ${run_id}
method: spmpc
variant: ${variant}
experiment_group: ${EXPERIMENT_GROUP}
evidence_chain_version: ${EVIDENCE_CHAIN_VERSION}
solver_backend: ${SPMPC_SOLVER_BACKEND}
w_slosh_override: ${SPMPC_W_SLOSH}
path_id: ${PATH_ID}
path_file: ${PATH_FILE}
path_topic: ${PATH_TOPIC}
git_hash: ${git_hash}
record_sec: ${RECORD_SEC}
run_index: ${run_idx}
slosh_monitor_enable: ${SLOSH_MONITOR_ENABLE}
slosh_monitor_odom_topic: ${SLOSH_MONITOR_ODOM_TOPIC}
slosh_monitor_cmd_vel_topic: ${SLOSH_MONITOR_CMD_VEL_TOPIC}
shared_linear_accel_limit_enable: ${SPMPC_SHARED_LINEAR_ACCEL_LIMIT_ENABLE}
shared_linear_accel_max: ${SPMPC_SHARED_LINEAR_ACCEL_MAX}
shared_linear_accel_max_dt: ${SPMPC_SHARED_LINEAR_ACCEL_MAX_DT}
slosh_height_unit: m
slosh_eval_only: true
slosh_feedback_forbidden: true
external_baseline_uses_slosh: false
EOF

    if [[ "${PRE_PATH_WAIT_SEC}" != "0" ]]; then
      echo "[settle] 等待定位/仿真稳定 ${PRE_PATH_WAIT_SEC}s 后再开始录包和发布 fixed path"
      sleep "${PRE_PATH_WAIT_SEC}"
    fi

    if [[ "${SLOSH_MONITOR_ENABLE}" == "true" ]]; then
      echo "[slosh_monitor] roslaunch slosh_models slosh_monitor.launch odom_topic:=${SLOSH_MONITOR_ODOM_TOPIC} cmd_vel_topic:=${SLOSH_MONITOR_CMD_VEL_TOPIC}"
      roslaunch slosh_models slosh_monitor.launch \
        odom_topic:="${SLOSH_MONITOR_ODOM_TOPIC}" \
        cmd_vel_topic:="${SLOSH_MONITOR_CMD_VEL_TOPIC}" \
        >"${run_dir}/${run_id}_slosh_monitor.log" 2>&1 &
      slosh_monitor_pid=$!
      sleep 1
      reset_slosh_monitor
    fi

    echo "[rec] ${bag}"
    rosbag record -O "${bag}" "${record_topics[@]}" \
      >"${run_dir}/${run_id}_rosbag.log" 2>&1 &
    rec_pid=$!
    sleep 1

    path_args=(--mode replay --path-file "${PATH_FILE}" --output-topic "${PATH_TOPIC}" --publish-once-keepalive)
    if [[ "${SKIP_START_WAIT}" == "true" ]]; then
      path_args+=(--skip-start-wait)
    fi
    echo "[path] fixed_global_path_runner.py ${path_args[*]}"
    rosrun scout_local_planner fixed_global_path_runner.py "${path_args[@]}" \
      >"${run_dir}/${run_id}_path.log" 2>&1 &
    path_pid=$!
    sleep 1

    launch_args=(planner_variant:="${variant}" solver_backend:="${SPMPC_SOLVER_BACKEND}" reference_path_topic:="${PATH_TOPIC}" costmap_topic:="${COSTMAP_TOPIC}" cmd_vel_topic:="${CMD_VEL_TOPIC}" w_slosh:="${SPMPC_W_SLOSH}" shared_linear_accel_limit_enable:="${SPMPC_SHARED_LINEAR_ACCEL_LIMIT_ENABLE}" shared_linear_accel_max:="${SPMPC_SHARED_LINEAR_ACCEL_MAX}" shared_linear_accel_max_dt:="${SPMPC_SHARED_LINEAR_ACCEL_MAX_DT}")
    if [[ -n "${REFERENCE_TARGET_FRAME}" ]]; then
      launch_args+=(reference_target_frame:="${REFERENCE_TARGET_FRAME}")
    fi
    echo "[planner] roslaunch spmpc_local_planner spmpc_fixed_path.launch ${launch_args[*]}"
    roslaunch spmpc_local_planner spmpc_fixed_path.launch "${launch_args[@]}" \
      >"${run_dir}/${run_id}_planner.log" 2>&1 &
    planner_pid=$!

    if ! wait_status_or_cmd /spmpc/status "${WAIT_READY_SEC}"; then
      echo "[WARN] ${WAIT_READY_SEC}s 内未观察到 /spmpc/status 或 ${CMD_VEL_TOPIC}; 仍继续录包" >&2
    fi

    sleep "${RECORD_SEC}"
    cleanup_run
    sleep 1
    echo "[done] ${run_id}"
  done
done

trap - EXIT
echo "[suite done] ${OUT_ROOT}"
