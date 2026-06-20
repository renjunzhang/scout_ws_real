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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

VARIANTS="${VARIANTS:-B0 B_smooth B_slosh B_ours}"
SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND:-continuous_mpcc_acados}"
SPMPC_W_SLOSH="${SPMPC_W_SLOSH:--1.0}"
SPMPC_V_REF="${SPMPC_V_REF:--1.0}"
SPMPC_SHARED_LINEAR_ACCEL_LIMIT_ENABLE="${SPMPC_SHARED_LINEAR_ACCEL_LIMIT_ENABLE:-true}"
SPMPC_SHARED_LINEAR_ACCEL_MAX="${SPMPC_SHARED_LINEAR_ACCEL_MAX:--1.0}"
SPMPC_SHARED_LINEAR_ACCEL_MAX_DT="${SPMPC_SHARED_LINEAR_ACCEL_MAX_DT:--1.0}"
SPMPC_ALPHA_MAX="${SPMPC_ALPHA_MAX:--1.0}"
SPEED_TIER="${SPEED_TIER:-fair_common}"
LIMIT_PROFILE="${LIMIT_PROFILE:-common_v0p8_w1p2_a0p6_alpha1p2}"
TARGET_V_MAX_MPS="${TARGET_V_MAX_MPS:-0.8}"
TARGET_OMEGA_MAX_RADPS="${TARGET_OMEGA_MAX_RADPS:-1.2}"
TARGET_ACC_LIM_X_MPS2="${TARGET_ACC_LIM_X_MPS2:-0.6}"
TARGET_ACC_LIM_THETA_RADPS2="${TARGET_ACC_LIM_THETA_RADPS2:-1.2}"
OUT_ROOT="${OUT_ROOT:-/data/${USER}/spmpc_paper_compare/fixed_path_smoke}"
PATH_FILE="${PATH_FILE:-}"
PATH_ID="${PATH_ID:-fixed_path}"
PATH_SOURCE_MODE="${PATH_SOURCE_MODE:-replay}"  # replay | stable_goal
RUNS="${RUNS:-1}"
RECORD_SEC="${RECORD_SEC:-35}"
RUN_TIMEOUT_SEC="${RUN_TIMEOUT_SEC:-60}"
PATH_TOPIC="${PATH_TOPIC:-/scout/global_path_fixed}"
PATH_TEMPLATE="${PATH_TEMPLATE:-s_curve}"
START_HEADING="${START_HEADING:-current}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/current_start_fixed_goal}"
GOAL_FRAME="${GOAL_FRAME:-map}"
GOAL_X="${GOAL_X:-}"
GOAL_Y="${GOAL_Y:-}"
GOAL_YAW="${GOAL_YAW:-0.0}"
PATH_SPACING="${PATH_SPACING:-0.05}"
PATH_AMPLITUDE_RATIO="${PATH_AMPLITUDE_RATIO:-0.18}"
PATH_MIN_AMPLITUDE="${PATH_MIN_AMPLITUDE:-0.25}"
PATH_MAX_AMPLITUDE="${PATH_MAX_AMPLITUDE:-1.20}"
PATH_SIDE="${PATH_SIDE:-left}"
PATH_GENERATION_TIMEOUT_SEC="${PATH_GENERATION_TIMEOUT_SEC:-20}"
PATH_GENERATOR_SETTLE_SEC="${PATH_GENERATOR_SETTLE_SEC:-1}"
PATH_GENERATOR_PUBLISH_COUNT="${PATH_GENERATOR_PUBLISH_COUNT:-1}"
FEASIBILITY_ANALYZE="${FEASIBILITY_ANALYZE:-false}"
FEASIBILITY_V_REF="${FEASIBILITY_V_REF:-0.25}"
FEASIBILITY_OMEGA_MAX="${FEASIBILITY_OMEGA_MAX:-1.2}"
FEASIBILITY_FAIL_ON_OMEGA_LIMIT="${FEASIBILITY_FAIL_ON_OMEGA_LIMIT:-false}"
COSTMAP_TOPIC="${COSTMAP_TOPIC:-/map}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
REFERENCE_TARGET_FRAME="${REFERENCE_TARGET_FRAME:-}"
SKIP_START_WAIT="${SKIP_START_WAIT:-true}"
WAIT_READY_SEC="${WAIT_READY_SEC:-30}"
PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC:-0}"
SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE:-false}"
SLOSH_MONITOR_ODOM_TOPIC="${SLOSH_MONITOR_ODOM_TOPIC:-/odom}"
SLOSH_MONITOR_CMD_VEL_TOPIC="${SLOSH_MONITOR_CMD_VEL_TOPIC:-${CMD_VEL_TOPIC}}"
SLOSH_MONITOR_OUTPUT_NAMESPACE="${SLOSH_MONITOR_OUTPUT_NAMESPACE:-/benchmark/slosh_monitor}"
SKIP_PHASE0_PREFLIGHT="${SKIP_PHASE0_PREFLIGHT:-false}"
EXPERIMENT_GROUP="${EXPERIMENT_GROUP:-fixed_path_internal_ablation}"
EVIDENCE_CHAIN_VERSION="${EVIDENCE_CHAIN_VERSION:-20260605}"
SLOSH_RESET_BEFORE_RUN="${SLOSH_RESET_BEFORE_RUN:-true}"

planner_pid=""
path_pid=""
rec_pid=""
slosh_monitor_pid=""
generator_pid=""

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
  if [[ -n "${generator_pid}" ]]; then
    kill -INT "${generator_pid}" 2>/dev/null || true
    wait "${generator_pid}" 2>/dev/null || true
    generator_pid=""
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
  local probe_timeout_sec=3
  start="$(date +%s)"
  while true; do
    if timeout "${probe_timeout_sec}s" rostopic echo -n 1 "${status_topic}" >/dev/null 2>&1; then
      return 0
    fi
    if timeout "${probe_timeout_sec}s" rostopic echo -n 1 "${CMD_VEL_TOPIC}" >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      return 1
    fi
    sleep 0.5
  done
}

wait_file_nonempty() {
  local file="$1"
  local timeout_sec="$2"
  local start
  start="$(date +%s)"
  while true; do
    if [[ -s "${file}" ]]; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      echo "[ERR] ${timeout_sec}s 内没有生成 fixed path JSON: ${file}" >&2
      return 1
    fi
    sleep 0.2
  done
}

generate_stable_goal_path() {
  local run_path_file="$1"
  local run_dir="$2"
  local run_id="$3"
  mkdir -p "$(dirname "${run_path_file}")"
  echo "[stable_path] 当前位姿起点 -> 固定终点，生成 ${run_path_file}"
  rosrun scout_local_planner template_fixed_path_generator.py \
    --template "${PATH_TEMPLATE}" \
    --goal-topic "${GOAL_TOPIC}" \
    --output-topic "${PATH_TOPIC}" \
    --path-file "${run_path_file}" \
    --start-heading "${START_HEADING}" \
    --spacing "${PATH_SPACING}" \
    --amplitude-ratio "${PATH_AMPLITUDE_RATIO}" \
    --min-amplitude "${PATH_MIN_AMPLITUDE}" \
    --max-amplitude "${PATH_MAX_AMPLITUDE}" \
    --side "${PATH_SIDE}" \
    --publish-count "${PATH_GENERATOR_PUBLISH_COUNT}" \
    --wait-subscriber-timeout 0 \
    >"${run_dir}/${run_id}_path_generator.log" 2>&1 &
  generator_pid=$!
  sleep "${PATH_GENERATOR_SETTLE_SEC}"

  rosrun scout_local_planner send_fixed_goal.py \
    --goal-topic "${GOAL_TOPIC}" \
    --frame "${GOAL_FRAME}" \
    --x "${GOAL_X}" \
    --y "${GOAL_Y}" \
    --yaw "${GOAL_YAW}" \
    --repeat-count 1 \
    --repeat-rate 1 \
    --wait-subscriber-timeout 2 \
    >"${run_dir}/${run_id}_send_goal.log" 2>&1

  wait_file_nonempty "${run_path_file}" "${PATH_GENERATION_TIMEOUT_SEC}"
  if [[ -n "${generator_pid}" ]]; then
    kill -INT "${generator_pid}" 2>/dev/null || true
    wait "${generator_pid}" 2>/dev/null || true
    generator_pid=""
  fi
}

run_feasibility_analysis() {
  local run_path_file="$1"
  local run_dir="$2"
  local run_id="$3"
  if [[ "${FEASIBILITY_ANALYZE}" != "true" ]]; then
    return 0
  fi
  local analyzer
  analyzer="$(rospack find scout_local_planner)/scripts/analysis/analyze_fixed_path_feasibility.py"
  local args=(
    --path-file "${run_path_file}"
    --v-ref "${FEASIBILITY_V_REF}"
    --omega-max "${FEASIBILITY_OMEGA_MAX}"
    --alpha-max "${SPMPC_ALPHA_MAX}"
    --json-out "${run_dir}/${run_id}_feasibility.json"
    --csv-out "${run_dir}/${run_id}_feasibility.csv"
  )
  if [[ "${FEASIBILITY_FAIL_ON_OMEGA_LIMIT}" == "true" ]]; then
    args+=(--fail-on-omega-limit)
  fi
  echo "[feasibility] analyze_fixed_path_feasibility.py ${args[*]}"
  python3 "${analyzer}" "${args[@]}" >"${run_dir}/${run_id}_feasibility.log" 2>&1
}

reset_slosh_monitor() {
  if [[ "${SLOSH_MONITOR_ENABLE}" != "true" || "${SLOSH_RESET_BEFORE_RUN}" != "true" ]]; then
    return 0
  fi
  local service="${SLOSH_MONITOR_OUTPUT_NAMESPACE%/}/reset"
  if timeout 2s rosservice call "${service}" >/dev/null 2>&1; then
    echo "[slosh_monitor] reset ${service}"
  else
    echo "[WARN] ${service} 不可用，跳过本次 slosh monitor reset" >&2
  fi
}

preflight_id_for_variant() {
  case "$1" in
    B0) echo "spmpc_b0" ;;
    B_smooth) echo "spmpc_smooth_only" ;;
    B_slosh|B_ours) echo "spmpc_full" ;;
    *) echo "[ERR] unknown SPMPC variant for preflight: $1" >&2; return 2 ;;
  esac
}

run_phase0_preflight() {
  if [[ "${SKIP_PHASE0_PREFLIGHT}" == "true" ]]; then
    echo "[phase0][WARN] SKIP_PHASE0_PREFLIGHT=true; current run is diagnostics only"
  fi
  bash "${SCRIPT_DIR}/bench_run_phase0_preflight.sh" "$@"
}

case "${PATH_SOURCE_MODE}" in
  replay)
    if [[ -z "${PATH_FILE}" ]]; then
      echo "[ERR] replay 模式下 PATH_FILE 不能为空；请指定固定路径 JSON，例如 /data/a/fixed_paths/sim/P2_s_curve.json" >&2
      exit 2
    fi
    if [[ ! -f "${PATH_FILE}" ]]; then
      echo "[ERR] PATH_FILE 不存在: ${PATH_FILE}" >&2
      exit 2
    fi
    ;;
  stable_goal)
    if [[ -z "${GOAL_X}" || -z "${GOAL_Y}" ]]; then
      echo "[ERR] stable_goal 模式下 GOAL_X/GOAL_Y 不能为空" >&2
      exit 2
    fi
    ;;
  *)
    echo "[ERR] PATH_SOURCE_MODE 只能是 replay 或 stable_goal，当前=${PATH_SOURCE_MODE}" >&2
    exit 2
    ;;
esac
PREFLIGHT_VARIANTS=""
for variant in ${VARIANTS}; do
  PREFLIGHT_VARIANTS="${PREFLIGHT_VARIANTS} $(preflight_id_for_variant "${variant}")"
done
run_phase0_preflight ${PREFLIGHT_VARIANTS}

if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/仿真栈未检测到。请先启动仿真。" >&2
  exit 1
fi

mkdir -p "${OUT_ROOT}"

echo "================ SPMPC fixed-path suite ================"
echo "PATH_ID=${PATH_ID}"
echo "PATH_SOURCE_MODE=${PATH_SOURCE_MODE}"
echo "PATH_FILE=${PATH_FILE}"
echo "VARIANTS=${VARIANTS}"
echo "OUT_ROOT=${OUT_ROOT}"
echo "EXPERIMENT_GROUP=${EXPERIMENT_GROUP}"
echo "SPMPC_ALPHA_MAX=${SPMPC_ALPHA_MAX}"
echo "SPMPC_V_REF=${SPMPC_V_REF}"
echo "SPEED_TIER=${SPEED_TIER}"
echo "LIMIT_PROFILE=${LIMIT_PROFILE}"
echo "RUN_TIMEOUT_SEC=${RUN_TIMEOUT_SEC}"
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
  /benchmark/slosh_monitor/height
  /benchmark/slosh_monitor/state
  /benchmark/slosh_monitor/event
  /benchmark/slosh_monitor/debug
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
    run_path_file="${PATH_FILE}"
    path_original_file="${PATH_FILE}"
    stable_goal_enabled="false"
    if [[ "${PATH_SOURCE_MODE}" == "stable_goal" ]]; then
      run_path_file="${run_dir}/${run_id}_generated_path.json"
      path_original_file=""
      stable_goal_enabled="true"
    fi

    if [[ "${PRE_PATH_WAIT_SEC}" != "0" ]]; then
      echo "[settle] 等待定位/仿真稳定 ${PRE_PATH_WAIT_SEC}s 后再生成/发布 fixed path"
      sleep "${PRE_PATH_WAIT_SEC}"
    fi

    if [[ "${PATH_SOURCE_MODE}" == "stable_goal" ]]; then
      generate_stable_goal_path "${run_path_file}" "${run_dir}" "${run_id}"
    fi
    run_feasibility_analysis "${run_path_file}" "${run_dir}" "${run_id}"

    cat >"${meta}" <<EOF
run_id: ${run_id}
method: spmpc
variant: ${variant}
experiment_group: ${EXPERIMENT_GROUP}
evidence_chain_version: ${EVIDENCE_CHAIN_VERSION}
solver_backend: ${SPMPC_SOLVER_BACKEND}
w_slosh_override: ${SPMPC_W_SLOSH}
v_ref_override: ${SPMPC_V_REF}
speed_tier: ${SPEED_TIER}
limit_profile: ${LIMIT_PROFILE}
target_v_max_mps: ${TARGET_V_MAX_MPS}
target_omega_max_radps: ${TARGET_OMEGA_MAX_RADPS}
target_acc_lim_x_mps2: ${TARGET_ACC_LIM_X_MPS2}
target_acc_lim_theta_radps2: ${TARGET_ACC_LIM_THETA_RADPS2}
path_id: ${PATH_ID}
path_source_mode: ${PATH_SOURCE_MODE}
path_file: ${run_path_file}
path_original_file: ${path_original_file}
path_topic: ${PATH_TOPIC}
git_hash: ${git_hash}
record_sec: ${RECORD_SEC}
run_timeout_sec: ${RUN_TIMEOUT_SEC}
run_index: ${run_idx}
skip_phase0_preflight: ${SKIP_PHASE0_PREFLIGHT}
stable_goal_enabled: ${stable_goal_enabled}
template_name: ${PATH_TEMPLATE}
template_start_heading: ${START_HEADING}
template_goal_topic: ${GOAL_TOPIC}
template_goal_frame: ${GOAL_FRAME}
template_goal_x: ${GOAL_X}
template_goal_y: ${GOAL_Y}
template_goal_yaw: ${GOAL_YAW}
path_spacing: ${PATH_SPACING}
path_amplitude_ratio: ${PATH_AMPLITUDE_RATIO}
path_min_amplitude: ${PATH_MIN_AMPLITUDE}
path_max_amplitude: ${PATH_MAX_AMPLITUDE}
path_side: ${PATH_SIDE}
feasibility_analyze: ${FEASIBILITY_ANALYZE}
feasibility_v_ref: ${FEASIBILITY_V_REF}
feasibility_omega_max: ${FEASIBILITY_OMEGA_MAX}
slosh_monitor_enable: ${SLOSH_MONITOR_ENABLE}
slosh_monitor_odom_topic: ${SLOSH_MONITOR_ODOM_TOPIC}
slosh_monitor_cmd_vel_topic: ${SLOSH_MONITOR_CMD_VEL_TOPIC}
slosh_monitor_output_namespace: ${SLOSH_MONITOR_OUTPUT_NAMESPACE}
shared_linear_accel_limit_enable: ${SPMPC_SHARED_LINEAR_ACCEL_LIMIT_ENABLE}
shared_linear_accel_max: ${SPMPC_SHARED_LINEAR_ACCEL_MAX}
shared_linear_accel_max_dt: ${SPMPC_SHARED_LINEAR_ACCEL_MAX_DT}
alpha_max_override: ${SPMPC_ALPHA_MAX}
slosh_height_unit: m
slosh_eval_only: true
slosh_feedback_forbidden: true
external_baseline_uses_slosh: false
EOF

    if [[ "${SLOSH_MONITOR_ENABLE}" == "true" ]]; then
      echo "[slosh_monitor] roslaunch slosh_models slosh_monitor.launch odom_topic:=${SLOSH_MONITOR_ODOM_TOPIC} cmd_vel_topic:=${SLOSH_MONITOR_CMD_VEL_TOPIC} output_namespace:=${SLOSH_MONITOR_OUTPUT_NAMESPACE}"
      roslaunch slosh_models slosh_monitor.launch \
        odom_topic:="${SLOSH_MONITOR_ODOM_TOPIC}" \
        cmd_vel_topic:="${SLOSH_MONITOR_CMD_VEL_TOPIC}" \
        output_namespace:="${SLOSH_MONITOR_OUTPUT_NAMESPACE}" \
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

    path_args=(--mode replay --path-file "${run_path_file}" --output-topic "${PATH_TOPIC}" --publish-once-keepalive)
    if [[ "${SKIP_START_WAIT}" == "true" ]]; then
      path_args+=(--skip-start-wait)
    fi
    echo "[path] fixed_global_path_runner.py ${path_args[*]}"
    rosrun scout_local_planner fixed_global_path_runner.py "${path_args[@]}" \
      >"${run_dir}/${run_id}_path.log" 2>&1 &
    path_pid=$!
    sleep 1

    launch_args=(planner_variant:="${variant}" solver_backend:="${SPMPC_SOLVER_BACKEND}" reference_path_topic:="${PATH_TOPIC}" costmap_topic:="${COSTMAP_TOPIC}" cmd_vel_topic:="${CMD_VEL_TOPIC}" w_slosh:="${SPMPC_W_SLOSH}" v_ref:="${SPMPC_V_REF}" shared_linear_accel_limit_enable:="${SPMPC_SHARED_LINEAR_ACCEL_LIMIT_ENABLE}" shared_linear_accel_max:="${SPMPC_SHARED_LINEAR_ACCEL_MAX}" shared_linear_accel_max_dt:="${SPMPC_SHARED_LINEAR_ACCEL_MAX_DT}" alpha_max:="${SPMPC_ALPHA_MAX}")
    if [[ -n "${REFERENCE_TARGET_FRAME}" ]]; then
      launch_args+=(reference_target_frame:="${REFERENCE_TARGET_FRAME}")
    fi
    echo "[planner] roslaunch spmpc_local_planner spmpc_fixed_path.launch ${launch_args[*]}"
    roslaunch spmpc_local_planner spmpc_fixed_path.launch "${launch_args[@]}" \
      >"${run_dir}/${run_id}_planner.log" 2>&1 &
    planner_pid=$!
    run_start_epoch="$(date +%s)"

    if ! wait_status_or_cmd /spmpc/status "${WAIT_READY_SEC}"; then
      echo "[WARN] ${WAIT_READY_SEC}s 内未观察到 /spmpc/status 或 ${CMD_VEL_TOPIC}; 仍继续录包" >&2
    fi

    if [[ "${RUN_TIMEOUT_SEC}" == "0" ]]; then
      sleep "${RECORD_SEC}"
    else
      elapsed_sec=$(( $(date +%s) - run_start_epoch ))
      remaining_sec=$(( RUN_TIMEOUT_SEC - elapsed_sec ))
      if (( remaining_sec <= 0 )); then
        echo "[timeout] planner 启动后已超过 ${RUN_TIMEOUT_SEC}s，停止本 run"
      elif ! timeout "${remaining_sec}s" sleep "${RECORD_SEC}"; then
        echo "[timeout] planner 启动后达到 ${RUN_TIMEOUT_SEC}s，停止本 run"
      fi
    fi
    cleanup_run
    sleep 1
    echo "[done] ${run_id}"
  done
done

trap - EXIT
echo "[suite done] ${OUT_ROOT}"
