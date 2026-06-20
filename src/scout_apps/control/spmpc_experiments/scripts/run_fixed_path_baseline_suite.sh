#!/usr/bin/env bash
# Fixed-path simulation suite for external baselines (TEB / DWA / mpc_local_planner / LT-DWA adapter).
#
# 前提：先启动仿真与定位，例如：
#   source devel/setup.bash
#   SIM_ENV=open USE_RVIZ=true SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
#     rosrun scout_local_planner launch_sim_nav_stack.sh
#
# 用法：
#   BASELINE=teb OUT_ROOT=/data/a/spmpc_paper_compare/fixed_path_smoke \
#   PATH_FILE=/data/a/fixed_paths/sim/P2_s_curve.json PATH_ID=P2_s_curve RUNS=1 \
#   bash src/scout_apps/control/spmpc_experiments/scripts/run_fixed_path_baseline_suite.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BASELINE="${BASELINE:-teb}"  # teb | dwa | mpc | mpc_local_planner | lt_dwa | all
OUT_ROOT="${OUT_ROOT:-/data/${USER}/spmpc_paper_compare/fixed_path_smoke}"
PATH_FILE="${PATH_FILE:-}"
PATH_ID="${PATH_ID:-fixed_path}"
RUNS="${RUNS:-1}"
RECORD_SEC="${RECORD_SEC:-35}"
PATH_TOPIC="${PATH_TOPIC:-/scout/global_path_fixed}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
COSTMAP_TOPIC="${COSTMAP_TOPIC:-/map}"
SKIP_START_WAIT="${SKIP_START_WAIT:-true}"
WAIT_READY_SEC="${WAIT_READY_SEC:-30}"
PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC:-0}"
SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE:-false}"
SLOSH_MONITOR_ODOM_TOPIC="${SLOSH_MONITOR_ODOM_TOPIC:-/odom}"
SLOSH_MONITOR_CMD_VEL_TOPIC="${SLOSH_MONITOR_CMD_VEL_TOPIC:-${CMD_VEL_TOPIC}}"
SLOSH_MONITOR_OUTPUT_NAMESPACE="${SLOSH_MONITOR_OUTPUT_NAMESPACE:-/benchmark/slosh_monitor}"
SKIP_PHASE0_PREFLIGHT="${SKIP_PHASE0_PREFLIGHT:-false}"
EXPERIMENT_GROUP="${EXPERIMENT_GROUP:-fixed_path_external_baseline}"
EVIDENCE_CHAIN_VERSION="${EVIDENCE_CHAIN_VERSION:-20260605}"
SPEED_TIER="${SPEED_TIER:-fair_common}"
LIMIT_PROFILE="${LIMIT_PROFILE:-common_v0p8_w1p2_a0p6_alpha1p2}"
TARGET_V_MAX_MPS="${TARGET_V_MAX_MPS:-0.8}"
TARGET_OMEGA_MAX_RADPS="${TARGET_OMEGA_MAX_RADPS:-1.2}"
TARGET_ACC_LIM_X_MPS2="${TARGET_ACC_LIM_X_MPS2:-0.6}"
TARGET_ACC_LIM_THETA_RADPS2="${TARGET_ACC_LIM_THETA_RADPS2:-1.2}"
INCLUDE_MPC_LOCAL_PLANNER="${INCLUDE_MPC_LOCAL_PLANNER:-false}"
INCLUDE_LT_DWA="${INCLUDE_LT_DWA:-false}"
LT_DWA_PUBLISH_CMD_VEL="${LT_DWA_PUBLISH_CMD_VEL:-true}"
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
  local service="${SLOSH_MONITOR_OUTPUT_NAMESPACE%/}/reset"
  if timeout 2s rosservice call "${service}" >/dev/null 2>&1; then
    echo "[slosh_monitor] reset ${service}"
  else
    echo "[WARN] ${service} 不可用，跳过本次 slosh monitor reset" >&2
  fi
}

launch_for_baseline() {
  case "$1" in
    teb)
      echo "run_teb_fixed_path_sim.launch"
      ;;
    dwa)
      echo "run_dwa_fixed_path_sim.launch"
      ;;
    mpc|mpc_local_planner)
      echo "run_mpc_local_planner_fixed_path_sim.launch"
      ;;
    lt_dwa)
      echo "run_lt_dwa_fixed_path_sim.launch"
      ;;
    *)
      echo "[ERR] unknown baseline: $1" >&2
      return 2
      ;;
  esac
}

status_topic_for_baseline() {
  case "$1" in
    teb) echo "/baseline/teb/status" ;;
    dwa) echo "/baseline/dwa/status" ;;
    mpc|mpc_local_planner) echo "/baseline/mpc_local_planner/status" ;;
    lt_dwa) echo "/baseline/lt_dwa/status" ;;
  esac
}

config_for_baseline() {
  case "$1" in
    teb) echo "config/baselines/teb_local_planner_standalone_sim.yaml" ;;
    dwa) echo "config/baselines/dwa_local_planner_standalone_sim.yaml" ;;
    mpc|mpc_local_planner) echo "config/baselines/mpc_local_planner_standalone_sim.yaml" ;;
    lt_dwa) echo "config/baselines/lt_dwa_adapter_standalone_sim.yaml" ;;
  esac
}

preflight_id_for_baseline() {
  case "$1" in
    mpc) echo "mpc_local_planner" ;;
    *) echo "$1" ;;
  esac
}

plan_target_frame_for_baseline() {
  case "$1" in
    *) echo "odom" ;;
  esac
}

run_phase0_preflight() {
  if [[ "${SKIP_PHASE0_PREFLIGHT}" == "true" ]]; then
    echo "[phase0][WARN] SKIP_PHASE0_PREFLIGHT=true; current run is diagnostics only"
  fi
  bash "${SCRIPT_DIR}/bench_run_phase0_preflight.sh" "$@"
}

if [[ -z "${PATH_FILE}" ]]; then
  echo "[ERR] PATH_FILE 不能为空；请指定固定路径 JSON，例如 /data/a/fixed_paths/sim/P2_s_curve.json" >&2
  exit 2
fi
if [[ ! -f "${PATH_FILE}" ]]; then
  echo "[ERR] PATH_FILE 不存在: ${PATH_FILE}" >&2
  exit 2
fi
if [[ "${BASELINE}" == "all" ]]; then
  BASELINES="teb dwa"
  if [[ "${INCLUDE_MPC_LOCAL_PLANNER}" == "true" ]]; then
    BASELINES="${BASELINES} mpc_local_planner"
  fi
  if [[ "${INCLUDE_LT_DWA}" == "true" ]]; then
    BASELINES="${BASELINES} lt_dwa"
  fi
else
  BASELINES="${BASELINE}"
fi
PREFLIGHT_BASELINES=""
for baseline in ${BASELINES}; do
  PREFLIGHT_BASELINES="${PREFLIGHT_BASELINES} $(preflight_id_for_baseline "${baseline}")"
done
run_phase0_preflight ${PREFLIGHT_BASELINES}

if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/仿真栈未检测到。请先启动仿真。" >&2
  exit 1
fi

mkdir -p "${OUT_ROOT}"

echo "================ fixed-path baseline suite ================"
echo "BASELINES=${BASELINES}"
echo "EXPERIMENT_GROUP=${EXPERIMENT_GROUP}"
echo "SPEED_TIER=${SPEED_TIER}"
echo "LIMIT_PROFILE=${LIMIT_PROFILE}"
echo "PATH_ID=${PATH_ID}"
echo "PATH_FILE=${PATH_FILE}"
echo "OUT_ROOT=${OUT_ROOT}"
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
  /baseline/status
  /baseline/global_plan
  /baseline/teb/status
  /baseline/teb/global_plan
  /baseline/dwa/status
  /baseline/dwa/global_plan
  /baseline/mpc_local_planner/status
  /baseline/mpc_local_planner/global_plan
  /baseline/lt_dwa/status
  /baseline/lt_dwa/global_plan
  /baseline/lt_dwa/local_plan
  /baseline/lt_dwa/shadow_cmd_vel
  /scout/goal
  /scout/global_path
  /scout/move_base/GlobalPlanner/plan
  /scout/move_base/DWAPlannerROS/global_plan
  /scout/move_base/DWAPlannerROS/local_plan
  /scout/move_base/TebLocalPlannerROS/global_plan
  /scout/move_base/TebLocalPlannerROS/local_plan
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
  for baseline in ${BASELINES}; do
    launch_file="$(launch_for_baseline "${baseline}")"
    status_topic="$(status_topic_for_baseline "${baseline}")"
    baseline_config="$(config_for_baseline "${baseline}")"
    plan_target_frame="$(plan_target_frame_for_baseline "${baseline}")"
    method="${baseline}"
    if [[ "${method}" == "mpc" ]]; then
      method="mpc_local_planner"
    fi
    run_id="$(date +%Y%m%d_%H%M%S)_${method}_${PATH_ID}_run${run_idx}"
    run_dir="${OUT_ROOT}/${run_id}"
    mkdir -p "${run_dir}"
    bag="${run_dir}/${run_id}.bag"
    meta="${run_dir}/${run_id}_meta.yaml"

    echo "---------------- ${run_id} ----------------"
    cat >"${meta}" <<EOF
run_id: ${run_id}
method: ${method}
experiment_group: ${EXPERIMENT_GROUP}
evidence_chain_version: ${EVIDENCE_CHAIN_VERSION}
path_id: ${PATH_ID}
path_file: ${PATH_FILE}
path_topic: ${PATH_TOPIC}
baseline_config: ${baseline_config}
speed_tier: ${SPEED_TIER}
limit_profile: ${LIMIT_PROFILE}
target_v_max_mps: ${TARGET_V_MAX_MPS}
target_omega_max_radps: ${TARGET_OMEGA_MAX_RADPS}
target_acc_lim_x_mps2: ${TARGET_ACC_LIM_X_MPS2}
target_acc_lim_theta_radps2: ${TARGET_ACC_LIM_THETA_RADPS2}
plan_target_frame: ${plan_target_frame}
force_straight_plan_on_goal: false
use_wrapper_goal_check: true
git_hash: ${git_hash}
record_sec: ${RECORD_SEC}
run_index: ${run_idx}
skip_phase0_preflight: ${SKIP_PHASE0_PREFLIGHT}
slosh_monitor_enable: ${SLOSH_MONITOR_ENABLE}
slosh_monitor_odom_topic: ${SLOSH_MONITOR_ODOM_TOPIC}
slosh_monitor_cmd_vel_topic: ${SLOSH_MONITOR_CMD_VEL_TOPIC}
slosh_monitor_output_namespace: ${SLOSH_MONITOR_OUTPUT_NAMESPACE}
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

    path_args=(--mode replay --path-file "${PATH_FILE}" --output-topic "${PATH_TOPIC}" --publish-once-keepalive)
    if [[ "${SKIP_START_WAIT}" == "true" ]]; then
      path_args+=(--skip-start-wait)
    fi
    echo "[path] fixed_global_path_runner.py ${path_args[*]}"
    rosrun scout_local_planner fixed_global_path_runner.py "${path_args[@]}" \
      >"${run_dir}/${run_id}_path.log" 2>&1 &
    path_pid=$!
    sleep 1

    if [[ "${baseline}" == "lt_dwa" ]]; then
      echo "[planner] roslaunch spmpc_experiments ${launch_file} global_path_topic:=${PATH_TOPIC} cmd_vel_topic:=${CMD_VEL_TOPIC} publish_cmd_vel:=${LT_DWA_PUBLISH_CMD_VEL}"
      roslaunch spmpc_experiments "${launch_file}" \
        global_path_topic:="${PATH_TOPIC}" \
        cmd_vel_topic:="${CMD_VEL_TOPIC}" \
        publish_cmd_vel:="${LT_DWA_PUBLISH_CMD_VEL}" \
        >"${run_dir}/${run_id}_planner.log" 2>&1 &
    else
      echo "[planner] roslaunch spmpc_experiments ${launch_file} global_path_topic:=${PATH_TOPIC} cmd_vel_topic:=${CMD_VEL_TOPIC}"
      roslaunch spmpc_experiments "${launch_file}" global_path_topic:="${PATH_TOPIC}" cmd_vel_topic:="${CMD_VEL_TOPIC}" \
        >"${run_dir}/${run_id}_planner.log" 2>&1 &
    fi
    planner_pid=$!

    if ! wait_status_or_cmd "${status_topic}" "${WAIT_READY_SEC}"; then
      echo "[WARN] ${WAIT_READY_SEC}s 内未观察到 ${status_topic} 或 ${CMD_VEL_TOPIC}; 仍继续录包" >&2
    fi

    sleep "${RECORD_SEC}"
    cleanup_run
    sleep 1
    echo "[done] ${run_id}"
  done
done

trap - EXIT
echo "[suite done] ${OUT_ROOT}"
