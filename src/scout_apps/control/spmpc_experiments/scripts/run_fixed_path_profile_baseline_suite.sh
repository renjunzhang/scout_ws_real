#!/usr/bin/env bash
# Fixed-path profile-baseline suite for Hamaguchi/Lim offline profile generators.
#
# Scope and safety:
#   - This script assumes ROS/Gazebo are already running. It does not start or stop
#     the simulator and must not be used as strict fresh-sim proof by itself.
#   - It generates a fixed path/profile before launching the common tracker.
#   - It stops only child processes it starts; no broad killall/pkill cleanup.
#   - Slosh monitor topics are recorded for evaluation only and are not consumed by
#     the profile generator or control chain.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

PROFILE_BASELINE="${PROFILE_BASELINE:-hamaguchi_profile}"  # hamaguchi_profile | lim_profile | all
OUT_ROOT="${OUT_ROOT:-/data/${USER}/spmpc_paper_compare/profile_baseline_smoke}"
PATH_SOURCE_MODE="${PATH_SOURCE_MODE:-stable_goal}"  # stable_goal | replay
PATH_FILE="${PATH_FILE:-}"
PATH_ID="${PATH_ID:-P2_s_curve_current_start}"
RUNS="${RUNS:-1}"
RECORD_SEC="${RECORD_SEC:-60}"
RUN_TIMEOUT_SEC="${RUN_TIMEOUT_SEC:-70}"
PATH_TOPIC="${PATH_TOPIC:-/scout/global_path_fixed}"
CMD_VEL_TOPIC="${CMD_VEL_TOPIC:-/cmd_vel}"
COSTMAP_TOPIC="${COSTMAP_TOPIC:-/map}"
WAIT_READY_SEC="${WAIT_READY_SEC:-30}"
PRE_PATH_WAIT_SEC="${PRE_PATH_WAIT_SEC:-0}"
SKIP_START_WAIT="${SKIP_START_WAIT:-true}"

GOAL_TOPIC="${GOAL_TOPIC:-/scout/current_start_fixed_goal}"
GOAL_FRAME="${GOAL_FRAME:-map}"
GOAL_X="${GOAL_X:-5.0}"
GOAL_Y="${GOAL_Y:-0.0}"
GOAL_YAW="${GOAL_YAW:-0.0}"
PATH_TEMPLATE="${PATH_TEMPLATE:-s_curve}"
PATH_START_HEADING="${PATH_START_HEADING:-current}"
PATH_SPACING="${PATH_SPACING:-0.05}"
PATH_AMPLITUDE_RATIO="${PATH_AMPLITUDE_RATIO:-0.18}"
PATH_MIN_AMPLITUDE="${PATH_MIN_AMPLITUDE:-0.25}"
PATH_MAX_AMPLITUDE="${PATH_MAX_AMPLITUDE:-1.20}"
PATH_SIDE="${PATH_SIDE:-left}"
PATH_SMOOTH_ITERATIONS="${PATH_SMOOTH_ITERATIONS:-3}"
PATH_GENERATION_TIMEOUT_SEC="${PATH_GENERATION_TIMEOUT_SEC:-25}"
PATH_GENERATOR_SETTLE_SEC="${PATH_GENERATOR_SETTLE_SEC:-1}"
PATH_GENERATOR_PUBLISH_COUNT="${PATH_GENERATOR_PUBLISH_COUNT:-1}"

TARGET_V_MAX_MPS="${TARGET_V_MAX_MPS:-0.8}"
TARGET_OMEGA_MAX_RADPS="${TARGET_OMEGA_MAX_RADPS:-1.2}"
TARGET_ACC_LIM_X_MPS2="${TARGET_ACC_LIM_X_MPS2:-0.6}"
TARGET_ACC_LIM_THETA_RADPS2="${TARGET_ACC_LIM_THETA_RADPS2:-1.2}"
SPEED_TIER="${SPEED_TIER:-fair_common}"
LIMIT_PROFILE="${LIMIT_PROFILE:-common_v0p8_w1p2_a0p6_alpha1p2}"

PROFILE_OMEGA_N="${PROFILE_OMEGA_N:-31.25}"
PROFILE_DAMPING_RATIO="${PROFILE_DAMPING_RATIO:-0.05}"
PROFILE_DS="${PROFILE_DS:-0.02}"
PROFILE_NUM_SAMPLES="${PROFILE_NUM_SAMPLES:-201}"
PROFILE_START_SPEED="${PROFILE_START_SPEED:-0.0}"
PROFILE_GOAL_SPEED="${PROFILE_GOAL_SPEED:-0.0}"
PROFILE_DECEL_MAX="${PROFILE_DECEL_MAX:-${TARGET_ACC_LIM_X_MPS2}}"
HAMAGUCHI_DT="${HAMAGUCHI_DT:-0.02}"
HAMAGUCHI_LATERAL_ACCEL_MAX="${HAMAGUCHI_LATERAL_ACCEL_MAX:-${TARGET_ACC_LIM_X_MPS2}}"
HAMAGUCHI_CURVATURE_CAP_SCALE="${HAMAGUCHI_CURVATURE_CAP_SCALE:-0.0}"
LIM_HEIGHT_COEFF="${LIM_HEIGHT_COEFF:-1.0}"
LIM_SLOSH_LIMIT_MM="${LIM_SLOSH_LIMIT_MM:-5.0}"
LIM_ITER_MAX="${LIM_ITER_MAX:-6}"
LIM_SLOWDOWN_GAIN="${LIM_SLOWDOWN_GAIN:-0.6}"

EXPERIMENT_GROUP="${EXPERIMENT_GROUP:-LEGACY}"
CONTROLLER_VARIANT="${CONTROLLER_VARIANT:-mpc}"
EXTERNAL_PROFILE_MODE="${EXTERNAL_PROFILE_MODE:-custom_csv}"
Q_SLOSH="${Q_SLOSH:-0.0}"
Q_SLOSH_ETA_DOT="${Q_SLOSH_ETA_DOT:-0.0}"
MPC_Q_V="${MPC_Q_V:-8.0}"
MPC_R_A="${MPC_R_A:-0.4}"
MPC_R_DA="${MPC_R_DA:-0.5}"
MPC_CMD_VEL_LEAD_TIME="${MPC_CMD_VEL_LEAD_TIME:-0.15}"
VEHICLE_V_MAX="${VEHICLE_V_MAX:-${TARGET_V_MAX_MPS}}"
EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE="${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE:-true}"
EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT="${EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT:-${TARGET_ACC_LIM_X_MPS2}}"
EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT="${EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT:-${PROFILE_DECEL_MAX}}"
EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT="${EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT:-0.0}"

SLOSH_MONITOR_ENABLE="${SLOSH_MONITOR_ENABLE:-false}"
SLOSH_MONITOR_ODOM_TOPIC="${SLOSH_MONITOR_ODOM_TOPIC:-/odom}"
SLOSH_MONITOR_CMD_VEL_TOPIC="${SLOSH_MONITOR_CMD_VEL_TOPIC:-${CMD_VEL_TOPIC}}"
SLOSH_MONITOR_OUTPUT_NAMESPACE="${SLOSH_MONITOR_OUTPUT_NAMESPACE:-/slosh}"
SLOSH_RESET_BEFORE_RUN="${SLOSH_RESET_BEFORE_RUN:-true}"
EVIDENCE_CHAIN_VERSION="${EVIDENCE_CHAIN_VERSION:-20260619_profile_baseline_v1}"

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
      echo "[ERR] ${timeout_sec}s 内没有生成文件: ${file}" >&2
      return 1
    fi
    sleep 0.2
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

profile_method_label() {
  case "$1" in
    hamaguchi_profile) echo "HAMAGUCHI_STYLE" ;;
    lim_profile) echo "LIM_STYLE" ;;
    *) echo "[ERR] unknown PROFILE_BASELINE: $1" >&2; return 2 ;;
  esac
}

profile_generator_name() {
  case "$1" in
    hamaguchi_profile) echo "generate_hamaguchi_profile.py" ;;
    lim_profile) echo "generate_lim_style_profile.py" ;;
    *) echo "[ERR] unknown PROFILE_BASELINE: $1" >&2; return 2 ;;
  esac
}

build_profile_args() {
  local baseline="$1"
  local path_file="$2"
  local out_csv="$3"
  local plot_file="$4"
  local debug_prefix="$5"
  local method
  method="$(profile_method_label "${baseline}")"

  PROFILE_ARGS=(
    --path-file "${path_file}"
    --out-csv "${out_csv}"
    --plot "${plot_file}"
    --v-max "${TARGET_V_MAX_MPS}"
    --a-max "${TARGET_ACC_LIM_X_MPS2}"
    --decel-max "${PROFILE_DECEL_MAX}"
    --start-speed "${PROFILE_START_SPEED}"
    --goal-speed "${PROFILE_GOAL_SPEED}"
    --num-samples "${PROFILE_NUM_SAMPLES}"
    --ds "${PROFILE_DS}"
    --omega-n "${PROFILE_OMEGA_N}"
    --damping-ratio "${PROFILE_DAMPING_RATIO}"
    --method-name "${method}"
  )

  if [[ "${baseline}" == "hamaguchi_profile" ]]; then
    PROFILE_ARGS+=(
      --dt "${HAMAGUCHI_DT}"
      --lateral-accel-max "${HAMAGUCHI_LATERAL_ACCEL_MAX}"
      --curvature-cap-scale "${HAMAGUCHI_CURVATURE_CAP_SCALE}"
    )
  else
    PROFILE_ARGS+=(
      --debug-prefix "${debug_prefix}"
      --height-coeff "${LIM_HEIGHT_COEFF}"
      --slosh-limit-mm "${LIM_SLOSH_LIMIT_MM}"
      --iter-max "${LIM_ITER_MAX}"
      --slowdown-gain "${LIM_SLOWDOWN_GAIN}"
    )
  fi
}

generate_stable_goal_path() {
  local run_path_file="$1"
  local run_dir="$2"
  local run_id="$3"
  mkdir -p "$(dirname "${run_path_file}")"
  echo "[stable_path] 当前位姿起点 -> canonical fixed goal，生成 ${run_path_file}"
  rosrun scout_local_planner template_fixed_path_generator.py \
    --template "${PATH_TEMPLATE}" \
    --goal-topic "${GOAL_TOPIC}" \
    --output-topic "${PATH_TOPIC}" \
    --path-file "${run_path_file}" \
    --start-heading "${PATH_START_HEADING}" \
    --spacing "${PATH_SPACING}" \
    --amplitude-ratio "${PATH_AMPLITUDE_RATIO}" \
    --min-amplitude "${PATH_MIN_AMPLITUDE}" \
    --max-amplitude "${PATH_MAX_AMPLITUDE}" \
    --side "${PATH_SIDE}" \
    --smooth-iterations "${PATH_SMOOTH_ITERATIONS}" \
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

check_endpoint() {
  local path_file="$1"
  local run_dir="$2"
  local run_id="$3"
  python3 "${SCRIPT_DIR}/bench_check_profile_endpoint.py" \
    --goal-x "${GOAL_X}" \
    --goal-y "${GOAL_Y}" \
    --goal-yaw "${GOAL_YAW}" \
    --path-template "${PATH_TEMPLATE}" \
    --path-start-heading "${PATH_START_HEADING}" \
    --amplitude-ratio "${PATH_AMPLITUDE_RATIO}" \
    --max-amplitude "${PATH_MAX_AMPLITUDE}" \
    --smooth-iterations "${PATH_SMOOTH_ITERATIONS}" \
    --v-max "${TARGET_V_MAX_MPS}" \
    --omega-max "${TARGET_OMEGA_MAX_RADPS}" \
    --a-max "${TARGET_ACC_LIM_X_MPS2}" \
    --alpha-max "${TARGET_ACC_LIM_THETA_RADPS2}" \
    --path-file "${path_file}" \
    --format yaml \
    >"${run_dir}/${run_id}_endpoint_check.yaml"
}

case "${PATH_SOURCE_MODE}" in
  replay)
    if [[ -z "${PATH_FILE}" ]]; then
      echo "[ERR] replay 模式下 PATH_FILE 不能为空" >&2
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

if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/仿真栈未检测到。本脚本只做 current-sim suite；strict fresh 由 /data wrapper 负责。" >&2
  exit 1
fi

if [[ "${PROFILE_BASELINE}" == "all" ]]; then
  PROFILE_BASELINES="hamaguchi_profile lim_profile"
else
  PROFILE_BASELINES="${PROFILE_BASELINE}"
fi
for baseline in ${PROFILE_BASELINES}; do
  case "${baseline}" in
    hamaguchi_profile|lim_profile) ;;
    *) echo "[ERR] unsupported PROFILE_BASELINE=${baseline}" >&2; exit 2 ;;
  esac
done

mkdir -p "${OUT_ROOT}"

echo "================ fixed-path profile baseline suite ================"
echo "PROFILE_BASELINES=${PROFILE_BASELINES}"
echo "PATH_ID=${PATH_ID}"
echo "PATH_SOURCE_MODE=${PATH_SOURCE_MODE}"
echo "PATH_FILE=${PATH_FILE}"
echo "OUT_ROOT=${OUT_ROOT}"
echo "GOAL=(${GOAL_X}, ${GOAL_Y}, ${GOAL_YAW}) template=${PATH_TEMPLATE} start_heading=${PATH_START_HEADING}"
echo "LIMIT_PROFILE=${LIMIT_PROFILE} v=${TARGET_V_MAX_MPS} omega=${TARGET_OMEGA_MAX_RADPS} a=${TARGET_ACC_LIM_X_MPS2} alpha=${TARGET_ACC_LIM_THETA_RADPS2}"
echo "freshness_claim=current_sim_only"
echo "[preflight] 等待 /odom ${COSTMAP_TOPIC} ..."
wait_topic_once /odom 10
wait_topic_once "${COSTMAP_TOPIC}" 10

git_hash="$(git -C "${REPO_ROOT}" rev-parse --short HEAD 2>/dev/null || echo unknown)"

record_topics=(
  /clock
  "${CMD_VEL_TOPIC}"
  /cmd_vel_drive
  /odom
  /imu/data
  "${COSTMAP_TOPIC}"
  /scan_front
  "${PATH_TOPIC}"
  /mpc_status
  /mpc/status_val
  /mpc/solve_ms
  /mpc/cost_breakdown
  /terminal/mode
  /terminal/goal_info
  /profile_cap/active
  /profile_cap/v_profile
  /profile_cap/cmd_v_pre_cap
  /profile_cap/cmd_v_post_cap
  /profile_cap/implied_ax
  /profile_cap/implied_jerk
  /reference/v_ref
  /reference/v_path
  /reference/kappa
  /reference/s
  /reference/implied_ax
  /reference/implied_ay
  /reference/implied_jerk
  /experiment/config_summary
  /slosh/height
  /slosh/state
  /slosh/debug
  /tf
  /tf_static
)

for run_idx in $(seq 1 "${RUNS}"); do
  for baseline in ${PROFILE_BASELINES}; do
    method="${baseline}"
    method_label="$(profile_method_label "${baseline}")"
    generator_name="$(profile_generator_name "${baseline}")"
    run_id="$(date +%Y%m%d_%H%M%S)_${method}_${PATH_ID}_run${run_idx}"
    run_dir="${OUT_ROOT}/${run_id}"
    mkdir -p "${run_dir}"
    bag="${run_dir}/${run_id}.bag"
    meta="${run_dir}/${run_id}_meta.yaml"
    run_path_file="${PATH_FILE}"
    path_original_file="${PATH_FILE}"
    stable_goal_enabled="false"
    if [[ "${PATH_SOURCE_MODE}" == "stable_goal" ]]; then
      run_path_file="${run_dir}/${run_id}_generated_path.json"
      path_original_file=""
      stable_goal_enabled="true"
    fi
    profile_csv="${run_dir}/${run_id}_${method}_profile.csv"
    profile_plot="${run_dir}/${run_id}_${method}_profile.png"
    profile_debug_prefix="${run_dir}/${run_id}_${method}_debug"

    echo "---------------- ${run_id} ----------------"

    if [[ "${PRE_PATH_WAIT_SEC}" != "0" ]]; then
      echo "[settle] 等待定位/仿真稳定 ${PRE_PATH_WAIT_SEC}s 后再生成/发布 fixed path"
      sleep "${PRE_PATH_WAIT_SEC}"
    fi

    if [[ "${PATH_SOURCE_MODE}" == "stable_goal" ]]; then
      generate_stable_goal_path "${run_path_file}" "${run_dir}" "${run_id}"
    fi

    echo "[endpoint] checking canonical endpoint/template"
    check_endpoint "${run_path_file}" "${run_dir}" "${run_id}"

    build_profile_args "${baseline}" "${run_path_file}" "${profile_csv}" "${profile_plot}" "${profile_debug_prefix}"
    echo "[profile] rosrun scout_local_planner ${generator_name} ${PROFILE_ARGS[*]}"
    rosrun scout_local_planner "${generator_name}" "${PROFILE_ARGS[@]}" \
      >"${run_dir}/${run_id}_profile_generator.log" 2>&1
    wait_file_nonempty "${profile_csv}" 5

    cat >"${meta}" <<EOF_META
run_id: ${run_id}
method: ${method}
method_label: ${method_label}
category: slosh_specific_profile_baseline
experiment_group: ${EXPERIMENT_GROUP}
controller_variant: ${CONTROLLER_VARIANT}
external_profile_mode: ${EXTERNAL_PROFILE_MODE}
evidence_chain_version: ${EVIDENCE_CHAIN_VERSION}
freshness_claim: current_sim_only
strict_requested: false
one_case_per_fresh_sim: false
profile_generated_before_case: true
profile_generator_script: ${generator_name}
profile_csv: ${profile_csv}
profile_omega_n: ${PROFILE_OMEGA_N}
profile_damping_ratio: ${PROFILE_DAMPING_RATIO}
monitor_feedback_used_for_profile: false
monitor_feedback_used_for_control: false
runtime_profile_regeneration_allowed: false
online_liquid_feedback: false
q_slosh: ${Q_SLOSH}
q_slosh_eta_dot: ${Q_SLOSH_ETA_DOT}
profile_execution_cap_enable: ${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE}
profile_execution_accel_limit: ${EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT}
profile_execution_decel_limit: ${EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT}
profile_execution_jerk_limit: ${EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT}
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
stable_goal_enabled: ${stable_goal_enabled}
template_name: ${PATH_TEMPLATE}
template_start_heading: ${PATH_START_HEADING}
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
path_smooth_iterations: ${PATH_SMOOTH_ITERATIONS}
slosh_monitor_enable: ${SLOSH_MONITOR_ENABLE}
slosh_monitor_odom_topic: ${SLOSH_MONITOR_ODOM_TOPIC}
slosh_monitor_cmd_vel_topic: ${SLOSH_MONITOR_CMD_VEL_TOPIC}
slosh_monitor_output_namespace: ${SLOSH_MONITOR_OUTPUT_NAMESPACE}
slosh_height_unit: m
slosh_eval_only: true
slosh_feedback_forbidden: true
external_baseline_uses_slosh: false
EOF_META

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

    echo "[planner] roslaunch scout_local_planner slosh_experiment_sim.launch external_profile_mode:=${EXTERNAL_PROFILE_MODE} external_speed_profile_csv:=${profile_csv} Q_slosh:=${Q_SLOSH} global_path_topic:=${PATH_TOPIC} cmd_vel_topic:=${CMD_VEL_TOPIC}"
    roslaunch scout_local_planner slosh_experiment_sim.launch \
      experiment_group:="${EXPERIMENT_GROUP}" \
      controller_variant:="${CONTROLLER_VARIANT}" \
      external_profile_mode:="${EXTERNAL_PROFILE_MODE}" \
      global_path_topic:="${PATH_TOPIC}" \
      cmd_vel_topic:="${CMD_VEL_TOPIC}" \
      Q_slosh:="${Q_SLOSH}" \
      Q_slosh_eta_dot:="${Q_SLOSH_ETA_DOT}" \
      mpc_Q_v:="${MPC_Q_V}" \
      mpc_R_a:="${MPC_R_A}" \
      mpc_R_da:="${MPC_R_DA}" \
      mpc_cmd_vel_lead_time:="${MPC_CMD_VEL_LEAD_TIME}" \
      vehicle_v_max:="${VEHICLE_V_MAX}" \
      external_speed_profile_csv:="${profile_csv}" \
      external_profile_execution_cap_enable:="${EXTERNAL_PROFILE_EXECUTION_CAP_ENABLE}" \
      external_profile_execution_accel_limit:="${EXTERNAL_PROFILE_EXECUTION_ACCEL_LIMIT}" \
      external_profile_execution_decel_limit:="${EXTERNAL_PROFILE_EXECUTION_DECEL_LIMIT}" \
      external_profile_execution_jerk_limit:="${EXTERNAL_PROFILE_EXECUTION_JERK_LIMIT}" \
      >"${run_dir}/${run_id}_planner.log" 2>&1 &
    planner_pid=$!
    run_start_epoch="$(date +%s)"

    if ! wait_status_or_cmd /mpc_status "${WAIT_READY_SEC}"; then
      echo "[WARN] ${WAIT_READY_SEC}s 内未观察到 /mpc_status 或 ${CMD_VEL_TOPIC}; 仍继续录包" >&2
    fi

    elapsed_sec=$(( $(date +%s) - run_start_epoch ))
    remaining_sec=$(( RUN_TIMEOUT_SEC - elapsed_sec ))
    if [[ "${RUN_TIMEOUT_SEC}" == "0" ]]; then
      sleep "${RECORD_SEC}"
    elif (( remaining_sec <= 0 )); then
      echo "[timeout] planner 启动后已超过 ${RUN_TIMEOUT_SEC}s，停止本 run"
    elif ! timeout "${remaining_sec}s" sleep "${RECORD_SEC}"; then
      echo "[timeout] planner 启动后达到 ${RUN_TIMEOUT_SEC}s，停止本 run"
    fi
    cleanup_run
    sleep 1
    echo "[done] ${run_id}"
  done
done

trap - EXIT
echo "[suite done] ${OUT_ROOT}"
