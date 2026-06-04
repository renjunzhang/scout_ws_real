#!/usr/bin/env bash
# Phase 3 smoke runner.
#
# 前提:
#   你已经手动启动仿真栈, 并等待约 30s:
#     source devel/setup.bash
#     SIM_ENV=open USE_RVIZ=true SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
#       rosrun scout_local_planner launch_sim_nav_stack.sh
#
# 用法:
#   固定路径回归 (obstacle/homotopy 默认关闭):
#     PHASE3_MODE=fixed_path VARIANT=B0 OUT_DIR=/data/a/spmpc_phase3_fixed_smoke \
#       bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh
#
#   点到点 Phase 3 链路 (obstacle/homotopy 默认开启):
#     PHASE3_MODE=point_to_point VARIANT=B0 OUT_DIR=/data/a/spmpc_phase3_p2p_smoke \
#       bash src/scout_apps/control/spmpc_local_planner/scripts/phase3_smoke.sh

set -euo pipefail

PHASE3_MODE="${PHASE3_MODE:-fixed_path}"  # fixed_path | point_to_point
VARIANT="${VARIANT:-B0}"
RECORD_SEC="${RECORD_SEC:-20}"
OUT_DIR="${OUT_DIR:-/data/${USER}/spmpc_phase3_smoke}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
GOAL_X="${GOAL_X:--1.2}"
GOAL_Y="${GOAL_Y:-2.6}"
GOAL_YAW="${GOAL_YAW:-1.0}"
PATH_TEMPLATE="${PATH_TEMPLATE:-s_curve}"
PATH_FILE="${PATH_FILE:-/tmp/spmpc_phase3_smoke/P2_s_curve_spmpc_phase3.json}"
SOLVER_BACKEND="${SOLVER_BACKEND:-primitive}"
W_SLOSH="${W_SLOSH:--1.0}"

generator_pid=""
planner_pid=""
rec_pid=""

cleanup() {
  if [[ -n "${rec_pid}" ]]; then
    kill -INT "$rec_pid" 2>/dev/null || true
    wait "$rec_pid" 2>/dev/null || true
  fi
  if [[ -n "${planner_pid}" ]]; then
    kill -INT "$planner_pid" 2>/dev/null || true
    wait "$planner_pid" 2>/dev/null || true
  fi
  if [[ -n "${generator_pid}" ]]; then
    kill -INT "$generator_pid" 2>/dev/null || true
    wait "$generator_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

first_status() {
  timeout 3s rostopic echo -n 1 /spmpc/status 2>/dev/null | awk -F'"' '/data:/ {print $2; exit}'
}

first_cmd_v() {
  timeout 3s rostopic echo -n 1 /cmd_vel 2>/dev/null | awk '/x:/ {print $2; exit}'
}

wait_planner_ready() {
  local timeout_sec="$1"
  local start
  start="$(date +%s)"
  while true; do
    local status
    status="$(first_status || true)"
    case "${status:-}" in
      ""|INITIALIZED|WAITING_FOR_ODOM|WAITING_FOR_REFERENCE_PATH|WAITING_FOR_TF_POSE)
        ;;
      *_OK|GOAL_REACHED|CORRIDOR_REJECT)
        echo "$status"
        return 0
        ;;
      ACADOS_SOLVE_FAILED*|ACADOS_NOT_CREATED|ACADOS_NOT_IMPLEMENTED|NO_REFERENCE_PATH|PROJECTION_FAILED|NO_SOLVER|NO_CANDIDATE)
        echo "$status"
        return 2
        ;;
      *)
        echo "$status"
        return 2
        ;;
    esac
    if (( $(date +%s) - start >= timeout_sec )); then
      echo "${status:-NO_STATUS}"
      return 1
    fi
    sleep 0.5
  done
}

is_near_zero() {
  awk -v x="${1:-999}" 'BEGIN { if (x < 0) x = -x; exit !(x < 0.001) }'
}

wait_topic_header() {
  local topic="$1"
  local timeout_sec="$2"
  if ! timeout "${timeout_sec}s" rostopic echo -n 1 "${topic}/header" >/dev/null 2>&1; then
    echo "[ERR] ${timeout_sec}s 内没有收到 ${topic}" >&2
    return 1
  fi
}

wait_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  if ! timeout "${timeout_sec}s" rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "[ERR] ${timeout_sec}s 内没有收到 ${topic}" >&2
    return 1
  fi
}

RECORD_TOPICS=(
  /spmpc/status
  /spmpc/controller_variant
  /spmpc/experiment_mode
  /spmpc/solver_backend
  /spmpc/local_trajectory
  /spmpc/cost_breakdown
  /spmpc/slosh_horizon_summary
  /spmpc/corridor
  /spmpc/guidance
  /spmpc/primitive
  /spmpc/debug/slosh_state
  /spmpc/debug/progress_s
  /spmpc/solver_time_ms
  /cmd_vel
  /odom
  /scout/global_path
  /scout/global_path_fixed
  /map
  /tf
)

mkdir -p "$OUT_DIR"
mkdir -p "$(dirname "$PATH_FILE")"

if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/仿真栈未检测到。请先启动仿真栈。" >&2
  exit 1
fi

case "$PHASE3_MODE" in
  fixed_path)
    REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
    LAUNCH_FILE="spmpc_fixed_path.launch"
    BAG_NAME="${NAME:-${VARIANT}}"
    echo "================ Phase3 fixed_path variant=${VARIANT} ================"
    echo "[path] 启动固定路径生成器: output=${REF_TOPIC}"
    rosrun scout_local_planner template_fixed_path_generator.py \
      --template "${PATH_TEMPLATE}" \
      --goal-topic "${GOAL_TOPIC}" \
      --output-topic "${REF_TOPIC}" \
      --path-file "${PATH_FILE}" \
      --start-heading current \
      --spacing 0.05 \
      --amplitude-ratio 0.18 \
      --min-amplitude 0.25 \
      --max-amplitude 1.20 \
      --publish-count 0 \
      >"${OUT_DIR}/${BAG_NAME}_path_generator.log" 2>&1 &
    generator_pid=$!
    sleep 2
    ;;

  point_to_point|p2p)
    PHASE3_MODE="point_to_point"
    REF_TOPIC="${REF_TOPIC:-/scout/global_path}"
    LAUNCH_FILE="spmpc_point_to_point.launch"
    BAG_NAME="${NAME:-${VARIANT}_point_to_point}"
    echo "================ Phase3 point_to_point variant=${VARIANT} ================"
    echo "[map] 等待 /map ..."
    wait_topic_once /map 10
    ;;

  *)
    echo "[ERR] PHASE3_MODE 只能是 fixed_path 或 point_to_point, 当前=${PHASE3_MODE}" >&2
    exit 1
    ;;
esac

echo "[goal] 发送目标: x=${GOAL_X}, y=${GOAL_Y}, yaw=${GOAL_YAW}"
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic "${GOAL_TOPIC}" \
  --x "${GOAL_X}" \
  --y "${GOAL_Y}" \
  --yaw "${GOAL_YAW}" \
  --repeat-count 1 \
  --repeat-rate 1 \
  >"${OUT_DIR}/${BAG_NAME}_send_goal.log" 2>&1

echo "[path] 等待 ${REF_TOPIC} ..."
wait_topic_header "${REF_TOPIC}" 15

echo "[planner] 启动 ${LAUNCH_FILE}: solver_backend=${SOLVER_BACKEND}, w_slosh=${W_SLOSH}"
roslaunch spmpc_local_planner "${LAUNCH_FILE}" \
  planner_variant:="${VARIANT}" \
  reference_path_topic:="${REF_TOPIC}" \
  solver_backend:="${SOLVER_BACKEND}" \
  w_slosh:="${W_SLOSH}" \
  >"${OUT_DIR}/${BAG_NAME}_planner.log" 2>&1 &
planner_pid=$!
sleep 3

if ! status="$(wait_planner_ready 12)"; then
  echo "[ERR] planner 12s 内没有进入有效求解状态, 最后 status=${status:-NA}" >&2
  echo "      常见原因: map<-base_link TF 不可用, 或 reference path / odom 没准备好。" >&2
  echo "      查看 ${OUT_DIR}/${BAG_NAME}_planner.log" >&2
  exit 3
fi
cmd_v="$(first_cmd_v || true)"
echo "[preflight] status=${status:-NA}, cmd_v=${cmd_v:-NA}"
if [[ "${status}" == "GOAL_REACHED" ]] && is_near_zero "${cmd_v:-0}"; then
  echo "[ERR] planner 启动后立即 GOAL_REACHED 且 cmd_v≈0。" >&2
  echo "      通常表示仿真没有从固定 spawn 起点重启, 或当前机器人已经在目标附近。" >&2
  exit 2
fi

bag="${OUT_DIR}/${BAG_NAME}.bag"
echo "[rec] 录包 ${RECORD_SEC}s -> ${bag}"
rosbag record -O "$bag" "${RECORD_TOPICS[@]}" \
  >"${OUT_DIR}/${BAG_NAME}_rosbag.log" 2>&1 &
rec_pid=$!

sleep "$RECORD_SEC"

cleanup
trap - EXIT
sleep 1

echo "[done] ${PHASE3_MODE}/${VARIANT} -> ${bag}"
echo
echo "快速检查:"
echo "  rostopic echo -n 1 /spmpc/corridor"
echo "  rostopic echo -n 1 /spmpc/guidance"
echo "  rostopic echo -n 1 /spmpc/cost_breakdown"
