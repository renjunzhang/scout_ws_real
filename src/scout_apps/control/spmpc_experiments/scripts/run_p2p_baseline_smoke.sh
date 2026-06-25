#!/usr/bin/env bash
# Point-to-point simulation smoke for SPMPC / TEB / DWA / mpc_local_planner baselines.
#
# 前提:
#   先启动仿真与定位。TEB/DWA/mpc_local_planner 通过 baseline_local_planner_runner
#   独立加载 nav_core plugin，不再依赖 move_base。
#
# 推荐仿真启动：按隔离仿真 SOP 显式指定地图启动 /data/a/scout_sim_replacement：
#   MAP_FILE=/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream \
#   USE_RVIZ=true \
#   /data/a/scout_sim_replacement/scripts/launch_proxy_sim_localization_env.sh
#
# 不要把旧 launch_sim_nav_stack.sh 作为常规 smoke 入口。
#
# 用法:
#   BASELINE=spmpc VARIANT=B_ours SPMPC_SOLVER_BACKEND=continuous_mpcc_acados \
#     OUT_DIR=/data/a/spmpc_baseline_smoke \
#     bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
#
#   BASELINE=teb OUT_DIR=/data/a/spmpc_baseline_smoke \
#     bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh
#
#   BASELINE=dwa OUT_DIR=/data/a/spmpc_baseline_smoke \
#     bash src/scout_apps/control/spmpc_experiments/scripts/run_p2p_baseline_smoke.sh

set -euo pipefail

BASELINE="${BASELINE:-spmpc}"  # spmpc | teb | dwa | mpc
VARIANT="${VARIANT:-B0}"
SPMPC_SOLVER_BACKEND="${SPMPC_SOLVER_BACKEND:-continuous_mpcc_acados}"
SPMPC_W_SLOSH="${SPMPC_W_SLOSH:--1.0}"
OUT_DIR="${OUT_DIR:-/data/${USER}/spmpc_baseline_smoke}"
RECORD_SEC="${RECORD_SEC:-30}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
GOAL_X="${GOAL_X:--1.2}"
GOAL_Y="${GOAL_Y:-2.6}"
GOAL_YAW="${GOAL_YAW:-1.0}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)_${BASELINE}_${VARIANT}}"

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
}
trap cleanup EXIT

wait_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  if ! timeout "${timeout_sec}s" rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
    echo "[ERR] ${timeout_sec}s 内没有收到 ${topic}" >&2
    return 1
  fi
}

wait_status_or_cmd() {
  local timeout_sec="$1"
  local start
  start="$(date +%s)"
  while true; do
    if timeout 1s rostopic echo -n 1 /cmd_vel >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      return 1
    fi
    sleep 0.5
  done
}

mkdir -p "$OUT_DIR"

if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/仿真栈未检测到。请先启动仿真。" >&2
  exit 1
fi

echo "================ P2P baseline smoke: baseline=${BASELINE}, variant=${VARIANT} ================"
echo "[preflight] 等待 /odom /map ..."
wait_topic_once /odom 10
wait_topic_once /map 10

case "$BASELINE" in
  spmpc)
    launch_pkg="spmpc_experiments"
    launch_file="run_spmpc_p2p_sim.launch"
    launch_args=(planner_variant:="${VARIANT}" solver_backend:="${SPMPC_SOLVER_BACKEND}" w_slosh:="${SPMPC_W_SLOSH}")
    ;;
  teb)
    launch_pkg="spmpc_experiments"
    launch_file="run_teb_p2p_sim.launch"
    launch_args=()
    ;;
  dwa)
    launch_pkg="spmpc_experiments"
    launch_file="run_dwa_p2p_sim.launch"
    launch_args=()
    ;;
  mpc|mpc_local_planner)
    launch_pkg="spmpc_experiments"
    launch_file="run_mpc_local_planner_p2p_sim.launch"
    launch_args=()
    ;;
  *)
    echo "[ERR] BASELINE 只能是 spmpc|teb|dwa|mpc, 当前=${BASELINE}" >&2
    exit 2
    ;;
esac

echo "[planner] roslaunch ${launch_pkg} ${launch_file} ${launch_args[*]:-}"
roslaunch "$launch_pkg" "$launch_file" "${launch_args[@]}" \
  >"${OUT_DIR}/${RUN_ID}_planner.log" 2>&1 &
planner_pid=$!
sleep 4

echo "[goal] 发送目标: x=${GOAL_X}, y=${GOAL_Y}, yaw=${GOAL_YAW}"
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic "${GOAL_TOPIC}" \
  --x "${GOAL_X}" \
  --y "${GOAL_Y}" \
  --yaw "${GOAL_YAW}" \
  --repeat-count 1 \
  --repeat-rate 1 \
  >"${OUT_DIR}/${RUN_ID}_send_goal.log" 2>&1

if ! wait_status_or_cmd 12; then
  echo "[WARN] 12s 内没有观察到 /cmd_vel 单帧输出; 仍继续录包, 请检查 ${OUT_DIR}/${RUN_ID}_planner.log" >&2
fi

meta="${OUT_DIR}/${RUN_ID}_meta.yaml"
git_hash="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
cat >"$meta" <<EOF
run_id: ${RUN_ID}
baseline: ${BASELINE}
variant: ${VARIANT}
spmpc_solver_backend: ${SPMPC_SOLVER_BACKEND}
spmpc_w_slosh: ${SPMPC_W_SLOSH}
git_hash: ${git_hash}
record_sec: ${RECORD_SEC}
goal:
  topic: ${GOAL_TOPIC}
  x: ${GOAL_X}
  y: ${GOAL_Y}
  yaw: ${GOAL_YAW}
EOF

record_topics=(
  /cmd_vel
  /odom
  /map
  /scan_front
  /scout/goal
  /scout/global_path
  /scout/global_path_fixed
  /scout/move_base/global_costmap/costmap
  /scout/move_base/local_costmap/costmap
  /scout/move_base/GlobalPlanner/plan
  /scout/move_base/DWAPlannerROS/global_plan
  /scout/move_base/DWAPlannerROS/local_plan
  /scout/move_base/TebLocalPlannerROS/global_plan
  /scout/move_base/TebLocalPlannerROS/local_plan
  /baseline/status
  /baseline/global_plan
  /baseline/teb/status
  /baseline/teb/global_plan
  /baseline/dwa/status
  /baseline/dwa/global_plan
  /baseline/mpc_local_planner/status
  /baseline/mpc_local_planner/global_plan
  /spmpc/status
  /spmpc/controller_variant
  /spmpc/experiment_mode
  /spmpc/local_trajectory
  /spmpc/cost_breakdown
  /spmpc/slosh_horizon_summary
  /spmpc/corridor
  /spmpc/guidance
  /spmpc/primitive
  /spmpc/debug/slosh_state
  /spmpc/debug/progress_s
  /spmpc/solver_time_ms
  /slosh/height
  /slosh/height_pred_max
  /tf
  /tf_static
)

bag="${OUT_DIR}/${RUN_ID}.bag"
echo "[rec] 录包 ${RECORD_SEC}s -> ${bag}"
rosbag record -O "$bag" "${record_topics[@]}" \
  >"${OUT_DIR}/${RUN_ID}_rosbag.log" 2>&1 &
rec_pid=$!

sleep "$RECORD_SEC"

cleanup
trap - EXIT
sleep 1

echo "[done] ${BASELINE}/${VARIANT} -> ${bag}"
echo "[meta] ${meta}"
