#!/usr/bin/env bash
# Phase 2 有效性 smoke (单 variant 模式): 每次重开仿真后只跑一个 variant 并录包。
# 因为无法在同一会话复位车到同一起点, 所以 B0 / B_slosh 各自重开仿真后单独跑,
# 两个 bag 都录完再用 analyze_b0_bslosh_compare.py 对比。
#
# 前提 (你手动启动, 本脚本不负责):
#   1. 仿真栈已起 (odom / tf 在发):
#        source devel/setup.bash
#        SIM_ENV=open USE_RVIZ=true SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
#          rosrun scout_local_planner launch_sim_nav_stack.sh
#      每次切换 variant 前重启仿真场景, 并等待约 30s 让定位/TF/仿真稳定。
#   2. 本脚本会自动启动固定路径生成器、发送固定目标、等待 /scout/global_path_fixed。
#
# 用法 (跑两遍, 中间重开仿真):
#   source devel/setup.bash
#   VARIANT=B0      bash .../compare_b0_bslosh_smoke.sh   # 第一次仿真
#   # 重开仿真环境, 车回到同一 SPAWN 起点
#   VARIANT=B_slosh bash .../compare_b0_bslosh_smoke.sh   # 第二次仿真
#   # 两个 bag 都有了, 再分析:
#   python3 .../analyze_b0_bslosh_compare.py /data/a/spmpc_compare B0 B_slosh
#
# 可调环境变量:
#   VARIANT     本次跑哪个 (默认 B0)
#   RECORD_SEC  录包秒数 (默认 20)
#   OUT_DIR     输出目录 (默认 /data/<user>/spmpc_compare)
#   REF_TOPIC   参考路径话题 (默认 /scout/global_path_fixed)
#   GOAL_X/Y/YAW 固定目标 (默认 -1.2 / 2.6 / 1.0)
#   PATH_FILE   生成路径 JSON (默认 /tmp/spmpc_phase2_smoke/P2_s_curve_spmpc_phase2.json)

set -euo pipefail

VARIANT="${VARIANT:-B0}"
RECORD_SEC="${RECORD_SEC:-20}"
OUT_DIR="${OUT_DIR:-/data/${USER}/spmpc_compare}"
REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
GOAL_X="${GOAL_X:--1.2}"
GOAL_Y="${GOAL_Y:-2.6}"
GOAL_YAW="${GOAL_YAW:-1.0}"
PATH_FILE="${PATH_FILE:-/tmp/spmpc_phase2_smoke/P2_s_curve_spmpc_phase2.json}"
PATH_TEMPLATE="${PATH_TEMPLATE:-s_curve}"

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

is_near_zero() {
  awk -v x="${1:-999}" 'BEGIN { if (x < 0) x = -x; exit !(x < 0.001) }'
}

RECORD_TOPICS=(
  /spmpc/status
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
  /map
)

mkdir -p "$OUT_DIR"
mkdir -p "$(dirname "$PATH_FILE")"

if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] roscore/仿真栈未检测到。请先手动启动仿真栈再运行本脚本。" >&2
  exit 1
fi

bag="${OUT_DIR}/${VARIANT}.bag"
echo "================ 运行 variant=${VARIANT} ================"
echo "[out] ${bag}"
echo "[path] 启动固定路径生成器: template=${PATH_TEMPLATE}, output=${REF_TOPIC}"
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
  >"${OUT_DIR}/${VARIANT}_path_generator.log" 2>&1 &
generator_pid=$!
sleep 2

echo "[goal] 发送固定目标: x=${GOAL_X}, y=${GOAL_Y}, yaw=${GOAL_YAW}"
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic "${GOAL_TOPIC}" \
  --x "${GOAL_X}" \
  --y "${GOAL_Y}" \
  --yaw "${GOAL_YAW}" \
  --repeat-count 1 \
  --repeat-rate 1 \
  >"${OUT_DIR}/${VARIANT}_send_goal.log" 2>&1

echo "[path] 等待 ${REF_TOPIC} ..."
if ! timeout 10s rostopic echo -n 1 "${REF_TOPIC}/header" >/dev/null; then
  echo "[ERR] 10s 内没有收到 ${REF_TOPIC}，查看 ${OUT_DIR}/${VARIANT}_path_generator.log 和 ${OUT_DIR}/${VARIANT}_send_goal.log" >&2
  exit 1
fi

roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:="${VARIANT}" \
  reference_path_topic:="${REF_TOPIC}" \
  >"${OUT_DIR}/${VARIANT}_planner.log" 2>&1 &
planner_pid=$!
sleep 3   # 等 planner 起、拿到第一帧 odom/path

status="$(first_status || true)"
cmd_v="$(first_cmd_v || true)"
echo "[preflight] status=${status:-NA}, cmd_v=${cmd_v:-NA}"
if [[ "${status}" == "GOAL_REACHED" ]] && is_near_zero "${cmd_v:-0}"; then
  echo "[ERR] planner 启动后立即 GOAL_REACHED 且 cmd_v≈0。" >&2
  echo "      这通常表示仿真没有从固定 spawn 起点重启, 或当前机器人已经在目标附近。" >&2
  echo "      请关闭仿真, 重新执行 launch_sim_nav_stack.sh, 等 30s 后再跑本脚本。" >&2
  exit 2
fi

echo "[rec] 录包 ${RECORD_SEC}s ..."
rosbag record -O "$bag" "${RECORD_TOPICS[@]}" \
  >"${OUT_DIR}/${VARIANT}_rosbag.log" 2>&1 &
rec_pid=$!

sleep "$RECORD_SEC"

cleanup
trap - EXIT
sleep 1

echo "[done] ${VARIANT} 完成 -> ${bag}"
echo
echo ">>> 两个 variant 都录完后, 运行对比分析:"
echo "    python3 $(dirname "$0")/analyze_b0_bslosh_compare.py ${OUT_DIR} B0 B_slosh"
