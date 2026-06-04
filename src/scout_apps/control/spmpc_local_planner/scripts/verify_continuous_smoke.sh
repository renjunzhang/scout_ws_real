#!/usr/bin/env bash
# 连续 MPCC(continuous_mpcc_acados 后端)仿真闭环验证脚本。
#
# 用法:
#   # 终端1 先起仿真(从固定 spawn 起点):
#   source /home/a/scout_ws/devel/setup.bash
#   SIM_ENV=open USE_RVIZ=true SPAWN_X=-4.0 SPAWN_Y=0.0 SPAWN_Z=0.1 SPAWN_YAW=0.0 \
#     rosrun scout_local_planner launch_sim_nav_stack.sh
#   # 等约 30s 定位/TF/仿真稳定。
#
#   # 终端2 跑本脚本(每个 variant 一次, 切 variant 前重启仿真回到同一起点):
#   source /home/a/scout_ws/devel/setup.bash
#   source ~/.bashrc                     # 带 ACADOS LD_LIBRARY_PATH
#   VARIANT=B0      bash .../verify_continuous_smoke.sh
#   # 重开仿真后:
#   VARIANT=B_slosh bash .../verify_continuous_smoke.sh
#
#   # 两个 bag 都录完后对比(analyze 脚本对连续后端同样适用):
#   python3 .../analyze_b0_bslosh_compare.py /data/$USER/spmpc_continuous B0 B_slosh
#
# 可调环境变量(与 compare_b0_bslosh_smoke.sh 口径一致):
#   VARIANT(默认 B0) RECORD_SEC(20) OUT_DIR(/data/$USER/spmpc_continuous)
#   REF_TOPIC GOAL_X/Y/YAW PATH_FILE
#   SOLVER_BACKEND(默认 continuous_mpcc_acados)
#   ACADOS_SOURCE_DIR(默认 $HOME/acados)

set -euo pipefail

VARIANT="${VARIANT:-B0}"
RECORD_SEC="${RECORD_SEC:-20}"
OUT_DIR="${OUT_DIR:-/data/${USER}/spmpc_continuous}"
REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
GOAL_X="${GOAL_X:--1.2}"
GOAL_Y="${GOAL_Y:-2.6}"
GOAL_YAW="${GOAL_YAW:-1.0}"
PATH_FILE="${PATH_FILE:-/tmp/spmpc_continuous_smoke/P2_s_curve_spmpc_continuous.json}"
PATH_TEMPLATE="${PATH_TEMPLATE:-s_curve}"
SOLVER_BACKEND="${SOLVER_BACKEND:-continuous_mpcc_acados}"
W_SLOSH="${W_SLOSH:--1.0}"          # <0: 用 variants.yaml 原值; >=0: 覆盖(sweep 用)
BAG_NAME="${BAG_NAME:-$VARIANT}"     # 输出 bag 名(sweep 时用 ${VARIANT}_w<值>)
ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"

generator_pid=""
planner_pid=""
rec_pid=""
cleanup() {
  for pid in "$rec_pid" "$planner_pid" "$generator_pid"; do
    [[ -n "$pid" ]] && { kill -INT "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; }
  done
}
trap cleanup EXIT

echo_str() { timeout 3s rostopic echo -n 1 "$1" 2>/dev/null | awk -F'"' '/data:/ {print $2; exit}'; }
echo_num() { timeout 3s rostopic echo -n 1 "$1" 2>/dev/null | awk '/data:/ {print $2; exit}'; }
first_cmd_v() { timeout 3s rostopic echo -n 1 /cmd_vel 2>/dev/null | awk '/x:/ {print $2; exit}'; }
is_near_zero() { awk -v x="${1:-999}" 'BEGIN { if (x < 0) x = -x; exit !(x < 0.001) }'; }

RECORD_TOPICS=(
  /spmpc/status /spmpc/solver_backend /spmpc/cost_breakdown /spmpc/slosh_horizon_summary
  /spmpc/corridor /spmpc/guidance /spmpc/primitive
  /spmpc/debug/slosh_state /spmpc/slosh_height /spmpc/debug/progress_s /spmpc/solver_time_ms
  /cmd_vel /odom "${REF_TOPIC}" /map /tf /tf_static
)

# ---- acados 环境 preflight ----
echo "================ continuous 验证 variant=${VARIANT} backend=${SOLVER_BACKEND} ================"
if [[ "${SOLVER_BACKEND}" == "continuous_mpcc_acados" ]]; then
  export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"
  miss=0
  [[ -f "${ACADOS_SOURCE_DIR}/lib/libacados.so" ]] || { echo "[ERR] 找不到 ${ACADOS_SOURCE_DIR}/lib/libacados.so" >&2; miss=1; }
  [[ -f "${PKG_DIR}/generated/acados/spmpc_b0/libacados_ocp_solver_spmpc_b0.so" ]] || { echo "[ERR] 缺生成的 spmpc_b0 求解器, 先跑 generate_spmpc_acados.py --model b0" >&2; miss=1; }
  [[ -f "${PKG_DIR}/generated/acados/spmpc_slosh/libacados_ocp_solver_spmpc_slosh.so" ]] || { echo "[ERR] 缺生成的 spmpc_slosh 求解器, 先跑 generate_spmpc_acados.py --model slosh" >&2; miss=1; }
  [[ "$miss" == "1" ]] && exit 3
  echo "[preflight] acados 环境 OK (LD_LIBRARY_PATH 含 ${ACADOS_SOURCE_DIR}/lib)"
fi

mkdir -p "$OUT_DIR" "$(dirname "$PATH_FILE")"
if ! rostopic list >/dev/null 2>&1; then
  echo "[ERR] 未检测到 roscore/仿真栈, 先启动仿真栈。" >&2
  exit 1
fi

bag="${OUT_DIR}/${BAG_NAME}.bag"
echo "[out] ${bag}  (w_slosh override=${W_SLOSH})"

echo "[path] 启动固定路径生成器: template=${PATH_TEMPLATE} -> ${REF_TOPIC}"
rosrun scout_local_planner template_fixed_path_generator.py \
  --template "${PATH_TEMPLATE}" --goal-topic "${GOAL_TOPIC}" --output-topic "${REF_TOPIC}" \
  --path-file "${PATH_FILE}" --start-heading current \
  --spacing 0.05 --amplitude-ratio 0.18 --min-amplitude 0.25 --max-amplitude 1.20 --publish-count 0 \
  >"${OUT_DIR}/${BAG_NAME}_path_generator.log" 2>&1 &
generator_pid=$!
sleep 2

echo "[goal] 发送固定目标 x=${GOAL_X} y=${GOAL_Y} yaw=${GOAL_YAW}"
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic "${GOAL_TOPIC}" --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}" \
  --repeat-count 1 --repeat-rate 1 >"${OUT_DIR}/${BAG_NAME}_send_goal.log" 2>&1

echo "[path] 等待 ${REF_TOPIC} ..."
if ! timeout 10s rostopic echo -n 1 "${REF_TOPIC}/header" >/dev/null; then
  echo "[ERR] 10s 内没收到 ${REF_TOPIC}, 看 ${OUT_DIR}/${BAG_NAME}_path_generator.log" >&2
  exit 1
fi

echo "[launch] planner_variant=${VARIANT} solver_backend=${SOLVER_BACKEND}"
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:="${VARIANT}" reference_path_topic:="${REF_TOPIC}" \
  solver_backend:="${SOLVER_BACKEND}" w_slosh:="${W_SLOSH}" \
  >"${OUT_DIR}/${BAG_NAME}_planner.log" 2>&1 &
planner_pid=$!
sleep 4   # 等 planner 起 + 首次 RTI 解

# ---- 启动后 preflight: 连续后端专属判定 ----
backend="$(echo_str /spmpc/solver_backend || true)"
status="$(echo_str /spmpc/status || true)"
cmd_v="$(first_cmd_v || true)"
solver_ms="$(echo_num /spmpc/solver_time_ms || true)"
echo "[preflight] backend=${backend:-NA} status=${status:-NA} cmd_v=${cmd_v:-NA} solver_ms=${solver_ms:-NA}"
if [[ "${backend:-}" != "${SOLVER_BACKEND}" ]]; then
  echo "[FATAL] 期望 /spmpc/solver_backend=${SOLVER_BACKEND}, 实际=${backend:-NA}。看 ${OUT_DIR}/${BAG_NAME}_planner.log" >&2
  exit 4
fi

case "${status:-}" in
  ACADOS_NOT_IMPLEMENTED)
    echo "[FATAL] 节点是无 acados 的 stub 构建。请带 ACADOS_SOURCE_DIR 重新 catkin_make:" >&2
    echo "        export ACADOS_SOURCE_DIR=\$HOME/acados; catkin_make --pkg spmpc_local_planner --force-cmake" >&2
    exit 4 ;;
  ACADOS_NOT_CREATED)
    echo "[FATAL] acados 求解器创建失败(生成物/链接问题)。看 ${OUT_DIR}/${BAG_NAME}_planner.log" >&2
    exit 4 ;;
  ACADOS_SOLVE_FAILED*)
    echo "[WARN] 首次求解失败(${status})。可能初始 infeasible, 继续录包观察后续是否恢复。" >&2 ;;
  GOAL_REACHED)
    if is_near_zero "${cmd_v:-0}"; then
      echo "[ERR] 启动即 GOAL_REACHED 且 cmd_v≈0: 仿真未从固定 spawn 起点重启, 或已在目标附近。" >&2
      echo "      关仿真, 重跑 launch_sim_nav_stack.sh, 等 30s 再来。" >&2
      exit 2
    fi ;;
esac

echo "[rec] 录包 ${RECORD_SEC}s ..."
rosbag record -O "$bag" "${RECORD_TOPICS[@]}" >"${OUT_DIR}/${BAG_NAME}_rosbag.log" 2>&1 &
rec_pid=$!
sleep "$RECORD_SEC"

# ---- 闭环快照判定(录包结束前再采一次) ----
status_end="$(echo_str /spmpc/status || true)"
cmd_v_end="$(first_cmd_v || true)"
solver_ms_end="$(echo_num /spmpc/solver_time_ms || true)"

cleanup
trap - EXIT
sleep 1

echo
echo "================ ${VARIANT} 闭环快照 ================"
echo "  末态 status     = ${status_end:-NA}   (期望 ${VARIANT}_ACADOS_OK 或 GOAL_REACHED)"
echo "  cmd_v           = ${cmd_v_end:-NA} m/s"
echo "  solver_time_ms  = ${solver_ms_end:-NA}   (30Hz 期望 <33ms)"
echo "  bag             = ${bag}"
echo
echo ">>> h_peak / cost_breakdown / progress 单调性等用 analyze 脚本读 bag(权威):"
echo "    python3 $(dirname "$0")/analyze_b0_bslosh_compare.py ${OUT_DIR} B0 B_slosh"
echo "    (continuous 后端不发 primitive, analyze 中 primitive 字段显示 0 属正常)"
