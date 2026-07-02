#!/usr/bin/env bash
# SPMPC 连续 MPCC(continuous_mpcc_acados)实物单次运行 + 录包。
#
# 前提(你手动起好, 本脚本不负责; 与 Route A 20260527 同口径):
#   终端A: launch_real_sensors_stack.sh   (底盘/雷达/定位/IMU/RealSense; 提供 /odom /camera/color/*)
#   终端B: set_realsense_rgb_manual_params.sh  (冻结 RGB 曝光/增益/白平衡; 当天所有组同一份)
#   实物机已装 acados 并 source ~/.bashrc(ACADOS_SOURCE_DIR/LD_LIBRARY_PATH)。
#
# 本脚本(终端C, 每组一次, 每组前把车摆回同一地面标记):
#   source /home/geist/scout_ws/devel/setup.bash && source ~/.bashrc
#   DATE=20260610 VARIANT=B0      bash .../run_continuous_real.sh
#   DATE=20260610 VARIANT=B_slosh bash .../run_continuous_real.sh
#   ... B_smooth / B_ours
#   # w_slosh 实物扫: DATE=... VARIANT=B_slosh W_SLOSH=2.0 BAG_NAME=B_slosh_w2 bash ...
#
# 录完用 RGB 离线推断算真值(见 SOP 20260603):
#   rosrun realsense_liquid_measurement red_liquid_infer_from_bag.py \
#     --bag <bag> --calibration <cal.json> --topic /camera/color/image_raw --out-dir <dir>
#
# 真值: RGB max(left,center,right) 液面高度(离线); /spmpc/* 仅模型辅助。
# 在线 /liquid/* 由 realsense_liquid_measurement 单独启动，本脚本默认不依赖、不录制；
# 如需把在线 proxy 同步进 bag，显式设置 RECORD_ONLINE_LIQUID=true。

set -euo pipefail

VARIANT="${VARIANT:-B0}"
DATE="${DATE:-$(date +%Y%m%d)}"
RECORD_SEC="${RECORD_SEC:-25}"
BAG_DIR="${BAG_DIR:-/home/geist/slosh_bags/real/${DATE}_spmpc_continuous}"
BAG_NAME="${BAG_NAME:-$VARIANT}"
REF_TOPIC="${REF_TOPIC:-/scout/global_path_fixed}"
GOAL_TOPIC="${GOAL_TOPIC:-/scout/goal}"
GOAL_X="${GOAL_X:?请显式给 GOAL_X(当天固定 P2 终点, 全组一致)}"
GOAL_Y="${GOAL_Y:?请显式给 GOAL_Y}"
GOAL_YAW="${GOAL_YAW:?请显式给 GOAL_YAW}"
PATH_TEMPLATE="${PATH_TEMPLATE:-s_curve}"
PATH_FILE="${PATH_FILE:-/home/geist/fixed_paths/real/${DATE}/P2_${PATH_TEMPLATE}.json}"
SOLVER_BACKEND="${SOLVER_BACKEND:-continuous_mpcc_acados}"
W_SLOSH="${W_SLOSH:--1.0}"
V_REF="${V_REF:--1.0}"
SLOSH_HEIGHT_MAX="${SLOSH_HEIGHT_MAX:--1.0}"
DELAY_PHASE_MODE="${DELAY_PHASE_MODE:-off}"
DELAY_PHASE_LINEAR_DELAY_SEC="${DELAY_PHASE_LINEAR_DELAY_SEC:--1.0}"
DELAY_PHASE_ANGULAR_DELAY_SEC="${DELAY_PHASE_ANGULAR_DELAY_SEC:--1.0}"
# 控制器本身不依赖 RGB/在线液面节点；下面两个开关只影响本实验脚本的 preflight/录包。
REQUIRE_RGB_TRUTH="${REQUIRE_RGB_TRUTH:-true}"          # true: 要求 /camera/color/image_raw，用于离线 RGB 真值
RECORD_RGB_CAMERA="${RECORD_RGB_CAMERA:-${REQUIRE_RGB_TRUTH}}"  # true: 录 RGB 原始图像/相机内参
RECORD_ONLINE_LIQUID="${RECORD_ONLINE_LIQUID:-false}"   # true: 额外录 /liquid/* 在线调试 proxy
ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}"
PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"

generator_pid=""; planner_pid=""; rec_pid=""
cleanup() {
  for pid in "$rec_pid" "$planner_pid" "$generator_pid"; do
    [[ -n "$pid" ]] && { kill -INT "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; }
  done
}
trap cleanup EXIT

echo_str() { timeout 3s rostopic echo -n 1 "$1" 2>/dev/null | awk -F'"' '/data:/ {print $2; exit}'; }
echo_num() { timeout 3s rostopic echo -n 1 "$1" 2>/dev/null | awk '/data:/ {print $2; exit}'; }
first_cmd_v() { timeout 3s rostopic echo -n 1 /cmd_vel 2>/dev/null | awk '/x:/ {print $2; exit}'; }

# 实物录包: 相机(RGB真值离线用) + 控制/里程 + spmpc 诊断 + 参考路径 + tf。
# 在线 /liquid/* 是 realsense_liquid_measurement 的调试 proxy，不是 spmpc_local_planner 输入；默认不录，需显式 RECORD_ONLINE_LIQUID=true。
RECORD_TOPICS=(
  /cmd_vel /odom /tf /tf_static /imu/data
  "${REF_TOPIC}"
  /spmpc/status /spmpc/controller_variant /spmpc/experiment_mode /spmpc/solver_backend
  /spmpc/debug/effective_config /spmpc/cost_breakdown /spmpc/debug/slosh_cost_monitor
  /spmpc/slosh_horizon_summary /spmpc/debug/slosh_hard_constraint /spmpc/debug/slosh_hard_constraint_effective
  /spmpc/debug/slosh_state /spmpc/slosh_height /spmpc/debug/progress_s /spmpc/debug/v_ref_current
  /spmpc/solver_time_ms /spmpc/local_trajectory
  /spmpc/debug/delay_phase /spmpc/debug/odom_timing /spmpc/debug/execution_state
  /spmpc/debug/execution_alignment_status /spmpc/debug/delay_compensation /spmpc/debug/cmd_odom_alignment
  /spmpc/debug/cmd_vel_output /spmpc/debug/cmd_vel_output_status
)
if [[ "${RECORD_RGB_CAMERA}" == "true" ]]; then
  RECORD_TOPICS+=(/camera/color/image_raw /camera/color/camera_info)
fi
if [[ "${RECORD_ONLINE_LIQUID}" == "true" ]]; then
  RECORD_TOPICS+=(/liquid/height /liquid/height_lcr /liquid/height_median)
fi

echo "================ 实物 continuous: ${VARIANT} (w_slosh=${W_SLOSH}, v_ref=${V_REF}, slosh_height_max=${SLOSH_HEIGHT_MAX}, delay_phase=${DELAY_PHASE_MODE}) -> ${BAG_DIR}/${BAG_NAME}.bag ================"

# ---- acados 环境 preflight ----
if [[ "${SOLVER_BACKEND}" == "continuous_mpcc_acados" ]]; then
  ACADOS_LIB_DIR=""
  if [[ -f "${ACADOS_SOURCE_DIR}/lib/libacados.so" ]]; then
    ACADOS_LIB_DIR="${ACADOS_SOURCE_DIR}/lib"
  elif [[ -f "${ACADOS_SOURCE_DIR}/lib64/libacados.so" ]]; then
    ACADOS_LIB_DIR="${ACADOS_SOURCE_DIR}/lib64"
  fi
  [[ -n "${ACADOS_LIB_DIR}" ]] && export LD_LIBRARY_PATH="${ACADOS_LIB_DIR}:${LD_LIBRARY_PATH:-}"
  miss=0
  [[ -n "${ACADOS_LIB_DIR}" ]] || { echo "[ERR] 缺 ${ACADOS_SOURCE_DIR}/lib或lib64/libacados.so(实物机先按 SOP §1 装 acados)" >&2; miss=1; }
  [[ -f "${PKG_DIR}/generated/acados/spmpc_b0/libacados_ocp_solver_spmpc_b0.so" ]] || { echo "[ERR] 缺 spmpc_b0 求解器, 跑 generate_spmpc_acados.py --model b0" >&2; miss=1; }
  case "${VARIANT}" in
    *slosh*|*ours*)
      [[ -f "${PKG_DIR}/generated/acados/spmpc_slosh/libacados_ocp_solver_spmpc_slosh.so" ]] || { echo "[ERR] ${VARIANT} 需要 spmpc_slosh 求解器, 跑 generate_spmpc_acados.py --model slosh" >&2; miss=1; }
      ;;
  esac
  [[ "$miss" == "1" ]] && exit 3
fi

# ---- 传感器 preflight ----
if ! rostopic list >/dev/null 2>&1; then echo "[ERR] 未检测到 roscore, 先起传感器栈(终端A)。" >&2; exit 1; fi
if ! timeout 5s rostopic echo -n 1 /odom >/dev/null 2>&1; then echo "[ERR] 无 /odom, 检查底盘/定位(终端A)。" >&2; exit 1; fi
if [[ "${REQUIRE_RGB_TRUTH}" == "true" ]]; then
  if ! timeout 5s rostopic echo -n 1 /camera/color/image_raw >/dev/null 2>&1; then
    echo "[ERR] 无 /camera/color/image_raw；控制器可不依赖 RGB，但本实验脚本要求录 RGB 离线真值。若只做控制 smoke，可设 REQUIRE_RGB_TRUTH=false。" >&2
    exit 1
  fi
  echo "[preflight] acados + /odom + RGB truth OK"
else
  echo "[preflight] acados + /odom OK (REQUIRE_RGB_TRUTH=false, 不检查 RGB 真值话题)"
fi

mkdir -p "$BAG_DIR" "$(dirname "$PATH_FILE")"
bag="${BAG_DIR}/${BAG_NAME}.bag"
meta="${BAG_DIR}/${BAG_NAME}_run_meta.env"
{
  echo "variant=${VARIANT}"
  echo "solver_backend=${SOLVER_BACKEND}"
  echo "w_slosh=${W_SLOSH}"
  echo "v_ref=${V_REF}"
  echo "slosh_height_max=${SLOSH_HEIGHT_MAX}"
  echo "delay_phase_mode=${DELAY_PHASE_MODE}"
  echo "delay_phase_linear_delay_sec=${DELAY_PHASE_LINEAR_DELAY_SEC}"
  echo "delay_phase_angular_delay_sec=${DELAY_PHASE_ANGULAR_DELAY_SEC}"
  echo "record_sec=${RECORD_SEC}"
  echo "reference_topic=${REF_TOPIC}"
  echo "path_template=${PATH_TEMPLATE}"
  echo "path_file=${PATH_FILE}"
  printf 'record_topics='
  printf '%s ' "${RECORD_TOPICS[@]}"
  printf '\n'
} >"${meta}"

echo "[path] 生成模板路径(从当前车位姿到同一 goal): template=${PATH_TEMPLATE} -> ${REF_TOPIC}"
rosrun scout_local_planner template_fixed_path_generator.py \
  --template "${PATH_TEMPLATE}" --goal-topic "${GOAL_TOPIC}" --output-topic "${REF_TOPIC}" \
  --path-file "${PATH_FILE}" --start-heading current \
  --spacing 0.05 --amplitude-ratio 0.18 --min-amplitude 0.25 --max-amplitude 1.20 --publish-count 0 \
  >"${BAG_DIR}/${BAG_NAME}_path_generator.log" 2>&1 &
generator_pid=$!
sleep 2

echo "[goal] 发送固定目标 x=${GOAL_X} y=${GOAL_Y} yaw=${GOAL_YAW}"
rosrun scout_local_planner send_fixed_goal.py \
  --goal-topic "${GOAL_TOPIC}" --x "${GOAL_X}" --y "${GOAL_Y}" --yaw "${GOAL_YAW}" \
  --repeat-count 1 --repeat-rate 1 >"${BAG_DIR}/${BAG_NAME}_send_goal.log" 2>&1

echo "[path] 等待 ${REF_TOPIC} ..."
if ! timeout 10s rostopic echo -n 1 "${REF_TOPIC}/header" >/dev/null; then
  echo "[ERR] 10s 内没收到 ${REF_TOPIC}, 看 ${BAG_DIR}/${BAG_NAME}_path_generator.log" >&2
  exit 1
fi
echo "[json] $(ls -lh "${PATH_FILE}" 2>/dev/null | awk '{print $5, $9}')"

echo "[launch] planner_variant=${VARIANT} solver_backend=${SOLVER_BACKEND} w_slosh=${W_SLOSH} v_ref=${V_REF} slosh_height_max=${SLOSH_HEIGHT_MAX} delay_phase=${DELAY_PHASE_MODE}"
roslaunch spmpc_local_planner spmpc_fixed_path.launch \
  planner_variant:="${VARIANT}" reference_path_topic:="${REF_TOPIC}" \
  solver_backend:="${SOLVER_BACKEND}" w_slosh:="${W_SLOSH}" v_ref:="${V_REF}" \
  slosh_height_max:="${SLOSH_HEIGHT_MAX}" delay_phase_mode:="${DELAY_PHASE_MODE}" \
  delay_phase_linear_delay_sec:="${DELAY_PHASE_LINEAR_DELAY_SEC}" \
  delay_phase_angular_delay_sec:="${DELAY_PHASE_ANGULAR_DELAY_SEC}" \
  >"${BAG_DIR}/${BAG_NAME}_planner.log" 2>&1 &
planner_pid=$!
sleep 4

backend="$(echo_str /spmpc/solver_backend || true)"
status="$(echo_str /spmpc/status || true)"
cmd_v="$(first_cmd_v || true)"
solver_ms="$(echo_num /spmpc/solver_time_ms || true)"
echo "[preflight] backend=${backend:-NA} status=${status:-NA} cmd_v=${cmd_v:-NA} solver_ms=${solver_ms:-NA}"
case "${status:-}" in
  ACADOS_NOT_IMPLEMENTED)
    echo "[FATAL] 节点是无 acados 的 stub 构建。带 ACADOS_SOURCE_DIR 重编: catkin_make --pkg spmpc_local_planner --force-cmake" >&2; exit 4 ;;
  ACADOS_NOT_CREATED)
    echo "[FATAL] acados 求解器创建失败, 看 ${BAG_DIR}/${BAG_NAME}_planner.log" >&2; exit 4 ;;
  ACADOS_SOLVE_FAILED*)
    echo "[WARN] 首解失败(${status}); 继续录包观察(可能初始 infeasible)。" >&2 ;;
esac

echo "[rec] 录包 ${RECORD_SEC}s(含相机, bag 会较大)..."
rosbag record -O "$bag" "${RECORD_TOPICS[@]}" >"${BAG_DIR}/${BAG_NAME}_rosbag.log" 2>&1 &
rec_pid=$!
sleep "$RECORD_SEC"

status_end="$(echo_str /spmpc/status || true)"
cleanup
trap - EXIT
sleep 1

echo
echo "================ ${VARIANT} 完成 ================"
echo "  末态 status = ${status_end:-NA}"
echo "  bag         = ${bag}"
echo "  meta        = ${meta}"
echo
echo ">>> 离线算 RGB 真值(max-LCR 液面高度):"
echo "    python3 src/scout_apps/sensors/realsense_liquid_measurement/scripts/red_liquid_infer_from_bag.py \\"
echo "      --bag ${bag} --topic /camera/color/image_raw \\"
echo "      --calibration <当天 calibration.json> --out-dir ${BAG_DIR}/${BAG_NAME}_rgb"
echo ">>> 模型辅助量对比(可选): analyze_b0_bslosh_compare.py(读 /spmpc/* 与 observer)"
