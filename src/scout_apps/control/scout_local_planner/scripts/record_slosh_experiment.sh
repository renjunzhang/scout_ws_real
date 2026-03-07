#!/bin/bash
# ============================================================================
# 液体晃动抑制实验 — rosbag 录制脚本
# ============================================================================
#
# 使用方式：
#   ./record_slosh_experiment.sh                # 默认 Q_slosh=0
#   ./record_slosh_experiment.sh 10             # Q_slosh=10
#   ./record_slosh_experiment.sh 10 trial_3     # Q_slosh=10, 自定义后缀
#
# 录制内容：
#   - 晃动状态：/slosh/state, /slosh/height, /slosh/height_pred_max
#                /slosh/ax_est, /slosh/ay_est, /slosh/alpha_est
#                /slosh/episode_id, /slosh/constraint_active
#                /slosh/v_des_eff, /slosh/speed_governor_active
#   - MPC 性能：/mpc/solve_ms, /mpc/status_val, /mpc_status
#   - 控制指令：/cmd_vel
#   - 里程计：  /odom
#   - 目标/路径：/scout/goal, /scout/current_goal, /scout/global_path, /local_path
#   - TF：      /tf, /tf_static
#   - 仿真时钟：/clock（仿真时存在，实物可忽略）
#
# 输出路径：
#   1. 若设置环境变量 SLOSH_BAG_DIR，则优先使用它
#   2. 若存在 /data/$USER，则使用 /data/$USER/slosh_bags
#   3. 否则退回 ~/slosh_bags（适合实物）
# ============================================================================

set -euo pipefail

# ---------- 参数 ----------
Q_SLOSH="${1:-0}"
SUFFIX="${2:-}"
DATE_STR=$(date +%Y%m%d_%H%M%S)

# 目录
if [[ -n "${SLOSH_BAG_DIR:-}" ]]; then
    BAG_DIR="${SLOSH_BAG_DIR}"
elif [[ -d "/data/${USER}" ]]; then
    BAG_DIR="/data/${USER}/slosh_bags"
else
    BAG_DIR="${HOME}/slosh_bags"
fi
mkdir -p "${BAG_DIR}"

# 文件名
if [[ -n "${SUFFIX}" ]]; then
    BAG_NAME="slosh_Q${Q_SLOSH}_${DATE_STR}_${SUFFIX}"
else
    BAG_NAME="slosh_Q${Q_SLOSH}_${DATE_STR}"
fi

BAG_PATH="${BAG_DIR}/${BAG_NAME}"

# ---------- 录制话题 ----------
TOPICS=(
    # 晃动观测
    /slosh/state
    /slosh/height
    /slosh/height_pred_max
    /slosh/ax_est
    /slosh/ay_est
    /slosh/alpha_est
    /slosh/episode_id
    /slosh/constraint_active
    /slosh/v_des_eff
    /slosh/speed_governor_active

    # MPC 性能
    /mpc/solve_ms
    /mpc/status_val
    /mpc_status

    # 控制与状态
    /cmd_vel
    /odom

    # 目标与路径
    /scout/goal
    /scout/current_goal
    /scout/global_path
    /scout/global_path_smooth
    /local_path

    # 仿真时钟（实物环境无此话题也不影响录制）
    /clock

    # TF (用于回放可视化)
    /tf
    /tf_static
)

echo "============================================"
echo "  液体晃动抑制实验录制"
echo "============================================"
echo "  Q_slosh  = ${Q_SLOSH}"
echo "  输出文件 = ${BAG_PATH}.bag"
echo "  话题数   = ${#TOPICS[@]}"
echo "============================================"
echo "  Ctrl+C 停止录制"
echo "============================================"

rosbag record -O "${BAG_PATH}" "${TOPICS[@]}"
