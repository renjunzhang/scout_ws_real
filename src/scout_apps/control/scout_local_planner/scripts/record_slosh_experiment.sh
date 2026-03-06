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
#   - 晃动状态：/slosh/state, /slosh/height, /slosh/ax_est, /slosh/ay_est, /slosh/alpha_est
#   - MPC 性能：/mpc/solve_ms, /mpc/status_val, /mpc_status
#   - 控制指令：/cmd_vel
#   - 里程计：  /odom
#   - 路径：    /scout/global_path, /local_path
#   - TF：      /tf, /tf_static
#
# 输出路径：~/slosh_bags/slosh_Q{value}_{date}_{suffix}.bag
# ============================================================================

set -euo pipefail

# ---------- 参数 ----------
Q_SLOSH="${1:-0}"
SUFFIX="${2:-}"
DATE_STR=$(date +%Y%m%d_%H%M%S)

# 目录
BAG_DIR="${HOME}/slosh_bags"
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
    /slosh/ax_est
    /slosh/ay_est
    /slosh/alpha_est

    # MPC 性能
    /mpc/solve_ms
    /mpc/status_val
    /mpc_status

    # 控制与状态
    /cmd_vel
    /odom

    # 路径
    /scout/global_path
    /scout/global_path_smooth
    /local_path

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
