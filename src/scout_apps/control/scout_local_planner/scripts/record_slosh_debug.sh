#!/bin/bash
# ============================================================================
# 轻量级调试录包脚本（实物调参与问题定位专用）
# ============================================================================
#
# 目标：
#   - 避免 record_slosh_experiment.sh 录入相机图像、地图和 MBF 大量接口话题
#   - 只保留当前实物 MPC 调参与根因诊断需要的话题
#   - 方便在工控机上快速录制、快速拷贝、快速跑 diagnose_real_tuning.py
#
# 使用方式：
#   ./record_slosh_debug.sh                # 默认 Q_slosh=0
#   ./record_slosh_debug.sh 0 test_q0
#   ./record_slosh_debug.sh 5 q5_debug
#
# 输出目录：
#   1. 若设置 SLOSH_BAG_DIR，则优先使用
#   2. 若存在 /data/$USER，则默认写入 /data/$USER/slosh_bags/real/debug
#   3. 否则退回 ~/slosh_bags/real/debug
# ============================================================================

set -euo pipefail

Q_SLOSH="${1:-0}"
SUFFIX="${2:-}"
DATE_STR=$(date +%Y%m%d_%H%M%S)

if [[ -n "${SLOSH_BAG_DIR:-}" ]]; then
    BAG_DIR="${SLOSH_BAG_DIR}"
elif [[ -d "/data/${USER}" ]]; then
    BAG_DIR="/data/${USER}/slosh_bags/real/debug"
else
    BAG_DIR="${HOME}/slosh_bags/real/debug"
fi
mkdir -p "${BAG_DIR}"

if [[ -n "${SUFFIX}" ]]; then
    BAG_NAME="slosh_Q${Q_SLOSH}_${DATE_STR}_${SUFFIX}"
else
    BAG_NAME="slosh_Q${Q_SLOSH}_${DATE_STR}"
fi

BAG_PATH="${BAG_DIR}/${BAG_NAME}"

TOPICS=(
    # 核心控制/状态
    /cmd_vel
    /odom
    /mpc/solve_ms
    /mpc/status_val
    /mpc_status

    # 参考速度与终点逻辑
    /slosh/v_des_eff
    /terminal/mode
    /terminal/recovery_latched
    /terminal/goal_info

    # 路径/几何诊断
    /scout/goal
    /scout/current_goal
    /scout/global_path
    /scout/global_path_fixed
    /scout/global_path_smooth
    /mpc/reference_path
    /local_path

    # 关键观测与底盘状态
    /slosh/state
    /slosh/eta_norm
    /slosh/eta_dot_norm
    /slosh/modal_energy
    /slosh/modal_energy_norm
    /slosh/excitation_ay_abs
    /slosh/excitation_alpha_abs
    /slosh/height
    /slosh/height_pred_max
    /slosh/q_slosh_eta
    /slosh/speed_cap_active
    /slosh/speed_cap_v_limit
    /imu/data
    /scout_status
    /rs_status

    # 必需 TF
    /tf
    /tf_static
)

echo "============================================"
echo "  轻量级调试录包"
echo "============================================"
echo "  Q_slosh  = ${Q_SLOSH}"
echo "  输出文件 = ${BAG_PATH}.bag"
echo "  话题数   = ${#TOPICS[@]}"
echo "============================================"
echo "  建议用途 = 实物 Q0/Q5 调参与 diagnose_real_tuning.py"
echo "  Ctrl+C 停止录制"
echo "============================================"

rosbag record -O "${BAG_PATH}" "${TOPICS[@]}"
