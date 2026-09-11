#!/usr/bin/env bash
# Thin geometry-only profile of the existing recorder and motion runner.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MOTION_PROFILE=spin_center ARM_MOTION=NO VALIDATE_ONLY=true
export MOCAP_TRACKER="${MOCAP_TRACKER:-Tracker0}"
export SPIN_OMEGA="${SPIN_OMEGA:-0.20}" SPIN_HOLD_SEC="${SPIN_HOLD_SEC:-32}"
export STATIC_PRE_SEC="${STATIC_PRE_SEC:-5}" STATIC_POST_SEC="${STATIC_POST_SEC:-5}"
export SETTLE_SEC="${SETTLE_SEC:-5}"
export RUN_LABEL=nokov_spin_center
export OUT_DIR="${OUT_DIR:-/home/geist/slosh_bags/real/$(date +%Y%m%d)_mocap_spin_center}"
trial_id=01
usage() {
    cat <<'EOF'
用法：bash run_mocap_spin_center.sh [--check|--run] [参数]
默认 --check：只检查现有 ROS 输入与命令冲突，不录制、不发运动命令。
--run                  录制并执行：静止→逆时针→停稳→顺时针→静止→拟合
--tracker Tracker0     NOKOV 刚体名称
--omega 0.20           每方向角速度绝对值，最多 0.30 rad/s；线速度始终为零
--hold-sec 32          每方向时长，最多 45 s；实际转角由录制后拟合检查
--static-sec 5         前后静止时长，至少 5 s
--settle-sec 5         两方向间停稳时长，5–10 s
--trial-id 01          试次标签；文件名另加时间，避免覆盖
--output-dir PATH      保存目录
EOF
}
while (( $# )); do
    case "$1" in
        --run) export ARM_MOTION=YES VALIDATE_ONLY=false; shift ;;
        --check) export ARM_MOTION=NO VALIDATE_ONLY=true; shift ;;
        --help|-h) usage; exit 0 ;;
        --tracker|--omega|--hold-sec|--static-sec|--settle-sec|--trial-id|--output-dir)
            if (( $# < 2 )); then echo "缺少 $1 的参数" >&2; exit 2; fi
            case "$1" in
                --tracker) export MOCAP_TRACKER="$2" ;;
                --omega) export SPIN_OMEGA="$2" ;;
                --hold-sec) export SPIN_HOLD_SEC="$2" ;;
                --static-sec) export STATIC_PRE_SEC="$2" STATIC_POST_SEC="$2" ;;
                --settle-sec) export SETTLE_SEC="$2" ;;
                --trial-id) trial_id="$2" ;;
                --output-dir) export OUT_DIR="$2" ;;
            esac
            shift 2 ;;
        *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    esac
done
export NAME="SPIN_CENTER_${trial_id}_$(date +%H%M%S)"
ANALYZER="${SCRIPT_DIR}/analyze_mocap_spin_center.py"
if [[ ! -f "${ANALYZER}" ]]; then echo "缺少中心拟合脚本：${ANALYZER}" >&2; exit 1; fi
bash "${SCRIPT_DIR}/run_mocap_imu_calibration_sequence.sh"
if [[ "${VALIDATE_ONLY}" == "false" ]]; then
    # The existing wrapper already finalized the bag and passed stream/motion checks.
    if ! command -v rostopic >/dev/null 2>&1; then source /opt/ros/noetic/setup.bash; fi
    python3 "${ANALYZER}" --bag "${OUT_DIR}/${NAME}.bag" \
        --report "${OUT_DIR}/${NAME}_center_fit.json" \
        --mocap-topic "/vrpn_client_node/${MOCAP_TRACKER}/pose"
    echo "旋转中心拟合：${OUT_DIR}/${NAME}_center_fit.json"
fi
