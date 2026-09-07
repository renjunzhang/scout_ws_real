#!/usr/bin/env bash
# Thin operator entry; shared engine owns acquisition, cleanup and acceptance.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_motion=false
condition=full
jerk_max=1.0
usage() {
  cat <<'EOF'
用法：
  bash run_spmpc_ablation_smoke.sh [--condition full|nostate|smooth|b0|no_jerk]
       [--jerk-max 1.0] [--validate-only | --run]

默认只检查，不启动 ROS 节点或底盘。--run 录一包 70 秒以内的低速开发 smoke，
操作者须先完成急停、起点和净空检查。复用原运行、录包、停车和画图流程。
full：完整方法；nostate：仅 OCP 液体初态置零；smooth：关闭液体优化，保留硬约束；
b0：关闭液体优化和新增硬约束，保留共同软平滑权重；不是旧冻结 B0 参数。
no_jerk：完整方法只关闭新增硬约束。其余参数保持相同，I0 监视器持续运行。
v_ref=0.20 m/s，jerk_max 单位 m/s³，默认 1.0 仍是开发候选。
本入口不录 RGB；可按 I0 state_stamp 比较模型高度，检查通过不代表真实降晃有效。
各组均保留既有平滑验收门；b0/no_jerk 未通过平滑门也保留对照包及报告。
EOF
}
while (( $# )); do
  case "$1" in
    --condition|--jerk-max)
      (( $# >= 2 )) || { usage >&2; exit 2; }
      if [[ "$1" == --condition ]]; then condition="$2"; else jerk_max="$2"; fi
      shift 2 ;;
    --run) run_motion=true; shift ;;
    --validate-only) run_motion=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done
case "${condition}" in
  full|nostate|smooth|b0|no_jerk) ;;
  *) echo "未知组别：${condition}" >&2; exit 2 ;;
esac
if [[ "${run_motion}" == true ]]; then
  exec env SMOKE_PROFILE=ablation ABLATION_CONDITION="${condition}" \
    ABLATION_JERK_MAX="${jerk_max}" VALIDATE_ONLY=false \
    ARM_MOTION=YES CONFIRM_RUNTIME_SMOKE=YES CONFIRM_PATH_CLEAR=YES \
    bash "${SCRIPT_DIR}/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
fi
exec env SMOKE_PROFILE=ablation ABLATION_CONDITION="${condition}" \
  ABLATION_JERK_MAX="${jerk_max}" VALIDATE_ONLY=true \
  bash "${SCRIPT_DIR}/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
