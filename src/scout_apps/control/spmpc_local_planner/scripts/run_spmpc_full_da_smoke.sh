#!/usr/bin/env bash
# Frozen operator entry for one no-RGB, low-speed B_slosh full-horizon Delta-a smoke.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGINE="${SCRIPT_DIR}/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
run_motion=false

usage() {
  cat <<'EOF'
用法：
  run_spmpc_full_da_smoke.sh [--validate-only | --run]

默认只做 validate-only，不启动底盘。--run 仅供操作者在急停、起点和
净空检查完成后录制一包不带 RGB 的低速 B_slosh smoke。配置冻结为：
w_slosh=1.0, w_accel=0.3, w_du_a=0.1, w_alpha=0.1,
v_ref=0.20 m/s, v_safe_max=0.25 m/s, a_max=0.6 m/s^2, N=60, cond_N=10。
录包结束后会先生成六张离线诊断图，再汇总 runner 与 postflight 的
PASS/FAIL；失败包、已有诊断图和报告均保留。
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --run)
      run_motion=true
      shift
      ;;
    --validate-only)
      run_motion=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[run_spmpc_full_da_smoke][ERR] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${run_motion}" == "true" ]]; then
  exec env \
    SMOKE_PROFILE=full_da \
    VALIDATE_ONLY=false \
    ARM_MOTION=YES \
    CONFIRM_RUNTIME_SMOKE=YES \
    CONFIRM_PATH_CLEAR=YES \
    bash "${ENGINE}"
fi

exec env SMOKE_PROFILE=full_da VALIDATE_ONLY=true bash "${ENGINE}"
