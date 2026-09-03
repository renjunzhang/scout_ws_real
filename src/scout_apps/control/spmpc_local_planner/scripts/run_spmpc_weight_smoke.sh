#!/usr/bin/env bash
# Short operator-facing entry for one explicit-actuator B_slosh weight smoke.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGINE="${SCRIPT_DIR}/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"

run_motion=false
w_slosh="${TUNE_W_SLOSH:-1.0}"
w_accel="${TUNE_W_ACCEL:-0.3}"
w_du_a="${TUNE_W_DU_A:-0.1}"
w_alpha="${TUNE_W_ALPHA:-0.1}"

usage() {
  cat <<'EOF'
用法：
  run_spmpc_weight_smoke.sh [--run] [代价参数]

默认只做 validate-only，不启动底盘。加入 --run 表示操作者已确认：
底盘位于冻结路径起点、路径清空、急停可用，并正式启动一包录制。

代价参数（默认值）：
  --w-slosh VALUE   液体状态权重（1.0）
  --w-accel VALUE   全时域线加速度幅值权重（0.3）
  --w-du-a VALUE    stage-0 跨周期加速度变化权重（0.1）
  --w-alpha VALUE   全时域角加速度幅值权重（0.1）
EOF
}

need_value() {
  [[ $# -ge 2 && -n "${2}" ]] || {
    echo "[run_spmpc_weight_smoke][ERR] ${1} requires a value" >&2
    exit 2
  }
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
    --w-slosh)
      need_value "$@"
      w_slosh="$2"
      shift 2
      ;;
    --w-slosh=*)
      w_slosh="${1#*=}"
      shift
      ;;
    --w-accel)
      need_value "$@"
      w_accel="$2"
      shift 2
      ;;
    --w-accel=*)
      w_accel="${1#*=}"
      shift
      ;;
    --w-du-a)
      need_value "$@"
      w_du_a="$2"
      shift 2
      ;;
    --w-du-a=*)
      w_du_a="${1#*=}"
      shift
      ;;
    --w-alpha)
      need_value "$@"
      w_alpha="$2"
      shift 2
      ;;
    --w-alpha=*)
      w_alpha="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[run_spmpc_weight_smoke][ERR] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

common_env=(
  SMOKE_PROFILE=weight_tuning
  TUNE_W_SLOSH="${w_slosh}"
  TUNE_W_ACCEL="${w_accel}"
  TUNE_W_DU_A="${w_du_a}"
  TUNE_W_ALPHA="${w_alpha}"
)

if [[ "${run_motion}" == "true" ]]; then
  exec env "${common_env[@]}" \
    VALIDATE_ONLY=false \
    ARM_MOTION=YES \
    CONFIRM_RUNTIME_SMOKE=YES \
    CONFIRM_PATH_CLEAR=YES \
    bash "${ENGINE}"
fi

exec env "${common_env[@]}" VALIDATE_ONLY=true bash "${ENGINE}"
