#!/usr/bin/env bash
# Short operator entry for the frozen WS1/WA03 online-RGB ABBA protocol.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_RUNNER="${SCRIPT_DIR}/run_spmpc_i0_failclosed_explicit_actuator_ws1_wa03_abba_trial.sh"

row=""
run_motion=false

usage() {
  cat <<'EOF'
用法：
  run_spmpc_ws1_wa03_rgb_abba.sh --row 01 [--run]

固定顺序：01=B0，02=Bslosh，03=Bslosh，04=B0。
固定权重：两组 w_accel=0.3；B0 w_slosh=0，Bslosh w_slosh=1.0。
默认只做 validate-only；加入 --run 表示已确认路径清空、急停可用、
底盘位于冻结路径起点，并已检查相机、容器和标尺位置。
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --row)
      [[ $# -ge 2 ]] || { echo "[run_spmpc_ws1_wa03_rgb_abba][ERR] --row requires a value" >&2; exit 2; }
      row="$2"
      shift 2
      ;;
    --row=*)
      row="${1#*=}"
      shift
      ;;
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
      echo "[run_spmpc_ws1_wa03_rgb_abba][ERR] unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "${row}" =~ ^0[1-4]$ ]] || {
  echo "[run_spmpc_ws1_wa03_rgb_abba][ERR] --row must be 01, 02, 03, or 04" >&2
  exit 2
}

if [[ "${run_motion}" == "true" ]]; then
  exec env \
    PAIR_ROW="${row}" \
    VALIDATE_ONLY=false \
    ARM_MOTION=YES \
    CONFIRM_RGB_GEOMETRY=YES \
    CONFIRM_NEW_SPEED_PROFILE=YES \
    bash "${PROFILE_RUNNER}"
fi

exec env PAIR_ROW="${row}" VALIDATE_ONLY=true bash "${PROFILE_RUNNER}"
