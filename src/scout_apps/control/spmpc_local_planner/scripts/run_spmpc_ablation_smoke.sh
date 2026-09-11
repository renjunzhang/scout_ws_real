#!/usr/bin/env bash
# Thin operator entry; shared engine owns acquisition, cleanup and acceptance.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_motion=false
condition=full
jerk_max=1.0
jerk_explicit=false
scene=20260829_c02
observer_source=processed_imu
source_comparison=false
record_rgb=false
experiment=legacy
w_slosh=
v_ref=
trial_id=
phase=screening
usage() {
  cat <<'EOF'
用法：
  bash run_spmpc_ablation_smoke.sh [--condition full|nostate|smooth|b0|no_jerk]
       [--scene 20260829_c02|20260907_c03] [--jerk-max 1.0] [--validate-only | --run]
       [--observer-source imu|odom] [--record-rgb]
       [--experiment ablation-rgb|internal-slosh --trial-id 01_b0 --phase screening|validation]
       [--w-slosh NUMBER --v-ref MPS]

默认只检查，不启动 ROS 节点或底盘。--run 录一包 70 秒以内的低速开发 smoke，
操作者须先完成急停、起点和净空检查。复用原运行、录包、停车和画图流程。
full：完整方法；nostate：仅 OCP 液体初态置零；smooth：关闭液体优化，保留硬约束；
b0：关闭液体优化和新增硬约束，保留共同软平滑权重；不是旧冻结 B0 参数。
no_jerk：完整方法只关闭新增硬约束。其余参数保持相同，I0 监视器持续运行。
v_ref=0.20 m/s，jerk_max 单位 m/s³，默认 1.0 仍是开发候选。
默认不录 RGB。--observer-source 或 --record-rgb 进入完整方法的来源对照批次，
只允许 --condition full；--record-rgb 录原始 RGB、相机信息及 metadata。
来源对照始终保留 NOKOV 和 IMU/odom 两套液体监视器；只改变 OCP 液体初态来源。
--experiment ablation-rgb 是独立的 C03/IMU 三组协议，支持 smooth/nostate/full，
强制录 RGB、Tracker0 与双监视器；每次只录一包，必须给出 --trial-id。
该协议 jerk 固定 0.6、v_ref 固定 0.2；smooth 的 w_slosh 必须为 0，nostate/full 可选 1.0 或 0.5。
未指定权重时 smooth=0、nostate/full=1；默认 phase=screening。
--experiment internal-slosh 是独立的 C03/IMU 内部 slosh 对比，支持 b0/full，
不录 RGB/depth，保留 Tracker0、双监视器与四项验收，不要求相机启动。
该协议默认 jerk=0.6、v_ref=0.2；jerk 可设正数，0<v_ref<=0.2，
b0 权重固定 0 且硬 jerk 关闭；full 默认权重 1，可设 0<w_slosh<=20（首轮只比较 1/0.5）。
必须指定 trial-id；组别、权重、实际 jerk 开关、速度、轮次/阶段写入包名和验收元数据。
IMU 来源、30 Hz/N=60/2 秒、执行器及其他软权重保持现状；本轮首对保持 J0.6/v0.2。
RGB 使用 1920x1080@30Hz，需至少 20 GiB 空间；液面标尺/ROI 仍需核验后离线提取。
检查通过仅表示控制与录制契约通过，不代表真实降晃有效。
各组均保留既有平滑验收门；b0/no_jerk 未通过平滑门也保留对照包及报告。
--scene 默认保留旧地图/C02；新地图与 C03 必须显式选 20260907_c03。
场景只切换冻结地图/路径及输出批次，控制参数不变；地图与路径 SHA 均严格核验。
EOF
}
while (( $# )); do
  case "$1" in
    --condition|--jerk-max|--scene|--observer-source|--experiment|--w-slosh|--v-ref|--trial-id|--phase)
      (( $# >= 2 )) || { usage >&2; exit 2; }
      case "$1" in
        --condition) condition="$2" ;;
        --jerk-max) jerk_max="$2"; jerk_explicit=true ;;
        --scene) scene="$2" ;;
        --observer-source) observer_source="$2"; source_comparison=true ;;
        --experiment) experiment="$2" ;;
        --w-slosh) w_slosh="$2" ;;
        --v-ref) v_ref="$2" ;;
        --trial-id) trial_id="$2" ;;
        --phase) phase="$2" ;;
      esac
      shift 2 ;;
    --run) run_motion=true; shift ;;
    --validate-only) run_motion=false; shift ;;
    --record-rgb) record_rgb=true; source_comparison=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done
case "${condition}" in
  full|nostate|smooth|b0|no_jerk) ;;
  *) echo "未知组别：${condition}" >&2; exit 2 ;;
esac
case "${scene}" in
  20260829_c02|20260907_c03) ;;
  *) echo "未知场景：${scene}" >&2; exit 2 ;;
esac
case "${observer_source}" in
  imu|processed_imu) observer_source=processed_imu ;;
  odom) ;;
  *) echo "未知液体初态来源：${observer_source}" >&2; exit 2 ;;
esac
case "${experiment}" in
  ablation-rgb)
    source_comparison=false; record_rgb=true
    if [[ "${jerk_explicit}" == false ]]; then jerk_max=0.6; fi ;;
  internal-slosh)
    if [[ "${record_rgb}" == true ]]; then
      echo "internal-slosh 不录 RGB，请移除 --record-rgb" >&2; exit 2
    fi
    source_comparison=false; record_rgb=false
    if [[ "${jerk_explicit}" == false ]]; then jerk_max=0.6; fi ;;
  legacy)
    if [[ -n "${w_slosh}${v_ref}${trial_id}" || "${phase}" != screening ]]; then
      echo "权重/速度/轮次/阶段参数需要 --experiment ablation-rgb 或 internal-slosh" >&2; exit 2
    fi ;;
  *) echo "未知实验协议：${experiment}" >&2; exit 2 ;;
esac
if [[ "${source_comparison}" == true && "${condition}" != full ]]; then
  echo "来源对照仅允许 --condition full，避免混入其他消融变量" >&2
  exit 2
fi
if [[ "${run_motion}" == true ]]; then
  exec env SMOKE_PROFILE=ablation ABLATION_CONDITION="${condition}" \
    ABLATION_EXPERIMENT="${experiment}" ABLATION_W_SLOSH="${w_slosh}" \
    ABLATION_V_REF="${v_ref}" ABLATION_TRIAL_ID="${trial_id}" ABLATION_PHASE="${phase}" \
    ABLATION_SOURCE_COMPARISON="${source_comparison}" \
    ABLATION_OBSERVER_SOURCE="${observer_source}" ABLATION_RECORD_RGB="${record_rgb}" \
    ABLATION_SCENE="${scene}" \
    ABLATION_JERK_MAX="${jerk_max}" VALIDATE_ONLY=false \
    ARM_MOTION=YES CONFIRM_RUNTIME_SMOKE=YES CONFIRM_PATH_CLEAR=YES \
    bash "${SCRIPT_DIR}/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
fi
exec env SMOKE_PROFILE=ablation ABLATION_CONDITION="${condition}" \
  ABLATION_EXPERIMENT="${experiment}" ABLATION_W_SLOSH="${w_slosh}" \
  ABLATION_V_REF="${v_ref}" ABLATION_TRIAL_ID="${trial_id}" ABLATION_PHASE="${phase}" \
  ABLATION_SOURCE_COMPARISON="${source_comparison}" \
  ABLATION_OBSERVER_SOURCE="${observer_source}" ABLATION_RECORD_RGB="${record_rgb}" \
  ABLATION_SCENE="${scene}" \
  ABLATION_JERK_MAX="${jerk_max}" VALIDATE_ONLY=true \
  bash "${SCRIPT_DIR}/run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
