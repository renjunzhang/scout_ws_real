#!/usr/bin/env bash
# w_slosh 单值扫描(连续 MPCC, B_slosh 9 维模型)。
#
# 约束：每个 w_slosh 值跑一次，每次前重开仿真回固定起点(与 compare/verify 同口径)。
# 用同一模型(B_slosh 9维)只变 w_slosh，是纯权重效应；w_slosh=0 即"有 slosh 状态但不罚"基线。
#
# 用法(每个值一次, 中间重开仿真):
#   source /home/a/scout_ws/devel/setup.bash && source ~/.bashrc
#   W_SLOSH=0 bash .../sweep_w_slosh.sh
#   # 重开仿真后:
#   W_SLOSH=1 bash .../sweep_w_slosh.sh
#   W_SLOSH=2 bash .../sweep_w_slosh.sh
#   W_SLOSH=3 bash .../sweep_w_slosh.sh
#   W_SLOSH=5 bash .../sweep_w_slosh.sh
#   # 全部跑完后汇总找拐点:
#   python3 .../sweep_w_slosh_summary.py /data/$USER/spmpc_wsweep
#
# 可调: VARIANT(默认 B_slosh) OUT_DIR(默认 /data/$USER/spmpc_wsweep) RECORD_SEC 等(透传给 verify)。

set -euo pipefail

W_SLOSH="${W_SLOSH:?请指定 W_SLOSH, 例如 W_SLOSH=2.0}"
VARIANT="${VARIANT:-B_slosh}"
OUT_DIR="${OUT_DIR:-/data/${USER}/spmpc_wsweep}"
HERE="$(cd "$(dirname "$0")" && pwd)"

export VARIANT OUT_DIR W_SLOSH
export BAG_NAME="${VARIANT}_w${W_SLOSH}"
export SOLVER_BACKEND="continuous_mpcc_acados"

echo "######## w_slosh sweep: ${VARIANT} @ w_slosh=${W_SLOSH} -> ${OUT_DIR}/${BAG_NAME}.bag ########"
bash "${HERE}/verify_continuous_smoke.sh"

echo
echo ">>> 其余值各跑一次(每次前重开仿真回起点), 例如:"
echo "    for w in 0 1 2 3 5; do  # 每个 w 单独重开仿真后:  W_SLOSH=\$w bash $0  ; done"
echo ">>> 全部跑完后汇总找拐点:"
echo "    python3 ${HERE}/../tools/analysis/sweep_w_slosh_summary.py ${OUT_DIR}"
