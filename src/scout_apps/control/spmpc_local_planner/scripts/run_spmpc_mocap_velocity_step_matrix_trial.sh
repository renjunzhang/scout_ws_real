#!/usr/bin/env bash
# Run exactly one explicitly selected row of the frozen chassis step matrix.

set -euo pipefail

SCRIPT_NAME="run_spmpc_mocap_velocity_step_matrix_trial"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SINGLE_RUNNER="${SCRIPT_DIR}/run_spmpc_mocap_velocity_step_trial.sh"
SUMMARY_ANALYZER="${SCRIPT_DIR}/analysis/summarize_mocap_velocity_step_matrix.py"

fail() {
  echo "[${SCRIPT_NAME}][ERR] $*" >&2
  exit 2
}

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

MATRIX_ROW="${MATRIX_ROW:-}"
ATTEMPT="${ATTEMPT:-01}"
DATA_SPLIT="${DATA_SPLIT:-development}"
DATE="${DATE:-$(date +%Y%m%d)}"

[[ -s "${SINGLE_RUNNER}" ]] || fail "missing single-trial runner: ${SINGLE_RUNNER}"
[[ -s "${SUMMARY_ANALYZER}" ]] || fail "missing matrix analyzer: ${SUMMARY_ANALYZER}"
[[ "${MATRIX_ROW}" =~ ^(0[1-9]|1[0-2])$ ]] || \
  fail "set MATRIX_ROW=01..12; one invocation moves the robot for one row only"
[[ "${ATTEMPT}" =~ ^[0-9][0-9]$ ]] || fail "ATTEMPT must be two digits"

case "${DATA_SPLIT}" in
  development) SPLIT_TAG=DEV ;;
  validation) SPLIT_TAG=VAL ;;
  final_test) SPLIT_TAG=FINAL ;;
  *) fail "DATA_SPLIT must be development, validation, or final_test" ;;
esac

# Rows are paired by magnitude so left/right and forward/reverse largely undo
# each other's pose change.  Every row still requires a separate human launch.
case "${MATRIX_ROW}" in
  01) TEST_AXIS=angular; STEP_DIRECTION=left;    STEP_MAGNITUDE=0.10; ROW_TAG=ANG_L_W010 ;;
  02) TEST_AXIS=angular; STEP_DIRECTION=right;   STEP_MAGNITUDE=0.10; ROW_TAG=ANG_R_W010 ;;
  03) TEST_AXIS=angular; STEP_DIRECTION=left;    STEP_MAGNITUDE=0.20; ROW_TAG=ANG_L_W020 ;;
  04) TEST_AXIS=angular; STEP_DIRECTION=right;   STEP_MAGNITUDE=0.20; ROW_TAG=ANG_R_W020 ;;
  05) TEST_AXIS=angular; STEP_DIRECTION=left;    STEP_MAGNITUDE=0.30; ROW_TAG=ANG_L_W030 ;;
  06) TEST_AXIS=angular; STEP_DIRECTION=right;   STEP_MAGNITUDE=0.30; ROW_TAG=ANG_R_W030 ;;
  07) TEST_AXIS=linear;  STEP_DIRECTION=forward; STEP_MAGNITUDE=0.05; ROW_TAG=LIN_F_V005 ;;
  08) TEST_AXIS=linear;  STEP_DIRECTION=reverse; STEP_MAGNITUDE=0.05; ROW_TAG=LIN_R_V005 ;;
  09) TEST_AXIS=linear;  STEP_DIRECTION=forward; STEP_MAGNITUDE=0.10; ROW_TAG=LIN_F_V010 ;;
  10) TEST_AXIS=linear;  STEP_DIRECTION=reverse; STEP_MAGNITUDE=0.10; ROW_TAG=LIN_R_V010 ;;
  11) TEST_AXIS=linear;  STEP_DIRECTION=forward; STEP_MAGNITUDE=0.15; ROW_TAG=LIN_F_V015 ;;
  12) TEST_AXIS=linear;  STEP_DIRECTION=reverse; STEP_MAGNITUDE=0.15; ROW_TAG=LIN_R_V015 ;;
esac

RUN_LABEL="${RUN_LABEL:-${SPLIT_TAG}_CHASSIS_STEP_R${MATRIX_ROW}_${ROW_TAG}_a${ATTEMPT}}"
RUN_OUT_DIR="${RUN_OUT_DIR:-${HOME}/slosh_bags/real/${DATE}_mocap_velocity_step_v2/${DATA_SPLIT}}"

echo "================ frozen chassis-step matrix ================"
echo "  row / attempt = ${MATRIX_ROW}/12 / ${ATTEMPT}"
echo "  data split    = ${DATA_SPLIT}"
echo "  axis          = ${TEST_AXIS}"
echo "  direction     = ${STEP_DIRECTION}"
echo "  magnitude     = ${STEP_MAGNITUDE}"
echo "  run label     = ${RUN_LABEL}"
echo "  output dir    = ${RUN_OUT_DIR}"
echo "============================================================="

export DATE TEST_AXIS STEP_DIRECTION STEP_MAGNITUDE RUN_LABEL RUN_OUT_DIR
export DATA_SPLIT MATRIX_ROW ATTEMPT
bash "${SINGLE_RUNNER}"

if truthy "${VALIDATE_ONLY:-true}"; then
  exit 0
fi

shopt -s nullglob
REPORTS=("${RUN_OUT_DIR}"/*_response.json)
(( ${#REPORTS[@]} > 0 )) || fail "no response reports found after successful trial"
python3 "${SUMMARY_ANALYZER}" "${REPORTS[@]}" \
  --output-json "${RUN_OUT_DIR}/MOCAP_VELOCITY_STEP_MATRIX_SUMMARY_V2.json" \
  --output-csv "${RUN_OUT_DIR}/MOCAP_VELOCITY_STEP_MATRIX_TRIALS_V2.csv" \
  --plot-dir "${RUN_OUT_DIR}/matrix_summary_plots"
