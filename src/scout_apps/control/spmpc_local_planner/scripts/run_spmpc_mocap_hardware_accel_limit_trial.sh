#!/usr/bin/env bash
# Run one explicitly selected bounded velocity pulse to expose the effective
# chassis/driver acceleration and braking capability in high-speed mode.

set -euo pipefail

SCRIPT_NAME="run_spmpc_mocap_hardware_accel_limit_trial"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_RUNNER="${SCRIPT_DIR}/run_spmpc_mocap_velocity_step_trial.sh"

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

DATE="${DATE:-$(date +%Y%m%d)}"
ATTEMPT="${ATTEMPT:-01}"
HARDWARE_TEST_ID="${HARDWARE_TEST_ID:-}"
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_TEST_SETUP="${CONFIRM_TEST_SETUP:-NO}"

[[ -s "${BASE_RUNNER}" ]] || fail "missing base runner: ${BASE_RUNNER}"
[[ "${DATE}" =~ ^[0-9]{8}$ ]] || fail "DATE must be YYYYMMDD"
[[ "${ATTEMPT}" =~ ^[0-9][0-9]$ ]] || fail "ATTEMPT must be two digits"

case "${HARDWARE_TEST_ID}" in
  H01)
    STEP_MAGNITUDE=0.80
    STEP_SEC=2.0
    SPEED_TAG=V080
    CLEARANCE_M=4.0
    ACTION_TEXT="baseline +0.80 m/s velocity step"
    ;;
  H02)
    STEP_MAGNITUDE=1.50
    STEP_SEC=2.0
    SPEED_TAG=V150
    CLEARANCE_M=6.0
    ACTION_TEXT="high-excitation +1.50 m/s velocity step"
    ;;
  H03)
    STEP_MAGNITUDE=3.00
    STEP_SEC=1.0
    SPEED_TAG=V300
    CLEARANCE_M=7.0
    ACTION_TEXT="full-scale +3.00 m/s short pulse; reaching 3.00 m/s is not required"
    ;;
  *) fail "set HARDWARE_TEST_ID=H01, H02, or H03; execute them in that order" ;;
esac

RUN_LABEL="${RUN_LABEL:-DEV_CHASSIS_HWACC_${HARDWARE_TEST_ID}_LIN_F_${SPEED_TAG}_a${ATTEMPT}}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_mocap_hardware_accel_limit_v2/development}"

echo "================ hardware acceleration limit ================"
echo "  row             = ${HARDWARE_TEST_ID}; run H01 -> H02 -> H03 in order"
echo "  action          = ${ACTION_TEXT}"
echo "  phases          = zero 3.0s -> step ${STEP_SEC}s -> zero 4.0s"
echo "  command bound   = speed target x pulse <= $(awk -v v="${STEP_MAGNITUDE}" -v t="${STEP_SEC}" 'BEGIN {printf "%.2f", v*t}') m"
echo "  clear space     = >=${CLEARANCE_M} m straight ahead, including braking margin"
echo "  interpretation  = effective chassis+driver limit for this setup, not motor theory"
echo "  required setup  = high-speed switch, representative load, remote E-stop in hand"
echo "  output dir      = ${RUN_OUT_DIR}"
echo "==============================================================="

if ! truthy "${VALIDATE_ONLY}"; then
  [[ "${ARM_MOTION}" == YES ]] || fail "set ARM_MOTION=YES to authorize motion"
  [[ "${CONFIRM_TEST_SETUP}" == YES ]] || \
    fail "set CONFIRM_TEST_SETUP=YES after checking high-speed mode, >=${CLEARANCE_M} m forward clearance, and remote E-stop"
fi

TRIAL_CONTRACT=hardware_accel_limit \
HARDWARE_TEST_ID="${HARDWARE_TEST_ID}" \
TEST_AXIS=linear \
STEP_DIRECTION=forward \
STEP_MAGNITUDE="${STEP_MAGNITUDE}" \
PRE_SEC=3.0 \
STEP_SEC="${STEP_SEC}" \
POST_SEC=4.0 \
PUBLISH_RATE_HZ=50.0 \
DATA_SPLIT=development \
MATRIX_ROW=single \
ATTEMPT="${ATTEMPT}" \
VALIDATE_ONLY="${VALIDATE_ONLY}" \
ARM_MOTION="${ARM_MOTION}" \
CONFIRM_HARDWARE_ACCEL_LIMIT="${CONFIRM_TEST_SETUP}" \
STAMPED_CMD_TOPIC=/mocap_hardware_accel_limit/cmd_vel_stamped \
RUN_LABEL="${RUN_LABEL}" \
RUN_OUT_DIR="${RUN_OUT_DIR}" \
bash "${BASE_RUNNER}"
