#!/usr/bin/env bash
# Run one bounded 0 -> 0.8 m/s -> 0 velocity step to expose the effective
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
VALIDATE_ONLY="${VALIDATE_ONLY:-true}"
ARM_MOTION="${ARM_MOTION:-NO}"
CONFIRM_TEST_SETUP="${CONFIRM_TEST_SETUP:-NO}"

[[ -s "${BASE_RUNNER}" ]] || fail "missing base runner: ${BASE_RUNNER}"
[[ "${DATE}" =~ ^[0-9]{8}$ ]] || fail "DATE must be YYYYMMDD"
[[ "${ATTEMPT}" =~ ^[0-9][0-9]$ ]] || fail "ATTEMPT must be two digits"

RUN_LABEL="${RUN_LABEL:-DEV_CHASSIS_HWACC_STEP_LIN_F_V080_a${ATTEMPT}}"
RUN_OUT_DIR="${RUN_OUT_DIR:-/home/geist/slosh_bags/real/${DATE}_mocap_hardware_accel_limit_v1/development}"

echo "================ hardware acceleration limit ================"
echo "  action          = one direct +0.80 m/s velocity step"
echo "  phases          = zero 3.0s -> step 2.5s -> zero 4.0s"
echo "  distance bound  = <=2.0 m while nonzero command; allow >=4.0 m clear space"
echo "  interpretation  = effective chassis+driver limit for this setup, not motor theory"
echo "  required setup  = high-speed switch, representative load, remote E-stop in hand"
echo "  output dir      = ${RUN_OUT_DIR}"
echo "==============================================================="

if ! truthy "${VALIDATE_ONLY}"; then
  [[ "${ARM_MOTION}" == YES ]] || fail "set ARM_MOTION=YES to authorize motion"
  [[ "${CONFIRM_TEST_SETUP}" == YES ]] || \
    fail "set CONFIRM_TEST_SETUP=YES after checking high-speed mode, >=4 m forward clearance, and remote E-stop"
fi

TRIAL_CONTRACT=hardware_accel_limit \
TEST_AXIS=linear \
STEP_DIRECTION=forward \
STEP_MAGNITUDE=0.80 \
PRE_SEC=3.0 \
STEP_SEC=2.5 \
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
