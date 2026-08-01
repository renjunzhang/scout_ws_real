#!/usr/bin/env bash
# Apply and verify the frozen RealSense RGB parameters used by the G3 pilot.

set -euo pipefail

truthy() {
  case "${1:-}" in
    1|true|TRUE|yes|YES|on|ON) return 0 ;;
    *) return 1 ;;
  esac
}

fail() {
  echo "[G3 camera][ERR] $*" >&2
  exit 2
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
[[ -r /opt/ros/noetic/setup.bash ]] || fail "missing ROS Noetic setup"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
[[ -r "${REPO_ROOT}/devel/setup.bash" ]] || fail "missing ${REPO_ROOT}/devel/setup.bash"
# shellcheck disable=SC1090
source "${REPO_ROOT}/devel/setup.bash"

VALIDATE_ONLY="${VALIDATE_ONLY:-false}"
DYNPARAM_NS="/camera/rgb_camera"
IMAGE_TOPIC="/camera/color/image_raw"
CAMERA_INFO_TOPIC="/camera/color/camera_info"
EXPECTED_WIDTH=1920
EXPECTED_HEIGHT=1080
EXPECTED_EXPOSURE=166
EXPECTED_GAIN=64
EXPECTED_WHITE_BALANCE=4600

CAMERA_PARAMS_YAML="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/camera_params/realsense_rgb_fixed_params_20260731_203322.yaml"
CAMERA_PARAMS_YAML_SHA256="0fac946203c4e7592a3fd4e5302d92e4339a074d9cbb4094ecf309b69f84c4d9"
APPLY_SCRIPT="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/camera_params/apply_realsense_rgb_fixed_params_20260731_203322.sh"
APPLY_SCRIPT_SHA256="9a4ff44d141e6ea4ddaf73b2e863bfadc548e0fff95d3ae8514890339fa0741e"
RGB_CALIBRATION="/home/geist/slosh_bags/real/20260731_spmpc_g2s_source_selection/calibration/red_3ruler_g2s_20260731_relabel_frozen_v2.yaml"
RGB_CALIBRATION_SHA256="7186b4bda05a1b73c19fd97b3a34b08a82bfab0df52272eaf2829115de049d01"

for artifact in "${CAMERA_PARAMS_YAML}" "${APPLY_SCRIPT}" "${RGB_CALIBRATION}"; do
  [[ -s "${artifact}" ]] || fail "missing frozen artifact: ${artifact}"
done
[[ "$(sha256sum "${CAMERA_PARAMS_YAML}" | awk '{print $1}')" == "${CAMERA_PARAMS_YAML_SHA256}" ]] || \
  fail "camera parameter YAML hash mismatch"
[[ "$(sha256sum "${APPLY_SCRIPT}" | awk '{print $1}')" == "${APPLY_SCRIPT_SHA256}" ]] || \
  fail "camera apply-script hash mismatch"
[[ "$(sha256sum "${RGB_CALIBRATION}" | awk '{print $1}')" == "${RGB_CALIBRATION_SHA256}" ]] || \
  fail "G3 v2 RGB calibration hash mismatch"

echo "================ G3 RealSense freeze ================"
echo "  stream       = ${EXPECTED_WIDTH}x${EXPECTED_HEIGHT}@30"
echo "  exposure     = ${EXPECTED_EXPOSURE}"
echo "  gain         = ${EXPECTED_GAIN}"
echo "  white balance= ${EXPECTED_WHITE_BALANCE}"
echo "  calibration  = ${RGB_CALIBRATION}"
echo "  calib SHA    = ${RGB_CALIBRATION_SHA256}"
echo "====================================================="

if truthy "${VALIDATE_ONLY}"; then
  echo "[G3 camera] validate-only PASS: frozen artifacts and hashes are present."
  echo "[G3 camera] live apply command:"
  printf '  bash %q\n' "${APPLY_SCRIPT}"
  exit 0
fi

command -v rostopic >/dev/null 2>&1 || fail "rostopic is unavailable"
command -v rosrun >/dev/null 2>&1 || fail "rosrun is unavailable"
timeout 8s rostopic echo -n 1 "${IMAGE_TOPIC}" >/dev/null 2>&1 || \
  fail "no RGB frame on ${IMAGE_TOPIC}; start the base sensor stack first"

echo "[G3 camera] Applying the frozen manual parameters."
bash "${APPLY_SCRIPT}"

camera_info="$(timeout 8s rostopic echo -n 1 "${CAMERA_INFO_TOPIC}" 2>/dev/null)" || \
  fail "no camera_info on ${CAMERA_INFO_TOPIC}"
actual_width="$(awk '$1 == "width:" {print $2; exit}' <<< "${camera_info}")"
actual_height="$(awk '$1 == "height:" {print $2; exit}' <<< "${camera_info}")"
[[ "${actual_width}" == "${EXPECTED_WIDTH}" && "${actual_height}" == "${EXPECTED_HEIGHT}" ]] || \
  fail "RGB shape mismatch: got ${actual_width:-?}x${actual_height:-?}"

camera_config="$(rosrun dynamic_reconfigure dynparam get "${DYNPARAM_NS}" 2>/dev/null)" || \
  fail "cannot read ${DYNPARAM_NS}"
G3_CAMERA_CONFIG_TEXT="${camera_config}" python3 - \
  "${EXPECTED_EXPOSURE}" "${EXPECTED_GAIN}" "${EXPECTED_WHITE_BALANCE}" <<'PY'
import math
import os
import sys

import yaml

expected_exposure, expected_gain, expected_white_balance = map(float, sys.argv[1:])
config = yaml.safe_load(os.environ["G3_CAMERA_CONFIG_TEXT"])
if not isinstance(config, dict):
    raise SystemExit("invalid RealSense dynamic configuration")
if bool(config.get("enable_auto_exposure")):
    raise SystemExit("auto exposure is still enabled")
if bool(config.get("enable_auto_white_balance")):
    raise SystemExit("auto white balance is still enabled")
expected = {
    "exposure": expected_exposure,
    "gain": expected_gain,
    "white_balance": expected_white_balance,
}
for key, value in expected.items():
    actual = float(config.get(key, float("nan")))
    if not math.isclose(actual, value, rel_tol=0.0, abs_tol=0.5):
        raise SystemExit("{} mismatch: runtime={} expected={}".format(key, actual, value))
PY

echo "[G3 camera] PASS: manual exposure/gain/white balance and 1920x1080 stream verified."
echo "[G3 camera] Do not move the camera/container or change RGB parameters during the batch."
