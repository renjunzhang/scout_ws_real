#!/usr/bin/env bash
# Freeze or set RealSense RGB exposure/gain/white balance for slosh visual experiments.
#
# Run after the RealSense node is publishing /camera/color/image_raw.

set -euo pipefail

DYNPARAM_NS="${DYNPARAM_NS:-/camera/rgb_camera}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/color/image_raw}"
MODE="${MODE:-freeze_current}"  # freeze_current / manual
EXPOSURE="${EXPOSURE:-}"
GAIN="${GAIN:-}"
WHITE_BALANCE="${WHITE_BALANCE:-}"
READY_TIMEOUT="${READY_TIMEOUT:-10}"
AUTO_SETTLE_S="${AUTO_SETTLE_S:-3}"
OUT_DIR="${OUT_DIR:-/tmp/realsense_rgb_fixed_params_$(date +%Y%m%d)}"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi

if [[ -f /home/geist/scout_ws/devel/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/geist/scout_ws/devel/setup.bash
elif [[ -f /home/a/scout_ws/devel/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/a/scout_ws/devel/setup.bash
fi

echo "[set_realsense_rgb_manual_params] Waiting for ${IMAGE_TOPIC}..."
timeout "${READY_TIMEOUT}" rostopic echo -n 1 "${IMAGE_TOPIC}" >/dev/null

get_config() {
    rosrun dynamic_reconfigure dynparam get "${DYNPARAM_NS}"
}

get_param_from_config() {
    local config="$1"
    local name="$2"
    awk -F': ' -v key="${name}" '$1 == key {print $2; found=1; exit} END {if (!found) exit 1}' <<< "${config}"
}

if [[ "${MODE}" == "freeze_current" ]]; then
    echo "[set_realsense_rgb_manual_params] MODE=freeze_current; waiting ${AUTO_SETTLE_S}s before reading current params..."
    sleep "${AUTO_SETTLE_S}"
    current_config="$(get_config)"
    EXPOSURE="$(get_param_from_config "${current_config}" exposure)"
    GAIN="$(get_param_from_config "${current_config}" gain)"
    WHITE_BALANCE="$(get_param_from_config "${current_config}" white_balance)"
elif [[ "${MODE}" == "manual" ]]; then
    EXPOSURE="${EXPOSURE:-7000}"
    GAIN="${GAIN:-32}"
    WHITE_BALANCE="${WHITE_BALANCE:-4200}"
else
    echo "[set_realsense_rgb_manual_params] ERROR: unsupported MODE='${MODE}' (use freeze_current or manual)" >&2
    exit 2
fi

mkdir -p "${OUT_DIR}"
stamp="$(date +%Y%m%d_%H%M%S)"
yaml_path="${OUT_DIR}/realsense_rgb_fixed_params_${stamp}.yaml"
apply_path="${OUT_DIR}/apply_realsense_rgb_fixed_params_${stamp}.sh"

echo "[set_realsense_rgb_manual_params] Applying RGB params:"
echo "  dynparam_ns=${DYNPARAM_NS}"
echo "  mode=${MODE}"
echo "  exposure=${EXPOSURE}"
echo "  gain=${GAIN}"
echo "  white_balance=${WHITE_BALANCE}"

rosrun dynamic_reconfigure dynparam set "${DYNPARAM_NS}" enable_auto_exposure false
rosrun dynamic_reconfigure dynparam set "${DYNPARAM_NS}" exposure "${EXPOSURE}"
rosrun dynamic_reconfigure dynparam set "${DYNPARAM_NS}" gain "${GAIN}"
rosrun dynamic_reconfigure dynparam set "${DYNPARAM_NS}" enable_auto_white_balance false
rosrun dynamic_reconfigure dynparam set "${DYNPARAM_NS}" white_balance "${WHITE_BALANCE}"

cat > "${yaml_path}" <<EOF
dynparam_ns: ${DYNPARAM_NS}
image_topic: ${IMAGE_TOPIC}
mode: ${MODE}
timestamp: ${stamp}
camera:
  enable_auto_exposure: false
  exposure: ${EXPOSURE}
  gain: ${GAIN}
  enable_auto_white_balance: false
  white_balance: ${WHITE_BALANCE}
EOF

cat > "${apply_path}" <<EOF
#!/usr/bin/env bash
set -euo pipefail
rosrun dynamic_reconfigure dynparam set ${DYNPARAM_NS} enable_auto_exposure false
rosrun dynamic_reconfigure dynparam set ${DYNPARAM_NS} exposure ${EXPOSURE}
rosrun dynamic_reconfigure dynparam set ${DYNPARAM_NS} gain ${GAIN}
rosrun dynamic_reconfigure dynparam set ${DYNPARAM_NS} enable_auto_white_balance false
rosrun dynamic_reconfigure dynparam set ${DYNPARAM_NS} white_balance ${WHITE_BALANCE}
EOF
chmod +x "${apply_path}"

echo
echo "[set_realsense_rgb_manual_params] Saved fixed params:"
echo "  ${yaml_path}"
echo "  ${apply_path}"

echo
echo "[set_realsense_rgb_manual_params] Current RGB params:"
current_config="$(get_config)"
for key in enable_auto_exposure exposure gain enable_auto_white_balance white_balance; do
    value="$(get_param_from_config "${current_config}" "${key}" || echo unknown)"
    echo "${key}: ${value}"
done

echo
echo "[set_realsense_rgb_manual_params] Check image rate:"
timeout 5 rostopic hz "${IMAGE_TOPIC}" || true
