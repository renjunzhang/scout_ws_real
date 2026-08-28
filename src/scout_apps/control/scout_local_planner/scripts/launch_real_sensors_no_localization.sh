#!/usr/bin/env bash
# Mapping prerequisite stack: CAN, Scout base, front LiDAR, IMU, and RealSense.
# Localization and mapping are deliberately not started here. Start the chosen
# mapping launch separately after this script reports that the stack is ready.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Force these values so an inherited shell environment cannot accidentally
# start the frozen-map localization stack while a new map is being built.
export START_LOCALIZATION=false
export WAIT_FOR_LOCALIZATION_MAP=false

exec bash "${SCRIPT_DIR}/launch_real_sensors_stack.sh" "$@"
