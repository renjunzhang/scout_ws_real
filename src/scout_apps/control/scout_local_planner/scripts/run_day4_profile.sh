#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  cat <<'EOF'
Usage:
  rosrun scout_local_planner run_day4_profile.sh <profile> [extra roslaunch args...]

Profiles:
  baseline           Day4 C baseline
  conservative       Day4 conservative risk recommended init
  no_imu_ay          Day4 no IMU ay control
  relaxed_settling   Day4 relaxed settling check
  isr                ISR / ZV input shaping baseline

Examples:
  rosrun scout_local_planner run_day4_profile.sh baseline
  rosrun scout_local_planner run_day4_profile.sh conservative
  rosrun scout_local_planner run_day4_profile.sh no_imu_ay
  rosrun scout_local_planner run_day4_profile.sh isr
  rosrun scout_local_planner run_day4_profile.sh conservative Q_slosh:=10
EOF
  exit 1
fi

PROFILE="$1"
shift

COMMON_ARGS=(
  "Q_slosh:=5.0"
  "slosh_use_imu_yaw_rate:=true"
  "risk_scheduler_enable:=true"
)

PROFILE_ARGS=()

case "${PROFILE}" in
  baseline)
    PROFILE_ARGS+=(
      "slosh_use_imu_lateral_accel:=true"
    )
    ;;
  conservative)
    PROFILE_ARGS+=(
      "slosh_use_imu_lateral_accel:=true"
      "risk_scheduler_gamma:=3.0"
      "risk_scheduler_rho_0:=0.4"
      "risk_scheduler_rate_limit_per_step:=0.02"
      "risk_scheduler_beta:=0.2"
    )
    ;;
  no_imu_ay)
    PROFILE_ARGS+=(
      "slosh_use_imu_lateral_accel:=false"
      "risk_scheduler_gamma:=3.0"
      "risk_scheduler_rho_0:=0.4"
      "risk_scheduler_rate_limit_per_step:=0.02"
      "risk_scheduler_beta:=0.2"
    )
    ;;
  relaxed_settling)
    PROFILE_ARGS+=(
      "slosh_use_imu_lateral_accel:=true"
      "risk_scheduler_gamma:=3.0"
      "risk_scheduler_rho_0:=0.4"
      "risk_scheduler_rate_limit_per_step:=0.02"
      "risk_scheduler_beta:=0.2"
      "settling_eta_tol:=0.002"
      "settling_eta_dot_tol:=0.05"
      "settling_timeout_s:=3.0"
    )
    ;;
  isr)
    PROFILE_ARGS+=(
      "input_shaping_enable:=true"
      "input_shaping_type:=zv"
      "input_shaping_use_model_params:=true"
      "risk_scheduler_enable:=false"
      "Q_slosh:=0.0"
    )
    ;;
  *)
    echo "Unknown profile: ${PROFILE}" >&2
    exit 2
    ;;
esac

echo "Launching Day4 profile: ${PROFILE}"
echo "Command:"
printf '  %q' roslaunch scout_local_planner slosh_experiment_sim.launch "${COMMON_ARGS[@]}" "${PROFILE_ARGS[@]}" "$@"
printf '\n'

exec roslaunch scout_local_planner slosh_experiment_sim.launch \
  "${COMMON_ARGS[@]}" \
  "${PROFILE_ARGS[@]}" \
  "$@"
