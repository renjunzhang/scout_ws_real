# Sourced only by the shared smoke engine's new ablation profile.
# Historical profiles retain their own protocol IDs and defaults.
ABLATION_CONDITION="${ABLATION_CONDITION:-full}"
JERK_MAX="${ABLATION_JERK_MAX:-1.0}"
JERK_MAX="$(python3 - "${JERK_MAX}" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
    if not math.isfinite(value) or value <= 0:
        raise ValueError()
except ValueError:
    raise SystemExit("jerk_max must be finite and positive")
print(format(value, '.12g'))
PY
)" || fail "invalid jerk_max"
SLOSH_ENABLE=true
ZERO_LIQUID_INITIAL_STATE=false
JERK_LIMIT_ENABLE=true
EXACT_CONDITION=Bslosh
EXPECTED_ACTIVE_STATE_WIDTH=28
W_SLOSH=1.0
case "${ABLATION_CONDITION}" in
  full) ;;
  nostate) ZERO_LIQUID_INITIAL_STATE=true ;;
  smooth|b0)
    SLOSH_ENABLE=false
    W_SLOSH=0.0
    EXACT_CONDITION=B0
    EXPECTED_ACTIVE_STATE_WIDTH=24
    # Development B0 keeps the shared soft costs; only smooth adds hard jerk.
    if [[ "${ABLATION_CONDITION}" == b0 ]]; then JERK_LIMIT_ENABLE=false; fi
    ;;
  no_jerk) JERK_LIMIT_ENABLE=false ;;
  *) fail "unknown ablation condition: ${ABLATION_CONDITION}" ;;
esac
PROTOCOL_ID=SMPCC_I0_FAILCLOSED_EXPLICIT_ACTUATOR_ABLATION_SMOKE_DEV_V1
OUTPUT_SERIES=spmpc_i0_failclosed_explicit_actuator_ablation_smoke_v1
RUN_LABEL_PREFIX="DEV_I0FC_EXPACT_ABLATION_${ABLATION_CONDITION}_J${JERK_MAX}"
SMOKE_SCOPE=development_ablation_smoke_only
SMOKE_PURPOSE="startup-switch ablation smoke; one bag, no RGB efficacy claim"
OPERATOR_NOTE="one ${ABLATION_CONDITION} smoke; jerk_max=${JERK_MAX}; RGB disabled"
BLOCK_SEGMENT_ID="I0FC_EXPACT_ABLATION_${ABLATION_CONDITION}"
W_ACCEL=0.3
W_DU_A=0.1
W_ALPHA=0.1
EXPECTED_B0_STATE_WIDTH=24
EXPECTED_SLOSH_STATE_WIDTH=28
MINIMUM_SOLVER_SCHEMA_VERSION=5
CONTROL_CONTINUITY_GATE=true
PLOT_DIAGNOSTICS=true
TERMINAL_MPC_STOP_HANDOFF_ENABLE=true

FULL_HORIZON_DELTA_A=true
PREREG_CONDITION="${ABLATION_CONDITION}"
PASS_CONDITION="${ABLATION_CONDITION}"

# Source comparison uses the same full controller and acquisition engine.
# Explicit flags isolate its labels/evidence from historical image-free bags.
SOURCE_COMPARISON="${ABLATION_SOURCE_COMPARISON:-false}"
SELECTED_OBSERVER_SOURCE="${ABLATION_OBSERVER_SOURCE:-processed_imu}"
SMOKE_RECORD_RGB="${ABLATION_RECORD_RGB:-false}"
EXPERIMENT_KIND="${ABLATION_EXPERIMENT:-legacy}"
COMPARISON_RECORDING=false
TRIAL_ID=
EXPERIMENT_PHASE=
case "${EXPERIMENT_KIND}" in
  legacy) ;;
  ablation-rgb)
    [[ "${SOURCE_COMPARISON}" == false ]] || fail "ablation-rgb is separate from source comparison"
    [[ "${ABLATION_SCENE:-}" == 20260907_c03 ]] || fail "ablation-rgb requires --scene 20260907_c03"
    [[ "${SELECTED_OBSERVER_SOURCE}" == processed_imu ]] || fail "ablation-rgb freezes IMU source"
    [[ "${JERK_MAX}" == 0.6 ]] || fail "ablation-rgb freezes jerk_max=0.6"
    case "${ABLATION_CONDITION}" in smooth|nostate|full) ;; *) fail "ablation-rgb requires smooth, nostate or full" ;; esac
    TRIAL_ID="${ABLATION_TRIAL_ID:-}"
    [[ "${TRIAL_ID}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,39}$ ]] || fail "ablation-rgb requires a safe --trial-id (1-40 characters)"
    EXPERIMENT_PHASE="${ABLATION_PHASE:-screening}"
    case "${EXPERIMENT_PHASE}" in screening|validation) ;; *) fail "invalid experiment phase" ;; esac
    W_SLOSH="$(python3 - "${ABLATION_CONDITION}" "${ABLATION_W_SLOSH:-${W_SLOSH}}" "${ABLATION_V_REF:-0.2}" <<'PY'
import math
import sys
try:
    weight, speed = map(float, sys.argv[2:])
    allowed = (0.0,) if sys.argv[1] == 'smooth' else (0.5, 1.0)
    if not math.isfinite(weight) or weight not in allowed or speed != 0.2:
        raise ValueError()
except ValueError:
    raise SystemExit('ablation-rgb: smooth weight=0; nostate/full weight=0.5 or 1; v_ref=0.2')
print(format(weight, '.12g'))
PY
)" || fail "invalid frozen ablation-rgb weights/speed"
    V_REF=0.20
    SMOKE_RECORD_RGB=true
    COMPARISON_RECORDING=true
    ;;
  *) fail "unknown ablation experiment: ${EXPERIMENT_KIND}" ;;
esac
case "${SOURCE_COMPARISON}:${SMOKE_RECORD_RGB}" in
  true:true|true:false|false:false) ;;
  false:true) [[ "${EXPERIMENT_KIND}" == ablation-rgb ]] || fail "RGB requires an explicit comparison protocol" ;;
  *) fail "invalid source-comparison/RGB flags" ;;
esac
case "${SELECTED_OBSERVER_SOURCE}" in
  processed_imu|odom) ;;
  *) fail "invalid liquid observer source: ${SELECTED_OBSERVER_SOURCE}" ;;
esac
if [[ "${EXPERIMENT_KIND}" == ablation-rgb ]]; then
  PROTOCOL_ID=SMPCC_C03_ABLATION_RGB_DEV_V1
  OUTPUT_SERIES=spmpc_ablation_rgb_v1
  RUN_LABEL_PREFIX="DEV_ABLATION_RGB_${EXPERIMENT_PHASE}_${TRIAL_ID}_${ABLATION_CONDITION}_W${W_SLOSH}_J${JERK_MAX}"
  BLOCK_SEGMENT_ID="ABLATION_RGB_${EXPERIMENT_PHASE}_${TRIAL_ID}"
  SMOKE_SCOPE=development_three_condition_rgb_comparison
  SMOKE_PURPOSE="C03 three-condition comparison; shared IMU, RGB, NOKOV and dual monitors"
  OPERATOR_NOTE="phase=${EXPERIMENT_PHASE}; trial_id=${TRIAL_ID}; condition=${ABLATION_CONDITION}; w_slosh=${W_SLOSH}; jerk_max=${JERK_MAX}; IMU; RGB=true"
elif [[ "${SOURCE_COMPARISON}" == true ]]; then
  COMPARISON_RECORDING=true
  [[ "${ABLATION_CONDITION}" == full ]] || fail "source comparison requires full"
  PROTOCOL_ID=SMPCC_OBSERVER_SOURCE_SMOKE_DEV_V1
  OUTPUT_SERIES=spmpc_observer_source_smoke_v1
  RUN_LABEL_PREFIX="DEV_EXPACT_SOURCE_${SELECTED_OBSERVER_SOURCE}_RGB${SMOKE_RECORD_RGB}_J${JERK_MAX}"
  BLOCK_SEGMENT_ID="SOURCE_${SELECTED_OBSERVER_SOURCE}_RGB${SMOKE_RECORD_RGB}"
  SMOKE_SCOPE=development_liquid_source_comparison
  SMOKE_PURPOSE="full method liquid-initial-state source comparison; common dual monitors and NOKOV"
  OPERATOR_NOTE="liquid_source=${SELECTED_OBSERVER_SOURCE}; jerk_max=${JERK_MAX}; RGB=${SMOKE_RECORD_RGB}; robot state source unchanged"
elif [[ "${SELECTED_OBSERVER_SOURCE}" != processed_imu ]]; then
  fail "odom requires explicit source-comparison mode"
fi
