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
  smooth)
    SLOSH_ENABLE=false
    W_SLOSH=0.0
    EXACT_CONDITION=B0
    EXPECTED_ACTIVE_STATE_WIDTH=24
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
