#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_spmpc_delay_augmented_phase.h"
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

static_assert(
    manifest::kDiscreteDynamics == DELAY_AUGMENTED_DISCRETE_DYNAMICS,
    "generated discrete-dynamics capability bit drifted");
static_assert(
    manifest::kAugmentedInitialState == DELAY_AUGMENTED_INITIAL_STATE,
    "generated initial-state capability bit drifted");
static_assert(
    manifest::kPublishedCommandBounds ==
        DELAY_AUGMENTED_PUBLISHED_COMMAND_BOUNDS,
    "generated published-command capability bit drifted");
static_assert(
    manifest::kRobotSpeedBounds == DELAY_AUGMENTED_ROBOT_SPEED_BOUNDS,
    "generated speed capability bit drifted");
static_assert(
    manifest::kPublishedRateBounds ==
        DELAY_AUGMENTED_PUBLISHED_RATE_BOUNDS,
    "generated rate capability bit drifted");
static_assert(
    manifest::kTerminalEmpiricalGate ==
        DELAY_AUGMENTED_TERMINAL_EMPIRICAL_GATE,
    "generated terminal-gate capability bit drifted");
static_assert(
    manifest::kExecutionCompatibilitySet ==
        DELAY_AUGMENTED_EXECUTION_COMPATIBILITY_SET,
    "generated execution-set capability bit drifted");
static_assert(
    manifest::kCapabilities == kDelayAugmentedPhaseWp3cCapabilities,
    "generated WP3C capability mask drifted");
static_assert(
    manifest::kFormalRequiredCapabilities ==
        kDelayAugmentedPhaseFormalCapabilities,
    "generated formal capability mask drifted");

bool sameChannel(const ExecutionChannelContract& channel,
                 bool linear) {
    return channel.delay_sec ==
               (linear ? manifest::kLinearDelaySec
                       : manifest::kAngularDelaySec) &&
        channel.time_constant_sec ==
               (linear ? manifest::kLinearTimeConstantSec
                       : manifest::kAngularTimeConstantSec) &&
        channel.positive_gain ==
               (linear ? manifest::kLinearPositiveGain
                       : manifest::kAngularPositiveGain) &&
        channel.negative_gain ==
               (linear ? manifest::kLinearNegativeGain
                       : manifest::kAngularNegativeGain) &&
        channel.deadzone ==
               (linear ? manifest::kLinearDeadzone
                       : manifest::kAngularDeadzone) &&
        channel.output_min ==
               (linear ? manifest::kLinearOutputMin
                       : manifest::kAngularOutputMin) &&
        channel.output_max ==
               (linear ? manifest::kLinearOutputMax
                       : manifest::kAngularOutputMax) &&
        channel.integer_delay_steps ==
               (linear ? manifest::kLinearIntegerDelaySteps
                       : manifest::kAngularIntegerDelaySteps) &&
        channel.fractional_delay_sec ==
               (linear ? manifest::kLinearFractionalDelaySec
                       : manifest::kAngularFractionalDelaySec);
}

bool finiteInRange(double value, double lower, double upper) {
    return std::isfinite(value) && value >= lower && value <= upper;
}

bool validInitialState(const ExecutionHorizonContext& context) {
    const ExecutionAugmentedState& state = context.initial_state;
    if (!state.valid || !std::isfinite(context.initial_progress_s) ||
        context.initial_progress_s < 0.0 ||
        !std::isfinite(state.robot.x) || !std::isfinite(state.robot.y) ||
        !std::isfinite(state.robot.yaw) ||
        !finiteInRange(
            state.robot.v,
            manifest::kLinearOutputMin,
            manifest::kLinearOutputMax) ||
        !finiteInRange(
            state.robot.omega,
            manifest::kAngularOutputMin,
            manifest::kAngularOutputMax) ||
        !std::isfinite(state.slosh.eta_x) ||
        !std::isfinite(state.slosh.eta_x_dot) ||
        !std::isfinite(state.slosh.eta_y) ||
        !std::isfinite(state.slosh.eta_y_dot) ||
        state.robot.v != state.linear.actuator_output ||
        state.robot.omega != state.angular.actuator_output ||
        state.linear.pending_commands.size() !=
            static_cast<std::size_t>(manifest::kLinearBufferCount) ||
        state.angular.pending_commands.size() !=
            static_cast<std::size_t>(manifest::kAngularBufferCount)) {
        return false;
    }
    for (double value : state.linear.pending_commands) {
        if (!finiteInRange(
                value,
                manifest::kLinearOutputMin,
                manifest::kLinearOutputMax)) {
            return false;
        }
    }
    for (double value : state.angular.pending_commands) {
        if (!finiteInRange(
                value,
                manifest::kAngularOutputMin,
                manifest::kAngularOutputMax)) {
            return false;
        }
    }
    return true;
}

std::array<double, manifest::kStateCount> serializeInitialState(
    const ExecutionHorizonContext& context) {
    std::array<double, manifest::kStateCount> state{};
    state[0] = context.initial_state.robot.x;
    state[1] = context.initial_state.robot.y;
    state[2] = context.initial_state.robot.yaw;
    state[3] = context.initial_state.robot.v;
    state[4] = context.initial_progress_s;
    state[5] = context.initial_state.robot.omega;
    state[6] = context.initial_state.slosh.eta_x;
    state[7] = context.initial_state.slosh.eta_x_dot;
    state[8] = context.initial_state.slosh.eta_y;
    state[9] = context.initial_state.slosh.eta_y_dot;
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kLinearBufferOffset + index)] =
                context.initial_state.linear.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kAngularBufferOffset + index)] =
                context.initial_state.angular.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    return state;
}

}  // namespace

struct DelayAugmentedPhaseAcadosSolver::Impl {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    spmpc_delay_augmented_phase_solver_capsule* capsule = nullptr;
#endif
};

DelayAugmentedPhaseAcadosSolver::DelayAugmentedPhaseAcadosSolver()
    : impl_(new Impl()) {}

DelayAugmentedPhaseAcadosSolver::~DelayAugmentedPhaseAcadosSolver() {
    reset();
}

std::uint32_t DelayAugmentedPhaseAcadosSolver::compiledCapabilities() {
    return manifest::kCapabilities;
}

bool DelayAugmentedPhaseAcadosSolver::validateContextContract(
    const ExecutionHorizonContext& context,
    std::uint32_t required_capabilities,
    std::string& error) {
    error.clear();
    if (required_capabilities == 0u ||
        (manifest::kCapabilities & required_capabilities) !=
            required_capabilities) {
        error = "delay-augmented solver capability mismatch";
        return false;
    }
    if (!context.active ||
        context.contract.schema_version !=
            manifest::kExecutionContractSchemaVersion ||
        context.contract.contract_id != manifest::kContractId ||
        context.contract.contract_hash != manifest::kContractHash ||
        context.contract.dt != manifest::kDt ||
        !sameChannel(context.contract.linear, true) ||
        !sameChannel(context.contract.angular, false)) {
        error = "delay-augmented solver execution contract mismatch";
        return false;
    }
    if (context.execution_front_steps != manifest::kExecutionFrontSteps ||
        context.liquid_horizon_steps != manifest::kLiquidHorizonSteps ||
        context.horizon_steps != manifest::kHorizonSteps ||
        !validStamp(context.initial_epoch_ns) ||
        !validInitialState(context)) {
        error = "delay-augmented solver context shape mismatch";
        return false;
    }
    const StampNs physical_front = addSeconds(
        context.initial_epoch_ns,
        std::max(
            manifest::kLinearDelaySec,
            manifest::kAngularDelaySec));
    const StampNs grid_front = addSeconds(
        context.initial_epoch_ns,
        manifest::kExecutionFrontSteps * manifest::kDt);
    const StampNs terminal = addSeconds(
        context.initial_epoch_ns,
        manifest::kHorizonSteps * manifest::kDt);
    if (!validStamp(physical_front) || !validStamp(grid_front) ||
        !validStamp(terminal) ||
        context.physical_front_epoch_ns != physical_front ||
        context.grid_front_epoch_ns != grid_front ||
        context.terminal_epoch_ns != terminal) {
        error = "delay-augmented solver epoch mismatch";
        return false;
    }
    return true;
}

bool DelayAugmentedPhaseAcadosSolver::compiled() {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    return true;
#else
    return false;
#endif
}

void DelayAugmentedPhaseAcadosSolver::reset() {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (impl_->capsule != nullptr) {
        spmpc_delay_augmented_phase_acados_free(impl_->capsule);
        spmpc_delay_augmented_phase_acados_free_capsule(impl_->capsule);
        impl_->capsule = nullptr;
    }
#endif
}

bool DelayAugmentedPhaseAcadosSolver::create(
    const ExecutionHorizonContext& context,
    std::uint32_t required_capabilities,
    std::string& error) {
    reset();
    if (!validateContextContract(
            context, required_capabilities, error)) {
        return false;
    }
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NX == manifest::kStateCount,
        "generated capsule state width drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NU == manifest::kControlCount,
        "generated capsule control width drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_N == manifest::kHorizonSteps,
        "generated capsule horizon drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NP == 0,
        "candidate capsule unexpectedly requires parameters");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NBX == manifest::kStateBoundCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NBX0 ==
            manifest::kInitialStateBoundCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NBXN ==
            manifest::kTerminalStateBoundCount,
        "generated capsule state-bound contract drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NBU ==
            manifest::kControlBoundCount,
        "generated capsule control-bound contract drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NH ==
            manifest::kPublishedCommandConstraintCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NH0 ==
            manifest::kPublishedCommandConstraintCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NHN ==
            manifest::kTerminalPublishedCommandConstraintCount,
        "generated capsule published-command constraint drifted");

    impl_->capsule =
        spmpc_delay_augmented_phase_acados_create_capsule();
    if (impl_->capsule == nullptr ||
        spmpc_delay_augmented_phase_acados_create(impl_->capsule) != 0) {
        if (impl_->capsule != nullptr) {
            spmpc_delay_augmented_phase_acados_free_capsule(impl_->capsule);
            impl_->capsule = nullptr;
        }
        error = "failed to create delay-augmented acados capsule";
        return false;
    }

    const auto initial = serializeInitialState(context);
    ocp_nlp_config* config =
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule);
    ocp_nlp_dims* dims =
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule);
    ocp_nlp_in* input =
        spmpc_delay_augmented_phase_acados_get_nlp_in(impl_->capsule);
    ocp_nlp_out* output =
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule);
    ocp_nlp_constraints_model_set(
        config, dims, input, output, 0, "lbx",
        const_cast<double*>(initial.data()));
    ocp_nlp_constraints_model_set(
        config, dims, input, output, 0, "ubx",
        const_cast<double*>(initial.data()));
    const std::array<double, manifest::kControlCount> zero_control{};
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        ocp_nlp_out_set(
            config, dims, output, input, stage, "x",
            const_cast<double*>(initial.data()));
        if (stage < manifest::kHorizonSteps) {
            ocp_nlp_out_set(
                config, dims, output, input, stage, "u",
                const_cast<double*>(zero_control.data()));
        }
    }
    return true;
#else
    error = "delay-augmented acados capsule is not compiled";
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::ready() const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    return impl_->capsule != nullptr;
#else
    return false;
#endif
}

int DelayAugmentedPhaseAcadosSolver::stateWidth() const {
    return ready() ? manifest::kStateCount : 0;
}

int DelayAugmentedPhaseAcadosSolver::controlWidth() const {
    return ready() ? manifest::kControlCount : 0;
}

int DelayAugmentedPhaseAcadosSolver::horizonSteps() const {
    return ready() ? manifest::kHorizonSteps : 0;
}

bool DelayAugmentedPhaseAcadosSolver::setControlGuess(
    int stage, const double* control) {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || control == nullptr || stage < 0 ||
        stage >= manifest::kHorizonSteps) {
        return false;
    }
    ocp_nlp_out_set(
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_in(impl_->capsule),
        stage, "u", const_cast<double*>(control));
    return true;
#else
    (void)stage;
    (void)control;
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::getState(
    int stage, double* state) const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || state == nullptr || stage < 0 ||
        stage > manifest::kHorizonSteps) {
        return false;
    }
    ocp_nlp_out_get(
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule),
        stage, "x", state);
    return true;
#else
    (void)stage;
    (void)state;
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::getControl(
    int stage, double* control) const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || control == nullptr || stage < 0 ||
        stage >= manifest::kHorizonSteps) {
        return false;
    }
    ocp_nlp_out_get(
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule),
        stage, "u", control);
    return true;
#else
    (void)stage;
    (void)control;
    return false;
#endif
}

int DelayAugmentedPhaseAcadosSolver::solve() {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    return ready()
        ? spmpc_delay_augmented_phase_acados_solve(impl_->capsule)
        : -1;
#else
    return -1;
#endif
}

double DelayAugmentedPhaseAcadosSolver::solveTimeSec() const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready()) return 0.0;
    double solve_time = 0.0;
    ocp_nlp_get(
        spmpc_delay_augmented_phase_acados_get_nlp_solver(impl_->capsule),
        "time_tot", &solve_time);
    return solve_time;
#else
    return 0.0;
#endif
}

}  // namespace spmpc_local_planner
