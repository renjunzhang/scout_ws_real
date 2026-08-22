#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_spmpc_delay_augmented_phase.h"
#include "acados_solver_spmpc_delay_augmented_phase_rti.h"
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

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
    manifest::kPublishedResidualBounds ==
        DELAY_AUGMENTED_PUBLISHED_RESIDUAL_BOUNDS,
    "generated residual-bound capability bit drifted");
static_assert(
    manifest::kCapabilities == kDelayAugmentedPhaseFormalCapabilities,
    "generated delay-augmented capability mask drifted");
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

// Dimensionless OCP basis.  acados optimizes in scaled variables
// xs = x / scale_x, us = u / scale_u so the KKT system is well-conditioned;
// the capsule boundary below converts between physical and scaled units.
// The per-index mapping must match the codegen state_scaling_vectors()
// channel-by-channel: base states use their own scale, the linear pending
// queue uses the v scale, and the angular pending queue uses the omega scale.
double delayAugmentedStateScale(int index) {
    if (index == 0 || index == 1) return manifest::kPositionScale;
    if (index == 2) return manifest::kYawScale;
    if (index == 3) return manifest::kVelocityScale;
    if (index == 4) return manifest::kProgressScale;
    if (index == 5) return manifest::kAngularVelocityScale;
    if (index == 6 || index == 8) return manifest::kEtaScale;
    if (index == 7 || index == 9) return manifest::kEtaDotScale;
    if (index >= manifest::kLinearBufferOffset &&
        index < manifest::kLinearBufferOffset +
                 manifest::kLinearBufferCount) {
        return manifest::kVelocityScale;
    }
    return manifest::kAngularVelocityScale;
}

double delayAugmentedControlScale(int index) {
    if (index == 0) return manifest::kAccelerationScale;
    if (index == 1) return manifest::kAngularAccelerationScale;
    return manifest::kProgressRateScale;
}

std::array<double, manifest::kStateCount> scaleStateToOcp(
    const std::array<double, manifest::kStateCount>& physical) {
    std::array<double, manifest::kStateCount> scaled{};
    for (int index = 0; index < manifest::kStateCount; ++index) {
        scaled[static_cast<std::size_t>(index)] =
            physical[static_cast<std::size_t>(index)] /
            delayAugmentedStateScale(index);
    }
    return scaled;
}

std::array<double, manifest::kStateCount> unscaleStateFromOcp(
    const std::array<double, manifest::kStateCount>& scaled) {
    std::array<double, manifest::kStateCount> physical{};
    for (int index = 0; index < manifest::kStateCount; ++index) {
        physical[static_cast<std::size_t>(index)] =
            scaled[static_cast<std::size_t>(index)] *
            delayAugmentedStateScale(index);
    }
    return physical;
}

std::array<double, manifest::kControlCount> scaleControlToOcp(
    const std::array<double, manifest::kControlCount>& physical) {
    std::array<double, manifest::kControlCount> scaled{};
    for (int index = 0; index < manifest::kControlCount; ++index) {
        scaled[static_cast<std::size_t>(index)] =
            physical[static_cast<std::size_t>(index)] /
            delayAugmentedControlScale(index);
    }
    return scaled;
}

std::array<double, manifest::kControlCount> unscaleControlFromOcp(
    const std::array<double, manifest::kControlCount>& scaled) {
    std::array<double, manifest::kControlCount> physical{};
    for (int index = 0; index < manifest::kControlCount; ++index) {
        physical[static_cast<std::size_t>(index)] =
            scaled[static_cast<std::size_t>(index)] *
            delayAugmentedControlScale(index);
    }
    return physical;
}

std::array<double, manifest::kStateCount> serializeState(
    const DelayAugmentedPhaseState& phase_state) {
    std::array<double, manifest::kStateCount> state{};
    state[0] = phase_state.execution.robot.x;
    state[1] = phase_state.execution.robot.y;
    state[2] = phase_state.execution.robot.yaw;
    state[3] = phase_state.execution.robot.v;
    state[4] = phase_state.progress_s;
    state[5] = phase_state.execution.robot.omega;
    state[6] = phase_state.execution.slosh.eta_x;
    state[7] = phase_state.execution.slosh.eta_x_dot;
    state[8] = phase_state.execution.slosh.eta_y;
    state[9] = phase_state.execution.slosh.eta_y_dot;
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kLinearBufferOffset + index)] =
                phase_state.execution.linear.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kAngularBufferOffset + index)] =
                phase_state.execution.angular.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    return state;
}

bool validDecodedState(
    const std::array<double, manifest::kStateCount>& state) {
    if (!std::all_of(state.begin(), state.end(), [](double value) {
            return std::isfinite(value);
        }) || state[4] < 0.0 ||
        !finiteInRange(state[3], manifest::kLinearOutputMin,
                       manifest::kLinearOutputMax) ||
        !finiteInRange(state[5], manifest::kAngularOutputMin,
                       manifest::kAngularOutputMax)) {
        return false;
    }
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        if (!finiteInRange(
                state[static_cast<std::size_t>(
                    manifest::kLinearBufferOffset + index)],
                manifest::kLinearOutputMin,
                manifest::kLinearOutputMax)) {
            return false;
        }
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        if (!finiteInRange(
                state[static_cast<std::size_t>(
                    manifest::kAngularBufferOffset + index)],
                manifest::kAngularOutputMin,
                manifest::kAngularOutputMax)) {
            return false;
        }
    }
    return true;
}

double wrappedAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
class GeneratedCapsule {
public:
    virtual ~GeneratedCapsule() = default;
    virtual bool create() = 0;
    virtual void reset() = 0;
    virtual int updateParameters(int stage, double* values, int count) = 0;
    virtual int solve() = 0;
    virtual ocp_nlp_config* config() const = 0;
    virtual ocp_nlp_dims* dims() const = 0;
    virtual ocp_nlp_in* input() const = 0;
    virtual ocp_nlp_out* output() const = 0;
    virtual ocp_nlp_solver* solver() const = 0;
};

struct FullSqpCapsuleTraits {
    using Capsule = spmpc_delay_augmented_phase_solver_capsule;
    static Capsule* allocate() {
        return spmpc_delay_augmented_phase_acados_create_capsule();
    }
    static int create(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_create(capsule);
    }
    static int free(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_free(capsule);
    }
    static int release(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_free_capsule(capsule);
    }
    static int update(Capsule* capsule, int stage, double* values,
                      int count) {
        return spmpc_delay_augmented_phase_acados_update_params(
            capsule, stage, values, count);
    }
    static int solve(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_solve(capsule);
    }
    static ocp_nlp_config* config(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_get_nlp_config(capsule);
    }
    static ocp_nlp_dims* dims(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_get_nlp_dims(capsule);
    }
    static ocp_nlp_in* input(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_get_nlp_in(capsule);
    }
    static ocp_nlp_out* output(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_get_nlp_out(capsule);
    }
    static ocp_nlp_solver* solver(Capsule* capsule) {
        return spmpc_delay_augmented_phase_acados_get_nlp_solver(capsule);
    }
};

struct RtiCapsuleTraits {
    using Capsule = spmpc_delay_augmented_phase_rti_solver_capsule;
    static Capsule* allocate() {
        return spmpc_delay_augmented_phase_rti_acados_create_capsule();
    }
    static int create(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_create(capsule);
    }
    static int free(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_free(capsule);
    }
    static int release(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_free_capsule(capsule);
    }
    static int update(Capsule* capsule, int stage, double* values,
                      int count) {
        return spmpc_delay_augmented_phase_rti_acados_update_params(
            capsule, stage, values, count);
    }
    static int solve(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_solve(capsule);
    }
    static ocp_nlp_config* config(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_get_nlp_config(capsule);
    }
    static ocp_nlp_dims* dims(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_get_nlp_dims(capsule);
    }
    static ocp_nlp_in* input(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_get_nlp_in(capsule);
    }
    static ocp_nlp_out* output(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_get_nlp_out(capsule);
    }
    static ocp_nlp_solver* solver(Capsule* capsule) {
        return spmpc_delay_augmented_phase_rti_acados_get_nlp_solver(capsule);
    }
};

template <typename Traits>
class GeneratedCapsuleAdapter final : public GeneratedCapsule {
public:
    ~GeneratedCapsuleAdapter() override { reset(); }

    bool create() override {
        reset();
        capsule_ = Traits::allocate();
        if (capsule_ == nullptr || Traits::create(capsule_) != 0) {
            reset();
            return false;
        }
        return true;
    }

    void reset() override {
        if (capsule_ != nullptr) {
            Traits::free(capsule_);
            Traits::release(capsule_);
            capsule_ = nullptr;
        }
    }

    int updateParameters(int stage, double* values, int count) override {
        return Traits::update(capsule_, stage, values, count);
    }
    int solve() override { return Traits::solve(capsule_); }
    ocp_nlp_config* config() const override {
        return Traits::config(capsule_);
    }
    ocp_nlp_dims* dims() const override {
        return Traits::dims(capsule_);
    }
    ocp_nlp_in* input() const override {
        return Traits::input(capsule_);
    }
    ocp_nlp_out* output() const override {
        return Traits::output(capsule_);
    }
    ocp_nlp_solver* solver() const override {
        return Traits::solver(capsule_);
    }

private:
    typename Traits::Capsule* capsule_ = nullptr;
};
#endif

}  // namespace

struct DelayAugmentedPhaseAcadosSolver::Impl {
    DelayAugmentedPhaseSolveDiagnostics diagnostics;
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    std::unique_ptr<GeneratedCapsule> capsule;
#endif
};

DelayAugmentedPhaseAcadosSolver::DelayAugmentedPhaseAcadosSolver(
    DelayAugmentedPhaseAcadosBackend backend)
    : backend_(backend), impl_(new Impl()) {}

DelayAugmentedPhaseAcadosSolver::~DelayAugmentedPhaseAcadosSolver() {
    reset();
}

std::uint32_t DelayAugmentedPhaseAcadosSolver::compiledCapabilities() {
    return manifest::kCapabilities;
}

DelayAugmentedPhaseCompiledContract
DelayAugmentedPhaseAcadosSolver::compiledContract() {
    DelayAugmentedPhaseCompiledContract compiled;
    compiled.execution.schema_version =
        manifest::kExecutionContractSchemaVersion;
    compiled.execution.contract_id = manifest::kContractId;
    compiled.execution.contract_hash = manifest::kContractHash;
    compiled.execution.dt = manifest::kDt;
    compiled.execution.linear.delay_sec = manifest::kLinearDelaySec;
    compiled.execution.linear.time_constant_sec =
        manifest::kLinearTimeConstantSec;
    compiled.execution.linear.positive_gain = manifest::kLinearPositiveGain;
    compiled.execution.linear.negative_gain = manifest::kLinearNegativeGain;
    compiled.execution.linear.deadzone = manifest::kLinearDeadzone;
    compiled.execution.linear.output_min = manifest::kLinearOutputMin;
    compiled.execution.linear.output_max = manifest::kLinearOutputMax;
    compiled.execution.linear.integer_delay_steps =
        manifest::kLinearIntegerDelaySteps;
    compiled.execution.linear.fractional_delay_sec =
        manifest::kLinearFractionalDelaySec;
    compiled.execution.angular.delay_sec = manifest::kAngularDelaySec;
    compiled.execution.angular.time_constant_sec =
        manifest::kAngularTimeConstantSec;
    compiled.execution.angular.positive_gain =
        manifest::kAngularPositiveGain;
    compiled.execution.angular.negative_gain =
        manifest::kAngularNegativeGain;
    compiled.execution.angular.deadzone = manifest::kAngularDeadzone;
    compiled.execution.angular.output_min = manifest::kAngularOutputMin;
    compiled.execution.angular.output_max = manifest::kAngularOutputMax;
    compiled.execution.angular.integer_delay_steps =
        manifest::kAngularIntegerDelaySteps;
    compiled.execution.angular.fractional_delay_sec =
        manifest::kAngularFractionalDelaySec;
    compiled.slosh.dt = manifest::kDt;
    compiled.slosh.container_radius = manifest::kContainerRadius;
    compiled.slosh.liquid_height = manifest::kLiquidHeight;
    compiled.slosh.liquid_density = manifest::kLiquidDensity;
    compiled.slosh.damping_ratio = manifest::kDampingRatio;
    compiled.slosh.mode_index = manifest::kModeIndex;
    compiled.slosh.slosh_height_ref = manifest::kSloshHeightRef;
    compiled.slosh.slosh_eta_dot_ratio = manifest::kSloshEtaDotRatio;
    compiled.slosh.use_linear_model = manifest::kUseLinearModel;
    compiled.slosh.use_parabola_term = manifest::kUseParabolaTerm;
    compiled.state_width = manifest::kStateCount;
    compiled.control_width = manifest::kControlCount;
    compiled.horizon_steps = manifest::kHorizonSteps;
    compiled.execution_front_steps = manifest::kExecutionFrontSteps;
    compiled.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    compiled.parameter_schema_version = manifest::kParameterSchemaVersion;
    compiled.parameter_schema_id = manifest::kParameterSchemaId;
    compiled.parameter_schema_hash = manifest::kParameterSchemaHash;
    compiled.execution_compatibility_contract =
        manifest::kExecutionCompatibilityContract;
    compiled.capabilities = manifest::kCapabilities;
    compiled.acceleration_max = manifest::kAccelerationMax;
    compiled.angular_acceleration_max =
        manifest::kAngularAccelerationMax;
    compiled.progress_rate_max = manifest::kProgressRateMax;
    compiled.solver_id = manifest::kSolverId;
    compiled.nlp_solver_type = manifest::kNlpSolverType;
    compiled.globalization = manifest::kGlobalization;
    compiled.solver_config_hash = manifest::kSolverConfigHash;
    return compiled;
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
    impl_->diagnostics = DelayAugmentedPhaseSolveDiagnostics{};
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    impl_->capsule.reset();
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
        SPMPC_DELAY_AUGMENTED_PHASE_NP == manifest::kParameterCount,
        "generated capsule parameter image drifted");
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
            manifest::kStageConstraintCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NH0 ==
            manifest::kStageConstraintCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NHN ==
            manifest::kTerminalRecoveryConstraintCount,
        "generated capsule terminal recovery constraint drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NX == manifest::kStateCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NU == manifest::kControlCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_N == manifest::kHorizonSteps &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NP == manifest::kParameterCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NBX ==
            manifest::kStateBoundCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NBX0 ==
            manifest::kInitialStateBoundCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NBXN ==
            manifest::kTerminalStateBoundCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NBU ==
            manifest::kControlBoundCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NH ==
            manifest::kStageConstraintCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NH0 ==
            manifest::kStageConstraintCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_RTI_NHN ==
            manifest::kTerminalRecoveryConstraintCount,
        "generated RTI reference capsule contract drifted");

    if (backend_ == DelayAugmentedPhaseAcadosBackend::RtiReference) {
        impl_->capsule.reset(
            new GeneratedCapsuleAdapter<RtiCapsuleTraits>());
    } else {
        impl_->capsule.reset(
            new GeneratedCapsuleAdapter<FullSqpCapsuleTraits>());
    }
    if (!impl_->capsule->create()) {
        impl_->capsule.reset();
        error = "failed to create delay-augmented acados capsule";
        return false;
    }

    const auto initial_physical = serializeInitialState(context);
    const auto initial = scaleStateToOcp(initial_physical);
    ocp_nlp_config* config = impl_->capsule->config();
    ocp_nlp_dims* dims = impl_->capsule->dims();
    ocp_nlp_in* input = impl_->capsule->input();
    ocp_nlp_out* output = impl_->capsule->output();
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

bool DelayAugmentedPhaseAcadosSolver::setParameterImage(
    const DelayAugmentedPhaseParameterMatrix& parameters,
    std::string& error) {
    error.clear();
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || !parameters.hasCanonicalShape()) {
        error = "delay-augmented parameter image shape mismatch";
        return false;
    }
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        const double* data = parameters.stageData(stage);
        if (data == nullptr ||
            impl_->capsule->updateParameters(
                stage,
                const_cast<double*>(data),
                manifest::kParameterCount) != 0) {
            error = "failed to update delay-augmented parameters at stage " +
                std::to_string(stage);
            return false;
        }
    }
    return true;
#else
    (void)parameters;
    error = "delay-augmented acados capsule is not compiled";
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::ready() const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    return static_cast<bool>(impl_->capsule);
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
    std::array<double, manifest::kControlCount> physical{};
    for (int index = 0; index < manifest::kControlCount; ++index) {
        physical[static_cast<std::size_t>(index)] = control[index];
    }
    const auto scaled = scaleControlToOcp(physical);
    ocp_nlp_out_set(
        impl_->capsule->config(),
        impl_->capsule->dims(),
        impl_->capsule->output(),
        impl_->capsule->input(),
        stage, "u", const_cast<double*>(scaled.data()));
    return true;
#else
    (void)stage;
    (void)control;
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::setCausalWarmStart(
    const ExecutionHorizonContext& context,
    const std::vector<DelayAugmentedPhaseControl>& controls,
    std::string& error) {
    error.clear();
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || controls.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps)) {
        error = "invalid causal warm-start shape";
        return false;
    }
    if (!validateContextContract(
            context, kDelayAugmentedPhaseFormalCapabilities, error)) {
        return false;
    }
    for (std::size_t stage = 0; stage < controls.size(); ++stage) {
        const DelayAugmentedPhaseControl& control = controls[stage];
        if (!finiteInRange(control.acceleration,
                           -manifest::kAccelerationMax,
                           manifest::kAccelerationMax) ||
            !finiteInRange(control.angular_acceleration,
                           -manifest::kAngularAccelerationMax,
                           manifest::kAngularAccelerationMax) ||
            !finiteInRange(control.progress_rate, 0.0,
                           manifest::kProgressRateMax)) {
            error = "causal warm-start control out of bounds at stage " +
                std::to_string(stage);
            return false;
        }
    }

    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    DelayAugmentedPhaseDynamics dynamics;
    if (!dynamics.configure(compiled.execution, compiled.slosh, error)) {
        error = "causal warm-start model rejected: " + error;
        return false;
    }
    const DelayAugmentedPhaseRolloutResult rollout =
        dynamics.rollout(context, controls);
    if (!rollout.valid || rollout.states.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps + 1)) {
        error = "causal warm-start rollout failed: " + rollout.status;
        return false;
    }

    ocp_nlp_config* config = impl_->capsule->config();
    ocp_nlp_dims* dims = impl_->capsule->dims();
    ocp_nlp_in* input = impl_->capsule->input();
    ocp_nlp_out* output = impl_->capsule->output();
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        const auto state_physical = serializeState(
            rollout.states[static_cast<std::size_t>(stage)]);
        const auto state = scaleStateToOcp(state_physical);
        ocp_nlp_out_set(
            config, dims, output, input, stage, "x",
            const_cast<double*>(state.data()));
        if (stage < manifest::kHorizonSteps) {
            const DelayAugmentedPhaseControl& control = controls[
                static_cast<std::size_t>(stage)];
            const std::array<double, manifest::kControlCount> raw_physical = {{
                control.acceleration,
                control.angular_acceleration,
                control.progress_rate,
            }};
            const auto raw = scaleControlToOcp(raw_physical);
            ocp_nlp_out_set(
                config, dims, output, input, stage, "u",
                const_cast<double*>(raw.data()));
        }
    }
    return true;
#else
    (void)context;
    (void)controls;
    error = "delay-augmented acados capsule is not compiled";
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
        impl_->capsule->config(),
        impl_->capsule->dims(),
        impl_->capsule->output(),
        stage, "x", state);
    for (int index = 0; index < manifest::kStateCount; ++index) {
        state[index] *= delayAugmentedStateScale(index);
    }
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
        impl_->capsule->config(),
        impl_->capsule->dims(),
        impl_->capsule->output(),
        stage, "u", control);
    for (int index = 0; index < manifest::kControlCount; ++index) {
        control[index] *= delayAugmentedControlScale(index);
    }
    return true;
#else
    (void)stage;
    (void)control;
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::captureTrajectory(
    std::vector<double>& states,
    std::vector<double>& controls) const {
    states.clear();
    controls.clear();
    if (!ready()) return false;
    states.resize(static_cast<std::size_t>(
        (manifest::kHorizonSteps + 1) * manifest::kStateCount));
    controls.resize(static_cast<std::size_t>(
        manifest::kHorizonSteps * manifest::kControlCount));
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        if (!getState(
                stage,
                states.data() + static_cast<std::size_t>(
                    stage * manifest::kStateCount))) {
            states.clear();
            controls.clear();
            return false;
        }
        if (stage < manifest::kHorizonSteps &&
            !getControl(
                stage,
                controls.data() + static_cast<std::size_t>(
                    stage * manifest::kControlCount))) {
            states.clear();
            controls.clear();
            return false;
        }
    }
    return true;
}

bool DelayAugmentedPhaseAcadosSolver::evaluateCurrentResiduals(
    DelayAugmentedPhaseResidualDiagnostics& diagnostics) const {
    diagnostics = DelayAugmentedPhaseResidualDiagnostics{};
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready()) return false;
    ocp_nlp_solver* solver = impl_->capsule->solver();
    ocp_nlp_eval_residuals(
        solver, impl_->capsule->input(), impl_->capsule->output());
    ocp_nlp_get(solver, "res_stat", &diagnostics.stationarity);
    ocp_nlp_get(solver, "res_eq", &diagnostics.equality);
    ocp_nlp_get(solver, "res_ineq", &diagnostics.inequality);
    ocp_nlp_get(solver, "res_comp", &diagnostics.complementarity);
    diagnostics.evaluated =
        std::isfinite(diagnostics.stationarity) &&
        std::isfinite(diagnostics.equality) &&
        std::isfinite(diagnostics.inequality) &&
        std::isfinite(diagnostics.complementarity);
    return diagnostics.evaluated;
#else
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::perStageStationarity(
    int stage, std::vector<double>& values) const {
    values.clear();
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || stage < 0 || stage > manifest::kHorizonSteps) {
        return false;
    }
    const int width = manifest::kStateCount +
        (stage < manifest::kHorizonSteps ? manifest::kControlCount : 0);
    values.resize(static_cast<std::size_t>(width), 0.0);
    ocp_nlp_get_at_stage(
        impl_->capsule->solver(), stage, "res_stat", values.data());
    return true;
#else
    (void)stage;
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::perStagePi(
    int stage, std::vector<double>& values) const {
    values.clear();
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    // pi lives per stage for the dynamics between stage and stage+1, so it is
    // only defined for stage < N (no dynamics enters the terminal node).
    if (!ready() || stage < 0 || stage >= manifest::kHorizonSteps) {
        return false;
    }
    const int width = ocp_nlp_dims_get_from_attr(
        impl_->capsule->config(), impl_->capsule->dims(),
        impl_->capsule->output(), stage, "pi");
    if (width <= 0) return false;
    values.resize(static_cast<std::size_t>(width), 0.0);
    ocp_nlp_out_get(
        impl_->capsule->config(), impl_->capsule->dims(),
        impl_->capsule->output(), stage, "pi", values.data());
    return true;
#else
    (void)stage;
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::perStageLam(
    int stage, std::vector<double>& values) const {
    values.clear();
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || stage < 0 || stage > manifest::kHorizonSteps) {
        return false;
    }
    const int width = ocp_nlp_dims_get_from_attr(
        impl_->capsule->config(), impl_->capsule->dims(),
        impl_->capsule->output(), stage, "lam");
    if (width <= 0) return false;
    values.resize(static_cast<std::size_t>(width), 0.0);
    ocp_nlp_out_get(
        impl_->capsule->config(), impl_->capsule->dims(),
        impl_->capsule->output(), stage, "lam", values.data());
    return true;
#else
    (void)stage;
    return false;
#endif
}

DelayAugmentedPhaseConstraintAudit
DelayAugmentedPhaseAcadosSolver::auditTrajectory(
    const ExecutionHorizonContext& context,
    const DelayAugmentedPhaseParameterMatrix& parameters,
    const std::vector<double>& states,
    const std::vector<double>& controls) {
    DelayAugmentedPhaseConstraintAudit audit;
    audit.tolerance = manifest::kMaxInequalityResidual;
    const std::size_t expected_states = static_cast<std::size_t>(
        (manifest::kHorizonSteps + 1) * manifest::kStateCount);
    const std::size_t expected_controls = static_cast<std::size_t>(
        manifest::kHorizonSteps * manifest::kControlCount);
    if (!parameters.hasCanonicalShape() ||
        states.size() != expected_states ||
        controls.size() != expected_controls) {
        audit.status = "TRAJECTORY_SHAPE_MISMATCH";
        return audit;
    }

    const auto update_max = [&audit](
        const DelayAugmentedPhaseNamedConstraintDiagnostics& item) {
        if (item.violation > audit.max_violation) {
            audit.max_violation = item.violation;
            audit.max_violation_stage = item.stage;
            audit.max_violation_index = item.index;
            audit.max_violation_name = item.name;
            audit.max_violation_value = item.value;
        }
    };
    const auto append_bound = [&update_max](
        std::vector<DelayAugmentedPhaseNamedConstraintDiagnostics>& output,
        int stage, int index, const std::string& name, double value,
        double lower, double upper, double normalized_error) {
        DelayAugmentedPhaseNamedConstraintDiagnostics item;
        item.stage = stage;
        item.index = index;
        item.name = name;
        item.value = value;
        item.lower = lower;
        item.upper = upper;
        item.normalized_error = normalized_error;
        item.violation = std::max(
            0.0, std::max(lower - value, value - upper));
        output.push_back(item);
        update_max(item);
    };

    std::vector<DelayAugmentedPhaseControl> decoded_controls;
    decoded_controls.reserve(manifest::kHorizonSteps);
    const int execution_indices[manifest::kExecutionBoundCount] = {
        3, 5, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
    const char* execution_component_names[
        manifest::kExecutionBoundCount] = {
        "linear_output",
        "angular_output",
        "linear_pending_0",
        "linear_pending_1",
        "linear_pending_2",
        "linear_pending_3",
        "linear_pending_4",
        "angular_pending_0",
        "angular_pending_1",
        "angular_pending_2",
        "angular_pending_3",
        "angular_pending_4",
        "angular_pending_5",
        "angular_pending_6",
    };
    for (int stage = 0; stage < manifest::kHorizonSteps; ++stage) {
        const double* state = states.data() + static_cast<std::size_t>(
            stage * manifest::kStateCount);
        const double* control = controls.data() + static_cast<std::size_t>(
            stage * manifest::kControlCount);
        const double* parameter = parameters.stageData(stage);
        if (!std::all_of(
                state, state + manifest::kStateCount,
                [](double value) { return std::isfinite(value); }) ||
            !std::all_of(
                control, control + manifest::kControlCount,
                [](double value) { return std::isfinite(value); })) {
            audit.status = "NONFINITE_TRAJECTORY";
            return audit;
        }
        DelayAugmentedPhaseControl decoded;
        decoded.acceleration = control[0];
        decoded.angular_acceleration = control[1];
        decoded.progress_rate = control[2];
        decoded_controls.push_back(decoded);
        append_bound(
            audit.control_constraints, stage, 0, "acceleration",
            control[0], -manifest::kAccelerationMax,
            manifest::kAccelerationMax,
            std::abs(control[0]) / manifest::kAccelerationMax);
        append_bound(
            audit.control_constraints, stage, 1,
            "angular_acceleration", control[1],
            -manifest::kAngularAccelerationMax,
            manifest::kAngularAccelerationMax,
            std::abs(control[1]) / manifest::kAngularAccelerationMax);
        append_bound(
            audit.control_constraints, stage, 2, "progress_rate",
            control[2], 0.0, manifest::kProgressRateMax,
            control[2] / manifest::kProgressRateMax);

        const double published_v =
            state[manifest::kLinearBufferOffset +
                  manifest::kLinearBufferCount - 1] +
            control[0] * manifest::kDt;
        const double published_omega =
            state[manifest::kAngularBufferOffset +
                  manifest::kAngularBufferCount - 1] +
            control[1] * manifest::kDt;
        const double nominal_v =
            parameter[manifest::kNominalPublishOffset];
        const double nominal_omega =
            parameter[manifest::kNominalPublishOffset + 1];
        const double residual_v =
            parameter[manifest::kResidualBoundOffset];
        const double residual_omega =
            parameter[manifest::kResidualBoundOffset + 1];
        append_bound(
            audit.stage_constraints, stage, 0,
            "published_linear_envelope", published_v,
            manifest::kLinearOutputMin, manifest::kLinearOutputMax,
            0.0);
        append_bound(
            audit.stage_constraints, stage, 1,
            "published_angular_envelope", published_omega,
            manifest::kAngularOutputMin, manifest::kAngularOutputMax,
            0.0);
        append_bound(
            audit.stage_constraints, stage, 2,
            "published_linear_residual_upper",
            published_v - nominal_v - residual_v,
            -1.0e15, 0.0,
            residual_v > 0.0
                ? std::abs(published_v - nominal_v) / residual_v
                : std::abs(published_v - nominal_v));
        append_bound(
            audit.stage_constraints, stage, 3,
            "published_linear_residual_lower",
            nominal_v - published_v - residual_v,
            -1.0e15, 0.0,
            residual_v > 0.0
                ? std::abs(published_v - nominal_v) / residual_v
                : std::abs(published_v - nominal_v));
        append_bound(
            audit.stage_constraints, stage, 4,
            "published_angular_residual_upper",
            published_omega - nominal_omega - residual_omega,
            -1.0e15, 0.0,
            residual_omega > 0.0
                ? std::abs(published_omega - nominal_omega) /
                    residual_omega
                : std::abs(published_omega - nominal_omega));
        append_bound(
            audit.stage_constraints, stage, 5,
            "published_angular_residual_lower",
            nominal_omega - published_omega - residual_omega,
            -1.0e15, 0.0,
            residual_omega > 0.0
                ? std::abs(published_omega - nominal_omega) /
                    residual_omega
                : std::abs(published_omega - nominal_omega));
        for (int bound = 0;
             bound < manifest::kExecutionBoundCount; ++bound) {
            const int state_index = execution_indices[bound];
            const double nominal = parameter[
                manifest::kNominalStateOffset + state_index];
            const double beta = parameter[
                manifest::kExecutionBoundOffset + bound];
            const double error_value = state[state_index] - nominal;
            const int upper_index =
                manifest::kPublishedCommandConstraintCount + 2 * bound;
            const int lower_index = upper_index + 1;
            const double normalized = std::abs(error_value) / beta;
            const std::string prefix = std::string("stage_exec_") +
                execution_component_names[bound];
            append_bound(
                audit.stage_constraints, stage, upper_index,
                prefix + "_upper", error_value - beta,
                -1.0e15, 0.0, normalized);
            append_bound(
                audit.stage_constraints, stage, lower_index,
                prefix + "_lower", -error_value - beta,
                -1.0e15, 0.0, normalized);
        }
    }

    const double* terminal = states.data() + static_cast<std::size_t>(
        manifest::kHorizonSteps * manifest::kStateCount);
    const double* terminal_parameter =
        parameters.stageData(manifest::kHorizonSteps);
    const int gate_indices[manifest::kGateRadiusCount] = {
        0, 1, 2, 3, 5, 6, 7, 8, 9};
    double empirical_metric = 0.0;
    for (int index = 0; index < manifest::kGateRadiusCount; ++index) {
        const int state_index = gate_indices[index];
        double error = terminal[state_index] -
            terminal_parameter[manifest::kNominalStateOffset + state_index];
        if (state_index == 2) error = wrappedAngle(error);
        const double radius = terminal_parameter[
            manifest::kGateRadiusOffset + index];
        empirical_metric += (error / radius) * (error / radius);
    }
    audit.terminal_empirical_metric = empirical_metric;
    audit.terminal_empirical_violation = std::max(0.0,
        empirical_metric - 1.0);
    DelayAugmentedPhaseNamedConstraintDiagnostics empirical;
    empirical.stage = manifest::kHorizonSteps;
    empirical.index = 0;
    empirical.name = "terminal_empirical_9d_ellipsoid";
    empirical.value = empirical_metric - 1.0;
    empirical.lower = -1.0e15;
    empirical.upper = 0.0;
    empirical.normalized_error = std::sqrt(empirical_metric);
    empirical.violation = audit.terminal_empirical_violation;
    update_max(empirical);

    for (int index = 0; index < manifest::kExecutionBoundCount; ++index) {
        const int state_index = execution_indices[index];
        const double beta = terminal_parameter[
            manifest::kExecutionBoundOffset + index];
        const double error = terminal[state_index] - terminal_parameter[
            manifest::kNominalStateOffset + state_index];
        const double normalized = std::abs(error) / beta;
        append_bound(
            audit.terminal_execution_constraints,
            manifest::kHorizonSteps, index + 1,
            std::string("terminal_exec_") +
                execution_component_names[index],
            normalized * normalized - 1.0,
            -1.0e15, 0.0, normalized);
    }

    DelayAugmentedPhaseDynamics dynamics;
    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    std::string error;
    if (!dynamics.configure(compiled.execution, compiled.slosh, error)) {
        audit.status = "CAUSAL_MODEL_REJECTED_" + error;
        return audit;
    }
    const DelayAugmentedPhaseRolloutResult rollout =
        dynamics.rollout(context, decoded_controls);
    if (!rollout.valid || rollout.states.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps + 1)) {
        audit.status = "CAUSAL_ROLLOUT_FAILED_" + rollout.status;
        return audit;
    }
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        const auto causal = serializeState(
            rollout.states[static_cast<std::size_t>(stage)]);
        const double* raw = states.data() + static_cast<std::size_t>(
            stage * manifest::kStateCount);
        for (int index = 0; index < manifest::kStateCount; ++index) {
            const double error_value = std::abs(
                raw[index] - causal[static_cast<std::size_t>(index)]);
            if (error_value > audit.max_causal_state_error) {
                audit.max_causal_state_error = error_value;
                audit.max_causal_state_error_stage = stage;
                audit.max_causal_state_error_index = index;
            }
        }
    }
    if (audit.max_causal_state_error > audit.max_violation) {
        audit.max_violation = audit.max_causal_state_error;
        audit.max_violation_stage = audit.max_causal_state_error_stage;
        audit.max_violation_index = audit.max_causal_state_error_index;
        audit.max_violation_name = "causal_dynamics_state_error";
        audit.max_violation_value = audit.max_causal_state_error;
    }
    audit.evaluated = true;
    audit.passed = audit.max_violation <= audit.tolerance &&
        audit.max_causal_state_error <= manifest::kMaxCausalStateError;
    audit.status = audit.passed ? "OK" : "CONSTRAINT_VIOLATION";
    return audit;
}

int DelayAugmentedPhaseAcadosSolver::solve() {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    impl_->diagnostics = DelayAugmentedPhaseSolveDiagnostics{};
    if (!ready()) {
        impl_->diagnostics.status = "CAPSULE_NOT_READY";
        return -1;
    }
    impl_->diagnostics.optimizer_invoked = true;
    const int status = impl_->capsule->solve();
    impl_->diagnostics.nlp_status = status;
    ocp_nlp_solver* solver = impl_->capsule->solver();
    ocp_nlp_in* input = impl_->capsule->input();
    ocp_nlp_out* output = impl_->capsule->output();
    ocp_nlp_eval_residuals(solver, input, output);
    ocp_nlp_eval_cost(solver, input, output);
    ocp_nlp_get(solver, "qp_status", &impl_->diagnostics.qp_status);
    ocp_nlp_get(solver, "res_stat",
                &impl_->diagnostics.stationarity_residual);
    ocp_nlp_get(solver, "res_eq",
                &impl_->diagnostics.equality_residual);
    ocp_nlp_get(solver, "res_ineq",
                &impl_->diagnostics.inequality_residual);
    ocp_nlp_get(solver, "res_comp",
                &impl_->diagnostics.complementarity_residual);
    ocp_nlp_get(solver, "sqp_iter", &impl_->diagnostics.sqp_iterations);
    ocp_nlp_get(solver, "qp_iter", &impl_->diagnostics.qp_iterations);
    ocp_nlp_get(solver, "cost_value", &impl_->diagnostics.cost);
    impl_->diagnostics.evaluated = true;

    int stat_n = 0;
    int stat_m = 0;
    ocp_nlp_get(solver, "stat_n", &stat_n);
    ocp_nlp_get(solver, "stat_m", &stat_m);
    const int row_count = std::max(
        0, std::min(stat_m, impl_->diagnostics.sqp_iterations + 1));
    if (stat_n > 0 && row_count > 0) {
        std::vector<double> statistics(static_cast<std::size_t>(
            (stat_n + 1) * row_count), 0.0);
        ocp_nlp_get(solver, "statistics", statistics.data());
        for (int row = 0; row < row_count; ++row) {
            DelayAugmentedPhaseIterationDiagnostics iteration;
            iteration.iteration = row;
            if (backend_ ==
                    DelayAugmentedPhaseAcadosBackend::FullSqp &&
                stat_n >= 7) {
                iteration.stationarity = statistics[static_cast<std::size_t>(
                    row + row_count * 1)];
                iteration.equality = statistics[static_cast<std::size_t>(
                    row + row_count * 2)];
                iteration.inequality = statistics[static_cast<std::size_t>(
                    row + row_count * 3)];
                iteration.complementarity = statistics[
                    static_cast<std::size_t>(row + row_count * 4)];
                iteration.qp_status = static_cast<int>(statistics[
                    static_cast<std::size_t>(row + row_count * 5)]);
                iteration.qp_iterations = static_cast<int>(statistics[
                    static_cast<std::size_t>(row + row_count * 6)]);
                iteration.step_length = statistics[
                    static_cast<std::size_t>(row + row_count * 7)];
            } else if (stat_n >= 2) {
                iteration.qp_status = static_cast<int>(statistics[
                    static_cast<std::size_t>(row + row_count * 1)]);
                iteration.qp_iterations = static_cast<int>(statistics[
                    static_cast<std::size_t>(row + row_count * 2)]);
                iteration.step_length = 1.0;
            }
            impl_->diagnostics.iterations.push_back(iteration);
        }
        impl_->diagnostics.step_length =
            impl_->diagnostics.iterations.back().step_length;
    }
    impl_->diagnostics.residual_admitted =
        residualsAdmissible(impl_->diagnostics);
    if (status != 0) {
        impl_->diagnostics.status = "NLP_STATUS_" +
            std::to_string(status);
        return status;
    }
    impl_->diagnostics.status = impl_->diagnostics.residual_admitted
        ? "OK" : "RESIDUAL_REJECTED";
    return impl_->diagnostics.residual_admitted ? 0 : -2;
#else
    impl_->diagnostics = DelayAugmentedPhaseSolveDiagnostics{};
    impl_->diagnostics.status = "CAPSULE_NOT_COMPILED";
    return -1;
#endif
}

const DelayAugmentedPhaseSolveDiagnostics&
DelayAugmentedPhaseAcadosSolver::lastSolveDiagnostics() const {
    return impl_->diagnostics;
}

bool DelayAugmentedPhaseAcadosSolver::residualsAdmissible(
    const DelayAugmentedPhaseSolveDiagnostics& diagnostics) {
    return diagnostics.evaluated && diagnostics.nlp_status == 0 &&
        diagnostics.qp_status == 0 &&
        std::isfinite(diagnostics.stationarity_residual) &&
        std::isfinite(diagnostics.equality_residual) &&
        std::isfinite(diagnostics.inequality_residual) &&
        std::isfinite(diagnostics.complementarity_residual) &&
        diagnostics.stationarity_residual <=
            manifest::kMaxStationarityResidual &&
        diagnostics.equality_residual <=
            manifest::kMaxEqualityResidual &&
        diagnostics.inequality_residual <=
            manifest::kMaxInequalityResidual &&
        diagnostics.complementarity_residual <=
            manifest::kMaxComplementarityResidual;
}

bool DelayAugmentedPhaseAcadosSolver::causalRollout(
    const ExecutionHorizonContext& context,
    DelayAugmentedPhaseRolloutResult& rollout,
    std::string& error) const {
    rollout = DelayAugmentedPhaseRolloutResult{};
    error.clear();
    if (!ready()) {
        error = "delay-augmented capsule is not ready";
        return false;
    }
    if (!validateContextContract(
            context, kDelayAugmentedPhaseFormalCapabilities, error)) {
        return false;
    }

    std::vector<DelayAugmentedPhaseControl> controls;
    controls.reserve(static_cast<std::size_t>(manifest::kHorizonSteps));
    for (int stage = 0; stage < manifest::kHorizonSteps; ++stage) {
        std::array<double, manifest::kControlCount> raw{};
        if (!getControl(stage, raw.data()) ||
            !finiteInRange(raw[0], -manifest::kAccelerationMax,
                           manifest::kAccelerationMax) ||
            !finiteInRange(raw[1], -manifest::kAngularAccelerationMax,
                           manifest::kAngularAccelerationMax) ||
            !finiteInRange(raw[2], 0.0,
                           manifest::kProgressRateMax)) {
            error = "invalid decoded control at stage " +
                std::to_string(stage);
            return false;
        }
        DelayAugmentedPhaseControl control;
        control.acceleration = raw[0];
        control.angular_acceleration = raw[1];
        control.progress_rate = raw[2];
        controls.push_back(control);
    }

    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    DelayAugmentedPhaseDynamics dynamics;
    if (!dynamics.configure(compiled.execution, compiled.slosh, error)) {
        error = "causal audit model rejected: " + error;
        return false;
    }
    rollout = dynamics.rollout(context, controls);
    if (!rollout.valid || rollout.states.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps + 1) ||
        rollout.published_commands.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps)) {
        error = "causal rollout failed: " + rollout.status;
        return false;
    }

    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        std::array<double, manifest::kStateCount> decoded{};
        if (!getState(stage, decoded.data()) ||
            !validDecodedState(decoded)) {
            error = "invalid decoded state at stage " +
                std::to_string(stage);
            return false;
        }
        const auto causal = serializeState(
            rollout.states[static_cast<std::size_t>(stage)]);
        for (std::size_t index = 0; index < decoded.size(); ++index) {
            if (std::abs(decoded[index] - causal[index]) >
                    manifest::kMaxCausalStateError) {
                error = "causal state mismatch at stage " +
                    std::to_string(stage) + " index " +
                    std::to_string(index);
                return false;
            }
        }
    }
    for (std::size_t stage = 0;
         stage < rollout.published_commands.size(); ++stage) {
        const VelocityCommand& command =
            rollout.published_commands[stage];
        if (!finiteInRange(command.linear,
                           manifest::kLinearOutputMin,
                           manifest::kLinearOutputMax) ||
            !finiteInRange(command.angular,
                           manifest::kAngularOutputMin,
                           manifest::kAngularOutputMax)) {
            error = "causal published command out of bounds at stage " +
                std::to_string(stage);
            return false;
        }
    }
    return true;
}

double DelayAugmentedPhaseAcadosSolver::solveTimeSec() const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready()) return 0.0;
    double solve_time = 0.0;
    ocp_nlp_get(
        impl_->capsule->solver(),
        "time_tot", &solve_time);
    return solve_time;
#else
    return 0.0;
#endif
}

}  // namespace spmpc_local_planner
