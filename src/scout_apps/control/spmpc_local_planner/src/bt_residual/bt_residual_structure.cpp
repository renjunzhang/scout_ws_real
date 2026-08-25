#include "spmpc_local_planner/bt_residual/bt_residual_structure.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace bt_residual {
namespace {

constexpr double kNumericalTolerance = 1.0e-12;

bool finite(double value) {
    return std::isfinite(value);
}

double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

bool finiteVector(const StateVector& vector) {
    return vector.array().isFinite().all();
}

bool finiteResidual(const ResidualVector& vector) {
    return vector.array().isFinite().all();
}

bool maximumVolumeBoxPredecessor(
    const StateMatrix& absolute_jacobian_bound,
    const StateVector& successor_half_width,
    const StateVector& path_half_width,
    StateVector& predecessor_half_width) {
    // Solve the small (15 variable) strictly concave problem
    //
    //   maximize sum(log(y_j))
    //   subject to M diag(path) y <= successor, 0 < y < 1.
    //
    // A deterministic log-barrier Newton solve avoids coupling every state
    // dimension to the single tightest queue coordinate, while retaining the
    // auditable interval predecessor M*rho <= rho_next.
    if (!absolute_jacobian_bound.array().isFinite().all() ||
        !finiteVector(successor_half_width) ||
        !finiteVector(path_half_width) ||
        (absolute_jacobian_bound.array() < 0.0).any() ||
        (successor_half_width.array() <= 0.0).any() ||
        (path_half_width.array() <= 0.0).any()) {
        return false;
    }
    StateMatrix normalized = StateMatrix::Zero();
    for (int row = 0; row < kStateWidth; ++row) {
        for (int column = 0; column < kStateWidth; ++column) {
            normalized(row, column) =
                absolute_jacobian_bound(row, column) *
                path_half_width[column] / successor_half_width[row];
        }
    }
    if (!normalized.array().isFinite().all()) return false;
    const double maximum_row_sum =
        normalized.rowwise().sum().maxCoeff();
    const double initial_scale = maximum_row_sum > 0.0
        ? std::min(0.25, 0.25 / maximum_row_sum)
        : 0.25;
    if (!finite(initial_scale) ||
        initial_scale <= std::numeric_limits<double>::min()) {
        return false;
    }
    StateVector y = StateVector::Constant(initial_scale);

    auto barrierValue = [&](const StateVector& value, double weight) {
        const StateVector slack =
            StateVector::Ones() - normalized * value;
        if ((value.array() <= 0.0).any() ||
            (value.array() >= 1.0).any() ||
            (slack.array() <= 0.0).any()) {
            return std::numeric_limits<double>::infinity();
        }
        return -weight * value.array().log().sum() -
            (1.0 - value.array()).log().sum() -
            slack.array().log().sum();
    };

    double barrier_weight = 1.0;
    for (int outer = 0; outer < 5; ++outer) {
        for (int iteration = 0; iteration < 50; ++iteration) {
            const StateVector slack =
                StateVector::Ones() - normalized * y;
            if ((y.array() <= 0.0).any() ||
                (y.array() >= 1.0).any() ||
                (slack.array() <= 0.0).any()) {
                return false;
            }
            StateVector gradient =
                -barrier_weight * y.cwiseInverse() +
                (StateVector::Ones() - y).cwiseInverse() +
                normalized.transpose() * slack.cwiseInverse();
            StateMatrix hessian =
                (barrier_weight * y.cwiseInverse().array().square() +
                 (StateVector::Ones() - y)
                     .cwiseInverse().array().square())
                    .matrix().asDiagonal();
            hessian.noalias() += normalized.transpose() *
                slack.cwiseInverse().array().square().matrix().asDiagonal() *
                normalized;
            const Eigen::LDLT<StateMatrix> factorization(hessian);
            if (factorization.info() != Eigen::Success) return false;
            const StateVector direction = factorization.solve(-gradient);
            if (factorization.info() != Eigen::Success ||
                !finiteVector(direction)) {
                return false;
            }
            const double decrement_squared = -gradient.dot(direction);
            if (!finite(decrement_squared)) return false;
            if (decrement_squared * 0.5 <= 1.0e-12) break;
            const double current_value = barrierValue(y, barrier_weight);
            double step = 1.0;
            bool accepted = false;
            for (int search = 0; search < 80; ++search) {
                const StateVector candidate = y + step * direction;
                const double candidate_value =
                    barrierValue(candidate, barrier_weight);
                if (finite(candidate_value) &&
                    candidate_value <= current_value +
                        0.01 * step * gradient.dot(direction)) {
                    y = candidate;
                    accepted = true;
                    break;
                }
                step *= 0.5;
            }
            if (!accepted) return false;
        }
        barrier_weight *= 10.0;
    }
    predecessor_half_width =
        (1.0 - 1.0e-9) * path_half_width.cwiseProduct(y);
    if (!finiteVector(predecessor_half_width) ||
        (predecessor_half_width.array() <= 0.0).any()) {
        return false;
    }
    const StateVector image =
        absolute_jacobian_bound * predecessor_half_width;
    if ((image.array() >
         successor_half_width.array() + 1.0e-10).any()) {
        return false;
    }
    return true;
}

bool nearZero(const ResidualVector& residual, double tolerance) {
    return residual.cwiseAbs().maxCoeff() <= tolerance;
}

bool commandInEnvelope(const VelocityCommand& command,
                       const BoundedTrackingRecoveryPolicyParams& params) {
    return finite(command.linear) && finite(command.angular) &&
        command.linear >= params.published_linear_min - kNumericalTolerance &&
        command.linear <= params.published_linear_max + kNumericalTolerance &&
        command.angular >=
            params.published_angular_min - kNumericalTolerance &&
        command.angular <=
            params.published_angular_max + kNumericalTolerance;
}

bool stateInsideCommandEnvelope(
    const AugmentedState15& state,
    const BoundedTrackingRecoveryPolicyParams& params) {
    if (!state.execution.valid || !finite(state.progress_s) ||
        state.progress_s < 0.0 ||
        !commandInEnvelope(
            VelocityCommand{state.execution.robot.v,
                            state.execution.robot.omega},
            params)) {
        return false;
    }
    for (double value : state.execution.linear.pending_commands) {
        if (!finite(value) ||
            value < params.published_linear_min - kNumericalTolerance ||
            value > params.published_linear_max + kNumericalTolerance) {
            return false;
        }
    }
    for (double value : state.execution.angular.pending_commands) {
        if (!finite(value) ||
            value < params.published_angular_min - kNumericalTolerance ||
            value > params.published_angular_max + kNumericalTolerance) {
            return false;
        }
    }
    return true;
}

std::size_t tubeOffset(const RecoverableTube& tube,
                       std::size_t phase_index) {
    if (tube.stages.empty() ||
        phase_index < tube.stages.front().phase_index) {
        return tube.stages.size();
    }
    const std::size_t offset =
        phase_index - tube.stages.front().phase_index;
    return offset < tube.stages.size() &&
            tube.stages[offset].phase_index == phase_index
        ? offset
        : tube.stages.size();
}

}  // namespace

const char* compiledSourceHead() {
#ifdef SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
#else
    return "UNBOUND_BUILD_SOURCE_HEAD";
#endif
}

bool validateStructuralContract(
    const StructuralContract& contract,
    const ExecutionModelContract& execution,
    std::string& error) {
    error.clear();
    const bool identifiers_valid =
        contract.schema == "spmpc_bt_residual_structural_contract_v1" &&
        contract.implementation_id ==
            "bt_centered_residual_terminal_mpc_v1" &&
        contract.claim_level ==
            "linearized_deterministic_development_only" &&
        !contract.expected_artifact_sha256.empty() &&
        !contract.expected_artifact_contract_id.empty() &&
        !contract.expected_execution_contract_hash.empty() &&
        contract.expected_bt_policy_contract_id ==
            "bounded_tracking_recovery_policy_v1";
    if (!identifiers_valid) {
        error = "invalid structural identity or claim level";
        return false;
    }
    if (!finite(execution.dt) || execution.dt <= 0.0 ||
        !finite(execution.linear.delay_sec) ||
        execution.linear.delay_sec < 0.0 ||
        !finite(execution.angular.delay_sec) ||
        execution.angular.delay_sec < 0.0) {
        error = "invalid execution clock/delay in structural contract";
        return false;
    }
    const int linear_queue_count = static_cast<int>(std::floor(
        execution.linear.delay_sec / execution.dt + 1.0e-12)) + 1;
    const int angular_queue_count = static_cast<int>(std::floor(
        execution.angular.delay_sec / execution.dt + 1.0e-12)) + 1;
    const int queue_flush_steps = std::max(
        linear_queue_count, angular_queue_count);
    if (linear_queue_count != 4 || angular_queue_count != 1) {
        error = "bt_residual_v1 requires the frozen 4+1 command queues";
        return false;
    }
    if (contract.residual_prefix_steps <= 1 ||
        contract.recovery_suffix_steps < queue_flush_steps ||
        contract.authority_taper_begin_index >=
            contract.authority_zero_index) {
        error = "invalid residual horizon or authority schedule";
        return false;
    }
    const double positive[] = {
        contract.maximum_published_acceleration,
        contract.maximum_published_angular_acceleration,
        contract.maximum_residual_v,
        contract.maximum_residual_omega,
        contract.maximum_residual_slew_v,
        contract.maximum_residual_slew_omega,
        contract.cumulative_progress_budget_m,
        contract.cumulative_yaw_budget_rad,
        contract.finite_difference_relative_step,
        contract.maximum_finite_difference_reconstruction_error,
        contract.identity_tolerance,
        contract.terminal_liquid_increment_eta,
        contract.terminal_liquid_increment_eta_dot,
        contract.minimum_relative_tracking_improvement,
        contract.minimum_absolute_tracking_improvement,
        contract.minimum_nonzero_residual,
        contract.maximum_absolute_eta,
        contract.maximum_absolute_eta_dot,
    };
    if (std::any_of(std::begin(positive), std::end(positive),
                    [](double value) {
                        return !finite(value) || value <= 0.0;
                    }) ||
        !finite(contract.model_dominance_margin) ||
        contract.model_dominance_margin < 0.0 ||
        !finiteVector(contract.finite_difference_scales) ||
        !finiteVector(contract.candidate_path_deviation_bounds) ||
        !finiteVector(contract.recovery_path_deviation_bounds) ||
        !finiteVector(contract.terminal_deviation_bounds) ||
        !finiteVector(contract.terminal_absolute_bounds) ||
        (contract.finite_difference_scales.array() <= 0.0).any() ||
        (contract.candidate_path_deviation_bounds.array() <= 0.0).any() ||
        (contract.recovery_path_deviation_bounds.array() <= 0.0).any() ||
        (contract.terminal_deviation_bounds.array() <= 0.0).any() ||
        (contract.terminal_absolute_bounds.array() <= 0.0).any()) {
        error = "invalid finite structural limits";
        return false;
    }
    if (execution.contract_hash !=
            contract.expected_execution_contract_hash ||
        execution.dt <= 0.0 || !finite(execution.dt)) {
        error = "execution contract does not match structural freeze";
        return false;
    }
    return true;
}

double residualAuthority(const StructuralContract& contract,
                         std::size_t phase_index) {
    if (phase_index >= contract.authority_zero_index) return 0.0;
    if (phase_index <= contract.authority_taper_begin_index) return 1.0;
    const double numerator = static_cast<double>(
        contract.authority_zero_index - phase_index);
    const double denominator = static_cast<double>(
        contract.authority_zero_index -
        contract.authority_taper_begin_index);
    return std::max(0.0, std::min(1.0, numerator / denominator));
}

bool BtClosedLoopModel::configure(
    const ExecutionModelContract& execution,
    const SloshModelParams& slosh,
    const StructuralContract& structure,
    const std::vector<PhaseNominalSample>* samples,
    std::string& error) {
    configured_ = false;
    samples_ = nullptr;
    if (samples == nullptr || samples->size() < 2) {
        error = "BT artifact samples are unavailable";
        return false;
    }
    if (!validateStructuralContract(structure, execution, error)) {
        return false;
    }
    BoundedTrackingRecoveryPolicyParams params =
        boundedTrackingRecoveryPolicyV1Params();
    if (params.contract_id != structure.expected_bt_policy_contract_id ||
        !bt_policy_.configure(params, error)) {
        error = "BT policy does not match structural freeze: " + error;
        return false;
    }
    if (!execution_model_.configure(execution, slosh, error)) {
        error = "execution model configuration failed: " + error;
        return false;
    }
    const std::size_t linear_count = static_cast<std::size_t>(
        execution_model_.contract().linear.integer_delay_steps + 1);
    const std::size_t angular_count = static_cast<std::size_t>(
        execution_model_.contract().angular.integer_delay_steps + 1);
    for (std::size_t index = 0; index < samples->size(); ++index) {
        const PhaseNominalSample& sample = (*samples)[index];
        if (sample.index != index || !sample.augmented_execution_valid ||
            !sample.augmented_execution.valid ||
            sample.augmented_execution.linear.pending_commands.size() !=
                linear_count ||
            sample.augmented_execution.angular.pending_commands.size() !=
                angular_count) {
            error = "artifact has invalid 15D sample at phase " +
                std::to_string(index);
            return false;
        }
    }
    structure_ = structure;
    samples_ = samples;
    configured_ = true;
    return true;
}

AugmentedState15 BtClosedLoopModel::artifactState(
    std::size_t phase_index) const {
    AugmentedState15 state;
    if (!configured_ || samples_ == nullptr ||
        phase_index >= samples_->size()) {
        return state;
    }
    state.execution = (*samples_)[phase_index].augmented_execution;
    state.progress_s = (*samples_)[phase_index].s;
    return state;
}

ClosedLoopStepResult BtClosedLoopModel::step(
    const AugmentedState15& state,
    std::size_t phase_index,
    const ResidualVector& residual,
    const StagePublicationConstraint& publication) const {
    ClosedLoopStepResult result;
    result.state = state;
    result.applied_residual = residual;
    result.linear_cap_active = publication.linear_cap_active;
    if (!configured_ || samples_ == nullptr ||
        phase_index + 1 >= samples_->size() ||
        !finiteResidual(residual) ||
        !execution_model_.validState(state.execution) ||
        !stateInsideCommandEnvelope(state, bt_policy_.params()) ||
        state.execution.linear.pending_commands.empty() ||
        state.execution.angular.pending_commands.empty()) {
        result.status = "INVALID_BT_CLOSED_LOOP_STEP_INPUT";
        return result;
    }
    if (publication.linear_cap_active &&
        (!finite(publication.maximum_linear) ||
         publication.maximum_linear < 0.0)) {
        result.status = "INVALID_PRE_PUBLICATION_LINEAR_CAP";
        return result;
    }

    result.authority = residualAuthority(structure_, phase_index);
    const double maximum_v =
        result.authority * structure_.maximum_residual_v;
    const double maximum_omega =
        result.authority * structure_.maximum_residual_omega;
    if (std::abs(residual[0]) > maximum_v + kNumericalTolerance ||
        std::abs(residual[1]) > maximum_omega + kNumericalTolerance) {
        result.status = "RESIDUAL_POINTWISE_AUTHORITY_VIOLATION";
        return result;
    }
    // The external D2 cap is downstream of the ordinary rate limiter.  Its
    // active set is frozen by disabling residual authority, so candidate and
    // BT use exactly the same unmodelled discontinuity.
    if (publication.linear_cap_active &&
        !nearZero(residual, structure_.identity_tolerance)) {
        result.status = "RESIDUAL_FORBIDDEN_WHILE_LINEAR_CAP_ACTIVE";
        return result;
    }

    const PhaseNominalSample& nominal = (*samples_)[phase_index];
    const BoundedTrackingRecoveryPolicyResult desired =
        bt_policy_.evaluate(nominal, state.execution.robot);
    if (!desired.valid) {
        result.status = "BT_POLICY_FAILED_" + desired.status;
        return result;
    }
    VelocityCommand previous;
    previous.linear = state.execution.linear.pending_commands.back();
    previous.angular = state.execution.angular.pending_commands.back();
    const BoundedTrackingRecoveryCommandTransaction transaction =
        applyBoundedTrackingRecoveryCommandTransaction(
            desired.command, previous,
            structure_.maximum_published_acceleration,
            structure_.maximum_published_angular_acceleration,
            execution_model_.contract().dt, bt_policy_.params());
    if (!transaction.valid) {
        result.status = "BT_TRANSACTION_FAILED_" + transaction.status;
        return result;
    }
    result.bt_command = transaction.command;
    result.bt_rate_limited = transaction.rate_limited;
    if (publication.linear_cap_active) {
        const double capped = std::min(
            result.bt_command.linear, publication.maximum_linear);
        result.linear_cap_modified = capped != result.bt_command.linear;
        result.bt_command.linear = capped;
    }
    result.published_command.linear = result.bt_command.linear + residual[0];
    result.published_command.angular =
        result.bt_command.angular + residual[1];
    if (!commandInEnvelope(result.published_command, bt_policy_.params()) ||
        (publication.linear_cap_active &&
         result.published_command.linear >
             publication.maximum_linear + kNumericalTolerance)) {
        result.status = "RESIDUAL_PUBLISHED_COMMAND_BOUNDS_VIOLATION";
        return result;
    }
    if (!publication.linear_cap_active) {
        const double maximum_delta_v =
            structure_.maximum_published_acceleration *
            execution_model_.contract().dt;
        const double maximum_delta_omega =
            structure_.maximum_published_angular_acceleration *
            execution_model_.contract().dt;
        if (std::abs(result.published_command.linear - previous.linear) >
                maximum_delta_v + kNumericalTolerance ||
            std::abs(result.published_command.angular - previous.angular) >
                maximum_delta_omega + kNumericalTolerance) {
            result.status = "RESIDUAL_PUBLISHED_RATE_VIOLATION";
            return result;
        }
    }

    const ExecutionStepResult execution = execution_model_.step(
        state.execution, result.published_command);
    if (!execution.valid) {
        result.status = "EXECUTION_STEP_FAILED_" + execution.status;
        return result;
    }
    result.state.execution = execution.state;
    result.integrated_progress_m = 0.0;
    for (const ExecutionPropagationSegment& segment : execution.segments) {
        result.integrated_progress_m +=
            std::max(0.0, segment.output_v) * segment.duration_sec;
    }
    result.state.progress_s =
        state.progress_s + result.integrated_progress_m;
    if (!finite(result.state.progress_s)) {
        result.status = "NONFINITE_PHYSICAL_PROGRESS";
        return result;
    }
    result.valid = true;
    result.status = "OK";
    return result;
}

ClosedLoopRolloutResult BtClosedLoopModel::rollout(
    const AugmentedState15& initial_state,
    std::size_t initial_phase_index,
    const std::vector<ResidualVector>& residuals,
    const std::vector<StagePublicationConstraint>& publications) const {
    ClosedLoopRolloutResult result;
    result.initial_phase_index = initial_phase_index;
    if (!configured_ || samples_ == nullptr || residuals.empty() ||
        initial_phase_index + residuals.size() >= samples_->size() ||
        (!publications.empty() && publications.size() != residuals.size())) {
        result.status = "INVALID_BT_CLOSED_LOOP_ROLLOUT_INPUT";
        return result;
    }
    result.states.reserve(residuals.size() + 1);
    result.bt_commands.reserve(residuals.size());
    result.published_commands.reserve(residuals.size());
    result.residuals.reserve(residuals.size());
    result.states.push_back(initial_state);
    AugmentedState15 state = initial_state;
    ResidualVector previous = ResidualVector::Zero();
    double cumulative_progress = 0.0;
    double cumulative_yaw = 0.0;
    const double dt = execution_model_.contract().dt;
    for (std::size_t offset = 0; offset < residuals.size(); ++offset) {
        const ResidualVector& residual = residuals[offset];
        if (offset >= static_cast<std::size_t>(
                          structure_.residual_prefix_steps) &&
            !nearZero(residual, structure_.identity_tolerance)) {
            result.status = "BT_RECOVERY_SUFFIX_CONTAINS_RESIDUAL";
            return result;
        }
        const ResidualVector delta = residual - previous;
        if (std::abs(delta[0]) >
                structure_.maximum_residual_slew_v +
                    kNumericalTolerance ||
            std::abs(delta[1]) >
                structure_.maximum_residual_slew_omega +
                    kNumericalTolerance) {
            result.status = "RESIDUAL_SLEW_VIOLATION";
            return result;
        }
        cumulative_progress += residual[0] * dt;
        cumulative_yaw += residual[1] * dt;
        if (std::abs(cumulative_progress) >
                structure_.cumulative_progress_budget_m +
                    kNumericalTolerance ||
            std::abs(cumulative_yaw) >
                structure_.cumulative_yaw_budget_rad +
                    kNumericalTolerance) {
            result.status = "RESIDUAL_CUMULATIVE_BUDGET_VIOLATION";
            return result;
        }
        const StagePublicationConstraint publication = publications.empty()
            ? StagePublicationConstraint{}
            : publications[offset];
        const ClosedLoopStepResult stage = step(
            state, initial_phase_index + offset, residual, publication);
        if (!stage.valid) {
            result.status = stage.status;
            return result;
        }
        result.bt_commands.push_back(stage.bt_command);
        result.published_commands.push_back(stage.published_command);
        result.residuals.push_back(residual);
        result.states.push_back(stage.state);
        state = stage.state;
        previous = residual;
    }
    // A suffix is structurally present only if it begins with a reachable
    // zero residual.  This catches a candidate that ends its prefix with a
    // discontinuous residual and silently calls the next stage BT-only.
    if (residuals.size() > static_cast<std::size_t>(
                               structure_.residual_prefix_steps)) {
        const ResidualVector zero_jump = -residuals[
            static_cast<std::size_t>(structure_.residual_prefix_steps) - 1];
        if (std::abs(zero_jump[0]) >
                structure_.maximum_residual_slew_v +
                    kNumericalTolerance ||
            std::abs(zero_jump[1]) >
                structure_.maximum_residual_slew_omega +
                    kNumericalTolerance) {
            result.status = "RESIDUAL_TO_BT_SUFFIX_SLEW_VIOLATION";
            return result;
        }
    }
    result.valid = true;
    result.status = "OK";
    return result;
}

StateVector BtClosedLoopModel::pack(const AugmentedState15& state) const {
    StateVector packed = StateVector::Constant(
        std::numeric_limits<double>::quiet_NaN());
    if (!configured_ ||
        state.execution.linear.pending_commands.size() != 4u ||
        state.execution.angular.pending_commands.size() != 1u) {
        return packed;
    }
    packed << state.execution.robot.x,
        state.execution.robot.y,
        state.execution.robot.yaw,
        state.execution.robot.v,
        state.progress_s,
        state.execution.robot.omega,
        state.execution.slosh.eta_x,
        state.execution.slosh.eta_x_dot,
        state.execution.slosh.eta_y,
        state.execution.slosh.eta_y_dot,
        state.execution.linear.pending_commands[0],
        state.execution.linear.pending_commands[1],
        state.execution.linear.pending_commands[2],
        state.execution.linear.pending_commands[3],
        state.execution.angular.pending_commands[0];
    return packed;
}

bool BtClosedLoopModel::unpack(
    const StateVector& packed,
    std::uint64_t stage_index,
    AugmentedState15& state,
    std::string& error) const {
    state = AugmentedState15{};
    error.clear();
    if (!configured_ || !finiteVector(packed) || packed[4] < 0.0) {
        error = "invalid packed 15D state";
        return false;
    }
    state.execution.robot.x = packed[0];
    state.execution.robot.y = packed[1];
    state.execution.robot.yaw = wrapAngle(packed[2]);
    state.execution.robot.v = packed[3];
    state.progress_s = packed[4];
    state.execution.robot.omega = packed[5];
    state.execution.slosh.eta_x = packed[6];
    state.execution.slosh.eta_x_dot = packed[7];
    state.execution.slosh.eta_y = packed[8];
    state.execution.slosh.eta_y_dot = packed[9];
    state.execution.linear.pending_commands.assign(
        packed.data() + 10, packed.data() + 14);
    state.execution.angular.pending_commands.assign(
        packed.data() + 14, packed.data() + 15);
    state.execution.linear.actuator_output = packed[3];
    state.execution.angular.actuator_output = packed[5];
    state.execution.stage_index = stage_index;
    state.execution.valid = true;
    if (!execution_model_.validState(state.execution) ||
        !stateInsideCommandEnvelope(state, bt_policy_.params())) {
        state.execution.valid = false;
        error = "packed state violates execution/command envelope";
        return false;
    }
    return true;
}

StateVector BtClosedLoopModel::difference(
    const AugmentedState15& lhs,
    const AugmentedState15& rhs) const {
    StateVector difference = pack(lhs) - pack(rhs);
    difference[2] = wrapAngle(
        lhs.execution.robot.yaw - rhs.execution.robot.yaw);
    return difference;
}

ClosedLoopLinearization linearizeClosedLoop(
    const BtClosedLoopModel& model,
    const AugmentedState15& center,
    std::size_t phase_index,
    const StagePublicationConstraint& publication) {
    ClosedLoopLinearization output;
    output.phase_index = phase_index;
    output.center = center;
    if (!model.configured()) {
        output.status = "BT_MODEL_NOT_CONFIGURED";
        return output;
    }
    output.nominal_step = model.step(
        center, phase_index, ResidualVector::Zero(), publication);
    if (!output.nominal_step.valid) {
        output.status = "NOMINAL_LINEARIZATION_STEP_FAILED_" +
            output.nominal_step.status;
        return output;
    }
    const StructuralContract& contract = model.structure();
    const StateVector packed_center = model.pack(center);
    for (int column = 0; column < kStateWidth; ++column) {
        const double step_size =
            contract.finite_difference_relative_step *
            contract.finite_difference_scales[column];
        StateVector plus_vector = packed_center;
        StateVector minus_vector = packed_center;
        plus_vector[column] += step_size;
        minus_vector[column] -= step_size;
        AugmentedState15 plus_state;
        AugmentedState15 minus_state;
        std::string ignored;
        const bool plus_unpacked = model.unpack(
            plus_vector, center.execution.stage_index,
            plus_state, ignored);
        const bool minus_unpacked = model.unpack(
            minus_vector, center.execution.stage_index,
            minus_state, ignored);
        ClosedLoopStepResult plus;
        ClosedLoopStepResult minus;
        if (plus_unpacked) {
            plus = model.step(
                plus_state, phase_index, ResidualVector::Zero(),
                publication);
        }
        if (minus_unpacked) {
            minus = model.step(
                minus_state, phase_index, ResidualVector::Zero(),
                publication);
        }
        StateVector forward_derivative = StateVector::Zero();
        StateVector backward_derivative = StateVector::Zero();
        if (plus.valid) {
            forward_derivative = model.difference(
                plus.state, output.nominal_step.state) / step_size;
        }
        if (minus.valid) {
            backward_derivative = model.difference(
                output.nominal_step.state, minus.state) / step_size;
        }
        if (plus.valid && minus.valid) {
            output.a.col(column) =
                model.difference(plus.state, minus.state) /
                (2.0 * step_size);
            output.bt_command_state_jacobian(0, column) =
                (plus.published_command.linear -
                 minus.published_command.linear) /
                (2.0 * step_size);
            output.bt_command_state_jacobian(1, column) =
                (plus.published_command.angular -
                 minus.published_command.angular) /
                (2.0 * step_size);
            output.a_schemes[column] = DifferenceScheme::Central;
        } else if (plus.valid) {
            output.a.col(column) = forward_derivative;
            output.bt_command_state_jacobian(0, column) =
                (plus.published_command.linear -
                 output.nominal_step.published_command.linear) / step_size;
            output.bt_command_state_jacobian(1, column) =
                (plus.published_command.angular -
                 output.nominal_step.published_command.angular) / step_size;
            output.a_schemes[column] = DifferenceScheme::Forward;
        } else if (minus.valid) {
            output.a.col(column) = backward_derivative;
            output.bt_command_state_jacobian(0, column) =
                (output.nominal_step.published_command.linear -
                 minus.published_command.linear) / step_size;
            output.bt_command_state_jacobian(1, column) =
                (output.nominal_step.published_command.angular -
                 minus.published_command.angular) / step_size;
            output.a_schemes[column] = DifferenceScheme::Backward;
        } else {
            output.status = "STATE_FINITE_DIFFERENCE_FAILED_DIM_" +
                std::to_string(column);
            return output;
        }
        if (plus.valid && minus.valid) {
            output.a_absolute_bound.col(column) =
                forward_derivative.cwiseAbs().cwiseMax(
                    backward_derivative.cwiseAbs());
            output.maximum_directional_asymmetry = std::max(
                output.maximum_directional_asymmetry,
                ((forward_derivative - backward_derivative) * step_size)
                    .cwiseAbs().maxCoeff());
        } else {
            output.a_absolute_bound.col(column) =
                output.a.col(column).cwiseAbs();
        }

        // Audit each valid active-set branch at a held-out half step.  This
        // separates numerical reconstruction error from the expected
        // forward/backward asymmetry at rate/saturation kinks.
        const double half_step = 0.5 * step_size;
        StateVector half_vector = packed_center;
        half_vector[column] += half_step;
        AugmentedState15 half_state;
        if (plus.valid && model.unpack(
                half_vector, center.execution.stage_index,
                half_state, ignored)) {
            const ClosedLoopStepResult half = model.step(
                half_state, phase_index, ResidualVector::Zero(),
                publication);
            if (!half.valid) {
                output.status = "FORWARD_RECONSTRUCTION_STEP_FAILED_DIM_" +
                    std::to_string(column);
                return output;
            }
            const StateVector actual = model.difference(
                half.state, output.nominal_step.state);
            const StateVector predicted = forward_derivative * half_step;
            output.maximum_reconstruction_error = std::max(
                output.maximum_reconstruction_error,
                (actual - predicted).cwiseAbs().maxCoeff());
        }
        half_vector = packed_center;
        half_vector[column] -= half_step;
        if (minus.valid && model.unpack(
                half_vector, center.execution.stage_index,
                half_state, ignored)) {
            const ClosedLoopStepResult half = model.step(
                half_state, phase_index, ResidualVector::Zero(),
                publication);
            if (!half.valid) {
                output.status = "BACKWARD_RECONSTRUCTION_STEP_FAILED_DIM_" +
                    std::to_string(column);
                return output;
            }
            const StateVector actual = model.difference(
                half.state, output.nominal_step.state);
            const StateVector predicted = -backward_derivative * half_step;
            output.maximum_reconstruction_error = std::max(
                output.maximum_reconstruction_error,
                (actual - predicted).cwiseAbs().maxCoeff());
        }
    }

    const double authority = residualAuthority(contract, phase_index);
    const double residual_scales[kResidualWidth] = {
        contract.maximum_residual_v,
        contract.maximum_residual_omega,
    };
    for (int column = 0; column < kResidualWidth; ++column) {
        if (authority <= contract.identity_tolerance ||
            publication.linear_cap_active) {
            output.b.col(column).setZero();
            output.b_schemes[column] = DifferenceScheme::AuthorityZero;
            continue;
        }
        const double step_size =
            contract.finite_difference_relative_step *
            residual_scales[column];
        ResidualVector plus_residual = ResidualVector::Zero();
        ResidualVector minus_residual = ResidualVector::Zero();
        plus_residual[column] = step_size;
        minus_residual[column] = -step_size;
        const ClosedLoopStepResult plus = model.step(
            center, phase_index, plus_residual, publication);
        const ClosedLoopStepResult minus = model.step(
            center, phase_index, minus_residual, publication);
        StateVector forward_derivative = StateVector::Zero();
        StateVector backward_derivative = StateVector::Zero();
        if (plus.valid) {
            forward_derivative = model.difference(
                plus.state, output.nominal_step.state) / step_size;
        }
        if (minus.valid) {
            backward_derivative = model.difference(
                output.nominal_step.state, minus.state) / step_size;
        }
        if (plus.valid && minus.valid) {
            output.b.col(column) =
                model.difference(plus.state, minus.state) /
                (2.0 * step_size);
            output.b_schemes[column] = DifferenceScheme::Central;
        } else if (plus.valid) {
            output.b.col(column) = forward_derivative;
            output.b_schemes[column] = DifferenceScheme::Forward;
        } else if (minus.valid) {
            output.b.col(column) = backward_derivative;
            output.b_schemes[column] = DifferenceScheme::Backward;
        } else {
            output.status = "RESIDUAL_FINITE_DIFFERENCE_FAILED_DIM_" +
                std::to_string(column);
            return output;
        }
        if (plus.valid && minus.valid) {
            output.b_absolute_bound.col(column) =
                forward_derivative.cwiseAbs().cwiseMax(
                    backward_derivative.cwiseAbs());
            output.maximum_directional_asymmetry = std::max(
                output.maximum_directional_asymmetry,
                ((forward_derivative - backward_derivative) * step_size)
                    .cwiseAbs().maxCoeff());
        } else {
            output.b_absolute_bound.col(column) =
                output.b.col(column).cwiseAbs();
        }
        ResidualVector half_residual = ResidualVector::Zero();
        half_residual[column] = 0.5 * step_size;
        if (plus.valid) {
            const ClosedLoopStepResult half = model.step(
                center, phase_index, half_residual, publication);
            if (!half.valid) {
                output.status =
                    "FORWARD_RESIDUAL_RECONSTRUCTION_FAILED_DIM_" +
                    std::to_string(column);
                return output;
            }
            const StateVector actual = model.difference(
                half.state, output.nominal_step.state);
            const StateVector predicted =
                forward_derivative * (0.5 * step_size);
            output.maximum_reconstruction_error = std::max(
                output.maximum_reconstruction_error,
                (actual - predicted).cwiseAbs().maxCoeff());
        }
        half_residual[column] = -0.5 * step_size;
        if (minus.valid) {
            const ClosedLoopStepResult half = model.step(
                center, phase_index, half_residual, publication);
            if (!half.valid) {
                output.status =
                    "BACKWARD_RESIDUAL_RECONSTRUCTION_FAILED_DIM_" +
                    std::to_string(column);
                return output;
            }
            const StateVector actual = model.difference(
                half.state, output.nominal_step.state);
            const StateVector predicted =
                -backward_derivative * (0.5 * step_size);
            output.maximum_reconstruction_error = std::max(
                output.maximum_reconstruction_error,
                (actual - predicted).cwiseAbs().maxCoeff());
        }
    }
    if (!output.a.array().isFinite().all() ||
        !output.a_absolute_bound.array().isFinite().all() ||
        !output.b.array().isFinite().all() ||
        !output.b_absolute_bound.array().isFinite().all() ||
        !output.bt_command_state_jacobian.array().isFinite().all()) {
        output.status = "NONFINITE_CLOSED_LOOP_LINEARIZATION";
        return output;
    }
    if (output.maximum_reconstruction_error >
        contract.maximum_finite_difference_reconstruction_error) {
        output.status = "FINITE_DIFFERENCE_RECONSTRUCTION_ERROR_EXCEEDED";
        return output;
    }
    output.valid = true;
    output.status = "OK";
    return output;
}

RecoverableTube buildLinearizedRecoverableTube(
    const BtClosedLoopModel& model,
    const AugmentedState15& initial_center,
    std::size_t initial_phase_index,
    std::size_t terminal_phase_index) {
    RecoverableTube tube;
    tube.claim_level = model.structure().claim_level;
    if (!model.configured() || terminal_phase_index <= initial_phase_index) {
        tube.status = "INVALID_TUBE_BUILD_INPUT";
        return tube;
    }
    const std::size_t stage_count =
        terminal_phase_index - initial_phase_index + 1;
    tube.stages.resize(stage_count);
    tube.linearizations.reserve(stage_count - 1);
    AugmentedState15 center = initial_center;
    for (std::size_t offset = 0; offset + 1 < stage_count; ++offset) {
        const std::size_t phase = initial_phase_index + offset;
        tube.stages[offset].phase_index = phase;
        tube.stages[offset].center = center;
        ClosedLoopLinearization linearization = linearizeClosedLoop(
            model, center, phase);
        if (!linearization.valid) {
            tube.status = "TUBE_LINEARIZATION_FAILED_PHASE_" +
                std::to_string(phase) + "_" + linearization.status;
            return tube;
        }
        center = linearization.nominal_step.state;
        tube.linearizations.push_back(std::move(linearization));
    }
    tube.stages.back().phase_index = terminal_phase_index;
    tube.stages.back().center = center;
    tube.stages.back().terminal_map = StateMatrix::Identity();
    tube.stages.back().half_width =
        model.structure().terminal_deviation_bounds.cwiseMin(
            model.structure().recovery_path_deviation_bounds);
    tube.stages.back().valid = true;
    for (std::size_t reverse = stage_count - 1; reverse > 0; --reverse) {
        const std::size_t offset = reverse - 1;
        tube.stages[offset].terminal_map =
            tube.stages[offset + 1].terminal_map *
            tube.linearizations[offset].a;
        if (!tube.stages[offset].terminal_map.array().isFinite().all()) {
            tube.status = "NONFINITE_BACKWARD_TERMINAL_MAP_PHASE_" +
                std::to_string(tube.stages[offset].phase_index);
            return tube;
        }
        // Phase-specific maximum-volume interval predecessor.  Unlike a
        // common scalar shrink, a tight queue coordinate cannot collapse
        // unrelated pose/liquid dimensions.
        const StateVector path =
            model.structure().recovery_path_deviation_bounds;
        if (!maximumVolumeBoxPredecessor(
                tube.linearizations[offset].a_absolute_bound,
                tube.stages[offset + 1].half_width, path,
                tube.stages[offset].half_width)) {
            tube.status = "MAXIMUM_VOLUME_BOX_PREDECESSOR_FAILED_PHASE_" +
                std::to_string(tube.stages[offset].phase_index);
            return tube;
        }
        const StateVector predecessor_image =
            tube.linearizations[offset].a_absolute_bound *
            tube.stages[offset].half_width;
        if (!finiteVector(tube.stages[offset].half_width) ||
            (predecessor_image.array() >
             tube.stages[offset + 1].half_width.array() + 1.0e-10).any()) {
            tube.status = "BACKWARD_BOX_PREDECESSOR_FAILED_PHASE_" +
                std::to_string(tube.stages[offset].phase_index);
            return tube;
        }
        tube.stages[offset].valid = true;
    }
    tube.valid = true;
    tube.status = "OK";
    return tube;
}

TubeMembershipResult evaluateTubeMembership(
    const BtClosedLoopModel& model,
    const RecoverableTube& tube,
    const AugmentedState15& state,
    std::size_t phase_index) {
    TubeMembershipResult result;
    if (!model.configured() || !tube.valid) {
        result.status = "INVALID_TUBE_MEMBERSHIP_INPUT";
        return result;
    }
    const std::size_t offset = tubeOffset(tube, phase_index);
    if (offset >= tube.stages.size() || !tube.stages[offset].valid) {
        result.status = "TUBE_PHASE_UNAVAILABLE";
        return result;
    }
    const StateVector deviation = model.difference(
        state, tube.stages[offset].center);
    result.predicted_terminal_deviation =
        tube.stages[offset].terminal_map * deviation;
    if (!finiteVector(result.predicted_terminal_deviation)) {
        result.status = "NONFINITE_PREDICTED_TERMINAL_DEVIATION";
        return result;
    }
    result.minimum_margin = std::numeric_limits<double>::infinity();
    result.inside = true;
    for (int index = 0; index < kStateWidth; ++index) {
        const double terminal_margin =
            model.structure().terminal_deviation_bounds[index] -
            std::abs(result.predicted_terminal_deviation[index]);
        const double local_margin =
            tube.stages[offset].half_width[index] -
            std::abs(deviation[index]);
        result.minimum_margin = std::min(
            result.minimum_margin,
            std::min(terminal_margin, local_margin));
        result.inside = result.inside &&
            terminal_margin >= -kNumericalTolerance &&
            local_margin >= -kNumericalTolerance;
    }
    result.valid = true;
    result.status = result.inside ? "INSIDE" : "OUTSIDE";
    return result;
}

TerminalRecoveryResult auditNonlinearBtRecovery(
    const BtClosedLoopModel& model,
    const AugmentedState15& state,
    std::size_t phase_index,
    const RecoverableTube& tube) {
    TerminalRecoveryResult result;
    const std::size_t offset = tubeOffset(tube, phase_index);
    if (!model.configured() || !tube.valid ||
        offset >= tube.stages.size()) {
        result.status = "INVALID_NONLINEAR_RECOVERY_INPUT";
        return result;
    }
    const TubeMembershipResult membership = evaluateTubeMembership(
        model, tube, state, phase_index);
    if (!membership.valid) {
        result.status = "TUBE_MEMBERSHIP_FAILED_" + membership.status;
        return result;
    }
    const std::size_t steps = tube.stages.size() - offset - 1;
    std::vector<AugmentedState15> recovery_states;
    if (steps == 0) {
        result.terminal_state = state;
        recovery_states.push_back(state);
        result.nonlinear_rollout_completed = true;
    } else {
        const std::vector<ResidualVector> zeros(
            steps, ResidualVector::Zero());
        const ClosedLoopRolloutResult rollout = model.rollout(
            state, phase_index, zeros);
        if (!rollout.valid) {
            result.valid = true;
            result.status = "NONLINEAR_BT_RECOVERY_ROLLOUT_FAILED_" +
                rollout.status;
            return result;
        }
        result.terminal_state = rollout.states.back();
        recovery_states = rollout.states;
        result.nonlinear_rollout_completed = true;
    }
    bool path_tube_passed = true;
    bool nonlinear_path_passed = true;
    for (std::size_t stage_offset = 0;
         stage_offset < recovery_states.size(); ++stage_offset) {
        const AugmentedState15& stage = recovery_states[stage_offset];
        result.maximum_eta = std::max(
            result.maximum_eta,
            std::max(std::abs(stage.execution.slosh.eta_x),
                     std::abs(stage.execution.slosh.eta_y)));
        result.maximum_eta_dot = std::max(
            result.maximum_eta_dot,
            std::max(std::abs(stage.execution.slosh.eta_x_dot),
                     std::abs(stage.execution.slosh.eta_y_dot)));
        const TubeMembershipResult stage_membership = evaluateTubeMembership(
            model, tube, stage, phase_index + stage_offset);
        path_tube_passed = path_tube_passed &&
            stage_membership.valid && stage_membership.inside;
        const std::size_t stage_tube_offset = offset + stage_offset;
        if (stage_tube_offset >= tube.stages.size()) {
            nonlinear_path_passed = false;
        } else {
            const StateVector deviation = model.difference(
                stage, tube.stages[stage_tube_offset].center);
            nonlinear_path_passed = nonlinear_path_passed &&
                (deviation.cwiseAbs().array() <=
                 model.structure().recovery_path_deviation_bounds.array() +
                     kNumericalTolerance).all();
        }
    }
    result.tube_path_passed = path_tube_passed;
    result.nonlinear_path_passed = nonlinear_path_passed;
    const AugmentedState15& terminal_center = tube.stages.back().center;
    result.terminal_error = model.difference(
        result.terminal_state, terminal_center);
    const StateVector terminal = model.pack(result.terminal_state);
    bool terminal_passed = true;
    for (int index = 0; index < kStateWidth; ++index) {
        const bool deviation_dimension =
            index == 0 || index == 1 || index == 2 || index == 4;
        const double tested = deviation_dimension
            ? std::abs(result.terminal_error[index])
            : std::abs(terminal[index]);
        terminal_passed = terminal_passed &&
            tested <= model.structure().terminal_absolute_bounds[index] +
                kNumericalTolerance;
    }
    result.terminal_contract_passed = terminal_passed;
    result.liquid_path_passed =
        result.maximum_eta <=
            model.structure().maximum_absolute_eta + kNumericalTolerance &&
        result.maximum_eta_dot <=
            model.structure().maximum_absolute_eta_dot +
                kNumericalTolerance;
    result.valid = true;
    result.nonlinear_recovered = result.nonlinear_rollout_completed &&
        nonlinear_path_passed && terminal_passed &&
        result.liquid_path_passed;
    result.recovered = membership.inside && path_tube_passed &&
        result.nonlinear_recovered;
    result.status = result.recovered
        ? "RECOVERED"
        : (!result.nonlinear_recovered
               ? (!nonlinear_path_passed
                      ? "NONLINEAR_RECOVERY_PATH_CONTRACT_FAILED"
                      : (!terminal_passed
                             ? "NONLINEAR_TERMINAL_CONTRACT_FAILED"
                             : "NONLINEAR_LIQUID_PATH_CONTRACT_FAILED"))
               : "NONLINEAR_RECOVERED_TUBE_COVERAGE_FAILED");
    return result;
}

}  // namespace bt_residual
}  // namespace spmpc_local_planner
