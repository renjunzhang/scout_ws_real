#include "spmpc_local_planner/bt_residual/bt_residual_structure.h"

#include "spmpc_local_planner/controller/command/publication_transaction.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace bt_residual {
namespace {

class DeterministicOracleSink final : public ICommandSink {
public:
    StampNs publicationTimeNs() override { return stamp_ns_; }

    PublicationReceipt publish(const FinalCommand& command) override {
        PublicationReceipt receipt;
        receipt.cycle_id = command.cycle_id;
        receipt.attempted = true;
        receipt.delivered = command.publish_enabled;
        receipt.actual_publish_stamp_ns = stamp_ns_;
        receipt.command = command.command;
        receipt.status = receipt.delivered
            ? "ORACLE_DELIVERED"
            : "ORACLE_PUBLISH_DISABLED";
        return receipt;
    }

private:
    // Keeping one valid stamp makes CommandPipeline use its exact nominal
    // control period on every stage, without nanosecond quantization drift.
    StampNs stamp_ns_ = secondsToNanoseconds(1.0);
};

bool sameCommandWithin(const VelocityCommand& lhs,
                       const VelocityCommand& rhs,
                       double tolerance) {
    return std::abs(lhs.linear - rhs.linear) <= tolerance &&
        std::abs(lhs.angular - rhs.angular) <= tolerance;
}

}  // namespace

IndependentBtOracleRolloutResult rolloutIndependentBtOracle(
    const ExecutionModelContract& execution,
    const SloshModelParams& slosh,
    const StructuralContract& structure,
    const std::vector<PhaseNominalSample>& samples,
    const AugmentedState15& initial_state,
    std::size_t initial_phase_index,
    std::size_t step_count,
    const std::vector<StagePublicationConstraint>& publications) {
    IndependentBtOracleRolloutResult result;
    result.initial_phase_index = initial_phase_index;
    if (step_count == 0 || initial_phase_index + step_count >= samples.size() ||
        (!publications.empty() && publications.size() != step_count) ||
        initial_state.execution.linear.pending_commands.empty() ||
        initial_state.execution.angular.pending_commands.empty()) {
        result.status = "INVALID_INDEPENDENT_BT_ORACLE_INPUT";
        return result;
    }
    std::string error;
    if (!validateStructuralContract(structure, execution, error)) {
        result.status = "ORACLE_STRUCTURE_REJECTED_" + error;
        return result;
    }
    BoundedTrackingRecoveryPolicy policy;
    const BoundedTrackingRecoveryPolicyParams policy_params =
        boundedTrackingRecoveryPolicyV1Params();
    if (policy_params.contract_id != structure.expected_bt_policy_contract_id ||
        !policy.configure(policy_params, error)) {
        result.status = "ORACLE_BT_POLICY_REJECTED_" + error;
        return result;
    }
    ExecutionModel execution_model;
    if (!execution_model.configure(execution, slosh, error)) {
        result.status = "ORACLE_EXECUTION_MODEL_REJECTED_" + error;
        return result;
    }

    CommandPipelineConfig pipeline_config;
    pipeline_config.control_frequency = 1.0 / execution.dt;
    pipeline_config.linear_accel_limit_enable = true;
    pipeline_config.linear_accel_max =
        structure.maximum_published_acceleration;
    pipeline_config.linear_accel_max_dt = std::max(0.2, execution.dt);
    pipeline_config.angular_limit_enable = true;
    pipeline_config.angular_rate_max = std::max(
        std::abs(policy_params.published_angular_min),
        std::abs(policy_params.published_angular_max));
    pipeline_config.angular_accel_max =
        structure.maximum_published_angular_acceleration;
    pipeline_config.angular_accel_max_dt = std::max(0.2, execution.dt);
    pipeline_config.fail_closed_on_post_limit_change = false;
    if (!pipeline_config.control_frequency ||
        !std::isfinite(pipeline_config.control_frequency)) {
        result.status = "ORACLE_INVALID_CONTROL_FREQUENCY";
        return result;
    }
    CommandPipeline pipeline;
    if (!pipeline.configure(pipeline_config, error)) {
        result.status = "ORACLE_COMMAND_PIPELINE_REJECTED_" + error;
        return result;
    }
    DeterministicOracleSink sink;
    VelocityCommand previous;
    previous.linear =
        initial_state.execution.linear.pending_commands.back();
    previous.angular =
        initial_state.execution.angular.pending_commands.back();
    if (!pipeline.commitPublished(previous, sink.publicationTimeNs())) {
        result.status = "ORACLE_PIPELINE_SEED_FAILED";
        return result;
    }
    PublicationTransaction publication_transaction(pipeline);

    result.states.reserve(step_count + 1);
    result.published_commands.reserve(step_count);
    result.states.push_back(initial_state);
    AugmentedState15 state = initial_state;
    for (std::size_t offset = 0; offset < step_count; ++offset) {
        const std::size_t phase = initial_phase_index + offset;
        const BoundedTrackingRecoveryPolicyResult desired =
            policy.evaluate(samples[phase], state.execution.robot);
        if (!desired.valid) {
            result.status = "ORACLE_BT_POLICY_FAILED_" + desired.status;
            return result;
        }
        VelocityCommand stage_previous;
        stage_previous.linear =
            state.execution.linear.pending_commands.back();
        stage_previous.angular =
            state.execution.angular.pending_commands.back();
        if (!sameCommandWithin(stage_previous, pipeline.lastPublishedCommand(),
                               structure.identity_tolerance)) {
            result.status = "ORACLE_QUEUE_PIPELINE_HISTORY_MISMATCH";
            return result;
        }
        const BoundedTrackingRecoveryCommandTransaction bt_transaction =
            applyBoundedTrackingRecoveryCommandTransaction(
                desired.command, stage_previous,
                structure.maximum_published_acceleration,
                structure.maximum_published_angular_acceleration,
                execution.dt, policy_params);
        if (!bt_transaction.valid) {
            result.status = "ORACLE_BT_TRANSACTION_FAILED_" +
                bt_transaction.status;
            return result;
        }

        CommandPublicationRequest request;
        request.cycle_id = static_cast<std::uint64_t>(offset + 1);
        request.proposed.command = bt_transaction.command;
        request.proposed.source = CommandSource::PhaseRejoin;
        request.proposed.reason = "INDEPENDENT_BT_ORACLE";
        request.proposed.accepted = true;
        request.sink = &sink;
        if (!publications.empty() &&
            publications[offset].linear_cap_active) {
            request.linear_cap.active = true;
            request.linear_cap.max_linear =
                publications[offset].maximum_linear;
            request.linear_cap.id = "INDEPENDENT_ORACLE_D2_CAP";
        }
        const CommandPublicationResult published =
            publication_transaction.execute(request);
        if (!published.published() || !published.limiter_state_committed ||
            !sameCommandWithin(published.pre_publication_stage_command,
                               bt_transaction.command,
                               structure.identity_tolerance)) {
            result.status = "ORACLE_PUBLICATION_TRANSACTION_FAILED";
            return result;
        }

        const ExecutionStepResult stepped = execution_model.step(
            state.execution, published.finalized.command);
        if (!stepped.valid) {
            result.status = "ORACLE_EXECUTION_STEP_FAILED_" + stepped.status;
            return result;
        }
        state.execution = stepped.state;
        double progress_increment = 0.0;
        for (const ExecutionPropagationSegment& segment : stepped.segments) {
            progress_increment +=
                std::max(0.0, segment.output_v) * segment.duration_sec;
        }
        state.progress_s += progress_increment;
        if (!std::isfinite(state.progress_s)) {
            result.status = "ORACLE_NONFINITE_PROGRESS";
            return result;
        }
        result.published_commands.push_back(published.finalized.command);
        result.states.push_back(state);
    }
    result.valid = true;
    result.status = "OK";
    return result;
}

}  // namespace bt_residual
}  // namespace spmpc_local_planner
