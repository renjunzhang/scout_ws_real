#include "spmpc_local_planner/controller/command/publication_transaction.h"

#include <cmath>

namespace spmpc_local_planner {
namespace {

bool finiteCommand(const VelocityCommand& command) {
    return std::isfinite(command.linear) && std::isfinite(command.angular);
}

bool sameCommand(const VelocityCommand& lhs, const VelocityCommand& rhs) {
    return lhs.linear == rhs.linear && lhs.angular == rhs.angular;
}

CommandPublishMeta publishMeta(const CommandPipelineResult& pipeline) {
    CommandPublishMeta meta;
    meta.is_zero_cmd =
        std::abs(pipeline.final_command.linear) <= 1e-9 &&
        std::abs(pipeline.final_command.angular) <= 1e-9;
    meta.linear_limited = pipeline.linear_limited;
    meta.angular_rate_limited = pipeline.angular_rate_limited;
    meta.angular_accel_limited = pipeline.angular_accel_limited;
    return meta;
}

}  // namespace

CommandPublicationResult PublicationTransaction::execute(
    const CommandPublicationRequest& request) {
    CommandPublicationResult result;
    if (request.sink == nullptr) {
        result.receipt.cycle_id = request.cycle_id;
        result.receipt.status = "COMMAND_SINK_MISSING";
        return result;
    }

    CommandPipelineRequest pipeline_request;
    pipeline_request.stamp_ns = request.sink->publicationTimeNs();
    pipeline_request.desired = request.proposed.command;
    pipeline_request.source = request.proposed.source;
    pipeline_request.reason = request.proposed.reason;
    pipeline_request.force_zero = request.force_zero;
    pipeline_request.accepted = request.proposed.accepted;
    result.pipeline = pipeline_.finalize(pipeline_request);

    result.finalized.cycle_id = request.cycle_id;
    result.finalized.finalized_stamp_ns = pipeline_request.stamp_ns;
    result.finalized.command = result.pipeline.final_command;
    result.finalized.source = result.pipeline.decision.source;
    result.finalized.reason = result.pipeline.decision.reason;
    result.finalized.publish_enabled = request.publish_enabled;
    result.finalized.meta = publishMeta(result.pipeline);

    result.receipt = request.sink->publish(result.finalized);
    result.pipeline.command_was_published = result.receipt.delivered;
    result.receipt_consistent = request.publish_enabled &&
        result.receipt.attempted && result.receipt.delivered &&
        result.receipt.cycle_id == request.cycle_id &&
        validStamp(result.receipt.actual_publish_stamp_ns) &&
        finiteCommand(result.receipt.command) &&
        sameCommand(result.receipt.command, result.finalized.command);

    // The receipt is the best available execution truth.  Even an
    // inconsistent sink receipt must advance prediction/limiter state to the
    // command it claims was handed to the transport, while preventing phase
    // commit through receipt_consistent=false.
    if (request.publish_enabled && result.receipt.attempted &&
        result.receipt.delivered &&
        validStamp(result.receipt.actual_publish_stamp_ns) &&
        finiteCommand(result.receipt.command)) {
        result.limiter_state_committed = pipeline_.commitPublished(
            result.receipt.command,
            result.receipt.actual_publish_stamp_ns);
        if (request.history != nullptr) {
            TimedCommandSample sample;
            sample.stamp_ns = result.receipt.actual_publish_stamp_ns;
            sample.command = result.receipt.command;
            sample.meta = result.finalized.meta;
            request.history->push(sample);
            result.history_committed =
                request.history->latestStampNs() == sample.stamp_ns;
        }
    }
    return result;
}

}  // namespace spmpc_local_planner
