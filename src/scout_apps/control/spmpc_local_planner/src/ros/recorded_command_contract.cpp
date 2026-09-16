#include "spmpc_local_planner/ros/recorded_command_contract.h"

#include <cmath>

namespace spmpc_local_planner {
namespace {

bool sameRecord(const RecordedCommandRecord& a, const RecordedCommandRecord& b) {
    return a.schema_version == b.schema_version && a.cycle_id == b.cycle_id &&
        a.command_was_published == b.command_was_published &&
        a.publish_cmd_vel == b.publish_cmd_vel &&
        a.command_publish_stamp_ns == b.command_publish_stamp_ns &&
        a.published_cmd_v == b.published_cmd_v &&
        a.published_cmd_omega == b.published_cmd_omega && a.source == b.source;
}

}  // namespace

RecordedCommandDecision validateRecordedCommand(
    const RecordedCommandRecord* previous,
    const RecordedCommandRecord& current,
    double max_age_sec) {
    if (current.schema_version != 2 || current.cycle_id == 0 ||
        !current.command_was_published || !current.publish_cmd_vel ||
        current.command_publish_stamp_ns <= 0 || current.receive_stamp_ns <= 0 ||
        current.source.empty() ||
        !std::isfinite(current.published_cmd_v) ||
        !std::isfinite(current.published_cmd_omega) ||
        !std::isfinite(max_age_sec) || max_age_sec < 0.0) {
        return RecordedCommandDecision::Rejected;
    }

    if (previous && (current.source != previous->source ||
        (current.receive_stamp_ns < previous->receive_stamp_ns) ||
        (current.command_publish_stamp_ns < previous->command_publish_stamp_ns) ||
        current.cycle_id < previous->cycle_id)) {
        return RecordedCommandDecision::RestartRequired;
    }
    if (previous && current.cycle_id == previous->cycle_id) {
        return sameRecord(*previous, current)
            ? RecordedCommandDecision::Duplicate
            : RecordedCommandDecision::RestartRequired;
    }
    if (previous && current.command_publish_stamp_ns <= previous->command_publish_stamp_ns) {
        return RecordedCommandDecision::RestartRequired;
    }
    const std::int64_t age_ns = current.receive_stamp_ns - current.command_publish_stamp_ns;
    if (age_ns < 0 || static_cast<double>(age_ns) * 1e-9 > max_age_sec) {
        return RecordedCommandDecision::Rejected;
    }
    return RecordedCommandDecision::Accepted;
}

}  // namespace spmpc_local_planner
