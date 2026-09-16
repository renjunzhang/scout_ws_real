#pragma once

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

enum class RecordedCommandDecision {
    Accepted,
    Duplicate,
    Rejected,
    RestartRequired,
};

struct RecordedCommandRecord {
    std::uint32_t schema_version = 0;
    std::uint64_t cycle_id = 0;
    bool command_was_published = false;
    bool publish_cmd_vel = false;
    std::int64_t command_publish_stamp_ns = 0;
    std::int64_t receive_stamp_ns = 0;
    double published_cmd_v = 0.0;
    double published_cmd_omega = 0.0;
    std::string source;
};

RecordedCommandDecision validateRecordedCommand(
    const RecordedCommandRecord* previous,
    const RecordedCommandRecord& current,
    double max_age_sec);

}  // namespace spmpc_local_planner
