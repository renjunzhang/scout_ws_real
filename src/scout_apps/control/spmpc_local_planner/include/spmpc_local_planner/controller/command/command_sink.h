#pragma once

#include "spmpc_local_planner/controller/command/command_pipeline.h"
#include "spmpc_local_planner/runtime/execution_prediction/types.h"

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

// Immutable command handed to the transport boundary.  A sink is not allowed
// to clamp, replace or otherwise reinterpret this value.
struct FinalCommand {
    std::uint64_t cycle_id = 0;
    StampNs finalized_stamp_ns = 0;
    VelocityCommand command;
    CommandSource source = CommandSource::None;
    std::string reason = "NO_COMMAND";
    bool publish_enabled = true;
    CommandPublishMeta meta;
};

// Receipt means that the command was handed to the configured transport.  It
// is not an acknowledgement from the Scout CAN bus; that stronger contract is
// introduced by the formal runtime supervisor in WP4/WP5.
struct PublicationReceipt {
    std::uint64_t cycle_id = 0;
    bool attempted = false;
    bool delivered = false;
    StampNs actual_publish_stamp_ns = 0;
    VelocityCommand command;
    std::string status = "NOT_ATTEMPTED";
};

class ICommandSink {
public:
    virtual ~ICommandSink() = default;

    // Sampled immediately before finalization so the limiter uses the same
    // transport clock as the publication receipt.
    virtual StampNs publicationTimeNs() = 0;
    virtual PublicationReceipt publish(const FinalCommand& command) = 0;
};

}  // namespace spmpc_local_planner
