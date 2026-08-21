#pragma once

#include "spmpc_local_planner/domain/state.h"

#include <cstdint>
#include <deque>

namespace spmpc_local_planner {

struct ExecutionChannelState {
    // Oldest to newest.  A configured state contains integer_delay_steps + 1
    // commands before the next command is appended.
    std::deque<double> pending_commands;
    double actuator_output = 0.0;
};

struct ExecutionAugmentedState {
    bool valid = false;
    std::uint64_t stage_index = 0;
    RobotState robot;
    SloshState slosh;
    ExecutionChannelState linear;
    ExecutionChannelState angular;
};

}  // namespace spmpc_local_planner
