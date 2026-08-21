#pragma once

#include <string>

namespace spmpc_local_planner {

struct ExecutionChannelContract {
    double delay_sec = 0.0;
    double time_constant_sec = 0.0;
    double positive_gain = 1.0;
    double negative_gain = 1.0;
    double deadzone = 0.0;
    double output_min = -1.0e6;
    double output_max = 1.0e6;

    // Derived by ExecutionModel::configure().  Callers do not provide a
    // second delay interpretation.
    int integer_delay_steps = 0;
    double fractional_delay_sec = 0.0;
};

struct ExecutionModelContract {
    int schema_version = 1;
    std::string contract_id;
    std::string contract_hash;
    double dt = 1.0 / 30.0;
    ExecutionChannelContract linear;
    ExecutionChannelContract angular;
};

}  // namespace spmpc_local_planner
