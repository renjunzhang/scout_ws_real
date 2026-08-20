#pragma once

namespace spmpc_local_planner {

struct VelocityCommand {
    double linear = 0.0;
    double angular = 0.0;
};

}  // namespace spmpc_local_planner
