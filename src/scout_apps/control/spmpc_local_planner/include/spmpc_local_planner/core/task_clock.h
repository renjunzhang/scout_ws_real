#pragma once
#include "spmpc_local_planner/core/types.h"
#include <cmath>

namespace spmpc_local_planner {
class TaskClock {
public:
    void reset() { epoch_ns_=0; elapsed_=0; initialized_=false; }
    bool observe(const SolverInput& input, double& elapsed) {
        double value = input.task_elapsed_sec;
        if (!input.has_task_elapsed) {
            const auto stamp = input.cycle_timing.solver_input_epoch_ns;
            if (stamp <= 0) return false;
            if (!initialized_) epoch_ns_ = stamp;
            value = static_cast<double>(stamp-epoch_ns_)*1e-9;
        }
        if (!std::isfinite(value) || value < 0 || (initialized_ && value+1e-9 < elapsed_)) return false;
        initialized_=true; elapsed_=value; elapsed=value;
        return true;
    }
private:
    std::int64_t epoch_ns_=0;
    double elapsed_=0;
    bool initialized_=false;
};
}  // namespace spmpc_local_planner
