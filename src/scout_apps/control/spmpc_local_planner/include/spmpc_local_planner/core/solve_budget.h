#pragma once

#include <chrono>

namespace spmpc_local_planner {

// An unset deadline permits offline runs with the configured iteration count.
// A running acados call cannot be preempted; the publication gate is final.
struct SolveBudget {
    using Clock = std::chrono::steady_clock;
    Clock::time_point deadline{};

    bool permits(double estimated_iteration_sec, Clock::time_point now = Clock::now()) const {
        return deadline == Clock::time_point{} ||
            std::chrono::duration<double>(deadline - now).count() > estimated_iteration_sec;
    }
};

}  // namespace spmpc_local_planner
