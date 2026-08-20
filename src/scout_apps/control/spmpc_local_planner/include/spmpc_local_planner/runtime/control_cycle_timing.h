#pragma once

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

// ROS-independent timestamps for one authoritative control cycle.  All stamps
// are nanoseconds in the application clock domain.  The solver input epoch is
// the physical time represented by both x_robot and x_liquid after alignment.
struct ControlCycleTimingDebug {
    std::uint64_t cycle_id = 0;
    std::int64_t cycle_start_stamp_ns = 0;
    std::int64_t raw_robot_state_stamp_ns = 0;
    std::int64_t raw_liquid_state_stamp_ns = 0;
    std::int64_t robot_state_stamp_ns = 0;
    std::int64_t liquid_state_stamp_ns = 0;
    std::int64_t solver_input_epoch_ns = 0;
    std::int64_t solve_start_stamp_ns = 0;
    std::int64_t solve_end_stamp_ns = 0;
    std::int64_t horizon_available_stamp_ns = 0;
    std::int64_t command_publish_stamp_ns = 0;
    double raw_state_skew_sec = 0.0;
    double aligned_state_skew_sec = 0.0;
    bool state_alignment_required = false;
    bool state_time_aligned = false;
    bool robot_state_interpolated = false;
    bool robot_state_extrapolated = false;
    std::string state_alignment_status = "NOT_EVALUATED";
};

}  // namespace spmpc_local_planner
