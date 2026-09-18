#pragma once

#include "spmpc_local_planner/estimation/slosh_observer_bank.h"
#include "spmpc_local_planner/ros/control_cycle_contract.h"
#include <mutex>

namespace spmpc_local_planner {

struct OdomControlSnapshot {
    StampedRobotState robot;
    SloshObserverSnapshot liquid;
    std::int64_t receive_stamp_ns = 0;
    std::string frame_id;
    std::deque<StampedRobotState> history;
};

// The observer may advance while a control cycle reads its input. Publish the
// accepted odometry and its observer result together; readers own a complete
// immutable copy, including the history used for alignment. No ROS dependency.
class OdomStateBuffer {
public:
    bool commit(const StampedRobotState& robot, const SloshObserverSnapshot& liquid,
                const std::string& frame_id, std::int64_t receive_stamp_ns,
                double history_window_sec);
    OdomControlSnapshot snapshot() const;

private:
    mutable std::mutex mutex_;
    OdomControlSnapshot committed_;
};

}  // namespace spmpc_local_planner
