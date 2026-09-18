#include "spmpc_local_planner/ros/odom_state_buffer.h"
#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

bool OdomStateBuffer::commit(const StampedRobotState& robot,
    const SloshObserverSnapshot& liquid, const std::string& frame_id,
    std::int64_t receive_stamp_ns, double history_window_sec) {
    if (robot.stamp_ns <= 0 || receive_stamp_ns <= 0 || frame_id.empty() ||
        !std::isfinite(history_window_sec) || history_window_sec <= 0.0 ||
        (liquid.valid && liquid.state_stamp_ns != robot.stamp_ns)) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    auto& history = committed_.history;
    if (robot.stamp_ns <= committed_.robot.stamp_ns || frame_id != committed_.frame_id)
        history.clear();  // A source reset cannot interpolate across epochs.
    history.push_back(robot);
    const auto history_ns = static_cast<std::int64_t>(std::max(0.1, history_window_sec)*1e9);
    while (history.size() > 1 && robot.stamp_ns-history.front().stamp_ns > history_ns)
        history.pop_front();
    committed_.robot = robot;
    committed_.liquid = liquid;  // Preserve invalidation; never invent liquid history.
    committed_.frame_id = frame_id;
    committed_.receive_stamp_ns = receive_stamp_ns;
    return true;
}

OdomControlSnapshot OdomStateBuffer::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return committed_;
}

}  // namespace spmpc_local_planner
