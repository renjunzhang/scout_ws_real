#include "spmpc_local_planner/ros/command_history_buffer.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

void CommandHistoryBuffer::configure(double window_sec) {
    if (std::isfinite(window_sec) && window_sec > 0.0) {
        window_sec_ = window_sec;
    }
    prune();
}

void CommandHistoryBuffer::clear() {
    samples_.clear();
    latest_period_sec_ = 0.0;
}

void CommandHistoryBuffer::push(const TimedCommandSample& sample) {
    if (sample.stamp.isZero()) {
        return;
    }
    if (!samples_.empty() && sample.stamp < samples_.back().stamp) {
        samples_.clear();
        latest_period_sec_ = 0.0;
    } else if (!samples_.empty()) {
        latest_period_sec_ = (sample.stamp - samples_.back().stamp).toSec();
        if (!std::isfinite(latest_period_sec_) || latest_period_sec_ < 0.0) {
            latest_period_sec_ = 0.0;
        }
    }
    samples_.push_back(sample);
    prune();
}

double CommandHistoryBuffer::spanSec() const {
    if (samples_.size() < 2) {
        return 0.0;
    }
    const double span = (samples_.back().stamp - samples_.front().stamp).toSec();
    return std::isfinite(span) && span > 0.0 ? span : 0.0;
}

ros::Time CommandHistoryBuffer::oldestStamp() const {
    return samples_.empty() ? ros::Time() : samples_.front().stamp;
}

ros::Time CommandHistoryBuffer::latestStamp() const {
    return samples_.empty() ? ros::Time() : samples_.back().stamp;
}

bool CommandHistoryBuffer::sampleAt(const ros::Time& stamp, TimedCommandSample& sample) const {
    if (samples_.empty() || stamp.isZero()) {
        return false;
    }
    if (stamp < samples_.front().stamp) {
        return false;
    }
    auto it = std::upper_bound(
        samples_.begin(),
        samples_.end(),
        stamp,
        [](const ros::Time& lhs, const TimedCommandSample& rhs) {
            return lhs < rhs.stamp;
        });
    if (it == samples_.begin()) {
        return false;
    }
    --it;
    sample = *it;
    return true;
}

std::vector<TimedCommandSample> CommandHistoryBuffer::segment(const ros::Time& start, const ros::Time& end) const {
    std::vector<TimedCommandSample> out;
    if (samples_.empty() || end < start) {
        return out;
    }
    for (const auto& sample : samples_) {
        if (sample.stamp < start) {
            continue;
        }
        if (end < sample.stamp) {
            break;
        }
        out.push_back(sample);
    }
    return out;
}

void CommandHistoryBuffer::prune() {
    if (samples_.empty()) {
        return;
    }
    const double latest_sec = samples_.back().stamp.toSec();
    const double window_sec = std::max(0.0, window_sec_);
    if (!std::isfinite(latest_sec) || latest_sec <= window_sec) {
        return;
    }
    ros::Time cutoff;
    cutoff.fromSec(latest_sec - window_sec);
    while (samples_.size() > 1 && samples_.front().stamp < cutoff) {
        samples_.pop_front();
    }
}

}  // namespace spmpc_local_planner
