#include "spmpc_local_planner/runtime/execution_prediction/command_history_buffer.h"

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
    if (!validStamp(sample.stamp_ns)) {
        return;
    }
    if (!samples_.empty() && sample.stamp_ns < samples_.back().stamp_ns) {
        samples_.clear();
        latest_period_sec_ = 0.0;
    } else if (!samples_.empty()) {
        latest_period_sec_ = secondsBetween(sample.stamp_ns, samples_.back().stamp_ns);
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
    const double span = secondsBetween(samples_.back().stamp_ns, samples_.front().stamp_ns);
    return std::isfinite(span) && span > 0.0 ? span : 0.0;
}

StampNs CommandHistoryBuffer::oldestStampNs() const {
    return samples_.empty() ? 0 : samples_.front().stamp_ns;
}

StampNs CommandHistoryBuffer::latestStampNs() const {
    return samples_.empty() ? 0 : samples_.back().stamp_ns;
}

bool CommandHistoryBuffer::sampleAt(StampNs stamp_ns, TimedCommandSample& sample) const {
    if (samples_.empty() || !validStamp(stamp_ns)) {
        return false;
    }
    if (stamp_ns < samples_.front().stamp_ns) {
        return false;
    }
    auto it = std::upper_bound(
        samples_.begin(),
        samples_.end(),
        stamp_ns,
        [](StampNs lhs, const TimedCommandSample& rhs) {
            return lhs < rhs.stamp_ns;
        });
    if (it == samples_.begin()) {
        return false;
    }
    --it;
    sample = *it;
    return true;
}

std::vector<TimedCommandSample> CommandHistoryBuffer::segment(StampNs start_ns, StampNs end_ns) const {
    std::vector<TimedCommandSample> out;
    if (samples_.empty() || end_ns < start_ns) {
        return out;
    }
    for (const auto& sample : samples_) {
        if (sample.stamp_ns < start_ns) {
            continue;
        }
        if (end_ns < sample.stamp_ns) {
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
    const double window_sec = std::max(0.0, window_sec_);
    const StampNs window_ns = secondsToNanoseconds(window_sec);
    if (window_ns <= 0 || samples_.back().stamp_ns <= window_ns) {
        return;
    }
    const StampNs cutoff_ns = samples_.back().stamp_ns - window_ns;
    while (samples_.size() > 1 && samples_.front().stamp_ns < cutoff_ns) {
        samples_.pop_front();
    }
}

}  // namespace spmpc_local_planner
