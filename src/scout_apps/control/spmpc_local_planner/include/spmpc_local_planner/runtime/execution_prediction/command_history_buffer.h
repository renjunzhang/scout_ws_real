#pragma once

#include "spmpc_local_planner/domain/command.h"
#include "spmpc_local_planner/domain/time.h"
#include "spmpc_local_planner/runtime/execution_prediction/types.h"
#include <deque>
#include <vector>

namespace spmpc_local_planner {

struct TimedCommandSample {
    StampNs stamp_ns = 0;
    VelocityCommand command;
    CommandPublishMeta meta;
};

class CommandHistoryBuffer {
public:
    void configure(double window_sec);
    void clear();
    void push(const TimedCommandSample& sample);

    bool empty() const { return samples_.empty(); }
    std::size_t size() const { return samples_.size(); }
    double windowSec() const { return window_sec_; }
    double spanSec() const;
    double latestPeriodSec() const { return latest_period_sec_; }
    StampNs oldestStampNs() const;
    StampNs latestStampNs() const;

    bool sampleAt(StampNs stamp_ns, TimedCommandSample& sample) const;
    std::vector<TimedCommandSample> segment(StampNs start_ns, StampNs end_ns) const;

private:
    void prune();

    std::deque<TimedCommandSample> samples_;
    double window_sec_ = 2.0;
    double latest_period_sec_ = 0.0;
};

}  // namespace spmpc_local_planner
