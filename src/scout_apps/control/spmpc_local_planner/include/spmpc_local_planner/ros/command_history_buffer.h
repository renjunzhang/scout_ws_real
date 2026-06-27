#pragma once

#include "spmpc_local_planner/ros/delay_phase_types.h"
#include <deque>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <vector>

namespace spmpc_local_planner {

struct TimedCommandSample {
    ros::Time stamp;
    geometry_msgs::Twist cmd;
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
    ros::Time oldestStamp() const;
    ros::Time latestStamp() const;

    bool sampleAt(const ros::Time& stamp, TimedCommandSample& sample) const;
    std::vector<TimedCommandSample> segment(const ros::Time& start, const ros::Time& end) const;

private:
    void prune();

    std::deque<TimedCommandSample> samples_;
    double window_sec_ = 2.0;
    double latest_period_sec_ = 0.0;
};

}  // namespace spmpc_local_planner
