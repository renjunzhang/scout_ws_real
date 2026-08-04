#pragma once

#include "spmpc_sim_local_planner/estimation/processed_imu_pipeline.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string>

namespace spmpc_sim_local_planner {

// Thin ROS1 boundary.  sensor_msgs and ros::Time intentionally stop here; the
// calibration/filtering core remains replayable as plain C++14.
class ImuShadowRosAdapter {
public:
    bool configure(const ProcessedImuParams& pipeline_params,
                   const std::string& expected_frame);
    void reset();

    ProcessedImuOutput process(const sensor_msgs::Imu& msg,
                               const ros::Time& receive_stamp);

    const std::string& expectedFrame() const { return expected_frame_; }
    const ProcessedImuPipeline& pipeline() const { return pipeline_; }

private:
    static ImuSample toSample(const sensor_msgs::Imu& msg,
                              const ros::Time& receive_stamp);

    std::string expected_frame_;
    ProcessedImuPipeline pipeline_;
};

}  // namespace spmpc_sim_local_planner
