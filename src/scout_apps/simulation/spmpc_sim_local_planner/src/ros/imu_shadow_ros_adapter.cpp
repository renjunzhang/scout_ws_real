#include "spmpc_sim_local_planner/ros/imu_shadow_ros_adapter.h"

#include <cstdint>

namespace spmpc_sim_local_planner {

bool ImuShadowRosAdapter::configure(
    const ProcessedImuParams& pipeline_params,
    const std::string& expected_frame) {
    expected_frame_ = expected_frame;
    return pipeline_.configure(pipeline_params);
}

void ImuShadowRosAdapter::reset() {
    pipeline_.reset();
}

ProcessedImuOutput ImuShadowRosAdapter::process(
    const sensor_msgs::Imu& msg,
    const ros::Time& receive_stamp) {
    ImuSample sample = toSample(msg, receive_stamp);
    if (!expected_frame_.empty() && msg.header.frame_id != expected_frame_) {
        return pipeline_.reject(sample, ImuPipelineStatusCode::FrameMismatch);
    }
    if (msg.orientation_covariance[0] == -1.0) {
        sample.orientation_available = false;
    }
    if (msg.angular_velocity_covariance[0] == -1.0 ||
        msg.linear_acceleration_covariance[0] == -1.0) {
        return pipeline_.reject(sample, ImuPipelineStatusCode::InvalidSample);
    }
    return pipeline_.process(sample);
}

ImuSample ImuShadowRosAdapter::toSample(
    const sensor_msgs::Imu& msg,
    const ros::Time& receive_stamp) {
    ImuSample sample;
    sample.source_stamp_ns = static_cast<std::int64_t>(msg.header.stamp.toNSec());
    sample.receive_stamp_ns = static_cast<std::int64_t>(receive_stamp.toNSec());
    sample.orientation_available = true;
    sample.orientation_x = msg.orientation.x;
    sample.orientation_y = msg.orientation.y;
    sample.orientation_z = msg.orientation.z;
    sample.orientation_w = msg.orientation.w;
    sample.angular_velocity_x = msg.angular_velocity.x;
    sample.angular_velocity_y = msg.angular_velocity.y;
    sample.angular_velocity_z = msg.angular_velocity.z;
    sample.linear_acceleration_x = msg.linear_acceleration.x;
    sample.linear_acceleration_y = msg.linear_acceleration.y;
    sample.linear_acceleration_z = msg.linear_acceleration.z;
    return sample;
}

}  // namespace spmpc_sim_local_planner
