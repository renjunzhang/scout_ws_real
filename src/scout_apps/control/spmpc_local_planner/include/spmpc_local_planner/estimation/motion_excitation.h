#pragma once

#include <cstdint>

namespace spmpc_local_planner {

enum class MotionExcitationSource : std::uint8_t {
    Unknown = 0,
    Odom = 1,
    ProcessedImu = 2,
};

// ROS-independent, timestamped rigid-body excitation at the liquid-observer
// reference point.  Components are expressed in the robot-base axes, but the
// reference point is source-specific: odom uses its twist reference, while the
// processed-IMU channel uses a configured nominal lever-arm target (a
// development ICR proxy, not a claimed physical base_link origin).
// All timestamps use the ROS clock domain but are stored as integer nanoseconds
// so the numerical core never depends on ros::Time.
struct MotionExcitation {
    MotionExcitationSource source = MotionExcitationSource::Unknown;
    bool valid = false;

    double ax = 0.0;
    double ay = 0.0;
    double omega_z = 0.0;
    double alpha_z = 0.0;
    double sample_dt_sec = 0.0;

    std::int64_t source_stamp_ns = 0;
    std::int64_t measurement_stamp_ns = 0;
    std::int64_t accel_effective_stamp_ns = 0;
    std::int64_t gyro_effective_stamp_ns = 0;
    std::int64_t alpha_effective_stamp_ns = 0;
    std::int64_t receive_stamp_ns = 0;

    std::uint32_t reset_epoch = 0;
};

}  // namespace spmpc_local_planner
