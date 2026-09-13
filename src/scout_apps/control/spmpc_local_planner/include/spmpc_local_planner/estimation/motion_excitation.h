#pragma once

#include <cstdint>
#include "spmpc_local_planner/dynamics/slosh_types.h"

namespace spmpc_local_planner {

enum class MotionExcitationSource : std::uint8_t {
    Unknown = 0,
    Odom = 1,
    ProcessedImu = 2,
};

// ROS-independent excitation at the container centre, in container/base axes.
// Current installation assumption: container centre == base_link, no relative
// yaw. Processed IMU applies its existing lever arm once after aligning the
// filtered signals. Odom assumes its twist reference is base_link. Model
// version 1 uses actual omega_z AND alpha_z in the relative liquid dynamics.
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

    ContainerExcitation atContainer() const { return {ax, ay, omega_z, alpha_z}; }
};

}  // namespace spmpc_local_planner
