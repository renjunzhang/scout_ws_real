#pragma once

#include "spmpc_local_planner/reference/motion_region.h"
#include <string>

namespace spmpc_local_planner {

enum class TrajectoryReferenceMode { Cruise = 0, Progress = 1, FixedTime = 2 };

struct GeometryObjectiveConfig {
    bool enabled = false;
    double curvature_weight = 0.0;
    double curvature_rate_weight = 0.0;
    double curvature_scale = 1.0;       // 1/m
    double curvature_rate_scale = 1.0;  // 1/m^2, curvature change per travelled metre
    double speed_regularization = 0.05; // m/s; keeps turning at rest penalized
    double goal_weight = 0.0;
    double goal_position_scale = 0.3;
    double goal_yaw_scale = 1.0;
    bool reference_curvature_speed_limit = true;
};

struct TrajectoryReferenceConfig {
    TrajectoryReferenceMode mode = TrajectoryReferenceMode::Cruise;
    std::string plan_file;
    double progress_window = 0.30;       // m, interpolation domain around stage seed
    double max_speed_error = 0.02;      // m/s, approximation check against stored plan
    double route_tolerance = 1e-5;      // m, route identity comparison
    double deadline_tolerance = 0.0;    // s; never resets the task clock
};

struct PlanningConfig {
    std::string experiment_profile_id = "legacy";
    bool liquid_free_baseline = false;
    double task_deadline_sec = 0.0;     // 0 disables the deadline for legacy tasks
    double evaluation_window_sec = 1.0;
    GeometryObjectiveConfig geometry;
    MotionRegionConfig region;
    TrajectoryReferenceConfig trajectory;
};

bool validatePlanningConfig(const PlanningConfig& config, std::string* reason = nullptr);
const char* trajectoryReferenceModeName(TrajectoryReferenceMode mode);
TrajectoryReferenceMode parseTrajectoryReferenceMode(const std::string& value);
std::string planningConfigJson(const PlanningConfig& config);

}  // namespace spmpc_local_planner
