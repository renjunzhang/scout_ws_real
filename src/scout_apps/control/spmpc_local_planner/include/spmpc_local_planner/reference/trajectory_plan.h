#pragma once
#include "spmpc_local_planner/reference/motion_region.h"
#include <array>
#include <string>
#include <vector>

namespace spmpc_local_planner {

// Full explicit-actuator/slosh state and held control; no rescaled liquid trace.
struct TrajectoryPlanSample {
    double t = 0;
    std::vector<double> state;
    std::array<double, 3> control{{0, 0, 0}};
    std::string phase;
};

struct TrajectoryPlan {
    int schema_version = 1, liquid_model_version = 1, cost_model_version = 3;
    std::string plan_id, frame_id, region_id;
    double dt = 0, transport_duration = 0, deadline = 0, stop_window = 0, height_coeff = 0;
    // tau_v, tau_omega, gain_v, gain_omega; 2*zeta*wn, wn^2, kx, ky.
    std::array<double, 4> actuator_parameters{}, liquid_parameters{};
    double actual_jerk_max = 0.0; // Optional actuator-output control-grid contract (0: archived/off).
    // actual_v_min, v_max, omega_max, a_max, alpha_max, jerk_max.
    std::array<double, 6> motion_limits{};
    std::array<double, 3> goal_pose{};
    double goal_position_tolerance = 0, goal_yaw_tolerance = 0;
    double stop_speed_tolerance = 0, stop_omega_tolerance = 0;
    MotionRegionConfig region;
    std::vector<RegionVertex> route;
    std::vector<TrajectoryPlanSample> samples;

    static TrajectoryPlan load(const std::string& path);
    void validate() const;
};

bool validateRoute(const std::vector<RegionVertex>& route, const std::string& frame,
                   double tolerance, const TrajectoryPlan& plan);

}  // namespace spmpc_local_planner
