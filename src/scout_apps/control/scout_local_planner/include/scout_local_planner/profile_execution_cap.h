/**
 * @file profile_execution_cap.h
 * @brief External speed profile execution cap for TOPPRA/Ruckig-style baselines.
 */

#pragma once

#include "scout_local_planner/path_handler.h"
#include "scout_local_planner/types.h"

#include <limits>

namespace scout_local_planner {

struct ProfileExecutionCapParams {
    bool enable = false;
    double accel_limit = 0.0;
    double decel_limit = 0.0;
    double jerk_limit = 0.0;
};

struct ProfileExecutionCapOutput {
    double cmd_v = 0.0;
    bool applied = false;
    int active = 0;
    double v_profile = std::numeric_limits<double>::quiet_NaN();
    double cmd_v_pre = std::numeric_limits<double>::quiet_NaN();
    double cmd_v_post = std::numeric_limits<double>::quiet_NaN();
    double implied_ax = std::numeric_limits<double>::quiet_NaN();
    double implied_jerk = std::numeric_limits<double>::quiet_NaN();
};

class ProfileExecutionCap {
public:
    void setParams(const ProfileExecutionCapParams& params);
    void reset();

    ProfileExecutionCapOutput apply(double cmd_v,
                                    double filtered_v,
                                    double dt,
                                    const PathHandler& path_handler,
                                    const PathHandlerParams& path_params,
                                    const VehicleParams& vehicle_params);

private:
    ProfileExecutionCapParams params_;
    bool has_last_ax_ = false;
    double last_ax_ = 0.0;
};

}  // namespace scout_local_planner
