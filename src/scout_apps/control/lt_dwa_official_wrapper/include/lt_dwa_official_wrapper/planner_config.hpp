#pragma once

#include <string>

namespace lt_dwa_official_wrapper {

struct PlannerConfig {
  std::string planning_frame{"odom"};

  double max_v{1.0};
  double min_v{0.0};
  double max_w{1.0};
  double max_acc{1.0};
  double max_angular_acc{1.0};
  double robot_radius{0.3};
  double scan_radius{3.5};
  double time_step{0.2};

  double path_resample_spacing{0.10};
  double input_stale_timeout_sec{0.5};
  double goal_xy_tolerance{0.3};
  double goal_yaw_tolerance{0.5};

  unsigned int deterministic_seed{0};
  bool enable_worker_isolation{false};
  bool publish_debug_topics{false};
};

}  // namespace lt_dwa_official_wrapper
