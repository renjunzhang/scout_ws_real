#pragma once

#include <string>

#include <ros/time.h>

#include "lt_dwa_official_wrapper/planner_config.hpp"
#include "lt_dwa_official_wrapper/status.hpp"
#include "lt_dwa_official_wrapper/types.hpp"

namespace lt_dwa_official_wrapper {

struct ValidationResult {
  WrapperStatus status{WrapperStatus::kOk};
  std::string reason;

  bool ok() const { return status == WrapperStatus::kOk; }
};

class FrameValidator {
 public:
  ValidationResult ValidateInput(const PlannerInput& input,
                                 const PlannerConfig& config,
                                 const ros::Time& now = ros::Time()) const;

 private:
  ValidationResult ValidateFrame(const std::string& actual,
                                 const std::string& expected,
                                 const std::string& field_name) const;
  ValidationResult ValidatePath(const std::vector<Pose2d>& path,
                                const std::string& expected_frame) const;
  ValidationResult ValidateMap(const nav_msgs::OccupancyGrid& map,
                               const std::string& expected_frame) const;
  ValidationResult ValidateObstacles(const std::vector<ObstacleTrack>& obstacles,
                                     const std::string& expected_frame) const;
};

}  // namespace lt_dwa_official_wrapper
