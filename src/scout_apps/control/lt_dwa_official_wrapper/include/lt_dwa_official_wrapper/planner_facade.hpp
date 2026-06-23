#pragma once

#include <string>

#include <ros/time.h>

#include "lt_dwa_official_wrapper/frame_validator.hpp"
#include "lt_dwa_official_wrapper/planner_config.hpp"
#include "lt_dwa_official_wrapper/types.hpp"

namespace lt_dwa_official_wrapper {

class PlannerFacade {
 public:
  explicit PlannerFacade(const PlannerConfig& config);

  PlannerOutput PlanOnce(const PlannerInput& input) const;
  PlannerOutput PlanOnce(const PlannerInput& input, const ros::Time& now) const;

 private:
  PlannerOutput MakeRejectedOutput(const PlannerInput& input,
                                   WrapperStatus status,
                                   const std::string& reason,
                                   const ros::Time& now) const;
  PlannerDiagnostics BuildBaseDiagnostics(const PlannerInput& input,
                                          const ros::Time& now) const;

  PlannerConfig config_;
  FrameValidator validator_;
};

}  // namespace lt_dwa_official_wrapper
