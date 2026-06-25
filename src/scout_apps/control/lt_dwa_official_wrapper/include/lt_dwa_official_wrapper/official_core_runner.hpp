#pragma once

#include <string>

#include "lt_dwa_official_wrapper/planner_config.hpp"
#include "lt_dwa_official_wrapper/status.hpp"
#include "lt_dwa_official_wrapper/types.hpp"

namespace lt_dwa_official_wrapper {

struct OfficialCoreResult {
  WrapperStatus status{WrapperStatus::kCoreProcessExited};
  std::string reason;
  Twist2d raw_command;
  Twist2d final_command;
  bool guard_applied{false};
  std::string guard_reason;
  int core_return{-1};
};

OfficialCoreResult RunOfficialCoreOnce(const PlannerInput& input,
                                       const PlannerConfig& config,
                                       const std::string& official_source_root);

}  // namespace lt_dwa_official_wrapper
