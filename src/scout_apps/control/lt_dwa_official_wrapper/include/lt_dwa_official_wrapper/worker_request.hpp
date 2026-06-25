#pragma once

#include <string>

#include "lt_dwa_official_wrapper/planner_config.hpp"
#include "lt_dwa_official_wrapper/types.hpp"

namespace lt_dwa_official_wrapper {

struct WorkerRequestParseResult {
  bool ok{false};
  std::string reason;
  PlannerInput input;
  PlannerConfig config;
  bool has_config{false};
  ros::Time now;
};

std::string SerializeWorkerRequest(const PlannerInput& input, const ros::Time& now = ros::Time());
std::string SerializeWorkerRequest(const PlannerInput& input,
                                   const PlannerConfig& config,
                                   const ros::Time& now = ros::Time());
WorkerRequestParseResult ParseWorkerRequestText(const std::string& text);
WorkerRequestParseResult LoadWorkerRequestFile(const std::string& path);

}  // namespace lt_dwa_official_wrapper
