#pragma once

#include <string>

#include "lt_dwa_official_wrapper/types.hpp"

namespace lt_dwa_official_wrapper {

struct WorkerRequestParseResult {
  bool ok{false};
  std::string reason;
  PlannerInput input;
  ros::Time now;
};

std::string SerializeWorkerRequest(const PlannerInput& input, const ros::Time& now = ros::Time());
WorkerRequestParseResult ParseWorkerRequestText(const std::string& text);
WorkerRequestParseResult LoadWorkerRequestFile(const std::string& path);

}  // namespace lt_dwa_official_wrapper
