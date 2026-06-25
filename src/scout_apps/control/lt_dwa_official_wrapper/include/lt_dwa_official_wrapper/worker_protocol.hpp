#pragma once

#include <string>

#include "lt_dwa_official_wrapper/status.hpp"

namespace lt_dwa_official_wrapper {

struct WorkerResponse {
  bool valid{false};
  WrapperStatus status{WrapperStatus::kCoreProcessExited};
  std::string reason;
  bool has_command{false};
  double command_v{0.0};
  double command_w{0.0};
  bool has_raw_command{false};
  double raw_command_v{0.0};
  double raw_command_w{0.0};
  bool has_final_command{false};
  double final_command_v{0.0};
  double final_command_w{0.0};
  bool guard_applied{false};
  std::string guard_reason;
  bool has_core_return{false};
  int core_return{0};
};

std::string FormatWorkerResponse(WrapperStatus status, const std::string& reason);
std::string FormatWorkerResponse(WrapperStatus status,
                                 const std::string& reason,
                                 double command_v,
                                 double command_w,
                                 int core_return);
std::string FormatWorkerResponse(WrapperStatus status,
                                 const std::string& reason,
                                 double raw_command_v,
                                 double raw_command_w,
                                 double final_command_v,
                                 double final_command_w,
                                 bool guard_applied,
                                 const std::string& guard_reason,
                                 int core_return);
WorkerResponse ParseWorkerResponse(const std::string& text);

}  // namespace lt_dwa_official_wrapper
