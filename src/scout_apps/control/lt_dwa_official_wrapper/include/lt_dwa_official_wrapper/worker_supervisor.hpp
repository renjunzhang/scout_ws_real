#pragma once

#include <string>
#include <vector>

#include "lt_dwa_official_wrapper/status.hpp"
#include "lt_dwa_official_wrapper/worker_protocol.hpp"

namespace lt_dwa_official_wrapper {

struct WorkerRunResult {
  WrapperStatus status{WrapperStatus::kCoreProcessExited};
  std::string reason;
  std::string output;
  int exit_code{-1};
  int term_signal{0};
  bool timed_out{false};
  bool valid_response{false};
  bool has_command{false};
  double command_v{0.0};
  double command_w{0.0};
  bool has_core_return{false};
  int core_return{0};
};

class WorkerSupervisor {
 public:
  WorkerRunResult Run(const std::string& executable,
                      const std::vector<std::string>& args,
                      double timeout_sec) const;
};

}  // namespace lt_dwa_official_wrapper
