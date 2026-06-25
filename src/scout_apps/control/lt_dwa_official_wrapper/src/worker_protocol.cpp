#include "lt_dwa_official_wrapper/worker_protocol.hpp"

#include <sstream>
#include <string>

namespace lt_dwa_official_wrapper {
namespace {

WrapperStatus StatusFromString(const std::string& status) {
  if (status == "OK") {
    return WrapperStatus::kOk;
  }
  if (status == "GOAL_REACHED") {
    return WrapperStatus::kGoalReached;
  }
  if (status == "WAITING_FOR_INPUT") {
    return WrapperStatus::kWaitingForInput;
  }
  if (status == "INVALID_FRAME") {
    return WrapperStatus::kInvalidFrame;
  }
  if (status == "STALE_INPUT") {
    return WrapperStatus::kStaleInput;
  }
  if (status == "EMPTY_PATH") {
    return WrapperStatus::kEmptyPath;
  }
  if (status == "DEGENERATE_PATH") {
    return WrapperStatus::kDegeneratePath;
  }
  if (status == "INVALID_MAP") {
    return WrapperStatus::kInvalidMap;
  }
  if (status == "INVALID_OBSTACLE") {
    return WrapperStatus::kInvalidObstacle;
  }
  if (status == "CORE_PLANNING_FAILED") {
    return WrapperStatus::kCorePlanningFailed;
  }
  if (status == "CORE_PROCESS_EXITED") {
    return WrapperStatus::kCoreProcessExited;
  }
  if (status == "CORE_EXCEPTION") {
    return WrapperStatus::kCoreException;
  }
  if (status == "COMMAND_REJECTED") {
    return WrapperStatus::kCommandRejected;
  }
  return WrapperStatus::kCoreProcessExited;
}

std::string EscapeReason(std::string reason) {
  for (auto& ch : reason) {
    if (ch == '\n' || ch == '\r' || ch == '\t') {
      ch = ' ';
    } else if (ch == ' ') {
      ch = '_';
    }
  }
  return reason;
}

bool ParseBool(const std::string& value) {
  return value == "1" || value == "true" || value == "True" || value == "TRUE";
}

}  // namespace

std::string FormatWorkerResponse(WrapperStatus status, const std::string& reason) {
  std::ostringstream oss;
  oss << "LT_DWA_WORKER_RESULT status=" << ToString(status)
      << " reason=" << EscapeReason(reason) << "\n";
  return oss.str();
}

std::string FormatWorkerResponse(WrapperStatus status,
                                 const std::string& reason,
                                 double command_v,
                                 double command_w,
                                 int core_return) {
  std::ostringstream oss;
  oss << "LT_DWA_WORKER_RESULT status=" << ToString(status)
      << " reason=" << EscapeReason(reason)
      << " command_v=" << command_v
      << " command_w=" << command_w
      << " core_return=" << core_return << "\n";
  return oss.str();
}

std::string FormatWorkerResponse(WrapperStatus status,
                                 const std::string& reason,
                                 double raw_command_v,
                                 double raw_command_w,
                                 double final_command_v,
                                 double final_command_w,
                                 bool guard_applied,
                                 const std::string& guard_reason,
                                 int core_return) {
  std::ostringstream oss;
  oss << "LT_DWA_WORKER_RESULT status=" << ToString(status)
      << " reason=" << EscapeReason(reason)
      << " raw_command_v=" << raw_command_v
      << " raw_command_w=" << raw_command_w
      << " final_command_v=" << final_command_v
      << " final_command_w=" << final_command_w
      << " guard_applied=" << (guard_applied ? 1 : 0)
      << " guard_reason=" << EscapeReason(guard_reason)
      << " core_return=" << core_return << "\n";
  return oss.str();
}

WorkerResponse ParseWorkerResponse(const std::string& text) {
  WorkerResponse response;
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    if (line.find("LT_DWA_WORKER_RESULT") != 0) {
      continue;
    }

    std::istringstream tokens(line);
    std::string token;
    tokens >> token;
    std::string status_value;
    std::string reason_value;
    bool saw_command_v = false;
    bool saw_command_w = false;
    bool saw_raw_command_v = false;
    bool saw_raw_command_w = false;
    bool saw_final_command_v = false;
    bool saw_final_command_w = false;
    bool saw_core_return = false;
    while (tokens >> token) {
      const auto sep = token.find('=');
      if (sep == std::string::npos) {
        continue;
      }
      const std::string key = token.substr(0, sep);
      const std::string value = token.substr(sep + 1);
      if (key == "status") {
        status_value = value;
      } else if (key == "reason") {
        reason_value = value;
      } else if (key == "command_v") {
        try {
          response.command_v = std::stod(value);
          saw_command_v = true;
        } catch (...) {
        }
      } else if (key == "command_w") {
        try {
          response.command_w = std::stod(value);
          saw_command_w = true;
        } catch (...) {
        }
      } else if (key == "raw_command_v") {
        try {
          response.raw_command_v = std::stod(value);
          saw_raw_command_v = true;
        } catch (...) {
        }
      } else if (key == "raw_command_w") {
        try {
          response.raw_command_w = std::stod(value);
          saw_raw_command_w = true;
        } catch (...) {
        }
      } else if (key == "final_command_v") {
        try {
          response.final_command_v = std::stod(value);
          saw_final_command_v = true;
        } catch (...) {
        }
      } else if (key == "final_command_w") {
        try {
          response.final_command_w = std::stod(value);
          saw_final_command_w = true;
        } catch (...) {
        }
      } else if (key == "guard_applied") {
        response.guard_applied = ParseBool(value);
      } else if (key == "guard_reason") {
        response.guard_reason = value;
      } else if (key == "core_return") {
        try {
          response.core_return = std::stoi(value);
          saw_core_return = true;
        } catch (...) {
        }
      }
    }

    if (!status_value.empty()) {
      response.valid = true;
      response.status = StatusFromString(status_value);
      response.reason = reason_value;
      response.has_command = saw_command_v && saw_command_w;
      response.has_raw_command = saw_raw_command_v && saw_raw_command_w;
      response.has_final_command = saw_final_command_v && saw_final_command_w;
      if (response.has_final_command) {
        response.command_v = response.final_command_v;
        response.command_w = response.final_command_w;
        response.has_command = true;
      } else if (response.has_command) {
        response.final_command_v = response.command_v;
        response.final_command_w = response.command_w;
        response.has_final_command = true;
      }
      response.has_core_return = saw_core_return;
      return response;
    }
  }
  response.valid = false;
  response.status = WrapperStatus::kCoreProcessExited;
  response.reason = "worker exited without structured response";
  return response;
}

}  // namespace lt_dwa_official_wrapper
