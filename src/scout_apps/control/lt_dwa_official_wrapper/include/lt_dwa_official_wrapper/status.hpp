#pragma once

#include <string>

namespace lt_dwa_official_wrapper {

enum class WrapperStatus {
  kOk,
  kGoalReached,
  kWaitingForInput,
  kInvalidFrame,
  kStaleInput,
  kEmptyPath,
  kDegeneratePath,
  kInvalidMap,
  kInvalidObstacle,
  kCorePlanningFailed,
  kCoreProcessExited,
  kCoreException,
  kCommandRejected,
};

inline const char* ToString(WrapperStatus status) {
  switch (status) {
    case WrapperStatus::kOk:
      return "OK";
    case WrapperStatus::kGoalReached:
      return "GOAL_REACHED";
    case WrapperStatus::kWaitingForInput:
      return "WAITING_FOR_INPUT";
    case WrapperStatus::kInvalidFrame:
      return "INVALID_FRAME";
    case WrapperStatus::kStaleInput:
      return "STALE_INPUT";
    case WrapperStatus::kEmptyPath:
      return "EMPTY_PATH";
    case WrapperStatus::kDegeneratePath:
      return "DEGENERATE_PATH";
    case WrapperStatus::kInvalidMap:
      return "INVALID_MAP";
    case WrapperStatus::kInvalidObstacle:
      return "INVALID_OBSTACLE";
    case WrapperStatus::kCorePlanningFailed:
      return "CORE_PLANNING_FAILED";
    case WrapperStatus::kCoreProcessExited:
      return "CORE_PROCESS_EXITED";
    case WrapperStatus::kCoreException:
      return "CORE_EXCEPTION";
    case WrapperStatus::kCommandRejected:
      return "COMMAND_REJECTED";
  }
  return "UNKNOWN";
}

inline bool IsFailure(WrapperStatus status) {
  return status != WrapperStatus::kOk && status != WrapperStatus::kGoalReached;
}

}  // namespace lt_dwa_official_wrapper
