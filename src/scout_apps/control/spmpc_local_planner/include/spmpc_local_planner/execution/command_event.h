#pragma once

#include <cstdint>

#include "spmpc_local_planner/domain/mainline_types.h"
#include "spmpc_local_planner/domain/solver_io.h"

namespace spmpc_local_planner {
namespace mainline {

struct PlanarCommand {
  double linear{0.0};
  double angular{0.0};
};

struct PlanarCommandAcceleration {
  double linear{0.0};
  double angular{0.0};
};

enum class EmissionReason : std::uint8_t {
  kNominal = 0,
  kWarmupZero = 1,
  // Values 2 through 5 belonged to retired zero/override reasons.  They stay
  // unused so persisted numeric audit records cannot be reinterpreted as a
  // new hard stop or smooth fallback by a newer binary.
  kClockFaultStop = 6,
  kHardSafetyStop = 7,
  kDeadlineFallback = 8,
  kSolverFailureFallback = 9,
  kPublishTimingFallback = 10,
  kInputStaleFallback = 11,
};

enum class PublicationStatus : std::uint8_t {
  kPublished = 0,
  kWouldPublish,
  kFailed,
};

// Created by the eventual FinalCommandPublisher boundary.  Only kPublished is
// eligible to become a live command-history fact; dry-run and failed calls are
// deliberately representable but cannot advance the history generation.
struct PublicationReceipt {
  PublicationReceipt() : actual_steady(0), actual_model(0) {}

  PublicationStatus status{PublicationStatus::kFailed};
  CycleRequest cycle;
  PlanarCommand command;
  SteadyTimeNs actual_steady;
  ModelTimeNs actual_model;
};

// Release-owner input for committing a command that has already crossed the
// unique publisher boundary.  expected_history_generation ties the release to
// the exact authoritative snapshot on which its final decision was based.
struct EmittedCommandCommit {
  CycleRequest cycle;
  std::uint64_t expected_history_generation{0};
  PublicationReceipt receipt;
  PlanarCommand command;
  PlanarCommandAcceleration authoritative_acceleration;
  EmissionReason reason{EmissionReason::kNominal};
};

struct PublishedCommandEvent {
  PublishedCommandEvent() : actual_steady(0), actual_model(0) {}

  CycleRequest cycle;
  SteadyTimeNs actual_steady;
  ModelTimeNs actual_model;
  PlanarCommand command;
  AuthoritativePublisherState publisher_state_after;
  EmissionReason reason{EmissionReason::kNominal};
  std::uint64_t release_generation{0};
  bool publish_late{false};
};

inline bool isKnownEmissionReason(EmissionReason reason) {
  switch (reason) {
    case EmissionReason::kNominal:
    case EmissionReason::kWarmupZero:
    case EmissionReason::kDeadlineFallback:
    case EmissionReason::kSolverFailureFallback:
    case EmissionReason::kHardSafetyStop:
    case EmissionReason::kPublishTimingFallback:
    case EmissionReason::kClockFaultStop:
    case EmissionReason::kInputStaleFallback:
      return true;
  }
  return false;
}

inline bool isSmoothFallbackReason(EmissionReason reason) {
  switch (reason) {
    case EmissionReason::kDeadlineFallback:
    case EmissionReason::kSolverFailureFallback:
    case EmissionReason::kPublishTimingFallback:
    case EmissionReason::kInputStaleFallback:
      return true;
    case EmissionReason::kNominal:
    case EmissionReason::kWarmupZero:
    case EmissionReason::kHardSafetyStop:
    case EmissionReason::kClockFaultStop:
      return false;
  }
  return false;
}

inline bool requiresZeroCommand(EmissionReason reason) {
  switch (reason) {
    case EmissionReason::kWarmupZero:
    case EmissionReason::kHardSafetyStop:
    case EmissionReason::kClockFaultStop:
      return true;
    case EmissionReason::kNominal:
    case EmissionReason::kDeadlineFallback:
    case EmissionReason::kSolverFailureFallback:
    case EmissionReason::kPublishTimingFallback:
    case EmissionReason::kInputStaleFallback:
      return false;
  }
  return false;
}

inline bool requiresZeroPublisherAcceleration(EmissionReason reason) {
  return requiresZeroCommand(reason);
}

}  // namespace mainline
}  // namespace spmpc_local_planner
