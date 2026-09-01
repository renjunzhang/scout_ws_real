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
  kWarmupZero,
  kDeadlineZero,
  kSolverFailureZero,
  kSafetyOverride,
  kPublishJitterZero,
  kClockFaultZero,
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
    case EmissionReason::kDeadlineZero:
    case EmissionReason::kSolverFailureZero:
    case EmissionReason::kSafetyOverride:
    case EmissionReason::kPublishJitterZero:
    case EmissionReason::kClockFaultZero:
      return true;
  }
  return false;
}

inline bool resetsPublisherAcceleration(EmissionReason reason) {
  return isKnownEmissionReason(reason) && reason != EmissionReason::kNominal;
}

inline bool requiresZeroCommand(EmissionReason reason) {
  switch (reason) {
    case EmissionReason::kWarmupZero:
    case EmissionReason::kDeadlineZero:
    case EmissionReason::kSolverFailureZero:
    case EmissionReason::kPublishJitterZero:
    case EmissionReason::kClockFaultZero:
      return true;
    case EmissionReason::kNominal:
    case EmissionReason::kSafetyOverride:
      return false;
  }
  return false;
}

}  // namespace mainline
}  // namespace spmpc_local_planner
