#pragma once

#include <cstddef>
#include <cstdint>

#include "spmpc_local_planner/domain/mainline_types.h"
#include "spmpc_local_planner/reference/reference_path_snapshot.h"

namespace spmpc_local_planner {
namespace mainline {

struct ReferenceProgressProjectorConfig {
  double start_s_min{0.0};
  double start_s_max{0.0};
  double v_progress_bound{0.0};
  double forward_guard{0.0};
  double contour_guard{0.0};
  double heading_guard{0.0};
  // Metres of Euclidean candidate-distance difference.  This is deliberately
  // not a squared-distance tolerance.
  double ambiguity_tolerance{0.0};
  double progress_equivalence_tolerance{0.0};
  double minimum_segment_length{0.0};
};

enum class ProgressAuthority : std::uint8_t {
  kNone = 0,
  kFrozenStartInterval,
  kNominalLiveRelease,
};

struct FrozenStartProgressAnchor {
  ProgressAuthority source{ProgressAuthority::kNone};
  ReferencePathIdentity identity;
  CycleRequest target_cycle;
  std::uint64_t history_generation{0};
};

struct NominalProgressCommit {
  ProgressAuthority source{ProgressAuthority::kNone};
  double s_commit{0.0};
  ReferencePathIdentity identity;
  CycleRequest release_cycle;
  std::uint64_t release_generation{0};
  std::uint64_t history_generation{0};
};

enum class ReferenceProgressStatus : std::uint8_t {
  kOk = 0,
  kInvalidPose,
  kInvalidPath,
  kInvalidIdentity,
  kPathMismatch,
  kEpochMismatch,
  kInvalidAuthority,
  kGenerationMismatch,
  kTargetCycleMismatch,
  kTimeRegression,
  kTimeOverflow,
  kProgressOverflow,
  kStartWindowViolation,
  kBackwardProgress,
  kForwardWindowViolation,
  kContourGuardViolation,
  kHeadingGuardViolation,
  kNoCandidate,
  kAmbiguous,
};

struct ReferenceProgressProjection {
  double s{0.0};
  double projected_x{0.0};
  double projected_y{0.0};
  double projected_heading{0.0};
  double distance{0.0};
  double signed_contour_error{0.0};
  double heading_error{0.0};
  double search_lower{0.0};
  double search_upper{0.0};
  std::size_t selected_segment{0};
  std::size_t accepted_candidate_count{0};
  ReferencePathIdentity identity;
  ProgressAuthority authority{ProgressAuthority::kNone};
  std::uint64_t authority_cycle_id{0};
  std::uint64_t history_generation{0};
};

}  // namespace mainline
}  // namespace spmpc_local_planner
