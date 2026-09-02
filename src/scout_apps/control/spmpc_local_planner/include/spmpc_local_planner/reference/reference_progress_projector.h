#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/domain/physical_state.h"
#include "spmpc_local_planner/domain/release_contract.h"
#include "spmpc_local_planner/reference/reference_progress_contract.h"

namespace spmpc_local_planner {
namespace mainline {

// Reconstructs virtual progress from pose only.  It has no command, actual
// velocity, historical progress velocity, warm-start, or global-search input.
// Both entry points are atomic: output is assigned only after a unique local
// candidate passes every identity, authority, window, and geometric guard.
template <std::size_t VertexCapacity>
class ReferenceProgressProjector {
 public:
  using Path = ReferencePathSnapshot<VertexCapacity>;

  explicit ReferenceProgressProjector(
      const ReferenceProgressProjectorConfig& config)
      : config_(config) {
    if (!validConfig(config_)) {
      throw std::invalid_argument(
          "invalid reference progress projector configuration");
    }
  }

  const ReferenceProgressProjectorConfig& config() const noexcept {
    return config_;
  }

  ReferenceProgressStatus projectFrozenStart(
      const PlanarPoseState& pose, const CycleRequest& target_cycle,
      const ReferencePathIdentity& expected_identity,
      std::uint64_t expected_history_generation,
      const FrozenStartProgressAnchor& anchor, const Path& path,
      ReferenceProgressProjection& output) const noexcept {
    PathFacts facts;
    const ReferenceProgressStatus common_status =
        validateCommon(pose, target_cycle, expected_identity, path, facts);
    if (common_status != ReferenceProgressStatus::kOk) {
      return common_status;
    }
    const ReferenceProgressStatus authority_status = identityStatus(
        expected_identity, anchor.identity, target_cycle.reset_epoch);
    if (authority_status != ReferenceProgressStatus::kOk) {
      return authority_status;
    }
    if (anchor.source != ProgressAuthority::kFrozenStartInterval) {
      return ReferenceProgressStatus::kInvalidAuthority;
    }
    if (anchor.history_generation == 0 ||
        anchor.history_generation != expected_history_generation) {
      return ReferenceProgressStatus::kGenerationMismatch;
    }
    if (!sameCycle(anchor.target_cycle, target_cycle)) {
      return ReferenceProgressStatus::kTargetCycleMismatch;
    }
    if (config_.start_s_min < facts.path_start ||
        config_.start_s_max > facts.path_end) {
      return ReferenceProgressStatus::kInvalidPath;
    }
    return projectWindow(pose, path, facts, config_.start_s_min,
                         config_.start_s_max,
                         ProgressAuthority::kFrozenStartInterval,
                         anchor.target_cycle.cycle_id,
                         anchor.history_generation,
                         output);
  }

  ReferenceProgressStatus projectNominalCommit(
      const PlanarPoseState& pose, const CycleRequest& target_cycle,
      const ReferencePathIdentity& expected_identity,
      std::uint64_t expected_history_generation,
      const NominalProgressCommit& commit, const Path& path,
      ReferenceProgressProjection& output) const noexcept {
    PathFacts facts;
    const ReferenceProgressStatus common_status =
        validateCommon(pose, target_cycle, expected_identity, path, facts);
    if (common_status != ReferenceProgressStatus::kOk) {
      return common_status;
    }
    const ReferenceProgressStatus authority_status = identityStatus(
        expected_identity, commit.identity, target_cycle.reset_epoch);
    if (authority_status != ReferenceProgressStatus::kOk) {
      return authority_status;
    }
    if (commit.source != ProgressAuthority::kNominalLiveRelease) {
      return ReferenceProgressStatus::kInvalidAuthority;
    }
    if (commit.release_generation == 0 ||
        commit.release_generation != commit.history_generation ||
        commit.history_generation != expected_history_generation) {
      return ReferenceProgressStatus::kGenerationMismatch;
    }
    if (target_cycle.cycle_id == 0 ||
        commit.release_cycle.cycle_id ==
            std::numeric_limits<std::uint64_t>::max() ||
        commit.release_cycle.cycle_id + 1 != target_cycle.cycle_id ||
        commit.release_cycle.reset_epoch != target_cycle.reset_epoch) {
      return ReferenceProgressStatus::kTargetCycleMismatch;
    }
    if (!std::isfinite(commit.s_commit) ||
        commit.s_commit < facts.path_start ||
        commit.s_commit > facts.path_end) {
      return ReferenceProgressStatus::kInvalidAuthority;
    }
    if (target_cycle.release_model.value <
            commit.release_cycle.release_model.value ||
        target_cycle.release_steady.value <
            commit.release_cycle.release_steady.value) {
      return ReferenceProgressStatus::kTimeRegression;
    }

    std::int64_t duration_ns = 0;
    std::int64_t steady_duration_ns = 0;
    if (!checkedNonnegativeDifference(target_cycle.release_model.value,
                                      commit.release_cycle.release_model.value,
                                      duration_ns) ||
        !checkedNonnegativeDifference(target_cycle.release_steady.value,
                                      commit.release_cycle.release_steady.value,
                                      steady_duration_ns)) {
      return ReferenceProgressStatus::kTimeOverflow;
    }
    std::int64_t expected_duration_ns = 0;
    try {
      const std::int64_t target_offset =
          ReleaseGridContract::boundaryOffsetNs(target_cycle.cycle_id);
      const std::int64_t commit_offset =
          ReleaseGridContract::boundaryOffsetNs(
              commit.release_cycle.cycle_id);
      if (!checkedNonnegativeDifference(target_offset, commit_offset,
                                        expected_duration_ns)) {
        return ReferenceProgressStatus::kTimeOverflow;
      }
    } catch (const std::overflow_error&) {
      return ReferenceProgressStatus::kTimeOverflow;
    }
    if (duration_ns != expected_duration_ns ||
        steady_duration_ns != expected_duration_ns) {
      return ReferenceProgressStatus::kTargetCycleMismatch;
    }
    const double duration_sec =
        static_cast<double>(duration_ns) * kSecondsPerNanosecond;
    const double bounded_progress =
        config_.v_progress_bound * duration_sec;
    const double guarded_progress = bounded_progress + config_.forward_guard;
    const double proposed_upper = commit.s_commit + guarded_progress;
    if (!std::isfinite(duration_sec) || !std::isfinite(bounded_progress) ||
        !std::isfinite(guarded_progress) ||
        !std::isfinite(proposed_upper) ||
        proposed_upper < commit.s_commit) {
      return ReferenceProgressStatus::kProgressOverflow;
    }
    const double upper = std::min(facts.path_end, proposed_upper);
    if (upper < commit.s_commit) {
      return ReferenceProgressStatus::kProgressOverflow;
    }

    return projectWindow(pose, path, facts, commit.s_commit, upper,
                         ProgressAuthority::kNominalLiveRelease,
                         commit.release_cycle.cycle_id,
                         commit.history_generation,
                         output);
  }

 private:
  static constexpr double kSecondsPerNanosecond = 1e-9;
  static constexpr double kPi = 3.141592653589793238462643383279502884;

  struct PathFacts {
    double path_start{0.0};
    double path_end{0.0};
    std::size_t first_usable_segment{0};
    std::size_t last_usable_segment{0};
  };

  struct Candidate {
    double s{0.0};
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    double distance{0.0};
    double signed_contour_error{0.0};
    double heading_error{0.0};
    std::size_t segment{0};
  };

  static bool validConfig(
      const ReferenceProgressProjectorConfig& config) noexcept {
    if (!std::isfinite(config.start_s_min) ||
        !std::isfinite(config.start_s_max) ||
        !std::isfinite(config.v_progress_bound) ||
        !std::isfinite(config.forward_guard) ||
        !std::isfinite(config.contour_guard) ||
        !std::isfinite(config.heading_guard) ||
        !std::isfinite(config.ambiguity_tolerance) ||
        !std::isfinite(config.progress_equivalence_tolerance) ||
        !std::isfinite(config.minimum_segment_length)) {
      return false;
    }
    return config.start_s_min >= 0.0 &&
           config.start_s_max >= config.start_s_min &&
           config.v_progress_bound >= 0.0 && config.forward_guard >= 0.0 &&
           config.contour_guard >= 0.0 && config.heading_guard >= 0.0 &&
           config.heading_guard <= kPi &&
           config.ambiguity_tolerance >= 0.0 &&
           config.progress_equivalence_tolerance >= 0.0 &&
           config.minimum_segment_length > 0.0 &&
           config.progress_equivalence_tolerance <
               config.minimum_segment_length;
  }

  static bool finitePose(const PlanarPoseState& pose) noexcept {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.heading);
  }

  static bool checkedNonnegativeDifference(std::int64_t later,
                                           std::int64_t earlier,
                                           std::int64_t& output) noexcept {
    if (later < earlier ||
        (earlier < 0 &&
         later > std::numeric_limits<std::int64_t>::max() + earlier)) {
      return false;
    }
    output = later - earlier;
    return true;
  }

  static bool sameCycle(const CycleRequest& lhs,
                        const CycleRequest& rhs) noexcept {
    return lhs.cycle_id == rhs.cycle_id &&
           lhs.release_steady.value == rhs.release_steady.value &&
           lhs.release_model.value == rhs.release_model.value &&
           lhs.reset_epoch == rhs.reset_epoch;
  }

  static ReferenceProgressStatus identityStatus(
      const ReferencePathIdentity& expected,
      const ReferencePathIdentity& actual,
      std::uint64_t target_reset_epoch) noexcept {
    if (isZeroDigest(expected.path_hash) || isZeroDigest(actual.path_hash)) {
      return ReferenceProgressStatus::kInvalidIdentity;
    }
    if (expected.reset_epoch != target_reset_epoch ||
        actual.reset_epoch != target_reset_epoch) {
      return ReferenceProgressStatus::kEpochMismatch;
    }
    if (expected.path_id != actual.path_id ||
        expected.path_hash != actual.path_hash) {
      return ReferenceProgressStatus::kPathMismatch;
    }
    return ReferenceProgressStatus::kOk;
  }

  ReferenceProgressStatus validateCommon(
      const PlanarPoseState& pose, const CycleRequest& target_cycle,
      const ReferencePathIdentity& expected_identity, const Path& path,
      PathFacts& facts) const noexcept {
    if (!finitePose(pose)) {
      return ReferenceProgressStatus::kInvalidPose;
    }
    const ReferenceProgressStatus identity_status = identityStatus(
        expected_identity, path.identity, target_cycle.reset_epoch);
    if (identity_status != ReferenceProgressStatus::kOk) {
      return identity_status;
    }
    if (path.vertex_count < 2 || path.vertex_count > VertexCapacity ||
        !std::isfinite(path.s_path_end)) {
      return ReferenceProgressStatus::kInvalidPath;
    }

    bool found_usable = false;
    for (std::size_t index = 0; index < path.vertex_count; ++index) {
      const ReferencePathVertex& vertex = path.vertices[index];
      if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) ||
          !std::isfinite(vertex.cumulative_s) ||
          vertex.cumulative_s < 0.0) {
        return ReferenceProgressStatus::kInvalidPath;
      }
      if (index == 0) {
        continue;
      }
      const ReferencePathVertex& previous = path.vertices[index - 1];
      if (vertex.cumulative_s < previous.cumulative_s) {
        return ReferenceProgressStatus::kInvalidPath;
      }
      const double dx = vertex.x - previous.x;
      const double dy = vertex.y - previous.y;
      const double length = std::hypot(dx, dy);
      const double progress_length =
          vertex.cumulative_s - previous.cumulative_s;
      if (!std::isfinite(dx) || !std::isfinite(dy) ||
          !std::isfinite(length) || !std::isfinite(progress_length)) {
        return ReferenceProgressStatus::kInvalidPath;
      }
      if (length < config_.minimum_segment_length) {
        if (progress_length > config_.progress_equivalence_tolerance) {
          return ReferenceProgressStatus::kInvalidPath;
        }
        continue;
      }
      if (progress_length <= 0.0) {
        return ReferenceProgressStatus::kInvalidPath;
      }
      const double scale =
          std::max(1.0, std::max(length, progress_length));
      const double numeric_tolerance =
          64.0 * std::numeric_limits<double>::epsilon() * scale;
      const double arc_tolerance =
          std::max(config_.progress_equivalence_tolerance,
                   numeric_tolerance);
      if (std::fabs(length - progress_length) > arc_tolerance) {
        return ReferenceProgressStatus::kInvalidPath;
      }
      if (!found_usable) {
        facts.first_usable_segment = index - 1;
        found_usable = true;
      }
      facts.last_usable_segment = index - 1;
    }
    const ReferencePathVertex& first = path.vertices[0];
    const ReferencePathVertex& last = path.vertices[path.vertex_count - 1];
    if (!found_usable || path.s_path_end != last.cumulative_s ||
        path.s_path_end < first.cumulative_s) {
      return ReferenceProgressStatus::kInvalidPath;
    }
    facts.path_start = first.cumulative_s;
    facts.path_end = path.s_path_end;
    return ReferenceProgressStatus::kOk;
  }

  ReferenceProgressStatus projectWindow(
      const PlanarPoseState& pose, const Path& path, const PathFacts& facts,
      double lower, double upper, ProgressAuthority authority,
      std::uint64_t authority_cycle_id,
      std::uint64_t history_generation,
      ReferenceProgressProjection& output) const noexcept {
    if (!std::isfinite(lower) || !std::isfinite(upper) ||
        lower < facts.path_start || upper > facts.path_end || upper < lower) {
      return ReferenceProgressStatus::kProgressOverflow;
    }

    std::array<Candidate, VertexCapacity - 1> candidates{};
    std::size_t candidate_count = 0;
    bool saw_intersecting_segment = false;
    bool saw_backward = false;
    bool saw_forward = false;
    bool saw_contour_violation = false;
    bool saw_heading_violation = false;

    for (std::size_t index = 0; index + 1 < path.vertex_count; ++index) {
      const ReferencePathVertex& a = path.vertices[index];
      const ReferencePathVertex& b = path.vertices[index + 1];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length = std::hypot(dx, dy);
      if (length < config_.minimum_segment_length) {
        continue;
      }
      if (b.cumulative_s < lower || a.cumulative_s > upper) {
        continue;
      }
      saw_intersecting_segment = true;

      const double tangent_x = dx / length;
      const double tangent_y = dy / length;
      const double relative_x = pose.x - a.x;
      const double relative_y = pose.y - a.y;
      const double along =
          relative_x * tangent_x + relative_y * tangent_y;
      const double raw_t = along / length;
      const double progress_length = b.cumulative_s - a.cumulative_s;
      const double raw_s = a.cumulative_s + raw_t * progress_length;
      if (!std::isfinite(tangent_x) || !std::isfinite(tangent_y) ||
          !std::isfinite(relative_x) || !std::isfinite(relative_y) ||
          !std::isfinite(along) || !std::isfinite(raw_t) ||
          !std::isfinite(raw_s)) {
        return ReferenceProgressStatus::kProgressOverflow;
      }

      const bool true_start_extension =
          index == facts.first_usable_segment && lower == facts.path_start &&
          raw_t < 0.0;
      const bool true_end_extension =
          index == facts.last_usable_segment && upper == facts.path_end &&
          raw_t > 1.0;
      const double boundary_scale = std::max(
          1.0, std::max(std::fabs(raw_s),
                        std::max(std::fabs(lower), std::fabs(upper))));
      const double boundary_tolerance = std::max(
          config_.progress_equivalence_tolerance,
          64.0 * std::numeric_limits<double>::epsilon() * boundary_scale);
      if (raw_s < lower && lower - raw_s > boundary_tolerance &&
          !true_start_extension) {
        saw_backward = true;
        continue;
      }
      if (raw_s > upper && raw_s - upper > boundary_tolerance &&
          !true_end_extension) {
        saw_forward = true;
        continue;
      }

      double t = std::max(0.0, std::min(1.0, raw_t));
      double normalized_s = a.cumulative_s + t * progress_length;
      if (!true_start_extension && normalized_s < lower &&
          lower - normalized_s <= boundary_tolerance) {
        normalized_s = lower;
        t = (lower - a.cumulative_s) / progress_length;
      } else if (!true_end_extension && normalized_s > upper &&
                 normalized_s - upper <= boundary_tolerance) {
        normalized_s = upper;
        t = (upper - a.cumulative_s) / progress_length;
      }
      Candidate candidate;
      candidate.s = normalized_s;
      candidate.x = a.x + t * dx;
      candidate.y = a.y + t * dy;
      candidate.heading = std::atan2(dy, dx);
      const double error_x = pose.x - candidate.x;
      const double error_y = pose.y - candidate.y;
      candidate.distance = std::hypot(error_x, error_y);
      candidate.signed_contour_error =
          error_x * (-tangent_y) + error_y * tangent_x;
      candidate.heading_error =
          std::atan2(std::sin(pose.heading - candidate.heading),
                     std::cos(pose.heading - candidate.heading));
      candidate.segment = index;
      if (!std::isfinite(candidate.s) || !std::isfinite(candidate.x) ||
          !std::isfinite(candidate.y) ||
          !std::isfinite(candidate.heading) ||
          !std::isfinite(candidate.distance) ||
          !std::isfinite(candidate.signed_contour_error) ||
          !std::isfinite(candidate.heading_error)) {
        return ReferenceProgressStatus::kProgressOverflow;
      }
      if (candidate.s < lower || candidate.s > upper) {
        return ReferenceProgressStatus::kProgressOverflow;
      }
      // Interior contour error is the signed normal distance.  Once the
      // nearest point is a polyline endpoint, Euclidean distance is required
      // so that a far tangent extension cannot masquerade as zero contour
      // error and silently clamp to the endpoint.
      const double guarded_contour_error =
          raw_t < 0.0 || raw_t > 1.0
              ? candidate.distance
              : std::fabs(candidate.signed_contour_error);
      if (guarded_contour_error > config_.contour_guard) {
        saw_contour_violation = true;
        continue;
      }
      if (std::fabs(candidate.heading_error) > config_.heading_guard) {
        saw_heading_violation = true;
        continue;
      }
      if (candidate_count >= candidates.size()) {
        return ReferenceProgressStatus::kProgressOverflow;
      }
      candidates[candidate_count++] = candidate;
    }

    if (candidate_count == 0) {
      if (authority == ProgressAuthority::kFrozenStartInterval &&
          (saw_backward || saw_forward)) {
        return ReferenceProgressStatus::kStartWindowViolation;
      }
      if (saw_backward && !saw_forward) {
        return ReferenceProgressStatus::kBackwardProgress;
      }
      if (saw_forward && !saw_backward) {
        return ReferenceProgressStatus::kForwardWindowViolation;
      }
      if (saw_contour_violation) {
        return ReferenceProgressStatus::kContourGuardViolation;
      }
      if (saw_heading_violation) {
        return ReferenceProgressStatus::kHeadingGuardViolation;
      }
      return saw_intersecting_segment ? ReferenceProgressStatus::kNoCandidate
                                      : ReferenceProgressStatus::kInvalidPath;
    }

    Candidate best_candidate = candidates[0];
    for (std::size_t index = 1; index < candidate_count; ++index) {
      if (candidateLess(candidates[index], best_candidate)) {
        best_candidate = candidates[index];
      }
    }
    for (std::size_t index = 0; index < candidate_count; ++index) {
      if (equivalentProgress(candidates[index], best_candidate, path)) {
        continue;
      }
      if (std::fabs(candidates[index].distance -
                    best_candidate.distance) <=
          config_.ambiguity_tolerance) {
        return ReferenceProgressStatus::kAmbiguous;
      }
    }

    for (std::size_t index = 0; index < candidate_count; ++index) {
      if (!equivalentProgress(candidates[index], best_candidate, path)) {
        continue;
      }
      if (equivalentCandidateLess(candidates[index], best_candidate)) {
        best_candidate = candidates[index];
      }
    }

    const Candidate& selected = best_candidate;
    ReferenceProgressProjection next;
    next.s = selected.s;
    next.projected_x = selected.x;
    next.projected_y = selected.y;
    next.projected_heading = selected.heading;
    next.distance = selected.distance;
    next.signed_contour_error = selected.signed_contour_error;
    next.heading_error = selected.heading_error;
    next.search_lower = lower;
    next.search_upper = upper;
    next.selected_segment = selected.segment;
    next.accepted_candidate_count = candidate_count;
    next.identity = path.identity;
    next.authority = authority;
    next.authority_cycle_id = authority_cycle_id;
    next.history_generation = history_generation;
    output = next;
    return ReferenceProgressStatus::kOk;
  }

  static bool candidateLess(const Candidate& lhs,
                            const Candidate& rhs) noexcept {
    if (lhs.distance != rhs.distance) {
      return lhs.distance < rhs.distance;
    }
    const double lhs_heading = std::fabs(lhs.heading_error);
    const double rhs_heading = std::fabs(rhs.heading_error);
    if (lhs_heading != rhs_heading) {
      return lhs_heading < rhs_heading;
    }
    return lhs.segment < rhs.segment;
  }

  bool equivalentProgress(const Candidate& lhs, const Candidate& rhs,
                          const Path& path) const noexcept {
    const double position_distance = std::hypot(lhs.x - rhs.x,
                                                lhs.y - rhs.y);
    if (!std::isfinite(position_distance) ||
        std::fabs(lhs.s - rhs.s) >
            config_.progress_equivalence_tolerance ||
        position_distance > config_.progress_equivalence_tolerance) {
      return false;
    }

    // Equivalent candidates may only be duplicate representations of one
    // topological vertex.  Every vertex between the two segments must be that
    // same point/progress; a geometrically crossing but non-adjacent branch is
    // never equivalent, even if a bad configuration approaches the allowed
    // tolerance bound.
    const std::size_t first_segment = std::min(lhs.segment, rhs.segment);
    const std::size_t last_segment = std::max(lhs.segment, rhs.segment);
    for (std::size_t vertex_index = first_segment + 1;
         vertex_index <= last_segment; ++vertex_index) {
      if (vertex_index >= path.vertex_count) {
        return false;
      }
      const ReferencePathVertex& vertex = path.vertices[vertex_index];
      const double vertex_distance =
          std::hypot(vertex.x - lhs.x, vertex.y - lhs.y);
      if (!std::isfinite(vertex_distance) ||
          vertex_distance > config_.progress_equivalence_tolerance ||
          std::fabs(vertex.cumulative_s - lhs.s) >
              config_.progress_equivalence_tolerance) {
        return false;
      }
    }
    return true;
  }

  static bool equivalentCandidateLess(const Candidate& lhs,
                                      const Candidate& rhs) noexcept {
    const double lhs_heading = std::fabs(lhs.heading_error);
    const double rhs_heading = std::fabs(rhs.heading_error);
    if (lhs_heading != rhs_heading) {
      return lhs_heading < rhs_heading;
    }
    if (lhs.distance != rhs.distance) {
      return lhs.distance < rhs.distance;
    }
    return lhs.segment < rhs.segment;
  }

  const ReferenceProgressProjectorConfig config_;
};

}  // namespace mainline
}  // namespace spmpc_local_planner
