#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/reference/reference_progress_projector.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr std::size_t kVertexCapacity = 8;
constexpr std::uint64_t kResetEpoch = 9;
constexpr std::uint64_t kPathId = 17;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTolerance = 1e-12;
using Projector = ReferenceProgressProjector<kVertexCapacity>;
using Path = Projector::Path;

ReferencePathIdentity identity(std::uint64_t reset_epoch = kResetEpoch) {
  ReferencePathIdentity result;
  result.path_id = kPathId;
  result.path_hash.bytes[0] = 0x42;
  result.path_hash.bytes[31] = 0xa7;
  result.reset_epoch = reset_epoch;
  return result;
}

ReferenceProgressProjectorConfig makeConfig() {
  ReferenceProgressProjectorConfig config;
  config.start_s_min = 0.0;
  config.start_s_max = 1.7;
  // The release grid advances one cycle by 1/30 s.  Use a synthetic 30 m/s
  // bound so the golden nominal window still advances by one metre before its
  // explicit forward guard.
  config.v_progress_bound = 30.0;
  config.forward_guard = 0.2;
  config.contour_guard = 0.25;
  config.heading_guard = 0.20;
  config.ambiguity_tolerance = 1e-3;
  config.progress_equivalence_tolerance = 1e-9;
  config.minimum_segment_length = 1e-6;
  return config;
}

Path straightPath() {
  Path path;
  path.identity = identity();
  path.vertices[0] = ReferencePathVertex{0.0, 0.0, 0.0};
  path.vertices[1] = ReferencePathVertex{10.0, 0.0, 10.0};
  path.vertex_count = 2;
  path.s_path_end = 10.0;
  return path;
}

Path cornerPath() {
  Path path;
  path.identity = identity();
  path.vertices[0] = ReferencePathVertex{0.0, 0.0, 0.0};
  path.vertices[1] = ReferencePathVertex{2.0, 0.0, 2.0};
  path.vertices[2] = ReferencePathVertex{2.0, 2.0, 4.0};
  path.vertices[3] = ReferencePathVertex{4.0, 2.0, 6.0};
  path.vertex_count = 4;
  path.s_path_end = 6.0;
  return path;
}

Path selfIntersectingPath() {
  const double diagonal = std::sqrt(8.0);
  Path path;
  path.identity = identity();
  path.vertices[0] = ReferencePathVertex{0.0, 0.0, 0.0};
  path.vertices[1] = ReferencePathVertex{2.0, 2.0, diagonal};
  path.vertices[2] = ReferencePathVertex{0.0, 2.0, diagonal + 2.0};
  path.vertices[3] =
      ReferencePathVertex{2.0, 0.0, 2.0 * diagonal + 2.0};
  path.vertex_count = 4;
  path.s_path_end = 2.0 * diagonal + 2.0;
  return path;
}

Path zeroLengthDuplicatePath() {
  Path path;
  path.identity = identity();
  path.vertices[0] = ReferencePathVertex{0.0, 0.0, 0.0};
  path.vertices[1] = ReferencePathVertex{0.0, 0.0, 0.0};
  path.vertices[2] = ReferencePathVertex{2.0, 0.0, 2.0};
  path.vertices[3] = ReferencePathVertex{4.0, 0.0, 4.0};
  path.vertex_count = 4;
  path.s_path_end = 4.0;
  return path;
}

ClockAnchor testAnchor() {
  return ClockAnchor{SteadyTimeNs(1000000000LL),
                     ModelTimeNs(2000000000LL)};
}

CycleRequest cycleAt(std::uint64_t cycle_id,
                     std::uint64_t reset_epoch = kResetEpoch) {
  CycleRequest result;
  result.cycle_id = cycle_id;
  result.release_steady =
      ReleaseGridContract::boundary(testAnchor().steady, cycle_id);
  result.release_model =
      mapSteadyToModel(testAnchor(), result.release_steady);
  result.reset_epoch = reset_epoch;
  return result;
}

CycleRequest targetCycle(std::uint64_t cycle_id = 1,
                         std::uint64_t reset_epoch = kResetEpoch) {
  return cycleAt(cycle_id, reset_epoch);
}

NominalProgressCommit nominalCommit(
    const ReferencePathIdentity& path_identity = identity(),
    double s_commit = 0.5, std::uint64_t cycle_id = 0,
    std::uint64_t generation = 7) {
  NominalProgressCommit result;
  result.source = ProgressAuthority::kNominalLiveRelease;
  result.s_commit = s_commit;
  result.identity = path_identity;
  result.release_cycle = cycleAt(cycle_id);
  result.release_generation = generation;
  result.history_generation = generation;
  return result;
}

FrozenStartProgressAnchor frozenAnchor(
    const ReferencePathIdentity& path_identity = identity(),
    std::uint64_t target_cycle_id = 1,
    std::uint64_t generation = 7) {
  FrozenStartProgressAnchor result;
  result.source = ProgressAuthority::kFrozenStartInterval;
  result.identity = path_identity;
  result.target_cycle = cycleAt(target_cycle_id);
  result.history_generation = generation;
  return result;
}

PlanarPoseState pose(double x, double y, double heading) {
  return PlanarPoseState{x, y, heading};
}

void expectProjectionEqual(const ReferenceProgressProjection& expected,
                           const ReferenceProgressProjection& actual) {
  EXPECT_DOUBLE_EQ(expected.s, actual.s);
  EXPECT_DOUBLE_EQ(expected.projected_x, actual.projected_x);
  EXPECT_DOUBLE_EQ(expected.projected_y, actual.projected_y);
  EXPECT_DOUBLE_EQ(expected.projected_heading, actual.projected_heading);
  EXPECT_DOUBLE_EQ(expected.distance, actual.distance);
  EXPECT_DOUBLE_EQ(expected.signed_contour_error,
                   actual.signed_contour_error);
  EXPECT_DOUBLE_EQ(expected.heading_error, actual.heading_error);
  EXPECT_DOUBLE_EQ(expected.search_lower, actual.search_lower);
  EXPECT_DOUBLE_EQ(expected.search_upper, actual.search_upper);
  EXPECT_EQ(expected.selected_segment, actual.selected_segment);
  EXPECT_EQ(expected.accepted_candidate_count,
            actual.accepted_candidate_count);
  EXPECT_EQ(expected.identity, actual.identity);
  EXPECT_EQ(expected.authority, actual.authority);
  EXPECT_EQ(expected.authority_cycle_id, actual.authority_cycle_id);
  EXPECT_EQ(expected.history_generation, actual.history_generation);
}

ReferenceProgressProjection sentinel() {
  ReferenceProgressProjection result;
  result.s = 101.0;
  result.projected_x = 102.0;
  result.projected_y = 103.0;
  result.projected_heading = 104.0;
  result.distance = 105.0;
  result.signed_contour_error = 106.0;
  result.heading_error = 107.0;
  result.search_lower = 108.0;
  result.search_upper = 109.0;
  result.selected_segment = 110;
  result.accepted_candidate_count = 111;
  result.identity = identity(12);
  result.authority = ProgressAuthority::kNominalLiveRelease;
  result.authority_cycle_id = 112;
  result.history_generation = 113;
  return result;
}

template <typename Callback>
void expectFailureAtomic(ReferenceProgressStatus expected_status,
                         Callback&& callback) {
  ReferenceProgressProjection output = sentinel();
  const ReferenceProgressProjection before = output;
  EXPECT_EQ(expected_status, callback(output));
  expectProjectionEqual(before, output);
}

TEST(MainlineReferenceProgressProjector, ProjectsStraightLineInsideNominalWindow) {
  const Projector projector(makeConfig());
  const Path path = straightPath();
  const ReferencePathIdentity expected = identity();
  const NominalProgressCommit commit = nominalCommit();
  ReferenceProgressProjection output;

  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(1.2, 0.1, 0.05), targetCycle(), expected, 7, commit,
                path, output));
  EXPECT_DOUBLE_EQ(1.2, output.s);
  EXPECT_DOUBLE_EQ(1.2, output.projected_x);
  EXPECT_DOUBLE_EQ(0.0, output.projected_y);
  EXPECT_DOUBLE_EQ(0.0, output.projected_heading);
  EXPECT_DOUBLE_EQ(0.1, output.distance);
  EXPECT_DOUBLE_EQ(0.1, output.signed_contour_error);
  EXPECT_DOUBLE_EQ(0.05, output.heading_error);
  EXPECT_DOUBLE_EQ(0.5, output.search_lower);
  // The rounded 30 Hz grid boundary is 33,333,333 ns, so the computed upper
  // bound is 1.69999999 rather than the decimal ideal 1.7.
  EXPECT_NEAR(1.7, output.search_upper, 1e-7);
  EXPECT_EQ(0u, output.selected_segment);
  EXPECT_EQ(1u, output.accepted_candidate_count);
  EXPECT_EQ(expected, output.identity);
  EXPECT_EQ(ProgressAuthority::kNominalLiveRelease, output.authority);
  EXPECT_EQ(0u, output.authority_cycle_id);
  EXPECT_EQ(7u, output.history_generation);
}

TEST(MainlineReferenceProgressProjector,
     DoesNotClampArtificialNominalWindowBoundaries) {
  const Projector projector(makeConfig());
  const Path path = straightPath();
  const ReferencePathIdentity expected = identity();
  const NominalProgressCommit commit = nominalCommit();
  ReferenceProgressProjection output;

  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(0.5, 0.0, 0.0), targetCycle(), expected, 7, commit,
                path, output));
  EXPECT_DOUBLE_EQ(0.5, output.s);
  const double exact_upper = output.search_upper;
  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(exact_upper, 0.0, 0.0), targetCycle(), expected, 7,
                commit, path, output));
  EXPECT_DOUBLE_EQ(exact_upper, output.s);

  expectFailureAtomic(
      ReferenceProgressStatus::kBackwardProgress, [&](auto& result) {
        return projector.projectNominalCommit(
            pose(0.4, 0.0, 0.0), targetCycle(), expected, 7, commit, path,
            result);
      });
  expectFailureAtomic(
      ReferenceProgressStatus::kForwardWindowViolation, [&](auto& result) {
        return projector.projectNominalCommit(
            pose(1.8, 0.0, 0.0), targetCycle(), expected, 7, commit, path,
            result);
      });
}

TEST(MainlineReferenceProgressProjector,
     AcceptsRealPathEndpointOnlyWhenEndpointGuardsPass) {
  const Projector projector(makeConfig());
  const Path path = straightPath();
  const ReferencePathIdentity expected = identity();
  const NominalProgressCommit commit = nominalCommit(identity(), 9.5);
  ReferenceProgressProjection output;

  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(10.2, 0.0, 0.0), targetCycle(), expected, 7, commit,
                path, output));
  EXPECT_DOUBLE_EQ(10.0, output.s);
  EXPECT_DOUBLE_EQ(10.0, output.projected_x);
  EXPECT_NEAR(0.2, output.distance, kTolerance);
  EXPECT_DOUBLE_EQ(10.0, output.search_upper);

  expectFailureAtomic(
      ReferenceProgressStatus::kContourGuardViolation, [&](auto& result) {
        return projector.projectNominalCommit(
            pose(10.3, 0.0, 0.0), targetCycle(), expected, 7, commit, path,
            result);
      });
}

TEST(MainlineReferenceProgressProjector, EnforcesContourAndWrappedHeadingGuards) {
  const Projector projector(makeConfig());
  const Path path = straightPath();
  const ReferencePathIdentity expected = identity();
  const NominalProgressCommit commit = nominalCommit();

  ReferenceProgressProjection output;
  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(1.0, 0.1, -2.0 * kPi + 0.05), targetCycle(), expected,
                7, commit, path, output));
  EXPECT_NEAR(0.05, output.heading_error, kTolerance);
  EXPECT_DOUBLE_EQ(0.1, output.signed_contour_error);

  expectFailureAtomic(
      ReferenceProgressStatus::kContourGuardViolation, [&](auto& result) {
        return projector.projectNominalCommit(
            pose(1.0, 0.26, 0.0), targetCycle(), expected, 7, commit, path,
            result);
      });
  expectFailureAtomic(
      ReferenceProgressStatus::kHeadingGuardViolation, [&](auto& result) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.21), targetCycle(), expected, 7, commit, path,
            result);
      });
}

TEST(MainlineReferenceProgressProjector,
     FrozenStartUsesOnlyTheConfiguredAnchorWindow) {
  ReferenceProgressProjectorConfig config = makeConfig();
  config.start_s_max = 0.5;
  const Projector projector(config);
  const Path path = straightPath();
  const ReferencePathIdentity expected = identity();
  const CycleRequest cycle = targetCycle();
  const FrozenStartProgressAnchor anchor = frozenAnchor();
  ReferenceProgressProjection output;

  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectFrozenStart(pose(0.3, 0.0, 0.0), cycle,
                                          expected, 7, anchor, path, output));
  EXPECT_DOUBLE_EQ(0.3, output.s);
  EXPECT_DOUBLE_EQ(0.0, output.search_lower);
  EXPECT_DOUBLE_EQ(0.5, output.search_upper);
  EXPECT_EQ(ProgressAuthority::kFrozenStartInterval, output.authority);
  EXPECT_EQ(1u, output.authority_cycle_id);
  EXPECT_EQ(7u, output.history_generation);

  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectFrozenStart(pose(0.5, 0.0, 0.0), cycle,
                                          expected, 7, anchor, path, output));
  EXPECT_DOUBLE_EQ(0.5, output.s);

  expectFailureAtomic(
      ReferenceProgressStatus::kStartWindowViolation, [&](auto& result) {
        return projector.projectFrozenStart(pose(0.6, 0.0, 0.0), cycle,
                                             expected, 7, anchor, path,
                                             result);
      });
}

TEST(MainlineReferenceProgressProjector,
     ChecksNominalIdentityGenerationCycleAndTimeAuthority) {
  const Projector projector(makeConfig());
  const Path path = straightPath();
  const ReferencePathIdentity expected = identity();
  const NominalProgressCommit commit = nominalCommit();

  expectFailureAtomic(
      ReferenceProgressStatus::kPathMismatch, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(),
            ReferencePathIdentity{18, expected.path_hash, kResetEpoch}, 7,
            commit, path, output);
      });
  expectFailureAtomic(
      ReferenceProgressStatus::kEpochMismatch, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(), expected, 7,
            nominalCommit(ReferencePathIdentity{kPathId, expected.path_hash,
                                                kResetEpoch + 1}),
            path, output);
      });
  expectFailureAtomic(
      ReferenceProgressStatus::kGenerationMismatch, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(), expected, 8, commit, path,
            output);
      });
  NominalProgressCommit inconsistent_generation = commit;
  inconsistent_generation.release_generation = 8;
  expectFailureAtomic(
      ReferenceProgressStatus::kGenerationMismatch, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(), expected, 7,
            inconsistent_generation, path, output);
      });
  expectFailureAtomic(
      ReferenceProgressStatus::kTargetCycleMismatch, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(2), expected, 7, commit, path,
            output);
      });
  NominalProgressCommit wrong_source = commit;
  wrong_source.source = ProgressAuthority::kFrozenStartInterval;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidAuthority, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(), expected, 7, wrong_source,
            path, output);
      });
  expectFailureAtomic(
      ReferenceProgressStatus::kTimeRegression, [&](auto& output) {
        NominalProgressCommit regressed = nominalCommit();
        regressed.release_cycle.release_model = ModelTimeNs(3000000000LL);
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(), expected, 7, regressed, path,
            output);
      });
  NominalProgressCommit wrong_model_grid = commit;
  ++wrong_model_grid.release_cycle.release_model.value;
  expectFailureAtomic(
      ReferenceProgressStatus::kTargetCycleMismatch, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(), expected, 7,
            wrong_model_grid, path, output);
      });
  NominalProgressCommit wrong_steady_grid = commit;
  ++wrong_steady_grid.release_cycle.release_steady.value;
  expectFailureAtomic(
      ReferenceProgressStatus::kTargetCycleMismatch, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(), expected, 7,
            wrong_steady_grid, path, output);
      });

  FrozenStartProgressAnchor wrong_anchor = frozenAnchor();
  wrong_anchor.source = ProgressAuthority::kNominalLiveRelease;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidAuthority, [&](auto& output) {
        return projector.projectFrozenStart(
            pose(0.3, 0.0, 0.0), targetCycle(), expected, 7, wrong_anchor,
            path, output);
      });
  wrong_anchor = frozenAnchor();
  wrong_anchor.history_generation = 8;
  expectFailureAtomic(
      ReferenceProgressStatus::kGenerationMismatch, [&](auto& output) {
        return projector.projectFrozenStart(
            pose(0.3, 0.0, 0.0), targetCycle(), expected, 7, wrong_anchor,
            path, output);
      });
  wrong_anchor = frozenAnchor(identity(), 1, 0);
  expectFailureAtomic(
      ReferenceProgressStatus::kGenerationMismatch, [&](auto& output) {
        return projector.projectFrozenStart(
            pose(0.3, 0.0, 0.0), targetCycle(), expected, 0, wrong_anchor,
            path, output);
      });
  wrong_anchor = frozenAnchor();
  wrong_anchor.target_cycle.cycle_id = 2;
  expectFailureAtomic(
      ReferenceProgressStatus::kTargetCycleMismatch, [&](auto& output) {
        return projector.projectFrozenStart(
            pose(0.3, 0.0, 0.0), targetCycle(), expected, 7, wrong_anchor,
            path, output);
      });
}

TEST(MainlineReferenceProgressProjector,
     DeduplicatesSharedVerticesAndIgnoresZeroLengthDuplicates) {
  ReferenceProgressProjectorConfig config = makeConfig();
  config.start_s_max = 6.0;
  config.forward_guard = 8.0;
  config.contour_guard = 0.3;
  config.heading_guard = kPi;
  const Projector projector(config);
  const ReferencePathIdentity expected = identity();
  const Path path = cornerPath();
  const NominalProgressCommit commit = nominalCommit(identity(), 0.0);
  const CycleRequest cycle = targetCycle();

  ReferenceProgressProjection output;
  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(2.0, 0.0, 0.0), cycle, expected, 7, commit, path,
                output));
  EXPECT_DOUBLE_EQ(2.0, output.s);
  EXPECT_EQ(2u, output.accepted_candidate_count);
  EXPECT_EQ(0u, output.selected_segment);

  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(2.0, 0.0, kPi / 2.0), cycle, expected, 7, commit,
                path, output));
  EXPECT_DOUBLE_EQ(2.0, output.s);
  EXPECT_EQ(1u, output.selected_segment);

  const Path duplicate = zeroLengthDuplicatePath();
  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(1.0, 0.0, 0.0), cycle, expected, 7, commit, duplicate,
                output));
  EXPECT_DOUBLE_EQ(1.0, output.s);
  EXPECT_EQ(1u, output.selected_segment);
  EXPECT_EQ(1u, output.accepted_candidate_count);
}

TEST(MainlineReferenceProgressProjector,
     SelectsLocalSelfIntersectionByHeadingAndNeverJumpsAcrossWindow) {
  ReferenceProgressProjectorConfig config = makeConfig();
  config.start_s_max = 8.0;
  config.forward_guard = 8.0;
  config.contour_guard = 0.25;
  config.heading_guard = 0.2;
  const Projector projector(config);
  const Path path = selfIntersectingPath();
  const ReferencePathIdentity expected = identity();
  const NominalProgressCommit whole_path = nominalCommit(identity(), 0.0);
  const CycleRequest whole_cycle = targetCycle();
  ReferenceProgressProjection output;

  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(1.0, 1.0, kPi / 4.0), whole_cycle, expected, 7,
                whole_path, path, output));
  EXPECT_NEAR(std::sqrt(2.0), output.s, kTolerance);
  EXPECT_EQ(0u, output.selected_segment);
  EXPECT_EQ(1u, output.accepted_candidate_count);

  // The low-s branch is below the local lower bound and the high-s branch is
  // above the local upper bound.  A global nearest-point fallback would make
  // this incorrectly succeed.
  const NominalProgressCommit narrow_commit = nominalCommit(identity(), 2.0);
  const ReferenceProgressProjectorConfig narrow_config = [&] {
    ReferenceProgressProjectorConfig result = config;
    result.start_s_max = 8.0;
    result.forward_guard = 0.2;
    return result;
  }();
  const Projector narrow_projector(narrow_config);
  const CycleRequest narrow_cycle = targetCycle();
  expectFailureAtomic(
      ReferenceProgressStatus::kNoCandidate, [&](auto& result) {
        return narrow_projector.projectNominalCommit(
            pose(1.0, 1.0, kPi / 4.0), narrow_cycle, expected, 7,
            narrow_commit, path, result);
      });

  const NominalProgressCommit high_commit = nominalCommit(identity(), 5.0);
  const CycleRequest high_cycle = targetCycle();
  ASSERT_EQ(ReferenceProgressStatus::kOk,
            projector.projectNominalCommit(
                pose(1.0, 1.0, -kPi / 4.0), high_cycle, expected, 7,
                high_commit, path, output));
  EXPECT_NEAR(std::sqrt(8.0) + 2.0 + std::sqrt(2.0), output.s,
              kTolerance);
  EXPECT_EQ(2u, output.selected_segment);
}

TEST(MainlineReferenceProgressProjector,
     RejectsEqualDistanceSelfIntersectionAndHonorsDistanceTolerance) {
  ReferenceProgressProjectorConfig config = makeConfig();
  config.start_s_max = 8.0;
  config.forward_guard = 8.0;
  config.heading_guard = kPi;
  // Even mathematically equal projections can differ by a few ulps after
  // each segment's independent atan/projection arithmetic.
  config.ambiguity_tolerance = kTolerance;
  const Projector exact_projector(config);
  const Path path = selfIntersectingPath();
  const ReferencePathIdentity expected = identity();
  const NominalProgressCommit commit = nominalCommit(identity(), 0.0);
  const CycleRequest cycle = targetCycle();

  expectFailureAtomic(
      ReferenceProgressStatus::kAmbiguous, [&](auto& output) {
        return exact_projector.projectNominalCommit(
            pose(1.0, 1.0, 0.0), cycle, expected, 7, commit, path, output);
      });

  const double offset = 0.0005 / std::sqrt(2.0);
  ReferenceProgressProjectorConfig tolerant_config = config;
  tolerant_config.ambiguity_tolerance = 0.001;
  const Projector tolerant_projector(tolerant_config);
  expectFailureAtomic(
      ReferenceProgressStatus::kAmbiguous, [&](auto& output) {
        return tolerant_projector.projectNominalCommit(
            pose(1.0 + offset, 1.0 + offset, 0.0), cycle, expected, 7,
            commit, path, output);
      });

  tolerant_config.ambiguity_tolerance = 0.0001;
  const Projector unique_projector(tolerant_config);
  ReferenceProgressProjection output;
  ASSERT_EQ(ReferenceProgressStatus::kOk,
            unique_projector.projectNominalCommit(
                pose(1.0 + offset, 1.0 + offset, 0.0), cycle, expected, 7,
                commit, path, output));
  EXPECT_NEAR(std::sqrt(2.0) + 0.0005, output.s, kTolerance);
}

TEST(MainlineReferenceProgressProjector,
     RejectsInvalidPathConfigPoseAndOverflowWithoutMutation) {
  ReferencePathIdentity expected = identity();
  const Path valid_path = straightPath();
  const NominalProgressCommit commit = nominalCommit();
  const CycleRequest cycle = targetCycle();

  ReferenceProgressProjectorConfig invalid_config = makeConfig();
  invalid_config.heading_guard = kPi + 1.0;
  EXPECT_THROW((Projector(invalid_config)), std::invalid_argument);
  invalid_config = makeConfig();
  invalid_config.minimum_segment_length = 0.0;
  EXPECT_THROW((Projector(invalid_config)), std::invalid_argument);
  invalid_config = makeConfig();
  invalid_config.progress_equivalence_tolerance =
      invalid_config.minimum_segment_length;
  EXPECT_THROW((Projector(invalid_config)), std::invalid_argument);
  invalid_config = makeConfig();
  invalid_config.progress_equivalence_tolerance =
      std::numeric_limits<double>::max();
  EXPECT_THROW((Projector(invalid_config)), std::invalid_argument);
  invalid_config = makeConfig();
  invalid_config.start_s_max = -1.0;
  EXPECT_THROW((Projector(invalid_config)), std::invalid_argument);
  invalid_config = makeConfig();
  invalid_config.contour_guard = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((Projector(invalid_config)), std::invalid_argument);

  const Projector projector(makeConfig());
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidPose, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0), cycle,
            expected, 7, commit, valid_path, output);
      });
  ReferencePathIdentity zero_identity;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidIdentity, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), cycle, zero_identity, 7, commit, valid_path,
            output);
      });

  Path invalid = valid_path;
  invalid.vertex_count = 1;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidPath, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), cycle, expected, 7, commit, invalid, output);
      });
  invalid = valid_path;
  invalid.vertex_count = kVertexCapacity + 1;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidPath, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), cycle, expected, 7, commit, invalid, output);
      });
  invalid = valid_path;
  invalid.s_path_end = 9.0;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidPath, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), cycle, expected, 7, commit, invalid, output);
      });
  invalid = valid_path;
  invalid.vertices[1].cumulative_s = 11.0;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidPath, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), cycle, expected, 7, commit, invalid, output);
      });
  invalid = valid_path;
  invalid.vertices[1].x = std::numeric_limits<double>::quiet_NaN();
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidPath, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), cycle, expected, 7, commit, invalid, output);
      });
  invalid = valid_path;
  invalid.vertices[1] = ReferencePathVertex{0.0, 0.0, 0.5};
  invalid.s_path_end = 0.5;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidPath, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(0.0, 0.0, 0.0), cycle, expected, 7, commit, invalid, output);
      });
  invalid = valid_path;
  invalid.vertices[1] = ReferencePathVertex{0.0, 0.0, 0.0};
  invalid.s_path_end = 0.0;
  expectFailureAtomic(
      ReferenceProgressStatus::kInvalidPath, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(0.0, 0.0, 0.0), cycle, expected, 7, commit, invalid, output);
      });

  expectFailureAtomic(
      ReferenceProgressStatus::kForwardWindowViolation, [&](auto& output) {
        return projector.projectNominalCommit(
            pose(std::numeric_limits<double>::max(),
                 std::numeric_limits<double>::max(), 0.0),
            cycle, expected, 7, commit, valid_path, output);
      });

  ReferenceProgressProjectorConfig overflow_config = makeConfig();
  overflow_config.v_progress_bound = std::numeric_limits<double>::max();
  overflow_config.forward_guard = std::numeric_limits<double>::max();
  const Projector overflow_projector(overflow_config);
  expectFailureAtomic(
      ReferenceProgressStatus::kProgressOverflow, [&](auto& output) {
        return overflow_projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), cycle, expected, 7, commit, valid_path,
            output);
      });
  expectFailureAtomic(
      ReferenceProgressStatus::kTimeOverflow, [&](auto& output) {
        NominalProgressCommit overflowing = nominalCommit();
        overflowing.release_cycle.release_model =
            ModelTimeNs(std::numeric_limits<std::int64_t>::min());
        return projector.projectNominalCommit(
            pose(1.0, 0.0, 0.0), targetCycle(), expected, 7, overflowing,
            valid_path, output);
      });
}

TEST(MainlineReferenceProgressProjector,
     RejectsInvalidFrozenIdentityAndPathAuthorityAtomically) {
  ReferenceProgressProjectorConfig config = makeConfig();
  config.start_s_max = 0.5;
  const Projector projector(config);
  const Path path = straightPath();
  const ReferencePathIdentity expected = identity();
  const CycleRequest cycle = targetCycle();

  FrozenStartProgressAnchor anchor = frozenAnchor();
  ReferencePathIdentity wrong_hash = expected;
  wrong_hash.path_hash.bytes[1] = 9;
  anchor.identity = wrong_hash;
  expectFailureAtomic(
      ReferenceProgressStatus::kPathMismatch, [&](auto& output) {
        return projector.projectFrozenStart(
            pose(0.3, 0.0, 0.0), cycle, expected, 7, anchor, path, output);
      });

  anchor = frozenAnchor();
  anchor.identity.reset_epoch = kResetEpoch + 1;
  expectFailureAtomic(
      ReferenceProgressStatus::kEpochMismatch, [&](auto& output) {
        return projector.projectFrozenStart(
            pose(0.3, 0.0, 0.0), cycle, expected, 7, anchor, path, output);
      });

  anchor = frozenAnchor();
  anchor.target_cycle.release_model = ModelTimeNs(2000000001LL);
  expectFailureAtomic(
      ReferenceProgressStatus::kTargetCycleMismatch, [&](auto& output) {
        return projector.projectFrozenStart(
            pose(0.3, 0.0, 0.0), cycle, expected, 7, anchor, path, output);
      });
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
