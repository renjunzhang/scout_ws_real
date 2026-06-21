#pragma once

#include <vector>

#include "lt_dwa_v2_adapter/core/trajectory_types.h"
#include "lt_dwa_v2_adapter/geometry/path_reference.h"

namespace lt_dwa_v2_adapter
{
using CandidateFrontier = std::vector<TrajectoryCandidate>;

TrajectoryCandidate makeRootCandidate(const PathProjection& initial_match);
TrajectoryCandidate appendCandidatePoint(const TrajectoryCandidate& parent,
                                         const TrajectoryPoint& point,
                                         const PathProjection& match);
}  // namespace lt_dwa_v2_adapter
