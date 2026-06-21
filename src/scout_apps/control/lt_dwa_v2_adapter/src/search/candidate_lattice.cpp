#include "lt_dwa_v2_adapter/search/candidate_lattice.h"

namespace lt_dwa_v2_adapter
{
namespace
{
void accumulateScore(ScoreBreakdown& total, const ScoreBreakdown& inc)
{
  total.obstacle += inc.obstacle;
  total.path_lateral += inc.path_lateral;
  total.heading += inc.heading;
  total.progress += inc.progress;
  total.terminal += inc.terminal;
  total.smooth += inc.smooth;
  total.speed += inc.speed;
}
}  // namespace

TrajectoryCandidate makeRootCandidate(const PathProjection& initial_match)
{
  TrajectoryCandidate root;
  root.valid = initial_match.valid;
  root.total_cost = 0.0;
  root.progress_index = initial_match.index;
  root.progress_s = initial_match.progress_s;
  return root;
}

TrajectoryCandidate appendCandidatePoint(const TrajectoryCandidate& parent,
                                         const TrajectoryPoint& point,
                                         const PathProjection& match)
{
  TrajectoryCandidate child = parent;
  child.points.push_back(point);
  if (parent.points.empty())
    child.first_command = point.command;
  child.total_cost += point.incremental_cost;
  accumulateScore(child.score, point.score);
  child.progress_index = match.index;
  child.progress_s = match.progress_s;
  child.valid = true;
  return child;
}
}  // namespace lt_dwa_v2_adapter
