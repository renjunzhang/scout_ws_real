#include "lt_dwa_v2_adapter/search/frontier_pruner.h"

#include <algorithm>
#include <cmath>

namespace lt_dwa_v2_adapter
{
namespace
{
double rankingCost(const TrajectoryCandidate& candidate)
{
  return candidate.total_cost - candidate.progress_s;
}
}  // namespace

FrontierPruner::FrontierPruner(const SearchConfig& config)
{
  configure(config);
}

void FrontierPruner::configure(const SearchConfig& config)
{
  config_ = config;
  config_.top_k_per_layer = std::max(1, config_.top_k_per_layer);
}

void FrontierPruner::prune(std::vector<TrajectoryCandidate>& frontier) const
{
  std::sort(frontier.begin(), frontier.end(), [this](const TrajectoryCandidate& a, const TrajectoryCandidate& b) {
    return better(a, b);
  });
  if (static_cast<int>(frontier.size()) > config_.top_k_per_layer)
    frontier.resize(static_cast<size_t>(config_.top_k_per_layer));
}

const TrajectoryCandidate* FrontierPruner::best(const std::vector<TrajectoryCandidate>& frontier) const
{
  if (frontier.empty())
    return nullptr;
  return &(*std::min_element(frontier.begin(), frontier.end(), [this](const TrajectoryCandidate& a,
                                                                      const TrajectoryCandidate& b) {
    return better(a, b);
  }));
}

bool FrontierPruner::better(const TrajectoryCandidate& a, const TrajectoryCandidate& b) const
{
  const double a_rank = rankingCost(a);
  const double b_rank = rankingCost(b);
  if (std::abs(a_rank - b_rank) > 1e-9)
    return a_rank < b_rank;
  if (std::abs(a.total_cost - b.total_cost) > 1e-9)
    return a.total_cost < b.total_cost;
  if (std::abs(a.progress_s - b.progress_s) > 1e-9)
    return a.progress_s > b.progress_s;
  if (a.points.size() != b.points.size())
    return a.points.size() > b.points.size();
  if (std::abs(a.first_command.v - b.first_command.v) > 1e-9)
    return a.first_command.v > b.first_command.v;
  return a.first_command.omega < b.first_command.omega;
}
}  // namespace lt_dwa_v2_adapter
