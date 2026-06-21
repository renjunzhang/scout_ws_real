#pragma once

#include <cstddef>
#include <vector>

#include "lt_dwa_adapter/trajectory_types.h"

namespace lt_dwa_adapter
{
struct PathProjection
{
  bool valid = false;
  double distance = 0.0;
  double signed_lateral_error = 0.0;
  double heading_error = 0.0;
  double progress_s = 0.0;
  double index = 0.0;
  size_t segment_index = 0;
  Pose2D pose;
};

class PathReference
{
public:
  bool setPath(const std::vector<Pose2D>& path);
  bool empty() const;
  size_t size() const;
  double totalLength() const;
  const std::vector<Pose2D>& points() const;

  PathProjection project(const RobotState& state, double min_progress_s = 0.0, double max_progress_s = -1.0) const;
  Pose2D sampleByProgress(double progress_s) const;

private:
  std::vector<Pose2D> points_;
  std::vector<double> cumulative_s_;
  double total_length_ = 0.0;
};
}  // namespace lt_dwa_adapter
