#include "lt_dwa_v2_adapter/geometry/path_reference.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lt_dwa_v2_adapter
{
namespace
{
double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

Pose2D interpolatePose(const Pose2D& a, const Pose2D& b, double t)
{
  Pose2D pose;
  pose.x = a.x + t * (b.x - a.x);
  pose.y = a.y + t * (b.y - a.y);
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  pose.yaw = (std::abs(dx) + std::abs(dy) > 1e-9) ? std::atan2(dy, dx) : a.yaw;
  return pose;
}
}  // namespace

bool PathReference::setPath(const std::vector<Pose2D>& path)
{
  points_ = path;
  cumulative_s_.clear();
  cumulative_s_.reserve(points_.size());
  total_length_ = 0.0;

  if (points_.empty())
    return false;

  cumulative_s_.push_back(0.0);
  for (size_t i = 1; i < points_.size(); ++i)
  {
    total_length_ += std::hypot(points_[i].x - points_[i - 1].x, points_[i].y - points_[i - 1].y);
    cumulative_s_.push_back(total_length_);
  }
  return true;
}

bool PathReference::empty() const
{
  return points_.empty();
}

size_t PathReference::size() const
{
  return points_.size();
}

double PathReference::totalLength() const
{
  return total_length_;
}

const std::vector<Pose2D>& PathReference::points() const
{
  return points_;
}

PathProjection PathReference::project(const RobotState& state, double min_progress_s, double max_progress_s) const
{
  PathProjection best;
  if (points_.empty())
    return best;

  min_progress_s = clamp(min_progress_s, 0.0, total_length_);
  if (max_progress_s < 0.0)
    max_progress_s = total_length_;
  max_progress_s = clamp(max_progress_s, min_progress_s, total_length_);
  best.distance = std::numeric_limits<double>::infinity();

  if (points_.size() == 1 || total_length_ <= 1e-9)
  {
    best.valid = true;
    best.pose = points_.front();
    best.distance = std::hypot(state.x - best.pose.x, state.y - best.pose.y);
    best.signed_lateral_error = best.distance;
    best.heading_error = std::abs(normalizeAngle(state.yaw - best.pose.yaw));
    best.progress_s = 0.0;
    best.index = 0.0;
    best.segment_index = 0;
    return best;
  }

  for (size_t i = 0; i + 1 < points_.size(); ++i)
  {
    const double segment_start_s = cumulative_s_[i];
    const double segment_end_s = cumulative_s_[i + 1];
    if (segment_end_s + 1e-9 < min_progress_s)
      continue;
    if (segment_start_s - 1e-9 > max_progress_s)
      break;

    const Pose2D& a = points_[i];
    const Pose2D& b = points_[i + 1];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    if (len2 <= 1e-12)
      continue;

    double t = ((state.x - a.x) * vx + (state.y - a.y) * vy) / len2;
    t = clamp(t, 0.0, 1.0);
    if (segment_end_s > segment_start_s && min_progress_s > segment_start_s)
    {
      const double min_t = clamp((min_progress_s - segment_start_s) / (segment_end_s - segment_start_s), 0.0, 1.0);
      t = std::max(t, min_t);
    }
    if (segment_end_s > segment_start_s && max_progress_s < segment_end_s)
    {
      const double max_t = clamp((max_progress_s - segment_start_s) / (segment_end_s - segment_start_s), 0.0, 1.0);
      t = std::min(t, max_t);
    }

    const Pose2D projected = interpolatePose(a, b, t);
    const double dx = state.x - projected.x;
    const double dy = state.y - projected.y;
    const double len = std::sqrt(len2);
    const double signed_lateral_error = (vx * dy - vy * dx) / len;
    const double distance = std::hypot(dx, dy);
    if (distance < best.distance)
    {
      best.valid = true;
      best.distance = distance;
      best.signed_lateral_error = signed_lateral_error;
      best.heading_error = std::abs(normalizeAngle(state.yaw - projected.yaw));
      best.progress_s = segment_start_s + t * (segment_end_s - segment_start_s);
      best.index = static_cast<double>(i) + t;
      best.segment_index = i;
      best.pose = projected;
    }
  }

  if (!best.valid)
  {
    best.valid = true;
    best.pose = sampleByProgress(max_progress_s);
    best.distance = std::hypot(state.x - best.pose.x, state.y - best.pose.y);
    best.signed_lateral_error = best.distance;
    best.heading_error = std::abs(normalizeAngle(state.yaw - best.pose.yaw));
    best.progress_s = max_progress_s;
    best.index = static_cast<double>(points_.size() - 1);
    best.segment_index = points_.size() - 1;
  }
  return best;
}

Pose2D PathReference::sampleByProgress(double progress_s) const
{
  if (points_.empty())
    return Pose2D{};
  if (points_.size() == 1 || total_length_ <= 1e-9)
    return points_.front();

  progress_s = clamp(progress_s, 0.0, total_length_);
  for (size_t i = 0; i + 1 < points_.size(); ++i)
  {
    const double segment_start_s = cumulative_s_[i];
    const double segment_end_s = cumulative_s_[i + 1];
    if (progress_s > segment_end_s && i + 2 < points_.size())
      continue;
    const double ds = segment_end_s - segment_start_s;
    const double t = ds > 1e-9 ? clamp((progress_s - segment_start_s) / ds, 0.0, 1.0) : 0.0;
    return interpolatePose(points_[i], points_[i + 1], t);
  }
  return points_.back();
}
}  // namespace lt_dwa_v2_adapter
