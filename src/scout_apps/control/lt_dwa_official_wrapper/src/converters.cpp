#include "lt_dwa_official_wrapper/converters.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lt_dwa_official_wrapper {
namespace {

constexpr double kEpsilon = 1.0e-9;

bool IsFinite(double value) {
  return std::isfinite(value);
}

bool IsFinitePose(const Pose2d& pose) {
  return IsFinite(pose.x) && IsFinite(pose.y) && IsFinite(pose.yaw);
}

double Distance(const Pose2d& a, const Pose2d& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

std::vector<Pose2d> CleanPath(const std::vector<Pose2d>& path) {
  std::vector<Pose2d> clean;
  clean.reserve(path.size());
  for (const auto& pose : path) {
    if (!IsFinitePose(pose)) {
      continue;
    }
    if (clean.empty() || Distance(clean.back(), pose) > kEpsilon) {
      clean.push_back(pose);
    }
  }
  return clean;
}

Pose2d Interpolate(const Pose2d& a, const Pose2d& b, double ratio) {
  Pose2d out = a;
  out.x = a.x + ratio * (b.x - a.x);
  out.y = a.y + ratio * (b.y - a.y);
  out.yaw = std::atan2(b.y - a.y, b.x - a.x);
  return out;
}

}  // namespace

Pose ToOfficialPose(const Pose2d& pose) {
  Pose out;
  out.x_ = pose.x;
  out.y_ = pose.y;
  out.theta_ = pose.yaw;
  return out;
}

Action ToOfficialAction(const Twist2d& twist) {
  Action out;
  out.v_ = twist.v;
  out.w_ = twist.w;
  return out;
}

double ComputePathLength(const std::vector<Pose2d>& path) {
  const auto clean = CleanPath(path);
  if (clean.size() < 2) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t i = 1; i < clean.size(); ++i) {
    length += Distance(clean[i - 1], clean[i]);
  }
  return length;
}

std::vector<PathPose> ToOfficialPath(const std::vector<Pose2d>& path,
                                     double resample_spacing_m) {
  const auto clean = CleanPath(path);
  std::vector<PathPose> out;
  if (clean.size() < 2) {
    return out;
  }

  std::vector<double> cumulative(clean.size(), 0.0);
  for (std::size_t i = 1; i < clean.size(); ++i) {
    cumulative[i] = cumulative[i - 1] + Distance(clean[i - 1], clean[i]);
  }

  const double total_length = cumulative.back();
  if (total_length <= kEpsilon) {
    return out;
  }

  const double spacing = resample_spacing_m > kEpsilon ? resample_spacing_m : total_length;
  std::vector<double> sample_distances;
  for (double d = 0.0; d < total_length; d += spacing) {
    sample_distances.push_back(d);
  }
  if (sample_distances.empty() || std::fabs(sample_distances.back() - total_length) > kEpsilon) {
    sample_distances.push_back(total_length);
  }

  out.reserve(sample_distances.size());
  std::size_t segment = 1;
  for (const double sample_dist : sample_distances) {
    while (segment + 1 < cumulative.size() && cumulative[segment] < sample_dist) {
      ++segment;
    }
    const double segment_len = cumulative[segment] - cumulative[segment - 1];
    const double ratio = segment_len > kEpsilon ?
        (sample_dist - cumulative[segment - 1]) / segment_len : 0.0;
    const Pose2d sampled = Interpolate(clean[segment - 1], clean[segment], ratio);

    PathPose official_pose;
    official_pose.x_ = sampled.x;
    official_pose.y_ = sampled.y;
    official_pose.theta_ = sampled.yaw;
    official_pose.dist_ = sample_dist;
    out.push_back(official_pose);
  }

  if (out.size() >= 2) {
    out.back().theta_ = out[out.size() - 2].theta_;
  }
  return out;
}

GridMap ToOfficialGridMap(const nav_msgs::OccupancyGrid& map) {
  return GridMap(map);
}

std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> ToOfficialObstacleHistory(
    const std::vector<ObstacleTrack>& obstacles,
    std::size_t fill_count) {
  std::map<int, Tools::FixedQueue<ObstacleInfo, OBSTACLE_INFO_LEN>> history;
  const std::size_t samples = std::max<std::size_t>(1, fill_count);
  for (const auto& obstacle : obstacles) {
    if (!IsFinite(obstacle.x) || !IsFinite(obstacle.y) ||
        !IsFinite(obstacle.vx) || !IsFinite(obstacle.vy) ||
        !IsFinite(obstacle.radius) || obstacle.radius < 0.0) {
      continue;
    }

    ObstacleInfo info;
    info.x_ = obstacle.x;
    info.y_ = obstacle.y;
    info.vx_ = obstacle.vx;
    info.vy_ = obstacle.vy;
    info.radius_ = obstacle.radius;

    auto& queue = history[obstacle.id];
    for (std::size_t i = 0; i < samples; ++i) {
      queue.push(info);
    }
  }
  return history;
}

}  // namespace lt_dwa_official_wrapper
