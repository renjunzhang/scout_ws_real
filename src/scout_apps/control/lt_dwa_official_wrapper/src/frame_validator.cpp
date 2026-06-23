#include "lt_dwa_official_wrapper/frame_validator.hpp"

#include <cmath>
#include <sstream>

namespace lt_dwa_official_wrapper {
namespace {

bool IsFinite(double value) {
  return std::isfinite(value);
}

double Distance(const Pose2d& a, const Pose2d& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

ValidationResult MakeResult(WrapperStatus status, const std::string& reason) {
  ValidationResult result;
  result.status = status;
  result.reason = reason;
  return result;
}

}  // namespace

ValidationResult FrameValidator::ValidateInput(const PlannerInput& input,
                                               const PlannerConfig& config,
                                               const ros::Time& now) const {
  const std::string expected_frame = config.planning_frame.empty() ? "odom" : config.planning_frame;

  if (!input.planning_frame.empty()) {
    const auto frame_result = ValidateFrame(input.planning_frame, expected_frame, "input.planning_frame");
    if (!frame_result.ok()) {
      return frame_result;
    }
  }

  const auto robot_frame = ValidateFrame(input.robot_pose.frame_id, expected_frame, "robot_pose.frame_id");
  if (!robot_frame.ok()) {
    return robot_frame;
  }
  const auto target_frame = ValidateFrame(input.target_pose.frame_id, expected_frame, "target_pose.frame_id");
  if (!target_frame.ok()) {
    return target_frame;
  }

  if (!IsFinite(input.robot_pose.x) || !IsFinite(input.robot_pose.y) || !IsFinite(input.robot_pose.yaw) ||
      !IsFinite(input.target_pose.x) || !IsFinite(input.target_pose.y) || !IsFinite(input.target_pose.yaw) ||
      !IsFinite(input.robot_twist.v) || !IsFinite(input.robot_twist.w)) {
    return MakeResult(WrapperStatus::kWaitingForInput, "robot pose, target pose, or twist contains non-finite values");
  }

  if (!now.isZero() && !input.stamp.isZero() && config.input_stale_timeout_sec > 0.0) {
    const double age = (now - input.stamp).toSec();
    if (age > config.input_stale_timeout_sec) {
      std::ostringstream oss;
      oss << "input stamp is stale: age=" << age << " timeout=" << config.input_stale_timeout_sec;
      return MakeResult(WrapperStatus::kStaleInput, oss.str());
    }
  }

  const auto path_result = ValidatePath(input.reference_path, expected_frame);
  if (!path_result.ok()) {
    return path_result;
  }

  const auto map_result = ValidateMap(input.occupancy_grid, expected_frame);
  if (!map_result.ok()) {
    return map_result;
  }

  const auto obstacle_result = ValidateObstacles(input.dynamic_obstacles, expected_frame);
  if (!obstacle_result.ok()) {
    return obstacle_result;
  }

  return MakeResult(WrapperStatus::kOk, "ok");
}

ValidationResult FrameValidator::ValidateFrame(const std::string& actual,
                                               const std::string& expected,
                                               const std::string& field_name) const {
  if (actual.empty()) {
    return MakeResult(WrapperStatus::kInvalidFrame, field_name + " is empty");
  }
  if (actual != expected) {
    return MakeResult(WrapperStatus::kInvalidFrame,
                      field_name + " expected " + expected + " but got " + actual);
  }
  return MakeResult(WrapperStatus::kOk, "ok");
}

ValidationResult FrameValidator::ValidatePath(const std::vector<Pose2d>& path,
                                              const std::string& expected_frame) const {
  if (path.size() < 2) {
    return MakeResult(WrapperStatus::kEmptyPath, "reference path has fewer than 2 points");
  }

  double length = 0.0;
  Pose2d previous_valid;
  bool have_previous = false;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto frame_result = ValidateFrame(path[i].frame_id, expected_frame, "reference_path.frame_id");
    if (!frame_result.ok()) {
      return frame_result;
    }
    if (!IsFinite(path[i].x) || !IsFinite(path[i].y) || !IsFinite(path[i].yaw)) {
      return MakeResult(WrapperStatus::kDegeneratePath, "reference path contains non-finite values");
    }
    if (have_previous) {
      length += Distance(previous_valid, path[i]);
    }
    previous_valid = path[i];
    have_previous = true;
  }

  if (length <= 1.0e-9) {
    return MakeResult(WrapperStatus::kDegeneratePath, "reference path length is zero");
  }
  return MakeResult(WrapperStatus::kOk, "ok");
}

ValidationResult FrameValidator::ValidateMap(const nav_msgs::OccupancyGrid& map,
                                             const std::string& expected_frame) const {
  const auto frame_result = ValidateFrame(map.header.frame_id, expected_frame, "occupancy_grid.header.frame_id");
  if (!frame_result.ok()) {
    return frame_result;
  }
  if (map.info.width == 0 || map.info.height == 0 || !(map.info.resolution > 0.0)) {
    return MakeResult(WrapperStatus::kInvalidMap, "occupancy grid dimensions or resolution are invalid");
  }
  const std::size_t expected_size = static_cast<std::size_t>(map.info.width) * static_cast<std::size_t>(map.info.height);
  if (map.data.size() != expected_size) {
    return MakeResult(WrapperStatus::kInvalidMap, "occupancy grid data size does not match width * height");
  }
  return MakeResult(WrapperStatus::kOk, "ok");
}

ValidationResult FrameValidator::ValidateObstacles(const std::vector<ObstacleTrack>& obstacles,
                                                   const std::string& expected_frame) const {
  for (const auto& obstacle : obstacles) {
    const auto frame_result = ValidateFrame(obstacle.frame_id, expected_frame, "dynamic_obstacles.frame_id");
    if (!frame_result.ok()) {
      return frame_result;
    }
    if (!IsFinite(obstacle.x) || !IsFinite(obstacle.y) ||
        !IsFinite(obstacle.vx) || !IsFinite(obstacle.vy) ||
        !IsFinite(obstacle.radius) || obstacle.radius < 0.0) {
      return MakeResult(WrapperStatus::kInvalidObstacle, "dynamic obstacle contains invalid values");
    }
  }
  return MakeResult(WrapperStatus::kOk, "ok");
}

}  // namespace lt_dwa_official_wrapper
