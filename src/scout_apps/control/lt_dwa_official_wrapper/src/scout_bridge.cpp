#include "lt_dwa_official_wrapper/scout_bridge.hpp"

#include <cmath>
#include <sstream>

namespace lt_dwa_official_wrapper {
namespace {

constexpr double kPi = 3.14159265358979323846;

ScoutBridgeBuildResult MakeResult(WrapperStatus status, const std::string& reason) {
  ScoutBridgeBuildResult result;
  result.status = status;
  result.reason = reason;
  return result;
}

std::string ExpectedFrame(const ScoutBridgeConfig& config) {
  return config.planner_config.planning_frame.empty() ? "odom" : config.planner_config.planning_frame;
}

bool FrameMatches(const std::string& actual, const std::string& expected) {
  return !actual.empty() && actual == expected;
}

std::string PoseFrame(const geometry_msgs::PoseStamped& pose, const std::string& fallback) {
  return pose.header.frame_id.empty() ? fallback : pose.header.frame_id;
}

ros::Time PoseStamp(const geometry_msgs::PoseStamped& pose, const ros::Time& fallback) {
  return pose.header.stamp.isZero() ? fallback : pose.header.stamp;
}

double YawFromQuaternion(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double NormalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

Pose2d ToPose2d(const geometry_msgs::PoseStamped& pose,
                const std::string& frame,
                const ros::Time& fallback_stamp) {
  Pose2d out;
  out.frame_id = frame;
  out.stamp = PoseStamp(pose, fallback_stamp);
  out.x = pose.pose.position.x;
  out.y = pose.pose.position.y;
  out.yaw = NormalizeAngle(YawFromQuaternion(pose.pose.orientation));
  return out;
}

Pose2d PathPoseToPose2d(const nav_msgs::Path& path,
                        const geometry_msgs::PoseStamped& pose,
                        const std::string& expected_frame) {
  return ToPose2d(pose, expected_frame, path.header.stamp);
}

bool IsOdomStale(const nav_msgs::Odometry& odom,
                 const ScoutBridgeConfig& config,
                 const ros::Time& now) {
  if (now.isZero() || odom.header.stamp.isZero() || config.planner_config.input_stale_timeout_sec <= 0.0) {
    return false;
  }
  return (now - odom.header.stamp).toSec() > config.planner_config.input_stale_timeout_sec;
}

}  // namespace

ScoutBridgeConfig DefaultScoutBridgeConfig() {
  return ScoutBridgeConfig();
}

ScoutBridgeBuildResult BuildPlannerInputForScoutBridge(const ScoutBridgeConfig& config,
                                                       const ScoutBridgeInputCache& cache,
                                                       const ros::Time& now) {
  const std::string expected_frame = ExpectedFrame(config);

  if (!cache.has_odom) {
    return MakeResult(WrapperStatus::kWaitingForInput, "missing_odom");
  }
  if (!cache.has_map) {
    return MakeResult(WrapperStatus::kWaitingForInput, "missing_map");
  }
  if (!cache.has_path) {
    return MakeResult(WrapperStatus::kWaitingForInput, "missing_path");
  }
  if (cache.path.poses.size() < 2) {
    return MakeResult(WrapperStatus::kEmptyPath, "path_has_fewer_than_2_points");
  }
  if (IsOdomStale(cache.odom, config, now)) {
    return MakeResult(WrapperStatus::kStaleInput, "odom_is_stale");
  }
  if (!FrameMatches(cache.odom.header.frame_id, expected_frame)) {
    return MakeResult(WrapperStatus::kInvalidFrame,
                      "odom_frame_expected_" + expected_frame + "_got_" + cache.odom.header.frame_id);
  }
  if (!FrameMatches(cache.map.header.frame_id, expected_frame)) {
    return MakeResult(WrapperStatus::kInvalidFrame,
                      "map_frame_expected_" + expected_frame + "_got_" + cache.map.header.frame_id);
  }
  if (!FrameMatches(cache.path.header.frame_id, expected_frame)) {
    return MakeResult(WrapperStatus::kInvalidFrame,
                      "path_frame_expected_" + expected_frame + "_got_" + cache.path.header.frame_id);
  }

  PlannerInput input;
  input.planning_frame = expected_frame;
  input.stamp = cache.odom.header.stamp.isZero() ? now : cache.odom.header.stamp;
  input.occupancy_grid = cache.map;

  input.robot_pose.frame_id = expected_frame;
  input.robot_pose.stamp = input.stamp;
  input.robot_pose.x = cache.odom.pose.pose.position.x;
  input.robot_pose.y = cache.odom.pose.pose.position.y;
  input.robot_pose.yaw = NormalizeAngle(YawFromQuaternion(cache.odom.pose.pose.orientation));
  input.robot_twist.v = cache.odom.twist.twist.linear.x;
  input.robot_twist.w = cache.odom.twist.twist.angular.z;

  input.reference_path.reserve(cache.path.poses.size());
  for (const auto& path_pose : cache.path.poses) {
    const std::string pose_frame = PoseFrame(path_pose, cache.path.header.frame_id);
    if (!FrameMatches(pose_frame, expected_frame)) {
      return MakeResult(WrapperStatus::kInvalidFrame,
                        "path_pose_frame_expected_" + expected_frame + "_got_" + pose_frame);
    }
    input.reference_path.push_back(PathPoseToPose2d(cache.path, path_pose, expected_frame));
  }

  const auto& path_end = cache.path.poses.back();
  input.target_pose = PathPoseToPose2d(cache.path, path_end, expected_frame);
  if (cache.has_goal) {
    const std::string goal_frame = PoseFrame(cache.goal, expected_frame);
    if (!FrameMatches(goal_frame, expected_frame)) {
      return MakeResult(WrapperStatus::kInvalidFrame,
                        "goal_frame_expected_" + expected_frame + "_got_" + goal_frame);
    }
    input.target_pose.x = cache.goal.pose.position.x;
    input.target_pose.y = cache.goal.pose.position.y;
    input.target_pose.stamp = PoseStamp(cache.goal, input.target_pose.stamp);
  }

  ScoutBridgeBuildResult result;
  result.status = WrapperStatus::kOk;
  result.reason = "ok";
  result.input = input;
  return result;
}

bool ShouldPublishCmdVel(const ScoutBridgeConfig& config) {
  return config.enable_actuated_output && config.publish_cmd_vel;
}

bool ShouldPublishBenchmarkRaw(const ScoutBridgeConfig& config) {
  return config.enable_actuated_output && config.publish_benchmark_raw;
}

ScoutBridgeCommandDecision DecideCommandPublication(const ScoutBridgeConfig& config,
                                                    const ScoutBridgeCommandState& state,
                                                    const ros::Time& now) {
  ScoutBridgeCommandDecision decision;
  decision.publish_cmd_vel = ShouldPublishCmdVel(config);
  decision.publish_benchmark_raw = ShouldPublishBenchmarkRaw(config);

  if (!state.has_command) {
    decision.reason = state.reason.empty() ? "missing_command" : state.reason;
    return decision;
  }
  if (state.status != WrapperStatus::kOk) {
    decision.reason = state.reason.empty() ? "command_status_not_ok" : state.reason;
    return decision;
  }

  if (!now.isZero() && !state.stamp.isZero()) {
    decision.command_age_sec = (now - state.stamp).toSec();
    if (config.command_stale_timeout_sec > 0.0 &&
        decision.command_age_sec > config.command_stale_timeout_sec) {
      decision.reason = "command_stale";
      return decision;
    }
  }

  decision.fresh = true;
  decision.command_v = state.command_v;
  decision.command_w = state.command_w;
  decision.reason = "command_fresh";
  return decision;
}

std::string FormatScoutBridgeDiagnostics(const ScoutBridgeConfig& config,
                                         WrapperStatus status,
                                         const std::string& reason,
                                         double command_v,
                                         double command_w,
                                         double command_age_sec,
                                         bool command_fresh,
                                         double worker_latency_ms) {
  std::ostringstream oss;
  oss << std::boolalpha
      << "status=" << ToString(status)
      << " reason=" << reason
      << " shadow_cmd_topic=" << config.shadow_cmd_topic
      << " expected_map_file=" << config.expected_map_file
      << " planner_rate_hz=" << config.planner_rate_hz
      << " command_publish_rate_hz=" << config.command_publish_rate_hz
      << " command_stale_timeout_sec=" << config.command_stale_timeout_sec
      << " enable_actuated_output=" << config.enable_actuated_output
      << " publish_cmd_vel=" << config.publish_cmd_vel
      << " publish_benchmark_raw=" << config.publish_benchmark_raw
      << " max_v=" << config.planner_config.max_v
      << " min_v=" << config.planner_config.min_v
      << " max_w=" << config.planner_config.max_w
      << " max_acc=" << config.planner_config.max_acc
      << " max_angular_acc=" << config.planner_config.max_angular_acc
      << " robot_radius=" << config.planner_config.robot_radius
      << " time_step=" << config.planner_config.time_step
      << " path_resample_spacing=" << config.planner_config.path_resample_spacing
      << " enable_path_tracking_guard=" << config.planner_config.enable_path_tracking_guard
      << " path_tracking_lookahead_m=" << config.planner_config.path_tracking_lookahead_m
      << " path_tracking_min_v=" << config.planner_config.path_tracking_min_v
      << " effective_cmd_vel=" << ShouldPublishCmdVel(config)
      << " effective_benchmark_raw=" << ShouldPublishBenchmarkRaw(config)
      << " cmd_vel_topic=" << config.cmd_vel_topic
      << " benchmark_raw_topic=" << config.benchmark_raw_topic
      << " worker_tf_topic=" << config.worker_tf_topic
      << " worker_tf_static_topic=" << config.worker_tf_static_topic
      << " command_fresh=" << command_fresh
      << " command_age_sec=" << command_age_sec
      << " worker_latency_ms=" << worker_latency_ms
      << " command_v=" << command_v
      << " command_w=" << command_w;
  return oss.str();
}

}  // namespace lt_dwa_official_wrapper
