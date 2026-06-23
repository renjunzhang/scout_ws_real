#pragma once

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/time.h>

#include "lt_dwa_official_wrapper/planner_config.hpp"
#include "lt_dwa_official_wrapper/status.hpp"
#include "lt_dwa_official_wrapper/types.hpp"

namespace lt_dwa_official_wrapper {

struct ScoutBridgeConfig {
  std::string odom_topic{"/odom"};
  std::string map_topic{"/map"};
  std::string path_topic{"/scout/global_path_fixed"};
  std::string goal_topic{"/scout/goal"};

  std::string shadow_cmd_topic{"/baseline/official_lt_dwa/shadow_cmd_vel"};
  std::string status_topic{"/baseline/official_lt_dwa/status"};
  std::string diagnostics_topic{"/baseline/official_lt_dwa/diagnostics"};
  std::string global_plan_topic{"/baseline/official_lt_dwa/global_plan"};
  std::string local_plan_topic{"/baseline/official_lt_dwa/local_plan"};
  std::string worker_result_topic{"/baseline/official_lt_dwa/worker_result"};
  std::string cmd_vel_topic{"/cmd_vel"};
  std::string benchmark_raw_topic{"/benchmark/cmd_vel_raw"};

  std::string expected_map_file{"/data/a/scout_sim_replacement/maps/proxy_world_manual_saved_20260611_154348.pbstream"};
  std::string worker_executable{"/home/geist/scout_ws/devel/lib/lt_dwa_official_wrapper/lt_dwa_worker"};
  std::string runtime_request_dir{"/tmp"};
  std::string worker_mode{"official-core-once"};
  std::string worker_tf_topic{"/baseline/official_lt_dwa/worker_tf_sandbox"};
  std::string worker_tf_static_topic{"/baseline/official_lt_dwa/worker_tf_static_sandbox"};

  double bridge_rate_hz{0.0};
  double planner_rate_hz{5.0};
  double command_publish_rate_hz{30.0};
  double command_stale_timeout_sec{0.25};
  double worker_timeout_sec{1.0};
  bool enable_worker_core{false};
  bool enable_actuated_output{false};
  bool publish_cmd_vel{false};
  bool publish_benchmark_raw{false};

  PlannerConfig planner_config;
};

struct ScoutBridgeInputCache {
  bool has_odom{false};
  bool has_map{false};
  bool has_path{false};
  bool has_goal{false};

  nav_msgs::Odometry odom;
  nav_msgs::OccupancyGrid map;
  nav_msgs::Path path;
  geometry_msgs::PoseStamped goal;
};

struct ScoutBridgeBuildResult {
  WrapperStatus status{WrapperStatus::kWaitingForInput};
  std::string reason;
  PlannerInput input;

  bool ok() const { return status == WrapperStatus::kOk; }
};

struct ScoutBridgeCommandState {
  bool has_command{false};
  ros::Time stamp;
  double command_v{0.0};
  double command_w{0.0};
  WrapperStatus status{WrapperStatus::kWaitingForInput};
  std::string reason{"no_command"};
  double worker_latency_ms{-1.0};
};

struct ScoutBridgeCommandDecision {
  bool fresh{false};
  double command_age_sec{-1.0};
  double command_v{0.0};
  double command_w{0.0};
  std::string reason{"missing_command"};
  bool publish_cmd_vel{false};
  bool publish_benchmark_raw{false};
};

ScoutBridgeConfig DefaultScoutBridgeConfig();
ScoutBridgeBuildResult BuildPlannerInputForScoutBridge(const ScoutBridgeConfig& config,
                                                       const ScoutBridgeInputCache& cache,
                                                       const ros::Time& now);
bool ShouldPublishCmdVel(const ScoutBridgeConfig& config);
bool ShouldPublishBenchmarkRaw(const ScoutBridgeConfig& config);
ScoutBridgeCommandDecision DecideCommandPublication(const ScoutBridgeConfig& config,
                                                    const ScoutBridgeCommandState& state,
                                                    const ros::Time& now);
std::string FormatScoutBridgeDiagnostics(const ScoutBridgeConfig& config,
                                         WrapperStatus status,
                                         const std::string& reason,
                                         double command_v,
                                         double command_w,
                                         double command_age_sec = -1.0,
                                         bool command_fresh = false,
                                         double worker_latency_ms = -1.0);

}  // namespace lt_dwa_official_wrapper
