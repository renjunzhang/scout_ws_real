#include <sys/stat.h>

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "lt_dwa_official_wrapper/frame_validator.hpp"
#include "lt_dwa_official_wrapper/scout_bridge.hpp"
#include "lt_dwa_official_wrapper/worker_protocol.hpp"
#include "lt_dwa_official_wrapper/worker_request.hpp"
#include "lt_dwa_official_wrapper/worker_supervisor.hpp"

#ifdef LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE
#include "lt_dwa_official_wrapper/official_core_runner.hpp"
#endif

#ifndef OFFICIAL_LT_DWA_ROOT
#define OFFICIAL_LT_DWA_ROOT ""
#endif

namespace lt_dwa_official_wrapper {
namespace {

void LoadStringParam(const ros::NodeHandle& nh,
                     const std::string& name,
                     std::string* value) {
  nh.param<std::string>(name, *value, *value);
}

bool EnsureDirectory(const std::string& path) {
  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    return S_ISDIR(info.st_mode);
  }
  return mkdir(path.c_str(), 0755) == 0;
}

std::string MakeRequestPath(const std::string& dir, const ros::Time& now) {
  return dir + "/lt_dwa_scout_bridge_request_" + std::to_string(now.toNSec()) + ".txt";
}

std_msgs::String MakeStringMsg(const std::string& text) {
  std_msgs::String msg;
  msg.data = text;
  return msg;
}

geometry_msgs::Twist MakeTwist(double v, double w) {
  geometry_msgs::Twist msg;
  msg.linear.x = v;
  msg.angular.z = w;
  return msg;
}

double ElapsedMs(const std::chrono::steady_clock::time_point& start) {
  const auto elapsed = std::chrono::steady_clock::now() - start;
  return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(elapsed).count();
}

double PositiveOrDefault(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

}  // namespace

class ScoutBridgeNode {
 public:
  ScoutBridgeNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh), config_(DefaultScoutBridgeConfig()) {
    LoadConfig();

    odom_sub_ = nh_.subscribe(config_.odom_topic, 1, &ScoutBridgeNode::OdomCallback, this);
    map_sub_ = nh_.subscribe(config_.map_topic, 1, &ScoutBridgeNode::MapCallback, this);
    path_sub_ = nh_.subscribe(config_.path_topic, 1, &ScoutBridgeNode::PathCallback, this);
    goal_sub_ = nh_.subscribe(config_.goal_topic, 1, &ScoutBridgeNode::GoalCallback, this);

    shadow_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.shadow_cmd_topic, 1);
    raw_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.raw_cmd_topic, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(config_.status_topic, 1, true);
    diagnostics_pub_ = nh_.advertise<std_msgs::String>(config_.diagnostics_topic, 1, true);
    global_plan_pub_ = nh_.advertise<nav_msgs::Path>(config_.global_plan_topic, 1, true);
    local_plan_pub_ = nh_.advertise<nav_msgs::Path>(config_.local_plan_topic, 1, true);
    worker_result_pub_ = nh_.advertise<std_msgs::String>(config_.worker_result_topic, 1, true);

    if (ShouldPublishCmdVel(config_)) {
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.cmd_vel_topic, 1);
    }
    if (ShouldPublishBenchmarkRaw(config_)) {
      benchmark_raw_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.benchmark_raw_topic, 1);
    }
    if ((config_.publish_cmd_vel || config_.publish_benchmark_raw) && !config_.enable_actuated_output) {
      ROS_WARN_STREAM("official LT-DWA bridge ignoring actuating route params until enable_actuated_output=true");
    }
    if (config_.enable_actuated_output) {
      ROS_WARN_STREAM("official LT-DWA bridge actuated output gate is enabled; stale/invalid commands will publish zero");
    }

    StoreCommandState(MakeEmptyCommandState(WrapperStatus::kWaitingForInput,
                                             "bridge_started_shadow_only",
                                             ros::Time(),
                                             "disabled"));

    const double planner_rate = PositiveOrDefault(config_.planner_rate_hz, 5.0);
    const double command_rate = PositiveOrDefault(config_.command_publish_rate_hz, 30.0);
    planner_timer_ = nh_.createTimer(ros::Duration(1.0 / planner_rate),
                                     &ScoutBridgeNode::PlannerTimerCallback,
                                     this);
    command_timer_ = nh_.createTimer(ros::Duration(1.0 / command_rate),
                                     &ScoutBridgeNode::CommandTimerCallback,
                                     this);
    PublishStatusSnapshot(ros::Time::now());
  }

  ~ScoutBridgeNode() {
    PublishFinalZeroIfNeeded();
  }

 private:
  void LoadConfig() {
    LoadStringParam(private_nh_, "odom_topic", &config_.odom_topic);
    LoadStringParam(private_nh_, "map_topic", &config_.map_topic);
    LoadStringParam(private_nh_, "path_topic", &config_.path_topic);
    LoadStringParam(private_nh_, "goal_topic", &config_.goal_topic);
    LoadStringParam(private_nh_, "shadow_cmd_topic", &config_.shadow_cmd_topic);
    LoadStringParam(private_nh_, "raw_cmd_topic", &config_.raw_cmd_topic);
    LoadStringParam(private_nh_, "status_topic", &config_.status_topic);
    LoadStringParam(private_nh_, "diagnostics_topic", &config_.diagnostics_topic);
    LoadStringParam(private_nh_, "global_plan_topic", &config_.global_plan_topic);
    LoadStringParam(private_nh_, "local_plan_topic", &config_.local_plan_topic);
    LoadStringParam(private_nh_, "worker_result_topic", &config_.worker_result_topic);
    LoadStringParam(private_nh_, "cmd_vel_topic", &config_.cmd_vel_topic);
    LoadStringParam(private_nh_, "benchmark_raw_topic", &config_.benchmark_raw_topic);
    LoadStringParam(private_nh_, "expected_map_file", &config_.expected_map_file);
    LoadStringParam(private_nh_, "planner_execution_mode", &config_.planner_execution_mode);
    LoadStringParam(private_nh_, "worker_executable", &config_.worker_executable);
    LoadStringParam(private_nh_, "runtime_request_dir", &config_.runtime_request_dir);
    LoadStringParam(private_nh_, "worker_mode", &config_.worker_mode);
    LoadStringParam(private_nh_, "worker_tf_topic", &config_.worker_tf_topic);
    LoadStringParam(private_nh_, "worker_tf_static_topic", &config_.worker_tf_static_topic);
    LoadStringParam(private_nh_, "planning_frame", &config_.planner_config.planning_frame);
    private_nh_.param("max_v", config_.planner_config.max_v, config_.planner_config.max_v);
    private_nh_.param("min_v", config_.planner_config.min_v, config_.planner_config.min_v);
    private_nh_.param("max_w", config_.planner_config.max_w, config_.planner_config.max_w);
    private_nh_.param("max_acc", config_.planner_config.max_acc, config_.planner_config.max_acc);
    private_nh_.param("max_angular_acc",
                      config_.planner_config.max_angular_acc,
                      config_.planner_config.max_angular_acc);
    private_nh_.param("robot_radius",
                      config_.planner_config.robot_radius,
                      config_.planner_config.robot_radius);
    private_nh_.param("scan_radius",
                      config_.planner_config.scan_radius,
                      config_.planner_config.scan_radius);
    private_nh_.param("time_step", config_.planner_config.time_step, config_.planner_config.time_step);
    private_nh_.param("goal_xy_tolerance",
                      config_.planner_config.goal_xy_tolerance,
                      config_.planner_config.goal_xy_tolerance);
    private_nh_.param("goal_yaw_tolerance",
                      config_.planner_config.goal_yaw_tolerance,
                      config_.planner_config.goal_yaw_tolerance);

    double legacy_bridge_rate = config_.bridge_rate_hz;
    if (private_nh_.getParam("bridge_rate_hz", legacy_bridge_rate)) {
      config_.bridge_rate_hz = legacy_bridge_rate;
      config_.planner_rate_hz = legacy_bridge_rate;
    }
    private_nh_.param("planner_rate_hz", config_.planner_rate_hz, config_.planner_rate_hz);
    private_nh_.param("command_publish_rate_hz",
                      config_.command_publish_rate_hz,
                      config_.command_publish_rate_hz);
    private_nh_.param("command_stale_timeout_sec",
                      config_.command_stale_timeout_sec,
                      config_.command_stale_timeout_sec);
    private_nh_.param("worker_timeout_sec", config_.worker_timeout_sec, config_.worker_timeout_sec);
    private_nh_.param("enable_worker_core", config_.enable_worker_core, config_.enable_worker_core);
    private_nh_.param("enable_actuated_output",
                      config_.enable_actuated_output,
                      config_.enable_actuated_output);
    private_nh_.param("publish_cmd_vel", config_.publish_cmd_vel, false);
    private_nh_.param("publish_benchmark_raw", config_.publish_benchmark_raw, false);
    private_nh_.param("input_stale_timeout_sec",
                      config_.planner_config.input_stale_timeout_sec,
                      config_.planner_config.input_stale_timeout_sec);
    private_nh_.param("path_resample_spacing",
                      config_.planner_config.path_resample_spacing,
                      config_.planner_config.path_resample_spacing);
    private_nh_.param("enable_path_tracking_guard",
                      config_.planner_config.enable_path_tracking_guard,
                      config_.planner_config.enable_path_tracking_guard);
    private_nh_.param("path_tracking_lookahead_m",
                      config_.planner_config.path_tracking_lookahead_m,
                      config_.planner_config.path_tracking_lookahead_m);
    private_nh_.param("path_tracking_min_v",
                      config_.planner_config.path_tracking_min_v,
                      config_.planner_config.path_tracking_min_v);

    config_.planner_rate_hz = PositiveOrDefault(config_.planner_rate_hz, 5.0);
    config_.command_publish_rate_hz = PositiveOrDefault(config_.command_publish_rate_hz, 30.0);
    config_.command_stale_timeout_sec = config_.command_stale_timeout_sec > 0.0
                                            ? config_.command_stale_timeout_sec
                                            : 0.25;
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.odom = *msg;
    cache_.has_odom = true;
  }

  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.map = *msg;
    cache_.has_map = true;
  }

  void PathCallback(const nav_msgs::Path::ConstPtr& msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cache_.path = *msg;
      cache_.has_path = true;
    }
    global_plan_pub_.publish(*msg);
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.goal = *msg;
    cache_.has_goal = true;
  }

  bool TrySetPlannerInFlight() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (planner_in_flight_) {
      return false;
    }
    planner_in_flight_ = true;
    return true;
  }

  void SetPlannerInFlight(bool value) {
    std::lock_guard<std::mutex> lock(mutex_);
    planner_in_flight_ = value;
  }

  ScoutBridgeInputCache CacheSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_;
  }

  ScoutBridgeCommandState CommandStateSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_command_;
  }

  void StoreCommandState(const ScoutBridgeCommandState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_command_ = state;
  }

  ScoutBridgeCommandState MakeEmptyCommandState(WrapperStatus status,
                                                const std::string& reason,
                                                const ros::Time& stamp,
                                                const std::string& execution_mode,
                                                double planner_latency_ms = -1.0) const {
    ScoutBridgeCommandState state;
    state.status = status;
    state.reason = reason;
    state.stamp = stamp;
    state.execution_mode = execution_mode;
    state.planner_latency_ms = planner_latency_ms;
    return state;
  }

  ScoutBridgeCommandState RunPlannerInProcess(const PlannerInput& input, const ros::Time& now) {
    const auto start = std::chrono::steady_clock::now();
    FrameValidator validator;
    const auto validation = validator.ValidateInput(input, config_.planner_config, now);
    if (!validation.ok()) {
      return MakeEmptyCommandState(validation.status,
                                   validation.reason,
                                   now,
                                   "in_process",
                                   ElapsedMs(start));
    }

#ifdef LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE
    const auto result = RunOfficialCoreOnce(input, config_.planner_config, OFFICIAL_LT_DWA_ROOT);
    ScoutBridgeCommandState state;
    state.status = result.status;
    state.reason = result.reason;
    state.stamp = ros::Time::now();
    state.has_raw_command = result.core_return >= 0;
    state.raw_command_v = state.has_raw_command ? result.raw_command.v : 0.0;
    state.raw_command_w = state.has_raw_command ? result.raw_command.w : 0.0;
    state.has_final_command = result.core_return >= 0;
    state.final_command_v = state.has_final_command ? result.final_command.v : 0.0;
    state.final_command_w = state.has_final_command ? result.final_command.w : 0.0;
    state.guard_applied = result.guard_applied;
    state.guard_reason = result.guard_reason;
    state.has_core_return = result.core_return >= 0;
    state.core_return = result.core_return;
    state.execution_mode = "in_process";
    state.planner_latency_ms = ElapsedMs(start);
    return state;
#else
    return MakeEmptyCommandState(WrapperStatus::kCommandRejected,
                                 "official_core_build_disabled",
                                 now,
                                 "in_process",
                                 ElapsedMs(start));
#endif
  }

  ScoutBridgeCommandState RunPlannerViaWorkerOnce(const PlannerInput& input, const ros::Time& now) {
    if (!EnsureDirectory(config_.runtime_request_dir)) {
      return MakeEmptyCommandState(WrapperStatus::kWaitingForInput,
                                   "runtime_request_dir_unavailable",
                                   now,
                                   "worker_once");
    }

    const std::string request_path = MakeRequestPath(config_.runtime_request_dir, now);
    std::ofstream out(request_path);
    if (!out.good()) {
      return MakeEmptyCommandState(WrapperStatus::kWaitingForInput,
                                   "request_file_open_failed",
                                   now,
                                   "worker_once");
    }
    out << SerializeWorkerRequest(input, config_.planner_config, now);
    out.close();

    std::vector<std::string> worker_args{"--mode", config_.worker_mode, "--request", request_path};
    if (!config_.worker_tf_topic.empty()) {
      worker_args.push_back("/tf:=" + config_.worker_tf_topic);
    }
    if (!config_.worker_tf_static_topic.empty()) {
      worker_args.push_back("/tf_static:=" + config_.worker_tf_static_topic);
    }

    const auto start = std::chrono::steady_clock::now();
    const auto result = worker_supervisor_.Run(
        config_.worker_executable,
        worker_args,
        config_.worker_timeout_sec);

    ScoutBridgeCommandState state;
    state.status = result.status;
    state.reason = result.reason;
    state.stamp = ros::Time::now();
    state.has_raw_command = result.valid_response && result.has_raw_command;
    state.raw_command_v = state.has_raw_command ? result.raw_command_v : 0.0;
    state.raw_command_w = state.has_raw_command ? result.raw_command_w : 0.0;
    state.has_final_command = result.valid_response && result.status == WrapperStatus::kOk && result.has_final_command;
    state.final_command_v = state.has_final_command ? result.final_command_v : 0.0;
    state.final_command_w = state.has_final_command ? result.final_command_w : 0.0;
    state.guard_applied = result.guard_applied;
    state.guard_reason = result.guard_reason;
    state.has_core_return = result.has_core_return;
    state.core_return = result.core_return;
    state.execution_mode = "worker_once";
    state.planner_latency_ms = ElapsedMs(start);
    if (!result.output.empty()) {
      worker_result_pub_.publish(MakeStringMsg(result.output));
    }
    return state;
  }

  std::string FormatInProcessResult(const ScoutBridgeCommandState& state) const {
    if (state.has_raw_command || state.has_final_command || state.has_core_return) {
      return FormatWorkerResponse(state.status,
                                  state.reason,
                                  state.raw_command_v,
                                  state.raw_command_w,
                                  state.final_command_v,
                                  state.final_command_w,
                                  state.guard_applied,
                                  state.guard_reason,
                                  state.has_core_return ? state.core_return : -1);
    }
    return FormatWorkerResponse(state.status, state.reason);
  }

  void PlannerTimerCallback(const ros::TimerEvent&) {
    if (!TrySetPlannerInFlight()) {
      ROS_WARN_THROTTLE(1.0, "official LT-DWA bridge planner still in flight; skipping tick");
      return;
    }

    struct InFlightGuard {
      ScoutBridgeNode* node;
      ~InFlightGuard() { node->SetPlannerInFlight(false); }
    } guard{this};

    const ros::Time now = ros::Time::now();
    const auto build = BuildPlannerInputForScoutBridge(config_, CacheSnapshot(), now);
    if (!build.ok()) {
      StoreCommandState(MakeEmptyCommandState(build.status, build.reason, now, "disabled"));
      return;
    }

    PublishLocalPlanSkeleton(build.input);

    ScoutBridgeCommandState state;
    if (!config_.enable_worker_core || config_.planner_execution_mode == "disabled") {
      state = MakeEmptyCommandState(WrapperStatus::kCommandRejected,
                                    "planner_execution_disabled",
                                    now,
                                    "disabled");
    } else if (config_.planner_execution_mode == "worker_once") {
      state = RunPlannerViaWorkerOnce(build.input, now);
    } else if (config_.planner_execution_mode == "in_process") {
      state = RunPlannerInProcess(build.input, now);
      worker_result_pub_.publish(MakeStringMsg(FormatInProcessResult(state)));
    } else {
      state = MakeEmptyCommandState(WrapperStatus::kCommandRejected,
                                    "unknown_planner_execution_mode_" + config_.planner_execution_mode,
                                    now,
                                    config_.planner_execution_mode);
    }

    if (state.status == WrapperStatus::kOk && state.has_raw_command) {
      raw_cmd_pub_.publish(MakeTwist(state.raw_command_v, state.raw_command_w));
    }
    StoreCommandState(state);
  }

  void CommandTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const ScoutBridgeCommandState state = CommandStateSnapshot();
    const ScoutBridgeCommandDecision decision = DecideCommandPublication(config_, state, now);
    const geometry_msgs::Twist twist = MakeTwist(decision.command_v, decision.command_w);

    shadow_cmd_pub_.publish(twist);
    if (decision.publish_cmd_vel) {
      cmd_vel_pub_.publish(twist);
    }
    if (decision.publish_benchmark_raw) {
      benchmark_raw_pub_.publish(twist);
    }

    PublishStatus(state, decision);
  }

  void PublishLocalPlanSkeleton(const PlannerInput& input) {
    nav_msgs::Path local_plan;
    local_plan.header.frame_id = input.planning_frame;
    local_plan.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped start;
    start.header = local_plan.header;
    start.pose.position.x = input.robot_pose.x;
    start.pose.position.y = input.robot_pose.y;
    start.pose.orientation.w = 1.0;
    local_plan.poses.push_back(start);

    geometry_msgs::PoseStamped target;
    target.header = local_plan.header;
    target.pose.position.x = input.target_pose.x;
    target.pose.position.y = input.target_pose.y;
    target.pose.orientation.w = 1.0;
    local_plan.poses.push_back(target);

    local_plan_pub_.publish(local_plan);
  }

  void PublishStatusSnapshot(const ros::Time& now) {
    const ScoutBridgeCommandState state = CommandStateSnapshot();
    const ScoutBridgeCommandDecision decision = DecideCommandPublication(config_, state, now);
    PublishStatus(state, decision);
  }

  void PublishStatus(const ScoutBridgeCommandState& state,
                     const ScoutBridgeCommandDecision& decision) {
    WrapperStatus status = state.status;
    std::string reason = state.reason;
    if (state.has_final_command && !decision.fresh && state.status == WrapperStatus::kOk) {
      status = WrapperStatus::kStaleInput;
      reason = decision.reason;
    }

    status_pub_.publish(MakeStringMsg(ToString(status)));
    diagnostics_pub_.publish(MakeStringMsg(
        FormatScoutBridgeDiagnostics(config_, state, decision, status, reason)));
  }

  void PublishFinalZeroIfNeeded() {
    if (!ShouldPublishCmdVel(config_) && !ShouldPublishBenchmarkRaw(config_)) {
      return;
    }
    const geometry_msgs::Twist zero = MakeTwist(0.0, 0.0);
    for (int i = 0; i < 3; ++i) {
      if (ShouldPublishCmdVel(config_)) {
        cmd_vel_pub_.publish(zero);
      }
      if (ShouldPublishBenchmarkRaw(config_)) {
        benchmark_raw_pub_.publish(zero);
      }
      ros::Duration(0.02).sleep();
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ScoutBridgeConfig config_;
  mutable std::mutex mutex_;
  ScoutBridgeInputCache cache_;
  ScoutBridgeCommandState latest_command_;
  bool planner_in_flight_{false};
  WorkerSupervisor worker_supervisor_;

  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher shadow_cmd_pub_;
  ros::Publisher raw_cmd_pub_;
  ros::Publisher status_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher global_plan_pub_;
  ros::Publisher local_plan_pub_;
  ros::Publisher worker_result_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher benchmark_raw_pub_;
  ros::Timer planner_timer_;
  ros::Timer command_timer_;
};

}  // namespace lt_dwa_official_wrapper

int main(int argc, char** argv) {
  ros::init(argc, argv, "lt_dwa_scout_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  lt_dwa_official_wrapper::ScoutBridgeNode node(nh, private_nh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
