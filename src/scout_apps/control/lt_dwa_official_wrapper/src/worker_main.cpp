#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "lt_dwa_official_wrapper/frame_validator.hpp"
#include "lt_dwa_official_wrapper/planner_config.hpp"
#include "lt_dwa_official_wrapper/planner_facade.hpp"
#include "lt_dwa_official_wrapper/status.hpp"
#include "lt_dwa_official_wrapper/worker_protocol.hpp"
#include "lt_dwa_official_wrapper/worker_request.hpp"

#ifdef LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE
#include <ros/ros.h>

#include "lt_dwa_official_wrapper/official_core_runner.hpp"
#endif

#ifndef OFFICIAL_LT_DWA_ROOT
#define OFFICIAL_LT_DWA_ROOT ""
#endif

namespace {

std::string GetArg(int argc, char** argv, const std::string& name, const std::string& default_value = "") {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == name) {
      return argv[i + 1];
    }
  }
  return default_value;
}

std::string GetMode(int argc, char** argv) {
  return GetArg(argc, argv, "--mode", "core-disabled");
}

int Emit(lt_dwa_official_wrapper::WrapperStatus status, const std::string& reason, int exit_code = 0) {
  std::cout << lt_dwa_official_wrapper::FormatWorkerResponse(status, reason);
  return exit_code;
}

int Emit(lt_dwa_official_wrapper::WrapperStatus status,
         const std::string& reason,
         double command_v,
         double command_w,
         int core_return,
         int exit_code = 0) {
  std::cout << lt_dwa_official_wrapper::FormatWorkerResponse(
      status, reason, command_v, command_w, core_return);
  return exit_code;
}

lt_dwa_official_wrapper::PlannerConfig ConfigFromRequest(
    const lt_dwa_official_wrapper::WorkerRequestParseResult& request) {
  lt_dwa_official_wrapper::PlannerConfig config = request.has_config
      ? request.config
      : lt_dwa_official_wrapper::PlannerConfig();
  if (!request.input.planning_frame.empty()) {
    config.planning_frame = request.input.planning_frame;
  }
  return config;
}

void EmitRuntimeDiagnostics(const lt_dwa_official_wrapper::PlannerInput& input,
                            const lt_dwa_official_wrapper::PlannerConfig& config) {
  std::size_t nearest_index = 0;
  double nearest_error = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t i = 0; i < input.reference_path.size(); ++i) {
    const double dx = input.reference_path[i].x - input.robot_pose.x;
    const double dy = input.reference_path[i].y - input.robot_pose.y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(nearest_error) || distance < nearest_error) {
      nearest_error = distance;
      nearest_index = i;
    }
  }
  const double goal_dist = std::hypot(input.target_pose.x - input.robot_pose.x,
                                     input.target_pose.y - input.robot_pose.y);
  std::cout << "LT_DWA_WORKER_CONFIG planning_frame=" << config.planning_frame
            << " max_v=" << config.max_v
            << " min_v=" << config.min_v
            << " max_w=" << config.max_w
            << " max_acc=" << config.max_acc
            << " max_angular_acc=" << config.max_angular_acc
            << " robot_radius=" << config.robot_radius
            << " time_step=" << config.time_step
            << " path_resample_spacing=" << config.path_resample_spacing
            << " enable_path_tracking_guard=" << (config.enable_path_tracking_guard ? 1 : 0)
            << " path_tracking_lookahead_m=" << config.path_tracking_lookahead_m
            << " path_tracking_min_v=" << config.path_tracking_min_v << "\n";
  std::cout << "LT_DWA_WORKER_INPUT path_points=" << input.reference_path.size()
            << " nearest_path_index=" << nearest_index
            << " nearest_path_error=" << nearest_error
            << " goal_dist=" << goal_dist
            << " robot_v=" << input.robot_twist.v
            << " robot_w=" << input.robot_twist.w << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  const std::string mode = GetMode(argc, argv);

  if (mode == "health") {
    return Emit(lt_dwa_official_wrapper::WrapperStatus::kOk, "worker_health_check");
  }

  if (mode == "core-disabled") {
    return Emit(lt_dwa_official_wrapper::WrapperStatus::kCommandRejected,
                "official_core_call_disabled_in_worker_skeleton");
  }

  if (mode == "plan-request") {
    const std::string request_path = GetArg(argc, argv, "--request");
    if (request_path.empty()) {
      return Emit(lt_dwa_official_wrapper::WrapperStatus::kWaitingForInput,
                  "missing_request_path", 2);
    }

    const auto request = lt_dwa_official_wrapper::LoadWorkerRequestFile(request_path);
    if (!request.ok) {
      return Emit(lt_dwa_official_wrapper::WrapperStatus::kWaitingForInput,
                  "request_parse_failed_" + request.reason, 2);
    }

    lt_dwa_official_wrapper::PlannerConfig config = ConfigFromRequest(request);
    lt_dwa_official_wrapper::PlannerFacade facade(config);
    const auto output = facade.PlanOnce(request.input, request.now);
    return Emit(output.status, output.diagnostics.reject_reason);
  }

  if (mode == "official-core-once") {
    const std::string request_path = GetArg(argc, argv, "--request");
    if (request_path.empty()) {
      return Emit(lt_dwa_official_wrapper::WrapperStatus::kWaitingForInput,
                  "missing_request_path", 2);
    }

    const auto request = lt_dwa_official_wrapper::LoadWorkerRequestFile(request_path);
    if (!request.ok) {
      return Emit(lt_dwa_official_wrapper::WrapperStatus::kWaitingForInput,
                  "request_parse_failed_" + request.reason, 0.0, 0.0, -1, 2);
    }

    lt_dwa_official_wrapper::PlannerConfig config = ConfigFromRequest(request);
    EmitRuntimeDiagnostics(request.input, config);
    lt_dwa_official_wrapper::FrameValidator validator;
    const auto validation = validator.ValidateInput(request.input, config, request.now);
    if (!validation.ok()) {
      return Emit(validation.status, validation.reason, 0.0, 0.0, -1);
    }

#ifdef LT_DWA_WRAPPER_ENABLE_OFFICIAL_CORE
    if (!ros::isInitialized()) {
      ros::init(argc,
                argv,
                "lt_dwa_official_core_worker",
                ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    }
    const auto result = lt_dwa_official_wrapper::RunOfficialCoreOnce(
        request.input, config, OFFICIAL_LT_DWA_ROOT);
    return Emit(result.status, result.reason, result.command.v, result.command.w, result.core_return);
#else
    return Emit(lt_dwa_official_wrapper::WrapperStatus::kCommandRejected,
                "official_core_build_disabled", 0.0, 0.0, -1);
#endif
  }

  if (mode == "simulate-upstream-exit-zero") {
    std::exit(0);
  }

  if (mode == "simulate-crash") {
    std::cerr << "simulated worker crash before structured response\n";
    return 42;
  }

  if (mode == "simulate-hang") {
    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
  }

  return Emit(lt_dwa_official_wrapper::WrapperStatus::kCoreException,
              "unknown_worker_mode", 2);
}
