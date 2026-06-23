#include <chrono>
#include <cstdlib>
#include <iostream>
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

    lt_dwa_official_wrapper::PlannerConfig config;
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

    lt_dwa_official_wrapper::PlannerConfig config;
    if (!request.input.planning_frame.empty()) {
      config.planning_frame = request.input.planning_frame;
    }
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
