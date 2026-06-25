#include <gtest/gtest.h>

#include <fcntl.h>
#include <unistd.h>

#include <fstream>
#include <string>

#include "lt_dwa_official_wrapper/worker_protocol.hpp"
#include "lt_dwa_official_wrapper/worker_request.hpp"
#include "lt_dwa_official_wrapper/worker_supervisor.hpp"

#ifndef TEST_WORKER_EXECUTABLE
#define TEST_WORKER_EXECUTABLE ""
#endif

#ifndef TEST_WORKER_FIXTURE_PATH
#define TEST_WORKER_FIXTURE_PATH ""
#endif

namespace lt_dwa_official_wrapper {
namespace {

const char* WorkerExecutable() {
  return TEST_WORKER_EXECUTABLE;
}

const char* WorkerFixturePath() {
  return TEST_WORKER_FIXTURE_PATH;
}

Pose2d MakePose(double x, double y, const std::string& frame = "odom") {
  Pose2d pose;
  pose.frame_id = frame;
  pose.stamp = ros::Time(40.0);
  pose.x = x;
  pose.y = y;
  pose.yaw = 0.0;
  return pose;
}

PlannerInput MakePlanRequestInput() {
  PlannerInput input;
  input.planning_frame = "odom";
  input.stamp = ros::Time(40.0);
  input.robot_pose = MakePose(0.0, 0.0);
  input.robot_twist.v = 0.2;
  input.robot_twist.w = 0.1;
  input.target_pose = MakePose(1.0, 0.0);
  input.reference_path = {MakePose(0.0, 0.0), MakePose(0.5, 0.0), MakePose(1.0, 0.0)};
  input.occupancy_grid.header.frame_id = "odom";
  input.occupancy_grid.header.stamp = ros::Time(40.0);
  input.occupancy_grid.info.width = 3;
  input.occupancy_grid.info.height = 3;
  input.occupancy_grid.info.resolution = 0.1;
  input.occupancy_grid.info.origin.orientation.w = 1.0;
  input.occupancy_grid.data.assign(9, 0);
  return input;
}

std::string WriteTempRequest(const PlannerInput& input) {
  char path[] = "/tmp/lt_dwa_worker_request_XXXXXX";
  const int fd = mkstemp(path);
  EXPECT_GE(fd, 0);
  close(fd);
  std::ofstream out(path);
  out << SerializeWorkerRequest(input, ros::Time(40.1));
  out.close();
  return std::string(path);
}

TEST(WorkerProtocolTest, ParsesStructuredResponse) {
  const std::string text = FormatWorkerResponse(WrapperStatus::kCommandRejected,
                                                "official_core_call_disabled");

  const auto response = ParseWorkerResponse(text);

  EXPECT_TRUE(response.valid);
  EXPECT_EQ(response.status, WrapperStatus::kCommandRejected);
  EXPECT_EQ(response.reason, "official_core_call_disabled");
}

TEST(WorkerProtocolTest, ParsesStructuredResponseWithCommandFields) {
  const std::string text = FormatWorkerResponse(WrapperStatus::kOk,
                                                "official_core_ok",
                                                0.2,
                                                -0.1,
                                                0);

  const auto response = ParseWorkerResponse(text);

  EXPECT_TRUE(response.valid);
  EXPECT_EQ(response.status, WrapperStatus::kOk);
  EXPECT_TRUE(response.has_command);
  EXPECT_DOUBLE_EQ(response.command_v, 0.2);
  EXPECT_DOUBLE_EQ(response.command_w, -0.1);
  EXPECT_TRUE(response.has_final_command);
  EXPECT_DOUBLE_EQ(response.final_command_v, 0.2);
  EXPECT_DOUBLE_EQ(response.final_command_w, -0.1);
  EXPECT_TRUE(response.has_core_return);
  EXPECT_EQ(response.core_return, 0);
}

TEST(WorkerProtocolTest, ParsesStructuredResponseWithRawAndFinalCommandFields) {
  const std::string text = FormatWorkerResponse(WrapperStatus::kOk,
                                                "official_core_ok_path_tracking_guard",
                                                0.1,
                                                -0.05,
                                                0.2,
                                                -0.1,
                                                true,
                                                "path_tracking_guard",
                                                0);

  const auto response = ParseWorkerResponse(text);

  EXPECT_TRUE(response.valid);
  EXPECT_EQ(response.status, WrapperStatus::kOk);
  EXPECT_TRUE(response.has_raw_command);
  EXPECT_DOUBLE_EQ(response.raw_command_v, 0.1);
  EXPECT_DOUBLE_EQ(response.raw_command_w, -0.05);
  EXPECT_TRUE(response.has_final_command);
  EXPECT_DOUBLE_EQ(response.final_command_v, 0.2);
  EXPECT_DOUBLE_EQ(response.final_command_w, -0.1);
  EXPECT_TRUE(response.has_command);
  EXPECT_DOUBLE_EQ(response.command_v, 0.2);
  EXPECT_DOUBLE_EQ(response.command_w, -0.1);
  EXPECT_TRUE(response.guard_applied);
  EXPECT_EQ(response.guard_reason, "path_tracking_guard");
  EXPECT_TRUE(response.has_core_return);
  EXPECT_EQ(response.core_return, 0);
}

TEST(WorkerProtocolTest, MissingStructuredResponseMapsToCoreProcessExited) {
  const auto response = ParseWorkerResponse("plain process output without status\n");

  EXPECT_FALSE(response.valid);
  EXPECT_EQ(response.status, WrapperStatus::kCoreProcessExited);
}

TEST(WorkerSupervisorTest, HealthCheckReturnsOk) {
  WorkerSupervisor supervisor;

  const auto result = supervisor.Run(WorkerExecutable(), {"--mode", "health"}, 1.0);

  EXPECT_TRUE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kOk);
  EXPECT_EQ(result.exit_code, 0);
  EXPECT_EQ(result.term_signal, 0);
  EXPECT_FALSE(result.timed_out);
}

TEST(WorkerSupervisorTest, CoreDisabledReturnsStructuredCommandRejected) {
  WorkerSupervisor supervisor;

  const auto result = supervisor.Run(WorkerExecutable(), {"--mode", "core-disabled"}, 1.0);

  EXPECT_TRUE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kCommandRejected);
  EXPECT_EQ(result.exit_code, 0);
  EXPECT_NE(result.reason.find("official_core_call_disabled"), std::string::npos);
}

TEST(WorkerSupervisorTest, PlanRequestValidInputReturnsCoreDisabledCommandRejected) {
  WorkerSupervisor supervisor;
  const std::string path = WriteTempRequest(MakePlanRequestInput());

  const auto result = supervisor.Run(WorkerExecutable(), {"--mode", "plan-request", "--request", path}, 1.0);
  unlink(path.c_str());

  EXPECT_TRUE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kCommandRejected);
  EXPECT_NE(result.reason.find("core_call_disabled"), std::string::npos);
}

TEST(WorkerSupervisorTest, FrozenFixturePlanRequestReturnsCoreDisabledCommandRejected) {
  WorkerSupervisor supervisor;

  const auto result = supervisor.Run(
      WorkerExecutable(), {"--mode", "plan-request", "--request", WorkerFixturePath()}, 1.0);

  EXPECT_TRUE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kCommandRejected);
  EXPECT_NE(result.reason.find("core_call_disabled"), std::string::npos);
}

TEST(WorkerSupervisorTest, PlanRequestInvalidFrameReturnsInvalidFrame) {
  WorkerSupervisor supervisor;
  auto input = MakePlanRequestInput();
  input.occupancy_grid.header.frame_id = "map";
  const std::string path = WriteTempRequest(input);

  const auto result = supervisor.Run(WorkerExecutable(), {"--mode", "plan-request", "--request", path}, 1.0);
  unlink(path.c_str());

  EXPECT_TRUE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kInvalidFrame);
}

TEST(WorkerSupervisorTest, PlanRequestMissingFileReturnsWaitingForInput) {
  WorkerSupervisor supervisor;

  const auto result = supervisor.Run(WorkerExecutable(), {"--mode", "plan-request", "--request", "/tmp/lt_dwa_missing_request"}, 1.0);

  EXPECT_TRUE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kWaitingForInput);
  EXPECT_EQ(result.exit_code, 2);
}

TEST(WorkerSupervisorTest, ZeroExitWithoutResponseMapsToCoreProcessExited) {
  WorkerSupervisor supervisor;

  const auto result = supervisor.Run(WorkerExecutable(), {"--mode", "simulate-upstream-exit-zero"}, 1.0);

  EXPECT_FALSE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kCoreProcessExited);
  EXPECT_EQ(result.exit_code, 0);
  EXPECT_FALSE(result.timed_out);
}

TEST(WorkerSupervisorTest, NonzeroCrashWithoutResponseMapsToCoreProcessExited) {
  WorkerSupervisor supervisor;

  const auto result = supervisor.Run(WorkerExecutable(), {"--mode", "simulate-crash"}, 1.0);

  EXPECT_FALSE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kCoreProcessExited);
  EXPECT_EQ(result.exit_code, 42);
  EXPECT_FALSE(result.timed_out);
}

TEST(WorkerSupervisorTest, TimeoutKillsWorkerAndMapsToCoreProcessExited) {
  WorkerSupervisor supervisor;

  const auto result = supervisor.Run(WorkerExecutable(), {"--mode", "simulate-hang"}, 0.05);

  EXPECT_FALSE(result.valid_response);
  EXPECT_EQ(result.status, WrapperStatus::kCoreProcessExited);
  EXPECT_TRUE(result.timed_out);
}

}  // namespace
}  // namespace lt_dwa_official_wrapper

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
