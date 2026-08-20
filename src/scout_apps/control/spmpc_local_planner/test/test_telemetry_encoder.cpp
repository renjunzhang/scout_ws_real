#include <gtest/gtest.h>

#include "spmpc_local_planner/ros/telemetry_encoder.h"

#include <ros/serialization.h>

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace {

std::string goldenPath() {
    std::string source = __FILE__;
    const std::string marker = "/test/test_telemetry_encoder.cpp";
    const auto offset = source.rfind(marker);
    EXPECT_NE(offset, std::string::npos);
    return source.substr(0, offset) +
        "/test/golden/control_cycle_audit_encoder.hex";
}

std::string readGolden() {
    std::ifstream input(goldenPath());
    EXPECT_TRUE(input.is_open());
    std::string golden;
    input >> golden;
    return golden;
}

std::string serializedHex(const ControlCycleAudit& msg) {
    const std::uint32_t length =
        ros::serialization::serializationLength(msg);
    std::vector<std::uint8_t> bytes(length);
    ros::serialization::OStream stream(bytes.data(), length);
    ros::serialization::serialize(stream, msg);
    std::ostringstream hex;
    hex << std::hex << std::setfill('0');
    for (const std::uint8_t byte : bytes) {
        hex << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return hex.str();
}

ControlCycleAuditDebug makeAuditFixture() {
    ControlCycleAuditDebug audit;
    audit.timing.cycle_id = 42;
    audit.timing.cycle_start_stamp_ns = 1000000002;
    audit.timing.raw_robot_state_stamp_ns = 2000000003;
    audit.timing.raw_liquid_state_stamp_ns = 3000000004;
    audit.timing.robot_state_stamp_ns = 4000000005;
    audit.timing.liquid_state_stamp_ns = 5000000006;
    audit.timing.solver_input_epoch_ns = 6000000007;
    audit.timing.solve_start_stamp_ns = 7000000008;
    audit.timing.solve_end_stamp_ns = 8000000009;
    audit.timing.horizon_available_stamp_ns = 9000000010;
    audit.timing.command_publish_stamp_ns = 10000000011;
    audit.timing.raw_state_skew_sec = 0.125;
    audit.timing.aligned_state_skew_sec = -0.25;
    audit.timing.state_alignment_required = true;
    audit.timing.state_time_aligned = false;
    audit.timing.robot_state_interpolated = true;
    audit.timing.robot_state_extrapolated = false;
    audit.timing.state_alignment_status = "INTERPOLATED";
    audit.variant = "B_slosh";
    audit.status = "TRACKING_UNSAFE_PROJECTION";
    audit.solver_status = "OK";
    audit.observer_source = 2;
    audit.solve_attempted = true;
    audit.solve_success = true;
    audit.command_accepted = false;
    audit.publish_cmd_vel = true;
    audit.command_was_published = true;
    audit.command_contract_violation = false;
    audit.terminal_phase = true;
    audit.terminal_controller_intervened = false;
    audit.safety_gate_intervened = true;
    audit.linear_limited = true;
    audit.angular_rate_limited = false;
    audit.angular_accel_limited = true;
    audit.solver_u0_a = 1.25;
    audit.solver_u0_alpha = -2.5;
    audit.planned_ax = 3.75;
    audit.planned_ay = -4.5;
    audit.solver_cmd_v = 5.25;
    audit.solver_cmd_omega = -6.5;
    audit.terminal_cmd_v = 7.75;
    audit.terminal_cmd_omega = -8.5;
    audit.post_gate_cmd_v = 9.25;
    audit.post_gate_cmd_omega = -10.5;
    audit.published_cmd_v = 11.75;
    audit.published_cmd_omega = -12.5;
    audit.previous_shifted_plan_available = true;
    audit.previous_plan_cycle_id = 41;
    audit.previous_shifted_plan_a = 13.25;
    audit.previous_shifted_plan_alpha = -14.5;
    audit.replanned_minus_shifted_a = 15.75;
    audit.replanned_minus_shifted_alpha = -16.5;
    audit.odom_excitation.valid = true;
    audit.odom_excitation.measurement_stamp_ns = 11000000012;
    audit.odom_excitation.accel_effective_stamp_ns = 12000000013;
    audit.odom_excitation.receive_stamp_ns = 13000000014;
    audit.odom_excitation.ax = 17.25;
    audit.odom_excitation.ay = -18.5;
    audit.odom_excitation.omega = 19.75;
    audit.odom_excitation.alpha = -20.5;
    audit.odom_excitation.sample_dt_sec = 0.03125;
    audit.imu_excitation.valid = false;
    audit.imu_excitation.measurement_stamp_ns = 14000000015;
    audit.imu_excitation.accel_effective_stamp_ns = 15000000016;
    audit.imu_excitation.receive_stamp_ns = 16000000017;
    audit.imu_excitation.ax = 21.25;
    audit.imu_excitation.ay = -22.5;
    audit.imu_excitation.omega = 23.75;
    audit.imu_excitation.alpha = -24.5;
    audit.imu_excitation.sample_dt_sec = 0.0625;
    return audit;
}

TEST(TelemetryEncoderTest, PreservesCommandInterventionSchema) {
    CommandInterventionDebug debug;
    debug.solver_cmd_v = 1.0;
    debug.solver_cmd_omega = 2.0;
    debug.post_gate_cmd_v = 3.0;
    debug.post_gate_cmd_omega = 4.0;
    debug.published_cmd_v = 5.0;
    debug.published_cmd_omega = 6.0;
    debug.output_success = true;
    debug.zero_due_to_waiting_for_odom = true;
    debug.zero_due_to_waiting_for_tf = true;
    debug.zero_due_to_terminal_spin_fail = true;
    debug.zero_due_to_command_contract = true;
    debug.angular_rate_limited = true;
    debug.publish_cmd_vel = true;

    const std_msgs::Float32MultiArray msg =
        encodeCommandIntervention(debug);
    ASSERT_EQ(1u, msg.layout.dim.size());
    EXPECT_EQ(19u, msg.layout.dim[0].size);
    EXPECT_EQ(19u, msg.layout.dim[0].stride);
    ASSERT_EQ(19u, msg.data.size());
    for (std::size_t index = 0; index < 6; ++index) {
        EXPECT_FLOAT_EQ(static_cast<float>(index + 1), msg.data[index]);
    }
    const std::vector<float> expected_flags = {
        1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
    for (std::size_t index = 0; index < expected_flags.size(); ++index) {
        EXPECT_FLOAT_EQ(expected_flags[index], msg.data[index + 6]);
    }
}

TEST(TelemetryEncoderTest, MatchesFrozenControlCycleAuditWireImage) {
    const ControlCycleAudit msg = encodeControlCycleAudit(
        makeAuditFixture(), "odom");
    EXPECT_EQ(2u, msg.schema_version);
    EXPECT_EQ("odom", msg.header.frame_id);
    EXPECT_EQ(10000000011u, msg.header.stamp.toNSec());
    EXPECT_EQ(42u, msg.cycle_id);
    EXPECT_EQ("INTERPOLATED", msg.state_alignment_status);
    EXPECT_EQ("B_slosh", msg.variant);
    EXPECT_EQ("TRACKING_UNSAFE_PROJECTION", msg.status);
    EXPECT_DOUBLE_EQ(-24.5, msg.imu_alpha);

    const std::string actual = serializedHex(msg);
    EXPECT_EQ(readGolden(), actual) << "actual=" << actual;
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
