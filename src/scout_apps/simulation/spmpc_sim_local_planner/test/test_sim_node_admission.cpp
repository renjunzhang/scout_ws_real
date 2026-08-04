#include "spmpc_sim_local_planner/ros/sim_node_admission.h"

#include <gtest/gtest.h>

namespace spmpc_sim_local_planner {
namespace {

SimNodeAdmissionInput validInput() {
    SimNodeAdmissionInput input;
    input.gate_token = std::string(64U, 'a');
    input.parameter_gate_token = input.gate_token;
    input.ros_master_uri = "http://127.0.0.1:17540";
    input.use_sim_time = true;
    input.target_id = kSimNodeAdmissionTargetId;
    input.gate_id = kSimNodeAdmissionGateId;
    input.environment_owner_package = kSimNodeAdmissionEnvironmentOwnerPackage;
    input.launch_marker = true;
    input.release_ack = true;
    return input;
}

TEST(SimNodeAdmission, AcceptsExactGateHandoff) {
    std::string reason;
    EXPECT_TRUE(validateSimNodeAdmission(validInput(), reason));
    EXPECT_TRUE(reason.empty());
}

TEST(SimNodeAdmission, RejectsDirectInvocationWithoutGateToken) {
    SimNodeAdmissionInput input = validInput();
    input.gate_token.clear();
    input.parameter_gate_token.clear();
    std::string reason;
    EXPECT_FALSE(validateSimNodeAdmission(input, reason));
    EXPECT_NE(std::string::npos, reason.find("GATE_HASH"));
}

TEST(SimNodeAdmission, RejectsRealMasterOrNonSimulationTime) {
    SimNodeAdmissionInput input = validInput();
    input.ros_master_uri = "http://192.168.1.5:11311";
    input.use_sim_time = false;
    std::string reason;
    EXPECT_FALSE(validateSimNodeAdmission(input, reason));
    EXPECT_NE(std::string::npos, reason.find("loopback"));
    EXPECT_NE(std::string::npos, reason.find("use_sim_time"));
}

TEST(SimNodeAdmission, RejectsUnboundOrUnacknowledgedGate) {
    SimNodeAdmissionInput input = validInput();
    input.parameter_gate_token = std::string(64U, 'b');
    input.target_id = "not-the-sim-target";
    input.environment_owner_package = "not-a-simulation-environment";
    input.release_ack = false;
    std::string reason;
    EXPECT_FALSE(validateSimNodeAdmission(input, reason));
    EXPECT_NE(std::string::npos, reason.find("differ"));
    EXPECT_NE(std::string::npos, reason.find("target_id"));
    EXPECT_NE(std::string::npos, reason.find("owner_package"));
    EXPECT_NE(std::string::npos, reason.find("release_ack"));
}

}  // namespace
}  // namespace spmpc_sim_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
