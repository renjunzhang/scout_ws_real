#include "spmpc_local_planner/phase_rejoin/empirical_joint_successor_admission.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

PhaseNominalSample validTarget() {
    PhaseNominalSample target;
    target.index = 23;
    target.x = 1.0;
    target.y = -0.5;
    target.yaw = 0.2;
    target.v = 0.4;
    target.omega = -0.1;
    target.eta_x = 0.03;
    target.eta_x_dot = -0.02;
    target.eta_y = 0.01;
    target.eta_y_dot = 0.04;
    target.radii.x = target.radii.y = target.radii.yaw = 1.0;
    target.radii.v = target.radii.omega = 1.0;
    target.radii.eta_x = target.radii.eta_x_dot = 1.0;
    target.radii.eta_y = target.radii.eta_y_dot = 1.0;

    target.augmented_execution_valid = true;
    target.augmented_execution.valid = true;
    target.augmented_execution.stage_index = target.index;
    target.augmented_execution.linear.actuator_output = 0.2;
    target.augmented_execution.angular.actuator_output = -0.1;
    target.augmented_execution.linear.pending_commands = {0.1, 0.2};
    target.augmented_execution.angular.pending_commands = {-0.1, 0.0};
    target.execution_bounds.valid = true;
    target.execution_bounds.linear_actuator_output = 1.0;
    target.execution_bounds.angular_actuator_output = 1.0;
    target.execution_bounds.linear_pending_commands = {1.0, 1.0};
    target.execution_bounds.angular_pending_commands = {1.0, 1.0};
    return target;
}

ExecutionAugmentedState validSuccessor(const PhaseNominalSample& target) {
    ExecutionAugmentedState successor = target.augmented_execution;
    successor.valid = true;
    successor.robot.x = target.x;
    successor.robot.y = target.y;
    successor.robot.yaw = target.yaw;
    successor.robot.v = target.v;
    successor.robot.omega = target.omega;
    successor.slosh.eta_x = target.eta_x;
    successor.slosh.eta_x_dot = target.eta_x_dot;
    successor.slosh.eta_y = target.eta_y;
    successor.slosh.eta_y_dot = target.eta_y_dot;
    return successor;
}

TEST(EmpiricalJointSuccessorAdmission, AcceptsJointlyWhenBothGatesPass) {
    const PhaseNominalSample target = validTarget();
    const ExecutionAugmentedState successor = validSuccessor(target);

    const EmpiricalJointSuccessorAdmissionResult result =
        EmpiricalJointSuccessorAdmission().evaluate(target, successor);

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.target_index, 23u);
    EXPECT_TRUE(result.empirical_gate.accepted);
    EXPECT_TRUE(result.execution_gate.accepted);
    EXPECT_EQ(result.status, "ACCEPTED");
}

TEST(EmpiricalJointSuccessorAdmission, RejectsNineDimensionalEmpiricalGate) {
    const PhaseNominalSample target = validTarget();
    ExecutionAugmentedState successor = validSuccessor(target);
    successor.robot.x += 2.0;

    const EmpiricalJointSuccessorAdmissionResult result =
        EmpiricalJointSuccessorAdmission().evaluate(target, successor);

    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.empirical_gate.accepted);
    EXPECT_TRUE(result.execution_gate.accepted);
    EXPECT_EQ(result.status, "REJECTED_EMPIRICAL_9D");
}

TEST(EmpiricalJointSuccessorAdmission,
     RejectsExecutionCompatibilityForPendingQueue) {
    const PhaseNominalSample target = validTarget();
    ExecutionAugmentedState successor = validSuccessor(target);
    successor.linear.pending_commands.front() = 2.0;

    const EmpiricalJointSuccessorAdmissionResult result =
        EmpiricalJointSuccessorAdmission().evaluate(target, successor);

    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.accepted);
    EXPECT_TRUE(result.empirical_gate.accepted);
    EXPECT_FALSE(result.execution_gate.accepted);
    EXPECT_EQ(result.status, "REJECTED_EXECUTION_COMPATIBILITY");
}

TEST(EmpiricalJointSuccessorAdmission, RejectsInvalidSuccessor) {
    const PhaseNominalSample target = validTarget();
    ExecutionAugmentedState successor = validSuccessor(target);
    successor.valid = false;

    const EmpiricalJointSuccessorAdmissionResult result =
        EmpiricalJointSuccessorAdmission().evaluate(target, successor);

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.status, "INVALID_SUCCESSOR");
    EXPECT_FALSE(result.execution_gate.valid);
}

TEST(EmpiricalJointSuccessorAdmission, RejectsInvalidTargetAsset) {
    PhaseNominalSample target = validTarget();
    target.augmented_execution_valid = false;
    const ExecutionAugmentedState successor = validSuccessor(target);

    const EmpiricalJointSuccessorAdmissionResult result =
        EmpiricalJointSuccessorAdmission().evaluate(target, successor);

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.status, "INVALID_TARGET_ASSET");
    EXPECT_TRUE(result.execution_gate.valid);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
