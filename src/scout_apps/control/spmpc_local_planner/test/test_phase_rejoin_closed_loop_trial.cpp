#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace spmpc_local_planner {
namespace simulation {
namespace closed_loop_trial {
int runPhaseRejoinClosedLoopTrial(int argc, char** argv);
}  // namespace closed_loop_trial
}  // namespace simulation
}  // namespace spmpc_local_planner

namespace {

namespace manifest =
    spmpc_local_planner::delay_augmented_phase_solver_manifest;

std::string number(double value) {
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(17) << value;
    return output.str();
}

spmpc_local_planner::EmpiricalRecoveryRadii unitRadii() {
    spmpc_local_planner::EmpiricalRecoveryRadii radii;
    radii.x = 1.0;
    radii.y = 1.0;
    radii.yaw = 1.0;
    radii.v = 1.0;
    radii.omega = 1.0;
    radii.eta_x = 1.0;
    radii.eta_x_dot = 1.0;
    radii.eta_y = 1.0;
    radii.eta_y_dot = 1.0;
    return radii;
}

spmpc_local_planner::ExecutionCompatibilityBounds unitBounds() {
    spmpc_local_planner::ExecutionCompatibilityBounds bounds;
    bounds.valid = true;
    bounds.linear_actuator_output = 1.0;
    bounds.angular_actuator_output = 1.0;
    bounds.linear_pending_commands.assign(
        static_cast<std::size_t>(manifest::kLinearBufferCount), 1.0);
    bounds.angular_pending_commands.assign(
        static_cast<std::size_t>(manifest::kAngularBufferCount), 1.0);
    return bounds;
}

std::vector<spmpc_local_planner::PhaseNominalSample> zeroSamples() {
    std::vector<spmpc_local_planner::PhaseNominalSample> samples(24);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        auto& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * manifest::kDt;
        sample.s = 0.09;
        sample.x = 0.09;
        sample.radii = unitRadii();
        sample.augmented_execution_valid = true;
        sample.augmented_execution.valid = true;
        sample.augmented_execution.stage_index = index;
        sample.augmented_execution.robot.x = 0.09;
        sample.augmented_execution.linear.pending_commands.assign(
            static_cast<std::size_t>(manifest::kLinearBufferCount), 0.0);
        sample.augmented_execution.angular.pending_commands.assign(
            static_cast<std::size_t>(manifest::kAngularBufferCount), 0.0);
        sample.execution_bounds = unitBounds();
    }
    return samples;
}

std::map<std::string, std::string> metadata() {
    const auto compiled = spmpc_local_planner::
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    spmpc_local_planner::SloshDynamics dynamics;
    EXPECT_TRUE(dynamics.configure(compiled.slosh));
    const double omega_n = dynamics.omegaN();
    const auto recovery = spmpc_local_planner::
        boundedTrackingRecoveryPolicyV1Params();
    return {
        {"schema", "phase_rejoin_empirical_augmented_v3"},
        {"evidence_level", "empirical_held_out"},
        {"source", "unit_test_zero_nominal"},
        {"contract_id", "closed_loop_trial_unit_test_v1"},
        {"frame_id", "map"},
        {"dt", number(manifest::kDt)},
        {"path_length", "0.09"},
        {"terminal_contract", "publish_zero_settle_hold_v2"},
        {"recovery_contract", recovery.contract_id},
        {"recovery_policy_longitudinal_position_gain",
         number(recovery.longitudinal_position_gain)},
        {"recovery_policy_lateral_position_gain",
         number(recovery.lateral_position_gain)},
        {"recovery_policy_yaw_gain", number(recovery.yaw_gain)},
        {"recovery_policy_linear_velocity_gain",
         number(recovery.linear_velocity_gain)},
        {"recovery_policy_angular_velocity_gain",
         number(recovery.angular_velocity_gain)},
        {"recovery_policy_max_residual_v",
         number(recovery.max_residual_v)},
        {"recovery_policy_max_residual_omega",
         number(recovery.max_residual_omega)},
        {"recovery_policy_published_linear_min",
         number(recovery.published_linear_min)},
        {"recovery_policy_published_linear_max",
         number(recovery.published_linear_max)},
        {"recovery_policy_published_angular_min",
         number(recovery.published_angular_min)},
        {"recovery_policy_published_angular_max",
         number(recovery.published_angular_max)},
        {"terminal_zero_hold_steps", "11"},
        {"terminal_eta_norm_max", "1e-9"},
        {"terminal_eta_dot_norm_max", "1e-9"},
        {"terminal_v_abs_max", "1e-9"},
        {"terminal_omega_abs_max", "1e-9"},
        {"terminal_linear_actuator_output_abs_max", "1e-9"},
        {"terminal_angular_actuator_output_abs_max", "1e-9"},
        {"terminal_linear_pending_command_abs_max", "1e-9"},
        {"terminal_angular_pending_command_abs_max", "1e-9"},
        {"two_zeta_omega_n",
         number(2.0 * compiled.slosh.damping_ratio * omega_n)},
        {"omega_n_sq", number(omega_n * omega_n)},
        {"kappa_x", "1"},
        {"kappa_y", "1"},
        {"dynamics_tolerance", "1e-9"},
        {"execution_contract_id", compiled.execution.contract_id},
        {"execution_contract_hash", compiled.execution.contract_hash},
        {"execution_state_width", std::to_string(compiled.state_width)},
        {"execution_linear_buffer_count",
         std::to_string(manifest::kLinearBufferCount)},
        {"execution_angular_buffer_count",
         std::to_string(manifest::kAngularBufferCount)},
        {"parameter_schema_version",
         std::to_string(compiled.parameter_schema_version)},
        {"parameter_schema_id", compiled.parameter_schema_id},
        {"parameter_schema_hash", compiled.parameter_schema_hash},
        {"recovery_artifact_hash", std::string(64, '0')},
        {"execution_compatibility_contract",
         compiled.execution_compatibility_contract},
    };
}

class TrialRunnerTest : public ::testing::Test {
protected:
    void SetUp() override {
        char pattern[] = "/tmp/spmpc_closed_loop_trial_XXXXXX";
        char* created = ::mkdtemp(pattern);
        ASSERT_NE(created, nullptr);
        root_ = created;
        path_ = root_ + "/path.json";
        artifact_ = root_ + "/artifact.csv";
        condition_ = root_ + "/condition.yaml";
        cycle_ = root_ + "/cycles.csv";
        summary_ = root_ + "/summary.json";
        {
            std::ofstream output(path_);
            output << "{\"frame_id\":\"map\",\"poses\":["
                   << "{\"x\":0,\"y\":0,\"qx\":0,\"qy\":0,"
                   << "\"qz\":0,\"qw\":1},"
                   << "{\"x\":0.09,\"y\":0,\"qx\":0,\"qy\":0,"
                   << "\"qz\":0,\"qw\":1}]}\n";
        }
        auto samples = zeroSamples();
        auto entries = metadata();
        entries["recovery_artifact_hash"] = spmpc_local_planner::
            NominalSequenceArtifact::canonicalRecoveryArtifactHash(
                entries, samples);
        spmpc_local_planner::NominalSequenceArtifact artifact;
        const auto assigned = artifact.assignValidated(
            entries, samples, "<closed-loop-trial-test>");
        ASSERT_TRUE(assigned.success)
            << assigned.status << ": " << assigned.detail;
        const auto written = artifact.writeCanonicalCsv(artifact_, false);
        ASSERT_TRUE(written.success)
            << written.status << ": " << written.detail;
    }

    void TearDown() override {
        for (const std::string& path : {
                 path_, artifact_, condition_, cycle_, summary_}) {
            if (!path.empty()) ::unlink(path.c_str());
        }
        if (!root_.empty()) ::rmdir(root_.c_str());
    }

    void writeCondition(const std::string& id,
                        const std::string& mode,
                        const std::string& implementation,
                        bool offline,
                        bool residual,
                        double max_motion_sec = 2.0) {
        std::ofstream output(condition_);
        output
            << "schema: spmpc_closed_loop_trial_condition_v1\n"
            << "condition_id: " << id << "\n"
            << "implementation_id: " << implementation << "\n"
            << "implementation_complete: true\n"
            << "mode: " << mode << "\n"
            << "offline_nominal: " << (offline ? "true" : "false") << "\n"
            << "online_residual: " << (residual ? "true" : "false") << "\n"
            << "recovery_gate: false\n"
            << "execution_compatibility_gate: false\n"
            << "stored_recovery_action: false\n"
            << "input_shaping: false\n"
            << "trial:\n"
            << "  control_rate_hz: 30.0\n"
            << "  max_motion_sec: " << max_motion_sec << "\n"
            << "  fixed_tail_sec: 4.0\n"
            << "  publish_latency_sec: 0.01\n";
    }

    void writeC4Condition() {
        std::ofstream output(condition_);
        output
            << "schema: spmpc_closed_loop_trial_condition_v1\n"
            << "condition_id: C4\n"
            << "implementation_id: test_delay_augmented_c4_v1\n"
            << "implementation_complete: true\n"
            << "mode: phase_rejoin_full\n"
            << "offline_nominal: true\n"
            << "online_residual: true\n"
            << "recovery_gate: true\n"
            << "execution_compatibility_gate: true\n"
            << "stored_recovery_action: true\n"
            << "input_shaping: false\n"
            << "residual_feedback:\n"
            << "  max_residual_v: 0.08\n"
            << "  max_residual_omega: 0.20\n"
            << "trial:\n"
            << "  control_rate_hz: 30.0\n"
            << "  max_motion_sec: 2.0\n"
            << "  fixed_tail_sec: 4.0\n"
            << "  publish_latency_sec: 0.01\n";
    }

    void writeIsCondition() {
        std::ofstream output(condition_);
        output
            << "schema: spmpc_closed_loop_trial_condition_v1\n"
            << "condition_id: IS\n"
            << "implementation_id: test_zvd_30hz_v1\n"
            << "implementation_complete: true\n"
            << "mode: input_shaping\n"
            << "offline_nominal: false\n"
            << "online_residual: false\n"
            << "recovery_gate: false\n"
            << "execution_compatibility_gate: false\n"
            << "stored_recovery_action: false\n"
            << "input_shaping: true\n"
            << "input_shaper:\n"
            << "  max_discrete_residual: 0.03\n"
            << "trial:\n"
            << "  control_rate_hz: 30.0\n"
            << "  max_motion_sec: 2.0\n"
            << "  fixed_tail_sec: 4.0\n"
            << "  publish_latency_sec: 0.01\n";
    }

    int run() {
        std::vector<std::string> values = {
            "trial", "--plant", SPMPC_SIMULATION_CONFIG_PATH,
            "--path", path_, "--artifact", artifact_,
            "--condition", condition_, "--seed", "1234",
            "--cycle-csv", cycle_, "--summary-json", summary_,
        };
        std::vector<char*> argv;
        for (std::string& value : values) argv.push_back(&value[0]);
        return spmpc_local_planner::simulation::closed_loop_trial::
            runPhaseRejoinClosedLoopTrial(
                static_cast<int>(argv.size()), argv.data());
    }

    std::string root_;
    std::string path_;
    std::string artifact_;
    std::string condition_;
    std::string cycle_;
    std::string summary_;
};

TEST_F(TrialRunnerTest,
       C2WritesTruthIsolatedTrialMetricAndRefusesOverwrite) {
    writeCondition("C2", "offline_replay", "test_c2_v1", true, false);
    ASSERT_EQ(run(), 0);
    const YAML::Node summary = YAML::LoadFile(summary_);
    EXPECT_EQ(summary["status"].as<std::string>(), "TRIAL_COMPLETE");
    EXPECT_TRUE(summary["implementation_complete"].as<bool>());
    EXPECT_FALSE(summary["physical_parameter_claim"].as<bool>());
    EXPECT_FALSE(summary["plant_truth_visible_to_controller"].as<bool>());
    EXPECT_FALSE(summary["external_liquid_truth_used_for_control"].as<bool>());
    EXPECT_EQ(summary["primary_metric"]["window"].as<std::string>(),
              "motion_plus_fixed_tail");
    EXPECT_GT(summary["primary_metric"]["sample_count"].as<int>(), 100);
    EXPECT_GE(summary["primary_metric"]["value_m"].as<double>(), 0.0);
    struct stat before;
    ASSERT_EQ(::stat(summary_.c_str(), &before), 0);
    EXPECT_EQ(run(), 3);
    struct stat after;
    ASSERT_EQ(::stat(summary_.c_str(), &after), 0);
    EXPECT_EQ(before.st_size, after.st_size);
}

TEST_F(TrialRunnerTest,
       C3RunsWithoutGateButIsExplicitlyPilotOnly) {
    writeCondition("C3", "residual_no_gate", "test_c3_pilot_v1",
                   true, true);
    ASSERT_EQ(run(), 0);
    const YAML::Node summary = YAML::LoadFile(summary_);
    EXPECT_TRUE(summary["baseline_contract"]["pilot_only"].as<bool>());
    EXPECT_FALSE(summary["baseline_contract"]
                        ["formal_c3_c4_causal_comparison_ready"].as<bool>());
    EXPECT_TRUE(summary["baseline_contract"]
                       ["c3_does_not_widen_gate_radii"].as<bool>());
    EXPECT_EQ(summary["controller_audit"]["gate_evaluations"].as<int>(), 0);
}

TEST_F(TrialRunnerTest,
       OfflineConditionsRejectMotionWindowShorterThanArtifactTail) {
    writeCondition("C2", "offline_replay", "test_c2_short_window_v1",
                   true, false, 0.1);
    EXPECT_EQ(run(), 3);
    struct stat info;
    EXPECT_NE(::stat(cycle_.c_str(), &info), 0);
    EXPECT_NE(::stat(summary_.c_str(), &info), 0);
}

TEST_F(TrialRunnerTest,
       C4InvokesCompiled22DGateAndFinalCommandTransaction) {
    if (!spmpc_local_planner::DelayAugmentedPhaseAcadosSolver::compiled()) {
        GTEST_SKIP() << "delay-augmented development capsule is not enabled";
    }
    writeC4Condition();
    ASSERT_EQ(run(), 0);
    const YAML::Node summary = YAML::LoadFile(summary_);
    EXPECT_EQ(summary["condition_id"].as<std::string>(), "C4");
    EXPECT_GT(summary["controller_audit"]["gate_evaluations"].as<int>(), 0);
    EXPECT_GT(summary["controller_audit"]["publications"].as<int>(), 0);
    EXPECT_EQ(summary["controller_audit"]["publication_failures"].as<int>(),
              0);
}

TEST_F(TrialRunnerTest,
       C0InvokesProductionContinuousMpccWithoutSolverFailures) {
    writeCondition("C0", "ordinary_mpcc", "test_c0_v1", false, false);
    ASSERT_EQ(run(), 0);
    const YAML::Node summary = YAML::LoadFile(summary_);
    EXPECT_EQ(summary["controller_audit"]["solver_failures"].as<int>(), 0);
    EXPECT_GT(summary["controller_audit"]["publications"].as<int>(), 0);
}

TEST_F(TrialRunnerTest,
       InputShapingRunsQuantizedZvdAndReportsDiscreteSelfTest) {
    writeIsCondition();
    ASSERT_EQ(run(), 0);
    const YAML::Node summary = YAML::LoadFile(summary_);
    EXPECT_TRUE(summary["baseline_contract"]
                       ["is_zvd_single_mode_self_test_passed"].as<bool>());
    EXPECT_EQ(summary["baseline_contract"]["is_zvd_delay_steps"][1].as<int>(),
              3);
    EXPECT_LT(summary["baseline_contract"]
                     ["is_zvd_discrete_single_mode_residual"].as<double>(),
              0.03);
}

TEST(PhaseRejoinClosedLoopConditionAssets,
     AllSixImplementationsAreExplicitAndComplete) {
    const std::vector<std::string> files = {
        "C0_ordinary_mpcc.yaml", "C1_smooth_match_mpcc.yaml",
        "C2_offline_replay.yaml", "C3_residual_no_gate.yaml",
        "C4_phase_rejoin_full.yaml", "IS_zvd_input_shaping.yaml",
    };
    std::map<std::string, std::string> implementations;
    for (const std::string& file : files) {
        const YAML::Node root = YAML::LoadFile(
            std::string(SPMPC_SIMULATION_CONDITIONS_DIR) + "/" + file);
        ASSERT_EQ(root["schema"].as<std::string>(),
                  "spmpc_closed_loop_trial_condition_v1");
        ASSERT_TRUE(root["implementation_complete"].as<bool>());
        const std::string id = root["condition_id"].as<std::string>();
        const std::string implementation =
            root["implementation_id"].as<std::string>();
        EXPECT_FALSE(id.empty());
        EXPECT_FALSE(implementation.empty());
        EXPECT_TRUE(implementations.emplace(id, implementation).second);
    }
    EXPECT_EQ(implementations.size(), 6u);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
