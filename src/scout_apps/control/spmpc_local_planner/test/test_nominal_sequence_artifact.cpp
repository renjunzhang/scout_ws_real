#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"
#include "../generated/acados/spmpc_delay_augmented_phase_solver_manifest.h"
#include "phase_rejoin_artifact_fixture.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace spmpc_local_planner {
namespace {

namespace augmented_manifest = delay_augmented_phase_solver_manifest;

std::string artifactText(int count = 20,
                         bool negative_radius = false,
                         bool nonmonotonic_time = false,
                         const std::vector<double>& sample_times = {}) {
    std::ostringstream out;
    out << "# schema=phase_rejoin_empirical_v1\n"
        << "# evidence_level=development_only\n"
        << "# source=unit_test\n"
        << "# contract_id=test_contract\n"
        << "# frame_id=map\n"
        << "# dt=0.1\n"
        << "# path_length=2.0\n"
        << "index,t,s,x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot,"
        << "a,alpha,v_s,u_pub_v,u_pub_omega,kappa_v,kappa_omega,"
        << "r_x,r_y,r_yaw,r_v,r_omega,r_eta_x,r_eta_x_dot,r_eta_y,r_eta_y_dot\n";
    for (int i = 0; i < count; ++i) {
        double t = sample_times.empty()
            ? 0.1 * i
            : sample_times.at(static_cast<std::size_t>(i));
        if (nonmonotonic_time && i == 3) {
            t = 0.1;
        }
        const double radius = negative_radius && i == 2 ? -0.1 : 0.5;
        out << i << ',' << t << ',' << 0.1 * i << ',' << 0.1 * i
            << ",0,0,0.1,0,0,0,0,0,0,0,0.1,0.1,0,0.1,0,"
            << radius << ",0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5\n";
    }
    return out.str();
}

std::string developmentArtifactText() {
    std::string text = artifactText();
    const std::string insertion =
        "# artifact_role=interface_smoke_only\n"
        "# nominal_sequence_kind=rolling_local_planner_first_stage_proxy\n"
        "# offline_slosh_ocp=false\n"
        "# hardware_formal_release=false\n"
        "# paper_main_result_eligible=false\n"
        "# cycle_id_first=10\n"
        "# cycle_id_last=29\n"
        "# cycle_count=20\n"
        "# planner_variant=B_development_proxy\n"
        "# gate_parameter_source=operator_supplied_per_cycle_development_csv\n"
        "# recovery_policy_source=operator_supplied_per_cycle_development_csv\n"
        "# gate_evidence=none_development_input_only\n"
        "# recovery_policy_evidence=none_development_input_only\n"
        "# bag_sha256=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
        "# development_parameter_sha256=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n"
        "# row_state_semantics=predicted_horizon_stage0_at_solver_input_epoch\n"
        "# row_command_semantics=same_cycle_final_published_command\n"
        "# custom_note=preserved_by_canonical_writer\n";
    const std::string source = "# source=unit_test\n";
    const std::size_t source_position = text.find(source);
    EXPECT_NE(source_position, std::string::npos);
    text.replace(source_position, source.size(),
                 "# source=development_proxy_replay\n" + insertion);
    return text;
}

std::string writeTemp(const std::string& text) {
    const std::string path = "/tmp/spmpc_phase_artifact_" +
        std::to_string(static_cast<long long>(::getpid())) + "_" +
        std::to_string(text.size()) + ".csv";
    std::ofstream output(path);
    output << text;
    output.close();
    return path;
}

std::string readText(const std::string& path) {
    std::ifstream input(path);
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}

std::string replaceDataColumn(const std::string& text,
                              std::size_t row_index,
                              std::size_t column_index,
                              const std::string& replacement) {
    std::istringstream input(text);
    std::ostringstream output;
    std::string line;
    bool header_seen = false;
    std::size_t current_row = 0;
    while (std::getline(input, line)) {
        if (!line.empty() && line[0] != '#') {
            if (!header_seen) {
                header_seen = true;
            } else {
                if (current_row == row_index) {
                    std::vector<std::string> columns;
                    std::istringstream row(line);
                    std::string value;
                    while (std::getline(row, value, ',')) columns.push_back(value);
                    if (column_index < columns.size()) {
                        columns[column_index] = replacement;
                    }
                    std::ostringstream rebuilt;
                    for (std::size_t i = 0; i < columns.size(); ++i) {
                        if (i != 0) rebuilt << ',';
                        rebuilt << columns[i];
                    }
                    line = rebuilt.str();
                }
                ++current_row;
            }
        }
        output << line << '\n';
    }
    return output.str();
}

std::string preciseNumber(double value) {
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}

ExecutionModelContract augmentedExecutionContract() {
    ExecutionModelContract contract;
    contract.schema_version =
        augmented_manifest::kExecutionContractSchemaVersion;
    contract.contract_id = augmented_manifest::kContractId;
    contract.contract_hash = augmented_manifest::kContractHash;
    contract.dt = augmented_manifest::kDt;
    contract.linear.delay_sec = augmented_manifest::kLinearDelaySec;
    contract.linear.time_constant_sec =
        augmented_manifest::kLinearTimeConstantSec;
    contract.linear.positive_gain =
        augmented_manifest::kLinearPositiveGain;
    contract.linear.negative_gain =
        augmented_manifest::kLinearNegativeGain;
    contract.linear.deadzone = augmented_manifest::kLinearDeadzone;
    contract.linear.output_min = augmented_manifest::kLinearOutputMin;
    contract.linear.output_max = augmented_manifest::kLinearOutputMax;
    contract.angular.delay_sec = augmented_manifest::kAngularDelaySec;
    contract.angular.time_constant_sec =
        augmented_manifest::kAngularTimeConstantSec;
    contract.angular.positive_gain =
        augmented_manifest::kAngularPositiveGain;
    contract.angular.negative_gain =
        augmented_manifest::kAngularNegativeGain;
    contract.angular.deadzone = augmented_manifest::kAngularDeadzone;
    contract.angular.output_min = augmented_manifest::kAngularOutputMin;
    contract.angular.output_max = augmented_manifest::kAngularOutputMax;
    return contract;
}

SloshModelParams augmentedSloshParams() {
    SloshModelParams params;
    params.container_radius = augmented_manifest::kContainerRadius;
    params.liquid_height = augmented_manifest::kLiquidHeight;
    params.liquid_density = augmented_manifest::kLiquidDensity;
    params.damping_ratio = augmented_manifest::kDampingRatio;
    params.mode_index = augmented_manifest::kModeIndex;
    params.dt = augmented_manifest::kDt;
    params.slosh_height_ref = augmented_manifest::kSloshHeightRef;
    params.slosh_eta_dot_ratio =
        augmented_manifest::kSloshEtaDotRatio;
    params.use_linear_model = true;
    params.use_parabola_term = false;
    return params;
}

std::map<std::string, std::string> augmentedMetadata(
    double path_length = 0.09) {
    const SloshModelParams slosh_params = augmentedSloshParams();
    SloshDynamics slosh;
    EXPECT_TRUE(slosh.configure(slosh_params));
    const double omega_n = slosh.omegaN();
    return {
        {"schema", "phase_rejoin_empirical_augmented_v3"},
        {"evidence_level", "empirical_held_out"},
        {"source", "unit_test_augmented_nominal"},
        {"contract_id", "test_augmented_nominal_v3"},
        {"frame_id", "map"},
        {"dt", preciseNumber(augmented_manifest::kDt)},
        {"path_length", preciseNumber(path_length)},
        {"terminal_contract", "stop_settle_zero_hold_v1"},
        {"recovery_contract", "nominal_command_v1"},
        {"terminal_zero_hold_steps", "11"},
        {"terminal_eta_norm_max", "1.0"},
        {"terminal_eta_dot_norm_max", "1.0"},
        {"two_zeta_omega_n", preciseNumber(
             2.0 * slosh_params.damping_ratio * omega_n)},
        {"omega_n_sq", preciseNumber(omega_n * omega_n)},
        {"kappa_x", "1.0"},
        {"kappa_y", "1.0"},
        {"dynamics_tolerance", preciseNumber(
            augmented_manifest::kPublishedConsistencyTolerance)},
        {"execution_contract_id", augmented_manifest::kContractId},
        {"execution_contract_hash", augmented_manifest::kContractHash},
        {"execution_state_width",
         std::to_string(augmented_manifest::kStateCount)},
        {"execution_linear_buffer_count",
         std::to_string(augmented_manifest::kLinearBufferCount)},
        {"execution_angular_buffer_count",
         std::to_string(augmented_manifest::kAngularBufferCount)},
        {"parameter_schema_version",
         std::to_string(augmented_manifest::kParameterSchemaVersion)},
        {"parameter_schema_id", augmented_manifest::kParameterSchemaId},
        {"parameter_schema_hash", augmented_manifest::kParameterSchemaHash},
        {"recovery_artifact_hash",
         "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"},
        {"execution_compatibility_contract",
         augmented_manifest::kExecutionCompatibilityContract},
    };
}

EmpiricalRecoveryRadii unitRadii() {
    EmpiricalRecoveryRadii radii;
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

ExecutionCompatibilityBounds unitExecutionBounds() {
    ExecutionCompatibilityBounds bounds;
    bounds.valid = true;
    bounds.linear_actuator_output = 1.0;
    bounds.angular_actuator_output = 1.0;
    bounds.linear_pending_commands.assign(
        augmented_manifest::kLinearBufferCount, 1.0);
    bounds.angular_pending_commands.assign(
        augmented_manifest::kAngularBufferCount, 1.0);
    return bounds;
}

std::vector<PhaseNominalSample> zeroAugmentedSamples() {
    constexpr double dt = augmented_manifest::kDt;
    std::vector<PhaseNominalSample> samples(24);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        PhaseNominalSample& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * dt;
        sample.s = 0.09;
        sample.x = 0.09;
        sample.radii = unitRadii();
        sample.augmented_execution_valid = true;
        sample.augmented_execution.valid = true;
        sample.augmented_execution.stage_index = index;
        sample.augmented_execution.linear.pending_commands.assign(
            augmented_manifest::kLinearBufferCount, 0.0);
        sample.augmented_execution.angular.pending_commands.assign(
            augmented_manifest::kAngularBufferCount, 0.0);
        sample.execution_bounds = unitExecutionBounds();
    }
    return samples;
}

std::map<std::string, std::string> sealedAugmentedMetadata(
    const std::vector<PhaseNominalSample>& samples,
    double path_length = 0.09) {
    std::map<std::string, std::string> metadata =
        augmentedMetadata(path_length);
    metadata["recovery_artifact_hash"] =
        NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);
    return metadata;
}

struct NontrivialAugmentedFixture {
    std::map<std::string, std::string> metadata;
    std::vector<PhaseNominalSample> samples;
};

NontrivialAugmentedFixture nontrivialAugmentedFixture() {
    constexpr double dt = augmented_manifest::kDt;
    const ExecutionModelContract contract = augmentedExecutionContract();
    const SloshModelParams slosh_params = augmentedSloshParams();
    ExecutionModel dynamics;
    std::string error;
    EXPECT_TRUE(dynamics.configure(contract, slosh_params, error)) << error;

    ExecutionAugmentedState state;
    EXPECT_TRUE(dynamics.initializeHeld(
        RobotState{}, SloshState{}, VelocityCommand{}, state, error)) << error;
    double progress_s = 0.09;
    NontrivialAugmentedFixture fixture;
    fixture.samples.resize(40);
    for (std::size_t index = 0; index < fixture.samples.size(); ++index) {
        const double linear_step = 0.5 * dt;
        const double angular_step = 1.0 * dt;
        const double published_v = index < 8
            ? static_cast<double>(index + 1) * linear_step
            : (index < 16
                ? static_cast<double>(15 - index) * linear_step
                : 0.0);
        const double published_omega = index < 8
            ? static_cast<double>(index + 1) * angular_step
            : (index < 16
                ? static_cast<double>(15 - index) * angular_step
                : 0.0);
        PhaseNominalSample& sample = fixture.samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * dt;
        sample.s = progress_s;
        sample.x = state.robot.x;
        sample.y = state.robot.y;
        sample.yaw = state.robot.yaw;
        sample.v = state.robot.v;
        sample.omega = state.robot.omega;
        sample.eta_x = state.slosh.eta_x;
        sample.eta_x_dot = state.slosh.eta_x_dot;
        sample.eta_y = state.slosh.eta_y;
        sample.eta_y_dot = state.slosh.eta_y_dot;
        sample.a = (published_v -
            state.linear.pending_commands.back()) / dt;
        sample.alpha = (published_omega -
            state.angular.pending_commands.back()) / dt;
        sample.v_s = index < 16 ? 0.1 : 0.0;
        sample.u_pub_v = published_v;
        sample.u_pub_omega = published_omega;
        sample.kappa_v = published_v;
        sample.kappa_omega = published_omega;
        sample.radii = unitRadii();
        sample.augmented_execution_valid = true;
        sample.augmented_execution = state;
        sample.execution_bounds = unitExecutionBounds();
        if (index + 1 < fixture.samples.size()) {
            VelocityCommand published;
            published.linear = published_v;
            published.angular = published_omega;
            const ExecutionStepResult result = dynamics.step(state, published);
            EXPECT_TRUE(result.valid) << result.status;
            state = result.state;
            progress_s += sample.v_s * dt;
        }
    }
    fixture.metadata = sealedAugmentedMetadata(
        fixture.samples, fixture.samples.back().s);
    return fixture;
}

TEST(NominalSequenceArtifact, LoadsStrictValidArtifact) {
    const std::string path = writeTemp(artifactText());
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    ASSERT_TRUE(result.success) << result.status << ": " << result.detail;
    ASSERT_TRUE(artifact.valid());
    ASSERT_EQ(artifact.size(), 20u);
    EXPECT_EQ(artifact.metadata().schema, "phase_rejoin_empirical_v1");
    EXPECT_EQ(artifact.metadata().evidence_level,
              PhaseRejoinEvidenceLevel::DevelopmentOnly);
    EXPECT_EQ(artifact.metadata().contract_id, "test_contract");
    EXPECT_NEAR(artifact.metadata().dt, 0.1, 1e-12);
    ASSERT_NE(artifact.sample(3), nullptr);
    EXPECT_EQ(artifact.sample(3)->index, 3u);
    EXPECT_NEAR(artifact.sample(3)->s, 0.3, 1e-12);
}

TEST(NominalSequenceArtifact, PreservesAndValidatesDevelopmentMetadata) {
    const std::string path = writeTemp(developmentArtifactText());
    NominalSequenceArtifact artifact;
    const auto load_result = artifact.loadCsv(path);
    const auto development_result = artifact.validateDevelopmentOnly();
    std::remove(path.c_str());

    ASSERT_TRUE(load_result.success)
        << load_result.status << ": " << load_result.detail;
    EXPECT_TRUE(development_result.success)
        << development_result.status << ": " << development_result.detail;
    EXPECT_EQ(artifact.metadataEntries().at("cycle_id_first"), "10");
    EXPECT_EQ(artifact.metadataEntries().at("custom_note"),
              "preserved_by_canonical_writer");
}

TEST(NominalSequenceArtifact, RejectsDevelopmentArtifactEvidenceRelabeling) {
    std::string text = developmentArtifactText();
    const std::string original = "# evidence_level=development_only\n";
    const std::size_t position = text.find(original);
    ASSERT_NE(position, std::string::npos);
    text.replace(position, original.size(),
                 "# evidence_level=empirical_held_out\n");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto load_result = artifact.loadCsv(path);
    const auto development_result = artifact.validateDevelopmentOnly();
    std::remove(path.c_str());

    ASSERT_TRUE(load_result.success)
        << load_result.status << ": " << load_result.detail;
    EXPECT_FALSE(development_result.success);
    EXPECT_EQ(development_result.status, "DEVELOPMENT_METADATA_MISMATCH");
    EXPECT_EQ(development_result.detail, "evidence_level");
}

TEST(NominalSequenceArtifact, RejectsDevelopmentHashAndCycleMismatch) {
    {
        std::string text = developmentArtifactText();
        const std::string hash(64, 'a');
        text.replace(text.find(hash), hash.size(), std::string(64, 'A'));
        const std::string path = writeTemp(text);
        NominalSequenceArtifact artifact;
        ASSERT_TRUE(artifact.loadCsv(path).success);
        const auto result = artifact.validateDevelopmentOnly();
        std::remove(path.c_str());
        EXPECT_FALSE(result.success);
        EXPECT_EQ(result.status, "INVALID_SHA256");
        EXPECT_EQ(result.detail, "bag_sha256");
    }
    {
        std::string text = developmentArtifactText();
        const std::string original = "# cycle_count=20\n";
        text.replace(text.find(original), original.size(),
                     "# cycle_count=19\n");
        const std::string path = writeTemp(text);
        NominalSequenceArtifact artifact;
        ASSERT_TRUE(artifact.loadCsv(path).success);
        const auto result = artifact.validateDevelopmentOnly();
        std::remove(path.c_str());
        EXPECT_FALSE(result.success);
        EXPECT_EQ(result.status, "CYCLE_RANGE_MISMATCH");
    }
}

TEST(NominalSequenceArtifact, CanonicalWriterRoundTripsAllMetadata) {
    const std::string input_path = writeTemp(developmentArtifactText());
    const std::string output_path = input_path + ".canonical";
    const std::string second_path = output_path + ".second";
    NominalSequenceArtifact artifact;
    ASSERT_TRUE(artifact.loadCsv(input_path).success);
    const auto write_result = artifact.writeCanonicalCsv(output_path);
    ASSERT_TRUE(write_result.success)
        << write_result.status << ": " << write_result.detail;

    NominalSequenceArtifact round_trip;
    const auto load_result = round_trip.loadCsv(output_path);
    ASSERT_TRUE(load_result.success)
        << load_result.status << ": " << load_result.detail;
    EXPECT_TRUE(round_trip.validateDevelopmentOnly().success);
    EXPECT_EQ(round_trip.metadataEntries(), artifact.metadataEntries());
    ASSERT_TRUE(round_trip.writeCanonicalCsv(second_path).success);
    EXPECT_EQ(readText(output_path), readText(second_path));
    const auto no_overwrite = artifact.writeCanonicalCsv(output_path);
    EXPECT_FALSE(no_overwrite.success);
    EXPECT_EQ(no_overwrite.status, "OUTPUT_EXISTS");

    std::remove(input_path.c_str());
    std::remove(output_path.c_str());
    std::remove(second_path.c_str());
}

TEST(NominalSequenceArtifact, RejectsMissingMetadata) {
    std::string text = artifactText();
    const std::string line = "# contract_id=test_contract\n";
    text.erase(text.find(line), line.size());
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "MISSING_METADATA");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsMissingColumn) {
    std::string text = artifactText();
    const std::string header_tail = ",r_eta_y,r_eta_y_dot\n";
    const std::size_t position = text.find(header_tail);
    ASSERT_NE(position, std::string::npos);
    text.replace(position, header_tail.size(), ",r_eta_y\n");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "HEADER_MISMATCH");
}

TEST(NominalSequenceArtifact, RejectsNonfiniteValue) {
    std::string text = artifactText();
    const std::string row_prefix = "2,0.2,0.2,0.2,";
    const std::size_t position = text.find(row_prefix);
    ASSERT_NE(position, std::string::npos);
    text.replace(position, row_prefix.size(), "2,0.2,0.2,nan,");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "NONFINITE_OR_INVALID_VALUE");
}

TEST(NominalSequenceArtifact, RejectsNoncontiguousIndex) {
    std::string text = artifactText();
    const std::string row_prefix = "2,0.2,0.2,0.2,";
    const std::size_t position = text.find(row_prefix);
    ASSERT_NE(position, std::string::npos);
    text.replace(position, row_prefix.size(), "3,0.2,0.2,0.2,");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "INDEX_NOT_CONTIGUOUS");
}

TEST(NominalSequenceArtifact, RejectsUnsupportedEvidenceClaim) {
    std::string text = artifactText();
    const std::string evidence = "# evidence_level=development_only\n";
    const std::size_t position = text.find(evidence);
    ASSERT_NE(position, std::string::npos);
    text.replace(position, evidence.size(),
                 "# evidence_level=robust_certificate\n");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "INVALID_EVIDENCE_LEVEL");
}

TEST(NominalSequenceArtifact, RejectsNonpositiveRadius) {
    const std::string path = writeTemp(artifactText(20, true, false));
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "NONPOSITIVE_GATE_RADIUS");
}

TEST(NominalSequenceArtifact, RejectsNonmonotonicTime) {
    const std::string path = writeTemp(artifactText(20, false, true));
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "NONMONOTONIC_SEQUENCE");
}

TEST(NominalSequenceArtifact, RejectsTruncatedSequenceWithFullPathMetadata) {
    const std::string path = writeTemp(artifactText(10));
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "PATH_LENGTH_MISMATCH");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, LoadsBoundedClockQuantizedSequence) {
    std::vector<double> times(20, 0.0);
    const double increments[] = {0.12, 0.12, 0.06};
    for (std::size_t i = 1; i < times.size(); ++i) {
        times[i] = times[i - 1] + increments[(i - 1) % 3];
    }
    const std::string path = writeTemp(
        artifactText(20, false, false, times));
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_TRUE(result.success) << result.status << ": " << result.detail;
    EXPECT_TRUE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsAccumulatingClockQuantizationDrift) {
    std::vector<double> times(20, 0.0);
    for (std::size_t i = 1; i < times.size(); ++i) {
        times[i] = times[i - 1] + 0.06;
    }
    const std::string path = writeTemp(
        artifactText(20, false, false, times));
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "SAMPLE_PHASE_DRIFT");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, LoadsDynamicsConsistentV2CompleteTail) {
    const std::string path = writeTemp(
        spmpc_local_planner_test::completeArtifactText());
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    ASSERT_TRUE(result.success) << result.status << ": " << result.detail;
    EXPECT_EQ(artifact.metadata().schema, "phase_rejoin_empirical_v2");
    EXPECT_TRUE(artifact.metadata().complete_terminal_tail);
    EXPECT_EQ(artifact.metadata().terminal_zero_hold_steps, 11u);
    EXPECT_EQ(artifact.metadata().terminal_contract,
              "stop_settle_zero_hold_v1");
    EXPECT_EQ(artifact.metadata().recovery_contract,
              "nominal_command_v1");
}

TEST(NominalSequenceArtifact, LoadsDynamicsConsistentV3AugmentedSequence) {
    const std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealedAugmentedMetadata(samples), samples, "<v3-valid>");

    ASSERT_TRUE(result.success) << result.status << ": " << result.detail;
    EXPECT_TRUE(artifact.valid());
    EXPECT_EQ(artifact.metadata().execution_state_width, 22);
    EXPECT_EQ(artifact.metadata().linear_buffer_count, 5);
    EXPECT_EQ(artifact.metadata().angular_buffer_count, 7);
}

TEST(NominalSequenceArtifact,
     LoadsNontrivialV3SequenceFromReferenceExecutionDynamics) {
    const NontrivialAugmentedFixture fixture =
        nontrivialAugmentedFixture();
    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        fixture.metadata, fixture.samples, "<v3-reference-dynamics>");

    ASSERT_TRUE(result.success) << result.status << ": " << result.detail;
    EXPECT_TRUE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsV3PublishedCommandMismatch) {
    std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    samples[4].u_pub_v = 0.1;
    samples[4].kappa_v = 0.1;
    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealedAugmentedMetadata(samples), samples, "<v3-bad-published>");

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "PUBLISHED_COMMAND_MISMATCH");
    EXPECT_EQ(result.detail, "index 4");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact,
     RejectsV3ControlOutsideGeneratedOcpBoundsAtAdmission) {
    std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    samples[0].a = augmented_manifest::kAccelerationMax + 0.01;
    samples[0].u_pub_v = samples[0].a * augmented_manifest::kDt;
    samples[0].kappa_v = samples[0].u_pub_v;

    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealedAugmentedMetadata(samples), samples,
        "<v3-out-of-bounds-control>");

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "V3_NOMINAL_CONTROL_BOUNDS_MISMATCH");
}

TEST(NominalSequenceArtifact,
     RejectsV3ValuesTooSmallForOnlineParameterization) {
    {
        std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
        samples[0].radii.x =
            0.5 * augmented_manifest::kMinimumRecoveryDenominator;
        NominalSequenceArtifact artifact;
        const auto result = artifact.assignValidated(
            sealedAugmentedMetadata(samples), samples,
            "<v3-ill-conditioned-radius>");
        EXPECT_FALSE(result.success);
        EXPECT_EQ(result.status, "INVALID_AUGMENTED_EXECUTION_ROW");
    }
    {
        std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
        samples[0].execution_bounds.linear_actuator_output =
            0.5 * augmented_manifest::kMinimumRecoveryDenominator;
        NominalSequenceArtifact artifact;
        const auto result = artifact.assignValidated(
            sealedAugmentedMetadata(samples), samples,
            "<v3-ill-conditioned-execution-bound>");
        EXPECT_FALSE(result.success);
        EXPECT_EQ(result.status, "INVALID_AUGMENTED_EXECUTION_ROW");
    }
}

TEST(NominalSequenceArtifact,
     RejectsV3ToleranceWiderThanOnlinePublishedCommandContract) {
    const std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    std::map<std::string, std::string> metadata = augmentedMetadata();
    metadata["dynamics_tolerance"] = preciseNumber(
        2.0 * augmented_manifest::kPublishedConsistencyTolerance);
    metadata["recovery_artifact_hash"] =
        NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);

    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        metadata, samples, "<v3-overwide-dynamics-tolerance>");
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "INVALID_V3_METADATA_VALUE");
}

TEST(NominalSequenceArtifact,
     RejectsV3ExecutionStateOutsideGeneratedEnvelope) {
    std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    samples[0].augmented_execution.linear.pending_commands.front() =
        augmented_manifest::kLinearOutputMax + 0.01;

    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealedAugmentedMetadata(samples), samples,
        "<v3-out-of-bounds-execution-state>");
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "V3_EXECUTION_STATE_BOUNDS_MISMATCH");
}

TEST(NominalSequenceArtifact,
     RejectsNegativeProgressThatOnlineParameterizationCannotSerialize) {
    std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    samples.front().s = 0.0;
    for (std::size_t index = 1; index < samples.size(); ++index) {
        samples[index].s = -5.0e-10;
    }
    constexpr double kPositiveDeclaredPathLength = 1.0e-10;

    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealedAugmentedMetadata(samples, kPositiveDeclaredPathLength),
        samples, "<v3-negative-progress>");
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "V3_NOMINAL_PROGRESS_BOUNDS_MISMATCH");
}

TEST(NominalSequenceArtifact,
     RejectsTerminalPublishedCommandMismatchBeforeOnlineUse) {
    std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    samples.back().augmented_execution.linear.pending_commands.back() =
        0.01;

    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealedAugmentedMetadata(samples), samples,
        "<v3-terminal-published-mismatch>");
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "PUBLISHED_COMMAND_MISMATCH");
    EXPECT_EQ(result.detail,
              "index " + std::to_string(samples.size() - 1));
}

TEST(NominalSequenceArtifact,
     RejectsV3DtThatDiffersFromCompiledExecutableImage) {
    const std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    std::map<std::string, std::string> metadata = augmentedMetadata();
    metadata["dt"] = preciseNumber(augmented_manifest::kDt + 5.0e-13);
    metadata["recovery_artifact_hash"] =
        NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);

    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        metadata, samples, "<v3-noncompiled-dt>");
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "V3_EXECUTION_CONTRACT_MISMATCH");
}

TEST(NominalSequenceArtifact, RejectsV3ExecutionContractMismatch) {
    const std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    std::map<std::string, std::string> metadata = augmentedMetadata();
    metadata["execution_contract_hash"] = std::string(64, 'b');
    metadata["recovery_artifact_hash"] =
        NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);
    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        metadata, samples, "<v3-bad-contract>");

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "V3_EXECUTION_CONTRACT_MISMATCH");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsV3PendingQueueShiftMismatch) {
    std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    samples[5].augmented_execution.linear.pending_commands[1] = 0.1;
    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealedAugmentedMetadata(samples), samples, "<v3-bad-queue>");

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "DYNAMICS_TRANSITION_MISMATCH");
    EXPECT_EQ(result.detail, "index 4");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsV3ActuatorTransitionMismatch) {
    std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    samples[5].v = 0.1;
    samples[5].augmented_execution.linear.actuator_output = 0.1;
    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealedAugmentedMetadata(samples), samples, "<v3-bad-actuator>");

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "DYNAMICS_TRANSITION_MISMATCH");
    EXPECT_EQ(result.detail, "index 4");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsV3RobotAndSloshTransitionMismatch) {
    for (int mutation = 0; mutation < 2; ++mutation) {
        std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
        if (mutation == 0) {
            samples[5].x += 0.1;
        } else {
            samples[5].eta_x = 0.1;
        }
        NominalSequenceArtifact artifact;
        const auto result = artifact.assignValidated(
            sealedAugmentedMetadata(samples), samples,
            "<v3-bad-physical-state>");

        EXPECT_FALSE(result.success);
        EXPECT_EQ(result.status, "DYNAMICS_TRANSITION_MISMATCH");
        EXPECT_EQ(result.detail, "index 4");
        EXPECT_FALSE(artifact.valid());
    }
}

TEST(NominalSequenceArtifact, RejectsSelfReportedV3RecoveryHash) {
    const std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        augmentedMetadata(), samples, "<v3-self-reported-hash>");

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "RECOVERY_ARTIFACT_HASH_MISMATCH");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsV3PayloadTamperingAfterSeal) {
    std::vector<PhaseNominalSample> samples = zeroAugmentedSamples();
    const std::map<std::string, std::string> sealed_metadata =
        sealedAugmentedMetadata(samples);
    samples[4].radii.eta_x = 0.5;

    NominalSequenceArtifact artifact;
    const auto result = artifact.assignValidated(
        sealed_metadata, samples, "<v3-tampered-row>");

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "RECOVERY_ARTIFACT_HASH_MISMATCH");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsV2PublishedCommandNotGeneratedByControl) {
    std::string text = spmpc_local_planner_test::completeArtifactText();
    text = replaceDataColumn(text, 4, 15, "0.5");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "PUBLISHED_COMMAND_MISMATCH");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RequiresV2RecoveryContractMetadata) {
    std::string text = spmpc_local_planner_test::completeArtifactText();
    const std::string metadata =
        "# recovery_contract=nominal_command_v1\n";
    const std::size_t position = text.find(metadata);
    ASSERT_NE(position, std::string::npos);
    text.erase(position, metadata.size());
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "MISSING_METADATA");
    EXPECT_EQ(result.detail, "recovery_contract");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsUnsupportedV2RecoveryContract) {
    std::string text = spmpc_local_planner_test::completeArtifactText();
    const std::string original =
        "# recovery_contract=nominal_command_v1\n";
    const std::size_t position = text.find(original);
    ASSERT_NE(position, std::string::npos);
    text.replace(position, original.size(),
                 "# recovery_contract=independent_policy_v0\n");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "UNSUPPORTED_RECOVERY_CONTRACT");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsV2RecoveryCommandDifferentFromPublished) {
    std::string text = spmpc_local_planner_test::completeArtifactText();
    text = replaceDataColumn(text, 4, 17, "0.5");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "RECOVERY_COMMAND_MISMATCH");
    EXPECT_EQ(result.detail, "index 4");
    EXPECT_FALSE(artifact.valid());
}

TEST(NominalSequenceArtifact, RejectsV2TailThatStartsBeforeCommandsAreZero) {
    std::string text = spmpc_local_planner_test::completeArtifactText();
    const std::string original = "# terminal_zero_hold_steps=11\n";
    const std::size_t position = text.find(original);
    ASSERT_NE(position, std::string::npos);
    text.replace(position, original.size(), "# terminal_zero_hold_steps=12\n");
    const std::string path = writeTemp(text);
    NominalSequenceArtifact artifact;
    const auto result = artifact.loadCsv(path);
    std::remove(path.c_str());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.status, "ZERO_HOLD_COMMAND_NONZERO");
    EXPECT_FALSE(artifact.valid());
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
