#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "phase_rejoin_artifact_fixture.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace spmpc_local_planner {
namespace {

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

std::string writeTemp(const std::string& text) {
    const std::string path = "/tmp/spmpc_phase_artifact_" +
        std::to_string(static_cast<long long>(::getpid())) + "_" +
        std::to_string(text.size()) + ".csv";
    std::ofstream output(path);
    output << text;
    output.close();
    return path;
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
