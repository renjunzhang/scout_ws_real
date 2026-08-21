#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/phase_rejoin/types.h"

#include <iostream>
#include <string>

namespace {

using spmpc_local_planner::NominalArtifactLoadResult;
using spmpc_local_planner::NominalSequenceArtifact;

int usage(const std::string& detail = std::string()) {
    if (!detail.empty()) {
        std::cerr << "ERROR: " << detail << '\n';
    }
    std::cerr
        << "usage:\n"
        << "  spmpc_phase_rejoin_artifact_tool validate --artifact PATH "
           "[--development-only]\n"
        << "  spmpc_phase_rejoin_artifact_tool canonicalize --input PATH "
           "--output PATH [--development-only] [--overwrite]\n";
    return 2;
}

int reportFailure(const NominalArtifactLoadResult& result) {
    std::cerr << "ERROR: " << result.status;
    if (!result.detail.empty()) {
        std::cerr << ": " << result.detail;
    }
    std::cerr << '\n';
    return 2;
}

bool takeValue(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc) {
        return false;
    }
    value = argv[++index];
    return !value.empty();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        return usage();
    }
    const std::string command = argv[1];
    std::string input_path;
    std::string input_option;
    std::string output_path;
    bool development_only = false;
    bool overwrite = false;
    for (int i = 2; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--artifact" || argument == "--input") {
            if (!input_path.empty() || !takeValue(argc, argv, i, input_path)) {
                return usage("invalid or duplicate input path");
            }
            input_option = argument;
        } else if (argument == "--output") {
            if (!output_path.empty() || !takeValue(argc, argv, i, output_path)) {
                return usage("invalid or duplicate output path");
            }
        } else if (argument == "--development-only") {
            if (development_only) {
                return usage("duplicate --development-only");
            }
            development_only = true;
        } else if (argument == "--overwrite") {
            if (overwrite) {
                return usage("duplicate --overwrite");
            }
            overwrite = true;
        } else {
            return usage("unknown argument: " + argument);
        }
    }
    if (command == "validate") {
        if (input_option != "--artifact" || !output_path.empty() || overwrite) {
            return usage("validate requires only --artifact PATH");
        }
    } else if (command == "canonicalize") {
        if (input_option != "--input" || output_path.empty()) {
            return usage("canonicalize requires --input and --output");
        }
    } else {
        return usage("unknown command: " + command);
    }

    NominalSequenceArtifact artifact;
    const NominalArtifactLoadResult load_result = artifact.loadCsv(input_path);
    if (!load_result.success) {
        return reportFailure(load_result);
    }
    if (development_only) {
        const NominalArtifactLoadResult development_result =
            artifact.validateDevelopmentOnly();
        if (!development_result.success) {
            return reportFailure(development_result);
        }
    }
    if (command == "canonicalize") {
        const NominalArtifactLoadResult write_result =
            artifact.writeCanonicalCsv(output_path, overwrite);
        if (!write_result.success) {
            return reportFailure(write_result);
        }
        std::cout << "CANONICAL artifact: " << output_path << '\n';
        return 0;
    }

    const auto& metadata = artifact.metadata();
    std::cout << "VALID artifact: schema=" << metadata.schema
              << " evidence_level="
              << spmpc_local_planner::phaseRejoinEvidenceLevelName(
                     metadata.evidence_level)
              << " source=" << metadata.source
              << " contract_id=" << metadata.contract_id;
    if (!metadata.recovery_artifact_hash.empty()) {
        std::cout << " recovery_artifact_hash="
                  << metadata.recovery_artifact_hash;
    }
    std::cout << '\n';
    return 0;
}
