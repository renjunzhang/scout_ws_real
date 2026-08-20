#include "spmpc_local_planner/phase_rejoin/development_nominal_generator.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

using spmpc_local_planner::DevelopmentNominalGenerationResult;
using spmpc_local_planner::DevelopmentNominalGenerator;
using spmpc_local_planner::DevelopmentNominalGeneratorConfig;
using spmpc_local_planner::DevelopmentNominalPoint;
using spmpc_local_planner::NominalArtifactLoadResult;
using spmpc_local_planner::NominalSequenceArtifact;

int usage(const std::string& detail = std::string()) {
    if (!detail.empty()) {
        std::cerr << "ERROR: " << detail << '\n';
    }
    std::cerr
        << "usage: spmpc_phase_rejoin_development_nominal"
        << " --path-csv PATH --output PATH --contract-id ID --frame-id FRAME"
        << " --dt SEC --cruise-speed MPS --ramp-sec SEC --lookahead M"
        << " --heading-gain VALUE --omega-max VALUE --alpha-max VALUE"
        << " --omega-n VALUE --damping-ratio VALUE --kappa-x VALUE"
        << " --kappa-y VALUE --zero-hold-sec SEC"
        << " --terminal-eta-norm-max VALUE"
        << " --terminal-eta-dot-norm-max VALUE"
        << " --gate-radii RX RY RYAW RV ROMEGA REX REXD REY REYD"
        << " --source-bag-sha256 SHA256 --path-topic TOPIC [--overwrite]\n";
    return 2;
}

std::string trim(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t begin = value.find_first_not_of(whitespace);
    if (begin == std::string::npos) {
        return std::string();
    }
    const std::size_t end = value.find_last_not_of(whitespace);
    return value.substr(begin, end - begin + 1);
}

bool parseDouble(const std::string& text, double& value) {
    const std::string input = trim(text);
    if (input.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    value = std::strtod(input.c_str(), &end);
    return errno == 0 && end != input.c_str() && *end == '\0' &&
           std::isfinite(value);
}

bool takeString(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc) {
        return false;
    }
    value = argv[++index];
    return !value.empty();
}

bool takeDouble(int argc, char** argv, int& index, double& value) {
    std::string text;
    return takeString(argc, argv, index, text) && parseDouble(text, value);
}

bool readPathCsv(const std::string& path,
                 std::vector<DevelopmentNominalPoint>& points,
                 std::string& detail) {
    std::ifstream input(path);
    if (!input.is_open()) {
        detail = "cannot open path CSV: " + path;
        return false;
    }
    bool header_seen = false;
    std::size_t line_number = 0;
    std::string line;
    while (std::getline(input, line)) {
        ++line_number;
        const std::string clean = trim(line);
        if (clean.empty()) {
            continue;
        }
        const std::size_t comma = clean.find(',');
        if (!header_seen) {
            if (clean != "x,y") {
                detail = "path CSV header must be x,y";
                return false;
            }
            header_seen = true;
            continue;
        }
        if (comma == std::string::npos ||
            clean.find(',', comma + 1) != std::string::npos) {
            detail = "path CSV column mismatch at line " +
                std::to_string(line_number);
            return false;
        }
        DevelopmentNominalPoint point;
        if (!parseDouble(clean.substr(0, comma), point.x) ||
            !parseDouble(clean.substr(comma + 1), point.y)) {
            detail = "invalid path point at line " +
                std::to_string(line_number);
            return false;
        }
        points.push_back(point);
    }
    if (!header_seen) {
        detail = "path CSV is empty";
        return false;
    }
    return true;
}

int reportArtifactFailure(const NominalArtifactLoadResult& result) {
    std::cerr << "ERROR: " << result.status;
    if (!result.detail.empty()) {
        std::cerr << ": " << result.detail;
    }
    std::cerr << '\n';
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    DevelopmentNominalGeneratorConfig config;
    std::string path_csv;
    std::string output;
    bool overwrite = false;
    bool gate_radii_seen = false;
    bool omega_n_seen = false;
    std::set<std::string> seen_arguments;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (!seen_arguments.insert(argument).second) {
            return usage("duplicate argument: " + argument);
        }
        if (argument == "--path-csv") {
            if (!path_csv.empty() || !takeString(argc, argv, i, path_csv)) {
                return usage("invalid or duplicate --path-csv");
            }
        } else if (argument == "--output") {
            if (!output.empty() || !takeString(argc, argv, i, output)) {
                return usage("invalid or duplicate --output");
            }
        } else if (argument == "--contract-id") {
            if (!config.contract_id.empty() ||
                !takeString(argc, argv, i, config.contract_id)) {
                return usage("invalid or duplicate --contract-id");
            }
        } else if (argument == "--frame-id") {
            if (!takeString(argc, argv, i, config.frame_id)) {
                return usage("invalid --frame-id");
            }
        } else if (argument == "--dt") {
            if (!takeDouble(argc, argv, i, config.dt)) return usage("invalid --dt");
        } else if (argument == "--cruise-speed") {
            if (!takeDouble(argc, argv, i, config.cruise_speed)) return usage("invalid --cruise-speed");
        } else if (argument == "--ramp-sec") {
            if (!takeDouble(argc, argv, i, config.ramp_sec)) return usage("invalid --ramp-sec");
        } else if (argument == "--lookahead") {
            if (!takeDouble(argc, argv, i, config.lookahead)) return usage("invalid --lookahead");
        } else if (argument == "--heading-gain") {
            if (!takeDouble(argc, argv, i, config.heading_gain)) return usage("invalid --heading-gain");
        } else if (argument == "--omega-max") {
            if (!takeDouble(argc, argv, i, config.omega_max)) return usage("invalid --omega-max");
        } else if (argument == "--alpha-max") {
            if (!takeDouble(argc, argv, i, config.alpha_max)) return usage("invalid --alpha-max");
        } else if (argument == "--omega-n") {
            if (omega_n_seen || !takeDouble(argc, argv, i, config.omega_n)) {
                return usage("invalid or duplicate --omega-n");
            }
            omega_n_seen = true;
        } else if (argument == "--damping-ratio") {
            if (!takeDouble(argc, argv, i, config.damping_ratio)) return usage("invalid --damping-ratio");
        } else if (argument == "--kappa-x") {
            if (!takeDouble(argc, argv, i, config.kappa_x)) return usage("invalid --kappa-x");
        } else if (argument == "--kappa-y") {
            if (!takeDouble(argc, argv, i, config.kappa_y)) return usage("invalid --kappa-y");
        } else if (argument == "--zero-hold-sec") {
            if (!takeDouble(argc, argv, i, config.zero_hold_sec)) return usage("invalid --zero-hold-sec");
        } else if (argument == "--terminal-eta-norm-max") {
            if (!takeDouble(argc, argv, i, config.terminal_eta_norm_max)) return usage("invalid --terminal-eta-norm-max");
        } else if (argument == "--terminal-eta-dot-norm-max") {
            if (!takeDouble(argc, argv, i, config.terminal_eta_dot_norm_max)) return usage("invalid --terminal-eta-dot-norm-max");
        } else if (argument == "--gate-radii") {
            if (gate_radii_seen) return usage("duplicate --gate-radii");
            double* values[] = {
                &config.radii.x, &config.radii.y, &config.radii.yaw,
                &config.radii.v, &config.radii.omega,
                &config.radii.eta_x, &config.radii.eta_x_dot,
                &config.radii.eta_y, &config.radii.eta_y_dot,
            };
            for (double* value : values) {
                if (!takeDouble(argc, argv, i, *value)) {
                    return usage("--gate-radii requires nine finite values");
                }
            }
            gate_radii_seen = true;
        } else if (argument == "--source-bag-sha256") {
            if (!config.source_bag_sha256.empty() ||
                !takeString(argc, argv, i, config.source_bag_sha256)) {
                return usage("invalid or duplicate --source-bag-sha256");
            }
        } else if (argument == "--path-topic") {
            if (!takeString(argc, argv, i, config.path_topic)) {
                return usage("invalid --path-topic");
            }
        } else if (argument == "--overwrite") {
            if (overwrite) return usage("duplicate --overwrite");
            overwrite = true;
        } else {
            return usage("unknown argument: " + argument);
        }
    }
    if (path_csv.empty() || output.empty() || config.contract_id.empty() ||
        config.source_bag_sha256.empty() || !omega_n_seen || !gate_radii_seen) {
        return usage("missing required argument");
    }

    std::vector<DevelopmentNominalPoint> points;
    std::string detail;
    if (!readPathCsv(path_csv, points, detail)) {
        return usage(detail);
    }
    const DevelopmentNominalGenerationResult generated =
        DevelopmentNominalGenerator().generate(points, config);
    if (!generated.success) {
        std::cerr << "ERROR: " << generated.status << ": "
                  << generated.detail << '\n';
        return 2;
    }
    NominalSequenceArtifact artifact;
    const NominalArtifactLoadResult assign_result = artifact.assignValidated(
        generated.metadata, generated.samples, "<generated-development-nominal>");
    if (!assign_result.success) {
        return reportArtifactFailure(assign_result);
    }
    const NominalArtifactLoadResult development_result =
        artifact.validateDevelopmentOnly();
    if (!development_result.success) {
        return reportArtifactFailure(development_result);
    }
    const NominalArtifactLoadResult write_result =
        artifact.writeCanonicalCsv(output, overwrite);
    if (!write_result.success) {
        return reportArtifactFailure(write_result);
    }
    const double duration =
        static_cast<double>(generated.samples.size() - 1) * config.dt;
    std::cout << std::fixed << std::setprecision(4)
              << "[OK] development-only V2 artifact: " << output << '\n'
              << "rows=" << generated.samples.size()
              << " duration=" << std::setprecision(3) << duration << "s"
              << " path=" << generated.path_length << "m"
              << " zero_hold_steps=" << generated.zero_hold_steps
              << " max_path_deviation=" << std::setprecision(4)
              << generated.max_path_deviation << "m\n"
              << "NOT OfflineSloshOCP; NOT hardware/formal/paper evidence\n";
    return 0;
}
