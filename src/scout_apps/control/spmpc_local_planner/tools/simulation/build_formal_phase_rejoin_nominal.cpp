#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/reference/reference_path.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"
#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"

#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace spmpc = spmpc_local_planner;

namespace {

constexpr char kPlanSchema[] = "spmpc_offline_slosh_ocp_plan_v1";
constexpr char kFitManifestSchema[] =
    "spmpc_phase_rejoin_recovery_fit_manifest_v1";
constexpr char kHeldOutReportSchema[] =
    "spmpc_phase_rejoin_recovery_held_out_report_v1";
constexpr char kRecoveryDatasetSchema[] =
    "spmpc_phase_rejoin_recovery_dataset_v1";
constexpr char kRecoveryScalesSchema[] =
    "spmpc_phase_rejoin_recovery_scales_v1";
constexpr char kNominalReportSchema[] =
    "spmpc_nominal_validation_report_v1";
constexpr char kTerminalContract[] = "publish_zero_settle_hold_v2";
constexpr double kTolerance = 1.0e-9;

struct Arguments {
    std::string path_json;
    std::string plan_csv;
    std::string plan_report;
    std::string recovery_scales;
    std::string recovery_manifest;
    std::string held_out_report;
    std::string artifact_validator;
    std::string output;
    std::string report_output;
    std::string contract_id;
    bool overwrite = false;
    double max_path_deviation_m = 0.075;
    double goal_position_tolerance_m = 0.10;
    double goal_yaw_tolerance_rad = 0.10;
    double terminal_eta_norm_max = 5.0e-5;
    double terminal_eta_dot_norm_max = 2.0e-3;
    double terminal_velocity_abs_max = 1.0e-8;
    double terminal_actuator_abs_max = 1.0e-8;
    double terminal_pending_abs_max = 1.0e-10;
};

struct PlanRow {
    std::size_t index = 0;
    double t = 0.0;
    double published_v = 0.0;
    double published_omega = 0.0;
    double progress_rate = 0.0;
};

struct RecoveryScaleRow {
    std::size_t phase_index = 0;
    spmpc::EmpiricalRecoveryRadii radii;
    spmpc::ExecutionCompatibilityBounds execution;
};

struct PathAsset {
    spmpc::ReferencePath reference;
    std::string frame_id;
};

std::string trim(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) return std::string();
    const std::size_t last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1);
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream input(line);
    std::string field;
    while (std::getline(input, field, ',')) fields.push_back(trim(field));
    if (!line.empty() && line.back() == ',') fields.emplace_back();
    return fields;
}

bool parseDouble(const std::string& text, double& value) {
    const std::string clean = trim(text);
    if (clean.empty()) return false;
    errno = 0;
    char* end = nullptr;
    value = std::strtod(clean.c_str(), &end);
    return errno == 0 && end != clean.c_str() && *end == '\0' &&
        std::isfinite(value);
}

bool parseIndex(const std::string& text, std::size_t& value) {
    const std::string clean = trim(text);
    if (clean.empty() || clean.front() == '-') return false;
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(clean.c_str(), &end, 10);
    if (errno != 0 || end == clean.c_str() || *end != '\0' ||
        parsed > static_cast<unsigned long long>(
                     std::numeric_limits<std::size_t>::max())) {
        return false;
    }
    value = static_cast<std::size_t>(parsed);
    return true;
}

std::string number(double value) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << value;
    return out.str();
}

std::string sha256File(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) return std::string();
    SHA256_CTX context;
    if (::SHA256_Init(&context) != 1) return std::string();
    std::array<char, 1 << 16> buffer{};
    while (input.good()) {
        input.read(buffer.data(), buffer.size());
        const std::streamsize count = input.gcount();
        if (count > 0 && ::SHA256_Update(
                &context, buffer.data(), static_cast<std::size_t>(count)) != 1) {
            return std::string();
        }
    }
    if (!input.eof()) return std::string();
    std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
    if (::SHA256_Final(digest.data(), &context) != 1) return std::string();
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (unsigned char byte : digest) {
        out << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return out.str();
}

bool lowercaseSha256(const std::string& value) {
    return value.size() == 64 &&
        std::all_of(value.begin(), value.end(), [](char character) {
            return (character >= '0' && character <= '9') ||
                (character >= 'a' && character <= 'f');
        });
}

std::string baseName(const std::string& path) {
    const std::size_t separator = path.find_last_of('/');
    return separator == std::string::npos ? path : path.substr(separator + 1);
}

std::string parentName(const std::string& path) {
    const std::size_t separator = path.find_last_of('/');
    if (separator == std::string::npos) return ".";
    if (separator == 0) return "/";
    return path.substr(0, separator);
}

bool canonicalExistingPath(const std::string& path,
                           std::string& canonical,
                           std::string& error) {
    char* resolved = ::realpath(path.c_str(), nullptr);
    if (resolved == nullptr) {
        error = "cannot resolve input path: " + path;
        return false;
    }
    canonical.assign(resolved);
    std::free(resolved);
    struct stat status;
    if (::stat(canonical.c_str(), &status) != 0 || !S_ISREG(status.st_mode)) {
        error = "input path is not a regular file: " + path;
        return false;
    }
    return true;
}

bool canonicalOutputPath(const std::string& path,
                         std::string& canonical,
                         std::string& error) {
    char* resolved = ::realpath(path.c_str(), nullptr);
    if (resolved != nullptr) {
        canonical.assign(resolved);
        std::free(resolved);
        return true;
    }
    const std::string leaf = baseName(path);
    if (leaf.empty() || leaf == "." || leaf == "..") {
        error = "invalid output path: " + path;
        return false;
    }
    resolved = ::realpath(parentName(path).c_str(), nullptr);
    if (resolved == nullptr) {
        error = "cannot resolve output parent: " + path;
        return false;
    }
    canonical.assign(resolved);
    std::free(resolved);
    if (canonical != "/") canonical += "/";
    canonical += leaf;
    return true;
}

bool sameExistingFile(const std::string& lhs, const std::string& rhs) {
    struct stat lhs_status;
    struct stat rhs_status;
    return ::stat(lhs.c_str(), &lhs_status) == 0 &&
        ::stat(rhs.c_str(), &rhs_status) == 0 &&
        lhs_status.st_dev == rhs_status.st_dev &&
        lhs_status.st_ino == rhs_status.st_ino;
}

bool resolveAndRejectPathAliases(Arguments& args, std::string& error) {
    std::vector<std::string*> inputs = {
        &args.path_json, &args.plan_csv, &args.plan_report,
        &args.recovery_scales, &args.recovery_manifest,
        &args.held_out_report, &args.artifact_validator,
    };
    std::vector<std::string*> outputs = {&args.output, &args.report_output};
    for (std::string* path : inputs) {
        std::string canonical;
        if (!canonicalExistingPath(*path, canonical, error)) return false;
        *path = canonical;
    }
    for (std::string* path : outputs) {
        std::string canonical;
        if (!canonicalOutputPath(*path, canonical, error)) return false;
        *path = canonical;
    }
    std::vector<std::string*> all = inputs;
    all.insert(all.end(), outputs.begin(), outputs.end());
    for (std::size_t left = 0; left < all.size(); ++left) {
        for (std::size_t right = left + 1; right < all.size(); ++right) {
            if (*all[left] == *all[right] ||
                sameExistingFile(*all[left], *all[right])) {
                error = "input/output paths must not alias: " + *all[left];
                return false;
            }
        }
    }
    struct stat validator_status;
    if (::stat(args.artifact_validator.c_str(), &validator_status) != 0 ||
        !S_ISREG(validator_status.st_mode) ||
        ::access(args.artifact_validator.c_str(), X_OK) != 0) {
        error = "artifact validator is not an executable regular file";
        return false;
    }
    return true;
}

std::string escapeJson(const std::string& value) {
    std::ostringstream out;
    for (char character : value) {
        switch (character) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: out << character; break;
        }
    }
    return out.str();
}

int usage(const std::string& detail = std::string()) {
    if (!detail.empty()) std::cerr << "ERROR: " << detail << '\n';
    std::cerr
        << "usage: spmpc_build_formal_phase_rejoin_nominal"
        << " --path-json PATH --plan-csv PATH --plan-report PATH"
        << " --recovery-scales PATH --recovery-manifest PATH"
        << " --held-out-report PATH --artifact-validator PATH"
        << " --output PATH --report-output PATH"
        << " --contract-id ID [--overwrite]"
        << " [--max-path-deviation M] [--goal-position-tolerance M]"
        << " [--goal-yaw-tolerance RAD]"
        << " [--terminal-eta-norm-max VALUE]"
        << " [--terminal-eta-dot-norm-max VALUE]\n";
    return 2;
}

bool takeString(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc || !value.empty()) return false;
    value = argv[++index];
    return !value.empty();
}

bool takeDouble(int argc, char** argv, int& index, double& value) {
    if (index + 1 >= argc) return false;
    return parseDouble(argv[++index], value);
}

bool parseArguments(int argc, char** argv, Arguments& args) {
    std::set<std::string> seen;
    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        if (!seen.insert(option).second) return false;
        if (option == "--path-json") {
            if (!takeString(argc, argv, i, args.path_json)) return false;
        } else if (option == "--plan-csv") {
            if (!takeString(argc, argv, i, args.plan_csv)) return false;
        } else if (option == "--plan-report") {
            if (!takeString(argc, argv, i, args.plan_report)) return false;
        } else if (option == "--recovery-scales") {
            if (!takeString(argc, argv, i, args.recovery_scales)) return false;
        } else if (option == "--recovery-manifest") {
            if (!takeString(argc, argv, i, args.recovery_manifest)) return false;
        } else if (option == "--held-out-report") {
            if (!takeString(argc, argv, i, args.held_out_report)) return false;
        } else if (option == "--artifact-validator") {
            if (!takeString(argc, argv, i, args.artifact_validator)) return false;
        } else if (option == "--output") {
            if (!takeString(argc, argv, i, args.output)) return false;
        } else if (option == "--report-output") {
            if (!takeString(argc, argv, i, args.report_output)) return false;
        } else if (option == "--contract-id") {
            if (!takeString(argc, argv, i, args.contract_id)) return false;
        } else if (option == "--max-path-deviation") {
            if (!takeDouble(argc, argv, i, args.max_path_deviation_m)) return false;
        } else if (option == "--goal-position-tolerance") {
            if (!takeDouble(argc, argv, i, args.goal_position_tolerance_m)) return false;
        } else if (option == "--goal-yaw-tolerance") {
            if (!takeDouble(argc, argv, i, args.goal_yaw_tolerance_rad)) return false;
        } else if (option == "--terminal-eta-norm-max") {
            if (!takeDouble(argc, argv, i, args.terminal_eta_norm_max)) return false;
        } else if (option == "--terminal-eta-dot-norm-max") {
            if (!takeDouble(argc, argv, i, args.terminal_eta_dot_norm_max)) return false;
        } else if (option == "--overwrite") {
            args.overwrite = true;
        } else {
            return false;
        }
    }
    const bool paths = !args.path_json.empty() && !args.plan_csv.empty() &&
        !args.plan_report.empty() && !args.recovery_scales.empty() &&
        !args.recovery_manifest.empty() && !args.held_out_report.empty() &&
        !args.artifact_validator.empty() &&
        !args.output.empty() && !args.report_output.empty() &&
        !args.contract_id.empty();
    const double values[] = {
        args.max_path_deviation_m, args.goal_position_tolerance_m,
        args.goal_yaw_tolerance_rad, args.terminal_eta_norm_max,
        args.terminal_eta_dot_norm_max, args.terminal_velocity_abs_max,
        args.terminal_actuator_abs_max, args.terminal_pending_abs_max,
    };
    return paths && std::all_of(std::begin(values), std::end(values),
        [](double value) { return std::isfinite(value) && value >= 0.0; });
}

bool loadPlan(const std::string& path,
              std::map<std::string, std::string>& metadata,
              std::vector<PlanRow>& rows,
              std::string& error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        error = "cannot open plan CSV";
        return false;
    }
    const std::vector<std::string> expected = {
        "index", "t", "u_pub_v", "u_pub_omega", "v_s"};
    bool header_seen = false;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const std::string clean = trim(line);
        if (clean.empty()) continue;
        if (clean.front() == '#') {
            if (header_seen) {
                error = "plan metadata appears after the header";
                return false;
            }
            const std::string payload = trim(clean.substr(1));
            const std::size_t separator = payload.find('=');
            if (separator == std::string::npos) {
                error = "invalid plan metadata at line " +
                    std::to_string(line_number);
                return false;
            }
            const std::string key = trim(payload.substr(0, separator));
            const std::string value = trim(payload.substr(separator + 1));
            if (key.empty() || value.empty() || metadata.count(key) != 0) {
                error = "duplicate or empty plan metadata";
                return false;
            }
            metadata[key] = value;
            continue;
        }
        const std::vector<std::string> fields = splitCsv(clean);
        if (!header_seen) {
            if (fields != expected) {
                error = "plan CSV header mismatch";
                return false;
            }
            header_seen = true;
            continue;
        }
        if (fields.size() != expected.size()) {
            error = "plan CSV column mismatch";
            return false;
        }
        PlanRow row;
        if (!parseIndex(fields[0], row.index) ||
            !parseDouble(fields[1], row.t) ||
            !parseDouble(fields[2], row.published_v) ||
            !parseDouble(fields[3], row.published_omega) ||
            !parseDouble(fields[4], row.progress_rate) ||
            row.index != rows.size()) {
            error = "invalid plan row at line " +
                std::to_string(line_number);
            return false;
        }
        rows.push_back(row);
    }
    if (!header_seen || rows.size() < 20) {
        error = "plan is empty or too short";
        return false;
    }
    const char* required[] = {
        "schema", "status", "simulation_only", "formal_robot_release",
        "path_frame_id", "path_length", "path_sha256",
        "execution_contract_hash", "dt", "zero_hold_steps",
    };
    for (const char* key : required) {
        if (metadata.count(key) == 0) {
            error = std::string("plan metadata missing: ") + key;
            return false;
        }
    }
    if (metadata["schema"] != kPlanSchema ||
        metadata["status"] != "PASS" ||
        metadata["simulation_only"] != "true" ||
        metadata["formal_robot_release"] != "false" ||
        !lowercaseSha256(metadata["path_sha256"]) ||
        !lowercaseSha256(metadata["execution_contract_hash"])) {
        error = "plan provenance/status contract rejected";
        return false;
    }
    return true;
}

bool loadRecoveryScales(const std::string& path,
                        std::vector<RecoveryScaleRow>& rows,
                        std::string& error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        error = "cannot open recovery scales";
        return false;
    }
    const std::vector<std::string> expected = {
        "phase_index", "phase_bin_start", "phase_bin_end", "shrinkage",
        "r_x", "r_y", "r_yaw", "r_v", "r_omega", "r_eta_x",
        "r_eta_x_dot", "r_eta_y", "r_eta_y_dot",
        "beta_linear_output", "beta_angular_output",
        "beta_linear_pending_0", "beta_linear_pending_1",
        "beta_linear_pending_2", "beta_linear_pending_3",
        "beta_linear_pending_4", "beta_angular_pending_0",
        "beta_angular_pending_1", "beta_angular_pending_2",
        "beta_angular_pending_3", "beta_angular_pending_4",
        "beta_angular_pending_5", "beta_angular_pending_6",
    };
    std::string line;
    if (!std::getline(input, line) || splitCsv(line) != expected) {
        error = "recovery scales header mismatch";
        return false;
    }
    std::size_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        if (trim(line).empty()) continue;
        const std::vector<std::string> fields = splitCsv(line);
        if (fields.size() != expected.size()) {
            error = "recovery scales column mismatch";
            return false;
        }
        RecoveryScaleRow row;
        std::size_t bin_start = 0;
        std::size_t bin_end = 0;
        double shrinkage = 0.0;
        if (!parseIndex(fields[0], row.phase_index) ||
            !parseIndex(fields[1], bin_start) ||
            !parseIndex(fields[2], bin_end) ||
            !parseDouble(fields[3], shrinkage) ||
            row.phase_index != rows.size() || shrinkage <= 0.0 ||
            shrinkage > 1.0 || bin_start > row.phase_index ||
            bin_end < row.phase_index) {
            error = "invalid recovery scale identity at line " +
                std::to_string(line_number);
            return false;
        }
        double* radii[] = {
            &row.radii.x, &row.radii.y, &row.radii.yaw,
            &row.radii.v, &row.radii.omega, &row.radii.eta_x,
            &row.radii.eta_x_dot, &row.radii.eta_y,
            &row.radii.eta_y_dot,
        };
        std::size_t column = 4;
        for (double* radius : radii) {
            if (!parseDouble(fields[column++], *radius) ||
                *radius < 1.0e-9) {
                error = "invalid recovery radius";
                return false;
            }
        }
        row.execution.valid = true;
        if (!parseDouble(fields[column++],
                         row.execution.linear_actuator_output) ||
            !parseDouble(fields[column++],
                         row.execution.angular_actuator_output)) {
            error = "invalid actuator execution bound";
            return false;
        }
        row.execution.linear_pending_commands.resize(5);
        row.execution.angular_pending_commands.resize(7);
        for (double& bound : row.execution.linear_pending_commands) {
            if (!parseDouble(fields[column++], bound) || bound < 1.0e-9) {
                error = "invalid linear pending bound";
                return false;
            }
        }
        for (double& bound : row.execution.angular_pending_commands) {
            if (!parseDouble(fields[column++], bound) || bound < 1.0e-9) {
                error = "invalid angular pending bound";
                return false;
            }
        }
        if (row.execution.linear_actuator_output < 1.0e-9 ||
            row.execution.angular_actuator_output < 1.0e-9 ||
            column != fields.size()) {
            error = "invalid execution bound row";
            return false;
        }
        rows.push_back(row);
    }
    if (rows.empty()) {
        error = "recovery scales are empty";
        return false;
    }
    return true;
}

bool yamlString(const YAML::Node& node,
                const std::string& key,
                std::string& value) {
    try {
        if (!node[key] || !node[key].IsScalar()) return false;
        value = node[key].as<std::string>();
        return !value.empty();
    } catch (const YAML::Exception&) {
        return false;
    }
}

std::string sha256SidecarPath(const std::string& path) {
    const std::size_t separator = path.find_last_of('/');
    const std::size_t suffix = path.find_last_of('.');
    if (suffix == std::string::npos ||
        (separator != std::string::npos && suffix < separator)) {
        return path + ".sha256";
    }
    return path.substr(0, suffix) + ".sha256";
}

bool readFile(const std::string& path, std::string& contents) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) return false;
    std::ostringstream buffer;
    buffer << input.rdbuf();
    if (!input.good() && !input.eof()) return false;
    contents = buffer.str();
    return true;
}

bool manifestOutputBinding(const YAML::Node& outputs,
                           const std::string& label,
                           const std::string& schema,
                           const std::string& filename,
                           const std::string& manifest_path,
                           const std::string& actual_path) {
    try {
        const YAML::Node entry = outputs[label];
        if (!entry || entry["schema"].as<std::string>() != schema ||
            entry["path"].as<std::string>() != filename) {
            return false;
        }
        std::string bound;
        std::string error;
        return canonicalExistingPath(
                   parentName(manifest_path) + "/" + filename,
                   bound, error) &&
            (bound == actual_path || sameExistingFile(bound, actual_path));
    } catch (const std::exception&) {
        return false;
    }
}

bool yamlStringSequenceEquals(const YAML::Node& node,
                              const std::vector<std::string>& expected) {
    try {
        if (!node || !node.IsSequence() || node.size() != expected.size()) {
            return false;
        }
        for (std::size_t index = 0; index < expected.size(); ++index) {
            if (node[index].as<std::string>() != expected[index]) return false;
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool verifyRecoveryBundle(const Arguments& args,
                          std::string& dataset_path,
                          std::string& dataset_hash,
                          std::string& scales_hash,
                          std::string& manifest_hash,
                          std::string& report_hash,
                          std::string& error) {
    try {
        manifest_hash = sha256File(args.recovery_manifest);
        const std::string sidecar_path =
            sha256SidecarPath(args.recovery_manifest);
        std::string sidecar;
        if (!lowercaseSha256(manifest_hash) ||
            !readFile(sidecar_path, sidecar) ||
            sidecar != manifest_hash + "  " +
                baseName(args.recovery_manifest) + "\n") {
            error = "recovery manifest SHA-256 sidecar mismatch";
            return false;
        }
        const YAML::Node manifest = YAML::LoadFile(args.recovery_manifest);
        const YAML::Node report = YAML::LoadFile(args.held_out_report);
        std::string schema;
        std::string status;
        if (!yamlString(manifest, "schema", schema) ||
            schema != kFitManifestSchema ||
            !yamlString(manifest, "status", status) ||
            status != "EMPIRICAL_HELD_OUT_PASS" ||
            manifest["safety_certificate"].as<bool>() ||
            manifest["formal_robot_release"].as<bool>() ||
            manifest["physical_enforce_authorized"].as<bool>()) {
            error = "recovery fit manifest did not pass";
            return false;
        }
        if (!yamlString(report, "schema", schema) ||
            schema != kHeldOutReportSchema ||
            !yamlString(report, "status", status) || status != "PASS" ||
            report["safety_certificate"].as<bool>() ||
            report["formal_robot_release"].as<bool>() ||
            report["held_out_influenced_fit"].as<bool>() ||
            report["held_out_influenced_tuning"].as<bool>() ||
            report["held_out_evaluation_count"].as<std::size_t>() != 1) {
            error = "held-out recovery report did not pass";
            return false;
        }
        if (!manifest["input"] || !manifest["outputs"] ||
            !manifest["held_out"] || !manifest["split_contract"] ||
            !manifest["compiled_contract"] || !manifest["fit"] ||
            !manifest["tune"]) {
            error = "recovery fit manifest is incomplete";
            return false;
        }
        std::string declared_dataset_path =
            manifest["input"]["path"].as<std::string>();
        if (!canonicalExistingPath(
                declared_dataset_path, dataset_path, error)) {
            return false;
        }
        const std::string expected_dataset_hash =
            manifest["input"]["sha256"].as<std::string>();
        const std::vector<std::string> dataset_columns = {
            "split", "rollout_id", "seed", "phase_index", "recovered",
            "x", "y", "yaw", "v", "omega", "eta_x", "eta_x_dot",
            "eta_y", "eta_y_dot", "linear_output", "angular_output",
            "linear_pending_0", "linear_pending_1", "linear_pending_2",
            "linear_pending_3", "linear_pending_4", "angular_pending_0",
            "angular_pending_1", "angular_pending_2", "angular_pending_3",
            "angular_pending_4", "angular_pending_5", "angular_pending_6",
        };
        const std::string expected_scales_hash =
            manifest["outputs"]["scales"]["sha256"].as<std::string>();
        const std::string expected_report_hash =
            manifest["outputs"]["held_out_report"]["sha256"].as<std::string>();
        const std::string held_manifest_hash =
            manifest["held_out"]["report_sha256"].as<std::string>();
        dataset_hash = sha256File(dataset_path);
        scales_hash = sha256File(args.recovery_scales);
        report_hash = sha256File(args.held_out_report);
        if (!lowercaseSha256(dataset_hash) ||
            dataset_hash != expected_dataset_hash ||
            scales_hash != expected_scales_hash ||
            report_hash != expected_report_hash ||
            report_hash != held_manifest_hash ||
            report["input_sha256"].as<std::string>() != dataset_hash ||
            manifest["input"]["schema"].as<std::string>() !=
                kRecoveryDatasetSchema ||
            !yamlStringSequenceEquals(
                manifest["input"]["columns"], dataset_columns) ||
            manifest["outputs"]["scales"]["schema"].as<std::string>() !=
                kRecoveryScalesSchema ||
            manifest["outputs"]["held_out_report"]["schema"].as<std::string>() !=
                kHeldOutReportSchema) {
            error = "recovery bundle hash mismatch";
            return false;
        }
        if (!manifestOutputBinding(
                manifest["outputs"], "scales", kRecoveryScalesSchema,
                "phase_rejoin_recovery_radii_bounds.csv",
                args.recovery_manifest, args.recovery_scales) ||
            !manifestOutputBinding(
                manifest["outputs"], "held_out_report", kHeldOutReportSchema,
                "held_out_report.json", args.recovery_manifest,
                args.held_out_report)) {
            error = "recovery manifest output path/schema binding mismatch";
            return false;
        }
        const YAML::Node compiled = manifest["compiled_contract"];
        if (compiled["state_width"].as<std::size_t>() != 22 ||
            compiled["gate_radius_count"].as<std::size_t>() != 9 ||
            compiled["execution_bound_count"].as<std::size_t>() != 14 ||
            compiled["linear_pending_count"].as<std::size_t>() != 5 ||
            compiled["angular_pending_count"].as<std::size_t>() != 7 ||
            compiled["execution_compatibility_contract"].as<std::string>() !=
                "phase_indexed_execution_box_v1" ||
            std::abs(compiled["minimum_denominator"].as<double>() - 1.0e-9) >
                1.0e-18 ||
            manifest["fit"]["uses_only_split"].as<std::string>() != "fit" ||
            manifest["tune"]["uses_only_split"].as<std::string>() != "tune" ||
            manifest["held_out"]["uses_only_split"].as<std::string>() !=
                "held_out" ||
            manifest["held_out"]["evaluation_count"].as<std::size_t>() != 1 ||
            manifest["held_out"]["status"].as<std::string>() != "PASS" ||
            report["gate_contract"].as<std::string>() !=
                "phase_indexed_empirical_9d_ellipsoid_v1" ||
            report["execution_compatibility_contract"].as<std::string>() !=
                "phase_indexed_execution_box_v1") {
            error = "recovery fit/tune/held-out contract mismatch";
            return false;
        }
        const YAML::Node split = manifest["split_contract"];
        if (!split["mutually_exclusive"].as<bool>() ||
            split["unit"].as<std::string>() != "complete_rollout_and_seed") {
            error = "recovery split contract rejected";
            return false;
        }
        std::set<std::string> rollout_ids;
        std::set<std::uint32_t> seeds;
        for (const char* name : {"fit", "tune", "held_out"}) {
            const YAML::Node group = split[name];
            if (!group || !group["rollout_ids"] || !group["seeds"] ||
                group["rollout_count"].as<std::size_t>() == 0 ||
                group["rollout_count"].as<std::size_t>() !=
                    group["rollout_ids"].size() ||
                !lowercaseSha256(
                    group["canonical_rows_sha256"].as<std::string>())) {
                error = "recovery split is empty";
                return false;
            }
            for (const YAML::Node& value : group["rollout_ids"]) {
                if (!rollout_ids.insert(value.as<std::string>()).second) {
                    error = "rollout crosses recovery splits";
                    return false;
                }
            }
            for (const YAML::Node& value : group["seeds"]) {
                if (!seeds.insert(value.as<std::uint32_t>()).second) {
                    error = "seed crosses recovery splits";
                    return false;
                }
            }
        }
        if (sha256File(args.recovery_manifest) != manifest_hash ||
            sha256File(dataset_path) != dataset_hash ||
            sha256File(args.recovery_scales) != scales_hash ||
            sha256File(args.held_out_report) != report_hash) {
            error = "recovery bundle changed during verification";
            return false;
        }
    } catch (const std::exception& exception) {
        error = std::string("cannot verify recovery bundle: ") +
            exception.what();
        return false;
    }
    return true;
}

bool loadPath(const std::string& path, PathAsset& asset, std::string& error) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (!root["frame_id"] || !root["poses"] ||
            !root["poses"].IsSequence() || root["poses"].size() < 2) {
            error = "path JSON schema rejected";
            return false;
        }
        asset.frame_id = root["frame_id"].as<std::string>();
        if (asset.frame_id.empty()) {
            error = "path frame_id is empty";
            return false;
        }
        std::vector<spmpc::TrajectoryPoint> points;
        points.reserve(root["poses"].size());
        for (const YAML::Node& pose : root["poses"]) {
            const double qx = pose["qx"].as<double>();
            const double qy = pose["qy"].as<double>();
            const double qz = pose["qz"].as<double>();
            const double qw = pose["qw"].as<double>();
            const double scale = std::max({
                std::abs(qx), std::abs(qy), std::abs(qz), std::abs(qw)});
            spmpc::TrajectoryPoint point;
            point.x = pose["x"].as<double>();
            point.y = pose["y"].as<double>();
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(scale) || scale <= 1.0e-12) {
                error = "path contains invalid pose";
                return false;
            }
            const double scaled_x = qx / scale;
            const double scaled_y = qy / scale;
            const double scaled_z = qz / scale;
            const double scaled_w = qw / scale;
            const double norm = std::sqrt(
                scaled_x * scaled_x + scaled_y * scaled_y +
                scaled_z * scaled_z + scaled_w * scaled_w);
            const double x = scaled_x / norm;
            const double y = scaled_y / norm;
            const double z = scaled_z / norm;
            const double w = scaled_w / norm;
            point.yaw = std::atan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z));
            if (!std::isfinite(point.yaw) ||
                (!points.empty() &&
                 (!std::isfinite(std::hypot(
                      point.x - points.back().x,
                      point.y - points.back().y)) ||
                  std::hypot(point.x - points.back().x,
                             point.y - points.back().y) <= 1.0e-6))) {
                error = "path contains a duplicate or invalid segment";
                return false;
            }
            points.push_back(point);
        }
        asset.reference.setPoints(points, asset.frame_id);
        if (asset.reference.empty() ||
            !std::isfinite(asset.reference.length()) ||
            asset.reference.length() <= 0.0) {
            error = "path geometry is empty";
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        error = std::string("cannot parse path: ") + exception.what();
        return false;
    }
}

double angleError(double lhs, double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

bool validatePlanReport(const std::string& path,
                        const std::string& expected_plan_hash,
                        const std::string& expected_path_hash,
                        std::size_t expected_rows,
                        std::string& report_hash,
                        std::string& error) {
    try {
        const YAML::Node report = YAML::LoadFile(path);
        if (!report["schema"] ||
            report["schema"].as<std::string>() !=
                "spmpc_offline_slosh_ocp_report_v1" ||
            !report["status"] || report["status"].as<std::string>() != "PASS" ||
            !report["simulation_only"].as<bool>() ||
            !report["optimizer"] ||
            !report["optimizer"]["success"].as<bool>() ||
            !report["plan"] ||
            report["plan"]["sha256"].as<std::string>() !=
                expected_plan_hash ||
            report["plan"]["rows"].as<std::size_t>() != expected_rows ||
            report["path"]["sha256"].as<std::string>() != expected_path_hash ||
            report["terminal_contract"]["name"].as<std::string>() !=
                kTerminalContract ||
            !report["terminal_contract"]
                ["goal_yaw_preserved_from_path_quaternion"].as<bool>() ||
            report["formal_robot_release"].as<bool>() ||
            report["physical_parameter_claim"].as<bool>() ||
            !report["source_limitations_acknowledged"].as<bool>()) {
            error = "OfflineSloshOCP report did not pass";
            return false;
        }
        report_hash = sha256File(path);
        return lowercaseSha256(report_hash);
    } catch (const std::exception& exception) {
        error = std::string("cannot validate OfflineSloshOCP report: ") +
            exception.what();
        return false;
    }
}

std::string shellQuote(const std::string& value) {
    std::string quoted = "'";
    for (char character : value) {
        if (character == '\'') {
            quoted += "'\\''";
        } else {
            quoted.push_back(character);
        }
    }
    quoted += "'";
    return quoted;
}

bool runArtifactValidator(const std::string& executable,
                          const std::string& artifact,
                          const std::string& expected_recovery_hash,
                          std::string& error) {
    int descriptors[2];
    if (::pipe(descriptors) != 0) {
        error = "cannot create artifact-validator output pipe";
        return false;
    }
    const pid_t child = ::fork();
    if (child < 0) {
        ::close(descriptors[0]);
        ::close(descriptors[1]);
        error = "cannot start artifact validator";
        return false;
    }
    if (child == 0) {
        ::close(descriptors[0]);
        if (::dup2(descriptors[1], STDOUT_FILENO) < 0 ||
            ::dup2(descriptors[1], STDERR_FILENO) < 0) {
            _exit(126);
        }
        ::close(descriptors[1]);
        ::execl(executable.c_str(), executable.c_str(), "validate",
                "--artifact", artifact.c_str(),
                static_cast<char*>(nullptr));
        _exit(127);
    }
    ::close(descriptors[1]);
    std::string output;
    std::array<char, 4096> buffer{};
    while (true) {
        const ssize_t count = ::read(
            descriptors[0], buffer.data(), buffer.size());
        if (count > 0) {
            output.append(buffer.data(), static_cast<std::size_t>(count));
        } else if (count == 0) {
            break;
        } else if (errno != EINTR) {
            ::close(descriptors[0]);
            int ignored = 0;
            ::waitpid(child, &ignored, 0);
            error = "cannot read artifact-validator output";
            return false;
        }
    }
    ::close(descriptors[0]);
    int status = 0;
    while (::waitpid(child, &status, 0) < 0) {
        if (errno != EINTR) {
            error = "cannot wait for artifact validator";
            return false;
        }
    }
    const std::string expected =
        "recovery_artifact_hash=" + expected_recovery_hash;
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0 ||
        output.find(expected) == std::string::npos) {
        error = "artifact validator rejected output: " + trim(output);
        return false;
    }
    return true;
}

bool writeReport(const std::string& path,
                 const std::string& contents,
                 bool overwrite,
                 std::string& error) {
    std::vector<char> temporary(path.begin(), path.end());
    const std::string suffix = ".tmp.XXXXXX";
    temporary.insert(temporary.end(), suffix.begin(), suffix.end());
    temporary.push_back('\0');
    const int descriptor = ::mkstemp(temporary.data());
    if (descriptor < 0) {
        error = "cannot open temporary report";
        return false;
    }
    const std::string temporary_path(temporary.data());
    bool write_ok = true;
    std::size_t offset = 0;
    while (offset < contents.size()) {
        const ssize_t count = ::write(
            descriptor, contents.data() + offset, contents.size() - offset);
        if (count <= 0) {
            if (count < 0 && errno == EINTR) continue;
            write_ok = false;
            break;
        }
        offset += static_cast<std::size_t>(count);
    }
    if (write_ok && ::fsync(descriptor) != 0) write_ok = false;
    if (::close(descriptor) != 0) write_ok = false;
    if (!write_ok) {
        ::unlink(temporary_path.c_str());
        error = "cannot write temporary report";
        return false;
    }
    bool published = false;
    if (overwrite) {
        published = ::rename(temporary_path.c_str(), path.c_str()) == 0;
    } else {
        published = ::link(temporary_path.c_str(), path.c_str()) == 0;
        if (published) ::unlink(temporary_path.c_str());
    }
    if (!published) {
        const bool exists = !overwrite && errno == EEXIST;
        ::unlink(temporary_path.c_str());
        error = exists ? "report output already exists"
                       : "cannot publish nominal validation report";
        return false;
    }
    const int directory = ::open(
        parentName(path).c_str(), O_RDONLY | O_DIRECTORY);
    if (directory >= 0) {
        ::fsync(directory);
        ::close(directory);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Arguments args;
    if (!parseArguments(argc, argv, args)) return usage();

    std::string error;
    if (!resolveAndRejectPathAliases(args, error)) return usage(error);
    const std::string frozen_path_hash = sha256File(args.path_json);
    const std::string frozen_plan_hash = sha256File(args.plan_csv);
    const std::string frozen_plan_report_hash = sha256File(args.plan_report);
    const std::string frozen_scales_hash = sha256File(args.recovery_scales);
    const std::string frozen_manifest_hash = sha256File(args.recovery_manifest);
    const std::string frozen_manifest_sidecar_hash = sha256File(
        sha256SidecarPath(args.recovery_manifest));
    const std::string frozen_held_out_hash = sha256File(args.held_out_report);
    const std::string artifact_validator_hash =
        sha256File(args.artifact_validator);
    const std::string initial_hashes[] = {
        frozen_path_hash, frozen_plan_hash, frozen_plan_report_hash,
        frozen_scales_hash, frozen_manifest_hash,
        frozen_manifest_sidecar_hash, frozen_held_out_hash,
        artifact_validator_hash,
    };
    if (!std::all_of(std::begin(initial_hashes), std::end(initial_hashes),
                     lowercaseSha256)) {
        return usage("cannot freeze all builder inputs");
    }
    std::map<std::string, std::string> plan_metadata;
    std::vector<PlanRow> plan;
    if (!loadPlan(args.plan_csv, plan_metadata, plan, error)) {
        return usage(error);
    }
    const std::string plan_hash = frozen_plan_hash;
    const std::string path_hash = frozen_path_hash;
    if (plan_hash.empty() || path_hash != plan_metadata["path_sha256"]) {
        return usage("plan/path hash mismatch");
    }
    std::string plan_report_hash;
    if (!validatePlanReport(
            args.plan_report, plan_hash, path_hash, plan.size(),
            plan_report_hash, error)) {
        return usage(error);
    }

    std::vector<RecoveryScaleRow> recovery;
    if (!loadRecoveryScales(args.recovery_scales, recovery, error)) {
        return usage(error);
    }
    if (recovery.size() != plan.size()) {
        return usage("recovery phase coverage does not match the nominal plan");
    }
    std::string dataset_path;
    std::string dataset_hash;
    std::string scales_hash;
    std::string fit_manifest_hash;
    std::string held_out_report_hash;
    if (!verifyRecoveryBundle(
            args, dataset_path, dataset_hash, scales_hash, fit_manifest_hash,
            held_out_report_hash, error)) {
        return usage(error);
    }
    if (scales_hash != frozen_scales_hash ||
        fit_manifest_hash != frozen_manifest_hash ||
        held_out_report_hash != frozen_held_out_hash ||
        plan_report_hash != frozen_plan_report_hash) {
        return usage("builder input hash changed before validation");
    }
    const std::vector<std::string> explicit_paths = {
        args.path_json, args.plan_csv, args.plan_report, args.recovery_scales,
        args.recovery_manifest, args.held_out_report, args.artifact_validator,
        args.output, args.report_output,
    };
    for (const std::string& path : explicit_paths) {
        if (dataset_path == path || sameExistingFile(dataset_path, path)) {
            return usage("recovery dataset aliases another builder path");
        }
    }

    PathAsset path;
    if (!loadPath(args.path_json, path, error)) return usage(error);
    double declared_dt = 0.0;
    double declared_length = 0.0;
    std::size_t zero_hold_steps = 0;
    if (!parseDouble(plan_metadata["dt"], declared_dt) ||
        !parseDouble(plan_metadata["path_length"], declared_length) ||
        !parseIndex(plan_metadata["zero_hold_steps"], zero_hold_steps) ||
        zero_hold_steps < 5 || zero_hold_steps >= plan.size() ||
        path.frame_id != plan_metadata["path_frame_id"] ||
        std::abs(declared_length - path.reference.length()) > 1.0e-9) {
        return usage("plan/path geometry metadata mismatch");
    }

    const spmpc::DelayAugmentedPhaseCompiledContract compiled =
        spmpc::DelayAugmentedPhaseAcadosSolver::compiledContract();
    if (plan_metadata["execution_contract_hash"] !=
            compiled.execution.contract_hash ||
        std::abs(declared_dt - compiled.execution.dt) > 1.0e-12) {
        return usage("plan does not match the compiled execution contract");
    }
    spmpc::ExecutionModel execution_model;
    if (!execution_model.configure(
            compiled.execution, compiled.slosh, error)) {
        return usage("compiled execution model unavailable: " + error);
    }
    const spmpc::TrajectoryPoint start = path.reference.sample(0.0);
    spmpc::RobotState robot;
    robot.x = start.x;
    robot.y = start.y;
    robot.yaw = start.yaw;
    spmpc::SloshState slosh;
    spmpc::VelocityCommand held;
    spmpc::ExecutionAugmentedState execution;
    if (!execution_model.initializeHeld(
            robot, slosh, held, execution, error)) {
        return usage("cannot initialize nominal execution state: " + error);
    }

    std::vector<spmpc::PhaseNominalSample> samples;
    samples.reserve(plan.size());
    double progress = 0.0;
    double max_path_deviation = 0.0;
    for (std::size_t index = 0; index < plan.size(); ++index) {
        const PlanRow& control = plan[index];
        if (std::abs(control.t - static_cast<double>(index) * declared_dt) >
                1.0e-9 ||
            control.published_v < compiled.execution.linear.output_min - kTolerance ||
            control.published_v > compiled.execution.linear.output_max + kTolerance ||
            control.published_omega < compiled.execution.angular.output_min - kTolerance ||
            control.published_omega > compiled.execution.angular.output_max + kTolerance ||
            control.progress_rate < -kTolerance ||
            control.progress_rate > compiled.progress_rate_max + kTolerance) {
            return usage("plan control bound/time mismatch at index " +
                         std::to_string(index));
        }
        const double raw_a = (control.published_v -
            execution.linear.pending_commands.back()) / declared_dt;
        const double raw_alpha = (control.published_omega -
            execution.angular.pending_commands.back()) / declared_dt;
        if (std::abs(raw_a) > compiled.acceleration_max + kTolerance ||
            std::abs(raw_alpha) >
                compiled.angular_acceleration_max + kTolerance) {
            return usage("published command rate mismatch at index " +
                         std::to_string(index));
        }
        // The plan is serialized in decimal and the compiled dt is not exact
        // in binary.  Clamp only the sub-tolerance round-off at a generated
        // hard bound; the loader independently checks that a*dt still
        // reconstructs the published command within its 1e-9 contract.
        const double a = std::max(
            -compiled.acceleration_max,
            std::min(compiled.acceleration_max, raw_a));
        const double alpha = std::max(
            -compiled.angular_acceleration_max,
            std::min(compiled.angular_acceleration_max, raw_alpha));
        spmpc::PhaseNominalSample sample;
        sample.index = index;
        sample.t = control.t;
        sample.s = progress;
        sample.x = execution.robot.x;
        sample.y = execution.robot.y;
        sample.yaw = execution.robot.yaw;
        sample.v = execution.robot.v;
        sample.omega = execution.robot.omega;
        sample.eta_x = execution.slosh.eta_x;
        sample.eta_x_dot = execution.slosh.eta_x_dot;
        sample.eta_y = execution.slosh.eta_y;
        sample.eta_y_dot = execution.slosh.eta_y_dot;
        sample.a = a;
        sample.alpha = alpha;
        sample.v_s = control.progress_rate;
        sample.u_pub_v = control.published_v;
        sample.u_pub_omega = control.published_omega;
        sample.kappa_v = control.published_v;
        sample.kappa_omega = control.published_omega;
        sample.radii = recovery[index].radii;
        sample.augmented_execution_valid = true;
        sample.augmented_execution = execution;
        sample.execution_bounds = recovery[index].execution;
        samples.push_back(sample);
        const spmpc::TrajectoryPoint reference =
            path.reference.sample(progress);
        max_path_deviation = std::max(
            max_path_deviation,
            std::hypot(sample.x - reference.x, sample.y - reference.y));
        if (index + 1 < plan.size()) {
            spmpc::VelocityCommand published;
            published.linear = control.published_v;
            published.angular = control.published_omega;
            const spmpc::ExecutionStepResult step =
                execution_model.step(execution, published);
            if (!step.valid) {
                return usage("nominal execution step failed at index " +
                             std::to_string(index));
            }
            execution = step.state;
            progress += control.progress_rate * declared_dt;
            if (progress > declared_length &&
                progress - declared_length <= 1.0e-8) {
                progress = declared_length;
            }
        }
    }
    if (std::abs(samples.back().s - declared_length) > kTolerance) {
        return usage("nominal progress does not end at the path length");
    }
    const spmpc::TrajectoryPoint goal = path.reference.sample(declared_length);
    const double goal_position_error = std::hypot(
        samples.back().x - goal.x, samples.back().y - goal.y);
    const double goal_yaw_error = std::abs(
        angleError(samples.back().yaw, goal.yaw));
    if (max_path_deviation > args.max_path_deviation_m ||
        goal_position_error > args.goal_position_tolerance_m ||
        goal_yaw_error > args.goal_yaw_tolerance_rad) {
        return usage("nominal path/goal validation failed");
    }

    spmpc::SloshDynamics dynamics;
    if (!dynamics.configure(compiled.slosh)) {
        return usage("compiled liquid model unavailable");
    }
    const double omega_n = dynamics.omegaN();
    const spmpc::BoundedTrackingRecoveryPolicyParams recovery_policy =
        spmpc::boundedTrackingRecoveryPolicyV1Params();
    std::map<std::string, std::string> metadata = {
        {"schema", "phase_rejoin_empirical_augmented_v3"},
        {"evidence_level", "empirical_held_out"},
        {"source", "simulation_offline_slosh_ocp_held_out_recovery"},
        {"contract_id", args.contract_id},
        {"frame_id", path.frame_id},
        {"dt", number(declared_dt)},
        {"path_length", number(declared_length)},
        {"artifact_role", "simulation_phase_rejoin_nominal_and_recovery"},
        {"nominal_sequence_kind", "offline_slosh_ocp_complete_augmented_tail"},
        {"offline_slosh_ocp", "true"},
        {"hardware_formal_release", "false"},
        {"physical_parameter_claim", "false"},
        {"source_limitations_acknowledged", "true"},
        {"paper_main_result_eligible", "simulation_after_frozen_session_only"},
        {"terminal_contract", kTerminalContract},
        {"recovery_contract", recovery_policy.contract_id},
        {"recovery_policy_longitudinal_position_gain",
         number(recovery_policy.longitudinal_position_gain)},
        {"recovery_policy_lateral_position_gain",
         number(recovery_policy.lateral_position_gain)},
        {"recovery_policy_yaw_gain", number(recovery_policy.yaw_gain)},
        {"recovery_policy_linear_velocity_gain",
         number(recovery_policy.linear_velocity_gain)},
        {"recovery_policy_angular_velocity_gain",
         number(recovery_policy.angular_velocity_gain)},
        {"recovery_policy_max_residual_v",
         number(recovery_policy.max_residual_v)},
        {"recovery_policy_max_residual_omega",
         number(recovery_policy.max_residual_omega)},
        {"recovery_policy_published_linear_min",
         number(recovery_policy.published_linear_min)},
        {"recovery_policy_published_linear_max",
         number(recovery_policy.published_linear_max)},
        {"recovery_policy_published_angular_min",
         number(recovery_policy.published_angular_min)},
        {"recovery_policy_published_angular_max",
         number(recovery_policy.published_angular_max)},
        {"terminal_zero_hold_steps", std::to_string(zero_hold_steps)},
        {"terminal_eta_norm_max", number(args.terminal_eta_norm_max)},
        {"terminal_eta_dot_norm_max", number(args.terminal_eta_dot_norm_max)},
        {"terminal_v_abs_max", number(args.terminal_velocity_abs_max)},
        {"terminal_omega_abs_max", number(args.terminal_velocity_abs_max)},
        {"terminal_linear_actuator_output_abs_max",
         number(args.terminal_actuator_abs_max)},
        {"terminal_angular_actuator_output_abs_max",
         number(args.terminal_actuator_abs_max)},
        {"terminal_linear_pending_command_abs_max",
         number(args.terminal_pending_abs_max)},
        {"terminal_angular_pending_command_abs_max",
         number(args.terminal_pending_abs_max)},
        {"two_zeta_omega_n",
         number(2.0 * compiled.slosh.damping_ratio * omega_n)},
        {"omega_n_sq", number(omega_n * omega_n)},
        {"kappa_x", "1"},
        {"kappa_y", "1"},
        {"dynamics_tolerance", number(kTolerance)},
        {"execution_contract_id", compiled.execution.contract_id},
        {"execution_contract_hash", compiled.execution.contract_hash},
        {"execution_state_width", std::to_string(compiled.state_width)},
        {"execution_linear_buffer_count",
         std::to_string(compiled.execution.linear.integer_delay_steps + 1)},
        {"execution_angular_buffer_count",
         std::to_string(compiled.execution.angular.integer_delay_steps + 1)},
        {"parameter_schema_version",
         std::to_string(compiled.parameter_schema_version)},
        {"parameter_schema_id", compiled.parameter_schema_id},
        {"parameter_schema_hash", compiled.parameter_schema_hash},
        {"recovery_artifact_hash", std::string(64, '0')},
        {"execution_compatibility_contract",
         compiled.execution_compatibility_contract},
        {"path_sha256", path_hash},
        {"offline_plan_sha256", plan_hash},
        {"offline_plan_report_sha256", plan_report_hash},
        {"recovery_dataset_sha256", dataset_hash},
        {"recovery_scales_sha256", scales_hash},
        {"recovery_fit_manifest_sha256", fit_manifest_hash},
        {"recovery_held_out_report_sha256", held_out_report_hash},
        {"max_nominal_path_deviation_m", number(max_path_deviation)},
        {"goal_position_error_m", number(goal_position_error)},
        {"goal_yaw_error_rad", number(goal_yaw_error)},
    };
    metadata["recovery_artifact_hash"] =
        spmpc::NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);
    if (!lowercaseSha256(metadata["recovery_artifact_hash"])) {
        return usage("cannot compute canonical recovery artifact hash");
    }
    if (sha256File(args.path_json) != frozen_path_hash ||
        sha256File(args.plan_csv) != frozen_plan_hash ||
        sha256File(args.plan_report) != frozen_plan_report_hash ||
        sha256File(args.recovery_scales) != frozen_scales_hash ||
        sha256File(args.recovery_manifest) != frozen_manifest_hash ||
        sha256File(sha256SidecarPath(args.recovery_manifest)) !=
            frozen_manifest_sidecar_hash ||
        sha256File(args.held_out_report) != frozen_held_out_hash ||
        sha256File(args.artifact_validator) != artifact_validator_hash ||
        sha256File(dataset_path) != dataset_hash) {
        return usage("builder input changed during nominal construction");
    }
    spmpc::NominalSequenceArtifact artifact;
    const spmpc::NominalArtifactLoadResult assigned =
        artifact.assignValidated(metadata, samples, "<formal-simulation-builder>");
    if (!assigned.success) {
        return usage("production V3 loader rejected artifact: " +
                     assigned.status + ": " + assigned.detail);
    }
    const spmpc::NominalArtifactLoadResult written =
        artifact.writeCanonicalCsv(args.output, args.overwrite);
    if (!written.success) {
        return usage("cannot write artifact: " + written.status);
    }
    const std::string artifact_file_hash = sha256File(args.output);
    if (!lowercaseSha256(artifact_file_hash)) {
        std::remove(args.output.c_str());
        return usage("cannot hash written artifact");
    }
    if (!runArtifactValidator(
            args.artifact_validator, args.output,
            metadata["recovery_artifact_hash"], error) ||
        sha256File(args.output) != artifact_file_hash ||
        sha256File(args.artifact_validator) != artifact_validator_hash) {
        std::remove(args.output.c_str());
        if (error.empty()) {
            error = "artifact or validator changed during external validation";
        }
        return usage(error);
    }
    const std::string validation_command =
        shellQuote(args.artifact_validator) + " validate --artifact " +
        shellQuote(args.output);

    std::ostringstream report;
    report.imbue(std::locale::classic());
    report << std::setprecision(17)
           << "{\n"
           << "  \"schema\": \"" << kNominalReportSchema << "\",\n"
           << "  \"status\": \"PASS\",\n"
           << "  \"simulation_only\": true,\n"
           << "  \"formal_robot_release\": false,\n"
           << "  \"physical_parameter_claim\": false,\n"
           << "  \"source_limitations_acknowledged\": true,\n"
           << "  \"production_v3_loader_passed\": true,\n"
           << "  \"all_constraints_satisfied\": true,\n"
           << "  \"dynamics_consistency_passed\": true,\n"
           << "  \"has_publish_zero_settle_hold\": true,\n"
           << "  \"validation_data_used_for_optimization\": false,\n"
           << "  \"artifact\": {\n"
           << "    \"path\": \"" << escapeJson(args.output) << "\",\n"
           << "    \"sha256\": \"" << artifact_file_hash << "\",\n"
           << "    \"recovery_artifact_hash\": \""
           << metadata["recovery_artifact_hash"] << "\",\n"
           << "    \"rows\": " << samples.size() << "\n"
           << "  },\n"
           << "  \"phase_rejoin_artifact_sha256\": \""
           << artifact_file_hash << "\",\n"
           << "  \"path_sha256\": \"" << path_hash << "\",\n"
           << "  \"offline_plan_sha256\": \"" << plan_hash << "\",\n"
           << "  \"offline_plan_report_sha256\": \""
           << plan_report_hash << "\",\n"
           << "  \"recovery_fit_manifest_sha256\": \""
           << fit_manifest_hash << "\",\n"
           << "  \"recovery_radii_bounds_sha256\": \""
           << scales_hash << "\",\n"
           << "  \"recovery_held_out_report_sha256\": \""
           << held_out_report_hash << "\",\n"
           << "  \"artifact_validator_path\": \""
           << escapeJson(args.artifact_validator) << "\",\n"
           << "  \"artifact_validator_sha256\": \""
           << artifact_validator_hash << "\",\n"
           << "  \"validation_command\": \""
           << escapeJson(validation_command) << "\",\n"
           << "  \"max_path_deviation_m\": " << max_path_deviation << ",\n"
           << "  \"goal_position_error_m\": " << goal_position_error << ",\n"
           << "  \"goal_yaw_error_rad\": " << goal_yaw_error << ",\n"
           << "  \"recovery_policy\": {\n"
           << "    \"contract_id\": \""
           << recovery_policy.contract_id << "\",\n"
           << "    \"longitudinal_position_gain\": "
           << recovery_policy.longitudinal_position_gain << ",\n"
           << "    \"lateral_position_gain\": "
           << recovery_policy.lateral_position_gain << ",\n"
           << "    \"yaw_gain\": " << recovery_policy.yaw_gain << ",\n"
           << "    \"linear_velocity_gain\": "
           << recovery_policy.linear_velocity_gain << ",\n"
           << "    \"angular_velocity_gain\": "
           << recovery_policy.angular_velocity_gain << ",\n"
           << "    \"max_residual_v\": "
           << recovery_policy.max_residual_v << ",\n"
           << "    \"max_residual_omega\": "
           << recovery_policy.max_residual_omega << ",\n"
           << "    \"published_linear_min\": "
           << recovery_policy.published_linear_min << ",\n"
           << "    \"published_linear_max\": "
           << recovery_policy.published_linear_max << ",\n"
           << "    \"published_angular_min\": "
           << recovery_policy.published_angular_min << ",\n"
           << "    \"published_angular_max\": "
           << recovery_policy.published_angular_max << "\n"
           << "  },\n"
           << "  \"terminal_contract\": \"" << kTerminalContract << "\",\n"
           << "  \"terminal_zero_hold_steps\": " << zero_hold_steps << "\n"
           << "}\n";
    if (!writeReport(
            args.report_output, report.str(), args.overwrite, error)) {
        std::remove(args.output.c_str());
        return usage(error);
    }
    std::cout << "[PASS] formal simulation V3 artifact rows=" << samples.size()
              << " hash=" << metadata["recovery_artifact_hash"]
              << " path_deviation_m=" << max_path_deviation << '\n';
    return 0;
}
