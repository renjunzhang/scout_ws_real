#include "spmpc_local_planner/bt_residual/bt_residual_structure.h"
#include "spmpc_local_planner/controller/command/publication_transaction.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/provenance/compiled_source_identity.h"
#include "spmpc_delay_augmented_phase_manifest.h"

#include <OsqpEigen/OsqpEigen.h>
#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace spmpc = spmpc_local_planner;
namespace btr = spmpc_local_planner::bt_residual;
namespace execution_manifest =
    spmpc_local_planner::delay_augmented_phase_manifest;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

const char* compiledBuilderSourceHead() {
#ifdef SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
#else
    return "UNBOUND_BUILD_SOURCE_HEAD";
#endif
}

bool compiledDependencyHeadsMatch(const std::string& expected,
                                  std::string& mismatch) {
    const std::array<std::pair<const char*, const char*>, 9> identities = {{
        {"builder", compiledBuilderSourceHead()},
        {"bt_residual_core", btr::compiledSourceHead()},
        {"model", spmpc::provenance::modelCompiledSourceHead()},
        {"controller", spmpc::provenance::controllerCompiledSourceHead()},
        {"phase_rejoin", spmpc::provenance::phaseRejoinCompiledSourceHead()},
        {"safety", spmpc::provenance::safetyCompiledSourceHead()},
        {"reference", spmpc::provenance::referenceCompiledSourceHead()},
        {"runtime", spmpc::provenance::runtimeCompiledSourceHead()},
        {"estimation", spmpc::provenance::estimationCompiledSourceHead()},
    }};
    for (const auto& identity : identities) {
        if (identity.second == nullptr || expected != identity.second) {
            mismatch = identity.first;
            return false;
        }
    }
    return true;
}

struct Arguments {
    std::string config;
    std::string artifact;
    std::string output_dir;
    std::string source_head;
};

struct CampaignConfig {
    std::string schema;
    std::string freeze_id;
    btr::StructuralContract structure;
    spmpc::ExecutionModelContract execution;
    spmpc::SloshModelParams slosh;
    btr::StateVector tracking_scales = btr::StateVector::Zero();
    double liquid_weight = 0.0;
    double residual_weight = 0.0;
    std::vector<std::size_t> nominal_phases;
    std::string d1_id;
    double d1_lateral_m = 0.0;
    double d1_yaw_rad = 0.0;
    std::vector<double> d1_scales;
    std::string d2_id;
    std::size_t d2_begin = 0;
    int d2_cycles = 0;
    double d2_cap = 0.0;
    std::vector<int> d2_snapshots;
    std::string qp_solver;
    int qp_maximum_iterations = 0;
    double qp_absolute_tolerance = 0.0;
    double qp_relative_tolerance = 0.0;
    bool qp_polishing = false;
    double qp_hessian_regularization = 0.0;
    double qp_constraint_audit_tolerance_multiplier = 0.0;
    double qp_value_tolerance_multiplier = 0.0;
    double qp_tracking_multiplier_initial = 0.0;
    double qp_tracking_multiplier_growth = 0.0;
    int qp_tracking_multiplier_expansions = 0;
    int qp_tracking_multiplier_bisection_iterations = 0;
    std::vector<double> nonlinear_validation_scales;
    double minimum_useful_fraction_each = 0.0;
    std::string failure_decision;
    std::string success_decision;
};

struct Scenario {
    std::string id;
    std::string disturbance;
    std::size_t phase_index = 0;
    btr::AugmentedState15 state;
    std::vector<btr::StagePublicationConstraint> publications;
    bool require_useful_candidate = false;
};

struct CostSummary {
    bool valid = false;
    double tracking = 0.0;
    double liquid = 0.0;
    double residual = 0.0;
    double objective = 0.0;
};

struct ScenarioResult {
    std::string id;
    std::string disturbance;
    std::size_t phase_index = 0;
    bool evidence_valid = false;
    bool baseline_valid = false;
    bool zero_residual_oracle_passed = false;
    double zero_residual_maximum_error = 0.0;
    bool baseline_inside_tube = false;
    bool baseline_recovered = false;
    bool baseline_nonlinear_recovered = false;
    double baseline_tracking = 0.0;
    double baseline_objective = 0.0;
    double baseline_tube_margin = 0.0;
    bool useful_candidate = false;
    double best_tracking = 0.0;
    double best_objective = 0.0;
    double relative_tracking_improvement = 0.0;
    double absolute_tracking_improvement = 0.0;
    double maximum_residual_v = 0.0;
    double maximum_residual_omega = 0.0;
    double terminal_liquid_increment_eta = 0.0;
    double terminal_liquid_increment_eta_dot = 0.0;
    double candidate_tube_margin = 0.0;
    double maximum_candidate_path_fraction = 0.0;
    std::size_t candidates_tested = 0;
    std::size_t candidates_dynamically_valid = 0;
    std::size_t candidates_terminal_feasible = 0;
    bool qp_solved = false;
    int qp_variable_count = 0;
    int qp_constraint_count = 0;
    double qp_maximum_constraint_violation = 0.0;
    double qp_predicted_tracking = 0.0;
    double qp_predicted_objective = 0.0;
    double qp_tracking_target = 0.0;
    double qp_objective_target = 0.0;
    double qp_tracking_multiplier = 0.0;
    bool qp_certificate_valid = false;
    bool qp_solution_audited = false;
    double qp_certificate_lower_bound = 0.0;
    double qp_certificate_threshold = 0.0;
    std::string qp_status = "NOT_RUN";
    double best_validation_scale = 0.0;
    std::vector<btr::ResidualVector> accepted_residuals;
    std::string status = "NOT_RUN";
};

struct QpCandidate {
    bool valid = false;
    bool solved = false;
    bool certificate_valid = false;
    bool solution_audited = false;
    std::string status = "NOT_RUN";
    std::vector<btr::ResidualVector> residuals;
    int variable_count = 0;
    int constraint_count = 0;
    double maximum_constraint_violation = 0.0;
    double predicted_tracking = 0.0;
    double predicted_objective = 0.0;
    double tracking_target = 0.0;
    double objective_target = 0.0;
    double tracking_multiplier = 0.0;
    double certificate_lower_bound = 0.0;
    double certificate_threshold = 0.0;
};

struct FileDigest {
    std::string path;
    std::string sha256;
};

std::string jsonEscape(const std::string& value) {
    std::ostringstream output;
    for (char character : value) {
        switch (character) {
        case '\\': output << "\\\\"; break;
        case '"': output << "\\\""; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default: output << character; break;
        }
    }
    return output.str();
}

std::string jsonNumber(double value) {
    if (!std::isfinite(value)) return "null";
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(17) << value;
    return output.str();
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
                &context, buffer.data(),
                static_cast<std::size_t>(count)) != 1) {
            return std::string();
        }
    }
    if (!input.eof()) return std::string();
    std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
    if (::SHA256_Final(digest.data(), &context) != 1) return std::string();
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (unsigned char byte : digest) {
        output << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return output.str();
}

std::string trimProcessOutput(std::string value) {
    while (!value.empty() &&
           (value.back() == '\n' || value.back() == '\r' ||
            value.back() == ' ' || value.back() == '\t')) {
        value.pop_back();
    }
    return value;
}

bool runProcessCapture(const std::vector<std::string>& arguments,
                       std::string& output,
                       std::string& error) {
    output.clear();
    if (arguments.empty()) {
        error = "empty process arguments";
        return false;
    }
    int descriptors[2];
    if (::pipe(descriptors) != 0) {
        error = "pipe failed, errno=" + std::to_string(errno);
        return false;
    }
    const pid_t child = ::fork();
    if (child < 0) {
        ::close(descriptors[0]);
        ::close(descriptors[1]);
        error = "fork failed, errno=" + std::to_string(errno);
        return false;
    }
    if (child == 0) {
        ::close(descriptors[0]);
        if (::dup2(descriptors[1], STDOUT_FILENO) < 0 ||
            ::dup2(descriptors[1], STDERR_FILENO) < 0) {
            ::_exit(126);
        }
        ::close(descriptors[1]);
        std::vector<char*> argv;
        argv.reserve(arguments.size() + 1);
        for (const std::string& argument : arguments) {
            argv.push_back(const_cast<char*>(argument.c_str()));
        }
        argv.push_back(nullptr);
        ::execvp(argv.front(), argv.data());
        ::_exit(127);
    }
    ::close(descriptors[1]);
    std::array<char, 4096> buffer{};
    for (;;) {
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
            error = "process output read failed, errno=" +
                std::to_string(errno);
            return false;
        }
    }
    ::close(descriptors[0]);
    int status = 0;
    while (::waitpid(child, &status, 0) < 0) {
        if (errno != EINTR) {
            error = "waitpid failed, errno=" + std::to_string(errno);
            return false;
        }
    }
    output = trimProcessOutput(output);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        error = "process failed: " + output;
        return false;
    }
    return true;
}

bool verifyGitProvenance(const Arguments& args,
                         std::string& repository_root,
                         std::string& error) {
    std::string actual_head;
    std::string tracked_status;
    if (!runProcessCapture(
            {"git", "rev-parse", "--show-toplevel"},
            repository_root, error) ||
        repository_root.empty() ||
        !runProcessCapture(
            {"git", "-C", repository_root, "rev-parse", "HEAD"},
            actual_head, error) ||
        actual_head != args.source_head ||
        !runProcessCapture(
            {"git", "-C", repository_root, "status", "--porcelain=v1",
             "--untracked-files=no"},
            tracked_status, error)) {
        if (error.empty()) {
            error = "provided source HEAD does not match repository HEAD";
        }
        return false;
    }
    if (!tracked_status.empty()) {
        error = "tracked source tree is not clean";
        return false;
    }
    return true;
}

const std::vector<std::string>& frozenSourceRelativePaths() {
    static const std::vector<std::string> paths = {
        "src/scout_apps/control/spmpc_local_planner/CMakeLists.txt",
        "src/scout_apps/control/spmpc_local_planner/config/simulation/"
            "bt_residual_structural_go_no_go_v1.yaml",
        "src/scout_apps/control/spmpc_local_planner/generated/casadi/"
            "spmpc_delay_augmented_phase_manifest.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/bt_residual/bt_residual_structure.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/controller/command/command_pipeline.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/controller/command/publication_transaction.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/dynamics/slosh_dynamics.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/phase_rejoin/"
            "bounded_tracking_recovery_policy.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/phase_rejoin/"
            "nominal_sequence_artifact.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/phase_rejoin/types.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/provenance/"
            "compiled_source_identity.h",
        "src/scout_apps/control/spmpc_local_planner/include/"
            "spmpc_local_planner/runtime/execution_prediction/"
            "execution_model.h",
        "src/scout_apps/control/spmpc_local_planner/src/bt_residual/"
            "bt_residual_structure.cpp",
        "src/scout_apps/control/spmpc_local_planner/src/bt_residual/"
            "bt_residual_bt_oracle.cpp",
        "src/scout_apps/control/spmpc_local_planner/src/controller/command/"
            "command_pipeline.cpp",
        "src/scout_apps/control/spmpc_local_planner/src/controller/command/"
            "publication_transaction.cpp",
        "src/scout_apps/control/spmpc_local_planner/src/dynamics/"
            "slosh_dynamics.cpp",
        "src/scout_apps/control/spmpc_local_planner/src/phase_rejoin/"
            "bounded_tracking_recovery_policy.cpp",
        "src/scout_apps/control/spmpc_local_planner/src/phase_rejoin/"
            "nominal_sequence_artifact.cpp",
        "src/scout_apps/control/spmpc_local_planner/src/provenance/"
            "compiled_source_identity.cpp",
        "src/scout_apps/control/spmpc_local_planner/src/runtime/"
            "execution_prediction/execution_model.cpp",
        "src/scout_apps/control/spmpc_local_planner/test/"
            "test_bt_residual_structure.cpp",
        "src/scout_apps/control/spmpc_local_planner/tools/simulation/"
            "build_bt_residual_structure_bundle.cpp",
    };
    return paths;
}

bool verifyFrozenSourcesTracked(const std::string& repository_root,
                                std::string& error) {
    std::vector<std::string> command = {
        "git", "-C", repository_root, "ls-files", "--error-unmatch", "--"};
    const std::vector<std::string>& paths = frozenSourceRelativePaths();
    command.insert(command.end(), paths.begin(), paths.end());
    std::string tracked_paths;
    if (!runProcessCapture(command, tracked_paths, error)) {
        error = "frozen source is untracked: " + error;
        return false;
    }
    return true;
}

bool collectSourceDigests(const std::string& repository_root,
                          std::vector<FileDigest>& digests,
                          std::string& error) {
    digests.clear();
    for (const std::string& relative : frozenSourceRelativePaths()) {
        FileDigest digest;
        digest.path = relative;
        digest.sha256 = sha256File(repository_root + "/" + relative);
        if (digest.sha256.empty()) {
            error = "cannot hash frozen source: " + digest.path;
            return false;
        }
        digests.push_back(std::move(digest));
    }
    return true;
}

bool collectOutputDigests(const std::vector<std::string>& paths,
                          std::vector<FileDigest>& digests) {
    digests.clear();
    for (const std::string& path : paths) {
        FileDigest digest;
        digest.path = path;
        digest.sha256 = sha256File(path);
        if (digest.sha256.empty()) return false;
        digests.push_back(std::move(digest));
    }
    return true;
}

bool collectRuntimeDependencyDigests(
    const std::string& repository_root,
    std::vector<FileDigest>& digests,
    std::string& error) {
    std::ifstream maps("/proc/self/maps");
    if (!maps.is_open()) {
        error = "cannot read /proc/self/maps";
        return false;
    }
    std::set<std::string> paths;
    std::string line;
    while (std::getline(maps, line)) {
        const std::size_t path_begin = line.find('/');
        if (path_begin == std::string::npos) continue;
        std::string path = line.substr(path_begin);
        const std::string deleted_suffix = " (deleted)";
        if (path.size() >= deleted_suffix.size() &&
            path.compare(path.size() - deleted_suffix.size(),
                         deleted_suffix.size(), deleted_suffix) == 0) {
            error = "mapped runtime dependency was deleted: " + path;
            return false;
        }
        if (path.find(".so") != std::string::npos) paths.insert(path);
    }
    const std::string required_local_prefix = repository_root +
        "/devel/.private/spmpc_local_planner/lib/";
    const std::set<std::string> required_spmpc_libraries = {
        "libspmpc_bt_residual_structure.so",
        "libspmpc_controller.so",
        "libspmpc_estimation.so",
        "libspmpc_model.so",
        "libspmpc_phase_rejoin.so",
        "libspmpc_reference.so",
        "libspmpc_runtime.so",
        "libspmpc_safety.so",
    };
    std::set<std::string> mapped_spmpc_libraries;
    digests.clear();
    for (const std::string& path : paths) {
        const std::size_t separator = path.find_last_of('/');
        const std::string basename = separator == std::string::npos
            ? path
            : path.substr(separator + 1);
        if (basename.find("libspmpc_") == 0) {
            if (path.find(required_local_prefix) != 0) {
                error = "stale or foreign spmpc runtime library: " + path;
                return false;
            }
            if (required_spmpc_libraries.count(basename) == 0) {
                error = "unexpected spmpc runtime library: " + path;
                return false;
            }
            mapped_spmpc_libraries.insert(basename);
        }
        FileDigest digest;
        digest.path = path;
        digest.sha256 = sha256File(path);
        if (digest.sha256.empty()) {
            error = "cannot hash mapped runtime dependency: " + path;
            return false;
        }
        digests.push_back(std::move(digest));
    }
    if (mapped_spmpc_libraries != required_spmpc_libraries) {
        error = "required spmpc runtime library set is incomplete";
        return false;
    }
    return true;
}

template <typename T>
bool required(const YAML::Node& node,
              const char* key,
              T& value,
              std::string& error) {
    try {
        if (!node || !node[key] || !node[key].IsScalar()) {
            error = std::string("missing scalar: ") + key;
            return false;
        }
        value = node[key].as<T>();
        return true;
    } catch (const std::exception& exception) {
        error = std::string("invalid scalar ") + key + ": " +
            exception.what();
        return false;
    }
}

template <typename T>
bool requiredVector(const YAML::Node& node,
                    const char* key,
                    std::vector<T>& values,
                    std::string& error) {
    try {
        if (!node || !node[key] || !node[key].IsSequence() ||
            node[key].size() == 0) {
            error = std::string("missing sequence: ") + key;
            return false;
        }
        values.clear();
        for (const YAML::Node& item : node[key]) {
            values.push_back(item.as<T>());
        }
        return true;
    } catch (const std::exception& exception) {
        error = std::string("invalid sequence ") + key + ": " +
            exception.what();
        return false;
    }
}

bool requiredStateVector(const YAML::Node& node,
                         const char* key,
                         btr::StateVector& values,
                         std::string& error) {
    std::vector<double> parsed;
    if (!requiredVector(node, key, parsed, error)) return false;
    if (parsed.size() != static_cast<std::size_t>(btr::kStateWidth)) {
        error = std::string(key) + " must contain exactly 15 values";
        return false;
    }
    for (int index = 0; index < btr::kStateWidth; ++index) {
        values[index] = parsed[static_cast<std::size_t>(index)];
    }
    return true;
}

bool loadChannel(const YAML::Node& node,
                 spmpc::ExecutionChannelContract& channel,
                 std::string& error) {
    return required(node, "delay_sec", channel.delay_sec, error) &&
        required(node, "time_constant_sec", channel.time_constant_sec,
                 error) &&
        required(node, "positive_gain", channel.positive_gain, error) &&
        required(node, "negative_gain", channel.negative_gain, error) &&
        required(node, "deadzone", channel.deadzone, error) &&
        required(node, "output_min", channel.output_min, error) &&
        required(node, "output_max", channel.output_max, error);
}

bool matchesCompiledExecutionContract(const CampaignConfig& config) {
    auto same = [](double lhs, double rhs) {
        return std::abs(lhs - rhs) <=
            1.0e-15 * std::max({1.0, std::abs(lhs), std::abs(rhs)});
    };
    const spmpc::ExecutionChannelContract& linear = config.execution.linear;
    const spmpc::ExecutionChannelContract& angular =
        config.execution.angular;
    return config.execution.schema_version ==
            execution_manifest::kSchemaVersion &&
        config.execution.contract_id == execution_manifest::kContractId &&
        config.execution.contract_hash == execution_manifest::kContractHash &&
        config.structure.expected_execution_contract_hash ==
            execution_manifest::kContractHash &&
        same(config.execution.dt, execution_manifest::kDt) &&
        same(linear.delay_sec, execution_manifest::kLinearDelaySec) &&
        same(linear.time_constant_sec,
             execution_manifest::kLinearTimeConstantSec) &&
        same(linear.positive_gain, 1.0) &&
        same(linear.negative_gain, 1.0) &&
        same(linear.deadzone, 0.0) &&
        same(linear.output_min, execution_manifest::kLinearOutputMin) &&
        same(linear.output_max, execution_manifest::kLinearOutputMax) &&
        same(angular.delay_sec, execution_manifest::kAngularDelaySec) &&
        same(angular.time_constant_sec,
             execution_manifest::kAngularTimeConstantSec) &&
        same(angular.positive_gain, 1.0) &&
        same(angular.negative_gain, 1.0) &&
        same(angular.deadzone, 0.0) &&
        same(angular.output_min, execution_manifest::kAngularOutputMin) &&
        same(angular.output_max, execution_manifest::kAngularOutputMax) &&
        same(config.slosh.container_radius,
             execution_manifest::kContainerRadius) &&
        same(config.slosh.liquid_height,
             execution_manifest::kLiquidHeight) &&
        same(config.slosh.liquid_density,
             execution_manifest::kLiquidDensity) &&
        same(config.slosh.damping_ratio,
             execution_manifest::kDampingRatio) &&
        config.slosh.mode_index == execution_manifest::kModeIndex &&
        same(config.slosh.dt, execution_manifest::kDt) &&
        config.slosh.use_linear_model &&
        !config.slosh.use_parabola_term;
}

bool loadConfig(const std::string& path,
                CampaignConfig& config,
                std::string& error) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
    bool scope_development = false;
    bool online_solver = true;
    bool plant_admission = true;
    bool robust_claim = true;
    bool one_shot = false;
    const YAML::Node scope = root["scope"];
    const YAML::Node method = root["method"];
    const YAML::Node execution = root["execution_model"];
    const YAML::Node slosh = root["controller_slosh_model"];
    const YAML::Node horizon = root["horizon"];
    const YAML::Node publication = root["publication_contract"];
    const YAML::Node residual = root["residual_contract"];
    const YAML::Node linearization = root["linearization"];
    const YAML::Node terminal = root["terminal_contract"];
    const YAML::Node dominance = root["model_dominance"];
    const YAML::Node envelopes = root["state_envelopes"];
    const YAML::Node d1 = envelopes["d1"];
    const YAML::Node d2 = envelopes["d2"];
    const YAML::Node search = root["candidate_search"];
    const YAML::Node decision = root["decision"];
    bool require_baselines = false;
    bool require_identity = false;
    bool require_finite = false;
    if (!required(root, "schema", config.schema, error) ||
        !required(root, "freeze_id", config.freeze_id, error) ||
        !required(scope, "development_only", scope_development, error) ||
        !required(scope, "online_solver_implemented", online_solver,
                  error) ||
        !required(scope, "independent_plant_used_for_admission",
                  plant_admission, error) ||
        !required(scope, "robust_invariance_claim", robust_claim, error) ||
        !required(scope, "one_shot_no_retune", one_shot, error) ||
        !required(method, "schema", config.structure.schema, error) ||
        !required(method, "implementation_id",
                  config.structure.implementation_id, error) ||
        !required(method, "claim_level", config.structure.claim_level,
                  error) ||
        !required(method, "expected_artifact_sha256",
                  config.structure.expected_artifact_sha256, error) ||
        !required(method, "expected_artifact_contract_id",
                  config.structure.expected_artifact_contract_id, error) ||
        !required(method, "expected_execution_contract_hash",
                  config.structure.expected_execution_contract_hash,
                  error) ||
        !required(method, "expected_bt_policy_contract_id",
                  config.structure.expected_bt_policy_contract_id, error) ||
        !required(execution, "schema_version",
                  config.execution.schema_version, error) ||
        !required(execution, "contract_id", config.execution.contract_id,
                  error) ||
        !required(execution, "contract_hash",
                  config.execution.contract_hash, error) ||
        !required(execution, "dt", config.execution.dt, error) ||
        !loadChannel(execution["linear"], config.execution.linear, error) ||
        !loadChannel(execution["angular"], config.execution.angular,
                     error) ||
        !required(slosh, "container_radius", config.slosh.container_radius,
                  error) ||
        !required(slosh, "liquid_height", config.slosh.liquid_height,
                  error) ||
        !required(slosh, "liquid_density", config.slosh.liquid_density,
                  error) ||
        !required(slosh, "damping_ratio", config.slosh.damping_ratio,
                  error) ||
        !required(slosh, "mode_index", config.slosh.mode_index, error) ||
        !required(slosh, "dt", config.slosh.dt, error) ||
        !required(slosh, "slosh_height_ref",
                  config.slosh.slosh_height_ref, error) ||
        !required(slosh, "slosh_eta_dot_ratio",
                  config.slosh.slosh_eta_dot_ratio, error) ||
        !required(slosh, "use_linear_model",
                  config.slosh.use_linear_model, error) ||
        !required(slosh, "use_parabola_term",
                  config.slosh.use_parabola_term, error) ||
        !required(horizon, "residual_prefix_steps",
                  config.structure.residual_prefix_steps, error) ||
        !required(horizon, "recovery_suffix_steps",
                  config.structure.recovery_suffix_steps, error) ||
        !required(horizon, "authority_taper_begin_index",
                  config.structure.authority_taper_begin_index, error) ||
        !required(horizon, "authority_zero_index",
                  config.structure.authority_zero_index, error) ||
        !required(publication, "maximum_published_acceleration",
                  config.structure.maximum_published_acceleration, error) ||
        !required(publication,
                  "maximum_published_angular_acceleration",
                  config.structure.maximum_published_angular_acceleration,
                  error) ||
        !required(residual, "maximum_v",
                  config.structure.maximum_residual_v, error) ||
        !required(residual, "maximum_omega",
                  config.structure.maximum_residual_omega, error) ||
        !required(residual, "maximum_slew_v",
                  config.structure.maximum_residual_slew_v, error) ||
        !required(residual, "maximum_slew_omega",
                  config.structure.maximum_residual_slew_omega, error) ||
        !required(residual, "cumulative_progress_budget_m",
                  config.structure.cumulative_progress_budget_m, error) ||
        !required(residual, "cumulative_yaw_budget_rad",
                  config.structure.cumulative_yaw_budget_rad, error) ||
        !required(residual, "minimum_nonzero",
                  config.structure.minimum_nonzero_residual, error) ||
        !required(linearization, "relative_step",
                  config.structure.finite_difference_relative_step,
                  error) ||
        !required(linearization, "maximum_reconstruction_error",
                  config.structure
                      .maximum_finite_difference_reconstruction_error,
                  error) ||
        !required(linearization, "identity_tolerance",
                  config.structure.identity_tolerance, error) ||
        !requiredStateVector(linearization, "state_scales",
                  config.structure.finite_difference_scales, error) ||
        !requiredStateVector(terminal, "deviation_bounds",
                  config.structure.terminal_deviation_bounds, error) ||
        !requiredStateVector(terminal, "absolute_bounds",
                  config.structure.terminal_absolute_bounds, error) ||
        !required(terminal, "liquid_increment_eta",
                  config.structure.terminal_liquid_increment_eta, error) ||
        !required(terminal, "liquid_increment_eta_dot",
                  config.structure.terminal_liquid_increment_eta_dot,
                  error) ||
        !required(terminal, "maximum_absolute_eta",
                  config.structure.maximum_absolute_eta, error) ||
        !required(terminal, "maximum_absolute_eta_dot",
                  config.structure.maximum_absolute_eta_dot, error) ||
        !requiredStateVector(terminal, "candidate_path_deviation_bounds",
                  config.structure.candidate_path_deviation_bounds, error) ||
        !requiredStateVector(terminal, "recovery_path_deviation_bounds",
                  config.structure.recovery_path_deviation_bounds, error) ||
        !required(dominance, "minimum_relative_tracking_improvement",
                  config.structure.minimum_relative_tracking_improvement,
                  error) ||
        !required(dominance, "minimum_absolute_tracking_improvement",
                  config.structure.minimum_absolute_tracking_improvement,
                  error) ||
        !required(dominance, "objective_margin",
                  config.structure.model_dominance_margin, error) ||
        !requiredStateVector(dominance, "tracking_scales",
                  config.tracking_scales, error) ||
        !required(dominance, "liquid_weight", config.liquid_weight,
                  error) ||
        !required(dominance, "residual_weight", config.residual_weight,
                  error) ||
        !requiredVector(envelopes, "nominal_phase_indices",
                  config.nominal_phases, error) ||
        !required(d1, "id", config.d1_id, error) ||
        !required(d1, "lateral_offset_m", config.d1_lateral_m, error) ||
        !required(d1, "yaw_offset_rad", config.d1_yaw_rad, error) ||
        !requiredVector(d1, "scale_fractions", config.d1_scales, error) ||
        !required(d2, "id", config.d2_id, error) ||
        !required(d2, "begin_artifact_index", config.d2_begin, error) ||
        !required(d2, "cycle_count", config.d2_cycles, error) ||
        !required(d2, "linear_cap_mps", config.d2_cap, error) ||
        !requiredVector(d2, "snapshot_cycles", config.d2_snapshots,
                  error) ||
        !required(search, "solver", config.qp_solver, error) ||
        !required(search, "maximum_iterations",
                  config.qp_maximum_iterations, error) ||
        !required(search, "absolute_tolerance",
                  config.qp_absolute_tolerance, error) ||
        !required(search, "relative_tolerance",
                  config.qp_relative_tolerance, error) ||
        !required(search, "polishing", config.qp_polishing, error) ||
        !required(search, "hessian_regularization",
                  config.qp_hessian_regularization, error) ||
        !required(search, "constraint_audit_tolerance_multiplier",
                  config.qp_constraint_audit_tolerance_multiplier, error) ||
        !required(search, "value_tolerance_multiplier",
                  config.qp_value_tolerance_multiplier, error) ||
        !required(search, "tracking_multiplier_initial",
                  config.qp_tracking_multiplier_initial, error) ||
        !required(search, "tracking_multiplier_growth",
                  config.qp_tracking_multiplier_growth, error) ||
        !required(search, "tracking_multiplier_expansions",
                  config.qp_tracking_multiplier_expansions, error) ||
        !required(search, "tracking_multiplier_bisection_iterations",
                  config.qp_tracking_multiplier_bisection_iterations,
                  error) ||
        !requiredVector(search, "nonlinear_validation_scales",
                  config.nonlinear_validation_scales, error) ||
        !required(decision,
                  "require_all_disturbance_baselines_recoverable",
                  require_baselines, error) ||
        !required(decision, "minimum_useful_fraction_each_disturbance",
                  config.minimum_useful_fraction_each, error) ||
        !required(decision, "require_zero_residual_identity",
                  require_identity, error) ||
        !required(decision, "require_all_linearizations_finite",
                  require_finite, error) ||
        !required(decision, "failure_decision", config.failure_decision,
                  error) ||
        !required(decision, "success_decision", config.success_decision,
                  error)) {
        return false;
    }
    if (config.schema !=
            "spmpc_bt_residual_structural_campaign_v1" ||
        !scope_development || online_solver || plant_admission ||
        robust_claim || !one_shot ||
        config.qp_solver !=
            "osqp_full_stage_residual_qp_v2_convex_joint_certificate" ||
        config.qp_maximum_iterations <= 0 ||
        !std::isfinite(config.qp_absolute_tolerance) ||
        config.qp_absolute_tolerance <= 0.0 ||
        !std::isfinite(config.qp_relative_tolerance) ||
        config.qp_relative_tolerance <= 0.0 ||
        !config.qp_polishing ||
        !std::isfinite(config.qp_hessian_regularization) ||
        config.qp_hessian_regularization <= 0.0 ||
        !std::isfinite(config.qp_constraint_audit_tolerance_multiplier) ||
        config.qp_constraint_audit_tolerance_multiplier < 1.0 ||
        !std::isfinite(config.qp_value_tolerance_multiplier) ||
        config.qp_value_tolerance_multiplier < 1.0 ||
        !std::isfinite(config.qp_tracking_multiplier_initial) ||
        config.qp_tracking_multiplier_initial <= 0.0 ||
        !std::isfinite(config.qp_tracking_multiplier_growth) ||
        config.qp_tracking_multiplier_growth <= 1.0 ||
        config.qp_tracking_multiplier_expansions <= 0 ||
        config.qp_tracking_multiplier_bisection_iterations <= 0 ||
        !require_baselines || !require_identity || !require_finite ||
        config.failure_decision != "NO_GO_ROUTE_B" ||
        config.success_decision != "GO_CR3" ||
        !std::isfinite(config.liquid_weight) ||
        config.liquid_weight < 0.0 ||
        !std::isfinite(config.residual_weight) ||
        config.residual_weight < 0.0 ||
        !std::isfinite(config.minimum_useful_fraction_each) ||
        config.minimum_useful_fraction_each <= 0.0 ||
        config.minimum_useful_fraction_each > 1.0 ||
        std::any_of(config.nonlinear_validation_scales.begin(),
                    config.nonlinear_validation_scales.end(),
                    [](double value) {
                        return !std::isfinite(value) || value <= 0.0 ||
                            value > 1.0;
                    }) ||
        config.d2_cycles <= 0 || config.d2_cap < 0.0) {
        error = "campaign violates the frozen one-shot development scope";
        return false;
    }
    return btr::validateStructuralContract(
        config.structure, config.execution, error);
}

bool parseArguments(int argc, char** argv, Arguments& arguments) {
    for (int index = 1; index < argc; ++index) {
        const std::string option(argv[index]);
        if (index + 1 >= argc) return false;
        if (option == "--config") arguments.config = argv[++index];
        else if (option == "--artifact") arguments.artifact = argv[++index];
        else if (option == "--output-dir") {
            arguments.output_dir = argv[++index];
        } else if (option == "--source-head") {
            arguments.source_head = argv[++index];
        } else {
            return false;
        }
    }
    return !arguments.config.empty() && !arguments.artifact.empty() &&
        !arguments.output_dir.empty() && arguments.source_head.size() == 40u &&
        std::all_of(arguments.source_head.begin(),
                    arguments.source_head.end(), [](char character) {
                        return (character >= '0' && character <= '9') ||
                            (character >= 'a' && character <= 'f');
                    });
}

bool createExclusiveDirectory(const std::string& path,
                              std::string& error) {
    struct stat status;
    if (::stat(path.c_str(), &status) == 0) {
        error = "output directory already exists";
        return false;
    }
    if (::mkdir(path.c_str(), 0755) != 0) {
        error = "cannot create output directory, errno=" +
            std::to_string(errno);
        return false;
    }
    return true;
}

const btr::RecoverableTubeStage* tubeStage(
    const btr::RecoverableTube& tube,
    std::size_t phase_index) {
    if (tube.stages.empty() || phase_index < tube.stages.front().phase_index) {
        return nullptr;
    }
    const std::size_t offset =
        phase_index - tube.stages.front().phase_index;
    return offset < tube.stages.size() &&
            tube.stages[offset].phase_index == phase_index
        ? &tube.stages[offset]
        : nullptr;
}

bool exactIndependentBtOracle(
    const CampaignConfig& config,
    const std::vector<spmpc::PhaseNominalSample>& samples,
    const btr::BtClosedLoopModel& model,
    const btr::ClosedLoopRolloutResult& candidate,
    const btr::AugmentedState15& state,
    std::size_t phase,
    std::size_t steps,
    const std::vector<btr::StagePublicationConstraint>& caps,
    double& maximum_error) {
    const btr::IndependentBtOracleRolloutResult oracle =
        btr::rolloutIndependentBtOracle(
            config.execution, config.slosh, config.structure, samples,
            state, phase, steps, caps);
    if (!candidate.valid || !oracle.valid ||
        candidate.states.size() != oracle.states.size() ||
        candidate.published_commands.size() !=
            oracle.published_commands.size()) {
        return false;
    }
    maximum_error = 0.0;
    for (std::size_t offset = 0; offset < candidate.states.size(); ++offset) {
        maximum_error = std::max(
            maximum_error,
            model.difference(candidate.states[offset], oracle.states[offset])
                .cwiseAbs().maxCoeff());
        if (candidate.states[offset].execution.stage_index !=
                oracle.states[offset].execution.stage_index ||
            candidate.states[offset].execution.valid !=
                oracle.states[offset].execution.valid) {
            return false;
        }
    }
    for (std::size_t offset = 0;
         offset < candidate.published_commands.size(); ++offset) {
        maximum_error = std::max(
            maximum_error,
            std::max(
                std::abs(candidate.published_commands[offset].linear -
                         oracle.published_commands[offset].linear),
                std::abs(candidate.published_commands[offset].angular -
                         oracle.published_commands[offset].angular)));
    }
    return maximum_error <= model.structure().identity_tolerance;
}

CostSummary evaluateCost(const CampaignConfig& config,
                         const btr::BtClosedLoopModel& model,
                         const btr::RecoverableTube& tube,
                         const btr::ClosedLoopRolloutResult& rollout) {
    CostSummary cost;
    if (!rollout.valid) return cost;
    for (std::size_t offset = 0; offset < rollout.states.size(); ++offset) {
        const std::size_t phase = rollout.initial_phase_index + offset;
        const btr::RecoverableTubeStage* center = tubeStage(tube, phase);
        if (center == nullptr) return CostSummary{};
        const btr::StateVector error = model.difference(
            rollout.states[offset], center->center);
        for (int index = 0; index < 6; ++index) {
            const double normalized =
                error[index] / config.tracking_scales[index];
            cost.tracking += normalized * normalized;
        }
        const btr::StateVector absolute = model.pack(rollout.states[offset]);
        for (int index = 6; index < 10; ++index) {
            const double normalized =
                absolute[index] / config.tracking_scales[index];
            cost.liquid += normalized * normalized;
        }
    }
    const double state_count = static_cast<double>(rollout.states.size());
    cost.tracking /= state_count;
    cost.liquid /= state_count;
    for (const btr::ResidualVector& residual : rollout.residuals) {
        const double normalized_v =
            residual[0] / config.structure.maximum_residual_v;
        const double normalized_omega =
            residual[1] / config.structure.maximum_residual_omega;
        cost.residual += normalized_v * normalized_v +
            normalized_omega * normalized_omega;
    }
    if (!rollout.residuals.empty()) {
        cost.residual /= static_cast<double>(rollout.residuals.size());
    }
    cost.objective = cost.tracking +
        config.liquid_weight * cost.liquid +
        config.residual_weight * cost.residual;
    cost.valid = std::isfinite(cost.objective);
    return cost;
}

void appendConstraint(const Eigen::VectorXd& row,
                      double lower,
                      double upper,
                      std::vector<Eigen::VectorXd>& rows,
                      std::vector<double>& lowers,
                      std::vector<double>& uppers) {
    rows.push_back(row);
    lowers.push_back(lower);
    uppers.push_back(upper);
}

QpCandidate solveFullStageResidualQp(
    const CampaignConfig& config,
    const btr::BtClosedLoopModel& model,
    const btr::RecoverableTube& tube,
    const Scenario& scenario,
    const btr::ClosedLoopRolloutResult& baseline) {
    QpCandidate result;
    const int prefix = config.structure.residual_prefix_steps;
    const int horizon = prefix + config.structure.recovery_suffix_steps;
    const int variables = btr::kResidualWidth * prefix;
    result.variable_count = variables;
    if (!baseline.valid ||
        baseline.states.size() != static_cast<std::size_t>(horizon + 1) ||
        baseline.published_commands.size() !=
            static_cast<std::size_t>(horizon)) {
        result.status = "INVALID_QP_BASELINE";
        return result;
    }

    using Sensitivity = Eigen::Matrix<
        double, btr::kStateWidth, Eigen::Dynamic>;
    using CommandSensitivity = Eigen::Matrix<
        double, btr::kResidualWidth, Eigen::Dynamic>;
    std::vector<Sensitivity> state_sensitivity(
        static_cast<std::size_t>(horizon + 1),
        Sensitivity::Zero(btr::kStateWidth, variables));
    std::vector<CommandSensitivity> command_sensitivity(
        static_cast<std::size_t>(horizon),
        CommandSensitivity::Zero(btr::kResidualWidth, variables));
    std::vector<btr::ClosedLoopLinearization> linearizations;
    linearizations.reserve(static_cast<std::size_t>(horizon));
    for (int stage = 0; stage < horizon; ++stage) {
        const btr::StagePublicationConstraint publication =
            scenario.publications.empty()
            ? btr::StagePublicationConstraint{}
            : scenario.publications[static_cast<std::size_t>(stage)];
        btr::ClosedLoopLinearization linearization =
            btr::linearizeClosedLoop(
                model, baseline.states[static_cast<std::size_t>(stage)],
                scenario.phase_index + static_cast<std::size_t>(stage),
                publication);
        if (!linearization.valid) {
            result.status = "SCENARIO_LINEARIZATION_FAILED_STAGE_" +
                std::to_string(stage) + "_" + linearization.status;
            return result;
        }
        command_sensitivity[static_cast<std::size_t>(stage)] =
            linearization.bt_command_state_jacobian *
            state_sensitivity[static_cast<std::size_t>(stage)];
        if (stage < prefix && !publication.linear_cap_active) {
            command_sensitivity[static_cast<std::size_t>(stage)](
                0, 2 * stage) += 1.0;
            command_sensitivity[static_cast<std::size_t>(stage)](
                1, 2 * stage + 1) += 1.0;
        }
        state_sensitivity[static_cast<std::size_t>(stage + 1)] =
            linearization.a *
            state_sensitivity[static_cast<std::size_t>(stage)];
        if (stage < prefix) {
            state_sensitivity[static_cast<std::size_t>(stage + 1)]
                .block(0, 2 * stage, btr::kStateWidth,
                       btr::kResidualWidth) += linearization.b;
        }
        linearizations.push_back(std::move(linearization));
    }

    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variables, variables);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variables);
    Eigen::MatrixXd tracking_hessian =
        Eigen::MatrixXd::Zero(variables, variables);
    Eigen::VectorXd tracking_gradient = Eigen::VectorXd::Zero(variables);
    double baseline_tracking = 0.0;
    double baseline_objective = 0.0;
    const double state_denominator = static_cast<double>(horizon + 1);
    for (int stage = 0; stage <= horizon; ++stage) {
        const std::size_t phase = scenario.phase_index +
            static_cast<std::size_t>(stage);
        const btr::RecoverableTubeStage* center = tubeStage(tube, phase);
        if (center == nullptr) {
            result.status = "QP_TUBE_CENTER_UNAVAILABLE";
            return result;
        }
        const btr::StateVector baseline_error = model.difference(
            baseline.states[static_cast<std::size_t>(stage)],
            center->center);
        const btr::StateVector baseline_absolute = model.pack(
            baseline.states[static_cast<std::size_t>(stage)]);
        for (int state_index = 0; state_index < 10; ++state_index) {
            if (state_index >= 6 && state_index < 10) {
                // Liquid cost is absolute, matching evaluateCost().
            } else if (state_index >= 6) {
                continue;
            }
            const double cost_weight =
                (state_index < 6 ? 1.0 : config.liquid_weight) /
                (config.tracking_scales[state_index] *
                 config.tracking_scales[state_index] * state_denominator);
            const double base = state_index < 6
                ? baseline_error[state_index]
                : baseline_absolute[state_index];
            const Eigen::RowVectorXd sensitivity =
                state_sensitivity[static_cast<std::size_t>(stage)]
                    .row(state_index);
            hessian.noalias() +=
                2.0 * cost_weight *
                sensitivity.transpose() * sensitivity;
            gradient.noalias() +=
                2.0 * cost_weight * base * sensitivity.transpose();
            baseline_objective += cost_weight * base * base;
            if (state_index < 6) {
                tracking_hessian.noalias() +=
                    2.0 * cost_weight *
                    sensitivity.transpose() * sensitivity;
                tracking_gradient.noalias() +=
                    2.0 * cost_weight * base * sensitivity.transpose();
                baseline_tracking += cost_weight * base * base;
            }
        }
    }
    for (int stage = 0; stage < prefix; ++stage) {
        const double residual_denominator = static_cast<double>(horizon);
        hessian(2 * stage, 2 * stage) +=
            2.0 * config.residual_weight /
            (config.structure.maximum_residual_v *
             config.structure.maximum_residual_v * residual_denominator);
        hessian(2 * stage + 1, 2 * stage + 1) +=
            2.0 * config.residual_weight /
            (config.structure.maximum_residual_omega *
             config.structure.maximum_residual_omega *
             residual_denominator);
    }
    const CostSummary audited_baseline_cost = evaluateCost(
        config, model, tube, baseline);
    const double formulation_tolerance = std::max(
        1.0e-10, config.structure.identity_tolerance);
    if (!audited_baseline_cost.valid ||
        std::abs(audited_baseline_cost.tracking - baseline_tracking) >
            formulation_tolerance ||
        std::abs(audited_baseline_cost.objective - baseline_objective) >
            formulation_tolerance) {
        result.status = "QP_COST_FORMULATION_MISMATCH";
        return result;
    }
    std::vector<Eigen::VectorXd> rows;
    std::vector<double> lower;
    std::vector<double> upper;
    for (int stage = 0; stage < prefix; ++stage) {
        const std::size_t phase = scenario.phase_index +
            static_cast<std::size_t>(stage);
        const bool cap_active = !scenario.publications.empty() &&
            scenario.publications[static_cast<std::size_t>(stage)]
                .linear_cap_active;
        const double authority = cap_active
            ? 0.0
            : btr::residualAuthority(config.structure, phase);
        for (int channel = 0; channel < btr::kResidualWidth; ++channel) {
            Eigen::VectorXd row = Eigen::VectorXd::Zero(variables);
            row[2 * stage + channel] = 1.0;
            const double bound = authority * (channel == 0
                ? config.structure.maximum_residual_v
                : config.structure.maximum_residual_omega);
            appendConstraint(row, -bound, bound, rows, lower, upper);

            Eigen::VectorXd slew = row;
            if (stage > 0) slew[2 * (stage - 1) + channel] = -1.0;
            const double slew_bound = channel == 0
                ? config.structure.maximum_residual_slew_v
                : config.structure.maximum_residual_slew_omega;
            appendConstraint(
                slew, -slew_bound, slew_bound, rows, lower, upper);

            Eigen::VectorXd cumulative = Eigen::VectorXd::Zero(variables);
            for (int previous = 0; previous <= stage; ++previous) {
                cumulative[2 * previous + channel] =
                    config.execution.dt;
            }
            const double cumulative_bound = channel == 0
                ? config.structure.cumulative_progress_budget_m
                : config.structure.cumulative_yaw_budget_rad;
            appendConstraint(cumulative, -cumulative_bound,
                             cumulative_bound, rows, lower, upper);
        }
    }
    // The first BT-only suffix knot also enforces residual-to-zero slew.
    for (int channel = 0; channel < btr::kResidualWidth; ++channel) {
        Eigen::VectorXd row = Eigen::VectorXd::Zero(variables);
        row[2 * (prefix - 1) + channel] = 1.0;
        const double bound = channel == 0
            ? config.structure.maximum_residual_slew_v
            : config.structure.maximum_residual_slew_omega;
        appendConstraint(row, -bound, bound, rows, lower, upper);
    }

    for (int stage = 1; stage <= horizon; ++stage) {
        for (int state_index = 0;
             state_index < btr::kStateWidth; ++state_index) {
            appendConstraint(
                state_sensitivity[static_cast<std::size_t>(stage)]
                    .row(state_index).transpose(),
                -config.structure
                    .candidate_path_deviation_bounds[state_index],
                config.structure
                    .candidate_path_deviation_bounds[state_index],
                rows, lower, upper);
        }
    }

    for (int stage = 0; stage < horizon; ++stage) {
        const bool cap_active = !scenario.publications.empty() &&
            scenario.publications[static_cast<std::size_t>(stage)]
                .linear_cap_active;
        const spmpc::VelocityCommand& base =
            baseline.published_commands[static_cast<std::size_t>(stage)];
        const double command_lower[2] = {
            config.execution.linear.output_min,
            config.execution.angular.output_min};
        double command_upper[2] = {
            config.execution.linear.output_max,
            config.execution.angular.output_max};
        if (cap_active) {
            command_upper[0] = std::min(
                command_upper[0],
                scenario.publications[static_cast<std::size_t>(stage)]
                    .maximum_linear);
        }
        const double base_values[2] = {base.linear, base.angular};
        for (int channel = 0; channel < 2; ++channel) {
            appendConstraint(
                command_sensitivity[static_cast<std::size_t>(stage)]
                    .row(channel).transpose(),
                command_lower[channel] - base_values[channel],
                command_upper[channel] - base_values[channel],
                rows, lower, upper);
        }
        if (!cap_active) {
            const spmpc::VelocityCommand previous = stage == 0
                ? spmpc::VelocityCommand{
                      scenario.state.execution.linear.pending_commands.back(),
                      scenario.state.execution.angular.pending_commands.back()}
                : baseline.published_commands[
                      static_cast<std::size_t>(stage - 1)];
            const double previous_values[2] = {
                previous.linear, previous.angular};
            const double rate_bounds[2] = {
                config.structure.maximum_published_acceleration *
                    config.execution.dt,
                config.structure.maximum_published_angular_acceleration *
                    config.execution.dt};
            for (int channel = 0; channel < 2; ++channel) {
                Eigen::VectorXd rate =
                    command_sensitivity[static_cast<std::size_t>(stage)]
                        .row(channel).transpose();
                if (stage > 0) {
                    rate -= command_sensitivity[
                        static_cast<std::size_t>(stage - 1)]
                            .row(channel).transpose();
                }
                const double base_delta =
                    base_values[channel] - previous_values[channel];
                appendConstraint(rate,
                    -rate_bounds[channel] - base_delta,
                    rate_bounds[channel] - base_delta,
                    rows, lower, upper);
            }
        }
    }

    const std::size_t terminal_phase = scenario.phase_index +
        static_cast<std::size_t>(horizon);
    const btr::RecoverableTubeStage* terminal_stage = tubeStage(
        tube, terminal_phase);
    if (terminal_stage == nullptr) {
        result.status = "QP_TERMINAL_TUBE_UNAVAILABLE";
        return result;
    }
    const btr::StateVector terminal_base_deviation = model.difference(
        baseline.states.back(), terminal_stage->center);
    const Sensitivity& terminal_sensitivity = state_sensitivity.back();
    for (int state_index = 0;
         state_index < btr::kStateWidth; ++state_index) {
        appendConstraint(
            terminal_sensitivity.row(state_index).transpose(),
            -terminal_stage->half_width[state_index] -
                terminal_base_deviation[state_index],
            terminal_stage->half_width[state_index] -
                terminal_base_deviation[state_index],
            rows, lower, upper);
    }
    const btr::StateVector mapped_base =
        terminal_stage->terminal_map * terminal_base_deviation;
    const Sensitivity mapped_sensitivity =
        terminal_stage->terminal_map * terminal_sensitivity;
    for (int state_index = 0;
         state_index < btr::kStateWidth; ++state_index) {
        appendConstraint(
            mapped_sensitivity.row(state_index).transpose(),
            -config.structure.terminal_deviation_bounds[state_index] -
                mapped_base[state_index],
            config.structure.terminal_deviation_bounds[state_index] -
                mapped_base[state_index],
            rows, lower, upper);
    }
    const int liquid_indices[4] = {6, 7, 8, 9};
    for (int liquid_index : liquid_indices) {
        const double bound = liquid_index == 6 || liquid_index == 8
            ? config.structure.terminal_liquid_increment_eta
            : config.structure.terminal_liquid_increment_eta_dot;
        appendConstraint(
            terminal_sensitivity.row(liquid_index).transpose(),
            -bound, bound, rows, lower, upper);
    }

    result.constraint_count = static_cast<int>(rows.size());
    Eigen::MatrixXd constraints(rows.size(), variables);
    Eigen::VectorXd lower_vector(rows.size());
    Eigen::VectorXd upper_vector(rows.size());
    for (std::size_t index = 0; index < rows.size(); ++index) {
        constraints.row(static_cast<Eigen::Index>(index)) =
            rows[index].transpose();
        lower_vector[static_cast<Eigen::Index>(index)] = lower[index];
        upper_vector[static_cast<Eigen::Index>(index)] = upper[index];
    }
    Eigen::SparseMatrix<double> sparse_constraints = constraints.sparseView();
    sparse_constraints.makeCompressed();
    const double residual_squared_norm_bound =
        static_cast<double>(prefix) *
        (config.structure.maximum_residual_v *
             config.structure.maximum_residual_v +
         config.structure.maximum_residual_omega *
             config.structure.maximum_residual_omega);
    auto solveQuadratic = [&](const Eigen::MatrixXd& requested_hessian,
                              const Eigen::VectorXd& requested_gradient,
                              Eigen::VectorXd& solution,
                              double& maximum_violation,
                              double& objective_lower_bound,
                              std::string& solve_status) {
        Eigen::MatrixXd regularized_hessian =
            0.5 * (requested_hessian + requested_hessian.transpose());
        const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(
            regularized_hessian, Eigen::EigenvaluesOnly);
        if (spectrum.info() != Eigen::Success ||
            !spectrum.eigenvalues().array().isFinite().all()) {
            solve_status = "QP_REQUESTED_HESSIAN_SPECTRUM_FAILED";
            return false;
        }
        const double eigenvalue_guard =
            1024.0 * std::numeric_limits<double>::epsilon() *
            (1.0 + regularized_hessian.norm());
        const double added_regularization =
            config.qp_hessian_regularization + eigenvalue_guard +
            std::max(0.0, -spectrum.eigenvalues().minCoeff());
        regularized_hessian.diagonal().array() += added_regularization;
        Eigen::SparseMatrix<double> sparse_hessian =
            regularized_hessian.sparseView();
        Eigen::VectorXd solver_gradient = requested_gradient;
        sparse_hessian.makeCompressed();
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.settings()->setMaxIteration(config.qp_maximum_iterations);
        solver.settings()->setAbsoluteTolerance(config.qp_absolute_tolerance);
        solver.settings()->setRelativeTolerance(config.qp_relative_tolerance);
        solver.settings()->setPolish(config.qp_polishing);
        solver.data()->setNumberOfVariables(variables);
        solver.data()->setNumberOfConstraints(result.constraint_count);
        if (!solver.data()->setHessianMatrix(sparse_hessian) ||
            !solver.data()->setGradient(solver_gradient) ||
            !solver.data()->setLinearConstraintsMatrix(sparse_constraints) ||
            !solver.data()->setLowerBound(lower_vector) ||
            !solver.data()->setUpperBound(upper_vector) ||
            !solver.initSolver()) {
            solve_status = "OSQP_INITIALIZATION_FAILED";
            return false;
        }
        const OsqpEigen::ErrorExitFlag exit = solver.solveProblem();
        const OsqpEigen::Status status = solver.getStatus();
        if (exit != OsqpEigen::ErrorExitFlag::NoError ||
            (status != OsqpEigen::Status::Solved &&
             status != OsqpEigen::Status::SolvedInaccurate)) {
            const Eigen::VectorXd zero_values =
                constraints * Eigen::VectorXd::Zero(variables);
            double zero_violation = 0.0;
            Eigen::Index zero_violation_row = -1;
            for (Eigen::Index index = 0; index < zero_values.size(); ++index) {
                const double violation = std::max(
                    lower_vector[index] - zero_values[index],
                    zero_values[index] - upper_vector[index]);
                if (violation > zero_violation) {
                    zero_violation = violation;
                    zero_violation_row = index;
                }
            }
            solve_status = "OSQP_NOT_SOLVED_STATUS_" +
                std::to_string(static_cast<int>(status)) +
                "_ZERO_VIOLATION_" + std::to_string(zero_violation) +
                "_ROW_" + std::to_string(zero_violation_row);
            return false;
        }
        solution = solver.getSolution();
        if (!solution.array().isFinite().all()) {
            solve_status = "OSQP_NONFINITE_SOLUTION";
            return false;
        }
        const Eigen::VectorXd dual = solver.getDualSolution().cast<double>();
        if (dual.size() != result.constraint_count ||
            !dual.array().isFinite().all()) {
            solve_status = "OSQP_NONFINITE_DUAL_SOLUTION";
            return false;
        }
        const Eigen::VectorXd values = constraints * solution;
        maximum_violation = 0.0;
        for (Eigen::Index index = 0; index < values.size(); ++index) {
            maximum_violation = std::max(
                maximum_violation,
                std::max(lower_vector[index] - values[index],
                         values[index] - upper_vector[index]));
        }
        if (maximum_violation >
            config.qp_constraint_audit_tolerance_multiplier *
                std::max(config.qp_absolute_tolerance,
                         config.qp_relative_tolerance)) {
            solve_status = "OSQP_SOLUTION_CONSTRAINT_AUDIT_FAILED";
            return false;
        }
        // Any finite bound multiplier y gives a valid lower bound for the
        // regularized convex QP, even when the returned iterate is not
        // exactly stationary.  Remove a worst-case regularization term using
        // the frozen pointwise residual bounds so the result also lower-bounds
        // the unregularized requested objective.
        const Eigen::LDLT<Eigen::MatrixXd> factorization(
            regularized_hessian);
        if (factorization.info() != Eigen::Success ||
            (factorization.vectorD().array() <= 0.0).any()) {
            solve_status = "QP_DUAL_LOWER_BOUND_FACTORIZATION_FAILED";
            return false;
        }
        const double unit_roundoff =
            std::numeric_limits<double>::epsilon();
        const double stationarity_dot_count =
            static_cast<double>(constraints.rows() + 2);
        const double stationarity_gamma =
            8.0 * stationarity_dot_count * unit_roundoff /
            (1.0 - stationarity_dot_count * unit_roundoff);
        const Eigen::VectorXd stationarity_vector =
            solver_gradient + constraints.transpose() * dual;
        const Eigen::VectorXd stationarity_roundoff_bound =
            stationarity_gamma *
            (solver_gradient.cwiseAbs() +
             constraints.cwiseAbs().transpose() * dual.cwiseAbs());
        const Eigen::VectorXd inverse_stationarity =
            factorization.solve(stationarity_vector);
        if (factorization.info() != Eigen::Success ||
            !inverse_stationarity.array().isFinite().all()) {
            solve_status = "QP_DUAL_LOWER_BOUND_SOLVE_FAILED";
            return false;
        }
        const Eigen::VectorXd solve_residual = stationarity_vector -
            regularized_hessian * inverse_stationarity;
        const double dot_count =
            static_cast<double>(regularized_hessian.cols() + 2);
        const double gamma =
            8.0 * dot_count * unit_roundoff /
            (1.0 - dot_count * unit_roundoff);
        const Eigen::VectorXd residual_roundoff_bound = gamma *
            (stationarity_vector.cwiseAbs() +
             regularized_hessian.cwiseAbs() *
                 inverse_stationarity.cwiseAbs());
        const double residual_norm_upper_bound =
            solve_residual.norm() + residual_roundoff_bound.norm() +
            stationarity_roundoff_bound.norm();
        const double certified_lambda_minimum =
            0.5 * config.qp_hessian_regularization;
        if (!std::isfinite(residual_norm_upper_bound) ||
            !std::isfinite(certified_lambda_minimum) ||
            certified_lambda_minimum <= 0.0) {
            solve_status = "QP_DUAL_RESIDUAL_BOUND_FAILED";
            return false;
        }
        objective_lower_bound =
            0.5 * inverse_stationarity.dot(
                      regularized_hessian * inverse_stationarity) -
            stationarity_vector.dot(inverse_stationarity) -
            0.5 * residual_norm_upper_bound *
                residual_norm_upper_bound /
                certified_lambda_minimum;
        double dual_constant_magnitude = 0.0;
        for (Eigen::Index index = 0; index < dual.size(); ++index) {
            const double term = dual[index] >= 0.0
                ? upper_vector[index] * dual[index]
                : lower_vector[index] * dual[index];
            objective_lower_bound -= term;
            dual_constant_magnitude += std::abs(term);
        }
        const double dual_sum_count =
            static_cast<double>(dual.size() + 2);
        const double dual_sum_gamma =
            8.0 * dual_sum_count * unit_roundoff /
            (1.0 - dual_sum_count * unit_roundoff);
        objective_lower_bound -=
            dual_sum_gamma * dual_constant_magnitude;
        objective_lower_bound -= 0.5 *
            added_regularization *
            residual_squared_norm_bound;
        const double rounding_guard =
            256.0 * std::numeric_limits<double>::epsilon() *
            (1.0 + std::abs(objective_lower_bound) +
             stationarity_vector.norm() *
                 inverse_stationarity.norm() +
             regularized_hessian.norm() *
                 inverse_stationarity.squaredNorm() +
             dual_constant_magnitude +
             added_regularization * residual_squared_norm_bound);
        objective_lower_bound -= rounding_guard;
        if (!std::isfinite(objective_lower_bound)) {
            solve_status = "QP_NONFINITE_DUAL_LOWER_BOUND";
            return false;
        }
        solve_status = "OK";
        return true;
    };
    auto trackingValue = [&](const Eigen::VectorXd& residual) {
        return baseline_tracking + tracking_gradient.dot(residual) +
            0.5 * residual.dot(tracking_hessian * residual);
    };
    auto objectiveValue = [&](const Eigen::VectorXd& residual) {
        return baseline_objective + gradient.dot(residual) +
            0.5 * residual.dot(hessian * residual);
    };

    result.tracking_target = std::min(
        baseline_tracking -
            config.structure.minimum_absolute_tracking_improvement,
        baseline_tracking *
            (1.0 -
             config.structure.minimum_relative_tracking_improvement));
    result.objective_target = baseline_objective -
        config.structure.model_dominance_margin;
    if (!std::isfinite(result.tracking_target) ||
        !std::isfinite(result.objective_target) ||
        result.tracking_target < 0.0 || result.objective_target < 0.0) {
        result.solved = true;
        result.certificate_valid = true;
        result.certificate_lower_bound = 0.0;
        result.certificate_threshold = std::min(
            result.tracking_target, result.objective_target);
        result.status = "FROZEN_IMPROVEMENT_TARGET_BELOW_ZERO";
        return result;
    }

    std::string solve_status;
    Eigen::VectorXd objective_solution;
    double objective_violation = 0.0;
    double objective_variable_lower_bound = 0.0;
    if (!solveQuadratic(hessian, gradient, objective_solution,
                        objective_violation,
                        objective_variable_lower_bound, solve_status)) {
        result.status = solve_status;
        return result;
    }
    result.solved = true;
    Eigen::VectorXd solution = objective_solution;
    result.maximum_constraint_violation = objective_violation;
    result.predicted_tracking = trackingValue(solution);
    result.predicted_objective = objectiveValue(solution);
    double separating_multiplier = 0.0;
    double combined_lower_bound =
        baseline_objective + objective_variable_lower_bound;

    const double value_tolerance = config.qp_value_tolerance_multiplier *
        std::max(config.qp_absolute_tolerance,
                 config.qp_relative_tolerance);
    if (result.predicted_tracking >
        result.tracking_target + value_tolerance) {
        Eigen::VectorXd tracking_solution;
        double tracking_violation = 0.0;
        double tracking_variable_lower_bound = 0.0;
        if (!solveQuadratic(tracking_hessian, tracking_gradient,
                            tracking_solution, tracking_violation,
                            tracking_variable_lower_bound, solve_status)) {
            result.status = "TRACKING_QP_" + solve_status;
            return result;
        }
        const double minimum_tracking = trackingValue(tracking_solution);
        const double tracking_lower_bound =
            baseline_tracking + tracking_variable_lower_bound;
        if (minimum_tracking > result.tracking_target + value_tolerance) {
            result.predicted_tracking = minimum_tracking;
            result.predicted_objective = objectiveValue(tracking_solution);
            result.maximum_constraint_violation = tracking_violation;
            result.certificate_lower_bound = tracking_lower_bound;
            result.certificate_threshold = result.tracking_target;
            result.certificate_valid = tracking_lower_bound >
                result.tracking_target + value_tolerance;
            result.status = result.certificate_valid
                ? "CONVEX_TRACKING_IMPROVEMENT_THRESHOLD_INFEASIBLE"
                : "TRACKING_QP_DUAL_CERTIFICATE_GAP";
            return result;
        }

        double lower_multiplier = 0.0;
        double upper_multiplier = config.qp_tracking_multiplier_initial;
        Eigen::VectorXd upper_solution;
        double upper_violation = 0.0;
        double upper_variable_lower_bound = 0.0;
        bool bracketed = false;
        for (int expansion = 0;
             expansion < config.qp_tracking_multiplier_expansions;
             ++expansion) {
            if (!solveQuadratic(
                    hessian + upper_multiplier * tracking_hessian,
                    gradient + upper_multiplier * tracking_gradient,
                    upper_solution, upper_violation,
                    upper_variable_lower_bound, solve_status)) {
                result.status = "MULTIPLIER_QP_" + solve_status;
                return result;
            }
            if (trackingValue(upper_solution) <=
                result.tracking_target + value_tolerance) {
                bracketed = true;
                break;
            }
            lower_multiplier = upper_multiplier;
            upper_multiplier *= config.qp_tracking_multiplier_growth;
        }
        if (!bracketed) {
            result.status = "TRACKING_MULTIPLIER_BRACKET_FAILED";
            return result;
        }
        for (int iteration = 0;
             iteration <
                 config.qp_tracking_multiplier_bisection_iterations;
             ++iteration) {
            const double middle =
                0.5 * (lower_multiplier + upper_multiplier);
            Eigen::VectorXd middle_solution;
            double middle_violation = 0.0;
            double middle_variable_lower_bound = 0.0;
            if (!solveQuadratic(
                    hessian + middle * tracking_hessian,
                    gradient + middle * tracking_gradient,
                    middle_solution, middle_violation,
                    middle_variable_lower_bound, solve_status)) {
                result.status = "BISECTION_QP_" + solve_status;
                return result;
            }
            if (trackingValue(middle_solution) <=
                result.tracking_target + value_tolerance) {
                upper_multiplier = middle;
                upper_solution = std::move(middle_solution);
                upper_violation = middle_violation;
                upper_variable_lower_bound =
                    middle_variable_lower_bound;
            } else {
                lower_multiplier = middle;
            }
        }
        solution = upper_solution;
        result.tracking_multiplier = upper_multiplier;
        result.maximum_constraint_violation = upper_violation;
        result.predicted_tracking = trackingValue(solution);
        result.predicted_objective = objectiveValue(solution);
        separating_multiplier = upper_multiplier;
        combined_lower_bound = baseline_objective +
            upper_multiplier * baseline_tracking +
            upper_variable_lower_bound;
    }
    if (result.predicted_tracking >
            result.tracking_target + value_tolerance ||
        result.predicted_objective >
            result.objective_target + value_tolerance) {
        result.certificate_lower_bound = combined_lower_bound;
        result.certificate_threshold = result.objective_target +
            separating_multiplier * result.tracking_target;
        result.certificate_valid = result.certificate_lower_bound >
            result.certificate_threshold + value_tolerance;
        result.status = result.certificate_valid
            ? "CONVEX_JOINT_USEFUL_RESIDUAL_SET_INFEASIBLE"
            : "JOINT_QP_DUAL_CERTIFICATE_GAP";
        return result;
    }
    result.residuals.assign(
        static_cast<std::size_t>(horizon), btr::ResidualVector::Zero());
    for (int stage = 0; stage < prefix; ++stage) {
        result.residuals[static_cast<std::size_t>(stage)][0] =
            solution[2 * stage];
        result.residuals[static_cast<std::size_t>(stage)][1] =
            solution[2 * stage + 1];
    }
    result.valid = true;
    result.solution_audited = true;
    result.status = "OSQP_FULL_CONVEX_USEFUL_SET_SOLVED_AND_AUDITED";
    return result;
}

bool liquidIncrementPassed(
    const CampaignConfig& config,
    const btr::AugmentedState15& candidate,
    const btr::AugmentedState15& baseline,
    double& eta,
    double& eta_dot) {
    eta = std::max(
        std::abs(candidate.execution.slosh.eta_x -
                 baseline.execution.slosh.eta_x),
        std::abs(candidate.execution.slosh.eta_y -
                 baseline.execution.slosh.eta_y));
    eta_dot = std::max(
        std::abs(candidate.execution.slosh.eta_x_dot -
                 baseline.execution.slosh.eta_x_dot),
        std::abs(candidate.execution.slosh.eta_y_dot -
                 baseline.execution.slosh.eta_y_dot));
    return eta <= config.structure.terminal_liquid_increment_eta &&
        eta_dot <= config.structure.terminal_liquid_increment_eta_dot;
}

bool absoluteLiquidPathPassed(
    const CampaignConfig& config,
    const btr::ClosedLoopRolloutResult& rollout) {
    if (!rollout.valid) return false;
    for (const btr::AugmentedState15& state : rollout.states) {
        const double eta = std::max(
            std::abs(state.execution.slosh.eta_x),
            std::abs(state.execution.slosh.eta_y));
        const double eta_dot = std::max(
            std::abs(state.execution.slosh.eta_x_dot),
            std::abs(state.execution.slosh.eta_y_dot));
        if (eta > config.structure.maximum_absolute_eta ||
            eta_dot > config.structure.maximum_absolute_eta_dot) {
            return false;
        }
    }
    return true;
}

bool candidatePathDeviationPassed(
    const CampaignConfig& config,
    const btr::BtClosedLoopModel& model,
    const btr::ClosedLoopRolloutResult& candidate,
    const btr::ClosedLoopRolloutResult& baseline,
    double& maximum_fraction) {
    maximum_fraction = 0.0;
    if (!candidate.valid || !baseline.valid ||
        candidate.states.size() != baseline.states.size()) {
        return false;
    }
    for (std::size_t stage = 0; stage < candidate.states.size(); ++stage) {
        const btr::StateVector deviation = model.difference(
            candidate.states[stage], baseline.states[stage]);
        for (int state_index = 0;
             state_index < btr::kStateWidth; ++state_index) {
            const double bound = config.structure
                .candidate_path_deviation_bounds[state_index];
            maximum_fraction = std::max(
                maximum_fraction,
                std::abs(deviation[state_index]) / bound);
            if (std::abs(deviation[state_index]) >
                bound + config.structure.identity_tolerance) {
                return false;
            }
        }
    }
    return std::isfinite(maximum_fraction);
}

ScenarioResult evaluateScenario(
    const CampaignConfig& config,
    const btr::BtClosedLoopModel& model,
    const btr::RecoverableTube& tube,
    const std::vector<spmpc::PhaseNominalSample>& samples,
    const Scenario& scenario) {
    ScenarioResult result;
    result.id = scenario.id;
    result.disturbance = scenario.disturbance;
    result.phase_index = scenario.phase_index;
    const std::size_t horizon = static_cast<std::size_t>(
        config.structure.residual_prefix_steps +
        config.structure.recovery_suffix_steps);
    const std::vector<btr::ResidualVector> zeros(
        horizon, btr::ResidualVector::Zero());
    const btr::ClosedLoopRolloutResult baseline = model.rollout(
        scenario.state, scenario.phase_index, zeros,
        scenario.publications);
    if (!baseline.valid) {
        result.status = "BASELINE_ROLLOUT_FAILED_" + baseline.status;
        return result;
    }
    result.zero_residual_oracle_passed = exactIndependentBtOracle(
        config, samples, model, baseline, scenario.state,
        scenario.phase_index, horizon, scenario.publications,
        result.zero_residual_maximum_error);
    if (!result.zero_residual_oracle_passed) {
        result.status = "ZERO_RESIDUAL_INDEPENDENT_ORACLE_FAILED";
        return result;
    }
    result.baseline_valid = true;
    if (!absoluteLiquidPathPassed(config, baseline)) {
        result.evidence_valid = true;
        result.status = "BT_COUNTERFACTUAL_LIQUID_PATH_FAILED";
        return result;
    }
    const CostSummary baseline_cost = evaluateCost(
        config, model, tube, baseline);
    if (!baseline_cost.valid) {
        result.status = "BASELINE_COST_FAILED";
        return result;
    }
    const std::size_t terminal_phase = scenario.phase_index + horizon;
    const btr::TubeMembershipResult baseline_membership =
        btr::evaluateTubeMembership(
            model, tube, baseline.states.back(), terminal_phase);
    const btr::TerminalRecoveryResult baseline_recovery =
        btr::auditNonlinearBtRecovery(
            model, baseline.states.back(), terminal_phase, tube);
    result.baseline_tracking = baseline_cost.tracking;
    result.baseline_objective = baseline_cost.objective;
    result.best_tracking = baseline_cost.tracking;
    result.best_objective = baseline_cost.objective;
    result.baseline_inside_tube = baseline_membership.valid &&
        baseline_membership.inside;
    result.baseline_tube_margin = baseline_membership.minimum_margin;
    result.baseline_recovered = baseline_recovery.valid &&
        baseline_recovery.recovered;
    result.baseline_nonlinear_recovered = baseline_recovery.valid &&
        baseline_recovery.nonlinear_recovered;
    if (!baseline_membership.valid || !baseline_recovery.valid) {
        result.status = "INVALID_BASELINE_TUBE_OR_RECOVERY_AUDIT";
        return result;
    }
    if (!result.baseline_nonlinear_recovered) {
        result.evidence_valid = true;
        result.status = "BT_COUNTERFACTUAL_NOT_RECOVERABLE_" +
            baseline_recovery.status;
        return result;
    }
    if (!result.baseline_inside_tube || !result.baseline_recovered) {
        result.status = "INVALID_TUBE_COVERAGE_FOR_NONLINEAR_BT_BASELINE_" +
            baseline_recovery.status;
        return result;
    }
    if (!scenario.require_useful_candidate) {
        result.evidence_valid = true;
        result.status = "NOMINAL_ZERO_RESIDUAL_FEASIBLE";
        return result;
    }

    const QpCandidate qp = solveFullStageResidualQp(
        config, model, tube, scenario, baseline);
    result.qp_solved = qp.solved;
    result.qp_variable_count = qp.variable_count;
    result.qp_constraint_count = qp.constraint_count;
    result.qp_maximum_constraint_violation =
        qp.maximum_constraint_violation;
    result.qp_predicted_tracking = qp.predicted_tracking;
    result.qp_predicted_objective = qp.predicted_objective;
    result.qp_tracking_target = qp.tracking_target;
    result.qp_objective_target = qp.objective_target;
    result.qp_tracking_multiplier = qp.tracking_multiplier;
    result.qp_certificate_valid = qp.certificate_valid;
    result.qp_solution_audited = qp.solution_audited;
    result.qp_certificate_lower_bound = qp.certificate_lower_bound;
    result.qp_certificate_threshold = qp.certificate_threshold;
    result.qp_status = qp.status;
    if (!qp.valid) {
        result.evidence_valid = qp.certificate_valid;
        result.status = "FULL_STAGE_RESIDUAL_QP_FAILED_" + qp.status;
        return result;
    }

    bool found = false;
    bool invalid_candidate_audit = false;
    for (double scale : config.nonlinear_validation_scales) {
        ++result.candidates_tested;
        std::vector<btr::ResidualVector> residuals = qp.residuals;
        for (btr::ResidualVector& residual : residuals) {
            residual *= scale;
        }
        const btr::ClosedLoopRolloutResult candidate = model.rollout(
            scenario.state, scenario.phase_index, residuals,
            scenario.publications);
        if (!candidate.valid) continue;
        ++result.candidates_dynamically_valid;
        if (!absoluteLiquidPathPassed(config, candidate)) continue;
        double path_fraction = 0.0;
        if (!candidatePathDeviationPassed(
                config, model, candidate, baseline, path_fraction)) {
            continue;
        }
        double increment_eta = 0.0;
        double increment_eta_dot = 0.0;
        if (!liquidIncrementPassed(
                config, candidate.states.back(), baseline.states.back(),
                increment_eta, increment_eta_dot)) {
            continue;
        }
        const btr::TubeMembershipResult membership =
            btr::evaluateTubeMembership(
                model, tube, candidate.states.back(), terminal_phase);
        if (!membership.valid) {
            invalid_candidate_audit = true;
            continue;
        }
        if (!membership.inside) continue;
        const btr::TerminalRecoveryResult recovery =
            btr::auditNonlinearBtRecovery(
                model, candidate.states.back(), terminal_phase, tube);
        if (!recovery.valid) {
            invalid_candidate_audit = true;
            continue;
        }
        if (!recovery.recovered) continue;
        ++result.candidates_terminal_feasible;
        const CostSummary cost = evaluateCost(
            config, model, tube, candidate);
        if (!cost.valid) {
            invalid_candidate_audit = true;
            continue;
        }
        const double absolute_improvement =
            baseline_cost.tracking - cost.tracking;
        const double relative_improvement =
            baseline_cost.tracking > 1.0e-15
            ? absolute_improvement / baseline_cost.tracking
            : 0.0;
        const bool dominance =
            cost.objective + config.structure.model_dominance_margin <=
            baseline_cost.objective;
        double maximum_residual_v = 0.0;
        double maximum_residual_omega = 0.0;
        for (const btr::ResidualVector& residual : residuals) {
            maximum_residual_v = std::max(
                maximum_residual_v, std::abs(residual[0]));
            maximum_residual_omega = std::max(
                maximum_residual_omega, std::abs(residual[1]));
        }
        const bool nonzero = std::max(
            maximum_residual_v, maximum_residual_omega) >=
            config.structure.minimum_nonzero_residual;
        const bool useful = nonzero && dominance &&
            absolute_improvement >= config.structure
                .minimum_absolute_tracking_improvement &&
            relative_improvement >= config.structure
                .minimum_relative_tracking_improvement;
        if (!useful ||
            (found && cost.tracking >= result.best_tracking)) {
            continue;
        }
        found = true;
        result.best_tracking = cost.tracking;
        result.best_objective = cost.objective;
        result.absolute_tracking_improvement = absolute_improvement;
        result.relative_tracking_improvement = relative_improvement;
        result.maximum_residual_v = maximum_residual_v;
        result.maximum_residual_omega = maximum_residual_omega;
        result.terminal_liquid_increment_eta = increment_eta;
        result.terminal_liquid_increment_eta_dot = increment_eta_dot;
        result.candidate_tube_margin = membership.minimum_margin;
        result.maximum_candidate_path_fraction = path_fraction;
        result.best_validation_scale = scale;
        result.accepted_residuals = residuals;
    }
    if (invalid_candidate_audit) {
        result.status = "INVALID_NONLINEAR_CANDIDATE_AUDIT";
        return result;
    }
    result.evidence_valid = true;
    result.useful_candidate = found;
    result.status = result.useful_candidate
        ? "USEFUL_NONZERO_RESIDUAL_FOUND"
        : "NO_USEFUL_NONZERO_RESIDUAL_FOUND_BY_FROZEN_QP_AND_SCALES";
    return result;
}

std::vector<Scenario> makeScenarios(
    const CampaignConfig& config,
    const btr::BtClosedLoopModel& model,
    const btr::RecoverableTube& tube,
    const std::vector<spmpc::PhaseNominalSample>& samples,
    std::string& error) {
    std::vector<Scenario> scenarios;
    const std::size_t horizon = static_cast<std::size_t>(
        config.structure.residual_prefix_steps +
        config.structure.recovery_suffix_steps);
    for (std::size_t phase : config.nominal_phases) {
        const btr::RecoverableTubeStage* stage = tubeStage(tube, phase);
        if (stage == nullptr || phase + horizon >= tube.stages.size()) {
            error = "nominal scenario phase is outside tube";
            return {};
        }
        Scenario scenario;
        scenario.id = "nominal_phase_" + std::to_string(phase);
        scenario.disturbance = "NOMINAL";
        scenario.phase_index = phase;
        scenario.state = stage->center;
        scenario.require_useful_candidate = false;
        scenarios.push_back(scenario);
    }
    const btr::RecoverableTubeStage* initial = tubeStage(tube, 0);
    if (initial == nullptr) {
        error = "D1 initial tube state unavailable";
        return {};
    }
    for (double scale : config.d1_scales) {
        Scenario scenario;
        scenario.id = "d1_scale_" + jsonNumber(scale);
        scenario.disturbance = config.d1_id;
        scenario.phase_index = 0;
        scenario.state = initial->center;
        const double yaw = scenario.state.execution.robot.yaw;
        scenario.state.execution.robot.x +=
            -std::sin(yaw) * config.d1_lateral_m * scale;
        scenario.state.execution.robot.y +=
            std::cos(yaw) * config.d1_lateral_m * scale;
        scenario.state.execution.robot.yaw = std::atan2(
            std::sin(yaw + config.d1_yaw_rad * scale),
            std::cos(yaw + config.d1_yaw_rad * scale));
        scenario.require_useful_candidate = true;
        scenarios.push_back(scenario);
    }

    const btr::RecoverableTubeStage* d2_initial = tubeStage(
        tube, config.d2_begin);
    if (d2_initial == nullptr) {
        error = "D2 initial tube state unavailable";
        return {};
    }
    const std::vector<btr::ResidualVector> d2_zeros(
        static_cast<std::size_t>(config.d2_cycles),
        btr::ResidualVector::Zero());
    std::vector<btr::StagePublicationConstraint> d2_caps(
        d2_zeros.size());
    for (btr::StagePublicationConstraint& cap : d2_caps) {
        cap.linear_cap_active = true;
        cap.maximum_linear = config.d2_cap;
    }
    const btr::ClosedLoopRolloutResult d2_rollout = model.rollout(
        d2_initial->center, config.d2_begin, d2_zeros, d2_caps);
    double d2_identity_error = 0.0;
    if (!d2_rollout.valid || !exactIndependentBtOracle(
            config, samples, model, d2_rollout, d2_initial->center,
            config.d2_begin, d2_zeros.size(), d2_caps,
            d2_identity_error)) {
        error = "D2 envelope independent publication oracle failed";
        return {};
    }
    for (int snapshot : config.d2_snapshots) {
        if (snapshot <= 0 || snapshot > config.d2_cycles ||
            static_cast<std::size_t>(snapshot) >=
                d2_rollout.states.size()) {
            error = "invalid D2 snapshot cycle";
            return {};
        }
        Scenario scenario;
        scenario.id = "d2_after_" + std::to_string(snapshot) + "_cycles";
        scenario.disturbance = config.d2_id;
        scenario.phase_index = config.d2_begin +
            static_cast<std::size_t>(snapshot);
        scenario.state =
            d2_rollout.states[static_cast<std::size_t>(snapshot)];
        scenario.require_useful_candidate = true;
        scenario.publications.resize(horizon);
        const int remaining = config.d2_cycles - snapshot;
        for (int offset = 0; offset < remaining; ++offset) {
            scenario.publications[static_cast<std::size_t>(offset)]
                .linear_cap_active = true;
            scenario.publications[static_cast<std::size_t>(offset)]
                .maximum_linear = config.d2_cap;
        }
        scenarios.push_back(scenario);
    }
    return scenarios;
}

bool writeTubeCsv(const std::string& path,
                  const btr::BtClosedLoopModel& model,
                  const btr::RecoverableTube& tube) {
    std::ofstream output(path);
    if (!output.is_open()) return false;
    output.imbue(std::locale::classic());
    output << std::setprecision(17);
    output << "phase_index";
    for (int index = 0; index < btr::kStateWidth; ++index) {
        output << ",center_" << index;
    }
    for (int index = 0; index < btr::kStateWidth; ++index) {
        output << ",half_width_" << index;
    }
    for (int row = 0; row < btr::kStateWidth; ++row) {
        for (int column = 0; column < btr::kStateWidth; ++column) {
            output << ",terminal_map_" << row << '_' << column;
        }
    }
    output << '\n';
    for (const btr::RecoverableTubeStage& stage : tube.stages) {
        output << stage.phase_index;
        const btr::StateVector center = model.pack(stage.center);
        for (int index = 0; index < btr::kStateWidth; ++index) {
            output << ',' << center[index];
        }
        for (int index = 0; index < btr::kStateWidth; ++index) {
            output << ',' << stage.half_width[index];
        }
        for (int row = 0; row < btr::kStateWidth; ++row) {
            for (int column = 0; column < btr::kStateWidth; ++column) {
                output << ',' << stage.terminal_map(row, column);
            }
        }
        output << '\n';
    }
    return output.good();
}

bool writeLinearizationCsv(const std::string& path,
                           const btr::RecoverableTube& tube) {
    std::ofstream output(path);
    if (!output.is_open()) return false;
    output.imbue(std::locale::classic());
    output << std::setprecision(17);
    output << "phase_index,max_reconstruction_error,"
              "max_directional_asymmetry";
    for (int index = 0; index < btr::kStateWidth; ++index) {
        output << ",a_scheme_" << index;
    }
    for (int index = 0; index < btr::kResidualWidth; ++index) {
        output << ",b_scheme_" << index;
    }
    for (int row = 0; row < btr::kStateWidth; ++row) {
        for (int column = 0; column < btr::kStateWidth; ++column) {
            output << ",a_" << row << '_' << column;
        }
    }
    for (int row = 0; row < btr::kStateWidth; ++row) {
        for (int column = 0; column < btr::kResidualWidth; ++column) {
            output << ",b_" << row << '_' << column;
        }
    }
    for (int row = 0; row < btr::kStateWidth; ++row) {
        for (int column = 0; column < btr::kStateWidth; ++column) {
            output << ",a_abs_bound_" << row << '_' << column;
        }
    }
    for (int row = 0; row < btr::kStateWidth; ++row) {
        for (int column = 0; column < btr::kResidualWidth; ++column) {
            output << ",b_abs_bound_" << row << '_' << column;
        }
    }
    output << '\n';
    for (const btr::ClosedLoopLinearization& linearization :
         tube.linearizations) {
        output << linearization.phase_index << ','
               << linearization.maximum_reconstruction_error << ','
               << linearization.maximum_directional_asymmetry;
        for (btr::DifferenceScheme scheme : linearization.a_schemes) {
            output << ',' << static_cast<int>(scheme);
        }
        for (btr::DifferenceScheme scheme : linearization.b_schemes) {
            output << ',' << static_cast<int>(scheme);
        }
        for (int row = 0; row < btr::kStateWidth; ++row) {
            for (int column = 0; column < btr::kStateWidth; ++column) {
                output << ',' << linearization.a(row, column);
            }
        }
        for (int row = 0; row < btr::kStateWidth; ++row) {
            for (int column = 0; column < btr::kResidualWidth; ++column) {
                output << ',' << linearization.b(row, column);
            }
        }
        for (int row = 0; row < btr::kStateWidth; ++row) {
            for (int column = 0; column < btr::kStateWidth; ++column) {
                output << ','
                       << linearization.a_absolute_bound(row, column);
            }
        }
        for (int row = 0; row < btr::kStateWidth; ++row) {
            for (int column = 0; column < btr::kResidualWidth; ++column) {
                output << ','
                       << linearization.b_absolute_bound(row, column);
            }
        }
        output << '\n';
    }
    return output.good();
}

bool writeScenarioReport(const std::string& path,
                         const CampaignConfig& config,
                         const std::vector<ScenarioResult>& results,
                         bool all_evidence_valid,
                         bool all_baselines,
                         double d1_fraction,
                         double d2_fraction,
                         const std::string& decision) {
    std::ofstream output(path);
    if (!output.is_open()) return false;
    output << "{\n"
           << "  \"schema\": \"spmpc_bt_residual_structural_report_v1\",\n"
           << "  \"freeze_id\": \"" << jsonEscape(config.freeze_id)
           << "\",\n"
           << "  \"implementation_id\": \""
           << jsonEscape(config.structure.implementation_id) << "\",\n"
           << "  \"claim_level\": \""
           << jsonEscape(config.structure.claim_level) << "\",\n"
           << "  \"decision_scope\": "
              "\"bt_centered_residual_terminal_mpc_v1_frozen_CR0_CR2_method_only\",\n"
           << "  \"full_32d_nonlinear_residual_set_excluded\": false,\n"
           << "  \"nonlinear_search_complete\": false,\n"
           << "  \"all_evidence_valid\": "
           << (all_evidence_valid ? "true" : "false") << ",\n"
           << "  \"all_baselines_recoverable\": "
           << (all_baselines ? "true" : "false") << ",\n"
           << "  \"d1_useful_fraction\": " << jsonNumber(d1_fraction)
           << ",\n"
           << "  \"d2_useful_fraction\": " << jsonNumber(d2_fraction)
           << ",\n"
           << "  \"minimum_useful_fraction_each\": "
           << jsonNumber(config.minimum_useful_fraction_each) << ",\n"
           << "  \"decision\": \"" << jsonEscape(decision) << "\",\n"
           << "  \"scenarios\": [\n";
    for (std::size_t index = 0; index < results.size(); ++index) {
        const ScenarioResult& result = results[index];
        output << "    {\"id\":\"" << jsonEscape(result.id)
               << "\",\"disturbance\":\""
               << jsonEscape(result.disturbance)
               << "\",\"phase_index\":" << result.phase_index
               << ",\"evidence_valid\":"
               << (result.evidence_valid ? "true" : "false")
               << ",\"zero_residual_oracle_passed\":"
               << (result.zero_residual_oracle_passed ? "true" : "false")
               << ",\"zero_residual_maximum_error\":"
               << jsonNumber(result.zero_residual_maximum_error)
               << ",\"baseline_valid\":"
               << (result.baseline_valid ? "true" : "false")
               << ",\"baseline_inside_tube\":"
               << (result.baseline_inside_tube ? "true" : "false")
               << ",\"baseline_recovered\":"
               << (result.baseline_recovered ? "true" : "false")
               << ",\"baseline_nonlinear_recovered\":"
               << (result.baseline_nonlinear_recovered
                       ? "true"
                       : "false")
               << ",\"baseline_tracking\":"
               << jsonNumber(result.baseline_tracking)
               << ",\"baseline_objective\":"
               << jsonNumber(result.baseline_objective)
               << ",\"baseline_tube_margin\":"
               << jsonNumber(result.baseline_tube_margin)
               << ",\"useful_candidate\":"
               << (result.useful_candidate ? "true" : "false")
               << ",\"best_tracking\":"
               << jsonNumber(result.best_tracking)
               << ",\"best_objective\":"
               << jsonNumber(result.best_objective)
               << ",\"relative_tracking_improvement\":"
               << jsonNumber(result.relative_tracking_improvement)
               << ",\"absolute_tracking_improvement\":"
               << jsonNumber(result.absolute_tracking_improvement)
               << ",\"maximum_residual_v\":"
               << jsonNumber(result.maximum_residual_v)
               << ",\"maximum_residual_omega\":"
               << jsonNumber(result.maximum_residual_omega)
               << ",\"terminal_liquid_increment_eta\":"
               << jsonNumber(result.terminal_liquid_increment_eta)
               << ",\"terminal_liquid_increment_eta_dot\":"
               << jsonNumber(result.terminal_liquid_increment_eta_dot)
               << ",\"candidate_tube_margin\":"
               << jsonNumber(result.candidate_tube_margin)
               << ",\"maximum_candidate_path_fraction\":"
               << jsonNumber(result.maximum_candidate_path_fraction)
               << ",\"candidates_tested\":"
               << result.candidates_tested
               << ",\"candidates_dynamically_valid\":"
               << result.candidates_dynamically_valid
               << ",\"candidates_terminal_feasible\":"
               << result.candidates_terminal_feasible
               << ",\"qp_solved\":"
               << (result.qp_solved ? "true" : "false")
               << ",\"qp_variable_count\":"
               << result.qp_variable_count
               << ",\"qp_constraint_count\":"
               << result.qp_constraint_count
               << ",\"qp_maximum_constraint_violation\":"
               << jsonNumber(result.qp_maximum_constraint_violation)
               << ",\"qp_predicted_tracking\":"
               << jsonNumber(result.qp_predicted_tracking)
               << ",\"qp_predicted_objective\":"
               << jsonNumber(result.qp_predicted_objective)
               << ",\"qp_tracking_target\":"
               << jsonNumber(result.qp_tracking_target)
               << ",\"qp_objective_target\":"
               << jsonNumber(result.qp_objective_target)
               << ",\"qp_tracking_multiplier\":"
               << jsonNumber(result.qp_tracking_multiplier)
               << ",\"qp_certificate_valid\":"
               << (result.qp_certificate_valid ? "true" : "false")
               << ",\"qp_solution_audited\":"
               << (result.qp_solution_audited ? "true" : "false")
               << ",\"qp_certificate_lower_bound\":"
               << jsonNumber(result.qp_certificate_lower_bound)
               << ",\"qp_certificate_threshold\":"
               << jsonNumber(result.qp_certificate_threshold)
               << ",\"qp_status\":\""
               << jsonEscape(result.qp_status) << "\""
               << ",\"best_validation_scale\":"
               << jsonNumber(result.best_validation_scale)
               << ",\"accepted_residuals\":[";
        for (std::size_t residual_index = 0;
             residual_index < result.accepted_residuals.size();
             ++residual_index) {
            if (residual_index > 0) output << ',';
            output << '['
                   << jsonNumber(
                          result.accepted_residuals[residual_index][0])
                   << ','
                   << jsonNumber(
                          result.accepted_residuals[residual_index][1])
                   << ']';
        }
        output << ']'
               << ",\"status\":\"" << jsonEscape(result.status)
               << "\"}" << (index + 1 == results.size() ? "\n" : ",\n");
    }
    output << "  ]\n}\n";
    return output.good();
}

bool writeManifest(const std::string& path,
                   const CampaignConfig& config,
                   const Arguments& args,
                   const std::string& repository_root,
                   const std::string& config_hash,
                   const std::string& artifact_hash,
                   const std::string& builder_hash,
                   const btr::RecoverableTube& tube,
                   const std::vector<FileDigest>& sources,
                   const std::vector<FileDigest>& runtime_dependencies,
                   const std::vector<FileDigest>& outputs) {
    std::ofstream output(path);
    if (!output.is_open()) return false;
    double maximum_linearization_error = 0.0;
    std::size_t one_sided_a = 0;
    std::size_t one_sided_b = 0;
    std::size_t authority_zero_b = 0;
    double maximum_directional_asymmetry = 0.0;
    for (const btr::ClosedLoopLinearization& stage : tube.linearizations) {
        maximum_linearization_error = std::max(
            maximum_linearization_error,
            stage.maximum_reconstruction_error);
        maximum_directional_asymmetry = std::max(
            maximum_directional_asymmetry,
            stage.maximum_directional_asymmetry);
        for (btr::DifferenceScheme scheme : stage.a_schemes) {
            one_sided_a += scheme == btr::DifferenceScheme::Forward ||
                scheme == btr::DifferenceScheme::Backward;
        }
        for (btr::DifferenceScheme scheme : stage.b_schemes) {
            one_sided_b += scheme == btr::DifferenceScheme::Forward ||
                scheme == btr::DifferenceScheme::Backward;
            authority_zero_b +=
                scheme == btr::DifferenceScheme::AuthorityZero;
        }
    }
    output << "{\n"
           << "  \"schema\": \"spmpc_bt_residual_structure_bundle_v1\",\n"
           << "  \"freeze_id\": \"" << jsonEscape(config.freeze_id)
           << "\",\n"
           << "  \"source_head\": \"" << args.source_head << "\",\n"
           << "  \"builder_compiled_source_head\": \""
           << jsonEscape(compiledBuilderSourceHead()) << "\",\n"
           << "  \"core_compiled_source_head\": \""
           << jsonEscape(btr::compiledSourceHead()) << "\",\n"
           << "  \"dependency_compiled_source_heads\": {"
           << "\"model\":\""
           << jsonEscape(spmpc::provenance::modelCompiledSourceHead())
           << "\",\"controller\":\""
           << jsonEscape(spmpc::provenance::controllerCompiledSourceHead())
           << "\",\"phase_rejoin\":\""
           << jsonEscape(spmpc::provenance::phaseRejoinCompiledSourceHead())
           << "\",\"safety\":\""
           << jsonEscape(spmpc::provenance::safetyCompiledSourceHead())
           << "\",\"reference\":\""
           << jsonEscape(spmpc::provenance::referenceCompiledSourceHead())
           << "\",\"runtime\":\""
           << jsonEscape(spmpc::provenance::runtimeCompiledSourceHead())
           << "\",\"estimation\":\""
           << jsonEscape(spmpc::provenance::estimationCompiledSourceHead())
           << "\"},\n"
           << "  \"all_local_spmpc_compiled_source_heads_match\": true,\n"
           << "  \"repository_root\": \""
           << jsonEscape(repository_root) << "\",\n"
           << "  \"tracked_source_tree_clean\": true,\n"
           << "  \"builder_sha256\": \"" << builder_hash << "\",\n"
           << "  \"config\": {\"path\":\"" << jsonEscape(args.config)
           << "\",\"sha256\":\"" << config_hash << "\"},\n"
           << "  \"artifact\": {\"path\":\""
           << jsonEscape(args.artifact) << "\",\"sha256\":\""
           << artifact_hash << "\"},\n"
           << "  \"source_files\": [\n";
    for (std::size_t index = 0; index < sources.size(); ++index) {
        output << "    {\"path\":\""
               << jsonEscape(sources[index].path)
               << "\",\"sha256\":\"" << sources[index].sha256
               << "\"}"
               << (index + 1 == sources.size() ? "\n" : ",\n");
    }
    output << "  ],\n"
           << "  \"runtime_dependencies\": [\n";
    for (std::size_t index = 0;
         index < runtime_dependencies.size(); ++index) {
        output << "    {\"path\":\""
               << jsonEscape(runtime_dependencies[index].path)
               << "\",\"sha256\":\""
               << runtime_dependencies[index].sha256 << "\"}"
               << (index + 1 == runtime_dependencies.size()
                       ? "\n"
                       : ",\n");
    }
    output << "  ],\n"
           << "  \"output_files\": [\n";
    for (std::size_t index = 0; index < outputs.size(); ++index) {
        output << "    {\"path\":\""
               << jsonEscape(outputs[index].path)
               << "\",\"sha256\":\"" << outputs[index].sha256
               << "\"}"
               << (index + 1 == outputs.size() ? "\n" : ",\n");
    }
    output << "  ],\n"
           << "  \"implementation_id\": \""
           << jsonEscape(config.structure.implementation_id) << "\",\n"
           << "  \"claim_level\": \""
           << jsonEscape(config.structure.claim_level) << "\",\n"
           << "  \"state_layout\": "
              "[\"x\",\"y\",\"yaw\",\"v\",\"s\",\"omega\","
              "\"eta_x\",\"eta_x_dot\",\"eta_y\",\"eta_y_dot\","
              "\"linear_pending_0\",\"linear_pending_1\","
              "\"linear_pending_2\",\"linear_pending_3\","
              "\"angular_pending_0\"],\n"
           << "  \"phase_semantics\": \"fixed_artifact_clock_not_optimized\",\n"
           << "  \"progress_semantics\": \"integrated_executed_velocity_not_v_s\",\n"
           << "  \"additive_stage_uncertainty\": \"none_CR0_CR2_deterministic_only\",\n"
           << "  \"tube_construction\": "
              "\"directional_jacobian_maximum_volume_box_predecessor_v1\",\n"
           << "  \"tube_enclosure_scope\": "
              "\"center_directional_heuristic_inner_approximation\",\n"
           << "  \"nonlinear_candidate_search_scope\": "
              "\"global_linearized_convex_certificate_then_frozen_solution_scales_not_global_nonlinear_exclusion\",\n"
           << "  \"full_32d_nonlinear_residual_set_excluded\": false,\n"
           << "  \"convex_certificate_audit\": "
              "\"primal_constraints_plus_regularization_corrected_dual_lower_bound\",\n"
           << "  \"accepted_witness_frozen_in_structural_report\": true,\n"
           << "  \"zero_residual_oracle_scope\": "
              "\"independent_publication_composition_shared_BT_policy_and_execution_model\",\n"
           << "  \"residual_prefix_steps\": "
           << config.structure.residual_prefix_steps << ",\n"
           << "  \"recovery_suffix_steps\": "
           << config.structure.recovery_suffix_steps << ",\n"
           << "  \"tube_phase_count\": " << tube.stages.size() << ",\n"
           << "  \"linearization_phase_count\": "
           << tube.linearizations.size() << ",\n"
           << "  \"maximum_fd_reconstruction_error\": "
           << jsonNumber(maximum_linearization_error) << ",\n"
           << "  \"maximum_directional_asymmetry\": "
           << jsonNumber(maximum_directional_asymmetry) << ",\n"
           << "  \"one_sided_a_columns\": " << one_sided_a << ",\n"
           << "  \"one_sided_b_columns\": " << one_sided_b << ",\n"
           << "  \"authority_zero_b_columns\": "
           << authority_zero_b << ",\n"
           << "  \"old_empirical_gate_used\": false,\n"
           << "  \"tail_commit_used\": false,\n"
           << "  \"independent_plant_truth_used\": false\n"
           << "}\n";
    return output.good();
}

bool writeRouteDecision(const std::string& path,
                        const std::string& decision,
                        const std::vector<std::string>& reasons) {
    std::ofstream output(path);
    if (!output.is_open()) return false;
    output << "{\n"
           << "  \"schema\": \"spmpc_bt_residual_route_decision_v1\",\n"
           << "  \"decision\": \"" << jsonEscape(decision) << "\",\n"
           << "  \"decision_scope\": "
              "\"bt_centered_residual_terminal_mpc_v1_frozen_CR0_CR2_method_only\",\n"
           << "  \"all_tailored_mpc_methods_excluded\": false,\n"
           << "  \"full_32d_nonlinear_residual_set_excluded\": false,\n"
           << "  \"no_go_semantics\": "
              "\"frozen_method_failed_to_produce_an_admissible_witness_not_global_nonexistence\",\n"
           << "  \"online_solver_authorized\": "
           << (decision == "GO_CR3" ? "true" : "false") << ",\n"
           << "  \"retuning_authorized_after_failure\": false,\n"
           << "  \"reasons\": [";
    for (std::size_t index = 0; index < reasons.size(); ++index) {
        if (index > 0) output << ',';
        output << "\"" << jsonEscape(reasons[index]) << "\"";
    }
    output << "]\n}\n";
    return output.good();
}

bool writeInvalidEvidence(const std::string& path,
                          const std::vector<std::string>& reasons) {
    std::ofstream output(path);
    if (!output.is_open()) return false;
    output << "{\n"
           << "  \"schema\": \"spmpc_bt_residual_invalid_evidence_v1\",\n"
           << "  \"status\": \"INVALID_EVIDENCE_CR0_CR2\",\n"
           << "  \"route_decision_emitted\": false,\n"
           << "  \"reasons\": [";
    for (std::size_t index = 0; index < reasons.size(); ++index) {
        if (index > 0) output << ',';
        output << "\"" << jsonEscape(reasons[index]) << "\"";
    }
    output << "]\n}\n";
    return output.good();
}

int runSyntheticQpSelfTest(const std::string& config_path) {
    CampaignConfig config;
    std::string error;
    if (!loadConfig(config_path, config, error) ||
        !matchesCompiledExecutionContract(config)) {
        std::cerr << "SYNTHETIC_QP_SELF_TEST_CONFIG_FAILED: " << error
                  << '\n';
        return 2;
    }
    config.structure.residual_prefix_steps = 4;
    config.structure.recovery_suffix_steps = 8;
    config.structure.authority_taper_begin_index = 24;
    config.structure.authority_zero_index = 32;
    config.structure.maximum_published_acceleration = 2.0;
    config.structure.maximum_published_angular_acceleration = 2.0;
    config.structure.maximum_residual_slew_v = 0.02;
    config.structure.maximum_residual_slew_omega = 0.05;
    config.structure.cumulative_progress_budget_m = 0.02;
    config.structure.cumulative_yaw_budget_rad = 0.02;
    config.structure.candidate_path_deviation_bounds.setConstant(0.5);
    config.structure.recovery_path_deviation_bounds.setConstant(0.5);
    config.structure.terminal_deviation_bounds.setConstant(0.5);
    config.structure.terminal_absolute_bounds.setConstant(0.5);
    config.structure.terminal_liquid_increment_eta = 0.1;
    config.structure.terminal_liquid_increment_eta_dot = 0.1;
    config.structure.maximum_absolute_eta = 0.5;
    config.structure.maximum_absolute_eta_dot = 0.5;
    config.structure.minimum_relative_tracking_improvement = 1.0e-8;
    config.structure.minimum_absolute_tracking_improvement = 1.0e-10;
    config.structure.minimum_nonzero_residual = 1.0e-8;
    config.structure.model_dominance_margin = 0.0;
    config.tracking_scales.setOnes();
    config.liquid_weight = 0.0;
    config.residual_weight = 0.0;

    std::vector<spmpc::PhaseNominalSample> samples(40u);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        spmpc::PhaseNominalSample& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * config.execution.dt;
        sample.s = 0.2 * sample.t;
        sample.x = sample.s;
        sample.v = 0.2;
        sample.kappa_v = 0.2;
        sample.augmented_execution_valid = true;
        sample.augmented_execution.valid = true;
        sample.augmented_execution.stage_index = index;
        sample.augmented_execution.robot.x = sample.x;
        sample.augmented_execution.robot.v = sample.v;
        sample.augmented_execution.linear.actuator_output = sample.v;
        sample.augmented_execution.linear.pending_commands.assign(4u, 0.2);
        sample.augmented_execution.angular.pending_commands.assign(1u, 0.0);
    }
    btr::BtClosedLoopModel model;
    if (!model.configure(config.execution, config.slosh, config.structure,
                         &samples, error)) {
        std::cerr << "SYNTHETIC_QP_SELF_TEST_MODEL_FAILED: " << error
                  << '\n';
        return 2;
    }
    constexpr std::size_t kSyntheticPhase = 5u;
    const std::size_t horizon = static_cast<std::size_t>(
        config.structure.residual_prefix_steps +
        config.structure.recovery_suffix_steps);
    const btr::RecoverableTube tube = btr::buildLinearizedRecoverableTube(
        model, model.artifactState(0), 0u, kSyntheticPhase + horizon);
    if (!tube.valid) {
        std::cerr << "SYNTHETIC_QP_SELF_TEST_TUBE_FAILED: " << tube.status
                  << '\n';
        return 2;
    }
    Scenario scenario;
    scenario.id = "synthetic_longitudinal_error";
    scenario.disturbance = "SYNTHETIC";
    scenario.phase_index = kSyntheticPhase;
    const btr::RecoverableTubeStage* initial = tubeStage(
        tube, scenario.phase_index);
    if (initial == nullptr) return 2;
    scenario.state = initial->center;
    scenario.state.execution.robot.x -= 0.02;
    scenario.require_useful_candidate = true;
    const btr::ClosedLoopRolloutResult baseline = model.rollout(
        scenario.state, scenario.phase_index,
        std::vector<btr::ResidualVector>(
            horizon, btr::ResidualVector::Zero()));
    if (!baseline.valid) {
        std::cerr << "SYNTHETIC_QP_SELF_TEST_BASELINE_FAILED: "
                  << baseline.status << '\n';
        return 2;
    }
    const CostSummary baseline_cost = evaluateCost(
        config, model, tube, baseline);
    const QpCandidate candidate = solveFullStageResidualQp(
        config, model, tube, scenario, baseline);
    if (!baseline_cost.valid || !candidate.valid ||
        !candidate.solution_audited || candidate.certificate_valid ||
        candidate.residuals.empty()) {
        const btr::RecoverableTubeStage* qp_terminal = tubeStage(
            tube, scenario.phase_index + horizon);
        const btr::StateVector terminal_deviation = qp_terminal == nullptr
            ? btr::StateVector::Constant(
                  std::numeric_limits<double>::quiet_NaN())
            : model.difference(baseline.states.back(), qp_terminal->center);
        std::cerr << "SYNTHETIC_QP_SELF_TEST_SOLVE_FAILED: "
                  << candidate.status << " initial_x_half_width="
                  << initial->half_width[0] << " terminal_x_half_width="
                  << (qp_terminal == nullptr
                          ? std::numeric_limits<double>::quiet_NaN()
                          : qp_terminal->half_width[0])
                  << " terminal_x_deviation=" << terminal_deviation[0]
                  << '\n';
        return 2;
    }
    double maximum_residual = 0.0;
    for (const btr::ResidualVector& residual : candidate.residuals) {
        maximum_residual = std::max(
            maximum_residual, residual.cwiseAbs().maxCoeff());
    }
    const btr::ClosedLoopRolloutResult nonlinear = model.rollout(
        scenario.state, scenario.phase_index, candidate.residuals);
    const CostSummary nonlinear_cost = evaluateCost(
        config, model, tube, nonlinear);
    if (maximum_residual < config.structure.minimum_nonzero_residual ||
        !nonlinear.valid || !nonlinear_cost.valid ||
        nonlinear_cost.tracking >= baseline_cost.tracking) {
        std::cerr << "SYNTHETIC_QP_SELF_TEST_NONLINEAR_FAILED\n";
        return 2;
    }

    CampaignConfig impossible = config;
    impossible.structure.minimum_absolute_tracking_improvement =
        0.99 * baseline_cost.tracking;
    const QpCandidate certificate = solveFullStageResidualQp(
        impossible, model, tube, scenario, baseline);
    if (certificate.valid || !certificate.solved ||
        !certificate.certificate_valid ||
        !std::isfinite(certificate.certificate_lower_bound) ||
        certificate.certificate_lower_bound <=
            certificate.certificate_threshold ||
        certificate.status !=
            "CONVEX_TRACKING_IMPROVEMENT_THRESHOLD_INFEASIBLE") {
        std::cerr << "SYNTHETIC_QP_SELF_TEST_CERTIFICATE_FAILED: "
                  << certificate.status << '\n';
        return 2;
    }
    std::cout << "SYNTHETIC_QP_SELF_TEST_PASSED residual="
              << maximum_residual << " tracking_before="
              << baseline_cost.tracking << " tracking_after="
              << nonlinear_cost.tracking << '\n';
    return 0;
}

int run(int argc, char** argv) {
    Arguments args;
    if (!parseArguments(argc, argv, args)) {
        std::cerr << "usage: spmpc_build_bt_residual_structure_bundle"
                  << " --config PATH --artifact PATH --output-dir DIR"
                  << " --source-head HEX40\n";
        return 2;
    }
    std::string mismatched_binary;
    if (!compiledDependencyHeadsMatch(
            args.source_head, mismatched_binary)) {
        std::cerr << "ERROR: executable/dependency build HEAD does not match "
                     "--source-head; reconfigure and rebuild after the "
                     "freeze commit; mismatch=" << mismatched_binary
                  << '\n';
        return 2;
    }
    CampaignConfig config;
    std::string error;
    if (!loadConfig(args.config, config, error)) {
        std::cerr << "ERROR: invalid frozen config: " << error << '\n';
        return 2;
    }
    if (!matchesCompiledExecutionContract(config)) {
        std::cerr << "ERROR: YAML execution/slosh values do not match the "
                     "compiled contract image\n";
        return 2;
    }
    std::string repository_root;
    std::vector<FileDigest> source_digests;
    std::vector<FileDigest> runtime_dependency_digests;
    if (!verifyGitProvenance(args, repository_root, error) ||
        !verifyFrozenSourcesTracked(repository_root, error) ||
        !collectSourceDigests(repository_root, source_digests, error) ||
        !collectRuntimeDependencyDigests(
            repository_root, runtime_dependency_digests, error)) {
        std::cerr << "ERROR: source provenance rejected: " << error << '\n';
        return 2;
    }
    const std::string config_hash = sha256File(args.config);
    const std::string artifact_hash = sha256File(args.artifact);
    const std::string builder_hash = sha256File(argv[0]);
    if (config_hash.empty() || artifact_hash.empty() || builder_hash.empty() ||
        artifact_hash != config.structure.expected_artifact_sha256) {
        std::cerr << "ERROR: input hash mismatch\n";
        return 2;
    }
    const std::string frozen_config_relative =
        "src/scout_apps/control/spmpc_local_planner/config/simulation/"
        "bt_residual_structural_go_no_go_v1.yaml";
    const auto frozen_config_digest = std::find_if(
        source_digests.begin(), source_digests.end(),
        [&](const FileDigest& digest) {
            return digest.path == frozen_config_relative;
        });
    if (frozen_config_digest == source_digests.end() ||
        frozen_config_digest->sha256 != config_hash) {
        std::cerr << "ERROR: input config is not the frozen tracked config\n";
        return 2;
    }
    spmpc::NominalSequenceArtifact artifact;
    const spmpc::NominalArtifactLoadResult loaded = artifact.loadCsv(
        args.artifact);
    if (!loaded.success ||
        artifact.metadata().contract_id !=
            config.structure.expected_artifact_contract_id ||
        artifact.metadata().execution_contract_hash !=
            config.structure.expected_execution_contract_hash ||
        artifact.metadata().execution_state_width != btr::kStateWidth ||
        artifact.metadata().linear_buffer_count != 4 ||
        artifact.metadata().angular_buffer_count != 1) {
        std::cerr << "ERROR: frozen artifact rejected: " << loaded.status
                  << ' ' << loaded.detail << '\n';
        return 2;
    }
    btr::BtClosedLoopModel model;
    if (!model.configure(config.execution, config.slosh,
                         config.structure, &artifact.samples(), error)) {
        std::cerr << "ERROR: BT closed-loop model rejected: " << error
                  << '\n';
        return 2;
    }
    if (!createExclusiveDirectory(args.output_dir, error)) {
        std::cerr << "ERROR: " << error << '\n';
        return 2;
    }

    const btr::RecoverableTube tube =
        btr::buildLinearizedRecoverableTube(
            model, model.artifactState(0), 0, artifact.size() - 1);
    if (!tube.valid) {
        const std::vector<std::string> reasons = {
            "CR1_OR_CR2_BUILD_FAILED:" + tube.status};
        const std::string invalid_path =
            args.output_dir + "/invalid_evidence.json";
        std::vector<FileDigest> output_digests;
        const bool evidence_written = writeInvalidEvidence(
                invalid_path, reasons) &&
            collectOutputDigests({invalid_path}, output_digests) &&
            writeManifest(
                args.output_dir + "/bundle_manifest.json", config, args,
                repository_root, config_hash, artifact_hash, builder_hash,
                tube, source_digests, runtime_dependency_digests,
                output_digests);
        if (!evidence_written) {
            std::cerr << "ERROR: failed to write build-failure evidence\n";
            return 2;
        }
        std::cerr << "INVALID_EVIDENCE_CR0_CR2: " << tube.status << '\n';
        return 2;
    }
    std::vector<Scenario> scenarios = makeScenarios(
        config, model, tube, artifact.samples(), error);
    if (scenarios.empty()) {
        const std::vector<std::string> reasons = {
            "STATE_ENVELOPE_BUILD_FAILED:" + error};
        const std::string invalid_path =
            args.output_dir + "/invalid_evidence.json";
        std::vector<FileDigest> output_digests;
        const bool evidence_written = writeInvalidEvidence(
                invalid_path, reasons) &&
            collectOutputDigests({invalid_path}, output_digests) &&
            writeManifest(
                args.output_dir + "/bundle_manifest.json", config, args,
                repository_root, config_hash, artifact_hash, builder_hash,
                tube, source_digests, runtime_dependency_digests,
                output_digests);
        if (!evidence_written) {
            std::cerr << "ERROR: failed to write envelope-failure evidence\n";
            return 2;
        }
        std::cerr << "INVALID_EVIDENCE_CR0_CR2: " << error << '\n';
        return 2;
    }
    std::vector<ScenarioResult> results;
    results.reserve(scenarios.size());
    for (const Scenario& scenario : scenarios) {
        results.push_back(evaluateScenario(
            config, model, tube, artifact.samples(), scenario));
    }
    bool all_evidence_valid = true;
    bool all_baselines = true;
    std::size_t d1_total = 0;
    std::size_t d1_useful = 0;
    std::size_t d2_total = 0;
    std::size_t d2_useful = 0;
    std::vector<std::string> reasons;
    std::vector<std::string> invalid_reasons;
    for (const ScenarioResult& result : results) {
        all_evidence_valid = all_evidence_valid && result.evidence_valid;
        all_baselines = all_baselines && result.baseline_valid &&
            result.baseline_inside_tube && result.baseline_recovered;
        if (result.disturbance == config.d1_id) {
            ++d1_total;
            d1_useful += result.useful_candidate;
        } else if (result.disturbance == config.d2_id) {
            ++d2_total;
            d2_useful += result.useful_candidate;
        }
        if (!result.evidence_valid) {
            invalid_reasons.push_back(result.id + ":" + result.status);
        } else if (!result.baseline_valid || !result.baseline_inside_tube ||
            !result.baseline_recovered) {
            reasons.push_back(result.id + ":" + result.status);
        } else if ((result.disturbance == config.d1_id ||
                    result.disturbance == config.d2_id) &&
                   !result.useful_candidate) {
            reasons.push_back(result.id + ":" + result.status);
        }
    }
    const double d1_fraction = d1_total > 0
        ? static_cast<double>(d1_useful) / static_cast<double>(d1_total)
        : 0.0;
    const double d2_fraction = d2_total > 0
        ? static_cast<double>(d2_useful) / static_cast<double>(d2_total)
        : 0.0;
    if (d1_fraction < config.minimum_useful_fraction_each) {
        reasons.push_back("D1_USEFUL_NONZERO_FRACTION_BELOW_FREEZE");
    }
    if (d2_fraction < config.minimum_useful_fraction_each) {
        reasons.push_back("D2_USEFUL_NONZERO_FRACTION_BELOW_FREEZE");
    }
    if (!all_baselines) {
        reasons.push_back("BT_COUNTERFACTUAL_BASELINE_NOT_RECOVERABLE");
    }
    const bool go = all_baselines &&
        d1_fraction >= config.minimum_useful_fraction_each &&
        d2_fraction >= config.minimum_useful_fraction_each;
    const std::string decision = go
        ? config.success_decision
        : config.failure_decision;
    if (go) reasons.push_back("ALL_FROZEN_CR0_CR2_CONDITIONS_PASSED");

    const std::string tube_path =
        args.output_dir + "/bt_recoverable_tube.csv";
    const std::string linearization_path =
        args.output_dir + "/bt_closed_loop_linearization.csv";
    const std::string report_path =
        args.output_dir + "/structural_report.json";
    if (!all_evidence_valid) {
        const std::string invalid_path =
            args.output_dir + "/invalid_evidence.json";
        std::vector<FileDigest> invalid_output_digests;
        const bool invalid_outputs_ok =
            writeTubeCsv(tube_path, model, tube) &&
            writeLinearizationCsv(linearization_path, tube) &&
            writeScenarioReport(
                report_path, config, results, false, all_baselines,
                d1_fraction, d2_fraction, "INVALID_EVIDENCE_CR0_CR2") &&
            writeInvalidEvidence(invalid_path, invalid_reasons) &&
            collectOutputDigests(
                {tube_path, linearization_path, report_path, invalid_path},
                invalid_output_digests) &&
            writeManifest(
                args.output_dir + "/bundle_manifest.json", config, args,
                repository_root, config_hash, artifact_hash, builder_hash,
                tube, source_digests, runtime_dependency_digests,
                invalid_output_digests);
        if (!invalid_outputs_ok) {
            std::cerr << "ERROR: failed to write invalid-evidence bundle\n";
            return 2;
        }
        std::cerr << "INVALID_EVIDENCE_CR0_CR2 output="
                  << args.output_dir << '\n';
        return 2;
    }
    const std::string decision_path =
        args.output_dir + "/route_decision.json";
    std::vector<FileDigest> output_digests;
    const bool outputs_ok =
        writeTubeCsv(tube_path, model, tube) &&
        writeLinearizationCsv(linearization_path, tube) &&
        writeScenarioReport(report_path, config, results, true,
                            all_baselines, d1_fraction, d2_fraction,
                            decision) &&
        writeRouteDecision(decision_path, decision, reasons) &&
        collectOutputDigests(
            {tube_path, linearization_path, report_path, decision_path},
            output_digests) &&
        // Manifest is deliberately last, so every other evidence file is
        // content-addressed by the final record.
        writeManifest(args.output_dir + "/bundle_manifest.json",
                      config, args, repository_root, config_hash,
                      artifact_hash, builder_hash, tube, source_digests,
                      runtime_dependency_digests, output_digests);
    if (!outputs_ok) {
        std::cerr << "ERROR: failed to write structural bundle\n";
        return 2;
    }
    std::cout << decision << " output=" << args.output_dir
              << " d1_useful_fraction=" << d1_fraction
              << " d2_useful_fraction=" << d2_fraction << '\n';
    return go ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 3 &&
        std::string(argv[1]) == "--internal-synthetic-qp-self-test") {
        return runSyntheticQpSelfTest(argv[2]);
    }
    return run(argc, argv);
}
