#include "spmpc_local_planner/controller/control_cycle_engine.h"
#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"
#include "spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h"
#include "spmpc_local_planner/phase_rejoin/execution_compatibility_gate.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_horizon_context_builder.h"
#include "spmpc_local_planner/simulation/independent_scout_liquid_plant.h"
#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_local_planner/solver/api/backend.h"
#include "spmpc_local_planner/config/variant_config.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <limits.h>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace spmpc_local_planner {
namespace simulation {
namespace closed_loop_trial {

namespace manifest = delay_augmented_phase_solver_manifest;

constexpr char kConditionSchema[] =
    "spmpc_closed_loop_trial_condition_v1";
constexpr char kSummarySchema[] =
    "spmpc_closed_loop_trial_summary_v1";
constexpr char kCycleSchema[] =
    "spmpc_closed_loop_trial_cycle_v2";
constexpr double kRequiredControlRateHz = 30.0;
constexpr StampNs kStampBaseNs = 10000000000LL;
constexpr int kUsageExit = 2;
constexpr int kConfigurationExit = 3;
constexpr int kRuntimeExit = 4;

enum class TrialMode {
    OrdinaryMpcc,
    SmoothMatchMpcc,
    OfflineReplay,
    ResidualNoGate,
    PhaseRejoinFull,
    InputShaping,
};

struct Arguments {
    std::string plant_path;
    std::string path_path;
    std::string artifact_path;
    std::string condition_path;
    std::string cycle_csv_path;
    std::string summary_json_path;
    std::uint32_t seed = 0;
};

struct ConditionConfig {
    std::string condition_id;
    std::string implementation_id;
    TrialMode mode = TrialMode::OrdinaryMpcc;
    double control_rate_hz = kRequiredControlRateHz;
    double max_motion_sec = 30.0;
    double fixed_tail_sec = 4.0;
    double publish_latency_sec = 0.01;
    double smooth_global_time_scale = 1.0;
    bool pilot_tuned_and_frozen = false;
    bool pilot_only = false;
    bool formal_c3_c4_causal_comparison_ready = false;
    double residual_longitudinal_gain = 0.8;
    double residual_lateral_gain = 1.2;
    double residual_yaw_gain = 1.5;
    double residual_velocity_gain = 0.4;
    double residual_omega_gain = 0.4;
    double max_residual_v = 0.08;
    double max_residual_omega = 0.20;
    double zvd_max_discrete_residual = 0.05;
    double task_success_goal_tolerance_m = 0.20;
};

struct PathAsset {
    ReferencePath reference;
    IndependentPlantInitialPose initial_pose;
};

struct CycleRecord {
    std::uint64_t cycle_id = 0;
    double time_sec = 0.0;
    std::string window = "motion";
    IndependentPlantState plant;
    double tracking_error_m = 0.0;
    double progress_s = 0.0;
    SloshState observer;
    double observer_height_m = 0.0;
    bool solver_success = false;
    std::string raw_solver_status = "NOT_RUN";
    std::string phase_status = "NOT_RUN";
    std::string final_status = "NOT_RUN";
    bool gate_evaluated = false;
    bool terminal_gate_accepted = false;
    bool current_execution_compatible = false;
    bool terminal_execution_compatible = false;
    bool recovery_used = false;
    bool controlled_stop_used = false;
    bool selected_phase_valid = false;
    std::size_t clock_index = 0;
    std::size_t candidate_window_begin_index = 0;
    std::size_t candidate_window_end_index = 0;
    std::size_t selected_phase_index = 0;
    int phase_lead_steps = 0;
    bool execution_candidate_filter_applied = false;
    std::size_t execution_rejected_candidate_count = 0;
    double selected_execution_max_normalized_error = 0.0;
    double acados_solve_time_ms = 0.0;
    double backend_wall_time_ms = 0.0;
    VelocityCommand final_command;
    CommandSource command_source = CommandSource::None;
    StampNs publish_stamp_ns = 0;
    IndependentPlantPublishReceipt plant_publish;
};

struct TrialCounters {
    std::size_t solver_failures = 0;
    std::size_t gate_evaluations = 0;
    std::size_t terminal_gate_accepts = 0;
    std::size_t recovery_actions = 0;
    std::size_t controlled_stops = 0;
    std::size_t publications = 0;
    std::size_t publication_failures = 0;
    std::size_t execution_candidate_filter_cycles = 0;
    std::size_t execution_rejected_candidates = 0;
    double max_selected_execution_normalized_error = 0.0;
    std::vector<double> acados_solve_times_ms;
    std::vector<double> backend_wall_times_ms;
    std::size_t kkt_residual_samples = 0;
    double max_stationarity_residual = 0.0;
    double max_equality_residual = 0.0;
    double max_inequality_residual = 0.0;
    double max_complementarity_residual = 0.0;
    int max_sqp_iterations = 0;
    int max_qp_iterations = 0;
    std::string solver_id;
    std::string nlp_solver_type;
    std::string solver_config_hash;
    std::map<std::string, std::size_t> raw_solver_status_counts;
    std::map<std::string, std::size_t> phase_status_counts;
    std::map<std::string, std::size_t> final_status_counts;
};

struct SolverFailureDiagnostic {
    bool valid = false;
    std::uint64_t cycle_id = 0;
    std::string raw_solver_status;
    std::size_t clock_index = 0;
    std::size_t candidate_window_begin_index = 0;
    std::size_t candidate_window_end_index = 0;
    std::string candidate_status = "NOT_RUN";
    std::vector<PhaseCandidateResult::ExecutionCandidateAudit>
        execution_candidate_audits;
    std::size_t selected_phase_index = 0;
    std::size_t terminal_phase_index = 0;
    ExecutionAugmentedState initial_execution;
    double initial_progress_s = 0.0;
    bool warm_start_rollout_valid = false;
    std::string warm_start_rollout_status = "NOT_RUN";
    ExecutionAugmentedState warm_start_terminal_execution;
    double warm_start_terminal_progress_s = 0.0;
    int worst_execution_stage = -1;
    double max_stage_execution_normalized_error = 0.0;
    double min_linear_residual_margin =
        std::numeric_limits<double>::infinity();
    double min_angular_residual_margin =
        std::numeric_limits<double>::infinity();
    double min_linear_output_margin =
        std::numeric_limits<double>::infinity();
    double min_angular_output_margin =
        std::numeric_limits<double>::infinity();
    double min_acceleration_margin =
        std::numeric_limits<double>::infinity();
    double min_angular_acceleration_margin =
        std::numeric_limits<double>::infinity();
    double min_progress_rate_margin =
        std::numeric_limits<double>::infinity();
    bool terminal_empirical_gate_valid = false;
    double terminal_empirical_metric = 0.0;
    double terminal_empirical_margin = 0.0;
    bool terminal_execution_gate_valid = false;
    double terminal_execution_normalized_error = 0.0;
    double terminal_execution_margin = 0.0;
    bool solver_residuals_evaluated = false;
    int solver_nlp_status = -1;
    int solver_qp_status = -1;
    double stationarity_residual = 0.0;
    double equality_residual = 0.0;
    double inequality_residual = 0.0;
    double complementarity_residual = 0.0;
    PreSolveSnapshotDebug solver_snapshot;
};

std::string jsonEscape(const std::string& text) {
    std::ostringstream out;
    for (const char character : text) {
        switch (character) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (static_cast<unsigned char>(character) < 0x20u) {
                out << "?";
            } else {
                out << character;
            }
            break;
        }
    }
    return out.str();
}

std::string csvEscape(const std::string& text) {
    if (text.find_first_of(",\"\n\r") == std::string::npos) {
        return text;
    }
    std::string escaped = "\"";
    for (const char character : text) {
        if (character == '"') escaped += '"';
        escaped += character;
    }
    escaped += '"';
    return escaped;
}

bool finite(double value) {
    return std::isfinite(value);
}

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

std::string modeName(TrialMode mode) {
    switch (mode) {
    case TrialMode::OrdinaryMpcc: return "ordinary_mpcc";
    case TrialMode::SmoothMatchMpcc: return "smooth_match_mpcc";
    case TrialMode::OfflineReplay: return "offline_replay";
    case TrialMode::ResidualNoGate: return "residual_no_gate";
    case TrialMode::PhaseRejoinFull: return "phase_rejoin_full";
    case TrialMode::InputShaping: return "input_shaping";
    }
    return "unknown";
}

bool parseMode(const std::string& text, TrialMode& mode) {
    if (text == "ordinary_mpcc") mode = TrialMode::OrdinaryMpcc;
    else if (text == "smooth_match_mpcc") mode = TrialMode::SmoothMatchMpcc;
    else if (text == "offline_replay") mode = TrialMode::OfflineReplay;
    else if (text == "residual_no_gate") mode = TrialMode::ResidualNoGate;
    else if (text == "phase_rejoin_full") mode = TrialMode::PhaseRejoinFull;
    else if (text == "input_shaping") mode = TrialMode::InputShaping;
    else return false;
    return true;
}

bool parseUint32(const std::string& text, std::uint32_t& value) {
    try {
        std::size_t parsed = 0;
        const unsigned long long raw = std::stoull(text, &parsed, 10);
        if (parsed != text.size() ||
            raw > std::numeric_limits<std::uint32_t>::max()) {
            return false;
        }
        value = static_cast<std::uint32_t>(raw);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseArguments(int argc, char** argv, Arguments& args,
                    std::string& error) {
    error.clear();
    for (int index = 1; index < argc; ++index) {
        const std::string key = argv[index];
        if (key == "--help") {
            error = "HELP";
            return false;
        }
        if (index + 1 >= argc) {
            error = "missing value after " + key;
            return false;
        }
        const std::string value = argv[++index];
        if (key == "--plant") args.plant_path = value;
        else if (key == "--path") args.path_path = value;
        else if (key == "--artifact") args.artifact_path = value;
        else if (key == "--condition") args.condition_path = value;
        else if (key == "--cycle-csv") args.cycle_csv_path = value;
        else if (key == "--summary-json") args.summary_json_path = value;
        else if (key == "--seed") {
            if (!parseUint32(value, args.seed)) {
                error = "invalid --seed";
                return false;
            }
        } else {
            error = "unknown argument " + key;
            return false;
        }
    }
    if (args.plant_path.empty() || args.path_path.empty() ||
        args.artifact_path.empty() || args.condition_path.empty() ||
        args.cycle_csv_path.empty() || args.summary_json_path.empty() ||
        args.seed == 0u) {
        error = "all six paths and a nonzero --seed are required";
        return false;
    }
    return true;
}

void printUsage(std::ostream& output) {
    output
        << "usage: spmpc_phase_rejoin_closed_loop_trial "
        << "--plant PLANT.yaml --path PATH.json --artifact V3.csv "
        << "--condition CONDITION.yaml --seed N "
        << "--cycle-csv CYCLES.csv --summary-json SUMMARY.json\n";
}

bool canonicalExistingPath(const std::string& path,
                           std::string& canonical,
                           std::string& error) {
    char resolved[PATH_MAX];
    if (::realpath(path.c_str(), resolved) == nullptr) {
        error = "cannot resolve existing path " + path + ": " +
            std::strerror(errno);
        return false;
    }
    canonical = resolved;
    return true;
}

bool canonicalOutputTarget(const std::string& path,
                           std::string& canonical,
                           std::string& error) {
    const std::size_t slash = path.find_last_of('/');
    const std::string parent = slash == std::string::npos
        ? "."
        : (slash == 0 ? "/" : path.substr(0, slash));
    const std::string name = slash == std::string::npos
        ? path
        : path.substr(slash + 1);
    if (name.empty() || name == "." || name == "..") {
        error = "invalid output filename " + path;
        return false;
    }
    std::string canonical_parent;
    if (!canonicalExistingPath(parent, canonical_parent, error)) {
        return false;
    }
    canonical = canonical_parent +
        (canonical_parent == "/" ? "" : "/") + name;
    struct stat info;
    if (::lstat(canonical.c_str(), &info) == 0) {
        error = "refusing to overwrite existing output " + canonical;
        return false;
    }
    if (errno != ENOENT) {
        error = "cannot inspect output " + canonical + ": " +
            std::strerror(errno);
        return false;
    }
    return true;
}

bool validateAndPrepareOutputPaths(
    const Arguments& args,
    Arguments& working,
    std::string& final_cycle,
    std::string& final_summary,
    std::string& error) {
    std::vector<std::string> inputs;
    for (const std::string& path : {
             args.plant_path, args.path_path, args.artifact_path,
             args.condition_path}) {
        std::string canonical;
        if (!canonicalExistingPath(path, canonical, error)) return false;
        if (std::find(inputs.begin(), inputs.end(), canonical) != inputs.end()) {
            error = "input path alias detected: " + canonical;
            return false;
        }
        inputs.push_back(canonical);
    }
    if (!canonicalOutputTarget(args.cycle_csv_path, final_cycle, error) ||
        !canonicalOutputTarget(args.summary_json_path, final_summary, error)) {
        return false;
    }
    if (final_cycle == final_summary ||
        std::find(inputs.begin(), inputs.end(), final_cycle) != inputs.end() ||
        std::find(inputs.begin(), inputs.end(), final_summary) != inputs.end()) {
        error = "output path aliases another input or output";
        return false;
    }
    working = args;
    const std::string suffix = ".partial." +
        std::to_string(static_cast<long long>(::getpid()));
    working.cycle_csv_path = final_cycle + suffix;
    working.summary_json_path = final_summary + suffix;
    for (const std::string& temporary : {
             working.cycle_csv_path, working.summary_json_path}) {
        struct stat info;
        if (::lstat(temporary.c_str(), &info) == 0 || errno != ENOENT) {
            error = "temporary output already exists: " + temporary;
            return false;
        }
    }
    return true;
}

bool publishOutputsAtomically(const Arguments& working,
                              const std::string& final_cycle,
                              const std::string& final_summary,
                              std::string& error) {
    error.clear();
    if (::link(working.cycle_csv_path.c_str(), final_cycle.c_str()) != 0) {
        error = "cannot publish cycle CSV without overwrite: " +
            std::string(std::strerror(errno));
        return false;
    }
    if (::link(working.summary_json_path.c_str(), final_summary.c_str()) != 0) {
        const std::string cause = std::strerror(errno);
        ::unlink(final_cycle.c_str());
        error = "cannot publish summary JSON without overwrite: " + cause;
        return false;
    }
    if (::unlink(working.cycle_csv_path.c_str()) != 0 ||
        ::unlink(working.summary_json_path.c_str()) != 0) {
        error = "published outputs but failed to remove temporary links";
        return false;
    }
    return true;
}

template <typename T>
bool requiredScalar(const YAML::Node& root, const char* key, T& value,
                    std::string& error) {
    try {
        if (!root[key] || !root[key].IsScalar()) {
            error = std::string("missing scalar condition field ") + key;
            return false;
        }
        value = root[key].as<T>();
        return true;
    } catch (const std::exception& exception) {
        error = std::string("invalid condition field ") + key + ": " +
            exception.what();
        return false;
    }
}

template <typename T>
void optionalScalar(const YAML::Node& root, const char* key, T& value) {
    if (root && root[key] && root[key].IsScalar()) {
        value = root[key].as<T>();
    }
}

struct ExpectedSemantics {
    bool offline_nominal = false;
    bool online_residual = false;
    bool recovery_gate = false;
    bool execution_compatibility_gate = false;
    bool stored_recovery_action = false;
    bool input_shaping = false;
};

ExpectedSemantics expectedSemantics(TrialMode mode) {
    ExpectedSemantics out;
    if (mode == TrialMode::OfflineReplay) {
        out.offline_nominal = true;
    } else if (mode == TrialMode::ResidualNoGate) {
        out.offline_nominal = true;
        out.online_residual = true;
        out.execution_compatibility_gate = true;
        out.stored_recovery_action = true;
    } else if (mode == TrialMode::PhaseRejoinFull) {
        out.offline_nominal = true;
        out.online_residual = true;
        out.recovery_gate = true;
        out.execution_compatibility_gate = true;
        out.stored_recovery_action = true;
    } else if (mode == TrialMode::InputShaping) {
        out.input_shaping = true;
    }
    return out;
}

bool loadCondition(const std::string& path, ConditionConfig& config,
                   std::string& error) {
    error.clear();
    try {
        const YAML::Node root = YAML::LoadFile(path);
        std::string schema;
        std::string mode;
        bool implementation_complete = false;
        if (!requiredScalar(root, "schema", schema, error) ||
            schema != kConditionSchema) {
            if (error.empty()) error = "condition schema rejected: " + schema;
            return false;
        }
        if (!requiredScalar(root, "condition_id", config.condition_id, error) ||
            !requiredScalar(root, "implementation_id",
                            config.implementation_id, error) ||
            !requiredScalar(root, "implementation_complete",
                            implementation_complete, error) ||
            !requiredScalar(root, "mode", mode, error)) {
            return false;
        }
        if (!implementation_complete || config.implementation_id.empty()) {
            error = "condition implementation is not complete";
            return false;
        }
        if (!parseMode(mode, config.mode)) {
            error = "unknown condition mode " + mode;
            return false;
        }
        const std::string expected_id = config.mode == TrialMode::InputShaping
            ? "IS"
            : "C" + std::to_string(static_cast<int>(config.mode));
        if (config.condition_id != expected_id) {
            error = "condition_id/mode mismatch";
            return false;
        }
        const YAML::Node trial = root["trial"];
        if (!trial || !requiredScalar(trial, "control_rate_hz",
                                     config.control_rate_hz, error) ||
            !requiredScalar(trial, "max_motion_sec",
                            config.max_motion_sec, error) ||
            !requiredScalar(trial, "fixed_tail_sec",
                            config.fixed_tail_sec, error) ||
            !requiredScalar(trial, "publish_latency_sec",
                            config.publish_latency_sec, error)) {
            if (error.empty()) error = "condition trial block is missing";
            return false;
        }
        optionalScalar(trial, "task_success_goal_tolerance_m",
                       config.task_success_goal_tolerance_m);
        optionalScalar(root, "global_time_scale",
                       config.smooth_global_time_scale);
        optionalScalar(root, "pilot_tuned_and_frozen",
                       config.pilot_tuned_and_frozen);
        optionalScalar(root, "pilot_only", config.pilot_only);
        optionalScalar(root, "formal_c3_c4_causal_comparison_ready",
                       config.formal_c3_c4_causal_comparison_ready);
        const YAML::Node residual = root["residual_feedback"];
        optionalScalar(residual, "longitudinal_gain",
                       config.residual_longitudinal_gain);
        optionalScalar(residual, "lateral_gain",
                       config.residual_lateral_gain);
        optionalScalar(residual, "yaw_gain", config.residual_yaw_gain);
        optionalScalar(residual, "velocity_gain",
                       config.residual_velocity_gain);
        optionalScalar(residual, "omega_gain",
                       config.residual_omega_gain);
        optionalScalar(residual, "max_residual_v",
                       config.max_residual_v);
        optionalScalar(residual, "max_residual_omega",
                       config.max_residual_omega);
        const YAML::Node shaper = root["input_shaper"];
        optionalScalar(shaper, "max_discrete_residual",
                       config.zvd_max_discrete_residual);
        if (!finite(config.control_rate_hz) ||
            std::abs(config.control_rate_hz - kRequiredControlRateHz) > 1e-12 ||
            !finite(config.max_motion_sec) || config.max_motion_sec <= 0.0 ||
            config.max_motion_sec > 300.0 ||
            !finite(config.fixed_tail_sec) || config.fixed_tail_sec < 0.0 ||
            config.fixed_tail_sec > 30.0 ||
            !finite(config.publish_latency_sec) ||
            config.publish_latency_sec < 0.0 ||
            config.publish_latency_sec > 0.5 ||
            !finite(config.task_success_goal_tolerance_m) ||
            config.task_success_goal_tolerance_m <= 0.0 ||
            config.task_success_goal_tolerance_m > 1.0 ||
            !finite(config.smooth_global_time_scale) ||
            config.smooth_global_time_scale < 0.5 ||
            config.smooth_global_time_scale > 2.0 ||
            !finite(config.max_residual_v) || config.max_residual_v < 0.0 ||
            !finite(config.max_residual_omega) ||
            config.max_residual_omega < 0.0 ||
            !finite(config.zvd_max_discrete_residual) ||
            config.zvd_max_discrete_residual < 0.0 ||
            config.zvd_max_discrete_residual > 1.0) {
            error = "condition numeric contract rejected";
            return false;
        }
        const ExpectedSemantics expected = expectedSemantics(config.mode);
        const char* keys[] = {
            "offline_nominal", "online_residual", "recovery_gate",
            "execution_compatibility_gate", "stored_recovery_action",
            "input_shaping",
        };
        const bool values[] = {
            expected.offline_nominal, expected.online_residual,
            expected.recovery_gate, expected.execution_compatibility_gate,
            expected.stored_recovery_action, expected.input_shaping,
        };
        for (std::size_t index = 0; index < 6; ++index) {
            bool actual = false;
            if (!requiredScalar(root, keys[index], actual, error) ||
                actual != values[index]) {
                if (error.empty()) {
                    error = std::string("condition semantics mismatch: ") +
                        keys[index];
                }
                return false;
            }
        }
        if (config.mode == TrialMode::ResidualNoGate ||
            config.mode == TrialMode::PhaseRejoinFull) {
            if (config.pilot_only ||
                !config.formal_c3_c4_causal_comparison_ready) {
                error = "C3/C4 strict causal comparison contract is not frozen";
                return false;
            }
        }
    } catch (const std::exception& exception) {
        error = std::string("cannot load condition: ") + exception.what();
        return false;
    }
    return true;
}

bool loadPath(const std::string& path, PathAsset& asset,
              std::string& error) {
    error.clear();
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (!root["frame_id"] || !root["poses"] ||
            !root["poses"].IsSequence() || root["poses"].size() < 2) {
            error = "path JSON schema rejected";
            return false;
        }
        const std::string frame_id = root["frame_id"].as<std::string>();
        if (frame_id.empty()) {
            error = "path frame_id is empty";
            return false;
        }
        std::vector<TrajectoryPoint> points;
        points.reserve(root["poses"].size());
        for (const YAML::Node& pose : root["poses"]) {
            if (!pose["x"] || !pose["y"] || !pose["qx"] ||
                !pose["qy"] || !pose["qz"] || !pose["qw"]) {
                error = "path pose requires x/y and all quaternion fields";
                return false;
            }
            const double qx = pose["qx"].as<double>();
            const double qy = pose["qy"].as<double>();
            const double qz = pose["qz"].as<double>();
            const double qw = pose["qw"].as<double>();
            const double norm = std::sqrt(
                qx * qx + qy * qy + qz * qz + qw * qw);
            TrajectoryPoint point;
            point.x = pose["x"].as<double>();
            point.y = pose["y"].as<double>();
            if (!finite(point.x) || !finite(point.y) || !finite(qx) ||
                !finite(qy) || !finite(qz) || !finite(qw) ||
                !finite(norm) || norm <= 1e-12) {
                error = "path contains invalid pose";
                return false;
            }
            const double x = qx / norm;
            const double y = qy / norm;
            const double z = qz / norm;
            const double w = qw / norm;
            point.yaw = std::atan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z));
            point.v = pose["v"] ? pose["v"].as<double>() : 0.0;
            if (!points.empty() &&
                std::hypot(point.x - points.back().x,
                           point.y - points.back().y) <= 1e-6) {
                error = "path contains duplicate consecutive poses";
                return false;
            }
            points.push_back(point);
        }
        asset.reference.setPoints(points, frame_id);
        if (asset.reference.empty() || asset.reference.length() <= 0.0) {
            error = "path length is zero";
            return false;
        }
        asset.initial_pose.x = points.front().x;
        asset.initial_pose.y = points.front().y;
        asset.initial_pose.yaw = points.front().yaw;
    } catch (const std::exception& exception) {
        error = std::string("cannot load path: ") + exception.what();
        return false;
    }
    return true;
}

class PlantCommandSink final : public ICommandSink {
public:
    explicit PlantCommandSink(IndependentScoutLiquidPlant& plant)
        : plant_(plant) {}

    void arm(double plant_publish_time_sec, StampNs publication_stamp_ns) {
        plant_publish_time_sec_ = plant_publish_time_sec;
        publication_stamp_ns_ = publication_stamp_ns;
        last_plant_receipt_ = IndependentPlantPublishReceipt{};
        last_error_.clear();
    }

    StampNs publicationTimeNs() override {
        return publication_stamp_ns_;
    }

    PublicationReceipt publish(const FinalCommand& final) override {
        PublicationReceipt receipt;
        receipt.cycle_id = final.cycle_id;
        receipt.attempted = final.publish_enabled;
        receipt.command = final.command;
        if (!final.publish_enabled) {
            receipt.status = "SIMULATION_PUBLICATION_DISABLED";
            return receipt;
        }
        IndependentPlantCommand command;
        command.linear = final.command.linear;
        command.angular = final.command.angular;
        const bool accepted = plant_.publishCommand(
            plant_publish_time_sec_, command, last_plant_receipt_,
            last_error_);
        receipt.delivered = accepted && last_plant_receipt_.accepted;
        receipt.actual_publish_stamp_ns = receipt.delivered
            ? publication_stamp_ns_
            : 0;
        receipt.status = receipt.delivered
            ? "SIMULATION_PLANT_ACCEPTED"
            : "SIMULATION_PLANT_REJECTED_" + last_error_;
        return receipt;
    }

    const IndependentPlantPublishReceipt& lastPlantReceipt() const {
        return last_plant_receipt_;
    }

    const std::string& lastError() const { return last_error_; }

private:
    IndependentScoutLiquidPlant& plant_;
    double plant_publish_time_sec_ = 0.0;
    StampNs publication_stamp_ns_ = 0;
    IndependentPlantPublishReceipt last_plant_receipt_;
    std::string last_error_;
};

class OfflineReplaySession final : public SolverSession {
public:
    explicit OfflineReplaySession(const NominalSequenceArtifact& artifact)
        : artifact_(artifact) {}

    void setCycle(std::size_t cycle) { cycle_ = cycle; }

    bool solve(const SolverInput& input, SolverOutput& output) override {
        output = SolverOutput{};
        output.failure_kind = SolverFailureKind::None;
        output.cycle_timing = input.cycle_timing;
        if (!artifact_.valid() || artifact_.empty()) {
            output.status = "OFFLINE_ARTIFACT_UNAVAILABLE";
            output.failure_kind = SolverFailureKind::Integrity;
            return false;
        }
        if (cycle_ >= artifact_.size()) {
            output.success = true;
            output.status = "GOAL_REACHED";
            output.progress_abs_s = artifact_.metadata().path_length;
            output.progress_s = 1.0;
            return true;
        }
        const PhaseNominalSample* sample = artifact_.sample(cycle_);
        if (sample == nullptr) {
            output.status = "OFFLINE_SAMPLE_MISSING";
            output.failure_kind = SolverFailureKind::Integrity;
            return false;
        }
        output.success = true;
        output.status = "OFFLINE_V3_FINAL_COMMAND_REPLAY";
        output.cmd_v = sample->u_pub_v;
        output.cmd_omega = sample->u_pub_omega;
        output.progress_abs_s = sample->s;
        output.progress_s = artifact_.metadata().path_length > 1e-12
            ? sample->s / artifact_.metadata().path_length
            : 0.0;
        output.projector_debug.raw_valid = true;
        output.projector_debug.guarded_valid = true;
        output.projector_debug.raw_s = sample->s;
        output.projector_debug.guarded_s = sample->s;
        output.projector_debug.raw_distance =
            std::hypot(input.robot.x - sample->x,
                       input.robot.y - sample->y);
        output.projector_debug.guarded_distance =
            output.projector_debug.raw_distance;
        return true;
    }

private:
    const NominalSequenceArtifact& artifact_;
    std::size_t cycle_ = 0;
};

struct ZvdAudit {
    bool valid = false;
    double weights[3] = {1.0, 0.0, 0.0};
    int delay_steps[3] = {0, 0, 0};
    double equivalent_delay_sec[3] = {0.0, 0.0, 0.0};
    double discrete_residual = std::numeric_limits<double>::infinity();
};

ZvdAudit makeZvdAudit(const SloshModelParams& params) {
    ZvdAudit audit;
    SloshDynamics dynamics;
    if (!dynamics.configure(params)) return audit;
    const double zeta = params.damping_ratio;
    const double root = std::sqrt(std::max(1e-12, 1.0 - zeta * zeta));
    const double omega_n = dynamics.omegaN();
    const double omega_d = omega_n * root;
    const double k = std::exp(-zeta * M_PI / root);
    const double denominator = (1.0 + k) * (1.0 + k);
    audit.weights[0] = 1.0 / denominator;
    audit.weights[1] = 2.0 * k / denominator;
    audit.weights[2] = k * k / denominator;
    const double ideal_first_delay = M_PI / omega_d;
    audit.delay_steps[0] = 0;
    audit.delay_steps[1] = std::max(
        1, static_cast<int>(std::llround(ideal_first_delay / params.dt)));
    audit.delay_steps[2] = 2 * audit.delay_steps[1];
    std::complex<double> residual(0.0, 0.0);
    for (int index = 0; index < 3; ++index) {
        audit.equivalent_delay_sec[index] =
            static_cast<double>(audit.delay_steps[index]) * params.dt;
        residual += audit.weights[index] * std::exp(
            std::complex<double>(-zeta * omega_n, omega_d) *
            audit.equivalent_delay_sec[index]);
    }
    audit.discrete_residual = std::abs(residual);
    audit.valid = finite(audit.discrete_residual);
    return audit;
}

class ZvdInputShapingSession final : public SolverSession {
public:
    SolverConfigureResult configure(const SolverParams& params,
                                    const VariantConfig& variant,
                                    const ReferencePath& reference) {
        const SolverConfigureResult configured = problem_.configure(
            params, variant);
        if (!configured.success) return configured;
        problem_.setReferencePath(reference);
        audit_ = makeZvdAudit(params.slosh);
        if (!audit_.valid) {
            SolverConfigureResult failed;
            failed.status = "ZVD_LIQUID_MODEL_INVALID";
            return failed;
        }
        SolverConfigureResult result = configured;
        result.detail += " ZVD_steps=0," +
            std::to_string(audit_.delay_steps[1]) + "," +
            std::to_string(audit_.delay_steps[2]);
        configured_ = true;
        return result;
    }

    bool solve(const SolverInput& input, SolverOutput& output) override {
        SolverOutput raw;
        const bool ok = problem_.solve(input, raw);
        output = raw;
        if (!ok || !raw.success) return ok;
        if (raw.status == "GOAL_REACHED") {
            output.cmd_v = 0.0;
            output.cmd_omega = 0.0;
            return true;
        }
        VelocityCommand command;
        command.linear = raw.cmd_v;
        command.angular = raw.cmd_omega;
        raw_history_.push_front(command);
        const std::size_t keep = static_cast<std::size_t>(
            audit_.delay_steps[2] + 2);
        while (raw_history_.size() > keep) raw_history_.pop_back();
        VelocityCommand shaped;
        for (int impulse = 0; impulse < 3; ++impulse) {
            const std::size_t delay = static_cast<std::size_t>(
                audit_.delay_steps[impulse]);
            if (delay < raw_history_.size()) {
                shaped.linear += audit_.weights[impulse] *
                    raw_history_[delay].linear;
                shaped.angular += audit_.weights[impulse] *
                    raw_history_[delay].angular;
            }
        }
        output.cmd_v = shaped.linear;
        output.cmd_omega = shaped.angular;
        output.status = "ZVD_SHAPED_" + raw.status;
        return true;
    }

private:
    SpmpcProblem problem_;
    std::deque<VelocityCommand> raw_history_;
    ZvdAudit audit_;
    bool configured_ = false;
};

VariantConfig controllerVariant(TrialMode mode,
                                const ConditionConfig& condition) {
    if (mode == TrialMode::OrdinaryMpcc ||
        mode == TrialMode::InputShaping) {
        return makeVariantConfig("B0");
    }
    if (mode == TrialMode::SmoothMatchMpcc) {
        VariantConfig variant = makeVariantConfig("B_smooth");
        variant.v_ref /= condition.smooth_global_time_scale;
        return variant;
    }
    VariantConfig variant;
    variant.name = mode == TrialMode::PhaseRejoinFull
        ? "C4_phase_rejoin_full"
        : "C3_residual_no_gate";
    variant.slosh_enable = true;
    variant.smooth_priority_enable = true;
    variant.w_contour = 1.0;
    variant.w_lag = 0.2;
    variant.w_progress = 0.2;
    variant.w_v = 1.0;
    variant.w_vs = 0.3;
    variant.v_ref = 0.25;
    variant.w_control = 0.1;
    variant.w_accel = 0.0;
    variant.w_smooth = 0.1;
    variant.w_alpha = 0.1;
    variant.w_du_a = 0.1;
    variant.w_du_vs = 0.1;
    variant.w_slosh = 1.0;
    return variant;
}

SolverParams commonSolverParams() {
    const DelayAugmentedPhaseCompiledContract compiled =
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    SolverParams params;
    params.v_max = manifest::kLinearOutputMax;
    params.omega_max = manifest::kAngularOutputMax;
    params.a_max = manifest::kAccelerationMax;
    params.alpha_max = manifest::kAngularAccelerationMax;
    params.slosh = compiled.slosh;
    params.terminal.enable = true;
    params.terminal.goal_tolerance = 0.08;
    params.terminal.goal_reached_max_speed = 0.03;
    params.terminal.goal_reached_max_omega = 0.05;
    params.terminal.slowdown_enable = true;
    params.terminal.slowdown_distance = 0.80;
    params.terminal.slowdown_v_max = 0.18;
    params.terminal.capture_stop_enable = true;
    params.terminal.capture_stop_distance = 0.50;
    params.terminal.capture_v_cap = 0.18;
    params.terminal.command_clamp_enable = false;
    return params;
}

SolverParams delayAugmentedSolverParams(
    const NominalSequenceArtifact& artifact) {
    const DelayAugmentedPhaseCompiledContract compiled =
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    SolverParams params = commonSolverParams();
    params.solver_backend = kSolverBackendDelayAugmentedPhaseAcados;
    DelayAugmentedPhaseBackendParams& augmented =
        params.delay_augmented_phase;
    augmented.enabled = true;
    augmented.execution_contract_id = compiled.execution.contract_id;
    augmented.execution_contract_hash = compiled.execution.contract_hash;
    augmented.expected_state_width = compiled.state_width;
    augmented.expected_control_width = compiled.control_width;
    augmented.expected_horizon_steps = compiled.horizon_steps;
    augmented.parameter_schema_version = compiled.parameter_schema_version;
    augmented.parameter_schema_id = compiled.parameter_schema_id;
    augmented.parameter_schema_hash = compiled.parameter_schema_hash;
    augmented.expected_recovery_artifact_hash =
        artifact.metadata().recovery_artifact_hash;
    augmented.required_capabilities =
        kDelayAugmentedPhaseFormalCapabilities;
    return params;
}

DelayAugmentedPhaseCostWeights delayAugmentedWeights(
    const VariantConfig& variant,
    const SloshModelParams& slosh) {
    DelayAugmentedPhaseCostWeights weights;
    weights.position = variant.w_contour;
    weights.yaw = variant.w_lag;
    weights.progress = variant.w_progress;
    weights.v = variant.w_v;
    weights.omega = variant.w_control;
    weights.slosh_eta = variant.w_slosh;
    weights.slosh_eta_dot =
        variant.w_slosh * slosh.slosh_eta_dot_ratio;
    weights.linear_pending = variant.w_v;
    weights.angular_pending = variant.w_control;
    weights.acceleration = variant.w_control + variant.w_accel;
    weights.angular_acceleration = variant.w_alpha;
    weights.progress_rate = variant.w_vs;
    return weights;
}

PhaseRejoinRuntimeContract phaseRuntimeContract(
    const SolverParams& solver,
    const VariantConfig& variant,
    const NominalSequenceArtifact& artifact) {
    const DelayAugmentedPhaseCompiledContract compiled =
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    PhaseRejoinRuntimeContract runtime;
    runtime.dt = compiled.execution.dt;
    runtime.min_command_v = manifest::kLinearOutputMin;
    runtime.max_command_v = manifest::kLinearOutputMax;
    runtime.max_abs_command_omega = manifest::kAngularOutputMax;
    SloshDynamics dynamics;
    runtime.liquid_model_configured = dynamics.configure(solver.slosh);
    if (runtime.liquid_model_configured) {
        const double omega_n = dynamics.omegaN();
        runtime.two_zeta_omega_n =
            2.0 * solver.slosh.damping_ratio * omega_n;
        runtime.omega_n_sq = omega_n * omega_n;
        runtime.kappa_x = 1.0;
        runtime.kappa_y = 1.0;
    }
    runtime.delay_augmented_solver_requested = true;
    runtime.execution_contract_id = compiled.execution.contract_id;
    runtime.execution_contract_hash = compiled.execution.contract_hash;
    runtime.execution_state_width = compiled.state_width;
    runtime.linear_buffer_count =
        compiled.execution.linear.integer_delay_steps + 1;
    runtime.angular_buffer_count =
        compiled.execution.angular.integer_delay_steps + 1;
    runtime.solver_control_width = compiled.control_width;
    runtime.execution_front_steps = compiled.execution_front_steps;
    runtime.solver_horizon_steps = compiled.horizon_steps;
    runtime.max_published_acceleration = compiled.acceleration_max;
    runtime.max_published_angular_acceleration =
        compiled.angular_acceleration_max;
    runtime.parameter_schema_version = compiled.parameter_schema_version;
    runtime.parameter_schema_id = compiled.parameter_schema_id;
    runtime.parameter_schema_hash = compiled.parameter_schema_hash;
    runtime.recovery_artifact_hash =
        artifact.metadata().recovery_artifact_hash;
    runtime.execution_compatibility_contract =
        compiled.execution_compatibility_contract;
    runtime.solver_capabilities = compiled.capabilities;
    runtime.required_solver_capabilities =
        kDelayAugmentedPhaseFormalCapabilities;
    runtime.delay_augmented_weights =
        delayAugmentedWeights(variant, solver.slosh);
    return runtime;
}

CommandPipelineConfig commandPipelineConfig(double control_rate_hz) {
    CommandPipelineConfig config;
    config.control_frequency = control_rate_hz;
    // All six implementations own their command-rate envelope before the
    // final transaction.  A second limiter would invalidate either the OCP
    // prediction or the offline replay and is therefore disabled uniformly.
    config.linear_accel_limit_enable = false;
    config.angular_limit_enable = false;
    config.fail_closed_on_post_limit_change = true;
    config.max_post_limit_delta_v = 1e-9;
    config.max_post_limit_delta_omega = 1e-9;
    return config;
}

SafetySupervisorConfig safetyConfig(double dt) {
    SafetySupervisorConfig config;
    config.nominal_period_sec = dt;
    config.terminal_spin.enable = true;
    config.terminal_spin.omega_threshold = 0.50;
    config.terminal_spin.max_duration_sec = 2.0;
    config.tracking.enable = true;
    config.tracking.projection_enable = true;
    config.tracking.max_projection_distance_m = 0.50;
    config.tracking.max_projection_duration_sec = 0.20;
    config.tracking.spin_enable = true;
    config.tracking.spin_omega_threshold = 0.80;
    config.tracking.spin_max_duration_sec = 2.0;
    return config;
}

void prefillZeroHistory(CommandHistoryBuffer& history, double dt) {
    history.configure(2.0);
    const int count = static_cast<int>(std::ceil(1.0 / dt));
    for (int index = count; index >= 0; --index) {
        TimedCommandSample sample;
        sample.stamp_ns = kStampBaseNs - secondsToNanoseconds(index * dt);
        sample.meta.is_zero_cmd = true;
        history.push(sample);
    }
}

double nearestRankQuantile(std::vector<double> values, double probability) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const double bounded = clamp(probability, 0.0, 1.0);
    const std::size_t rank = bounded <= 0.0
        ? 0u
        : static_cast<std::size_t>(
              std::ceil(bounded * static_cast<double>(values.size()))) - 1u;
    return values[std::min(rank, values.size() - 1u)];
}

double rms(const std::vector<double>& values) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    double sum = 0.0;
    for (const double value : values) sum += value * value;
    return std::sqrt(sum / static_cast<double>(values.size()));
}

std::string jsonNumber(double value) {
    if (!finite(value)) return "null";
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::setprecision(17) << value;
    return output.str();
}

void writeJsonStringCounts(
    std::ofstream& output,
    const std::map<std::string, std::size_t>& counts) {
    output << '{';
    bool first = true;
    for (const auto& entry : counts) {
        if (!first) output << ',';
        output << "\n      \"" << jsonEscape(entry.first) << "\": "
               << entry.second;
        first = false;
    }
    if (!counts.empty()) output << '\n' << "    ";
    output << '}';
}

std::vector<double> solverState22(
    const ExecutionAugmentedState& execution,
    double progress_s) {
    std::vector<double> state;
    state.reserve(static_cast<std::size_t>(manifest::kStateCount));
    state.push_back(execution.robot.x);
    state.push_back(execution.robot.y);
    state.push_back(execution.robot.yaw);
    state.push_back(execution.robot.v);
    state.push_back(progress_s);
    state.push_back(execution.robot.omega);
    state.push_back(execution.slosh.eta_x);
    state.push_back(execution.slosh.eta_x_dot);
    state.push_back(execution.slosh.eta_y);
    state.push_back(execution.slosh.eta_y_dot);
    state.insert(state.end(),
                 execution.linear.pending_commands.begin(),
                 execution.linear.pending_commands.end());
    state.insert(state.end(),
                 execution.angular.pending_commands.begin(),
                 execution.angular.pending_commands.end());
    return state;
}

void writeJsonNumberArray(std::ofstream& output,
                          const std::vector<double>& values) {
    output << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0) output << ", ";
        output << jsonNumber(values[index]);
    }
    output << ']';
}

void writeConstraintDiagnostics(
    std::ofstream& output,
    const std::vector<DelayAugmentedPhaseNamedConstraintDiagnostics>&
        constraints) {
    output << '[';
    for (std::size_t index = 0; index < constraints.size(); ++index) {
        if (index != 0) output << ',';
        const auto& item = constraints[index];
        output << "\n        {\"stage\": " << item.stage
            << ", \"index\": " << item.index
            << ", \"name\": \"" << jsonEscape(item.name)
            << "\", \"value\": " << jsonNumber(item.value)
            << ", \"lower\": " << jsonNumber(item.lower)
            << ", \"upper\": " << jsonNumber(item.upper)
            << ", \"normalized_error\": "
            << jsonNumber(item.normalized_error)
            << ", \"violation\": " << jsonNumber(item.violation)
            << '}';
    }
    if (!constraints.empty()) output << '\n' << "      ";
    output << ']';
}

void writeConstraintAudit(
    std::ofstream& output,
    const DelayAugmentedPhaseConstraintAudit& audit) {
    output << "{\n"
        << "      \"evaluated\": "
        << (audit.evaluated ? "true" : "false") << ",\n"
        << "      \"passed\": "
        << (audit.passed ? "true" : "false") << ",\n"
        << "      \"status\": \"" << jsonEscape(audit.status)
        << "\",\n"
        << "      \"tolerance\": " << jsonNumber(audit.tolerance)
        << ",\n"
        << "      \"terminal_empirical_metric\": "
        << jsonNumber(audit.terminal_empirical_metric) << ",\n"
        << "      \"terminal_empirical_violation\": "
        << jsonNumber(audit.terminal_empirical_violation) << ",\n"
        << "      \"max_causal_state_error\": "
        << jsonNumber(audit.max_causal_state_error) << ",\n"
        << "      \"max_causal_state_error_stage\": "
        << audit.max_causal_state_error_stage << ",\n"
        << "      \"max_causal_state_error_index\": "
        << audit.max_causal_state_error_index << ",\n"
        << "      \"max_violation_stage\": "
        << audit.max_violation_stage << ",\n"
        << "      \"max_violation_index\": "
        << audit.max_violation_index << ",\n"
        << "      \"max_violation_name\": \""
        << jsonEscape(audit.max_violation_name) << "\",\n"
        << "      \"max_violation_value\": "
        << jsonNumber(audit.max_violation_value) << ",\n"
        << "      \"max_violation\": "
        << jsonNumber(audit.max_violation) << ",\n"
        << "      \"stage_constraints\": ";
    writeConstraintDiagnostics(output, audit.stage_constraints);
    output << ",\n      \"control_constraints\": ";
    writeConstraintDiagnostics(output, audit.control_constraints);
    output << ",\n      \"terminal_execution_constraints\": ";
    writeConstraintDiagnostics(
        output, audit.terminal_execution_constraints);
    output << "\n    }";
}

void writeIterationDiagnostics(
    std::ofstream& output,
    const std::vector<DelayAugmentedPhaseIterationDiagnostics>& iterations) {
    output << '[';
    for (std::size_t index = 0; index < iterations.size(); ++index) {
        if (index != 0) output << ',';
        const auto& item = iterations[index];
        output << "\n        {\"iteration\": " << item.iteration
            << ", \"stationarity\": "
            << jsonNumber(item.stationarity)
            << ", \"equality\": " << jsonNumber(item.equality)
            << ", \"inequality\": " << jsonNumber(item.inequality)
            << ", \"complementarity\": "
            << jsonNumber(item.complementarity)
            << ", \"qp_status\": " << item.qp_status
            << ", \"qp_iterations\": " << item.qp_iterations
            << ", \"step_length\": "
            << jsonNumber(item.step_length) << '}';
    }
    if (!iterations.empty()) output << '\n' << "      ";
    output << ']';
}

SolverFailureDiagnostic captureSolverFailureDiagnostic(
    const ControlCycleResult& result,
    const NominalSequenceArtifact& artifact) {
    SolverFailureDiagnostic diagnostic;
    const PhaseCandidateResult& candidate =
        result.phase_preparation.candidate;
    const DelayAugmentedPhaseSolverContext& phase =
        result.phase_preparation.solver_context.delay_augmented;
    const ExecutionHorizonContext& horizon =
        result.solver_input.execution_horizon;
    if (result.solver_success || !horizon.active ||
        !horizon.initial_state.valid) {
        return diagnostic;
    }

    diagnostic.valid = true;
    diagnostic.cycle_id = result.telemetry.cycle_id;
    diagnostic.raw_solver_status = result.solver_output.status;
    diagnostic.clock_index = candidate.clock_index;
    diagnostic.candidate_window_begin_index =
        candidate.candidate_window_begin_index;
    diagnostic.candidate_window_end_index =
        candidate.candidate_window_end_index;
    diagnostic.candidate_status = candidate.status;
    diagnostic.execution_candidate_audits =
        candidate.execution_candidate_audits;
    diagnostic.selected_phase_index = candidate.current_index;
    diagnostic.terminal_phase_index = candidate.terminal_index;
    diagnostic.initial_execution = horizon.initial_state;
    diagnostic.initial_progress_s = horizon.initial_progress_s;

    // A pre-solver candidate failure has no phase parameter image or warm
    // start.  Preserve the complete current 22D state and the per-candidate
    // B_exec audit above, then stop before the solver-specific diagnostics.
    if (!candidate.valid || !phase.active) {
        return diagnostic;
    }

    const PreSolveSnapshotDebug& snapshot =
        result.solver_output.pre_solve_snapshot;
    diagnostic.solver_snapshot = snapshot;
    diagnostic.solver_residuals_evaluated =
        snapshot.solver_residuals_evaluated;
    diagnostic.solver_nlp_status = snapshot.solver_nlp_status;
    diagnostic.solver_qp_status = snapshot.solver_qp_status;
    diagnostic.stationarity_residual = snapshot.stationarity_residual;
    diagnostic.equality_residual = snapshot.equality_residual;
    diagnostic.inequality_residual = snapshot.inequality_residual;
    diagnostic.complementarity_residual =
        snapshot.complementarity_residual;

    const DelayAugmentedPhaseCompiledContract compiled =
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    DelayAugmentedPhaseDynamics dynamics;
    std::string error;
    if (!dynamics.configure(compiled.execution, compiled.slosh, error)) {
        diagnostic.warm_start_rollout_status =
            "CONFIGURATION_FAILED_" + error;
        return diagnostic;
    }
    std::vector<DelayAugmentedPhaseControl> controls;
    controls.reserve(static_cast<std::size_t>(phase.horizon_steps));
    for (int stage = 0; stage < phase.horizon_steps; ++stage) {
        const PhaseNominalStage& nominal =
            phase.stages[static_cast<std::size_t>(stage)];
        DelayAugmentedPhaseControl control;
        control.acceleration = nominal.a;
        control.angular_acceleration = nominal.alpha;
        control.progress_rate = nominal.v_s;
        controls.push_back(control);
    }
    const DelayAugmentedPhaseRolloutResult warm_start =
        dynamics.rollout(horizon, controls);
    diagnostic.warm_start_rollout_valid = warm_start.valid;
    diagnostic.warm_start_rollout_status = warm_start.status;
    if (!warm_start.valid || warm_start.states.size() !=
            static_cast<std::size_t>(phase.horizon_steps + 1) ||
        warm_start.published_commands.size() != controls.size()) {
        return diagnostic;
    }

    const DelayAugmentedPhaseState& terminal_state =
        warm_start.states.back();
    diagnostic.warm_start_terminal_execution = terminal_state.execution;
    diagnostic.warm_start_terminal_progress_s = terminal_state.progress_s;

    ExecutionCompatibilityGate execution_gate;
    for (int stage = 0; stage <= phase.horizon_steps; ++stage) {
        const PhaseNominalStage& nominal =
            phase.stages[static_cast<std::size_t>(stage)];
        const ExecutionCompatibilityGateResult compatibility =
            execution_gate.evaluate(
                nominal.augmented_execution,
                nominal.execution_bounds,
                warm_start.states[static_cast<std::size_t>(stage)].execution);
        if (compatibility.valid &&
            compatibility.max_normalized_error >=
                diagnostic.max_stage_execution_normalized_error) {
            diagnostic.max_stage_execution_normalized_error =
                compatibility.max_normalized_error;
            diagnostic.worst_execution_stage = stage;
        }
        if (stage == phase.horizon_steps) {
            diagnostic.terminal_execution_gate_valid = compatibility.valid;
            diagnostic.terminal_execution_normalized_error =
                compatibility.max_normalized_error;
            diagnostic.terminal_execution_margin =
                1.0 - compatibility.max_normalized_error;
            continue;
        }

        const DelayAugmentedPhaseControl& control =
            controls[static_cast<std::size_t>(stage)];
        const VelocityCommand& published =
            warm_start.published_commands[static_cast<std::size_t>(stage)];
        diagnostic.min_linear_residual_margin = std::min(
            diagnostic.min_linear_residual_margin,
            phase.max_residual_v -
                std::abs(published.linear - nominal.u_pub_v));
        diagnostic.min_angular_residual_margin = std::min(
            diagnostic.min_angular_residual_margin,
            phase.max_residual_omega -
                std::abs(published.angular - nominal.u_pub_omega));
        diagnostic.min_linear_output_margin = std::min(
            diagnostic.min_linear_output_margin,
            std::min(
                published.linear - manifest::kLinearOutputMin,
                manifest::kLinearOutputMax - published.linear));
        diagnostic.min_angular_output_margin = std::min(
            diagnostic.min_angular_output_margin,
            std::min(
                published.angular - manifest::kAngularOutputMin,
                manifest::kAngularOutputMax - published.angular));
        diagnostic.min_acceleration_margin = std::min(
            diagnostic.min_acceleration_margin,
            manifest::kAccelerationMax - std::abs(control.acceleration));
        diagnostic.min_angular_acceleration_margin = std::min(
            diagnostic.min_angular_acceleration_margin,
            manifest::kAngularAccelerationMax -
                std::abs(control.angular_acceleration));
        diagnostic.min_progress_rate_margin = std::min(
            diagnostic.min_progress_rate_margin,
            std::min(control.progress_rate,
                     manifest::kProgressRateMax - control.progress_rate));
    }

    const PhaseNominalSample* terminal =
        artifact.sample(candidate.terminal_index);
    if (terminal != nullptr) {
        const EmpiricalRecoveryGateResult gate =
            EmpiricalRecoveryGate{}.evaluate(
                *terminal,
                terminal_state.execution.robot,
                terminal_state.execution.slosh);
        diagnostic.terminal_empirical_gate_valid = gate.valid;
        diagnostic.terminal_empirical_metric = gate.metric;
        diagnostic.terminal_empirical_margin = 1.0 - gate.metric;
    }
    return diagnostic;
}

bool openCycleCsv(const std::string& path, std::ofstream& output,
                  std::string& error) {
    output.open(path.c_str(), std::ios::out | std::ios::trunc);
    output.imbue(std::locale::classic());
    output << std::setprecision(17);
    if (!output) {
        error = "cannot open cycle CSV " + path;
        return false;
    }
    output
        << "schema,cycle_id,time_sec,window,x,y,yaw,v,omega,ax,ay,"
        << "tracking_error_m,progress_s,true_height_m,measured_height_m,"
        << "observer_height_m,observer_eta_x,observer_eta_x_dot,"
        << "observer_eta_y,observer_eta_y_dot,solver_success,"
        << "raw_solver_status,phase_status,final_status,"
        << "gate_evaluated,terminal_gate_accepted,"
        << "current_execution_compatible,terminal_execution_compatible,"
        << "recovery_used,controlled_stop_used,selected_phase_valid,"
        << "clock_index,candidate_window_begin_index,"
        << "candidate_window_end_index,selected_phase_index,phase_lead_steps,"
        << "execution_candidate_filter_applied,"
        << "execution_rejected_candidate_count,"
        << "selected_execution_max_normalized_error,"
        << "acados_solve_time_ms,backend_wall_time_ms,"
        << "final_cmd_v,final_cmd_omega,command_source,publish_stamp_ns,"
        << "plant_publish_time_sec,linear_effective_time_sec,"
        << "angular_effective_time_sec\n";
    return true;
}

bool writeCycle(std::ofstream& output, const CycleRecord& record) {
    output
        << kCycleSchema << ',' << record.cycle_id << ','
        << record.time_sec << ',' << record.window << ','
        << record.plant.x << ',' << record.plant.y << ','
        << record.plant.yaw << ',' << record.plant.v << ','
        << record.plant.omega << ',' << record.plant.acceleration << ','
        << record.plant.lateral_acceleration << ','
        << record.tracking_error_m << ',' << record.progress_s << ','
        << record.plant.true_height_m << ','
        << record.plant.measured_height_m << ','
        << record.observer_height_m << ','
        << record.observer.eta_x << ',' << record.observer.eta_x_dot << ','
        << record.observer.eta_y << ',' << record.observer.eta_y_dot << ','
        << (record.solver_success ? "true" : "false") << ','
        << csvEscape(record.raw_solver_status) << ','
        << csvEscape(record.phase_status) << ','
        << csvEscape(record.final_status) << ','
        << (record.gate_evaluated ? "true" : "false") << ','
        << (record.terminal_gate_accepted ? "true" : "false") << ','
        << (record.current_execution_compatible ? "true" : "false") << ','
        << (record.terminal_execution_compatible ? "true" : "false") << ','
        << (record.recovery_used ? "true" : "false") << ','
        << (record.controlled_stop_used ? "true" : "false") << ','
        << (record.selected_phase_valid ? "true" : "false") << ','
        << record.clock_index << ','
        << record.candidate_window_begin_index << ','
        << record.candidate_window_end_index << ','
        << record.selected_phase_index << ','
        << record.phase_lead_steps << ','
        << (record.execution_candidate_filter_applied ? "true" : "false")
        << ',' << record.execution_rejected_candidate_count << ','
        << record.selected_execution_max_normalized_error << ','
        << record.acados_solve_time_ms << ','
        << record.backend_wall_time_ms << ','
        << record.final_command.linear << ','
        << record.final_command.angular << ','
        << commandSourceName(record.command_source) << ','
        << record.publish_stamp_ns << ','
        << record.plant_publish.publish_time_sec << ','
        << record.plant_publish.linear_effective_time_sec << ','
        << record.plant_publish.angular_effective_time_sec << '\n';
    output.flush();
    return static_cast<bool>(output);
}

bool writeSummary(
    const Arguments& args,
    const ConditionConfig& condition,
    const IndependentPlantConfig& plant_config,
    const NominalSequenceArtifact& artifact,
    const TrialCounters& counters,
    const SolverFailureDiagnostic& solver_failure,
    const std::vector<double>& measured_heights,
    const std::vector<double>& true_heights,
    const std::vector<double>& observer_heights,
    const std::vector<double>& tracking_errors,
    bool sequence_completed,
    bool task_success,
    const std::string& completion_reason,
    double motion_end_sec,
    double final_goal_error_m,
    const ZvdAudit& zvd,
    const std::string& runtime_error,
    std::string& error) {
    error.clear();
    std::ofstream output(
        args.summary_json_path.c_str(), std::ios::out | std::ios::trunc);
    output.imbue(std::locale::classic());
    output << std::setprecision(17);
    if (!output) {
        error = "cannot open summary JSON " + args.summary_json_path;
        return false;
    }
    const bool runtime_ok = runtime_error.empty();
    const std::string status = !runtime_ok
        ? "RUNTIME_ERROR"
        : (task_success ? "TRIAL_COMPLETE" : "TRIAL_COMPLETE_WITH_FAILURE");
    const double backend_budget_ms = 1000.0 / condition.control_rate_hz;
    const std::size_t backend_deadline_misses =
        static_cast<std::size_t>(std::count_if(
            counters.backend_wall_times_ms.begin(),
            counters.backend_wall_times_ms.end(),
            [backend_budget_ms](double value) {
                return value > backend_budget_ms;
            }));
    const double max_backend_wall_time_ms =
        counters.backend_wall_times_ms.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : *std::max_element(
                  counters.backend_wall_times_ms.begin(),
                  counters.backend_wall_times_ms.end());
    const bool kkt_contract_passed =
        !counters.backend_wall_times_ms.empty() &&
        counters.kkt_residual_samples ==
            counters.backend_wall_times_ms.size() &&
        counters.max_stationarity_residual <=
            manifest::kMaxStationarityResidual &&
        counters.max_equality_residual <=
            manifest::kMaxEqualityResidual &&
        counters.max_inequality_residual <=
            manifest::kMaxInequalityResidual &&
        counters.max_complementarity_residual <=
            manifest::kMaxComplementarityResidual;
    output << "{\n"
        << "  \"schema\": \"" << kSummarySchema << "\",\n"
        << "  \"status\": \"" << status << "\",\n"
        << "  \"implementation_complete\": true,\n"
        << "  \"condition_id\": \""
        << jsonEscape(condition.condition_id) << "\",\n"
        << "  \"mode\": \"" << modeName(condition.mode) << "\",\n"
        << "  \"implementation_id\": \""
        << jsonEscape(condition.implementation_id) << "\",\n"
        << "  \"seed\": " << args.seed << ",\n"
        << "  \"simulation_only\": true,\n"
        << "  \"formal_trials_started\": false,\n"
        << "  \"development_pilot_only\": true,\n"
        << "  \"formal_robot_release\": false,\n"
        << "  \"physical_parameter_claim\": false,\n"
        << "  \"preliminary_planar_r03_parameters_only\": true,\n"
        << "  \"plant_truth_visible_to_controller\": false,\n"
        << "  \"external_liquid_truth_used_for_control\": false,\n"
        << "  \"controller_observer_source\": "
        << "\"plant_motion_derived_odom\",\n"
        << "  \"final_command_transaction\": true,\n"
        << "  \"command_history_source\": "
        << "\"final_published_command\",\n"
        << "  \"dual_channel_execution_model\": true,\n"
        << "  \"plant_controller_parameter_independence\": true,\n"
        << "  \"plant_freeze_id\": \""
        << jsonEscape(plant_config.freeze_id) << "\",\n"
        << "  \"artifact_contract_id\": \""
        << jsonEscape(artifact.metadata().contract_id) << "\",\n"
        << "  \"trial\": {\n"
        << "    \"control_rate_hz\": " << condition.control_rate_hz << ",\n"
        << "    \"motion_end_sec\": " << motion_end_sec << ",\n"
        << "    \"fixed_tail_sec\": " << condition.fixed_tail_sec << ",\n"
        << "    \"publish_latency_sec\": "
        << condition.publish_latency_sec << ",\n"
        << "    \"sequence_completed\": "
        << (sequence_completed ? "true" : "false") << ",\n"
        << "    \"task_success\": " << (task_success ? "true" : "false")
        << ",\n"
        << "    \"failures_included_in_primary_metric\": true,\n"
        << "    \"completion_reason\": \""
        << jsonEscape(completion_reason) << "\",\n"
        << "    \"runtime_error\": \""
        << jsonEscape(runtime_error) << "\"\n"
        << "  },\n"
        << "  \"primary_metric\": {\n"
        << "    \"name\": \"external_measured_height_q95_m\",\n"
        << "    \"window\": \"motion_plus_fixed_tail\",\n"
        << "    \"statistics_unit\": \"complete_trial\",\n"
        << "    \"quantile_method\": \"nearest_rank\",\n"
        << "    \"sample_count\": " << measured_heights.size() << ",\n"
        << "    \"value_m\": "
        << jsonNumber(nearestRankQuantile(measured_heights, 0.95)) << "\n"
        << "  },\n"
        << "  \"secondary_metrics\": {\n"
        << "    \"external_true_height_q95_m\": "
        << jsonNumber(nearestRankQuantile(true_heights, 0.95)) << ",\n"
        << "    \"internal_observer_height_q95_m\": "
        << jsonNumber(nearestRankQuantile(observer_heights, 0.95)) << ",\n"
        << "    \"tracking_rms_m\": "
        << jsonNumber(rms(tracking_errors)) << ",\n"
        << "    \"tracking_q95_m\": "
        << jsonNumber(nearestRankQuantile(tracking_errors, 0.95)) << ",\n"
        << "    \"final_goal_error_m\": "
        << jsonNumber(final_goal_error_m) << "\n"
        << "  },\n"
        << "  \"solver_runtime\": {\n"
        << "    \"statistics_unit\": \"optimizer_invocation\",\n"
        << "    \"backend\": \""
        << (counters.solver_id.empty()
                ? "NOT_RECORDED"
                : kSolverBackendDelayAugmentedPhaseAcados)
        << "\",\n"
        << "    \"solver_id\": \""
        << jsonEscape(counters.solver_id) << "\",\n"
        << "    \"nlp_solver_type\": \""
        << jsonEscape(counters.nlp_solver_type) << "\",\n"
        << "    \"solver_config_hash\": \""
        << jsonEscape(counters.solver_config_hash) << "\",\n"
        << "    \"sample_count\": "
        << counters.backend_wall_times_ms.size() << ",\n"
        << "    \"acados_solve_p50_ms\": "
        << jsonNumber(nearestRankQuantile(
               counters.acados_solve_times_ms, 0.50)) << ",\n"
        << "    \"backend_wall_p50_ms\": "
        << jsonNumber(nearestRankQuantile(
               counters.backend_wall_times_ms, 0.50)) << ",\n"
        << "    \"backend_wall_p95_ms\": "
        << jsonNumber(nearestRankQuantile(
               counters.backend_wall_times_ms, 0.95)) << ",\n"
        << "    \"backend_wall_p99_ms\": "
        << jsonNumber(nearestRankQuantile(
               counters.backend_wall_times_ms, 0.99)) << ",\n"
        << "    \"backend_wall_max_ms\": "
        << jsonNumber(max_backend_wall_time_ms) << ",\n"
        << "    \"control_budget_ms\": "
        << backend_budget_ms << ",\n"
        << "    \"deadline_misses\": "
        << backend_deadline_misses << ",\n"
        << "    \"kkt_residual_sample_count\": "
        << counters.kkt_residual_samples << ",\n"
        << "    \"kkt_contract_passed\": "
        << (kkt_contract_passed ? "true" : "false") << ",\n"
        << "    \"kkt_contract_tolerance\": "
        << manifest::kMaxStationarityResidual << ",\n"
        << "    \"max_stationarity_residual\": "
        << counters.max_stationarity_residual << ",\n"
        << "    \"max_equality_residual\": "
        << counters.max_equality_residual << ",\n"
        << "    \"max_inequality_residual\": "
        << counters.max_inequality_residual << ",\n"
        << "    \"max_complementarity_residual\": "
        << counters.max_complementarity_residual << ",\n"
        << "    \"max_sqp_iterations\": "
        << counters.max_sqp_iterations << ",\n"
        << "    \"max_qp_iterations\": "
        << counters.max_qp_iterations << "\n"
        << "  },\n"
        << "  \"controller_audit\": {\n"
        << "    \"solver_failures\": " << counters.solver_failures << ",\n"
        << "    \"gate_evaluations\": " << counters.gate_evaluations << ",\n"
        << "    \"terminal_gate_accepts\": "
        << counters.terminal_gate_accepts << ",\n"
        << "    \"recovery_actions\": " << counters.recovery_actions << ",\n"
        << "    \"controlled_stops\": " << counters.controlled_stops << ",\n"
        << "    \"publications\": " << counters.publications << ",\n"
        << "    \"publication_failures\": "
        << counters.publication_failures << ",\n"
        << "    \"execution_candidate_filter_cycles\": "
        << counters.execution_candidate_filter_cycles << ",\n"
        << "    \"execution_rejected_candidates\": "
        << counters.execution_rejected_candidates << ",\n"
        << "    \"max_selected_execution_normalized_error\": "
        << counters.max_selected_execution_normalized_error << ",\n"
        << "    \"raw_solver_status_counts\": ";
    writeJsonStringCounts(output, counters.raw_solver_status_counts);
    output << ",\n    \"phase_status_counts\": ";
    writeJsonStringCounts(output, counters.phase_status_counts);
    output << ",\n    \"final_status_counts\": ";
    writeJsonStringCounts(output, counters.final_status_counts);
    output << "\n"
        << "  },\n"
        << "  \"first_solver_failure_diagnostic\": ";
    if (!solver_failure.valid) {
        output << "null,\n";
    } else {
        output << "{\n"
            << "    \"cycle_id\": " << solver_failure.cycle_id << ",\n"
            << "    \"raw_solver_status\": \""
            << jsonEscape(solver_failure.raw_solver_status) << "\",\n"
            << "    \"clock_index\": " << solver_failure.clock_index
            << ",\n"
            << "    \"candidate_window\": ["
            << solver_failure.candidate_window_begin_index << ", "
            << solver_failure.candidate_window_end_index << "],\n"
            << "    \"candidate_status\": \""
            << jsonEscape(solver_failure.candidate_status) << "\",\n"
            << "    \"execution_candidate_audits\": [";
        for (std::size_t index = 0;
             index < solver_failure.execution_candidate_audits.size();
             ++index) {
            if (index != 0) output << ',';
            const auto& audit =
                solver_failure.execution_candidate_audits[index];
            output << "\n      {\"phase_index\": " << audit.phase_index
                << ", \"valid\": " << (audit.valid ? "true" : "false")
                << ", \"accepted\": "
                << (audit.accepted ? "true" : "false")
                << ", \"max_normalized_error\": "
                << jsonNumber(audit.max_normalized_error)
                << ", \"max_error_name\": \""
                << jsonEscape(audit.max_error_name)
                << "\", \"max_error_index\": " << audit.max_error_index
                << ", \"actual\": " << jsonNumber(audit.actual)
                << ", \"nominal\": " << jsonNumber(audit.nominal)
                << ", \"bound\": " << jsonNumber(audit.bound)
                << ", \"status\": \"" << jsonEscape(audit.status)
                << "\"}";
        }
        if (!solver_failure.execution_candidate_audits.empty()) {
            output << '\n' << "    ";
        }
        output << "],\n"
            << "    \"selected_phase_index\": "
            << solver_failure.selected_phase_index << ",\n"
            << "    \"terminal_phase_index\": "
            << solver_failure.terminal_phase_index << ",\n"
            << "    \"state_order\": [\"x\", \"y\", \"yaw\", "
            << "\"v\", \"s\", \"omega\", \"eta_x\", "
            << "\"eta_x_dot\", \"eta_y\", \"eta_y_dot\", "
            << "\"linear_q0\", \"linear_q1\", \"linear_q2\", "
            << "\"linear_q3\", \"linear_q4\", \"angular_q0\", "
            << "\"angular_q1\", \"angular_q2\", \"angular_q3\", "
            << "\"angular_q4\", \"angular_q5\", \"angular_q6\"],\n"
            << "    \"initial_state_22d\": ";
        writeJsonNumberArray(output, solverState22(
            solver_failure.initial_execution,
            solver_failure.initial_progress_s));
        output << ",\n"
            << "    \"warm_start_rollout_valid\": "
            << (solver_failure.warm_start_rollout_valid
                    ? "true" : "false") << ",\n"
            << "    \"warm_start_rollout_status\": \""
            << jsonEscape(solver_failure.warm_start_rollout_status)
            << "\",\n"
            << "    \"warm_start_terminal_state_22d\": ";
        writeJsonNumberArray(output, solverState22(
            solver_failure.warm_start_terminal_execution,
            solver_failure.warm_start_terminal_progress_s));
        output << ",\n"
            << "    \"stage_constraint_margins\": {\n"
            << "      \"worst_execution_stage\": "
            << solver_failure.worst_execution_stage << ",\n"
            << "      \"max_execution_normalized_error\": "
            << jsonNumber(
                   solver_failure.max_stage_execution_normalized_error)
            << ",\n"
            << "      \"execution_margin\": "
            << jsonNumber(
                   1.0 - solver_failure
                       .max_stage_execution_normalized_error) << ",\n"
            << "      \"min_linear_residual_margin\": "
            << jsonNumber(solver_failure.min_linear_residual_margin)
            << ",\n"
            << "      \"min_angular_residual_margin\": "
            << jsonNumber(solver_failure.min_angular_residual_margin)
            << ",\n"
            << "      \"min_linear_output_margin\": "
            << jsonNumber(solver_failure.min_linear_output_margin)
            << ",\n"
            << "      \"min_angular_output_margin\": "
            << jsonNumber(solver_failure.min_angular_output_margin)
            << ",\n"
            << "      \"min_acceleration_margin\": "
            << jsonNumber(solver_failure.min_acceleration_margin)
            << ",\n"
            << "      \"min_angular_acceleration_margin\": "
            << jsonNumber(
                   solver_failure.min_angular_acceleration_margin)
            << ",\n"
            << "      \"min_progress_rate_margin\": "
            << jsonNumber(solver_failure.min_progress_rate_margin)
            << "\n"
            << "    },\n"
            << "    \"terminal_constraint_margins\": {\n"
            << "      \"empirical_gate_valid\": "
            << (solver_failure.terminal_empirical_gate_valid
                    ? "true" : "false") << ",\n"
            << "      \"empirical_metric\": "
            << jsonNumber(solver_failure.terminal_empirical_metric)
            << ",\n"
            << "      \"empirical_margin\": "
            << jsonNumber(solver_failure.terminal_empirical_margin)
            << ",\n"
            << "      \"execution_gate_valid\": "
            << (solver_failure.terminal_execution_gate_valid
                    ? "true" : "false") << ",\n"
            << "      \"execution_normalized_error\": "
            << jsonNumber(
                   solver_failure.terminal_execution_normalized_error)
            << ",\n"
            << "      \"execution_margin\": "
            << jsonNumber(solver_failure.terminal_execution_margin)
            << "\n"
            << "    },\n"
            << "    \"solver_residuals\": {\n"
            << "      \"evaluated\": "
            << (solver_failure.solver_residuals_evaluated
                    ? "true" : "false") << ",\n"
            << "      \"nlp_status\": "
            << solver_failure.solver_nlp_status << ",\n"
            << "      \"qp_status\": "
            << solver_failure.solver_qp_status << ",\n"
            << "      \"stationarity\": "
            << jsonNumber(solver_failure.stationarity_residual) << ",\n"
            << "      \"equality\": "
            << jsonNumber(solver_failure.equality_residual) << ",\n"
            << "      \"inequality\": "
            << jsonNumber(solver_failure.inequality_residual) << ",\n"
            << "      \"complementarity\": "
            << jsonNumber(solver_failure.complementarity_residual) << "\n"
            << "    },\n"
            << "    \"solver_backend_contract\": {\n"
            << "      \"solver_id\": \""
            << jsonEscape(solver_failure.solver_snapshot.solver_id)
            << "\",\n"
            << "      \"nlp_solver_type\": \""
            << jsonEscape(
                   solver_failure.solver_snapshot.nlp_solver_type)
            << "\",\n"
            << "      \"solver_config_hash\": \""
            << jsonEscape(
                   solver_failure.solver_snapshot.solver_config_hash)
            << "\",\n"
            << "      \"sqp_iterations\": "
            << solver_failure.solver_snapshot.solver_sqp_iterations
            << ",\n"
            << "      \"qp_iterations\": "
            << solver_failure.solver_snapshot.solver_qp_iterations
            << ",\n"
            << "      \"step_length\": "
            << jsonNumber(
                   solver_failure.solver_snapshot.solver_step_length)
            << ",\n"
            << "      \"cost\": "
            << jsonNumber(solver_failure.solver_snapshot.solver_cost)
            << ",\n"
            << "      \"acados_solve_time_ms\": "
            << jsonNumber(
                   solver_failure.solver_snapshot.acados_solve_time_ms)
            << ",\n"
            << "      \"backend_wall_time_ms\": "
            << jsonNumber(
                   solver_failure.solver_snapshot.backend_wall_time_ms)
            << "\n"
            << "    },\n"
            << "    \"warm_start_residuals\": {\n"
            << "      \"evaluated\": "
            << (solver_failure.solver_snapshot.warm_start_residuals.evaluated
                    ? "true" : "false") << ",\n"
            << "      \"stationarity\": "
            << jsonNumber(solver_failure.solver_snapshot
                              .warm_start_residuals.stationarity)
            << ",\n"
            << "      \"equality\": "
            << jsonNumber(solver_failure.solver_snapshot
                              .warm_start_residuals.equality)
            << ",\n"
            << "      \"inequality\": "
            << jsonNumber(solver_failure.solver_snapshot
                              .warm_start_residuals.inequality)
            << ",\n"
            << "      \"complementarity\": "
            << jsonNumber(solver_failure.solver_snapshot
                              .warm_start_residuals.complementarity)
            << "\n"
            << "    },\n"
            << "    \"solver_iterations\": ";
        writeIterationDiagnostics(
            output, solver_failure.solver_snapshot.solver_iterations);
        output << ",\n"
            << "    \"parameter_width\": "
            << solver_failure.solver_snapshot.parameter_width << ",\n"
            << "    \"stage_parameters\": ";
        writeJsonNumberArray(
            output, solver_failure.solver_snapshot.stage_parameters);
        output << ",\n"
            << "    \"failed_raw_solution_states\": ";
        writeJsonNumberArray(
            output,
            solver_failure.solver_snapshot.failed_raw_solution_states);
        output << ",\n"
            << "    \"failed_raw_solution_controls\": ";
        writeJsonNumberArray(
            output,
            solver_failure.solver_snapshot.failed_raw_solution_controls);
        output << ",\n"
            << "    \"warm_start_constraint_audit\": ";
        writeConstraintAudit(
            output,
            solver_failure.solver_snapshot.warm_start_constraint_audit);
        output << ",\n"
            << "    \"solution_constraint_audit\": ";
        writeConstraintAudit(
            output,
            solver_failure.solver_snapshot.solution_constraint_audit);
        output << "\n"
            << "  },\n";
    }
    output << "  \"baseline_contract\": {\n"
        << "    \"pilot_only\": "
        << (condition.pilot_only ? "true" : "false")
        << ",\n"
        << "    \"c1_pilot_tuned_and_frozen\": "
        << (condition.pilot_tuned_and_frozen ? "true" : "false") << ",\n"
        << "    \"formal_c3_c4_causal_comparison_ready\": "
        << (condition.formal_c3_c4_causal_comparison_ready
                ? "true" : "false") << ",\n"
        << "    \"c3_gate_disabled_by_separate_controller\": "
        << "false"
        << ",\n"
        << "    \"c3_does_not_widen_gate_radii\": true,\n"
        << "    \"c3_empirical_gate_monitor_only\": "
        << (condition.mode == TrialMode::ResidualNoGate
                ? "true" : "false") << ",\n"
        << "    \"c3_exact_c4_optimizer_match\": "
        << ((condition.mode == TrialMode::ResidualNoGate ||
             condition.mode == TrialMode::PhaseRejoinFull)
                ? "true" : "false") << ",\n"
        << "    \"is_zvd_delay_steps\": ["
        << zvd.delay_steps[0] << ", " << zvd.delay_steps[1] << ", "
        << zvd.delay_steps[2] << "],\n"
        << "    \"is_zvd_equivalent_delay_sec\": ["
        << zvd.equivalent_delay_sec[0] << ", "
        << zvd.equivalent_delay_sec[1] << ", "
        << zvd.equivalent_delay_sec[2] << "],\n"
        << "    \"is_zvd_discrete_single_mode_residual\": "
        << zvd.discrete_residual << ",\n"
        << "    \"is_zvd_max_discrete_residual\": "
        << condition.zvd_max_discrete_residual << ",\n"
        << "    \"is_zvd_single_mode_self_test_passed\": "
        << ((condition.mode != TrialMode::InputShaping ||
             (zvd.valid && zvd.discrete_residual <=
                  condition.zvd_max_discrete_residual))
                ? "true" : "false") << "\n"
        << "  }\n"
        << "}\n";
    output.flush();
    if (!output) {
        error = "cannot write summary JSON";
        return false;
    }
    return true;
}

int runPhaseRejoinClosedLoopTrial(int argc, char** argv) {
    Arguments requested;
    std::string error;
    if (!parseArguments(argc, argv, requested, error)) {
        if (error != "HELP") std::cerr << "ERROR: " << error << '\n';
        printUsage(error == "HELP" ? std::cout : std::cerr);
        return error == "HELP" ? 0 : kUsageExit;
    }

    Arguments args;
    std::string final_cycle_path;
    std::string final_summary_path;
    if (!validateAndPrepareOutputPaths(
            requested, args, final_cycle_path, final_summary_path, error)) {
        std::cerr << "ERROR: " << error << '\n';
        return kConfigurationExit;
    }
    const auto cleanup_temporary = [&args]() {
        if (!args.cycle_csv_path.empty()) ::unlink(args.cycle_csv_path.c_str());
        if (!args.summary_json_path.empty()) {
            ::unlink(args.summary_json_path.c_str());
        }
    };

    ConditionConfig condition;
    if (!loadCondition(args.condition_path, condition, error)) {
        std::cerr << "ERROR: " << error << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }

    IndependentPlantConfig plant_config;
    if (!loadIndependentPlantConfig(args.plant_path, plant_config, error)) {
        std::cerr << "ERROR: plant configuration rejected: " << error << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }
    if (!plant_config.simulation_only || plant_config.formal_robot_release ||
        plant_config.real_robot_enforce_allowed) {
        std::cerr << "ERROR: trial runner accepts simulation-only Plant "
                  << "configurations and can never release a robot\n";
        cleanup_temporary();
        return kConfigurationExit;
    }
    if (std::abs(plant_config.experiment_control_rate_hz -
                 condition.control_rate_hz) > 1e-12 ||
        std::abs(plant_config.experiment_fixed_tail_sec -
                 condition.fixed_tail_sec) > 1e-12) {
        std::cerr << "ERROR: condition rate/tail differs from Plant experiment "
                  << "contract\n";
        cleanup_temporary();
        return kConfigurationExit;
    }

    PathAsset path;
    if (!loadPath(args.path_path, path, error)) {
        std::cerr << "ERROR: " << error << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }

    NominalSequenceArtifact artifact;
    const NominalArtifactLoadResult artifact_load =
        artifact.loadCsv(args.artifact_path);
    if (!artifact_load.success ||
        artifact.metadata().schema !=
            "phase_rejoin_empirical_augmented_v3" ||
        !artifact.metadata().delay_augmented_nominal) {
        std::cerr << "ERROR: strict V3 artifact rejected: "
                  << artifact_load.status << ": "
                  << artifact_load.detail << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }
    if (condition.mode == TrialMode::PhaseRejoinFull &&
        artifact.metadata().evidence_level !=
            PhaseRejoinEvidenceLevel::EmpiricalHeldOut) {
        std::cerr << "ERROR: C4 requires empirical held-out recovery data\n";
        cleanup_temporary();
        return kConfigurationExit;
    }
    if (condition.mode == TrialMode::OfflineReplay ||
        condition.mode == TrialMode::ResidualNoGate ||
        condition.mode == TrialMode::PhaseRejoinFull) {
        const PhaseNominalSample* final_sample =
            artifact.sample(artifact.size() - 1u);
        if (final_sample == nullptr ||
            !finite(final_sample->t) ||
            condition.max_motion_sec + 1e-9 < final_sample->t) {
            std::cerr << "ERROR: max_motion_sec does not cover the complete "
                      << "offline nominal and recovery tail\n";
            cleanup_temporary();
            return kConfigurationExit;
        }
    }

    const DelayAugmentedPhaseCompiledContract compiled =
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    const double requested_dt = 1.0 / condition.control_rate_hz;
    if (std::abs(compiled.execution.dt - requested_dt) > 1e-8) {
        std::cerr << "ERROR: controller period differs from compiled execution "
                  << "contract\n";
        cleanup_temporary();
        return kConfigurationExit;
    }
    // The compiled contract is the authoritative discrete clock.  Its decimal
    // serialization differs from mathematical 1/30 by about 3e-11 s.
    const double dt = compiled.execution.dt;
    const ZvdAudit zvd = makeZvdAudit(compiled.slosh);
    if (condition.mode == TrialMode::InputShaping &&
        (!zvd.valid ||
         zvd.discrete_residual > condition.zvd_max_discrete_residual)) {
        std::cerr << "ERROR: quantized 30 Hz ZVD self-test failed residual="
                  << zvd.discrete_residual << " threshold="
                  << condition.zvd_max_discrete_residual << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }

    IndependentScoutLiquidPlant plant;
    if (!plant.configure(plant_config, error) ||
        !plant.reset(args.seed, path.initial_pose, error)) {
        std::cerr << "ERROR: independent Plant initialization failed: "
                  << error << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }

    const VariantConfig variant = controllerVariant(condition.mode, condition);
    SolverParams solver = commonSolverParams();
    std::unique_ptr<SpmpcProblem> production_problem;
    std::unique_ptr<OfflineReplaySession> replay_session;
    std::unique_ptr<ZvdInputShapingSession> shaping_session;
    SolverSession* solver_session = nullptr;

    if (condition.mode == TrialMode::OrdinaryMpcc ||
        condition.mode == TrialMode::SmoothMatchMpcc) {
        solver.solver_backend = kSolverBackendContinuousMpccAcados;
        production_problem.reset(new SpmpcProblem());
        const SolverConfigureResult configured =
            production_problem->configure(solver, variant);
        if (!configured.success) {
            std::cerr << "ERROR: continuous MPCC unavailable: "
                      << configured.status << ": " << configured.detail
                      << '\n';
            cleanup_temporary();
            return kConfigurationExit;
        }
        production_problem->setReferencePath(path.reference);
        solver_session = production_problem.get();
    } else if (condition.mode == TrialMode::OfflineReplay) {
        replay_session.reset(new OfflineReplaySession(artifact));
        solver_session = replay_session.get();
    } else if (condition.mode == TrialMode::ResidualNoGate ||
               condition.mode == TrialMode::PhaseRejoinFull) {
        if (!DelayAugmentedPhaseAcadosSolver::compiled()) {
            std::cerr << "ERROR: C3/C4 require the explicitly enabled 22D "
                      << "delay-augmented capsule\n";
            cleanup_temporary();
            return kConfigurationExit;
        }
        solver = delayAugmentedSolverParams(artifact);
        production_problem.reset(new SpmpcProblem());
        const SolverConfigureResult configured =
            production_problem->configure(solver, variant);
        if (!configured.success) {
            std::cerr << "ERROR: C3/C4 solver rejected: "
                      << configured.status
                      << ": " << configured.detail << '\n';
            cleanup_temporary();
            return kConfigurationExit;
        }
        production_problem->setReferencePath(path.reference);
        solver_session = production_problem.get();
    } else if (condition.mode == TrialMode::InputShaping) {
        solver.solver_backend = kSolverBackendContinuousMpccAcados;
        shaping_session.reset(new ZvdInputShapingSession());
        const SolverConfigureResult configured = shaping_session->configure(
            solver, variant, path.reference);
        if (!configured.success) {
            std::cerr << "ERROR: input-shaping baseline rejected: "
                      << configured.status << ": " << configured.detail
                      << '\n';
            cleanup_temporary();
            return kConfigurationExit;
        }
        solver_session = shaping_session.get();
    }
    if (solver_session == nullptr) {
        std::cerr << "ERROR: no solver session bound\n";
        cleanup_temporary();
        return kConfigurationExit;
    }

    ControlCycleEngine engine(*solver_session);
    if (!engine.configureCommandPipeline(
            commandPipelineConfig(condition.control_rate_hz), error) ||
        !engine.configureSafety(safetyConfig(dt), error)) {
        std::cerr << "ERROR: controller configuration failed: " << error
                  << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }
    PublishLatencyModelConfig publish_latency;
    publish_latency.enabled = true;
    publish_latency.estimated_dc_sec = condition.publish_latency_sec;
    if (!engine.configurePublishLatency(publish_latency, error)) {
        std::cerr << "ERROR: publish latency configuration failed: "
                  << error << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }
    PhaseRejoinParams phase_params;
    phase_params.mode = PhaseRejoinMode::Off;
    if (condition.mode == TrialMode::ResidualNoGate ||
        condition.mode == TrialMode::PhaseRejoinFull) {
        phase_params.mode = PhaseRejoinMode::Enforce;
        phase_params.empirical_gate_enforced =
            condition.mode == TrialMode::PhaseRejoinFull;
        phase_params.liquid_horizon_steps = compiled.liquid_horizon_steps;
        phase_params.max_residual_v = condition.max_residual_v;
        phase_params.max_residual_omega = condition.max_residual_omega;
        phase_params.required_contract_id = artifact.metadata().contract_id;
        phase_params.required_frame_id = artifact.metadata().frame_id;
        phase_params.allow_development_artifact_in_enforce = false;
    }
    if (!engine.configurePhaseRejoin(phase_params, error)) {
        std::cerr << "ERROR: phase controller configuration failed: "
                  << error << '\n';
        cleanup_temporary();
        return kConfigurationExit;
    }
    if (condition.mode == TrialMode::ResidualNoGate ||
        condition.mode == TrialMode::PhaseRejoinFull) {
        const NominalArtifactLoadResult loaded =
            engine.loadPhaseRejoinArtifact(args.artifact_path);
        if (!loaded.success || !engine.validatePhaseRejoinRuntimeContract(
                phaseRuntimeContract(solver, variant, artifact),
                path.reference, error)) {
            std::cerr << "ERROR: C3/C4 runtime/artifact contract rejected: "
                      << (loaded.success ? error : loaded.status) << '\n';
            cleanup_temporary();
            return kConfigurationExit;
        }
    }

    ExecutionHorizonContextBuilder execution_builder;
    const bool execution_horizon_required =
        condition.mode == TrialMode::ResidualNoGate ||
        condition.mode == TrialMode::PhaseRejoinFull;
    if (execution_horizon_required) {
        ExecutionHorizonBuilderConfig builder_config;
        builder_config.command_timeout_sec = 0.5;
        builder_config.max_alignment_sec = 0.5;
        builder_config.max_integration_step_sec = 0.01;
        builder_config.min_integration_step_sec = 0.0001;
        if (!execution_builder.configure(
                compiled.execution, compiled.slosh, builder_config, error)) {
            std::cerr << "ERROR: execution-horizon builder rejected: "
                      << error << '\n';
            cleanup_temporary();
            return kConfigurationExit;
        }
    }

    SloshDynamics observer_dynamics;
    if (!observer_dynamics.configure(compiled.slosh)) {
        std::cerr << "ERROR: internal liquid observer model rejected\n";
        cleanup_temporary();
        return kConfigurationExit;
    }
    SloshState observer;
    IndependentPlantState previous_plant = plant.state();
    CommandHistoryBuffer command_history;
    prefillZeroHistory(command_history, dt);
    PlantCommandSink sink(plant);
    std::ofstream cycle_output;
    if (!openCycleCsv(args.cycle_csv_path, cycle_output, error)) {
        std::cerr << "ERROR: " << error << '\n';
        cleanup_temporary();
        return kRuntimeExit;
    }

    std::vector<double> measured_heights;
    std::vector<double> true_heights;
    std::vector<double> observer_heights;
    std::vector<double> tracking_errors;
    TrialCounters counters;
    ProgressProjector projector;
    bool completed = false;
    std::string completion_reason = "MOTION_TIMEOUT";
    std::string runtime_error;
    SolverFailureDiagnostic first_solver_failure;
    double last_motion_time_sec = 0.0;
    std::uint64_t next_cycle_id = 1;

    const auto updateObserverAfterAdvance = [&]() -> bool {
        const IndependentPlantState current = plant.state();
        const double elapsed = current.time_sec - previous_plant.time_sec;
        if (elapsed > 1e-9) {
            const double ax = (current.v - previous_plant.v) / elapsed;
            const double ay = current.v * current.omega;
            SloshState next;
            if (!observer_dynamics.stepWithDt(
                    observer, ax, ay, current.omega, elapsed, next)) {
                return false;
            }
            observer = next;
        }
        previous_plant = current;
        return true;
    };

    const auto appendRecord = [&](const CycleRecord& record,
                                  bool motion) -> bool {
        if (!writeCycle(cycle_output, record)) return false;
        measured_heights.push_back(record.plant.measured_height_m);
        true_heights.push_back(record.plant.true_height_m);
        observer_heights.push_back(record.observer_height_m);
        if (motion) tracking_errors.push_back(record.tracking_error_m);
        return true;
    };

    const std::size_t max_motion_cycles = static_cast<std::size_t>(
        std::ceil(condition.max_motion_sec / dt));
    for (std::size_t cycle = 0; cycle < max_motion_cycles; ++cycle) {
        const double time_sec = static_cast<double>(cycle) * dt;
        if (!plant.advanceTo(time_sec, error) ||
            !updateObserverAfterAdvance()) {
            runtime_error = error.empty()
                ? "internal observer update failed"
                : error;
            break;
        }
        const IndependentPlantState state = plant.state();
        const ProgressProjection projection = projector.project(
            path.reference, state.x, state.y);
        if (!projection.valid) {
            runtime_error = "tracking projection failed";
            break;
        }
        RobotState robot;
        robot.x = state.x;
        robot.y = state.y;
        robot.yaw = state.yaw;
        robot.v = state.v;
        robot.omega = state.omega;

        CycleTimingContract timing;
        timing.cycle_id = next_cycle_id;
        timing.cycle_start_stamp_ns =
            kStampBaseNs + secondsToNanoseconds(time_sec);
        timing.control_period_sec = dt;
        const PublishEpochEstimate estimate = engine.estimatePublishEpoch(timing);
        if (!estimate.valid || estimate.expected_deadline_missed) {
            runtime_error = "publish epoch estimate rejected";
            break;
        }

        SolverInput solver_input;
        solver_input.robot = robot;
        solver_input.slosh = observer;
        solver_input.dt = dt;
        solver_input.horizon_steps = execution_horizon_required
            ? compiled.horizon_steps
            : 60;
        solver_input.cycle_timing.cycle_id = next_cycle_id;
        solver_input.cycle_timing.cycle_start_stamp_ns =
            timing.cycle_start_stamp_ns;
        solver_input.cycle_timing.raw_robot_state_stamp_ns =
            timing.cycle_start_stamp_ns;
        solver_input.cycle_timing.raw_liquid_state_stamp_ns =
            timing.cycle_start_stamp_ns;
        solver_input.cycle_timing.robot_state_stamp_ns =
            timing.cycle_start_stamp_ns;
        solver_input.cycle_timing.liquid_state_stamp_ns =
            timing.cycle_start_stamp_ns;
        solver_input.cycle_timing.solver_input_epoch_ns =
            timing.cycle_start_stamp_ns;
        if (execution_horizon_required) {
            ExecutionHorizonBuildRequest build;
            build.source_robot = robot;
            build.source_slosh = observer;
            build.source_epoch_ns = timing.cycle_start_stamp_ns;
            build.publish_epoch_estimate = estimate;
            build.command_history = &command_history;
            build.expected_execution_contract_hash =
                compiled.execution.contract_hash;
            build.initial_progress_s = projection.s;
            build.liquid_horizon_steps = compiled.liquid_horizon_steps;
            const ExecutionHorizonBuildResult built =
                execution_builder.build(build);
            if (!built.valid) {
                runtime_error = "execution horizon rejected: " + built.status;
                break;
            }
            solver_input.execution_horizon = built.context;
        }

        if (replay_session) replay_session->setCycle(cycle);
        const double plant_publish_time_sec =
            secondsBetween(estimate.expected_publish_stamp_ns, kStampBaseNs);
        sink.arm(plant_publish_time_sec,
                 estimate.expected_publish_stamp_ns);
        ControlCycleRequest request;
        request.cycle_id = next_cycle_id;
        request.cycle_start_ns = timing.cycle_start_stamp_ns;
        request.solver_input = solver_input;
        request.prediction_valid = execution_horizon_required;
        request.prediction_status = execution_horizon_required ? "OK" : "OFF";
        if (solver_input.execution_horizon.active) {
            request.execution_front_robot =
                solver_input.execution_horizon.initial_state.robot;
            request.execution_front_slosh =
                solver_input.execution_horizon.initial_state.slosh;
        } else {
            request.execution_front_robot = robot;
            request.execution_front_slosh = observer;
        }
        request.solver_origin_at_execution_front = false;
        request.execution_front_steps = compiled.execution_front_steps;
        request.phase_time_sec = static_cast<double>(
            estimate.expected_publish_stamp_ns) * kSecondsPerNanosecond;
        request.period_sec = dt;
        request.control_period_sec = dt;
        request.publish_epoch_estimate = estimate;
        request.publish_enabled = true;
        request.command_sink = &sink;
        request.command_history = &command_history;
        const ControlCycleResult result = engine.step(request);
        if (!result.solver_success && !first_solver_failure.valid) {
            first_solver_failure = captureSolverFailureDiagnostic(
                result, artifact);
        }

        CycleRecord record;
        record.cycle_id = next_cycle_id;
        record.time_sec = time_sec;
        record.window = "motion";
        record.plant = state;
        record.tracking_error_m = projection.distance;
        record.progress_s = projection.s;
        record.observer = observer;
        record.observer_height_m = observer_dynamics.height(
            observer, robot.omega);
        record.solver_success = result.solver_success;
        record.raw_solver_status = result.solver_output.status;
        record.phase_status = result.have_phase_decision
            ? result.phase_decision.status
            : result.phase_preparation.status;
        record.final_status = result.output.status;
        record.gate_evaluated = result.phase_decision.evaluated;
        record.terminal_gate_accepted =
            result.phase_decision.terminal_gate_accepted;
        record.current_execution_compatible =
            result.phase_decision.current_execution_compatible;
        record.terminal_execution_compatible =
            result.phase_decision.terminal_execution_compatible;
        record.recovery_used = result.phase_decision.recovery_command_used;
        record.controlled_stop_used =
            result.phase_decision.controlled_stop_used;
        record.selected_phase_valid =
            result.phase_preparation.candidate.valid;
        record.clock_index =
            result.phase_preparation.candidate.clock_index;
        record.candidate_window_begin_index =
            result.phase_preparation.candidate
                .candidate_window_begin_index;
        record.candidate_window_end_index =
            result.phase_preparation.candidate
                .candidate_window_end_index;
        record.selected_phase_index =
            result.phase_preparation.candidate.current_index;
        record.phase_lead_steps =
            result.phase_preparation.candidate.phase_lead_steps;
        record.execution_candidate_filter_applied =
            result.phase_preparation.candidate
                .execution_compatibility_filter_applied;
        record.execution_rejected_candidate_count =
            result.phase_preparation.candidate
                .execution_rejected_candidate_count;
        record.selected_execution_max_normalized_error =
            result.phase_preparation.candidate
                .selected_execution_max_normalized_error;
        const PreSolveSnapshotDebug& solver_snapshot =
            result.solver_output.pre_solve_snapshot;
        if (solver_snapshot.backend ==
                kSolverBackendDelayAugmentedPhaseAcados &&
            finite(solver_snapshot.acados_solve_time_ms) &&
            finite(solver_snapshot.backend_wall_time_ms) &&
            solver_snapshot.acados_solve_time_ms >= 0.0 &&
            solver_snapshot.backend_wall_time_ms > 0.0) {
            record.acados_solve_time_ms =
                solver_snapshot.acados_solve_time_ms;
            record.backend_wall_time_ms =
                solver_snapshot.backend_wall_time_ms;
        }
        record.final_command = result.final_command;
        record.command_source = result.publication.pipeline.decision.source;
        record.publish_stamp_ns =
            result.publication.receipt.actual_publish_stamp_ns;
        record.plant_publish = sink.lastPlantReceipt();
        if (!appendRecord(record, true)) {
            runtime_error = "cycle CSV write failed";
            break;
        }
        if (!result.solver_success) ++counters.solver_failures;
        if (record.backend_wall_time_ms > 0.0) {
            counters.solver_id = solver_snapshot.solver_id;
            counters.nlp_solver_type = solver_snapshot.nlp_solver_type;
            counters.solver_config_hash =
                solver_snapshot.solver_config_hash;
            counters.acados_solve_times_ms.push_back(
                record.acados_solve_time_ms);
            counters.backend_wall_times_ms.push_back(
                record.backend_wall_time_ms);
        }
        if (solver_snapshot.solver_residuals_evaluated &&
            finite(solver_snapshot.stationarity_residual) &&
            finite(solver_snapshot.equality_residual) &&
            finite(solver_snapshot.inequality_residual) &&
            finite(solver_snapshot.complementarity_residual)) {
            ++counters.kkt_residual_samples;
            counters.max_stationarity_residual = std::max(
                counters.max_stationarity_residual,
                solver_snapshot.stationarity_residual);
            counters.max_equality_residual = std::max(
                counters.max_equality_residual,
                solver_snapshot.equality_residual);
            counters.max_inequality_residual = std::max(
                counters.max_inequality_residual,
                solver_snapshot.inequality_residual);
            counters.max_complementarity_residual = std::max(
                counters.max_complementarity_residual,
                solver_snapshot.complementarity_residual);
            counters.max_sqp_iterations = std::max(
                counters.max_sqp_iterations,
                solver_snapshot.solver_sqp_iterations);
            counters.max_qp_iterations = std::max(
                counters.max_qp_iterations,
                solver_snapshot.solver_qp_iterations);
        }
        ++counters.raw_solver_status_counts[record.raw_solver_status];
        ++counters.phase_status_counts[record.phase_status];
        ++counters.final_status_counts[record.final_status];
        if (record.execution_candidate_filter_applied) {
            ++counters.execution_candidate_filter_cycles;
            counters.execution_rejected_candidates +=
                record.execution_rejected_candidate_count;
            if (record.selected_phase_valid) {
                counters.max_selected_execution_normalized_error = std::max(
                    counters.max_selected_execution_normalized_error,
                    record.selected_execution_max_normalized_error);
            }
        }
        if (record.gate_evaluated) ++counters.gate_evaluations;
        if (record.terminal_gate_accepted) ++counters.terminal_gate_accepts;
        if (record.recovery_used) ++counters.recovery_actions;
        if (record.controlled_stop_used) ++counters.controlled_stops;
        if (result.publication.published()) ++counters.publications;
        else ++counters.publication_failures;
        last_motion_time_sec = time_sec;
        ++next_cycle_id;

        if (!result.publication.published()) {
            runtime_error = "final command publication failed: " +
                sink.lastError();
            break;
        }
        if (record.controlled_stop_used) {
            completion_reason = "CONTROLLED_STOP";
            break;
        }
        if (condition.mode == TrialMode::OfflineReplay &&
            cycle + 1 >= artifact.size()) {
            completed = true;
            completion_reason = "ARTIFACT_TAIL_COMPLETE";
            break;
        }
        if ((condition.mode == TrialMode::ResidualNoGate ||
             condition.mode == TrialMode::PhaseRejoinFull) &&
            result.phase_debug.terminal_release_authorized) {
            completed = true;
            completion_reason = "PHASE_TAIL_RELEASED";
            break;
        }
        if ((condition.mode == TrialMode::OrdinaryMpcc ||
             condition.mode == TrialMode::SmoothMatchMpcc ||
             condition.mode == TrialMode::InputShaping) &&
            result.terminal_priority) {
            completed = true;
            completion_reason = "GOAL_REACHED";
            break;
        }
    }

    // A fixed physical tail is always collected after motion.  It uses the
    // same publication transaction/history as motion, while no solver or gate
    // is allowed to consume the external liquid measurement.
    const std::size_t tail_cycles = static_cast<std::size_t>(
        std::llround(condition.fixed_tail_sec / dt));
    for (std::size_t tail = 1;
         runtime_error.empty() && tail <= tail_cycles; ++tail) {
        const double time_sec = last_motion_time_sec +
            static_cast<double>(tail) * dt;
        if (!plant.advanceTo(time_sec, error) ||
            !updateObserverAfterAdvance()) {
            runtime_error = error.empty()
                ? "tail observer update failed"
                : error;
            break;
        }
        const IndependentPlantState state = plant.state();
        const ProgressProjection projection = projector.project(
            path.reference, state.x, state.y);
        if (!projection.valid) {
            runtime_error = "tail tracking projection failed";
            break;
        }
        const StampNs cycle_start_ns =
            kStampBaseNs + secondsToNanoseconds(time_sec);
        CycleTimingContract timing;
        timing.cycle_id = next_cycle_id;
        timing.cycle_start_stamp_ns = cycle_start_ns;
        timing.control_period_sec = dt;
        const PublishEpochEstimate estimate = engine.estimatePublishEpoch(timing);
        if (!estimate.valid || estimate.expected_deadline_missed) {
            runtime_error = "tail publish epoch estimate rejected";
            break;
        }
        sink.arm(secondsBetween(
                     estimate.expected_publish_stamp_ns, kStampBaseNs),
                 estimate.expected_publish_stamp_ns);
        const CommandPublicationResult publication =
            engine.publishFailClosedZero(
                next_cycle_id, cycle_start_ns, dt, &sink,
                &command_history, true, "FIXED_TAIL_ZERO");
        CycleRecord record;
        record.cycle_id = next_cycle_id;
        record.time_sec = time_sec;
        record.window = "fixed_tail";
        record.plant = state;
        record.tracking_error_m = projection.distance;
        record.progress_s = projection.s;
        record.observer = observer;
        record.observer_height_m = observer_dynamics.height(
            observer, state.omega);
        record.solver_success = false;
        record.raw_solver_status = "NOT_RUN_FIXED_TAIL";
        record.phase_status = "NOT_RUN_FIXED_TAIL";
        record.final_status = "FIXED_TAIL_ZERO";
        record.final_command = publication.pipeline.final_command;
        record.command_source = publication.pipeline.decision.source;
        record.publish_stamp_ns = publication.receipt.actual_publish_stamp_ns;
        record.plant_publish = sink.lastPlantReceipt();
        if (!appendRecord(record, false)) {
            runtime_error = "tail CSV write failed";
            break;
        }
        if (publication.published()) ++counters.publications;
        else {
            ++counters.publication_failures;
            runtime_error = "tail zero publication failed: " +
                sink.lastError();
            break;
        }
        ++next_cycle_id;
    }
    cycle_output.close();

    const IndependentPlantState final_state = plant.state();
    const TrajectoryPoint goal = path.reference.sample(path.reference.length());
    const double final_goal_error = std::hypot(
        final_state.x - goal.x, final_state.y - goal.y);
    const bool task_success = completed && runtime_error.empty() &&
        final_goal_error <= condition.task_success_goal_tolerance_m;
    if (!writeSummary(
            args, condition, plant_config, artifact, counters,
            first_solver_failure,
            measured_heights, true_heights, observer_heights,
            tracking_errors, completed, task_success, completion_reason,
            last_motion_time_sec, final_goal_error, zvd,
            runtime_error, error)) {
        std::cerr << "ERROR: " << error << '\n';
        cleanup_temporary();
        return kRuntimeExit;
    }
    if (!publishOutputsAtomically(
            args, final_cycle_path, final_summary_path, error)) {
        std::cerr << "ERROR: " << error << '\n';
        cleanup_temporary();
        return kRuntimeExit;
    }
    std::cout << (runtime_error.empty() ? "TRIAL_WRITTEN" : "TRIAL_FAILED")
              << " condition=" << condition.condition_id
              << " task_success=" << (task_success ? "true" : "false")
              << " q95_m="
              << nearestRankQuantile(measured_heights, 0.95)
              << " summary=" << final_summary_path << '\n';
    return runtime_error.empty() ? 0 : kRuntimeExit;
}

}  // namespace closed_loop_trial
}  // namespace simulation
}  // namespace spmpc_local_planner

#ifndef SPMPC_TRIAL_RUNNER_NO_MAIN
int main(int argc, char** argv) {
    return spmpc_local_planner::simulation::closed_loop_trial::
        runPhaseRejoinClosedLoopTrial(argc, argv);
}
#endif
