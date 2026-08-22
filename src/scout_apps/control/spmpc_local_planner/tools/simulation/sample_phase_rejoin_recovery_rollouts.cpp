#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"
#include "spmpc_local_planner/simulation/exclusive_output_pair.h"
#include "spmpc_local_planner/simulation/phase_rejoin_recovery_rollout.h"
#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace spmpc = spmpc_local_planner;
namespace simulation = spmpc_local_planner::simulation;
namespace manifest =
    spmpc_local_planner::delay_augmented_phase_solver_manifest;

namespace {

constexpr double kValueTolerance = 1.0e-9;

struct Arguments {
    std::string plant_config;
    std::string offline_plan;
    std::string offline_plan_report;
    std::string path_json;
    std::string sampling_config;
    std::string split;
    std::string dataset_output;
    std::string audit_output;
    std::uint32_t seed = 0;
    std::size_t phase_begin = 0;
    std::size_t phase_end = std::numeric_limits<std::size_t>::max();
    bool seed_set = false;
};

struct PlanRow {
    std::size_t index = 0;
    double t = 0.0;
    double published_v = 0.0;
    double published_omega = 0.0;
    double progress_rate = 0.0;
};

struct PathStart {
    std::string frame_id;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double length_m = 0.0;
    std::size_t pose_count = 0;
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
    if (text.empty() || text.front() == '-') return false;
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        parsed > static_cast<unsigned long long>(
            std::numeric_limits<std::size_t>::max())) {
        return false;
    }
    value = static_cast<std::size_t>(parsed);
    return true;
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

bool parseUnsigned(const std::string& text,
                   unsigned long long maximum,
                   unsigned long long& value) {
    if (text.empty() || text.front() == '-') return false;
    try {
        std::size_t consumed = 0;
        value = std::stoull(text, &consumed, 10);
        return consumed == text.size() && value <= maximum;
    } catch (const std::exception&) {
        return false;
    }
}

bool takeString(int argc, char** argv, int& index, std::string& value) {
    if (index + 1 >= argc || !value.empty()) return false;
    value = argv[++index];
    return !value.empty();
}

bool parseArguments(int argc, char** argv, Arguments& args) {
    std::set<std::string> seen;
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        if (!seen.insert(option).second) return false;
        if (option == "--plant-config") {
            if (!takeString(argc, argv, index, args.plant_config)) return false;
        } else if (option == "--offline-plan") {
            if (!takeString(argc, argv, index, args.offline_plan)) return false;
        } else if (option == "--offline-plan-report") {
            if (!takeString(argc, argv, index, args.offline_plan_report)) return false;
        } else if (option == "--path-json") {
            if (!takeString(argc, argv, index, args.path_json)) return false;
        } else if (option == "--sampling-config") {
            if (!takeString(argc, argv, index, args.sampling_config)) return false;
        } else if (option == "--split") {
            if (!takeString(argc, argv, index, args.split)) return false;
        } else if (option == "--dataset-output") {
            if (!takeString(argc, argv, index, args.dataset_output)) return false;
        } else if (option == "--audit-output") {
            if (!takeString(argc, argv, index, args.audit_output)) return false;
        } else if (option == "--seed") {
            if (index + 1 >= argc || args.seed_set) return false;
            unsigned long long value = 0;
            if (!parseUnsigned(
                    argv[++index],
                    std::numeric_limits<std::uint32_t>::max(), value)) {
                return false;
            }
            args.seed = static_cast<std::uint32_t>(value);
            args.seed_set = true;
        } else if (option == "--phase-begin") {
            if (index + 1 >= argc) return false;
            unsigned long long value = 0;
            if (!parseUnsigned(
                    argv[++index],
                    std::numeric_limits<std::size_t>::max(), value)) {
                return false;
            }
            args.phase_begin = static_cast<std::size_t>(value);
        } else if (option == "--phase-end") {
            if (index + 1 >= argc) return false;
            unsigned long long value = 0;
            if (!parseUnsigned(
                    argv[++index],
                    std::numeric_limits<std::size_t>::max(), value)) {
                return false;
            }
            args.phase_end = static_cast<std::size_t>(value);
        } else {
            return false;
        }
    }
    return !args.plant_config.empty() && !args.offline_plan.empty() &&
        !args.offline_plan_report.empty() && !args.path_json.empty() &&
        !args.sampling_config.empty() && !args.split.empty() &&
        !args.dataset_output.empty() && !args.audit_output.empty() &&
        args.dataset_output != args.audit_output && args.seed_set;
}

int usage(const std::string& detail = std::string()) {
    if (!detail.empty()) std::cerr << "ERROR: " << detail << '\n';
    std::cerr
        << "usage: spmpc_sample_phase_rejoin_recovery_rollouts"
        << " --plant-config PATH --offline-plan PATH"
        << " --offline-plan-report PATH --path-json PATH"
        << " --sampling-config PATH --split fit|tune|held_out"
        << " --seed UINT32 --dataset-output PATH --audit-output PATH"
        << " [--phase-begin INDEX] [--phase-end INDEX]\n";
    return 2;
}

spmpc::SloshModelParams controllerSloshParams() {
    spmpc::SloshModelParams params;
    params.container_radius = manifest::kContainerRadius;
    params.liquid_height = manifest::kLiquidHeight;
    params.liquid_density = manifest::kLiquidDensity;
    params.damping_ratio = manifest::kDampingRatio;
    params.mode_index = manifest::kModeIndex;
    params.dt = manifest::kDt;
    params.slosh_height_ref = manifest::kSloshHeightRef;
    params.slosh_eta_dot_ratio = manifest::kSloshEtaDotRatio;
    params.use_linear_model = manifest::kUseLinearModel;
    params.use_parabola_term = manifest::kUseParabolaTerm;
    return params;
}

spmpc::ExecutionModelContract controllerExecutionContract() {
    spmpc::ExecutionModelContract contract;
    contract.schema_version = manifest::kExecutionContractSchemaVersion;
    contract.contract_id = manifest::kContractId;
    contract.contract_hash = manifest::kContractHash;
    contract.dt = manifest::kDt;
#define SPMPC_ASSIGN_CHANNEL(channel, prefix) \
    contract.channel.delay_sec = manifest::k##prefix##DelaySec; \
    contract.channel.time_constant_sec = manifest::k##prefix##TimeConstantSec; \
    contract.channel.positive_gain = manifest::k##prefix##PositiveGain; \
    contract.channel.negative_gain = manifest::k##prefix##NegativeGain; \
    contract.channel.deadzone = manifest::k##prefix##Deadzone; \
    contract.channel.output_min = manifest::k##prefix##OutputMin; \
    contract.channel.output_max = manifest::k##prefix##OutputMax
    SPMPC_ASSIGN_CHANNEL(linear, Linear);
    SPMPC_ASSIGN_CHANNEL(angular, Angular);
#undef SPMPC_ASSIGN_CHANNEL
    return contract;
}

bool loadPlan(const std::string& path,
              std::map<std::string, std::string>& metadata,
              std::vector<PlanRow>& rows,
              std::string& error) {
    std::ifstream input(path);
    if (!input.is_open()) {
        error = "cannot open OfflineSloshOCP plan";
        return false;
    }
    const std::vector<std::string> expected = {
        "index", "t", "u_pub_v", "u_pub_omega", "v_s"};
    bool header_seen = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::string clean = trim(line);
        if (clean.empty()) continue;
        if (clean.front() == '#') {
            if (header_seen) {
                error = "plan metadata appears after header";
                return false;
            }
            const std::string payload = trim(clean.substr(1));
            const std::size_t separator = payload.find('=');
            if (separator == std::string::npos) {
                error = "malformed plan metadata";
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
                error = "OfflineSloshOCP plan header mismatch";
                return false;
            }
            header_seen = true;
            continue;
        }
        if (fields.size() != expected.size()) {
            error = "OfflineSloshOCP plan column mismatch";
            return false;
        }
        PlanRow row;
        if (!parseIndex(fields[0], row.index) ||
            !parseDouble(fields[1], row.t) ||
            !parseDouble(fields[2], row.published_v) ||
            !parseDouble(fields[3], row.published_omega) ||
            !parseDouble(fields[4], row.progress_rate) ||
            row.index != rows.size()) {
            error = "invalid OfflineSloshOCP plan row";
            return false;
        }
        rows.push_back(row);
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
    if (!header_seen || rows.size() < 20 ||
        metadata["schema"] != "spmpc_offline_slosh_ocp_plan_v1" ||
        metadata["status"] != "PASS" ||
        metadata["simulation_only"] != "true" ||
        metadata["formal_robot_release"] != "false" ||
        metadata["execution_contract_hash"] != manifest::kContractHash) {
        error = "OfflineSloshOCP plan provenance rejected";
        return false;
    }
    return true;
}

bool loadPathStart(const std::string& path,
                   PathStart& start,
                   std::string& error) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node poses = root["poses"];
        if (!root["frame_id"] || !poses || !poses.IsSequence() ||
            poses.size() < 2) {
            error = "path JSON schema rejected";
            return false;
        }
        start.frame_id = root["frame_id"].as<std::string>();
        double previous_x = 0.0;
        double previous_y = 0.0;
        for (std::size_t index = 0; index < poses.size(); ++index) {
            const YAML::Node pose = poses[index];
            const double px = pose["x"].as<double>();
            const double py = pose["y"].as<double>();
            const double qx = pose["qx"].as<double>();
            const double qy = pose["qy"].as<double>();
            const double qz = pose["qz"].as<double>();
            const double qw = pose["qw"].as<double>();
            const double norm = std::hypot(std::hypot(qx, qy),
                                           std::hypot(qz, qw));
            if (!std::isfinite(px) || !std::isfinite(py) ||
                !std::isfinite(norm) || norm <= 1.0e-12) {
                error = "path contains an invalid pose";
                return false;
            }
            if (index == 0) {
                start.x = px;
                start.y = py;
                const double x = qx / norm;
                const double y = qy / norm;
                const double z = qz / norm;
                const double w = qw / norm;
                start.yaw = std::atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
            } else {
                const double segment = std::hypot(px - previous_x,
                                                  py - previous_y);
                if (!std::isfinite(segment) || segment <= 1.0e-6) {
                    error = "path contains a duplicate segment";
                    return false;
                }
                start.length_m += segment;
            }
            previous_x = px;
            previous_y = py;
        }
        start.pose_count = poses.size();
        return !start.frame_id.empty() && std::isfinite(start.yaw) &&
            std::isfinite(start.length_m) && start.length_m > 0.0;
    } catch (const std::exception& exception) {
        error = std::string("cannot parse path JSON: ") + exception.what();
        return false;
    }
}

bool validatePlanReport(const std::string& report_path,
                        const std::string& plan_hash,
                        const std::string& path_hash,
                        std::size_t row_count,
                        std::string& error) {
    try {
        const YAML::Node report = YAML::LoadFile(report_path);
        if (!report["schema"] ||
            report["schema"].as<std::string>() !=
                "spmpc_offline_slosh_ocp_report_v1" ||
            report["status"].as<std::string>() != "PASS" ||
            !report["simulation_only"].as<bool>() ||
            report["formal_robot_release"].as<bool>() ||
            !report["optimizer"]["success"].as<bool>() ||
            report["plan"]["sha256"].as<std::string>() != plan_hash ||
            report["plan"]["rows"].as<std::size_t>() != row_count ||
            report["path"]["sha256"].as<std::string>() != path_hash ||
            report["execution_contract_hash"].as<std::string>() !=
                manifest::kContractHash ||
            report["terminal_contract"]["name"].as<std::string>() !=
                "publish_zero_settle_hold_v2" ||
            !report["terminal_contract"]
                ["goal_yaw_preserved_from_path_quaternion"].as<bool>() ||
            report["physical_parameter_claim"].as<bool>() ||
            !report["source_limitations_acknowledged"].as<bool>()) {
            error = "OfflineSloshOCP report rejected";
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        error = std::string("cannot validate OfflineSloshOCP report: ") +
            exception.what();
        return false;
    }
}

bool buildNominalProjection(
    const Arguments& args,
    std::vector<spmpc::PhaseNominalSample>& samples,
    double& dt,
    std::string& error) {
    std::map<std::string, std::string> metadata;
    std::vector<PlanRow> plan;
    if (!loadPlan(args.offline_plan, metadata, plan, error)) return false;
    const std::string plan_hash = sha256File(args.offline_plan);
    const std::string path_hash = sha256File(args.path_json);
    if (plan_hash.empty() || path_hash.empty() ||
        path_hash != metadata["path_sha256"] ||
        !validatePlanReport(
            args.offline_plan_report, plan_hash, path_hash,
            plan.size(), error)) {
        if (error.empty()) error = "plan/path hash binding failed";
        return false;
    }
    PathStart start;
    if (!loadPathStart(args.path_json, start, error)) return false;
    double declared_length = 0.0;
    if (!parseDouble(metadata["dt"], dt) ||
        !parseDouble(metadata["path_length"], declared_length) ||
        std::abs(dt - manifest::kDt) > 1.0e-12 ||
        start.frame_id != metadata["path_frame_id"] ||
        std::abs(start.length_m - declared_length) > 1.0e-9) {
        error = "plan/path timing or geometry mismatch";
        return false;
    }

    spmpc::ExecutionModel execution_model;
    if (!execution_model.configure(
            controllerExecutionContract(), controllerSloshParams(), error)) {
        error = "compiled execution model unavailable: " + error;
        return false;
    }
    spmpc::RobotState robot;
    robot.x = start.x;
    robot.y = start.y;
    robot.yaw = start.yaw;
    spmpc::SloshState slosh;
    spmpc::VelocityCommand held;
    spmpc::ExecutionAugmentedState execution;
    if (!execution_model.initializeHeld(
            robot, slosh, held, execution, error)) {
        error = "cannot initialize compiled nominal projection: " + error;
        return false;
    }

    samples.reserve(plan.size());
    double progress = 0.0;
    for (std::size_t index = 0; index < plan.size(); ++index) {
        const PlanRow& control = plan[index];
        if (std::abs(control.t - static_cast<double>(index) * dt) > 1.0e-9 ||
            control.published_v < manifest::kLinearOutputMin - kValueTolerance ||
            control.published_v > manifest::kLinearOutputMax + kValueTolerance ||
            control.published_omega <
                manifest::kAngularOutputMin - kValueTolerance ||
            control.published_omega >
                manifest::kAngularOutputMax + kValueTolerance ||
            control.progress_rate < -kValueTolerance ||
            control.progress_rate > manifest::kProgressRateMax + kValueTolerance) {
            error = "OfflineSloshOCP command/time bound mismatch at phase " +
                std::to_string(index);
            return false;
        }
        const double acceleration =
            (control.published_v -
             execution.linear.pending_commands.back()) / dt;
        const double angular_acceleration =
            (control.published_omega -
             execution.angular.pending_commands.back()) / dt;
        if (std::abs(acceleration) >
                manifest::kAccelerationMax + kValueTolerance ||
            std::abs(angular_acceleration) >
                manifest::kAngularAccelerationMax + kValueTolerance) {
            error = "OfflineSloshOCP command-rate mismatch at phase " +
                std::to_string(index);
            return false;
        }
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
        sample.a = acceleration;
        sample.alpha = angular_acceleration;
        sample.v_s = control.progress_rate;
        sample.u_pub_v = control.published_v;
        sample.u_pub_omega = control.published_omega;
        sample.kappa_v = control.published_v;
        sample.kappa_omega = control.published_omega;
        sample.augmented_execution_valid = true;
        sample.augmented_execution = execution;
        samples.push_back(sample);
        if (index + 1 < plan.size()) {
            spmpc::VelocityCommand published;
            published.linear = control.published_v;
            published.angular = control.published_omega;
            const spmpc::ExecutionStepResult stepped =
                execution_model.step(execution, published);
            if (!stepped.valid) {
                error = "compiled nominal transition failed at phase " +
                    std::to_string(index);
                return false;
            }
            execution = stepped.state;
            progress += control.progress_rate * dt;
        }
    }
    return true;
}

std::string datasetCsv(
    const std::vector<simulation::RecoveryDatasetRow>& rows) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "split,rollout_id,seed,phase_index,recovered,"
           "x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot,"
           "linear_output,angular_output,"
           "linear_pending_0,linear_pending_1,linear_pending_2,"
           "linear_pending_3,linear_pending_4,"
           "angular_pending_0,angular_pending_1,angular_pending_2,"
           "angular_pending_3,angular_pending_4,angular_pending_5,"
           "angular_pending_6\n";
    for (const simulation::RecoveryDatasetRow& row : rows) {
        out << row.split << ',' << row.rollout_id << ',' << row.seed << ','
            << row.phase_index << ',' << (row.recovered ? 1 : 0);
        for (double value : row.state_errors) out << ',' << value;
        for (double value : row.execution_errors) out << ',' << value;
        out << '\n';
    }
    return out.str();
}

std::string auditCsv(
    const std::vector<simulation::RecoveryRolloutAudit>& audits) {
    std::ostringstream out;
    out << std::setprecision(17);
    out << "split,rollout_id,profile_id,seed,phase_index,recovered,"
           "external_liquid_truth_visible_to_candidate_policy,"
           "external_liquid_truth_used_for_features,"
           "external_liquid_truth_used_for_label,"
           "path_position_passed,path_yaw_passed,external_height_passed,"
           "terminal_position_passed,terminal_yaw_passed,"
           "terminal_velocity_passed,terminal_external_height_passed,"
           "snapshot_time_sec,final_time_sec,"
           "maximum_path_position_error_m,maximum_path_yaw_error_rad,"
           "maximum_external_height_m,terminal_position_error_m,"
           "terminal_yaw_error_rad,terminal_v_abs_mps,"
           "terminal_omega_abs_radps,terminal_external_height_m\n";
    for (const simulation::RecoveryRolloutAudit& row : audits) {
        out << row.split << ',' << row.rollout_id << ',' << row.profile_id
            << ',' << row.seed << ',' << row.phase_index << ','
            << (row.recovered ? 1 : 0) << ",0,0,1,"
            << (row.path_position_passed ? 1 : 0) << ','
            << (row.path_yaw_passed ? 1 : 0) << ','
            << (row.external_height_passed ? 1 : 0) << ','
            << (row.terminal_position_passed ? 1 : 0) << ','
            << (row.terminal_yaw_passed ? 1 : 0) << ','
            << (row.terminal_velocity_passed ? 1 : 0) << ','
            << (row.terminal_external_height_passed ? 1 : 0) << ','
            << row.snapshot_time_sec << ',' << row.final_time_sec << ','
            << row.maximum_path_position_error_m << ','
            << row.maximum_path_yaw_error_rad << ','
            << row.maximum_external_height_m << ','
            << row.terminal_position_error_m << ','
            << row.terminal_yaw_error_rad << ','
            << row.terminal_v_abs_mps << ','
            << row.terminal_omega_abs_radps << ','
            << row.terminal_external_height_m << '\n';
    }
    return out.str();
}

}  // namespace

int main(int argc, char** argv) {
    Arguments args;
    if (!parseArguments(argc, argv, args)) return usage();

    std::string error;
    simulation::IndependentPlantConfig plant_config;
    if (!simulation::loadIndependentPlantConfig(
            args.plant_config, plant_config, error)) {
        return usage("plant config rejected: " + error);
    }
    simulation::RecoveryRolloutSamplingConfig sampling_config;
    if (!simulation::loadRecoveryRolloutSamplingConfig(
            args.sampling_config, sampling_config, error)) {
        return usage("sampling config rejected: " + error);
    }
    if (sampling_config.maximum_published_acceleration !=
            manifest::kAccelerationMax ||
        sampling_config.maximum_published_angular_acceleration !=
            manifest::kAngularAccelerationMax) {
        return usage(
            "sampling publication-rate contract does not exact-match "
            "the compiled 22D execution contract");
    }
    std::vector<spmpc::PhaseNominalSample> nominal_samples;
    double nominal_dt = 0.0;
    if (!buildNominalProjection(
            args, nominal_samples, nominal_dt, error)) {
        return usage("gate-free nominal projection rejected: " + error);
    }
    const std::size_t phase_end =
        args.phase_end == std::numeric_limits<std::size_t>::max()
        ? nominal_samples.size() - 1 : args.phase_end;
    simulation::PhaseRejoinRecoveryRolloutSampler sampler;
    if (!sampler.configure(
            plant_config, sampling_config, controllerSloshParams(),
            nominal_dt, nominal_samples, error)) {
        return usage("sampler configuration rejected: " + error);
    }
    const simulation::RecoverySeedSampleResult sampled = sampler.sampleSeed(
        args.split, args.seed, args.phase_begin, phase_end);
    if (!sampled.valid) return usage("sampling failed: " + sampled.status);

    simulation::ExclusiveOutputPair outputs;
    if (!outputs.stage(
            args.dataset_output, datasetCsv(sampled.rows),
            args.audit_output, auditCsv(sampled.audits), error) ||
        !outputs.commit(error)) {
        std::cerr << "ERROR: " << error << '\n';
        return 3;
    }
    std::size_t recovered_count = 0;
    for (const simulation::RecoveryDatasetRow& row : sampled.rows) {
        recovered_count += row.recovered ? 1u : 0u;
    }
    std::cout << "split=" << args.split << " seed=" << args.seed
              << " rows=" << sampled.rows.size()
              << " recovered=" << recovered_count
              << " nominal_source=offline_plan_compiled_22d_transition"
              << " external_truth_features=false"
              << " external_truth_label=true\n";
    return 0;
}
