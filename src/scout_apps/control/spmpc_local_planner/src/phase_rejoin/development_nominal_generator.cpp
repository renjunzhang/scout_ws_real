#include "spmpc_local_planner/phase_rejoin/development_nominal_generator.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace {

constexpr std::size_t kMaximumSamples = 1000000;
constexpr double kPi = 3.141592653589793238462643383279502884;

DevelopmentNominalGenerationResult failure(const std::string& status,
                                           const std::string& detail) {
    DevelopmentNominalGenerationResult result;
    result.status = status;
    result.detail = detail;
    return result;
}

bool finite(double value) {
    return std::isfinite(value);
}

bool positive(double value) {
    return finite(value) && value > 0.0;
}

bool positiveRadii(const EmpiricalRecoveryRadii& radii) {
    return positive(radii.x) && positive(radii.y) && positive(radii.yaw) &&
           positive(radii.v) && positive(radii.omega) &&
           positive(radii.eta_x) && positive(radii.eta_x_dot) &&
           positive(radii.eta_y) && positive(radii.eta_y_dot);
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

bool metadataText(const std::string& value) {
    const std::string clean = trim(value);
    return !clean.empty() && clean.find('\n') == std::string::npos &&
           clean.find('\r') == std::string::npos;
}

bool lowercaseSha256(const std::string& value) {
    if (value.size() != 64) {
        return false;
    }
    for (char character : value) {
        if (!((character >= '0' && character <= '9') ||
              (character >= 'a' && character <= 'f'))) {
            return false;
        }
    }
    return true;
}

std::string formatDouble(double value) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << value;
    return out.str();
}

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

double wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

std::vector<DevelopmentNominalPoint> cleanPoints(
    const std::vector<DevelopmentNominalPoint>& points) {
    std::vector<DevelopmentNominalPoint> clean;
    for (const DevelopmentNominalPoint& point : points) {
        if (!finite(point.x) || !finite(point.y)) {
            return {};
        }
        if (clean.empty() ||
            std::hypot(point.x - clean.back().x,
                       point.y - clean.back().y) > 1.0e-6) {
            clean.push_back(point);
        }
    }
    return clean;
}

std::vector<double> cumulativeProgress(
    const std::vector<DevelopmentNominalPoint>& points) {
    std::vector<double> progress(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        progress[i] = progress[i - 1] +
            std::hypot(points[i].x - points[i - 1].x,
                       points[i].y - points[i - 1].y);
    }
    return progress;
}

DevelopmentNominalPoint interpolatePosition(
    const std::vector<DevelopmentNominalPoint>& points,
    const std::vector<double>& path_progress,
    double query) {
    const double s = clamp(query, 0.0, path_progress.back());
    std::size_t lo = 0;
    std::size_t hi = path_progress.size() - 1;
    while (lo + 1 < hi) {
        const std::size_t mid = (lo + hi) / 2;
        if (path_progress[mid] <= s) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const double length = path_progress[lo + 1] - path_progress[lo];
    const double ratio = length <= 1.0e-12
        ? 0.0
        : (s - path_progress[lo]) / length;
    DevelopmentNominalPoint point;
    point.x = points[lo].x + ratio * (points[lo + 1].x - points[lo].x);
    point.y = points[lo].y + ratio * (points[lo + 1].y - points[lo].y);
    return point;
}

double pathHeading(const std::vector<DevelopmentNominalPoint>& points,
                   const std::vector<double>& path_progress,
                   double query) {
    const double path_length = path_progress.back();
    const double delta = std::min(0.05, 0.1 * path_length);
    const DevelopmentNominalPoint before = interpolatePosition(
        points, path_progress, std::max(0.0, query - delta));
    const DevelopmentNominalPoint after = interpolatePosition(
        points, path_progress, std::min(path_length, query + delta));
    return std::atan2(after.y - before.y, after.x - before.x);
}

bool speedSchedule(double path_length,
                   const DevelopmentNominalGeneratorConfig& config,
                   std::vector<double>& velocities,
                   std::vector<double>& progress,
                   std::string& detail) {
    if (path_length / config.cruise_speed <= config.ramp_sec) {
        detail = "ramp is too long for requested cruise speed";
        return false;
    }
    const double cruise_sec =
        path_length / config.cruise_speed - config.ramp_sec;
    const double nominal_duration = 2.0 * config.ramp_sec + cruise_sec;
    const double raw_steps = std::ceil(nominal_duration / config.dt);
    if (!finite(raw_steps) || raw_steps < 1.0 ||
        raw_steps >= static_cast<double>(kMaximumSamples)) {
        detail = "nominal schedule exceeds sample limit";
        return false;
    }
    const std::size_t steps = static_cast<std::size_t>(raw_steps);
    std::vector<double> shape(steps + 1, 0.0);
    for (std::size_t i = 0; i <= steps; ++i) {
        const double time_sec = static_cast<double>(i) * config.dt;
        if (time_sec <= config.ramp_sec) {
            shape[i] = 0.5 * (1.0 -
                std::cos(kPi * time_sec / config.ramp_sec));
        } else if (time_sec <= config.ramp_sec + cruise_sec) {
            shape[i] = 1.0;
        } else if (time_sec <= nominal_duration) {
            const double phase =
                (time_sec - config.ramp_sec - cruise_sec) / config.ramp_sec;
            shape[i] = 0.5 * (1.0 + std::cos(kPi * phase));
        }
    }
    double area = 0.0;
    for (std::size_t i = 0; i < steps; ++i) {
        area += 0.5 * (shape[i] + shape[i + 1]) * config.dt;
    }
    if (!positive(area)) {
        detail = "nominal speed schedule has zero area";
        return false;
    }
    const double actual_speed = path_length / area;
    velocities.resize(steps + 1);
    progress.assign(steps + 1, 0.0);
    for (std::size_t i = 0; i <= steps; ++i) {
        velocities[i] = actual_speed * shape[i];
    }
    for (std::size_t i = 0; i < steps; ++i) {
        progress[i + 1] = progress[i] +
            0.5 * (velocities[i] + velocities[i + 1]) * config.dt;
    }
    progress.back() = path_length;
    return true;
}

PhaseNominalSample makeSample(std::size_t index,
                              const NominalDynamicsState& state,
                              const NominalDynamicsControl& control,
                              const NominalDynamicsState& next_state,
                              const DevelopmentNominalGeneratorConfig& config) {
    PhaseNominalSample sample;
    sample.index = index;
    sample.t = static_cast<double>(index) * config.dt;
    sample.s = state.s;
    sample.x = state.x;
    sample.y = state.y;
    sample.yaw = state.yaw;
    sample.v = state.v;
    sample.omega = state.omega;
    sample.eta_x = state.eta_x;
    sample.eta_x_dot = state.eta_x_dot;
    sample.eta_y = state.eta_y;
    sample.eta_y_dot = state.eta_y_dot;
    sample.a = control.a;
    sample.alpha = control.alpha;
    sample.v_s = control.v_s;
    sample.u_pub_v = next_state.v;
    sample.u_pub_omega = next_state.omega;
    sample.kappa_v = next_state.v;
    sample.kappa_omega = next_state.omega;
    sample.radii = config.radii;
    return sample;
}

bool validConfig(const DevelopmentNominalGeneratorConfig& config,
                 std::string& detail) {
    const bool positive_values =
        positive(config.dt) && positive(config.cruise_speed) &&
        positive(config.ramp_sec) && positive(config.lookahead) &&
        positive(config.heading_gain) && positive(config.omega_max) &&
        positive(config.alpha_max) && positive(config.omega_n) &&
        positive(config.kappa_x) && positive(config.kappa_y) &&
        positive(config.zero_hold_sec) &&
        positive(config.terminal_eta_norm_max) &&
        positive(config.terminal_eta_dot_norm_max) &&
        positiveRadii(config.radii);
    if (!positive_values || !finite(config.damping_ratio) ||
        config.damping_ratio < 0.0 || config.damping_ratio > 1.0) {
        detail = "nonfinite or out-of-range numeric configuration";
        return false;
    }
    if (!metadataText(config.contract_id) || !metadataText(config.frame_id) ||
        !metadataText(config.path_topic)) {
        detail = "invalid artifact text metadata";
        return false;
    }
    if (!lowercaseSha256(config.source_bag_sha256)) {
        detail = "invalid source bag SHA-256";
        return false;
    }
    return true;
}

}  // namespace

DevelopmentNominalGenerationResult DevelopmentNominalGenerator::generate(
    const std::vector<DevelopmentNominalPoint>& input_points,
    const DevelopmentNominalGeneratorConfig& config) const {
    std::string detail;
    if (!validConfig(config, detail)) {
        return failure("INVALID_GENERATOR_CONFIG", detail);
    }
    const std::vector<DevelopmentNominalPoint> points = cleanPoints(input_points);
    if (points.size() < 3) {
        return failure("INVALID_REFERENCE_PATH",
                       "reference path needs at least three distinct points");
    }
    const std::vector<double> path_progress = cumulativeProgress(points);
    const double path_length = path_progress.back();
    if (!finite(path_length) || path_length <= 0.1) {
        return failure("INVALID_REFERENCE_PATH", "reference path is too short");
    }
    std::vector<double> velocities;
    std::vector<double> progress;
    if (!speedSchedule(path_length, config, velocities, progress, detail)) {
        return failure("INVALID_SPEED_SCHEDULE", detail);
    }

    NominalDynamicsModel model;
    model.dt = config.dt;
    model.two_zeta_omega_n =
        2.0 * config.damping_ratio * config.omega_n;
    model.omega_n_sq = config.omega_n * config.omega_n;
    model.kappa_x = config.kappa_x;
    model.kappa_y = config.kappa_y;
    NominalDynamicsState state;
    state.x = points.front().x;
    state.y = points.front().y;
    state.yaw = pathHeading(points, path_progress, 0.0);
    state.v = velocities.front();

    DevelopmentNominalGenerationResult result;
    result.samples.reserve(velocities.size() + 1024);
    for (std::size_t index = 0; index + 1 < velocities.size(); ++index) {
        const double next_v = velocities[index + 1];
        const double next_s = progress[index + 1];
        const DevelopmentNominalPoint target = interpolatePosition(
            points, path_progress,
            std::min(path_length, state.s + config.lookahead));
        const double target_heading =
            std::atan2(target.y - state.y, target.x - state.x);
        const double omega_target = clamp(
            config.heading_gain * wrap(target_heading - state.yaw),
            -config.omega_max, config.omega_max);
        NominalDynamicsControl control;
        control.a = (next_v - state.v) / config.dt;
        control.alpha = clamp(
            (omega_target - state.omega) / config.dt,
            -config.alpha_max, config.alpha_max);
        control.v_s = (next_s - state.s) / config.dt;
        const NominalDynamicsState next_state =
            phaseNominalRk4Step(state, control, model);
        result.samples.push_back(
            makeSample(index, state, control, next_state, config));
        const DevelopmentNominalPoint nominal =
            interpolatePosition(points, path_progress, state.s);
        result.max_path_deviation = std::max(
            result.max_path_deviation,
            std::hypot(state.x - nominal.x, state.y - nominal.y));
        state = next_state;
    }
    if (result.max_path_deviation > 0.20) {
        return failure("NOMINAL_PATH_DEVIATION_EXCEEDED",
                       formatDouble(result.max_path_deviation));
    }

    while (std::abs(state.omega) > 1.0e-10) {
        NominalDynamicsControl control;
        control.alpha = clamp(-state.omega / config.dt,
                              -config.alpha_max, config.alpha_max);
        const NominalDynamicsState next_state =
            phaseNominalRk4Step(state, control, model);
        result.samples.push_back(makeSample(
            result.samples.size(), state, control, next_state, config));
        state = next_state;
        if (result.samples.size() > kMaximumSamples) {
            return failure("ANGULAR_SETTLING_FAILED", "sample limit exceeded");
        }
    }

    const std::size_t zero_hold_begin = result.samples.size();
    const double raw_minimum_hold = std::ceil(config.zero_hold_sec / config.dt);
    if (!finite(raw_minimum_hold) ||
        raw_minimum_hold >= static_cast<double>(kMaximumSamples)) {
        return failure("INVALID_GENERATOR_CONFIG", "zero hold exceeds sample limit");
    }
    const std::size_t minimum_hold_rows = std::max<std::size_t>(
        5, static_cast<std::size_t>(raw_minimum_hold) + 1);
    const double raw_maximum_settle_rows = std::ceil(20.0 / config.dt);
    const std::size_t maximum_settle_rows =
        !finite(raw_maximum_settle_rows) ||
        raw_maximum_settle_rows >= static_cast<double>(kMaximumSamples)
        ? kMaximumSamples
        : static_cast<std::size_t>(raw_maximum_settle_rows);
    while (true) {
        const NominalDynamicsControl control;
        const NominalDynamicsState next_state =
            phaseNominalRk4Step(state, control, model);
        result.samples.push_back(makeSample(
            result.samples.size(), state, control, next_state, config));
        state = next_state;
        const std::size_t hold_rows = result.samples.size() - zero_hold_begin;
        const bool settled =
            std::hypot(state.eta_x, state.eta_y) <=
                config.terminal_eta_norm_max &&
            std::hypot(state.eta_x_dot, state.eta_y_dot) <=
                config.terminal_eta_dot_norm_max;
        if (hold_rows + 1 >= minimum_hold_rows && settled) {
            break;
        }
        if (hold_rows > maximum_settle_rows ||
            result.samples.size() > kMaximumSamples) {
            return failure("LIQUID_SETTLING_FAILED",
                           "liquid did not settle within 20 seconds");
        }
    }
    const NominalDynamicsControl final_control;
    result.samples.push_back(makeSample(
        result.samples.size(), state, final_control, state, config));
    result.zero_hold_steps = result.samples.size() - zero_hold_begin;
    result.path_length = path_length;

    result.metadata = {
        {"schema", "phase_rejoin_empirical_v2"},
        {"evidence_level", "development_only"},
        {"source", "development_dynamics_consistent_nominal"},
        {"contract_id", trim(config.contract_id)},
        {"frame_id", trim(config.frame_id)},
        {"dt", formatDouble(config.dt)},
        {"path_length", formatDouble(path_length)},
        {"terminal_contract", "stop_settle_zero_hold_v1"},
        {"recovery_contract", "nominal_command_v1"},
        {"terminal_zero_hold_steps", std::to_string(result.zero_hold_steps)},
        {"terminal_eta_norm_max", formatDouble(config.terminal_eta_norm_max)},
        {"terminal_eta_dot_norm_max",
         formatDouble(config.terminal_eta_dot_norm_max)},
        {"two_zeta_omega_n", formatDouble(model.two_zeta_omega_n)},
        {"omega_n_sq", formatDouble(model.omega_n_sq)},
        {"kappa_x", formatDouble(model.kappa_x)},
        {"kappa_y", formatDouble(model.kappa_y)},
        {"dynamics_tolerance", "1e-8"},
        {"artifact_role", "interface_smoke_only"},
        {"nominal_sequence_kind",
         "uniform_dynamics_consistent_complete_tail"},
        {"offline_slosh_ocp", "false"},
        {"hardware_formal_release", "false"},
        {"paper_main_result_eligible", "false"},
        {"gate_evidence", "none_operator_supplied_development_radii"},
        {"recovery_policy_source",
         "nominal_next_command_development_fallback"},
        {"recovery_policy_evidence", "none_development_only"},
        {"source_bag_sha256", config.source_bag_sha256},
        {"path_topic", trim(config.path_topic)},
        {"max_nominal_path_deviation_m",
         formatDouble(result.max_path_deviation)},
    };
    result.success = true;
    result.status = "OK";
    return result;
}

}  // namespace spmpc_local_planner
