#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace {

const char* const kSchemaV1 = "phase_rejoin_empirical_v1";
const char* const kSchemaV2 = "phase_rejoin_empirical_v2";
const char* const kTerminalContractV1 = "stop_settle_zero_hold_v1";
const char* const kRecoveryContractV1 = "nominal_command_v1";

const std::vector<std::string>& expectedHeader() {
    static const std::vector<std::string> header = {
        "index", "t", "s", "x", "y", "yaw", "v", "omega",
        "eta_x", "eta_x_dot", "eta_y", "eta_y_dot",
        "a", "alpha", "v_s", "u_pub_v", "u_pub_omega",
        "kappa_v", "kappa_omega",
        "r_x", "r_y", "r_yaw", "r_v", "r_omega",
        "r_eta_x", "r_eta_x_dot", "r_eta_y", "r_eta_y_dot",
    };
    return header;
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

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> values;
    std::stringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) {
        values.push_back(trim(value));
    }
    if (!line.empty() && line.back() == ',') {
        values.emplace_back();
    }
    return values;
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

bool parseIndex(const std::string& text, std::size_t& value) {
    const std::string input = trim(text);
    if (input.empty() || input[0] == '-') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(input.c_str(), &end, 10);
    if (errno != 0 || end == input.c_str() || *end != '\0' ||
        parsed > static_cast<unsigned long long>(
                     std::numeric_limits<std::size_t>::max())) {
        return false;
    }
    value = static_cast<std::size_t>(parsed);
    return true;
}

bool parsePositiveIndex(const std::string& text, std::size_t& value) {
    return parseIndex(text, value) && value > 0;
}

bool positiveRadii(const EmpiricalRecoveryRadii& radii) {
    return radii.x > 0.0 && radii.y > 0.0 && radii.yaw > 0.0 &&
           radii.v > 0.0 && radii.omega > 0.0 && radii.eta_x > 0.0 &&
           radii.eta_x_dot > 0.0 && radii.eta_y > 0.0 &&
           radii.eta_y_dot > 0.0;
}

NominalArtifactLoadResult failure(const std::string& status,
                                  const std::string& detail) {
    NominalArtifactLoadResult result;
    result.status = status;
    result.detail = detail;
    return result;
}

struct AugmentedState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double s = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

AugmentedState addScaled(const AugmentedState& state,
                         const AugmentedState& derivative,
                         double scale) {
    AugmentedState out;
    out.x = state.x + scale * derivative.x;
    out.y = state.y + scale * derivative.y;
    out.yaw = state.yaw + scale * derivative.yaw;
    out.v = state.v + scale * derivative.v;
    out.omega = state.omega + scale * derivative.omega;
    out.s = state.s + scale * derivative.s;
    out.eta_x = state.eta_x + scale * derivative.eta_x;
    out.eta_x_dot = state.eta_x_dot + scale * derivative.eta_x_dot;
    out.eta_y = state.eta_y + scale * derivative.eta_y;
    out.eta_y_dot = state.eta_y_dot + scale * derivative.eta_y_dot;
    return out;
}

AugmentedState derivative(const AugmentedState& state,
                          const PhaseNominalSample& control,
                          const NominalArtifactMetadata& metadata) {
    AugmentedState out;
    out.x = state.v * std::cos(state.yaw);
    out.y = state.v * std::sin(state.yaw);
    out.yaw = state.omega;
    out.v = control.a;
    out.omega = control.alpha;
    out.s = control.v_s;
    out.eta_x = state.eta_x_dot;
    out.eta_x_dot =
        -metadata.two_zeta_omega_n * state.eta_x_dot -
        metadata.omega_n_sq * state.eta_x -
        metadata.kappa_x * control.a;
    out.eta_y = state.eta_y_dot;
    out.eta_y_dot =
        -metadata.two_zeta_omega_n * state.eta_y_dot -
        metadata.omega_n_sq * state.eta_y -
        metadata.kappa_y * state.v * state.omega;
    return out;
}

AugmentedState rk4Step(const PhaseNominalSample& sample,
                       const NominalArtifactMetadata& metadata) {
    AugmentedState state;
    state.x = sample.x;
    state.y = sample.y;
    state.yaw = sample.yaw;
    state.v = sample.v;
    state.omega = sample.omega;
    state.s = sample.s;
    state.eta_x = sample.eta_x;
    state.eta_x_dot = sample.eta_x_dot;
    state.eta_y = sample.eta_y;
    state.eta_y_dot = sample.eta_y_dot;

    const double half_dt = 0.5 * metadata.dt;
    const AugmentedState k1 = derivative(state, sample, metadata);
    const AugmentedState k2 = derivative(
        addScaled(state, k1, half_dt), sample, metadata);
    const AugmentedState k3 = derivative(
        addScaled(state, k2, half_dt), sample, metadata);
    const AugmentedState k4 = derivative(
        addScaled(state, k3, metadata.dt), sample, metadata);

    AugmentedState out = state;
    const double scale = metadata.dt / 6.0;
#define SPMPC_RK4_FIELD(field) \
    out.field += scale * (k1.field + 2.0 * k2.field + \
                          2.0 * k3.field + k4.field)
    SPMPC_RK4_FIELD(x);
    SPMPC_RK4_FIELD(y);
    SPMPC_RK4_FIELD(yaw);
    SPMPC_RK4_FIELD(v);
    SPMPC_RK4_FIELD(omega);
    SPMPC_RK4_FIELD(s);
    SPMPC_RK4_FIELD(eta_x);
    SPMPC_RK4_FIELD(eta_x_dot);
    SPMPC_RK4_FIELD(eta_y);
    SPMPC_RK4_FIELD(eta_y_dot);
#undef SPMPC_RK4_FIELD
    return out;
}

double angleError(double lhs, double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

bool within(double lhs, double rhs, double tolerance) {
    return std::abs(lhs - rhs) <= tolerance;
}

NominalArtifactLoadResult validateV2Transitions(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    const double tolerance = metadata.dynamics_tolerance;
    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        const PhaseNominalSample& current = samples[i];
        const PhaseNominalSample& next = samples[i + 1];
        const AugmentedState predicted = rk4Step(current, metadata);
        const bool state_consistent =
            within(predicted.x, next.x, tolerance) &&
            within(predicted.y, next.y, tolerance) &&
            std::abs(angleError(predicted.yaw, next.yaw)) <= tolerance &&
            within(predicted.v, next.v, tolerance) &&
            within(predicted.omega, next.omega, tolerance) &&
            within(predicted.s, next.s, tolerance) &&
            within(predicted.eta_x, next.eta_x, tolerance) &&
            within(predicted.eta_x_dot, next.eta_x_dot, tolerance) &&
            within(predicted.eta_y, next.eta_y, tolerance) &&
            within(predicted.eta_y_dot, next.eta_y_dot, tolerance);
        if (!state_consistent) {
            return failure("DYNAMICS_TRANSITION_MISMATCH",
                           "index " + std::to_string(i));
        }
        if (!within(current.u_pub_v, predicted.v, tolerance) ||
            !within(current.u_pub_omega, predicted.omega, tolerance)) {
            return failure("PUBLISHED_COMMAND_MISMATCH",
                           "index " + std::to_string(i));
        }
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    return result;
}

NominalArtifactLoadResult validateV2RecoveryCommands(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    if (metadata.recovery_contract != kRecoveryContractV1) {
        return failure("UNSUPPORTED_RECOVERY_CONTRACT",
                       metadata.recovery_contract);
    }

    // V2's development recovery action is deliberately not an independent,
    // unverified policy: it must be the same command whose one-step dynamics
    // have already been checked as u_pub.  The CSV contract contains no
    // robot-specific v/omega limits, so runtime command and residual bounds
    // remain the responsibility of the solver/coordinator/publish chain.
    const double tolerance = metadata.dynamics_tolerance;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const PhaseNominalSample& sample = samples[i];
        if (!within(sample.kappa_v, sample.u_pub_v, tolerance) ||
            !within(sample.kappa_omega, sample.u_pub_omega, tolerance)) {
            return failure("RECOVERY_COMMAND_MISMATCH",
                           "index " + std::to_string(i));
        }
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    return result;
}

NominalArtifactLoadResult validateV2TerminalTail(
    const std::vector<PhaseNominalSample>& samples,
    const NominalArtifactMetadata& metadata) {
    if (metadata.terminal_contract != kTerminalContractV1) {
        return failure("UNSUPPORTED_TERMINAL_CONTRACT",
                       metadata.terminal_contract);
    }
    if (metadata.terminal_zero_hold_steps < 5 ||
        metadata.terminal_zero_hold_steps >= samples.size()) {
        return failure("INVALID_ZERO_HOLD_LENGTH",
                       std::to_string(metadata.terminal_zero_hold_steps));
    }
    const std::size_t begin = samples.size() -
        metadata.terminal_zero_hold_steps;
    const double tolerance = metadata.dynamics_tolerance;
    for (std::size_t i = begin; i < samples.size(); ++i) {
        const PhaseNominalSample& sample = samples[i];
        const double values[] = {
            sample.v, sample.omega, sample.a, sample.alpha, sample.v_s,
            sample.u_pub_v, sample.u_pub_omega,
            sample.kappa_v, sample.kappa_omega,
        };
        for (double value : values) {
            if (std::abs(value) > tolerance) {
                return failure("ZERO_HOLD_COMMAND_NONZERO",
                               "index " + std::to_string(i));
            }
        }
        if (!within(sample.s, metadata.path_length, tolerance)) {
            return failure("ZERO_HOLD_PROGRESS_MISMATCH",
                           "index " + std::to_string(i));
        }
    }
    const PhaseNominalSample& final = samples.back();
    if (std::hypot(final.eta_x, final.eta_y) >
            metadata.terminal_eta_norm_max + tolerance ||
        std::hypot(final.eta_x_dot, final.eta_y_dot) >
            metadata.terminal_eta_dot_norm_max + tolerance) {
        return failure("TERMINAL_LIQUID_NOT_SETTLED",
                       "index " + std::to_string(final.index));
    }
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    return result;
}

}  // namespace

void NominalSequenceArtifact::clear() {
    valid_ = false;
    path_.clear();
    metadata_ = NominalArtifactMetadata{};
    samples_.clear();
}

const PhaseNominalSample* NominalSequenceArtifact::sample(std::size_t index) const {
    return index < samples_.size() ? &samples_[index] : nullptr;
}

NominalArtifactLoadResult NominalSequenceArtifact::loadCsv(
    const std::string& path) {
    clear();
    std::ifstream input(path);
    if (!input.is_open()) {
        return failure("OPEN_FAILED", path);
    }

    std::map<std::string, std::string> metadata;
    bool header_seen = false;
    std::size_t line_number = 0;
    std::string line;
    while (std::getline(input, line)) {
        ++line_number;
        const std::string clean = trim(line);
        if (clean.empty()) {
            continue;
        }
        if (clean[0] == '#') {
            const std::string meta = trim(clean.substr(1));
            const std::size_t separator = meta.find('=');
            if (separator == std::string::npos) {
                return failure("INVALID_METADATA",
                               "line " + std::to_string(line_number));
            }
            const std::string key = trim(meta.substr(0, separator));
            const std::string value = trim(meta.substr(separator + 1));
            if (key.empty() || value.empty() || metadata.count(key) != 0) {
                return failure("INVALID_METADATA",
                               "line " + std::to_string(line_number));
            }
            metadata[key] = value;
            continue;
        }

        const std::vector<std::string> columns = splitCsv(clean);
        if (!header_seen) {
            if (columns != expectedHeader()) {
                return failure("HEADER_MISMATCH",
                               "line " + std::to_string(line_number));
            }
            header_seen = true;
            continue;
        }
        if (columns.size() != expectedHeader().size()) {
            return failure("COLUMN_COUNT_MISMATCH",
                           "line " + std::to_string(line_number));
        }

        PhaseNominalSample sample;
        std::size_t c = 0;
        bool ok = parseIndex(columns[c++], sample.index);
        double* values[] = {
            &sample.t, &sample.s, &sample.x, &sample.y, &sample.yaw,
            &sample.v, &sample.omega,
            &sample.eta_x, &sample.eta_x_dot, &sample.eta_y,
            &sample.eta_y_dot, &sample.a, &sample.alpha, &sample.v_s,
            &sample.u_pub_v, &sample.u_pub_omega,
            &sample.kappa_v, &sample.kappa_omega,
            &sample.radii.x, &sample.radii.y, &sample.radii.yaw,
            &sample.radii.v, &sample.radii.omega,
            &sample.radii.eta_x, &sample.radii.eta_x_dot,
            &sample.radii.eta_y, &sample.radii.eta_y_dot,
        };
        for (double* value : values) {
            ok = parseDouble(columns[c++], *value) && ok;
        }
        if (!ok || c != columns.size()) {
            return failure("NONFINITE_OR_INVALID_VALUE",
                           "line " + std::to_string(line_number));
        }
        if (sample.index != samples_.size()) {
            return failure("INDEX_NOT_CONTIGUOUS",
                           "line " + std::to_string(line_number));
        }
        if (!positiveRadii(sample.radii)) {
            return failure("NONPOSITIVE_GATE_RADIUS",
                           "line " + std::to_string(line_number));
        }
        if (!samples_.empty()) {
            const PhaseNominalSample& previous = samples_.back();
            if (sample.t <= previous.t || sample.s + 1e-9 < previous.s) {
                return failure("NONMONOTONIC_SEQUENCE",
                               "line " + std::to_string(line_number));
            }
        } else if (sample.t < 0.0 || sample.s < 0.0) {
            return failure("NEGATIVE_ORIGIN",
                           "line " + std::to_string(line_number));
        }
        samples_.push_back(sample);
    }

    if (!header_seen || samples_.size() < 2) {
        clear();
        return failure("EMPTY_OR_SHORT_ARTIFACT", path);
    }

    const char* required[] = {
        "schema", "evidence_level", "source", "contract_id",
        "frame_id", "dt", "path_length",
    };
    for (const char* key : required) {
        if (metadata.count(key) == 0) {
            clear();
            return failure("MISSING_METADATA", key);
        }
    }
    const bool schema_v1 = metadata["schema"] == kSchemaV1;
    const bool schema_v2 = metadata["schema"] == kSchemaV2;
    if (!schema_v1 && !schema_v2) {
        clear();
        return failure("UNSUPPORTED_SCHEMA", metadata["schema"]);
    }

    if (schema_v2) {
        const char* required_v2[] = {
            "terminal_contract", "recovery_contract",
            "terminal_zero_hold_steps",
            "terminal_eta_norm_max", "terminal_eta_dot_norm_max",
            "two_zeta_omega_n", "omega_n_sq", "kappa_x", "kappa_y",
            "dynamics_tolerance",
        };
        for (const char* key : required_v2) {
            if (metadata.count(key) == 0) {
                clear();
                return failure("MISSING_METADATA", key);
            }
        }
    }

    NominalArtifactMetadata parsed_metadata;
    parsed_metadata.schema = metadata["schema"];
    parsed_metadata.source = metadata["source"];
    parsed_metadata.contract_id = metadata["contract_id"];
    parsed_metadata.frame_id = metadata["frame_id"];
    if (!parsePhaseRejoinEvidenceLevel(metadata["evidence_level"],
                                       parsed_metadata.evidence_level)) {
        clear();
        return failure("INVALID_EVIDENCE_LEVEL", metadata["evidence_level"]);
    }
    if (!parseDouble(metadata["dt"], parsed_metadata.dt) ||
        !parseDouble(metadata["path_length"], parsed_metadata.path_length) ||
        parsed_metadata.dt <= 0.0 || parsed_metadata.path_length <= 0.0 ||
        parsed_metadata.contract_id.empty() || parsed_metadata.frame_id.empty()) {
        clear();
        return failure("INVALID_METADATA_VALUE", path);
    }

    if (schema_v2) {
        parsed_metadata.terminal_contract = metadata["terminal_contract"];
        parsed_metadata.recovery_contract = metadata["recovery_contract"];
        const bool valid_v2_metadata =
            parsePositiveIndex(metadata["terminal_zero_hold_steps"],
                               parsed_metadata.terminal_zero_hold_steps) &&
            parseDouble(metadata["terminal_eta_norm_max"],
                        parsed_metadata.terminal_eta_norm_max) &&
            parseDouble(metadata["terminal_eta_dot_norm_max"],
                        parsed_metadata.terminal_eta_dot_norm_max) &&
            parseDouble(metadata["two_zeta_omega_n"],
                        parsed_metadata.two_zeta_omega_n) &&
            parseDouble(metadata["omega_n_sq"],
                        parsed_metadata.omega_n_sq) &&
            parseDouble(metadata["kappa_x"], parsed_metadata.kappa_x) &&
            parseDouble(metadata["kappa_y"], parsed_metadata.kappa_y) &&
            parseDouble(metadata["dynamics_tolerance"],
                        parsed_metadata.dynamics_tolerance) &&
            parsed_metadata.terminal_eta_norm_max >= 0.0 &&
            parsed_metadata.terminal_eta_dot_norm_max >= 0.0 &&
            parsed_metadata.two_zeta_omega_n >= 0.0 &&
            parsed_metadata.omega_n_sq > 0.0 &&
            parsed_metadata.kappa_x > 0.0 &&
            parsed_metadata.kappa_y > 0.0 &&
            parsed_metadata.dynamics_tolerance >= 1e-12 &&
            parsed_metadata.dynamics_tolerance <= 1e-3;
        if (!valid_v2_metadata) {
            clear();
            return failure("INVALID_V2_METADATA_VALUE", path);
        }
    }

    // The development proxy publishes /clock at 50 Hz while the controller
    // runs at 30 Hz, yielding a bounded 40/40/20 ms timer pattern.  Permit the
    // corresponding local quantization, while the cumulative phase bound below
    // still rejects a stream that drifts away from the declared nominal dt.
    const double period_tolerance = schema_v2
        ? std::max(1e-9, 1e-7 * parsed_metadata.dt)
        : std::max(1e-4, 0.40 * parsed_metadata.dt) + 1e-9;
    const double phase_tolerance = schema_v2
        ? std::max(1e-9, 1e-7 * parsed_metadata.dt)
        : std::max(1e-4, parsed_metadata.dt) + 1e-9;
    const double first_time = samples_.front().t;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
        const double period = samples_[i].t - samples_[i - 1].t;
        if (std::abs(period - parsed_metadata.dt) > period_tolerance) {
            clear();
            return failure("SAMPLE_PERIOD_MISMATCH",
                           "index " + std::to_string(i));
        }
        const double nominal_time = first_time +
            static_cast<double>(i) * parsed_metadata.dt;
        if (std::abs(samples_[i].t - nominal_time) > phase_tolerance) {
            clear();
            return failure("SAMPLE_PHASE_DRIFT",
                           "index " + std::to_string(i));
        }
    }
    // The artifact is a complete nominal tail, not an arbitrary path prefix.
    // Keeping the tolerance small allows the final discrete sample to fall one
    // nominal step short of the geometric endpoint without accepting a
    // truncated sequence whose metadata merely claims the full path length.
    constexpr double kEndpointToleranceM = 0.100001;
    if (std::abs(samples_.back().s - parsed_metadata.path_length) >
        kEndpointToleranceM) {
        clear();
        return failure("PATH_LENGTH_MISMATCH", path);
    }

    if (schema_v2) {
        const NominalArtifactLoadResult transition_result =
            validateV2Transitions(samples_, parsed_metadata);
        if (!transition_result.success) {
            clear();
            return transition_result;
        }
        const NominalArtifactLoadResult recovery_result =
            validateV2RecoveryCommands(samples_, parsed_metadata);
        if (!recovery_result.success) {
            clear();
            return recovery_result;
        }
        const NominalArtifactLoadResult terminal_result =
            validateV2TerminalTail(samples_, parsed_metadata);
        if (!terminal_result.success) {
            clear();
            return terminal_result;
        }
        parsed_metadata.complete_terminal_tail = true;
    }

    path_ = path;
    metadata_ = parsed_metadata;
    valid_ = true;
    NominalArtifactLoadResult result;
    result.success = true;
    result.status = "OK";
    result.detail = path;
    return result;
}

}  // namespace spmpc_local_planner
