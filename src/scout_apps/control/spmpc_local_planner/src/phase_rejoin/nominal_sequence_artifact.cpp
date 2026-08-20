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

const char* const kSchema = "phase_rejoin_empirical_v1";

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
    if (metadata["schema"] != kSchema) {
        clear();
        return failure("UNSUPPORTED_SCHEMA", metadata["schema"]);
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

    // The development proxy publishes /clock at 50 Hz while the controller
    // runs at 30 Hz, yielding a bounded 40/40/20 ms timer pattern.  Permit the
    // corresponding local quantization, while the cumulative phase bound below
    // still rejects a stream that drifts away from the declared nominal dt.
    const double period_tolerance =
        std::max(1e-4, 0.40 * parsed_metadata.dt) + 1e-9;
    const double phase_tolerance =
        std::max(1e-4, parsed_metadata.dt) + 1e-9;
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
