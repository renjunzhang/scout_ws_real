#include "spmpc_local_planner/reference/speed_profile.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace spmpc_local_planner {
namespace {

std::string trimCopy(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<std::string> splitCsvSimple(const std::string& line) {
    std::vector<std::string> cells;
    std::stringstream stream(line);
    std::string cell;
    while (std::getline(stream, cell, ',')) {
        cells.push_back(trimCopy(cell));
    }
    return cells;
}

bool parseDoubleStrict(const std::string& text, double& value) {
    const std::string trimmed = trimCopy(text);
    if (trimmed.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(trimmed.c_str(), &end);
    if (errno != 0 || end == trimmed.c_str() || *end != '\0' ||
        !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

int findColumn(const std::vector<std::string>& header,
               const std::vector<std::string>& names) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        for (const auto& name : names) {
            if (header[i] == name) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

}  // namespace

SpeedProfileLoadResult SpeedProfile::loadCsv(const std::string& path) {
    clear();
    SpeedProfileLoadResult result;
    if (path.empty()) {
        result.status = "PROFILE_NOT_CONFIGURED";
        result.detail = "empty profile path";
        return result;
    }

    std::ifstream input(path);
    if (!input.is_open()) {
        result.status = "PROFILE_OPEN_FAILED";
        result.detail = path;
        return result;
    }

    std::vector<SpeedProfileSample> samples;
    bool header_parsed = false;
    bool have_header = false;
    int progress_column = 0;
    int speed_column = 1;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed.front() == '#') {
            continue;
        }
        const auto cells = splitCsvSimple(trimmed);
        if (cells.empty()) {
            continue;
        }

        if (!header_parsed) {
            double first = 0.0;
            double second = 0.0;
            if (cells.size() >= 2 && parseDoubleStrict(cells[0], first) &&
                parseDoubleStrict(cells[1], second)) {
                header_parsed = true;
            } else {
                header_parsed = true;
                have_header = true;
                progress_column = findColumn(
                    cells, {"s_m", "s", "progress_s_m"});
                speed_column = findColumn(
                    cells,
                    {"v_ref_map_mps", "v_ref_current_mps", "v_ref_mps", "v_safe_mps"});
                if (progress_column < 0 || speed_column < 0) {
                    result.status = "PROFILE_HEADER_INVALID";
                    result.detail =
                        "header must include s_m and v_ref_map_mps aliases";
                    return result;
                }
                continue;
            }
        }

        const int progress_index = have_header ? progress_column : 0;
        const int speed_index = have_header ? speed_column : 1;
        if (static_cast<int>(cells.size()) <=
            std::max(progress_index, speed_index)) {
            ++result.skipped_rows;
            continue;
        }

        double progress_m = 0.0;
        double speed_mps = 0.0;
        if (!parseDoubleStrict(cells[progress_index], progress_m) ||
            !parseDoubleStrict(cells[speed_index], speed_mps) ||
            progress_m < 0.0 || speed_mps < 0.0) {
            ++result.skipped_rows;
            continue;
        }
        samples.push_back({progress_m, speed_mps});
    }

    if (samples.empty()) {
        result.status = "PROFILE_EMPTY";
        result.detail = "no valid samples";
        return result;
    }

    std::sort(samples.begin(), samples.end(), [](const SpeedProfileSample& lhs,
                                                  const SpeedProfileSample& rhs) {
        return lhs.progress_m < rhs.progress_m;
    });
    samples_.reserve(samples.size());
    for (const auto& sample : samples) {
        if (!samples_.empty() &&
            std::abs(sample.progress_m - samples_.back().progress_m) < 1e-9) {
            samples_.back() = sample;
        } else {
            samples_.push_back(sample);
        }
    }

    source_path_ = path;
    result.success = true;
    result.status = "OK";
    result.accepted_rows = samples_.size();
    return result;
}

void SpeedProfile::clear() {
    source_path_.clear();
    samples_.clear();
}

bool SpeedProfile::lookup(double progress_m, double& speed_mps) const {
    if (samples_.empty() || !std::isfinite(progress_m)) {
        return false;
    }
    if (progress_m <= samples_.front().progress_m) {
        speed_mps = samples_.front().speed_mps;
        return true;
    }
    if (progress_m >= samples_.back().progress_m) {
        speed_mps = samples_.back().speed_mps;
        return true;
    }

    const auto upper = std::lower_bound(
        samples_.begin(), samples_.end(), progress_m,
        [](const SpeedProfileSample& sample, double value) {
            return sample.progress_m < value;
        });
    if (upper == samples_.begin() || upper == samples_.end()) {
        return false;
    }
    const auto lower = std::prev(upper);
    const double interval = upper->progress_m - lower->progress_m;
    if (interval <= 1e-9) {
        speed_mps = upper->speed_mps;
        return true;
    }
    const double ratio = (progress_m - lower->progress_m) / interval;
    speed_mps = lower->speed_mps + ratio * (upper->speed_mps - lower->speed_mps);
    return std::isfinite(speed_mps);
}

}  // namespace spmpc_local_planner
