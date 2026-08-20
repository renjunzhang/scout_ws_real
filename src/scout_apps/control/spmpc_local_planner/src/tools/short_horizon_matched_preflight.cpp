#include "spmpc_local_planner/tools/short_horizon_matched_preflight.h"

#include "spmpc_local_planner/config/variant_config.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>

namespace spmpc_local_planner {
namespace tools {
namespace {

using FieldMap = std::map<std::string, std::string>;

const char* const kMatched0 = "B_slosh_matched0";
const char* const kMatched5 = "B_slosh_matched5";

const std::set<std::string>& requiredFields() {
    static const std::set<std::string> fields = {
        "primitive_mode",
        "slosh_constraint_enable",
        "slosh_cost_horizon_steps",
        "slosh_cost_tail_discount",
        "slosh_enable",
        "smooth_priority_enable",
        "v_ref",
        "w_accel",
        "w_alpha",
        "w_contour",
        "w_control",
        "w_du_a",
        "w_du_vs",
        "w_lag",
        "w_progress",
        "w_slosh",
        "w_smooth",
        "w_v",
        "w_vs",
    };
    return fields;
}

std::string trim(const std::string& value) {
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) return std::string{};
    const std::size_t last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1);
}

bool startsWith(const std::string& value, const std::string& prefix) {
    return value.size() >= prefix.size() &&
           value.compare(0, prefix.size(), prefix) == 0;
}

bool parseBool(const std::string& value, bool& output) {
    if (value == "true") {
        output = true;
        return true;
    }
    if (value == "false") {
        output = false;
        return true;
    }
    return false;
}

bool parseDouble(const std::string& value, double& output) {
    if (value.empty()) return false;
    char* end = nullptr;
    errno = 0;
    output = std::strtod(value.c_str(), &end);
    return errno == 0 && end != value.c_str() && *end == '\0' &&
           std::isfinite(output);
}

bool parseInt(const std::string& value, int& output) {
    if (value.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (errno != 0 || end == value.c_str() || *end != '\0' ||
        parsed < static_cast<long>(std::numeric_limits<int>::min()) ||
        parsed > static_cast<long>(std::numeric_limits<int>::max())) {
        return false;
    }
    output = static_cast<int>(parsed);
    return true;
}

ShortHorizonMatchedPreflightResult failure(const std::string& detail) {
    ShortHorizonMatchedPreflightResult result;
    result.detail = detail;
    return result;
}

bool readVariant(const std::string& name,
                 const FieldMap& fields,
                 VariantConfig& variant,
                 std::string& detail) {
    for (const std::string& required : requiredFields()) {
        if (fields.count(required) == 0) {
            detail = "MISSING_FIELD_" + name + "_" + required;
            return false;
        }
    }
    variant.name = name;
    variant.primitive_mode = fields.at("primitive_mode");
    if (!parseBool(fields.at("slosh_enable"), variant.slosh_enable) ||
        !parseBool(fields.at("smooth_priority_enable"),
                   variant.smooth_priority_enable) ||
        !parseBool(fields.at("slosh_constraint_enable"),
                   variant.slosh_constraint_enable)) {
        detail = "INVALID_BOOLEAN_" + name;
        return false;
    }
    struct DoubleField {
        const char* name;
        double* destination;
    };
    const DoubleField double_fields[] = {
        {"w_contour", &variant.w_contour},
        {"w_lag", &variant.w_lag},
        {"w_progress", &variant.w_progress},
        {"w_v", &variant.w_v},
        {"w_vs", &variant.w_vs},
        {"v_ref", &variant.v_ref},
        {"w_control", &variant.w_control},
        {"w_accel", &variant.w_accel},
        {"w_smooth", &variant.w_smooth},
        {"w_alpha", &variant.w_alpha},
        {"w_du_a", &variant.w_du_a},
        {"w_du_vs", &variant.w_du_vs},
        {"w_slosh", &variant.w_slosh},
        {"slosh_cost_tail_discount", &variant.slosh_cost_tail_discount},
    };
    for (const DoubleField& field : double_fields) {
        if (!parseDouble(fields.at(field.name), *field.destination)) {
            detail = "INVALID_NUMBER_" + name + "_" + field.name;
            return false;
        }
    }
    if (!parseInt(fields.at("slosh_cost_horizon_steps"),
                  variant.slosh_cost_horizon_steps)) {
        detail = "INVALID_INTEGER_" + name +
            "_slosh_cost_horizon_steps";
        return false;
    }
    return true;
}

bool same(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= 1e-12;
}

bool frozenCommonContract(const VariantConfig& variant) {
    return variant.slosh_enable && variant.smooth_priority_enable &&
           !variant.slosh_constraint_enable &&
           variant.primitive_mode == "linear" &&
           same(variant.w_contour, 1.0) && same(variant.w_lag, 0.2) &&
           same(variant.w_progress, 0.2) && same(variant.w_v, 1.0) &&
           same(variant.w_vs, 0.3) && same(variant.v_ref, 0.2) &&
           same(variant.w_control, 0.3) && same(variant.w_accel, 0.0) &&
           same(variant.w_smooth, 1.0) && same(variant.w_alpha, 1.0) &&
           same(variant.w_du_a, 1.0) && same(variant.w_du_vs, 1.0) &&
           variant.slosh_cost_horizon_steps == 3 &&
           same(variant.slosh_cost_tail_discount, 0.0);
}

}  // namespace

ShortHorizonMatchedPreflightResult validateShortHorizonMatchedParamDump(
    const std::string& parameter_dump) {
    const std::string root = "/spmpc_local_planner/variants/";
    const std::string prefix0 = root + kMatched0 + "/";
    const std::string prefix5 = root + kMatched5 + "/";
    FieldMap fields0;
    FieldMap fields5;
    std::istringstream stream(parameter_dump);
    std::string line;
    while (std::getline(stream, line)) {
        const std::string normalized = trim(line);
        FieldMap* destination = nullptr;
        std::string remainder;
        if (startsWith(normalized, prefix0)) {
            destination = &fields0;
            remainder = normalized.substr(prefix0.size());
        } else if (startsWith(normalized, prefix5)) {
            destination = &fields5;
            remainder = normalized.substr(prefix5.size());
        } else {
            continue;
        }
        const std::size_t separator = remainder.find(':');
        if (separator == std::string::npos) {
            return failure("MALFORMED_PARAMETER_LINE");
        }
        const std::string field = trim(remainder.substr(0, separator));
        const std::string value = trim(remainder.substr(separator + 1));
        if (requiredFields().count(field) == 0) {
            return failure("UNKNOWN_MATCHED_FIELD_" + field);
        }
        if (!destination->emplace(field, value).second) {
            return failure("DUPLICATE_MATCHED_FIELD_" + field);
        }
    }

    VariantConfig matched0;
    VariantConfig matched5;
    std::string detail;
    if (!readVariant(kMatched0, fields0, matched0, detail) ||
        !readVariant(kMatched5, fields5, matched5, detail)) {
        return failure(detail);
    }
    if (!matchedVariantCommonConfigEqual(matched0, matched5)) {
        return failure("MATCHED_COMMON_CONFIG_MISMATCH");
    }
    if (!same(matched0.w_slosh, 0.0) || !same(matched5.w_slosh, 5.0)) {
        return failure("MATCHED_SLOSH_WEIGHT_CONTRACT_MISMATCH");
    }
    if (!frozenCommonContract(matched0) || !frozenCommonContract(matched5)) {
        return failure("SHORT_HORIZON_RELEASE_CONTRACT_MISMATCH");
    }
    ShortHorizonMatchedPreflightResult result;
    result.success = true;
    result.detail = "OK";
    return result;
}

}  // namespace tools
}  // namespace spmpc_local_planner
