#pragma once

#include <string>

namespace spmpc_local_planner {
namespace tools {

struct ShortHorizonMatchedPreflightResult {
    bool success = false;
    std::string detail = "NOT_RUN";
};

// Validates the two development matched variants from the exact parameter
// stream expanded by `roslaunch --dump-params`.  The parser is deliberately
// narrow and rejects missing, duplicate, unknown, or non-finite fields.
ShortHorizonMatchedPreflightResult validateShortHorizonMatchedParamDump(
    const std::string& parameter_dump);

}  // namespace tools
}  // namespace spmpc_local_planner
