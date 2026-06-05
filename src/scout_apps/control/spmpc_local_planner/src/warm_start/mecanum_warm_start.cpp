#include "spmpc_local_planner/warm_start/mecanum_warm_start.h"

namespace spmpc_local_planner {

bool MecanumWarmStart::generate(
    const WarmStartInput& input,
    WarmStartOutput& output,
    WarmStartDiagnostics& diagnostics) {
    (void)input;
    output = WarmStartOutput{};
    diagnostics = WarmStartDiagnostics{};
    diagnostics.failure_reason = "MECANUM_WARM_START_NOT_IMPLEMENTED";
    output.fallback_reason = diagnostics.failure_reason;
    output.diagnostics = diagnostics;
    return false;
}

}  // namespace spmpc_local_planner
