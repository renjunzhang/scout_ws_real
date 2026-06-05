#pragma once

#include "spmpc_local_planner/warm_start/warm_start_input.h"
#include "spmpc_local_planner/warm_start/warm_start_output.h"

namespace spmpc_local_planner {

class WarmStartGenerator {
public:
    virtual ~WarmStartGenerator() = default;

    virtual bool generate(
        const WarmStartInput& input,
        WarmStartOutput& output,
        WarmStartDiagnostics& diagnostics) = 0;
};

}  // namespace spmpc_local_planner
