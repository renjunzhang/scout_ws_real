#pragma once

#include "spmpc_sim_local_planner/warm_start/warm_start_generator.h"

namespace spmpc_sim_local_planner {

class DiffDriveFlatnessWarmStart : public WarmStartGenerator {
public:
    bool generate(
        const WarmStartInput& input,
        WarmStartOutput& output,
        WarmStartDiagnostics& diagnostics) override;
};

}  // namespace spmpc_sim_local_planner
