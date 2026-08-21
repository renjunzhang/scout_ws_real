#pragma once

#include "spmpc_local_planner/phase_rejoin/types.h"

#include <string>
#include <vector>

namespace spmpc_local_planner {

struct DelayAugmentedPhaseParameterMatrix {
    bool valid = false;
    std::string status = "NOT_BUILT";
    int stage_count = 0;
    int parameter_width = 0;
    std::vector<std::string> parameter_names;
    std::vector<double> values;

    bool hasCanonicalShape() const;
    const double* stageData(int stage) const;
    double value(int stage, int parameter_index) const;
};

class DelayAugmentedPhaseParameterBuilder {
public:
    static DelayAugmentedPhaseParameterMatrix build(
        const DelayAugmentedPhaseSolverContext& context);
};

}  // namespace spmpc_local_planner
