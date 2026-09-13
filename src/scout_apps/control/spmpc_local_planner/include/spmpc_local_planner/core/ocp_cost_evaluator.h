#pragma once

#include "spmpc_local_planner/core/types.h"

namespace spmpc_local_planner {

// Reconstruct stage-average + terminal cost from the exact x/u/p submitted to
// and returned by the OCP. Generated expressions share the objective source.
// Native acados slack penalties are supplied separately by its wrapper.
bool evaluateOcpCost(const PredictedHorizonDebug& horizon,
                     const std::vector<double>& stage_parameters,
                     int parameter_width, CostBreakdown& cost);

}  // namespace spmpc_local_planner
