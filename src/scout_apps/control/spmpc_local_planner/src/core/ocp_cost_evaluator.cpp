#include "spmpc_local_planner/core/ocp_cost_evaluator.h"
#include "generated/ocp_cost_generated.h"
#include "generated/ocp_cost_contract.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

namespace spmpc_local_planner {

bool evaluateOcpCost(const PredictedHorizonDebug& horizon,
                     const std::vector<double>& parameters,
                     int parameter_width, CostBreakdown& cost) {
    const size_t n = horizon.controls.size();
    const bool liquid = horizon.slosh_enabled;
    const int expected_np = liquid ? spmpc_slosh_stage_terms_sparsity_in(2)[0]
                                   : spmpc_b0_stage_terms_sparsity_in(2)[0];
    if (n != SPMPC_COST_N || horizon.states.size() != n+1 || parameter_width != expected_np ||
        parameters.size() != (n+1)*static_cast<size_t>(parameter_width)) return false;
    CostBreakdown result;
    for (size_t k = 0; k <= n; ++k) {
        const auto& x = horizon.states[k].model_state;
        if (x.size() != static_cast<size_t>(liquid ? kExplicitActuatorSloshStateSize : kExplicitActuatorB0StateSize)) return false;
        double u[3] = {};
        if (k < n) {
            u[0] = horizon.controls[k].a;
            u[1] = horizon.controls[k].alpha_or_omega;
            u[2] = horizon.controls[k].v_s;
        }
        const double* args[] = {x.data(), u, parameters.data() + k*parameter_width};
        double terms[SPMPC_COST_COMPONENTS] = {};
        double* outputs[] = {terms};
        const auto function = liquid ? (k == n ? spmpc_slosh_terminal_terms : spmpc_slosh_stage_terms)
                                     : (k == n ? spmpc_b0_terminal_terms : spmpc_b0_stage_terms);
        // SX generated functions use no work arrays; contract-check this at build time.
        static_assert(spmpc_b0_stage_terms_SZ_W == 0 && spmpc_slosh_stage_terms_SZ_W == 0 &&
                      spmpc_b0_terminal_terms_SZ_W == 0 && spmpc_slosh_terminal_terms_SZ_W == 0,
                      "Update generated cost evaluator workspace");
        if (function(args, outputs, nullptr, nullptr, 0) != 0) return false;
        for (double term : terms) if (!std::isfinite(term)) return false;
        result.J_slack += terms[10];
        result.J_stop += terms[11];
        result.J_curvature += terms[12];
        result.J_curvature_change += terms[13];
        if (k == n) {
            // Components 10..13 are accounted above at every node. Their
            // position is explicit so adding a component cannot double count.
            result.J_terminal = std::accumulate(std::begin(terms), std::begin(terms)+10, 0.0);
        } else {
            result.J_contour += terms[0]; result.J_lag += terms[1];
            result.J_progress += terms[2]; result.J_v_actual += terms[3];
            result.J_v_s += terms[4]; result.J_anti_creep += terms[5];
            result.J_control += terms[6]; result.J_smooth += terms[7];
            result.J_slosh_eta += terms[8]; result.J_slosh_eta_dot += terms[9];
        }
    }
    result.J_v = result.J_v_actual + result.J_v_s;
    cost = result;
    return true;
}

}  // namespace spmpc_local_planner
