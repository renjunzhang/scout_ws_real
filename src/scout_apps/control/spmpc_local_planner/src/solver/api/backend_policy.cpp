#include "spmpc_local_planner/solver/api/backend_policy.h"

#include "spmpc_local_planner/solver/api/backend.h"

namespace spmpc_local_planner {
namespace {

void appendPolicyError(std::string& reason, const std::string& message) {
    if (!reason.empty()) {
        reason += "; ";
    }
    reason += message;
}

}  // namespace

bool validateBackendPolicy(const SolverParams& params,
                           const VariantConfig& variant,
                           std::string& reason) {
    reason.clear();

    if (params.solver_backend == kSolverBackendContinuousMpccAcados) {
        if (variant.slosh_constraint_enable && !variant.slosh_enable) {
            appendPolicyError(
                reason,
                "slosh_constraint_enable requires slosh_enable on continuous_mpcc_acados");
        }
        if (params.corridor_enable) {
            appendPolicyError(
                reason,
                "continuous_mpcc_acados does not support corridor_enable until J_corridor is implemented in the OCP");
        }
        if (params.obstacle_enable) {
            appendPolicyError(
                reason,
                "continuous_mpcc_acados does not support obstacle_enable until obstacle OCP terms are implemented");
        }
        if (params.homotopy_enable) {
            appendPolicyError(
                reason,
                "continuous_mpcc_acados does not support homotopy_enable until multi-candidate/homotopy SPMPC is implemented");
        }
        if (params.corridor_hard_bound_enable) {
            appendPolicyError(
                reason,
                "continuous_mpcc_acados does not support corridor_hard_bound_enable yet");
        }
    } else if (params.solver_backend ==
               kSolverBackendContinuousMpccDirectOmegaLegacy) {
        if (variant.slosh_enable || variant.slosh_constraint_enable) {
            appendPolicyError(
                reason,
                "slosh-enabled variants must use continuous_mpcc_acados mainline, not RouteB legacy backend");
        }
        if (params.corridor_enable) {
            appendPolicyError(
                reason,
                "RouteB legacy backend does not support corridor_enable until J_corridor is implemented in the OCP");
        }
        if (params.obstacle_enable) {
            appendPolicyError(
                reason,
                "RouteB legacy backend does not support obstacle_enable under the SPMPC mainline policy");
        }
        if (params.homotopy_enable) {
            appendPolicyError(
                reason,
                "RouteB legacy backend does not support homotopy_enable under the SPMPC mainline policy");
        }
        if (params.corridor_hard_bound_enable) {
            appendPolicyError(
                reason,
                "RouteB legacy backend does not support corridor_hard_bound_enable");
        }
    } else if (params.solver_backend == kSolverBackendPrimitive) {
        if (variant.slosh_enable || variant.slosh_constraint_enable) {
            appendPolicyError(
                reason,
                "primitive is fallback/debug rollout sampling only and cannot run slosh-enabled SPMPC variants");
        }
        if (params.obstacle_enable) {
            appendPolicyError(
                reason,
                "primitive cannot run obstacle_enable=true under the SPMPC mainline policy");
        }
        if (params.homotopy_enable) {
            appendPolicyError(
                reason,
                "primitive cannot run homotopy_enable=true under the SPMPC mainline policy");
        }
        if (params.corridor_hard_bound_enable) {
            appendPolicyError(
                reason,
                "primitive cannot run corridor_hard_bound_enable=true under the SPMPC mainline policy");
        }
    } else {
        appendPolicyError(reason, "unknown solver backend");
    }

    return reason.empty();
}

}  // namespace spmpc_local_planner
