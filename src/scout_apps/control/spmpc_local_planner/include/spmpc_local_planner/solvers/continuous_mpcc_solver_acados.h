#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/warm_start/warm_start_generator.h"
#include "spmpc_local_planner/warm_start/warm_start_output.h"
#include <memory>

namespace spmpc_local_planner {

// Compile-time capability contract for the separately generated Phase-Rejoin
// OCP.  A positive horizon is returned only when that generated solver is
// linked into this package; callers must not infer this from the main solver's
// horizon.
bool continuousMpccPhaseRejoinAvailable();
int continuousMpccPhaseRejoinHorizonSteps();

// 连续 MPCC（acados 后端，alpha-state 主线：B0 6D / slosh 10D MPCC）。
//
// 编译期分两种形态：
//   - 定义了 SPMPC_WITH_ACADOS（CMake 探测到 acados 与生成的 spmpc_b0 求解器）：
//     真实现，包装生成的 acados 求解器；
//   - 否则：占位实现，solve() 返回 ACADOS_NOT_IMPLEMENTED，整包仍可编译（§10 红线 #8）。
//
// acados 类型不出现在本头文件（capsule 由不完整 Impl 以 RAII 持有），
// 使 solver_factory / core 在无 acados 环境下也能包含本头文件（§11.1）。
class ContinuousMpccSolverAcados : public SpmpcSolver {
public:
    ContinuousMpccSolverAcados();
    ~ContinuousMpccSolverAcados() override;

    SolverConfigureResult configure(
        const SolverParams& params,
        const VariantConfig& variant) override;
    bool solve(const SolverInput& input, const ReferencePath& reference, SolverOutput& output) const override;

private:
    struct Impl;

    SolverParams params_;
    VariantConfig variant_;
    SloshDynamics slosh_dyn_;             // 与 primitive 共用的液体物理核（注入 slosh 模型参数，§4.3）
    bool use_slosh_model_ = false;        // 由 variant.slosh_enable 决定接 b0(6维) 还是 slosh(10维)

    // Generated acados capsules live behind an incomplete RAII implementation.
    // No C ABI type, ownership flag, or raw pointer crosses this public header.
    std::unique_ptr<Impl> impl_;
    std::unique_ptr<WarmStartGenerator> warm_start_generator_;
    mutable WarmStartOutput previous_warm_start_solution_;
    mutable bool have_previous_solution_ = false;
    mutable double u_prev_[3] = {0.0, 0.0, 0.0};  // 上周期 OCP 控制 [a, alpha, v_s]
    mutable bool have_u_prev_ = false;
};

}  // namespace spmpc_local_planner
