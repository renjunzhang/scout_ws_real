#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/warm_start/warm_start_generator.h"
#include "spmpc_local_planner/warm_start/warm_start_output.h"
#include <memory>

namespace spmpc_local_planner {

// 连续 MPCC（acados 后端，B0：5 维 MPCC）。
//
// 编译期分两种形态：
//   - 定义了 SPMPC_WITH_ACADOS（CMake 探测到 acados 与生成的 spmpc_b0 求解器）：
//     真实现，包装生成的 acados 求解器；
//   - 否则：占位实现，solve() 返回 ACADOS_NOT_IMPLEMENTED，整包仍可编译（§10 红线 #8）。
//
// acados 类型不出现在本头文件（capsule 以 void* 不透明持有），
// 使 solver_factory / core 在无 acados 环境下也能包含本头文件（§11.1）。
class ContinuousMpccSolverAcados : public SpmpcSolver {
public:
    ContinuousMpccSolverAcados();
    ~ContinuousMpccSolverAcados() override;

    void configure(const SolverParams& params, const VariantConfig& variant) override;
    bool solve(const SolverInput& input, const ReferencePath& reference, SolverOutput& output) const override;

private:
    SolverParams params_;
    VariantConfig variant_;
    SloshDynamics slosh_dyn_;             // 与 primitive 共用的液体物理核（注入 slosh 模型参数，§4.3）
    bool use_slosh_model_ = false;        // 由 variant.slosh_enable 决定接 b0(5维) 还是 slosh(9维)

    void* capsule_ = nullptr;             // 不透明 acados capsule（仅 SPMPC_WITH_ACADOS 下有效）
    std::unique_ptr<WarmStartGenerator> warm_start_generator_;
    mutable WarmStartOutput previous_warm_start_solution_;
    mutable bool have_previous_solution_ = false;
    mutable double u_prev_[3] = {0.0, 0.0, 0.0};  // 上周期下发控制 [a, omega, v_s]
    mutable bool have_u_prev_ = false;
};

}  // namespace spmpc_local_planner
