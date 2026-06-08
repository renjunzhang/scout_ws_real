#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include <memory>

namespace spmpc_local_planner {

// 路 B 生产后端：direct-omega 连续 MPCC（omega 作为直接控制 u[1]，单次 SQP-RTI 求解稳定）。
//   B0:    x = [px, py, theta, v, s]                                  (5D)
//   slosh: x = [px, py, theta, v, s, eta_x, eta_x_dot, eta_y, eta_y_dot] (9D), a_y = v*omega
// omega 不在 OCP 内硬约束角加速度；转向 chatter 由 wrapper 出口 omega-rate 限幅
// (cmd_omega = clamp(prev ± alpha_max*dt))压制，实测 ω̇ 有界，与 TEB/DWA acc_lim_theta 同口径。
class ContinuousMpccDirectOmegaLegacySolverAcados : public SpmpcSolver {
public:
    ContinuousMpccDirectOmegaLegacySolverAcados();
    ~ContinuousMpccDirectOmegaLegacySolverAcados() override;

    void configure(const SolverParams& params, const VariantConfig& variant) override;
    bool solve(const SolverInput& input, const ReferencePath& reference, SolverOutput& output) const override;

private:
    SolverParams params_;
    VariantConfig variant_;
    SloshDynamics slosh_dyn_;             // 与 primitive / 主线共用的液体物理核
    bool use_slosh_model_ = false;        // variant.slosh_enable -> 接 b0(5D) 还是 slosh(9D)

    void* capsule_ = nullptr;
    mutable double u_prev_[3] = {0.0, 0.0, 0.0};  // 上周期下发控制 [a, omega, v_s]
    mutable bool have_u_prev_ = false;
};

}  // namespace spmpc_local_planner
