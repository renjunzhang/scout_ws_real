#pragma once

#include "spmpc_sim_local_planner/core/spmpc_solver.h"
#include <memory>
#include <string>

namespace spmpc_sim_local_planner {

// 已知的 solver 后端名称。
constexpr const char* kSolverBackendPrimitive = "primitive";
constexpr const char* kSolverBackendContinuousMpccAcados = "continuous_mpcc_acados";
constexpr const char* kSolverBackendContinuousMpccDirectOmegaLegacy = "continuous_mpcc_direct_omega_legacy";

// 判断后端名是否被工厂识别（供 ROS 层校验并告警，core/solvers 自身不依赖 ROS）。
bool isKnownSolverBackend(const std::string& backend);

// 按后端名创建 solver；调用方应先校验后端名，未识别名称会抛出异常而不是回退到 primitive。
std::unique_ptr<SpmpcSolver> makeSolver(const std::string& backend);

}  // namespace spmpc_sim_local_planner
