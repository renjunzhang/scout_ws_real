#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/solver/api/backend.h"
#include <memory>
#include <string>

namespace spmpc_local_planner {

// 按后端名创建 solver；调用方应先校验后端名，未识别名称会抛出异常而不是回退到 primitive。
std::unique_ptr<SpmpcSolver> makeSolver(const std::string& backend);

}  // namespace spmpc_local_planner
