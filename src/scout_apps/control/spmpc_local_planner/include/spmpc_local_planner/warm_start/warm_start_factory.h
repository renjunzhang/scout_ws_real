#pragma once

#include "spmpc_local_planner/warm_start/warm_start_generator.h"
#include <memory>

namespace spmpc_local_planner {

std::unique_ptr<WarmStartGenerator> makeWarmStartGenerator(
    const WarmStartConfig& config,
    const PlatformParams& platform);

}  // namespace spmpc_local_planner
