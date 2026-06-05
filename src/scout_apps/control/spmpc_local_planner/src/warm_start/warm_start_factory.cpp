#include "spmpc_local_planner/warm_start/warm_start_factory.h"
#include "spmpc_local_planner/warm_start/diff_drive_flatness_warm_start.h"
#include "spmpc_local_planner/warm_start/mecanum_warm_start.h"

namespace spmpc_local_planner {
namespace {

bool isDiffDriveName(const std::string& name) {
    return name == "diff_drive_flatness" || name == "diff_drive" ||
           name == "differential" || name == "unicycle";
}

}  // namespace

std::unique_ptr<WarmStartGenerator> makeWarmStartGenerator(
    const WarmStartConfig& config,
    const PlatformParams& platform) {
    const std::string selector = config.type.empty() ? platform.kinematics : config.type;
    if (isDiffDriveName(selector)) {
        return std::unique_ptr<WarmStartGenerator>(new DiffDriveFlatnessWarmStart());
    }
    if (selector == "mecanum") {
        return std::unique_ptr<WarmStartGenerator>(new MecanumWarmStart());
    }
    return std::unique_ptr<WarmStartGenerator>();
}

}  // namespace spmpc_local_planner
