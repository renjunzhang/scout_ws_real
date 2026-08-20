#include "spmpc_local_planner/ros/ros_config_loader.h"

namespace spmpc_local_planner {

AppConfig RosConfigLoader::load(const ros::NodeHandle& private_node,
                                ValidationReport& report) {
    AppConfig config;
    private_node.param(
        "map_vref/runtime_v_ref_enable",
        config.map_vref.runtime_override_enable,
        config.map_vref.runtime_override_enable);
    private_node.param(
        "map_vref/runtime_v_ref",
        config.map_vref.runtime_override_mps,
        config.map_vref.runtime_override_mps);
    private_node.param(
        "map_vref/profile_enable",
        config.map_vref.profile_enable,
        config.map_vref.profile_enable);
    private_node.param(
        "map_vref/profile_path",
        config.map_vref.profile_path,
        config.map_vref.profile_path);
    private_node.param(
        "map_vref/profile_lookahead_s",
        config.map_vref.profile_lookahead_m,
        config.map_vref.profile_lookahead_m);
    report = validateAndNormalize(config);
    return config;
}

}  // namespace spmpc_local_planner
