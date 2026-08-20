#pragma once

#include "spmpc_local_planner/config/app_config.h"

#include <ros/node_handle.h>

namespace spmpc_local_planner {

class RosConfigLoader {
public:
    static AppConfig load(const ros::NodeHandle& private_node,
                          ValidationReport& report);
};

}  // namespace spmpc_local_planner
