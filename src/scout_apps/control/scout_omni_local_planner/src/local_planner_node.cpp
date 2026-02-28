/**
 * @file local_planner_node.cpp
 * @brief 全向轮 MPC 局部规划器主节点
 */

#include "scout_omni_local_planner/local_planner_ros.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "scout_omni_local_planner");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    scout_omni_local_planner::LocalPlannerROS planner;
    
    if (!planner.initialize(nh, pnh)) {
        ROS_ERROR("Failed to initialize omni local planner");
        return 1;
    }
    
    ROS_INFO("Scout Omni Local Planner (MPC, 3-DOF) started");
    planner.run();
    
    return 0;
}
