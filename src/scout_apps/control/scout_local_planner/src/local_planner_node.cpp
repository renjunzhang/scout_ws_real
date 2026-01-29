/**
 * @file local_planner_node.cpp
 * @brief MPC 局部规划器主节点
 */

#include "scout_local_planner/local_planner_ros.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "scout_local_planner");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    scout_local_planner::LocalPlannerROS planner;
    
    if (!planner.initialize(nh, pnh)) {
        ROS_ERROR("Failed to initialize local planner");
        return 1;
    }
    
    ROS_INFO("Scout Local Planner (MPC) started");
    planner.run();
    
    return 0;
}