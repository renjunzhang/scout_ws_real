#include "spmpc_local_planner/ros/spmpc_local_planner_ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "spmpc_local_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    spmpc_local_planner::SpmpcLocalPlannerROS node;
    if (!node.initialize(nh, pnh)) {
        return 1;
    }
    node.spin();
    return 0;
}
