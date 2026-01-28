#include <ros/ros.h>
#include "scout_local_planner/local_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "scout_local_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  scout_local_planner::LocalPlanner planner(nh, pnh);

  if (!planner.initialize()) {
    ROS_ERROR("Failed to initialize local planner");
    return 1;
  }

  double control_rate;
  pnh.param<double>("control_rate", control_rate, 20.0);
  ros::Rate rate(control_rate);

  ROS_INFO("Scout Local Planner (MPC with Slosh Suppression) started");

  while (ros::ok()) {
    ros::spinOnce();

    geometry_msgs::Twist cmd_vel;
    if (planner.computeVelocityCommands(cmd_vel)) {
      // 速度指令由computeVelocityCommands内部发布
    }

    rate.sleep();
  }

  return 0;
}
