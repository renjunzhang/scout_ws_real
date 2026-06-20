#include "lt_dwa_adapter/lt_dwa_adapter_ros.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lt_dwa_adapter_node");
  lt_dwa_adapter::LtDwaAdapterROS node;
  node.spin();
  return 0;
}
