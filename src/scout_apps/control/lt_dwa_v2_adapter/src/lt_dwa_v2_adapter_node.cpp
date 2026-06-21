#include <ros/ros.h>

#include "lt_dwa_v2_adapter/ros/lt_dwa_v2_adapter_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lt_dwa_v2_adapter_node");
  lt_dwa_v2_adapter::LtDwaV2AdapterROS adapter;
  adapter.spin();
  return 0;
}
