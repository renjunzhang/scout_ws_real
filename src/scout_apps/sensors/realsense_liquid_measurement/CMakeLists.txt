cmake_minimum_required(VERSION 3.0.2)
project(realsense_liquid_measurement)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/calibrate_liquid_roi.py
  scripts/annotate_liquid_roi.py
  scripts/extract_liquid_height_from_bag.py
  scripts/debug_liquid_vs_mpc_frame_by_frame.py
  scripts/red_liquid_infer_from_bag.py
  scripts/export_liquid_variation_from_bags.py
  scripts/online_liquid_height_node.py
  scripts/liquid_experiment_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
