cmake_minimum_required(VERSION 3.0.2)
project(realsense_liquid_measurement)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/calibrate_liquid_roi.py
  scripts/annotate_liquid_roi.py
  scripts/extract_liquid_height_from_bag.py
  scripts/debug_liquid_vs_mpc_frame_by_frame.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
