cmake_minimum_required(VERSION 3.0.2)
project(nokov_mocap_monitor)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mocap_pose_monitor.py
  scripts/mocap_imu_motion_sequence.py
  scripts/validate_mocap_imu_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/check_nokov_env.sh
  scripts/record_mocap_imu_spin.sh
  scripts/run_mocap_imu_calibration_sequence.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
