cmake_minimum_required(VERSION 3.0.2)
project(nokov_mocap_monitor)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mocap_pose_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/check_nokov_env.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
