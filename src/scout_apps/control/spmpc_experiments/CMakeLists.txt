cmake_minimum_required(VERSION 3.0.2)
project(spmpc_experiments)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
