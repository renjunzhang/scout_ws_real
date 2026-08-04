cmake_minimum_required(VERSION 3.0.2)
project(scout_liquid_plant)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nav_msgs rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/liquid_plant_formal_evidence_intake.py
  scripts/liquid_plant_fidelity_verify.py
  scripts/liquid_plant_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch schema
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
