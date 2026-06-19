cmake_minimum_required(VERSION 3.0.2)
project(scout_profile_baselines)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/generate_hamaguchi_profile.py
  scripts/generate_lim_style_profile.py
  scripts/advanced_profile_common.py
  scripts/path_profile_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
