cmake_minimum_required(VERSION 3.0.2)
project(scout_profile_baselines)

find_package(catkin REQUIRED)

catkin_package()

# Stable user-facing rosrun entrypoints. These wrappers delegate to the
# method-specific implementation folders below, so suite commands stay stable.
catkin_install_python(PROGRAMS
  scripts/generate_hamaguchi_profile.py
  scripts/generate_lim_style_profile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Method implementations: not separate benchmark entrypoints, but installed so
# the wrappers work in devel/install spaces.
catkin_install_python(PROGRAMS
  scripts/hamaguchi/generate_profile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/hamaguchi
)

catkin_install_python(PROGRAMS
  scripts/lim/generate_profile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/lim
)

# Shared import sidecars for the offline profile generators.
catkin_install_python(PROGRAMS
  scripts/common/advanced_profile_common.py
  scripts/common/path_profile_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/common
)

install(FILES
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
