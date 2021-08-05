find_package(ompl REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Threads)

# ROS Stuff
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lib
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)
