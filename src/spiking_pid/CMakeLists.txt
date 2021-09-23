cmake_minimum_required(VERSION 3.0.2)
project(spiking_pid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES spiking_pid
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/gazebo_position_controller.py
  scripts/waypoint_node.py
  scripts/run_simulation_tests.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
