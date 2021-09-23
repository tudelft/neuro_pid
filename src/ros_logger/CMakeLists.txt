cmake_minimum_required(VERSION 2.8.3)
project(ros_logger)

add_definitions(-std=c++11)

#--------------------------#
# Generation of SDF models #
#--------------------------#

# We need Gazebo version >= 3.0.0 to generate iris.sdf file
# (gz sdf ... command needs to be available)
find_package(gazebo REQUIRED)
find_package(PythonInterp REQUIRED)

if(NO_ROS)
  message(STATUS "NO_ROS = true, not building logger node, returning now.")
  return()
endif()

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs mav_msgs roscpp sensor_msgs rosbag message_filters)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
    rosbag
    message_filters
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ros_logger_nodelet
  src/logger_nodelet.cpp
  src/logger.cpp
)

add_dependencies(ros_logger_nodelet ${catkin_EXPORTED_TARGETS})

add_executable(ros_logger src/logger.cpp src/logger_node.cpp)

target_link_libraries(ros_logger ${catkin_LIBRARIES})
target_link_libraries(ros_logger_nodelet ${catkin_LIBRARIES})

add_dependencies(ros_logger ${catkin_EXPORTED_TARGETS})
