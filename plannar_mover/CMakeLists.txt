cmake_minimum_required(VERSION 2.8.3)
project(plannar_mover)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  controller_manager
  joint_state_publisher
  robot_state_publisher
  rospy
  rviz
  std_msgs
  people_msgs
  geometry_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For earthquake_plugin plugin
add_library(plannar_mover_plugin src/plannar_mover.cpp)
target_link_libraries(plannar_mover_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    controller_manager
    joint_state_publisher
    robot_state_publisher
    rospy
    rviz
    std_msgs
)