cmake_minimum_required(VERSION 3.10)
project(ai_navigation_opencv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  image_transport
  message_generation
  tf
  tf_conversions
  actionlib
  actionlib_msgs
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Python setup
catkin_python_setup()

# Custom message definitions
add_message_files(
  FILES
  ObjectDetection.msg
  NavigationCommand.msg
  RobotState.msg
  ObstacleInfo.msg
)

# Custom service definitions
add_service_files(
  FILES
  PathPlanRequest.srv
  LocalizationService.srv
)

# Custom action definitions
add_action_files(
  FILES
  NavigateTo.action
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
)

# Catkin package definition
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    roscpp 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    nav_msgs 
    cv_bridge 
    image_transport 
    message_runtime
    tf
    tf_conversions
    actionlib
    actionlib_msgs
  DEPENDS OpenCV Boost
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  src/vision
  src/navigation
  src/ros_nodes
)

# Installation
install(
  DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()

message(STATUS "AI Navigation OpenCV ROS - Build Configuration")
message(STATUS "ROS Version: ${ROS_DISTRO}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")