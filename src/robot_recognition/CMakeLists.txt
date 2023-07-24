cmake_minimum_required(VERSION 2.8.3)

project(robot_recognition)

# 依赖包
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  nav_msgs
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
)
find_package( OpenCV REQUIRED )


# BOOST 库
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/character_recognition.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

