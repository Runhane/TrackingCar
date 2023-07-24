cmake_minimum_required(VERSION 2.8.3)
# robot_serial
project(robot_serial)

# 依赖包
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
)

add_compile_options(-g -std=c++11)  

# BOOST 库
find_package(Boost REQUIRED COMPONENTS system)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 头文件路径
include_directories( 
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
)
add_library(serialLib  STATIC
include/middleware.h
include/ros_interface.h
src/middleware.cpp
src/ros_interface.cpp
)
target_link_libraries(serialLib  ${catkin_LIBRARIES})
# 执行文件
add_executable(robot_serial src/serial_node.cpp)
target_link_libraries(robot_serial ${catkin_LIBRARIES} serialLib)
