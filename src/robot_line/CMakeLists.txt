cmake_minimum_required(VERSION 2.8.3)
project(robot_line)

add_compile_options(-g -std=c++11)  

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

find_package( OpenCV REQUIRED )
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  points.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 头文件路径
include_directories( 
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  include
  EDlib
  camera_models
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs 
)

add_library(lineLib  STATIC
include/line_detect.h
include/ros_interface.h
include/pole.cpp
include/pole.h
src/line_detect.cpp
src/ros_interface.cpp
)

add_library(camera_model STATIC
  camera_models/Camera.h
  camera_models/CataCamera.cc
  camera_models/CataCamera.h
  camera_models/PinholeCamera.cc
  camera_models/PinholeCamera.h
  camera_models/gpl.h
  camera_models/Camera.cc
)

add_library(edlib STATIC
EDlib/ED.cpp
EDlib/ED.h
EDlib/EDCircles.cpp
EDlib/EDCircles.h
EDlib/EDColor.cpp
EDlib/EDColor.h
EDlib/EDLib.h
EDlib/EDLines.cpp
EDlib/EDLines.h
EDlib/EDPF.cpp
EDlib/EDPF.h
EDlib/NFA.cpp
EDlib/NFA.h
EDlib/depth.hpp  
)

list(APPEND thirdparty_libraries
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(edlib  ${OpenCV_LIBS} yaml-cpp)
target_link_libraries(lineLib  ${catkin_LIBRARIES} edlib)

# 执行文件
add_executable(robot_line src/line_node.cpp)
target_link_libraries(robot_line ${catkin_LIBRARIES} ${OpenCV_LIBS} lineLib yaml-cpp camera_model)
