cmake_minimum_required(VERSION 3.0.2)
project(robot_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  geometry_msgs
  pluginlib
  tf
  nodelet
  message_generation
  diagnostic_updater
  nav_msgs
  sensor_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  actionlib 
  dynamic_reconfigure 
  geometry_msgs  
  nodelet 
  pluginlib 
  roscpp 
  rospy 
  std_msgs 
  tf 
  message_runtime
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python( 
  PROGRAMS
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(vel_control src/vel_control.cpp)
target_link_libraries(vel_control ${catkin_LIBRARIES})
add_dependencies(vel_control ${PROJECT_NAME}_gencpp)

