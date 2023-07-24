#include "ros_interface.h"
#include "line_detect.h"

int main(int argc, char** argv)
{
  // ROS初始化 并设置节点名称 
  ros::init(argc, argv, "robot_line"); 
  ros::NodeHandle nh;
  // 实例化一个对象
  ros_interface Robot_Control(nh); 
  ROS_INFO_STREAM("Start to launch Robot_Line!");
  // 循环执行数据采集和发布话题等操作
  ros::spin();
  return 0;  
} 