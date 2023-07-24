#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <mutex>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include "robot_line/points.h"
#include <sensor_msgs/Imu.h>
#include <vector>
#include "line_detect.h"



class ros_interface
{
public:
	ros_interface();
    ros_interface(const ros::NodeHandle& _n);

    ros::NodeHandle n;
    ros::Publisher pointsPub;
    ros::Subscriber enable;
    ros::Timer timer;

    float duration = 0.005;
    double now,past;
    float sampleTime = 0;
    int count = 0;
    int flag;
    double targetPointX, targetPointY;
    bool enableFlag = true;
    std::mutex mtx;
    imageProc comm;
	lineData rawData;

    void processData(const ros::TimerEvent&);
    void pubData();
    void enableCallBack(const std_msgs::Bool::ConstPtr& flag);  
};
#endif