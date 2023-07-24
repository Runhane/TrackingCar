#include "../include/ros_interface.h"


ros_interface::ros_interface()
{

}

ros_interface::ros_interface(const ros::NodeHandle & _n)
{
    n = _n;
    timer = n.createTimer(ros::Duration(duration), &ros_interface::processData,this);
    ROS_INFO_STREAM( "Start to initialize!" );
    pointsPub = n.advertise<geometry_msgs::Twist>("/robot_serial/cmd_vel",10);
    enable = n.subscribe("/robot_recognition/flag", 50, &ros_interface::enableCallBack, this);
    comm.init("/home/jetson/WorkSpace/TrackingCar/src/robot_line/config/line.yaml");
}

void ros_interface::processData(const ros::TimerEvent&)
{
    now = ros::Time::now().toSec();
    if (!comm.lineQue.empty() && enableFlag == true)
    {    
        mtx.lock();
        rawData = comm.lineQue.front();
        comm.lineQue.pop();
        sampleTime = now - past;
        flag = rawData.flag;
        targetPointX = rawData.pointX;
        targetPointY = rawData.pointY;
        mtx.unlock();
        past = now;
        pubData();
    }
    else
    {
        count = 0;
    }
    
}

void ros_interface::pubData()
{
    geometry_msgs::Twist target;
    target.linear.x = targetPointX;
    target.linear.y = targetPointY;
    target.linear.z = flag;
    pointsPub.publish(target);
    std::cout << "already publish data" << std::endl;
}

void ros_interface::enableCallBack(const std_msgs::Bool::ConstPtr& flag) 
{
    enableFlag = flag->data;
}