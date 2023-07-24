/** 
 * @brief 简单速度控制
 * @author WeiXuan <2020302121154@whu.edu.cn
 * @file vel_control.cpp
 * @addtogroup vel_control
 * @signature: 热爱漫无边际，生活自有分寸
 */
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

// 当前速度
geometry_msgs::Twist vs;

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "simple_car_vel");
    // Ros句柄
    ros::NodeHandle n;
    // 发布者：速度
    ros::Publisher vel_sp_pub = n.advertise<geometry_msgs::Twist>("/robot_serial/cmd_vel", 10);
    // ROS通信速率：20HZ
    ros::Rate rate(20.0);
    // 获取当前时间节点
    ros::Time last_request = ros::Time::now();
    // 初始速度
    vs.linear.x = 0.0;
    vs.linear.y = 0.0;
    vs.linear.z = 0.0;
    vs.angular.x = 0.0;
    vs.angular.y = 0.0;
    vs.angular.z = 0.0;
    int count = 0;
    // 获取此时时间节点
    last_request = ros::Time::now();
    while (ros::ok())
    {
        // 0-1s内
        if (ros::Time::now() - last_request < ros::Duration(2.0))
        {
            ROS_INFO("Stop");
            vs.linear.x = 0;
            vs.linear.y = 0;
            vs.linear.z = 0;
            vs.angular.x = 0;
            vs.angular.y = 0;
            vs.angular.z = 0;
        }
        if (ros::Time::now() - last_request > ros::Duration(2.0) && ros::Time::now() - last_request < ros::Duration(4.0))
        {
            ROS_INFO("Circle");
            vs.linear.x = 0;
            vs.linear.y = 0;
            vs.linear.z = 0;
            vs.angular.x = 0;
            vs.angular.y = 0;
            vs.angular.z = 1.57;
            count++;
        }
        if (ros::Time::now() - last_request > ros::Duration(4.0))
        {
            ROS_INFO("Stop");
            std::cout << count << std::endl;
            vs.linear.x = 0;
            vs.linear.y = 0;
            vs.linear.z = 0;
            vs.angular.x = 0;
            vs.angular.y = 0;
            vs.angular.z = 0;
        }
        // // 5-10s内
        // if (ros::Time::now() - last_request >= ros::Duration(5.0) && ros::Time::now() - last_request < ros::Duration(10.0))
        // {
        //     ROS_INFO("Left");
        //     vs.linear.x = 0;
        //     vs.linear.y = 0.5;
        //     vs.linear.z = 0;
        //     vs.angular.x = 0;
        //     vs.angular.y = 0;
        //     vs.angular.z = 0;
        // }
        // // 后退
        // if (ros::Time::now() - last_request >= ros::Duration(10.0) && ros::Time::now() - last_request < ros::Duration(15.0))
        // {
        //     ROS_INFO("Back");
        //     vs.linear.x = -0.5;
        //     vs.linear.y = 0;
        //     vs.linear.z = 0;
        //     vs.angular.x = 0;
        //     vs.angular.y = 0;
        //     vs.angular.z = 0;
        // }
        // // 15-20s
        // if (ros::Time::now() - last_request >= ros::Duration(15.0) && ros::Time::now() - last_request < ros::Duration(20.0))
        // {
        //     ROS_INFO("Right");
        //     vs.linear.x = 0;
        //     vs.linear.y = -0.5;
        //     vs.linear.z = 0;
        //     vs.angular.x = 0;
        //     vs.angular.y = 0;
        //     vs.angular.z = 0;
        // }
        // 发布速度信息
        vel_sp_pub.publish(vs);
        // 等待订阅信息
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}