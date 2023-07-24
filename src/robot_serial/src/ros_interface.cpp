#include "../include/ros_interface.h"

ros_interface::ros_interface()
{

}

ros_interface::ros_interface(const ros::NodeHandle & _n)
{
    n = _n;
    timer = n.createTimer(ros::Duration(duration), &ros_interface::processData,this);
    
    std::cout << "Start to initialize!";
    voltagePub = n.advertise<std_msgs::Float32>("/robot_serial/powervoltage", 10);
    wheelPub = n.advertise<nav_msgs::Odometry>("/robot_serial/odom", 50); 
    imuPub = n.advertise<sensor_msgs::Imu>("/robot_serial/imu", 20); 
    cmdSub = n.subscribe("robot_serial/cmd_vel", 10, &ros_interface::sendData, this);
    comm.init();

}

ros_interface::~ros_interface()
{
    if(comm.sendData(0,0,0) == true)
    {
        ROS_INFO("Send Data successfully!");
    }
    else
    {
        ROS_ERROR_STREAM("Unable To Send Data!");
    }
}

void ros_interface::processData(const ros::TimerEvent&)
{
    // if (!comm.recQue.empty())
    // {    
    //     mtx.lock();
    //     rawData = comm.recQue.front();
    //     comm.recQue.pop();
    //     now = rawData.dataTime;
    //     sampleTime = (now - past) * 1e-9;
    //     sensor_msgs::Imu imu; 
    //     nav_msgs::Odometry odom;
    //     std_msgs::Float32 voltage_msgs;
    //     ros::Time time;
    //     time.fromNSec(rawData.dataTime); 

    //     // publish wheel odometry topic
    //     odom.header.stamp = time;
    //     odom.header.frame_id = "odom_frame";

    //     poseX += static_cast<double>(rawData.velX * cos(poseZ)) * sampleTime;
    //     poseY += static_cast<double>(rawData.velX * sin(poseZ)) * sampleTime;
    //     poseZ += static_cast<double>(rawData.velZ) * sampleTime * PI / 180;
    //     if (poseZ > PI)
    //         poseZ = poseZ - 2 * PI;
    //     else if (poseZ <= -PI)
    //         poseZ = poseZ + 2 * PI;

    //     odom.pose.pose.position.x = poseX; 
    //     odom.pose.pose.position.y = poseY;
    //     odom.pose.pose.position.z = 0;

    //     odom.twist.twist.linear.x =  rawData.velX; 
    //     odom.twist.twist.linear.y =  rawData.velY; 
    //     odom.twist.twist.angular.z = rawData.velZ * PI /180;

    //     if(rawData.velX == 0 && rawData.velY == 0 && rawData.velZ == 0)
    //     {
    //         memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
    //         memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    //     }
    //     else
    //     {
    //         memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
    //         memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));    
    //     }
    //     if(rawData.flag == 1)
    //     {
    //         wheelPub.publish(odom); 
    //     }

    //     // publish imu topic
    //     imu.header.stamp = time;
    //     imu.header.frame_id = "imu_frame"; 
    //     yaw += rawData.gyroZ * sampleTime * PI / 180;
    //     if (yaw > PI)
    //         yaw -= 2 * PI;
    //     else if (yaw < -PI)
    //         yaw += 2 * PI;
    //     tf::Quaternion quat;
    //     quat = tf::createQuaternionFromYaw(yaw);
    //     imu.orientation.w = quat.w();
    //     imu.orientation.x = quat.x();
    //     imu.orientation.y = quat.y();
    //     imu.orientation.z = quat.z();
    //     // 3-axis pose covariance
    //     imu.orientation_covariance[0] = 1e6; 
    //     imu.orientation_covariance[4] = 1e6;
    //     imu.orientation_covariance[8] = 1e-6;
        
    //     imu.angular_velocity.x = rawData.gyroX * PI / 180;
    //     imu.angular_velocity.y = rawData.gyroY * PI / 180;
    //     imu.angular_velocity.z = rawData.gyroZ * PI / 180;
    //     // 3-axis angular-velocity covariance
    //     imu.angular_velocity_covariance[0] = 1e6; 
    //     imu.angular_velocity_covariance[4] = 1e6;
    //     imu.angular_velocity_covariance[8] = 1e-6;
    //     imu.linear_acceleration.x = rawData.accX; 
    //     imu.linear_acceleration.y = rawData.accY; 
    //     imu.linear_acceleration.z = rawData.accZ; 
    //     imuPub.publish(imu);

    //     // publish voltage topic
    //     voltage_msgs.data = rawData.powVol;
    //     voltagePub.publish(voltage_msgs);
    //     mtx.unlock();
    // }
    
}

void ros_interface::sendData(const geometry_msgs::Twist &targetTwist)
{
    uint8_t flag = static_cast<uint8_t>(targetTwist.linear.z);
    short sendVelX = static_cast<short>(targetTwist.linear.x * 1000);
    short sendVelY = static_cast<short>(targetTwist.linear.y * 1000);
    // short sendVelZ = targetTwist.angular.z * 1000;
    if(comm.sendData(flag,sendVelX,sendVelY) == true)
    {
        ROS_INFO("Send Data successfully!");
    }
    else
    {
        ROS_ERROR_STREAM("Unable To Send Data!");
    }
}