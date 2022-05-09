/*
 * @Author: Jian Xu @Xag
 * @Date: 2020-09-21 16:41:24
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2020-09-21 18:20:13
 */

//#include "chassis_driver/chassis_driver.h"
#include "../include/chassis_driver/chassis_driver.h"

using namespace Xag;

bool ChassisDriver::setup(ros::NodeHandle *node, ros::NodeHandle *private_node)
{
    sub_cmd_vel_ = node->subscribe("/smoother_cmd_vel", 10, &ChassisDriver::cmdVelHandler, this);
    pub_odom_ = node->advertise<nav_msgs::Odometry>("/odom", 50);

    loop_timer_ = node->createTimer(ros::Duration(0.02), &ChassisDriver::loopTimerHandler, this);
    odom_cmd_timer_ = node->createTimer(ros::Duration(0.02), &ChassisDriver::odomCmdTimerHandler, this);
    odom_rece_timer_ = node->createTimer(ros::Duration(0.02), &ChassisDriver::odomReceiveTimerHandler, this);
    cmd_vel_cmd_timer_ = node->createTimer(ros::Duration(0.1),&ChassisDriver::cmdVelCmdTimerHandler,this);
    return true;
}

void ChassisDriver::cmdVelHandler(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    int32_t speed_ = 0;
    int32_t yaw_rate_= 0;
    //ROS_INFO_STREAM("linear.x: "<<cmd_vel->linear.x);
    //ROS_INFO_STREAM("angular.z: "<<cmd_vel->angular.z);
    speed_ = cmd_vel->linear.x * 1000;
    yaw_rate_ = cmd_vel->angular.z * 57.3 *10;
    r150_chassis_.sendControlCmd(speed_, yaw_rate_);
}

void ChassisDriver::loopTimerHandler(const ros::TimerEvent &)
{
    r150_chassis_.initializeCommunication();
}

void ChassisDriver::odomCmdTimerHandler(const ros::TimerEvent &)
{
    r150_chassis_.sendOdomInquireCmd();
}

void ChassisDriver::odomReceiveTimerHandler(const ros::TimerEvent &)
{
    r150_chassis_.getOdomData();
    //pub_odom_.publish(r150_chassis_.odomData());
}

void ChassisDriver::cmdVelCmdTimerHandler(const ros::TimerEvent &)
{
    r150_chassis_.sendCmdVelCmd();
}
