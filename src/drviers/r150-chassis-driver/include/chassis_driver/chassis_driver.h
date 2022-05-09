/*
 * @Author: Jian Xu @Xag
 * @Date: 2020-09-21 11:31:49
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2020-09-21 20:20:51
 */

#ifndef CHASSIS_DRIVER_CHASSIS_DRIVER_H_
#define CHASSIS_DRIVER_CHASSIS_DRIVER_H_

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include "r150_chassis.h"

namespace Xag
{
    class ChassisDriver
    {
    public:
        bool setup(ros::NodeHandle *node, ros::NodeHandle *private_node);
        void run();
        void cmdVelHandler(const geometry_msgs::Twist::ConstPtr &cmd_vel);

        void loopTimerHandler(const ros::TimerEvent &);
        void odomCmdTimerHandler(const ros::TimerEvent &);
        void odomReceiveTimerHandler(const ros::TimerEvent &);
        void cmdVelCmdTimerHandler(const ros::TimerEvent &);

    private:
        // subscribe cmd vel
        ros::Subscriber sub_cmd_vel_;

        // publish odom(test)
        ros::Publisher pub_odom_;

        // loop timer
        ros::Timer loop_timer_;
        // send odom cmd timer
        ros::Timer odom_cmd_timer_;
        // odom timer
        ros::Timer odom_rece_timer_;
        // cmdVelCmd timer
        ros::Timer cmd_vel_cmd_timer_;

        R150Chassis r150_chassis_;
    };
} // namespace Xag

#endif