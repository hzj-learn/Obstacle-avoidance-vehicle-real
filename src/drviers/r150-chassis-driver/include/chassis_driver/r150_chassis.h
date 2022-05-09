/*
 * @Author: Jian Xu @Xag  
 * @Date: 2020-09-22 14:45:35 
 * @Last Modified by: Jian Xu @Xag 
 * @Last Modified time: 2020-09-22 17:40:23
 */

#ifndef CHASSIS_DRIVER_R150_CHASSIS_H_
#define CHASSIS_DRIVER_R150_CHASSIS_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
//#include "communication/fc_com.h"
#include "../../lib/communication/fc_com.h"

namespace Xag
{
    class R150Chassis
    {
    public:
        R150Chassis();
        void initializeCommunication();
        void sendOdomInquireCmd();
        void sendCmdVelCmd();
        void getOdomData();
        Eigen::Quaterniond convertAngleToQuaternion(const Eigen::Vector3d &angle);

        void sendControlCmd(const int32_t &speed, const int32_t &yawRate);

        nav_msgs::Odometry odomData() { return odom_data_; }

    private:
        std::shared_ptr<xag_chassis::communication::FCComComponent> xag_chassis_ptr_;
        nav_msgs::Odometry odom_data_;

        xag_chassis::communication::Fd_Request_Multiple_t odom_cmd_request_;
        xag_chassis::communication::R150_Control_Cmd_t r150_control_cmd_;
    };
} // namespace Xag

#endif