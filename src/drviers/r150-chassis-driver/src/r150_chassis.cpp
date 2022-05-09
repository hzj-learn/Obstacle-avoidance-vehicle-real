/*
 * @Author: Jian Xu @Xag  
 * @Date: 2020-09-22 17:39:54 
 * @Last Modified by: Jian Xu @Xag 
 * @Last Modified time: 2020-09-22 19:11:37
 */

//#include "chassis_driver/r150_chassis.h"
#include "../include/chassis_driver/r150_chassis.h"

using namespace Xag;

R150Chassis::R150Chassis()
{
    uint32_t rate = 50;
    xag_chassis_ptr_ = std::make_shared<xag_chassis::communication::FCComComponent>(rate);
    r150_control_cmd_.controlFlag = 0x00;
    r150_control_cmd_.Rev1[0] = 0x00;
    r150_control_cmd_.Rev1[1] = 0x00;
    r150_control_cmd_.Rev1[2] = 0x00;
    r150_control_cmd_.speed = 0x00;
    r150_control_cmd_.yawRate = 0x00;
}

void R150Chassis::initializeCommunication()
{
    xag_chassis_ptr_->initialize();
    // 注册
    xag_chassis_ptr_->process();

    if (xag_chassis_ptr_->fcComOnline())
        ROS_INFO_ONCE("========MODULE_STATUS_ONLINE========");
    else
        ROS_INFO_ONCE("========MODULE_STATUS_OFFLINE========");
}

void R150Chassis::sendOdomInquireCmd()
{
    odom_cmd_request_.subcmd = 0x02;
    odom_cmd_request_.number = 0x02;
    // 等待完成后需要3个结构体
    // odo_cmd_request_.number = 0x03;
    odom_cmd_request_.reserved1 = 0xFF;
    odom_cmd_request_.id = 0x01;
    odom_cmd_request_.reserved2 = 0xFF;
    // FC time
    odom_cmd_request_.offset[0] = (0x1C0 / 8);
    odom_cmd_request_.length[0] = (0x20 / 8);
    odom_cmd_request_.type[0] = 0;
    // RollAngle PitchAngle YawAngle
    odom_cmd_request_.offset[1] = (0x60 / 8);
    odom_cmd_request_.length[1] = (0x30 / 8);
    odom_cmd_request_.type[1] = 1;
    // xy 待完成
    // odom_cmd_request_.offset[2] = (0x80/8);
    // odom_cmd_request_.length[2] = (0x40/8);
    // odom_cmd_request_.type[2] = 13;

    xag_chassis_ptr_->sendCmdForStruct(0x42, MODULE_TYPE_FC, odom_cmd_request_);
}

void R150Chassis::getOdomData()
{
    // get data
    xag_chassis_ptr_->getOdomFromDataWarehouse();
    Eigen::Vector3d _eulerAngle(xag_chassis_ptr_->odomOrientation());
    // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle_[0], Eigen::Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle_[1], Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle_[2], Eigen::Vector3d::UnitZ()));
    // Eigen::Quaterniond _quaternion = yawAngle * pitchAngle * rollAngle;
    Eigen::Quaterniond _quaternion = convertAngleToQuaternion(_eulerAngle);

    // get position
    // Eigen::Vector3d _position (xag_chassis_ptr_->odomPosition());

    odom_data_.header.frame_id = "odom";
    odom_data_.header.stamp = ros::Time::now();
    odom_data_.pose.pose.position.x = 0.0;
    odom_data_.pose.pose.position.y = 0.0;
    // 待完善
    // odom_data_.pose.pose.position.x = _position[0];
    // odom_data_.pose.pose.position.y = _position[1];
    odom_data_.pose.pose.position.z = 0.0;
    odom_data_.pose.pose.orientation.x = _quaternion.x();
    odom_data_.pose.pose.orientation.y = _quaternion.y();
    odom_data_.pose.pose.orientation.z = _quaternion.z();
    odom_data_.pose.pose.orientation.w = _quaternion.w();

    //ROS_INFO_STREAM("pose.orientation.x: "<<odom_data_.pose.pose.orientation.x );
}

Eigen::Quaterniond R150Chassis::convertAngleToQuaternion(const Eigen::Vector3d &angle)
{
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(angle[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(angle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(angle[2], Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
    return quaternion;
}

void R150Chassis::sendControlCmd(const int32_t &speed, const int32_t &yawRate)
{
    //ROS_INFO_STREAM("speed: " << speed);
    //ROS_INFO_STREAM("yawRate: " << yawRate);
    r150_control_cmd_.controlFlag = 0x01;
    r150_control_cmd_.Rev1[0] = 0x00;
    r150_control_cmd_.Rev1[1] = 0x00;
    r150_control_cmd_.Rev1[2] = 0x00;
    r150_control_cmd_.speed = speed;
    r150_control_cmd_.yawRate = yawRate;    

    //ROS_INFO_STREAM("r150_control_cmd_.speed: " << r150_control_cmd_.speed);
    //ROS_INFO_STREAM("r150_control_cmd_.yawRate: " << r150_control_cmd_.yawRate);
}

void R150Chassis::sendCmdVelCmd()
{
    xag_chassis_ptr_->sendControlCmd(0x36, MODULE_TYPE_FC, 0x24, r150_control_cmd_);
}
