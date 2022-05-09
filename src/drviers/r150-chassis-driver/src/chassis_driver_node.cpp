/*
 * @Author: Jian Xu @Xag
 * @Date: 2020-09-21 14:09:06
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2020-09-21 14:09:55
 */

#include "chassis_driver/chassis_driver.h"

using ChassisDriver = Xag::ChassisDriver;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis_driver");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    ChassisDriver chassisDriver;
    if (chassisDriver.setup(&node, &private_node))
    {
        ros::spin();
    }

    return 0;
}