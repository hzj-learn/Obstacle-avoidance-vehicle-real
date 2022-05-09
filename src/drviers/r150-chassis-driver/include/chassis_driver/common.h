/*
 * @Author: Jian Xu @Xag  
 * @Date: 2020-07-23 14:23:00 
 * @Last Modified by: Jian Xu @Xag 
 * @Last Modified time: 2020-07-23 14:38:47
 */

#ifndef _TOOLS_COMMON_H_
#define _TOOLS_COMMON_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

namespace Tools
{
    /** \brief Construct a new point cloud message from the specified information and publish it via the given publisher.
 *
 * @tparam PointT the point type
 * @param publisher the publisher instance
 * @param cloud the cloud to publish
 * @param stamp the time stamp of the cloud message
 * @param frameID the message frame ID
 */
    inline void publishCloudMsg(ros::Publisher &publisher,
                                const pcl::PointCloud<PointT> &cloud,
                                const ros::Time &stamp,
                                std::string frameID)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = frameID;
        publisher.publish(msg);
    }
} // namespace Tools
#endif