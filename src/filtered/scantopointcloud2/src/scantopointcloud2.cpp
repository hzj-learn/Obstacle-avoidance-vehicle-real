#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include "laser_geometry/laser_geometry.h"

typedef pcl::PointXYZI PointType;

ros::Publisher *pubLaserCloud = NULL;
tf::TransformListener *listener = NULL;
laser_geometry::LaserProjection projector_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener->waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud2 scan_pointcloud;
  projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
          scan_pointcloud,*listener);
pubLaserCloud->publish(scan_pointcloud);
  // Do something with cloud.
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_pointcloud");

  ros::NodeHandle nh;

  ros::Subscriber scanSub = nh.subscribe("scan", 1000, scanCallback);
  ros::Publisher  cloudPub = nh.advertise<sensor_msgs::PointCloud2>("laser_points", 1000);

  pubLaserCloud = &cloudPub;

  tf::TransformListener lr;
  listener = &lr;

  ros::spin();
 
  return 0;
}