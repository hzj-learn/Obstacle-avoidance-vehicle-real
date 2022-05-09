#ifndef WAYPOINT_LOADER_WAYPOINT_LOADER_H_INCLUDED
#define WAYPOINT_LOADER_WAYPOINT_LOADER_H_INCLUDED

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include <fstream>
#include <sstream>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class WaypointLoader
{
public:
    WaypointLoader(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~WaypointLoader();
private:
    void PublishWaypoints_(void);
    void LoadWaypointsArray_(void);
    void LoadWaypoint_(const std::string& line, geometry_msgs::PoseStamped* wp);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Time current_time_;
    
    std::string path_topic_;
    std::string waypoints_path_;
    std::string map_frame_;
    nav_msgs::Path wps_;
    
    ros::Publisher wps_pub_;
};

#endif  //WAYPOINT_LOADER_WAYPOINT_LOADER_H_INCLUDED

