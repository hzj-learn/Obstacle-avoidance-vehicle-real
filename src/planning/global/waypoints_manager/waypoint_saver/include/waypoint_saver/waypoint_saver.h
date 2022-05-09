#ifndef WAYPOINT_SAVER_WAYPOINT_SAVER_H_INCLUDED
#define WAYPOINT_SAVER_WAYPOINT_SAVER_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>
#include <sstream>

class WaypointSaver
{
public:
    WaypointSaver(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~WaypointSaver();
private:
    void InitializeWaypointsFile_(void);
    void PublishPointsMarkerArray_(geometry_msgs::PoseStamped pose);
    void CurrentPoseCallback_(const geometry_msgs::PoseStamped::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::string wps_marker_topic_;
    std::string current_pose_topic_;
    std::string waypoints_path_;
    std::string map_frame_;
    double wp_interval_;

    std::ofstream ofs_;
    bool init_pose_received_;
    geometry_msgs::PoseStamped previous_pose_;

    
    ros::Subscriber current_pose_sub_;
    ros::Publisher wps_marker_pub_;
};

#endif  //WAYPOINT_SAVER_WAYPOINT_SAVER_H_INCLUDED

