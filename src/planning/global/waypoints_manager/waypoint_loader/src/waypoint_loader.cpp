#include <waypoint_loader/waypoint_loader.h>

WaypointLoader::WaypointLoader(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("path_topic", path_topic_, "/waypoints_raw");
    pnh_.param<std::string>("waypoints_csv", waypoints_path_, "/tmp/waypoints.csv");
    pnh_.param<std::string>("map_frame", map_frame_, "odom");
    wps_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 10);
    WaypointLoader::LoadWaypointsArray_();
    boost::thread wp_publish_thread(boost::bind(&WaypointLoader::PublishWaypoints_, this));
}

WaypointLoader::~WaypointLoader()
{

}

void WaypointLoader::PublishWaypoints_(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        wps_pub_.publish(wps_);
        loop_rate.sleep();
    }
    return;
}

void WaypointLoader::LoadWaypointsArray_(void)
{
    current_time_ = ros::Time::now();
    wps_.header.stamp = current_time_;
    wps_.header.frame_id = map_frame_;

    std::ifstream ifs(waypoints_path_);

    if(!ifs)
    {
        ROS_ERROR("Error! File can not be opened");
        return;
    }

    std::string line;
    std::getline(ifs, line);  // Remove first line

    while(std::getline(ifs, line))
    {
        geometry_msgs::PoseStamped wp;
        LoadWaypoint_(line, &wp);
        wps_.poses.push_back(wp);
    }
}

void WaypointLoader::LoadWaypoint_(const std::string& line, geometry_msgs::PoseStamped* wp)
{
    std::vector<std::string> columns;
    std::string column;
    std::istringstream stream(line);

    while (getline(stream, column, ','))
    {
        columns.push_back(column);
    }

    wp->pose.position.x = std::stod(columns[0]);
    wp->pose.position.y = std::stod(columns[1]);
    wp->pose.position.z = std::stod(columns[2]);

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(std::stod(columns[3]));
    wp->pose.orientation.x = quat.x;
    wp->pose.orientation.y = quat.y;
    wp->pose.orientation.z = quat.z;
    wp->pose.orientation.w = quat.w;

    wp->header.stamp=current_time_;
    wp->header.frame_id=map_frame_;
}
