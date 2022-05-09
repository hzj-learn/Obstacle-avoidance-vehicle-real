#include <waypoint_saver/waypoint_saver.h>

WaypointSaver::WaypointSaver(ros::NodeHandle nh,ros::NodeHandle pnh) : nh_(nh),pnh_(pnh)
{
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, "current_pose");
    pnh_.param<std::string>("wps_marker_topic", wps_marker_topic_, "waypoint_saver_marker");
    pnh_.param<std::string>("waypoints_csv", waypoints_path_, "/tmp/saved_waypoints.csv");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<double>("wp_interval", wp_interval_, 1.0);
    wps_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(wps_marker_topic_, 1);

    init_pose_received_ = false;
    WaypointSaver::InitializeWaypointsFile_();

    current_pose_sub_ = nh_.subscribe(current_pose_topic_, 10, &WaypointSaver::CurrentPoseCallback_, this);
}

WaypointSaver::~WaypointSaver()
{
    ofs_.close();
}

void WaypointSaver::InitializeWaypointsFile_(void)
{
    ofs_.open(waypoints_path_, std::ios::app);
    //ofs_ << "x,y,z,yaw,velocity,mission" << std::endl;
}

void WaypointSaver::PublishPointsMarkerArray_(geometry_msgs::PoseStamped pose)
{
    static visualization_msgs::MarkerArray marray;
    static int id = 0;

    visualization_msgs::Marker marker;
    marker.id = id;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time();
    marker.frame_locked = true;

    marker.ns = "saved_waypoints_arrow";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose = pose.pose;
    marray.markers.push_back(marker);

    wps_marker_pub_.publish(marray);
    id++;
}

void WaypointSaver::CurrentPoseCallback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    geometry_msgs::PoseStamped current_pose = *msg;

    if(!init_pose_received_)
    {
        ofs_ << std::fixed << std::setprecision(4) << current_pose.pose.position.x << "," << current_pose.pose.position.y << ","
             << current_pose.pose.position.z << "," << tf::getYaw(current_pose.pose.orientation) << ",0,0" << std::endl;
        PublishPointsMarkerArray_(current_pose);
        previous_pose_ = current_pose;
        init_pose_received_ = true;
    }
    else
    {
        double distance = sqrt(pow((current_pose.pose.position.x - previous_pose_.pose.position.x), 2) + pow((current_pose.pose.position.y - previous_pose_.pose.position.y), 2));

        if(distance > wp_interval_)
        {
            ofs_ << std::fixed << std::setprecision(4) << current_pose.pose.position.x << "," << current_pose.pose.position.y << ","
                 << current_pose.pose.position.z << "," << tf::getYaw(current_pose.pose.orientation) << ",0,0" << std::endl;
            PublishPointsMarkerArray_(current_pose);
            previous_pose_ = current_pose;
        }
    }
}
