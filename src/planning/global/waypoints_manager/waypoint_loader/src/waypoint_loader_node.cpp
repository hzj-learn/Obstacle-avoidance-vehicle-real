#include <waypoint_loader/waypoint_loader.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_loader_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    WaypointLoader waypoint_loader(nh,pnh);
    ros::spin();
    return 0;
}
