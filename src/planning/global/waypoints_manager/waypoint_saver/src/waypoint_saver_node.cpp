#include <waypoint_saver/waypoint_saver.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_saver_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    WaypointSaver waypoint_saver(nh,pnh);
    ros::spin();
    return 0;
}
