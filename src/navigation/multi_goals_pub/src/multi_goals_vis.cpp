#include <ros/ros.h>  
#include <iostream>
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

static std::deque<geometry_msgs::PoseStamped> goal_queue;

ros::Publisher pub_vis_goal;
ros::Subscriber sub_goal;

// 目标点可视化
void PubVisGoal(const std::deque<geometry_msgs::PoseStamped> &goal_queue){
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time::now();
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = "goals";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.emplace_back(marker);
  for(int i = 0; i < goal_queue.size(); i++){
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "goals";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = goal_queue[i].pose;
    marker.scale.z = 1.0;
    marker.color.r = 0.0; marker.color.g = 0.4; marker.color.b = 1.0; marker.color.a = 1.0;
    marker.text = std::to_string(i + 1);
    marker_array.markers.emplace_back(marker);
  }

  pub_vis_goal.publish(marker_array);
}

// 目标点回调
void GoalPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
  goal_queue.emplace_back(*msg);
  PubVisGoal(goal_queue); //可视化
}

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "multi_goals_vis");  
  ros::NodeHandle nh;

  sub_goal = nh.subscribe("goal", 100, GoalPoseCB);
 pub_vis_goal = nh.advertise<visualization_msgs::MarkerArray>("vis_goals", 100);
  
ros::spin();
  return 0;  
}  
 
