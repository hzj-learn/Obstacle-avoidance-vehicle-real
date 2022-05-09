#include <ros/ros.h>  
#include <move_base_msgs/MoveBaseAction.h>  
#include <actionlib/client/simple_action_client.h> 
#include <iostream>
#include<vector>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Int8.h"
#include "actionlib_msgs/GoalID.h"

ros::Subscriber sub_goal;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

std::vector<geometry_msgs::PoseStamped> pose_list;

// 发布目标点
void setGoal(geometry_msgs::PoseStamped pose_msg)
{
     // 定义客户端 
    MoveBaseClient ac("move_base", true);  
      
    // 等待服务器
    while(!ac.waitForServer(ros::Duration(5.0))){  
        ROS_WARN("Waiting for the move_base action server to come up");  
    }  
    
    // 创建action的goal
    move_base_msgs::MoveBaseGoal goal;  
      
    // 发送位置坐标 
    goal.target_pose.header.frame_id = "map";  
    goal.target_pose.header.stamp = ros::Time::now();  
   
    goal.target_pose.pose = pose_msg.pose;
    
    ROS_INFO("Sending goal");  
    
    // 发送action的goal给服务器端，并且设置回调函数
    ac.sendGoal(goal);

    // 等待结果 
    ac.waitForResult();  
      
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
        ROS_INFO("it is successful"); 
    } 
     else  
     {
         ROS_ERROR("The base failed  move to goal!!!");
     }
       
}

// 目标点回调
void GoalPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //pose_list.push_back(*msg);
    //for(int i=0; i < pose_list.size(); i++)
    setGoal(*msg);
}

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "multi_goals_nav");  
  ros::NodeHandle nh;

  sub_goal = nh.subscribe("goal", 1000, GoalPoseCB);
   
  MoveBaseClient ac("move_base", true);

  ros::spin();
  return 0;  
}  
 
