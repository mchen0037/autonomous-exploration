#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "movebasetest");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  while(!ac.waitForServer()) {
  }
  ROS_INFO_STREAM("Server Available!");

  geometry_msgs::Quaternion q;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -8.5;
  goal.target_pose.pose.position.y = 8.5;
  goal.target_pose.pose.orientation.w = 1;

  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Success");
  else
    ROS_ERROR_STREAM("Failure");

  for (double d = -8.0; d < 8.5; d = d + 0.5) {
    goal.target_pose.pose.position.x = d;
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO_STREAM("Success");
    else
      ROS_ERROR_STREAM("Failure");
  }



}
