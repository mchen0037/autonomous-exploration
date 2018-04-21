#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>


//jason edit
void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
    ROS_INFO_STREAM("Service still running");
    ROS_INFO_STREAM("Current pose (x,y) " <<
        fb->base_position.pose.position.x << "," <<
        fb->base_position.pose.position.y);
}
//jason edit


int main(int argc, char** argv) {
  ros::init(argc, argv, "movebasetest");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
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

  geometry_msgs::Twist twist;
  twist.linear.x = -1.5;

  for (double d = -8.0; d < 8.5; d = d + .75) {
    goal.target_pose.pose.position.x = d;
    ac.sendGoal(goal,&serviceFeedback);
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO_STREAM("Success");
    else {
      ROS_ERROR_STREAM("Failure, going back 1 meter");
      for (int i = 0; i < 20; ++i) {
        pub.publish(twist);
        rate.sleep();
      }
      d = d - 0.25;
    }
  }



}
