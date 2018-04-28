#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/GetPlan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


geometry_msgs::Pose current_pose;

void getCurrentPose(const geometry_msgs::Pose msg) {
  current_pose = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "movement");
  ros::NodeHandle nh;

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose>("perfect_localization", 1000, getCurrentPose);
  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  client.waitForExistence();

  for (int i = 0; i < 80; ++i) {
    rate.sleep();
    ros::spinOnce();
  }

  nav_msgs::GetPlan plannermsg;
  tf2::Quaternion q;

  while (ros::ok()) {

    plannermsg.request.start.header.frame_id = "map";
    plannermsg.request.start.header.stamp = ros::Time::now();
    plannermsg.request.start.pose.position.x = current_pose.position.x;
    plannermsg.request.start.pose.position.y = current_pose.position.y;
    plannermsg.request.start.pose.orientation = current_pose.orientation;

    q.setRPY(0, 0, 0);

    plannermsg.request.goal.header.frame_id = "map";
    plannermsg.request.goal.header.stamp = ros::Time::now();
    plannermsg.request.goal.pose.position.x = -8;
    plannermsg.request.goal.pose.position.y = 8;
    plannermsg.request.goal.pose.orientation = tf2::toMsg(q);

    plannermsg.request.tolerance = 0.2;

    if (client.call(plannermsg)) {
      ROS_INFO_STREAM("Called planner");
      ROS_INFO_STREAM("Plan:");
      for (int i = 0; i < plannermsg.response.plan.poses.size(); ++i) {
        tf2::fromMsg(plannermsg.response.plan.poses[i].pose.orientation, q);
        geometry_msgs::Pose nextGoal = plannermsg.response.plan.poses[i];




        ROS_INFO_STREAM("X " << plannermsg.response.plan.poses[i].pose.position.x <<
          " Y " << plannermsg.response.plan.poses[i].pose.position.y <<
          " T " << tf2::getYaw(q));
      }
    }
    else {
      ROS_ERROR_STREAM("Error");
    }

    ros::spinOnce();
    rate.sleep();
  }

}
