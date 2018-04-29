#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Empty.h>
#include <tf2/utils.h>
#include <iostream>

bool wait = true;

void toReset(const std_msgs::Empty msg){

  wait = false;
}


geometry_msgs::Pose current_pose;

void getCurrentPose(const geometry_msgs::Pose msg) {
  current_pose = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "movement");
  ros::NodeHandle nh;

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose>("perfect_localization", 1000, getCurrentPose);

  ros::Publisher pubTwist = nh.advertise<geometry_msgs::Pose>("targetpose", 1000);

  ros::Subscriber restart = nh.subscribe<std_msgs::Empty>("restartTopic", 1000, toReset);

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
      for (int i = 0; i < plannermsg.response.plan.poses.size(); i = (i +30>= plannermsg.response.plan.poses.size()) ? plannermsg.response.plan.poses.size()-1 : i+30 ) {
        tf2::fromMsg(plannermsg.response.plan.poses[i].pose.orientation, q);
        geometry_msgs::PoseStamped nextGoal = plannermsg.response.plan.poses[i];
        ROS_INFO_STREAM(nextGoal);
        getchar();

        // for (int j = 0; j < 20; ++j) {
        //   pubTwist.publish(nextGoal.pose);
        //   rate.sleep();
        // }

        // wait = true;

        // ROS_INFO_STREAM("X " << plannermsg.response.plan.poses[i].pose.position.x <<
        //   " Y " << plannermsg.response.plan.poses[i].pose.position.y <<
        //   " T " << tf2::getYaw(q));

        // ROS_INFO_STREAM("Waiting to finish..");
        // while(wait && ros::ok()){
        //   pubTwist.publish(nextGoal.pose);
        //   ros::spinOnce();
        //   rate.sleep();
        //   // ROS_INFO_STREAM("IM WAITING");
        // }


      }
    }
    else {
      ROS_ERROR_STREAM("Error");
    }

    ros::spinOnce();
    rate.sleep();
  }

}
