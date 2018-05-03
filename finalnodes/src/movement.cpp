#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Empty.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <stdlib.h>
#include <sensor_msgs/LaserScan.h>

bool wait = true;
geometry_msgs::Pose current_goal;
geometry_msgs::Pose current_pose;
float current_min_scan = 999.9;

void toReset(const std_msgs::Empty msg){
  wait = false;
}

void getCurrentPose(const geometry_msgs::Pose msg) {
  current_pose = msg;
}

void getGoal(const geometry_msgs::Pose msg) {
  current_goal = msg;
}

float calcOrientation(geometry_msgs::Pose* current, geometry_msgs::Pose* goal) {
  float x = goal->position.x - current->position.x;
  float y = goal->position.y - current->position.y;

  float angle = atan2(y, x);

  tf2::Quaternion q;
  q.setRPY(0,0,angle);

  ROS_INFO_STREAM("Angle: " << angle);

  return angle; 
}

void getScan(const sensor_msgs::LaserScan msg) {
  float min = 999.9;
  for (int i = 0; i < msg.ranges.size(); ++i) {
    if (msg.ranges[i] < min) {
      min = msg.ranges[i];
    }
  }

  current_min_scan = min;
}


int main(int argc, char** argv) { 
  ros::init(argc, argv, "movement");
  ros::NodeHandle nh;

  srand( time( NULL ) );

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose>("perfect_localization", 1000, getCurrentPose);
  ros::Publisher pubGoal = nh.advertise<geometry_msgs::Pose>("targetpose", 1000);
  ros::Subscriber restart = nh.subscribe<std_msgs::Empty>("restartTopic", 1000, toReset);
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::Pose>("goal", 1, getGoal);
  ros::Subscriber subScan = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, getScan);

  ros::Publisher sketchPub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);

  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  client.waitForExistence();

  for (int i = 0; i < 80; ++i) {
    rate.sleep();
    ros::spinOnce();
  }

  nav_msgs::GetPlan plannermsg;
  tf2::Quaternion q;


restart:
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();

    if (current_min_scan < 0.8) {
      geometry_msgs::Twist twist;
      twist.linear.x = -0.2;
      ROS_INFO_STREAM("Too close to wall.. backing up.");
      for (int i = 0; i < 40; ++i) {
        sketchPub.publish(twist);
        rate.sleep();
      }
      goto restart;
    }

    plannermsg.request.start.header.frame_id = "map";
    plannermsg.request.start.header.stamp = ros::Time::now();
    plannermsg.request.start.pose.position.x = current_pose.position.x;
    plannermsg.request.start.pose.position.y = current_pose.position.y;
    plannermsg.request.start.pose.orientation = current_pose.orientation;

    q.setRPY(0, 0, 0);

    ROS_INFO_STREAM(current_goal);

    plannermsg.request.goal.header.frame_id = "map";
    plannermsg.request.goal.header.stamp = ros::Time::now();
    plannermsg.request.goal.pose.position.x = current_goal.position.x;
    plannermsg.request.goal.pose.position.y = current_goal.position.y;
    plannermsg.request.goal.pose.orientation = current_goal.orientation;

    if (plannermsg.request.goal.pose.position.x > 8.5) {
      plannermsg.request.goal.pose.position.x = 8.5;
    }
    else if (plannermsg.request.goal.pose.position.x < -8.5) {
      plannermsg.request.goal.pose.position.x = -8.5;
    }
    if (plannermsg.request.goal.pose.position.y > 8.5) {
      plannermsg.request.goal.pose.position.y = 8.5;
    }
    else if (plannermsg.request.goal.pose.position.y < -8.5) {
      plannermsg.request.goal.pose.position.y = -8.5;
    }

    plannermsg.request.tolerance = 0.3;

    if (client.call(plannermsg)) {
      ROS_INFO_STREAM("Called planner");
      ROS_INFO_STREAM("Plan:");

      if (plannermsg.response.plan.poses.size() == 0) {
        BACKITUP:
        ROS_INFO_STREAM("I think I'm stuck... randomly move."); 
        geometry_msgs::Twist twist;

        twist.angular.z = -1 * calcOrientation(&current_pose, &current_goal);
        twist.linear.x = ((double)(rand() % 5 - 2))/5;

        for (int i = 0; i < 40; ++i) {
          sketchPub.publish(twist);
          rate.sleep();
        }

        goto restart;
      }

      for (int i = 0; i < plannermsg.response.plan.poses.size(); i = (i +10>= plannermsg.response.plan.poses.size()) ? plannermsg.response.plan.poses.size()-1 : i+10 ) {
        tf2::fromMsg(plannermsg.response.plan.poses[i].pose.orientation, q);
        geometry_msgs::PoseStamped nextGoal = plannermsg.response.plan.poses[i];

        // for (int j = 0; j < 20; ++j) {
        //   pubGoal.publish(nextGoal.pose);
        //   rate.sleep();
        // }

        wait = true;

        nextGoal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, calcOrientation(&plannermsg.response.plan.poses[i].pose, &plannermsg.response.plan.poses[i + 1].pose));


        ROS_INFO_STREAM("Subgoal: X " << nextGoal.pose.position.x <<
          " Y " << nextGoal.pose.position.y <<
          " T " << tf2::getYaw(nextGoal.pose.orientation));

        ROS_INFO_STREAM("Waiting to finish..");
        ros::Time start = ros::Time::now();

        while(wait && ros::ok()/* && ros::Time::now() - start < ros::Duration(20.0)*/) {
          pubGoal.publish(nextGoal.pose);
          ros::spinOnce();
          rate.sleep();
        }

        // if (ros::Time::now() - start < ros::Duration(20.0)) {
        //   ROS_INFO_STREAM("Took too long.");
        //   goto BACKITUP;
        // }

        if (i == plannermsg.response.plan.poses.size() -1) {
          break;
        }
      }
    }
    else {
      ROS_ERROR_STREAM("Error");
    }

    ros::spinOnce();
    rate.sleep();
  }

}
