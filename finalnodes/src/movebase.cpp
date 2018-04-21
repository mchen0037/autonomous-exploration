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


void serviceActivated() {
    ROS_INFO_STREAM("Service received goal");
}

void serviceDone(const actionlib::SimpleClientGoalState& state,
     const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO_STREAM("Service completed");
    ROS_INFO_STREAM("Final state " << state.toString().c_str());
    //ros::shutdown();
}

// end Jason edit

void mySendGoal(float x, float y, ros::Publisher* pub) {
  
  ros::Rate rate(20);

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = 1;

  ROS_INFO_STREAM("Goal: " << goal.target_pose.pose.position.x << " , " << goal.target_pose.pose.position.y);
  ac.sendGoal(goal,&serviceDone,&serviceActivated, &serviceFeedback);
  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Success");
  else {
    ROS_ERROR_STREAM("Failure, going back 1 meter");
    for (int i = 0; i < 60; ++i) {
      pub->publish(twist);
      rate.sleep();
    }
  }

}


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
  twist.linear.x = -0.75;

  bool toPositive = true;

  for (float y = 8.5; y > -8.5; y = y - 1.0) {
    if (toPositive) {
      for (float x = -8.0; x < 8.5; x = x + 1.0) {
        mySendGoal(x, y, &pub);
      }
    }
    else {
      for (float x = 8.0; x > -8.5; x = x - 1.0) {
        mySendGoal(x, y, &pub);
      }
    }
  }


  // for (double d = -8.0; d < 8.5; d = d + .75) {
  //   goal.target_pose.pose.position.x = d;
  //   //Jason added (&service feedback, &serviceActivated and &serviceDone)
  //   ROS_INFO_STREAM("Goal: " << goal.target_pose.pose.position.x << " , " << goal.target_pose.pose.position.y);
  //   ac.sendGoal(goal,&serviceDone,&serviceActivated, &serviceFeedback);
  //   ac.waitForResult();
  //   if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //     ROS_INFO_STREAM("Success");
  //   else {
  //     ROS_ERROR_STREAM("Failure, going back 1 meter");
  //     for (int i = 0; i < 60; ++i) {
  //       pub.publish(twist);
  //       rate.sleep();
  //     }
  //     // d = d - 0.25;
  //   }
  // }



}
