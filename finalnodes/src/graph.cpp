#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>

vector < vector < bool > > graph;

geometry_msgs::Pose current_pose;
void subPose(const geometry_msgs::Pose msg) {
  current_pose = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph");
  ros::NodeHandle nh;

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose>("perfect_localization", 1000, subPose);
  
}
