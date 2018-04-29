#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf2/utils.h>
#include <math.h>
#include <angles/angles.h>

// split the map into a 180x180 graph
// all nodes are spaced out by .1m
bool** graph = new bool*[180];
geometry_msgs::Pose current_pose;

float roundTenth(float a) {
  return round(10 * (a)) / 10;
}

void subPose(const geometry_msgs::Pose msg) {
  current_pose = msg;
  float angle = tf2::getYaw(msg.orientation);
  angle = angles::normalize_angle(angle);
  float seeUpToX = 2 * cos(angle) + current_pose.position.x;
  float seeUpToY = 2 * sin(angle) + current_pose.position.y;

  float rounded_current_X = roundTenth(current_pose.position.x);
  float rounded_current_Y = roundTenth(current_pose.position.y);

  std::cout << "I can see from " << rounded_current_X << "up to " << roundTenth(seeUpToX)
    << " and " << rounded_current_Y << " to " << roundTenth(seeUpToY) << "\n";

  

  for (rounded_current_X; rounded_current_X < seeUpToX; rounded_current_X += 0.1) {

    for (rounded_current_Y; rounded_current_Y < seeUpToY; rounded_current_Y += 0.1) {

    }
  }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "graph");
  ros::NodeHandle nh;

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose>("perfect_localization", 1000, subPose);

  for (int i = 0; i < 180; ++i) {
    graph[i] = new bool[180];
    for (int j = 0; j < 180; ++j) {
      graph[i][j] = false;
    }
  }


  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
