#include <ros/ros.h>
#include <tf2/utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <angles/angles.h>
#include <map>
#include <std_msgs/String.h>

std::string graph;
// std_msgs::Int32MultiArray arr;
geometry_msgs::Pose current_pose;
bool gotMap = false;

int matToArr(int row, int col, int row_tar, int col_tar){
  return (row_tar-1) * col + col_tar - 1;
}

int coordToArr(int x, int y) {
  return std::abs(-10 - x) * 2 + std::abs(10 - y) * 2 * 40;
}

float roundHalf(float a) {
  return (floor((a*2)+0.5)/2);
}

void discretize(const nav_msgs::OccupancyGrid msg){
  if (!gotMap) {
    gotMap = true;
    int square = 10; // square x square
    int interval = 0; //interval for my array

  //gets me to the start of every square
    for (int j = 200; j < 600; j+=square) {
      for (int i = 200; i < 600;i+=square) {

        for(int l = 0; l < square; l++) {
          for(int k = 0; k < square; k++) {
            if((int)msg.data[matToArr(800,800,j+l,i+k)] != 0 && graph[interval] == 0) { //if you want actual count delete second condition
              graph[interval] = 1;
            }
          }
        }
        interval++;
      }

    }
 }
}

void subPose(const geometry_msgs::Pose msg) {
  current_pose = msg;
  float angle = tf2::getYaw(msg.orientation);
  angle = angles::normalize_angle(angle);
  float seeUpToX = 2 * cos(angle) + current_pose.position.x;
  float seeUpToY = 2 * sin(angle) + current_pose.position.y;

  float rounded_current_X = roundHalf(current_pose.position.x);
  float rounded_current_Y = roundHalf(current_pose.position.y);

  // std::cout << "I can see from " << rounded_current_X << " up to " << roundHalf(seeUpToX)
    // << " and " << rounded_current_Y << " to " << roundHalf(seeUpToY) << "\n";

  for (rounded_current_X; rounded_current_X < seeUpToX; rounded_current_X += 0.5) {
    for (rounded_current_Y; rounded_current_Y < seeUpToY; rounded_current_Y += 0.5) {
      graph[coordToArr(rounded_current_X, rounded_current_Y)] = 2;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "discreteMap");
  ros::NodeHandle nh;
  ros::Rate rate(20);
  ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, discretize);
  ros::Subscriber sub2 = nh.subscribe<geometry_msgs::Pose>("perfect_localization", 1000, subPose);
  ros::Publisher pub = nh.advertise<std_msgs::String>("viewed_map", 1000);

  graph = new char[1600];
  ROS_INFO_STREAM(graph);
  for (int i = 0; i < 1600; ++i) {
    graph[i] = 0;
  }

  while (ros::ok()) {
    ROS_INFO_STREAM(graph);

    std_msgs::String msg;
    msg.data = graph;

    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

}
