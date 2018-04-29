#include <ros/ros.h>
#include <tf2/utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <angles/angles.h>
#include <map>

std::vector <int> graph;
geometry_msgs::Pose current_pose;
bool gotMap = false;

int matToArr(int row, int col, int row_tar, int col_tar){
  return (row_tar-1) * col + col_tar - 1;

}

float roundTenth(float a) {
  return round(10 * (a)) / 10;
}

void discretize(const nav_msgs::OccupancyGrid msg){
  if (!gotMap) {
    gotMap = true;
    int square = 10; // square x square
    //gets me to the start of every square
    for (int j = 200; j < 600; j+=square) {
      for (int i = 200; i < 600;i+=square) {
        int count = 0;
        for(int l = 0; l < square; l++) {
          for(int k = 0; k < square; k++) {
            if((int)msg.data[matToArr(800,800,j+l,i+k)] != 0 && count < 1) {
              count++;
            }
          }
        }
        graph.push_back(count);
      }
    }
    // for(int i =0; i < graph.size(); i++){
    //   std::cout << graph[i] << "\t";
    //   if ( (i+1)%(400/square) == 0 ){
    //     std::cout <<"\n\n";
    //   }
   // }
 }
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
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "discreteMap");
  ros::NodeHandle nh;
  ros::Rate rate(20);
  ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, discretize);
  ros::Subscriber sub2 = nh.subscribe<geometry_msgs::Pose>("perfect_localization", 1000, subPose);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

}
