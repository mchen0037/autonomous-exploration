#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "graph");
  ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, buildGraph);


}
