#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <stdlib.h> 

geometry_msgs::Pose goalToSee;
std_msgs::String map;

int coordToArr(int x, int y) {
  int val = std::abs(-10 - x) * 2 + std::abs(10 - y) * 2 * 40;
  return val;
}

int roundHalf(float a) {
  return (floor((a*2)+0.5)/2);
}

void findRandom(const ros::Publisher* pub) {

  int i = rand() % 1599;
  ros::Rate rate(20);

  while(map.data[i] != '0') {i = rand() % 1599;}
 

  float y = i / 40;
  float x = i % 40;
  goalToSee.position.x = -1 * 9 + (x * 0.5); 
  goalToSee.position.y = 9 - (y * 0.5);
  tf2::Quaternion q;
  q.setRPY(0,0,0);
  tf2::fromMsg(goalToSee.orientation, q);
  ROS_INFO_STREAM(goalToSee);
  for(int i = 0; i < 40; i++){
    pub->publish(goalToSee);
    rate.sleep();
  }
  
}


void getViewedMap(const std_msgs::String msg){
  map = msg;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "calcGoal");
  ros::NodeHandle nh;

  srand (time(NULL));

  ros::Rate rate(20);
  ros::Subscriber sub = nh.subscribe<std_msgs::String>("viewed_map", 1000, getViewedMap);
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("goal", 1); 

  for (int i = 0; i < 40; ++i) {
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok()) {

    ros::Time start = ros::Time::now();

    findRandom(&pub);
    while(ros::Time::now() - start < ros::Duration(10.0) && ros::ok()){

    }
    ros::spinOnce();
    rate.sleep();
  }
}
