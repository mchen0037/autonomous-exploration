#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::Pose goalToSee;
geometry_msgs::Pose current_pose;
std_msgs::String map;
float secondsToWait = 10.0;

int switcher = 2;

int coordToArr(int x, int y) {
  int val = std::abs(-10 - x) * 2 + std::abs(10 - y) * 2 * 40;
  return val;
}

int roundHalf(float a) {
  return (floor((a*2)+0.5)/2);
}

float euclidDistance(geometry_msgs::Pose* curr, geometry_msgs::Pose* goal) {
  return sqrt(pow(curr->position.x - goal->position.x, 2) + pow(curr->position.y - goal->position.y, 2));
}

void findRandom(const ros::Publisher* pub) {

  int i = rand() % 1599;

  if (switcher == 2) {
    while(map.data[i] != '0') {
      int x = rand() % 20;
      int y = rand() % 20;
      i = 40 * y + x;
    }
    switcher = 3;
  }
  else if (switcher == 3) {
    while(map.data[i] != '0') {
      int x = rand() % 20;
      int y = rand() % 20 + 20;
      i = 40 * y + x;
    }
    switcher = 4;
  }
  else if (switcher == 4) {
    while(map.data[i] != '0') {
      int x = rand() % 20 + 20 ;
      int y = rand() % 20 + 20 ;
      i = 40 * y + x;
    }
    switcher = 1;
  }
  else if (switcher == 1) {
    while(map.data[i] != '0') {
      int x = rand() % 20 + 20 ;
      int y = rand() % 20;
      i = 40 * y + x;
    }
    switcher = 2;    
  }
  else {
    switcher = 1;
  }
  
  ros::Rate rate(20);

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

  secondsToWait = 5 * euclidDistance(&current_pose, &goalToSee);
  
}

void getCurrentPose(const geometry_msgs::PoseWithCovarianceStamped msg) {
    current_pose = msg.pose.pose;
}

void getViewedMap(const std_msgs::String msg){
  map = msg;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "calcGoal");
  ros::NodeHandle nh;

  srand (time(NULL));

  ros::Rate rate(20);
  ros::Subscriber sub2 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1000, getCurrentPose);
  ros::Subscriber sub = nh.subscribe<std_msgs::String>("viewed_map", 1000, getViewedMap);
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("goal", 1); 

  for (int i = 0; i < 40; ++i) {
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok()) {

    ros::Time start = ros::Time::now();

    findRandom(&pub);
    while(ros::Time::now() - start < ros::Duration(secondsToWait) && ros::ok()){

    }
    ros::spinOnce();
    rate.sleep();
  }
}
