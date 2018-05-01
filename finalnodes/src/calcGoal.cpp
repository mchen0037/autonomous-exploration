#include <ros/ros.h>
#include <std_msgs/String.h>

void getViewedMap(const std_msgs::String msg) {
  int size = msg.data[0]/sizeof(msg.data);
  std::cout << size << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calcGoal");
  ros::NodeHandle nh;

  ros::Rate rate(20);
  ros::Subscriber sub = nh.subscribe<std_msgs::String>("viewed_map", 1000, getViewedMap);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
