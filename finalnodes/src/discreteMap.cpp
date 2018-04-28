#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <map>
//#include <tf/Matrix3x3.h>


using namespace std;

void discretize(nav_msgs::OccupancyGrid msg){
  vector<vector<int8> > cropped;

  for(int i = 199; i < 600 ; i++){
    for(int j = 199; j< 600; j++){
      cropped[i-199][j-199] = data[i][j];
    }
  }



}


int main(int argc, char** argv) {
  ros::init(argc, argv, "discreteMap");
  ros::NodeHandle nh;

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<logical_camera_plugin::logicalImage>("/map", 1000, discretize);


  while (ros::ok()) {
    ros::spinOnce();
  }

}
