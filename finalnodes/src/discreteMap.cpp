#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <map>
//#include <tf/Matrix3x3.h>


using namespace std;

int matToArr(int row, int col, int row_tar, int col_tar){
  return (row_tar-1) * col + col_tar - 1;

}

void discretize(const nav_msgs::OccupancyGrid msg){
//gets me to the start of every square
  for(int j = 200; j < 600; j+=25){
    for(int i = 200; i < 600;i+=25){

      for(int l = 0; l < 25; l++){
        for(int k = 0; k < 25; k++){
      
          cout << ((int)msg.data[matToArr(800,800,j+l,i+k)]) <<"\t";

        }
        cout << endl;
      }
      ros::shutdown();
        cout << endl;
          cout << endl;
            cout << endl;

    }
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "discreteMap");
  ros::NodeHandle nh;

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, discretize);


  while (ros::ok()) {
    ros::spinOnce();
  }

}

