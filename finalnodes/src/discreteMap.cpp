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
  int square = 10; // square x square
//gets me to the start of every square
  vector <int> graph;
  for(int j = 200; j < 600; j+=square){
    for(int i = 200; i < 600;i+=square){



      int count = 0;
      for(int l = 0; l < square; l++){
        for(int k = 0; k < square; k++){
      
          if((int)msg.data[matToArr(800,800,j+l,i+k)] != 0 && count < 1){
            count++;
          } 
        }
      }


      graph.push_back(count);


    }
  }

  for(int i =0; i < graph.size(); i++){
    cout << graph[i] << "\t";
    
    if ( (i+1)%(400/square) == 0 ){
      cout <<"\n\n";
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

