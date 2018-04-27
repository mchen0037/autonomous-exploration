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

using namespace std;
typedef pair<float, float> coordinates;

geometry_msgs::Pose2D TreasureRobotPose;

struct treasureChest{

  map <string, coordinates> chest;

  treasureChest(){}

  addTreasure(string id, float x, float y){
    if(chest.count(id)){
      ROS_INFO_STREAM("We already got that treasure my dude");
    }
    else{

      chest.insert(id, make_pair (a,b));

    }
  }

};

treasureChest tc;

void sawTreasure(const logical_camera_plugin::logicalImage msg) {

  geometry_msgs::Quaternion q;
  q.x = msg.pose_rot_x;
  q.y = msg.pose_rot_y;
  q.z = msg.pose_rot_z;
  q.w = msg.pose_rot_w;

  TreasureRobotPose.theta = angles::normalize_angle_positive(tf::getYaw(q));

  ROS_INFO_STREAM(TreasureRobotPose.theta);

}

int main(int argc, char** argv) {

  ros::Subscriber sub = nh.subscribe<logical_camera_plugin::logicalImage>("/objectsDetected", 1000, sawTreasure);





}
