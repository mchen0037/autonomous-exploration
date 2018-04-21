#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::Pose2D currentPose;
logical_camera_plugin::logicalImage recent;
geometry_msgs::Pose2D TreasureRobotPose;
bool foundObject = false;

// void getTransformation(tf2_ros::Buffer *tfBuffer, std::string obj1, std::string obj2) {
//   geometry_msgs::TransformStamped transform;
//   try {
//     transform = tfBuffer->lookupTransform(obj1, obj2, ros::Time(0));
//   }
//   catch (tf2::TransformException &ex) {
//     ROS_WARN("%s", ex.what());
//   }
//   double yaw = tf::getYaw(transform.transform.rotation);
//   currentPose.x = transform.transform.translation.x;
//   currentPose.y = transform.transform.translation.y;
//   currentPose.theta = yaw;
// }

void amclMessageReceived(const geometry_msgs::PoseWithCovarianceStamped msg){
  float amclAngle = tf::getYaw(msg.pose.pose.orientation);
  currentPose.x = msg.pose.pose.position.x;
  currentPose.y = msg.pose.pose.position.y;

  geometry_msgs::Quaternion q;
  q.x = msg.pose.pose.orientation.x;
  q.y = msg.pose.pose.orientation.y;
  q.z = msg.pose.pose.orientation.z;
  q.w = msg.pose.pose.orientation.w;

  currentPose.theta = tf::getYaw(q);
    //ROS_INFO_STREAM(currentPose);
    //ROS_INFO_STREAM("amcl angle " << amclAngle);
}

void sawTreasure(const logical_camera_plugin::logicalImage msg) {
  recent = msg;
  foundObject = true;
  geometry_msgs::Quaternion q;
  TreasureRobotPose.x = msg.pose_pos_x;
  TreasureRobotPose.y = msg.pose_pos_y;
  q.x = msg.pose_rot_x;
  q.y = msg.pose_rot_y;
  q.z = msg.pose_rot_z;
  q.w = msg.pose_rot_w;
  TreasureRobotPose.theta = tf::getYaw(q);
  //ROS_INFO_STREAM(TreasureRobotPose);
}

void treasureLocation(){
  geometry_msgs::Pose2D TreasurePose;
  TreasurePose.theta = currentPose.theta - TreasureRobotPose.theta;
  TreasurePose.x = TreasureRobotPose.x * cos(TreasureRobotPose.theta) - TreasureRobotPose.y * cos(TreasureRobotPose.theta) + currentPose.x;
  TreasurePose.y = TreasureRobotPose.x * sin(TreasureRobotPose.theta) + TreasureRobotPose.y * cos(TreasureRobotPose.theta) + currentPose.x;

  ROS_INFO_STREAM(TreasurePose);

}
int main(int argc, char** argv) {
  ros::init(argc, argv, "detectObjects");
  ros::NodeHandle nh;

  ros::Rate rate(20);

  ros::Subscriber sub = nh.subscribe<logical_camera_plugin::logicalImage>("/objectsDetected", 1000, sawTreasure);
  ros::Subscriber subAMCL = nh.subscribe("/amcl_pose", 1000, &amclMessageReceived);

  tf2_ros::Buffer tfBuffer; //buffer to hold several transforms
  tf2_ros::TransformListener tfListener(tfBuffer); //listener

  while (ros::ok()) {
    if (foundObject) {
      treasureLocation();
      // getTransformation(&tfBuffer);
      // ROS_INFO_STREAM(currentPose);



    }
    ros::spinOnce();
    rate.sleep();
  }

  ros::spin();
}
