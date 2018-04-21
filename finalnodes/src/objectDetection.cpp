// #include <ros/ros.h>
// #include <logical_camera_plugin/logicalImage.h>
// #include <tf/transform_datatypes.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometry_msgs/Pose2D.h>
//
// geometry_msgs::Pose2D currentPose;
// logical_camera_plugin::logicalImage recent;
// bool foundObject = false;
//
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
//
// void sawObject(const logical_camera_plugin::logicalImage msg) {
//   recent = msg;
//   foundObject = true;
// }
//
// int main(int argc, char** argv) {
//   ros::init(argc, argv, "detectObjects");
//   ros::NodeHandle nh;
//
//   ros::Rate rate(20);
//
//   ros::Subscriber sub = nh.subscribe<logical_camera_plugin::logicalImage>("/objectsDetected", 1000, sawObject);
//   tf2_ros::Buffer tfBuffer;	//buffer to hold several transforms
// 	tf2_ros::TransformListener tfListener(tfBuffer); //listener
//
//   while (ros::ok()) {
//     if (foundObject) {
//       getTransformation(&tfBuffer);
//       ROS_INFO_STREAM(currentPose);
//     }
//     ros::spinOnce();
//     rate.sleep();
//   }
//
//   ros::spin();
// }
int main(int argc, char** argv) {
  
}
