#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <angles/angles.h>
#include <tf2/utils.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <math.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>

using namespace Eigen;

float t_x;
float t_y;
float t_theta;
float vol = 0;
bool newValue = false;
geometry_msgs::Pose current_pose;

typedef Matrix<float, 3,3> Matrix3f;

Matrix3f covar;
EigenSolver<Matrix3f> es;

float current_min_scan = 999.9;

void serviceActivated() {
    ROS_INFO_STREAM("Service received goal");
}

void serviceDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO_STREAM("Service completed");
    ROS_INFO_STREAM("Final state " << state.toString().c_str());
}

void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb) {
    ROS_INFO_STREAM("Service still running");
    ROS_INFO_STREAM("Current pose (x,y) " <<
		    fb->base_position.pose.position.x << "," <<
		    fb->base_position.pose.position.y);
}

void destPose(const geometry_msgs::Pose msg){
    t_x = msg.position.x;
    t_y = msg.position.y;
    t_theta = 1; //the underlying path finding algorithm defaults to zero, so this works to avoid any wierd angle changes and formatting

	newValue = true;
}

void getCurrentPose(const geometry_msgs::PoseWithCovarianceStamped msg) {
  	current_pose = msg.pose.pose;

  	covar(0,0) = msg.pose.covariance[0];		covar(0,1) = msg.pose.covariance[1];		covar(0,2) = msg.pose.covariance[5];
	covar(1,0) = msg.pose.covariance[6];		covar(1,1) = msg.pose.covariance[7];		covar(1,2) = msg.pose.covariance[11];
	covar(2,0) = msg.pose.covariance[30];		covar(2,1) = msg.pose.covariance[31];		covar(2,2) = msg.pose.covariance[35];

	es.compute(covar, false); //eigen values based on std::complex which has a real and imaginary component

	vol = 4/3 * 3.14159265358979 *sqrt(es.eigenvalues()[0].real() * 9.348) * sqrt(es.eigenvalues()[1].real() * 9.348) * sqrt(es.eigenvalues()[2].real() * 9.348);
	ROS_INFO_STREAM("Ellipsoid Volume = " <<vol);

}

void getScan(const sensor_msgs::LaserScan msg) {
	float min = 999.9;
	for (int i = 0; i < msg.ranges.size(); ++i) {
		if (msg.ranges[i] < min) {
			min = msg.ranges[i];
		}
	}
	current_min_scan = min;
}

int main(int argc, char ** argv){

	ros::init(argc, argv, "gotopose");
	ros::NodeHandle nh;

	ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("/global_localization");
	ros::Subscriber subDestPose = nh.subscribe("goal", 1000, &destPose); 
	ros::Subscriber subCurrPose = nh.subscribe("amcl_pose", 1000, getCurrentPose);
	ros::Subscriber subScan = nh.subscribe("/scan", 1000, getScan);
	ros::Publisher pubTwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>ac("move_base",true);

	move_base_msgs::MoveBaseGoal goal;
	std_srvs::Empty emptymsg;

	ros::Rate rate(20);

	goal.target_pose.header.frame_id = "map";


	while(ros::ok()){

		if(newValue){

			newValue = false;
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = t_x;
			goal.target_pose.pose.position.y = t_y;
			goal.target_pose.pose.orientation.w = t_theta;

			ROS_INFO_STREAM("hi ");

			ac.sendGoal(goal,&serviceDone,&serviceActivated,&serviceFeedback);
			//ac.waitForResult();
			ros::Time start = ros::Time::now();
			while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && 
				ros::Time::now() - start < ros::Duration(20.0)) {
			  	
				ros::spinOnce();
			  	rate.sleep();

			  	if(vol > 9000){
			  		// ac.cancelGoal();
			  		ROS_WARN_STREAM("Covariance is LARGE");
			  		reset.call(emptymsg);
			  	}

			  	if (current_min_scan < 0.1) {
			  		ac.cancelGoal();
			  		ROS_INFO_STREAM("Too close to wall!!! Backing up.");
			  		geometry_msgs::Twist twist;
			  		twist.linear.x = -0.2;
			  		for (int i = 0; i < 40; ++i) {
			  			pubTwist.publish(twist);
			  			rate.sleep();
			  		}
			  		break;
			  	}
			  
			 }

			ROS_INFO_STREAM("bye ");
		}
		ros::spinOnce();
		rate.sleep();
	}
}