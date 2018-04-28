#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>


//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*This is the target destination we are trying to get to also we are assuming that the user won't try to get to a point that has a collision*/
float t_x;
float t_y;
float t_theta;

/*This is current pose*/
float curr_x;
float curr_y;
float curr_theta;

bool newValue = false;
float PI =  3.14159265358979323846;
int hertz = 20;

void currPose(const geometry_msgs::Pose msg){
    curr_x = msg.position.x;
    curr_y = msg.position.y;
    curr_theta = tf2::getYaw(msg.orientation);

}


void destPose(const geometry_msgs::Pose2D &thisPose){

	t_x = thisPose.x;
	t_y = thisPose.y;
	t_theta = thisPose.theta;

	ROS_INFO_STREAM("Target x "<< t_x << " Target y "<< t_y << " target Theta " << t_theta);

	newValue = true;

}

/*The reason we use pi and -pi is because if we want to find the distance we need to traverse its just destination_angle - curr_angle, converting to 0-2*pi range made my life a living nightmare*/
// float rosAngleToNorm(float num){
// 	return (num >= 0) ? num : PI + (PI + num);
// }

float absolute(float num){
	return (num >= 0) ? num : num * -1;
}

/*gives me the angle I need to orient myself to move to (NOT RELATIVE TO MYSELF, JUST THE ANGLE I NEED!)*/
float targetAngle(float curr_x, float curr_y){

	float dist_x = t_x - (curr_x);
	float dist_y = t_y - (curr_y);

	float destAngle = atan(absolute(dist_y/dist_x));

	if(dist_x < 0 && dist_y < 0){
		destAngle += PI;
	}			
	else if(dist_x > 0 && dist_y < 0){
		destAngle = 2*PI - destAngle;
	}
	else if(dist_x < 0 && dist_y > 0){
		destAngle = PI - destAngle;
	}

	//convert to -pi - pi range

	return (destAngle <= PI) ? destAngle : destAngle - 2 * PI;
} 

float dist(float curr_x, float curr_y){
	return sqrt(pow(t_x - curr_x, 2) + pow(t_y - curr_y, 2));

}

/*We need this because ros sometimes overshoots and it it causes a world of hurt*/
float calibrate(float dest_angle, float curr_angle){

	float calibration = dest_angle -curr_angle;

	if(calibration > PI){
		calibration - 2 * PI;
	}

	if(calibration < -PI){
		calibration + 2 * PI;
	}

	return calibration;
}


/*Takes in pose and sends it to topic*/
int main(int argc, char** argv){ 

	ros::init(argc, argv, "gotopose");
	ros::NodeHandle nh;
	ros::Rate rate(hertz);

	ros::Subscriber subPose = nh.subscribe("targetpose", 1000, &destPose); //so from what I understand, this is always listening for a input and will call the function if it finds one
	ros::Subscriber subCurr = nh.subscribe("perfect_localization", 1000, &currPose);
	
	//ros::Subscriber subTF = nh.subscribe("/tf2_ros", 1000, &tf2MessageReceived);

	ros::Publisher pubTwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
	ros::Publisher pubRestart = nh.advertise<std_msgs::Empty>("restartTopic", 1000);


	for (int i = 0; i < 80; ++i) {
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::Twist toRotate; 
	toRotate.angular.x = 0;
	toRotate.angular.y = 0;
	toRotate.angular.z = PI/4; //45 degrees per second

	geometry_msgs::Twist toMove;
	toMove.linear.x = 2;
	toMove.linear.y = 0;
	toMove.linear.z = 0;

	geometry_msgs::Twist stop;


	
	while(ros::ok()){//this is a constant infinite loop

		ros::spinOnce();
		if(newValue){//if user inputted a new value, move to that place
			/*get robot's current position*/
			// try{ 
			// 	transformStamped = buffer.lookupTransform("odom", "base_link",ros::Time(0)); //this will get us our current position
			// }

			// catch (tf2::TransformException &ex) {
			// 	ROS_WARN("%s",ex.what());
			// 	ros::Duration(1.0).sleep();
			// 	continue;
			// }

			//float yaw = tf::getYaw(transformStamped.transform.rotation);

			//ROS_INFO_STREAM("Obtained transformation " << transformStamped);

			/*find how far I am from destination and the angle*/
			// float dist_x = t_x - (transformStamped.transform.translation.x);
			// float dist_y = t_y - (transformStamped.transform.translation.y);


			float distance = dist(curr_x, curr_y);

			//ROS_INFO_STREAM("x = " << dist_x << "y = " << dist_y);

			/*I don't like how ros handles angles, I'am going to convert it from 0 to 2pi*/
			
			float currAngle = curr_theta;

			//float meme = abs(dist_y/dist_x);
			//ROS_INFO_STREAM("before tan = " << meme );

			float destAngle = targetAngle(curr_x,curr_y);

			ROS_INFO_STREAM("Angle I need = "<<destAngle <<" current Angle "<< currAngle);

			float targetRotation = destAngle - currAngle; //may need to change if angle between rotation is really tiny
			/*So now we have the distance we need to traverse and how much we need to rotate*/

			ROS_INFO_STREAM("Distance to target "<< distance << " | Rotation I need to take "<< targetRotation);


			//this is where my calibration and first rotation happens
			float calibration = destAngle - curr_theta;
			while(!(calibration < 0.2 && calibration > -0.2)){ 
				//transformStamped = buffer.lookupTransform("odom", "base_link",ros::Time(0));

				toRotate.angular.z = calibration * 2; 
				pubTwist.publish(toRotate);
				ros::spinOnce();
				rate.sleep();

				calibration = destAngle - curr_theta;


			//	ROS_INFO_STREAM ("dest "<<destAngle <<" curr Angle "<<tf::getYaw(transformStamped.transform.rotation) << " calibration "  << calibration);

			}

			pubTwist.publish(stop);
			rate.sleep();

			//The actual moving which controls both linear and angular, deaccalarates togiven bound
			while(distance > .03){ //Degree of accuracy
				//transformStamped = buffer.lookupTransform("odom", "base_link",ros::Time(0));

				toMove.angular.z = calibration * 2; 
				toMove.linear.x = distance * 2;
				pubTwist.publish(toMove);
				ros::spinOnce();
				rate.sleep();

				distance = dist(curr_x, curr_y);
				calibration = (targetAngle(curr_x,curr_y) - curr_theta); //our calibration is based on differenct angles because we're moving now

				ROS_INFO_STREAM("callobration = "<< calibration << " | distance = " << distance) ;
		

			}

			pubTwist.publish(stop);
			rate.sleep();

			// //The last rotation
			// calibration = t_theta - tf::getYaw(transformStamped.transform.rotation);
			// while(!(calibration < 0.075 && calibration > -0.075)){ 
			// 	transformStamped = buffer.lookupTransform("odom", "base_link",ros::Time(0));

			// 	toRotate.angular.z = calibration * 2; 
			// 	pubTwist.publish(toRotate);
			// 	ros::spinOnce();
			// 	rate.sleep();

			// 	calibration = t_theta - tf::getYaw(transformStamped.transform.rotation);


			// 	ROS_INFO_STREAM ("dest "<<destAngle <<" curr Angle "<<tf::getYaw(transformStamped.transform.rotation) << " calibration "  << calibration);

			// }			

			// // transformStamped = buffer.lookupTransform("odom", "base_link",ros::Time(0)); //this will get us our current position
			

			// ROS_INFO_STREAM("My destination" << transformStamped);


			ROS_INFO_STREAM("Finished.");

			//tells getpose that this part is done
			std_msgs::Empty emMsg;
			for(int i = 0; i < hertz+2; i++){
				pubRestart.publish(emMsg);
				rate.sleep();
			}


			newValue = false;
		}

	}

}