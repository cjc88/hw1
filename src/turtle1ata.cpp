#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;
double deg2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

void poseCallback(const turtlesim::Pose::ConstPtr& pose_msg){ //get current x, y, orientation
	turtlesim_pose.x=pose_msg->x;
	turtlesim_pose.y=pose_msg->y;
	turtlesim_pose.theta=pose_msg->theta; // current absolute orientation of the turtle
}

// rotate from current orientation
void rotate (double angular_speed, double angle2turn){ // assume angular_speed is positive
	geometry_msgs::Twist vel_msg;
	
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = (angle2turn > 0)? angular_speed : -angular_speed; 
	
	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(360);
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = abs(angular_speed) * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_angle < abs(angle2turn));
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

void setTargetAngle (double target_angle_radians) { // set absolute target angle (ATA)
	double relative_angle_radians = target_angle_radians - turtlesim_pose.theta;
	rotate (deg2radians(90), relative_angle_radians);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle1ata");
	ros::NodeHandle n;
	
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	ros::Rate loop_rate(0.2);

	loop_rate.sleep();
	setTargetAngle(deg2radians(90));
	loop_rate.sleep();
	setTargetAngle(deg2radians(-60));
	loop_rate.sleep();
	setTargetAngle(deg2radians(0));
	
	ros::spin();
	return 0;
}
