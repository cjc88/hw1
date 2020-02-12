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
	
	...
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
	
	setTargetAngle(deg2radians(90));
	
	ros::spin();
	return 0;
}

