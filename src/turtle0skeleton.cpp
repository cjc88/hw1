#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher velocity_pub;

void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg; // $ rosmsg show msg_name
	vel_msg.linear.x = (isForward)? abs(speed) : -abs(speed);

	velocity_pub.publish(vel_msg); // move the turtle
	
	vel_msg.linear.x = 0;
	velocity_pub.publish(vel_msg); // stop the turtle
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle0");
	ros::NodeHandle n;
	// ...
	velocity_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000); 

	move(speed, distance, isForward);
	ros::spin();
}

