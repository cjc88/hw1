#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// add "geometry_msgs into find_packes(...) in CMakeList.txt

using namespace std;

ros::Publisher velocity_pub;

// move the turtle forward or backward
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg; // $ rosmsg show msg_name
	
	vel_msg.linear.x = (isForward)? abs(speed) : -abs(speed);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do {
		velocity_pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce(); // to effectively publish vel_msg
 		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	} while(current_distance < distance);
	vel_msg.linear.x = 0;
	velocity_pub.publish(vel_msg); // stop the turtle
}

const double PI = 3.14159265359;
double deg2radians(double angle_in_degrees){
	return angle_in_degrees * PI /180.0;
}

void rotate (double angular_speed, double angle2turn) {
	geometry_msgs::Twist vel_msg;
	
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = (angle2turn>0)? abs(angular_speed) : -abs(angular_speed);
	
	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(100);
	do {
		velocity_pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = abs(angular_speed * (t1-t0));
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_angle < abs(angle2turn));
	vel_msg.angular.z = 0;
	velocity_pub.publish(vel_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle0");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;

	velocity_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000); // how do we know the topic name? 
                                                                                     // $ rostopic list & info

	while(ros::ok()) {
	    cout << "------------\nEnter speed: ";
	    cin >> speed;
	    cout << "Enter distance: ";
	    cin >> distance;
	    cout << "Forward (1: Forward, 0: Backward)? ";
	    cin >> isForward;
	    move(speed, distance, isForward);

	    cout << "Enter angular velocity (degree/sec): ";
	    cin >> angular_speed;
	    cout << "Enter desired angle to turn (+ or -  degrees): ";
	    cin >> angle;
	    rotate(deg2radians(angular_speed), deg2radians(angle));
        }
	ros::spin();
}
