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

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){ // get current x, y, orientation
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta; // current absolute orientation of the turtle
}

void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg; // $rosmsg show msg_name
	
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
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce(); // to effectively publish vel_msg
 		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	} while(current_distance < distance);
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg); // stop the turtle
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

// set absolute target angle
void setTargetAngle (double target_angle_radians) {
	double relative_angle_radians = target_angle_radians - turtlesim_pose.theta;
	cout << target_angle_radians << ", " << turtlesim_pose.theta << ", " << relative_angle_radians << ", " << endl;
	rotate (deg2radians(90), relative_angle_radians);
}

double getDistance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void move2goal(turtlesim::Pose  goal_pose, double distance_tolerance){

	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);
	double E = 0.0;
	do{
		/* vel_msg.linear.x =1; */  // constant velocity
		/* linear velocity in the x-axis for proportional control */
		double Kp=1.0;
		double Ki=0.02;
		//double v0 = 2.0;
		//double alpha = 0.5;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		double E = E+e;
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);
		vel_msg.linear.x = (Kp*e);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =4*(atan2(goal_pose.y-turtlesim_pose.y, goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();
		// cout << turtlesim_pose.x << " " << turtlesim_pose.y << " " << turtlesim_pose.theta << endl;

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)>distance_tolerance);
	cout<<"end move goal"<<endl;
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

void gridClean(){

	ros::Rate loop(1);
	turtlesim::Pose pose;
	pose.x=1;
	pose.y=1;
	pose.theta=0;
	move2goal(pose, 0.01);
	loop.sleep();
	setTargetAngle(0);
	loop.sleep();
/*
	move(2.0, 9.0, true);
	loop.sleep();
	rotate(deg2radians(10), deg2radians(90));
	loop.sleep();
	move(2.0, 9.0, true);


	rotate(deg2radians(10), deg2radians(90));
	loop.sleep();
	move(2.0, 1.0, true);
	rotate(deg2radians(10), deg2radians(90));
	loop.sleep();
	move(2.0, 9.0, true);

	rotate(deg2radians(30), deg2radians(-90));
	loop.sleep();
	move(2.0, 1.0, true);
	rotate(deg2radians(30), deg2radians(-90));
	loop.sleep();
	move(2.0, 9.0, true);
*/

        // retangular spiral clean
    	
	double d = 9.0;
	move(2.0, d, true);
	loop.sleep();
	rotate(deg2radians(90), deg2radians(90));
	geometry_msgs::Twist vel_msg;
    	do {
		move(5.0, d, true);
		loop.sleep();
		rotate(deg2radians(90), deg2radians(90));
		loop.sleep();
		move(5.0, d, true);
		loop.sleep();
		rotate(deg2radians(90), deg2radians(90));
		loop.sleep();
        	d -= 1.0;
		cout << turtlesim_pose.x << " " << turtlesim_pose.y << " " << turtlesim_pose.theta << endl;
	//}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, 5.5, 5.5)> 0.7);
	}while(d>=1);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);
}

void spiralClean(){
	geometry_msgs::Twist vel_msg;
	double count =0;

	double constant_speed=4;
	double vk = 1;
	double wk = 2;
	double rk = 0.5;
	ros::Rate loop(1);

	do{
		rk=rk+1.0;
		vel_msg.linear.x =rk;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//set a random angular velocity in the y-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =constant_speed;//((vk)/(0.5+rk));

		cout<<"vel_msg.linear.x = "<<vel_msg.linear.x<<endl;
		cout<<"vel_msg.angular.z = "<<vel_msg.angular.z<<endl;
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();

		loop.sleep();
		//vk = vel_msg.linear.x;
		//wk = vel_msg.angular.z;
		//rk = vk/wk;
		cout<<rk<<", "<<vk <<", "<<wk<<endl;
	}while((turtlesim_pose.x<10.5)&&(turtlesim_pose.y<10.5));
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle2spiral");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	//bool isForward, clockwise;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	ros::spinOnce();
	ros::Rate loop_rate(0.2);

/*
	rotate(deg2radians(30), deg2radians(-60)); 
	loop_rate.sleep();

	setTargetAngle(deg2radians(90));

	loop_rate.sleep();
	setTargetAngle(deg2radians(-30));
	loop_rate.sleep();
	setTargetAngle(deg2radians(0));

	turtlesim::Pose pose; // to set a goal
	pose.x = 2;
	pose.y = 2;
	pose.theta = 0; // we do not use.
	move2goal(pose, 0.01);

	pose.x = 5.5;
	pose.y = 5.5;
	pose.theta = 0; // we do not use
	move2goal(pose, 0.01);
*/
	//gridClean();
	spiralClean();
	ros::spin();
	return 0;
}
