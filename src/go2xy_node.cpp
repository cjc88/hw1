#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "turtlesim/Pose.h"

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
ros::Subscriber goal_subscriber;
turtlesim::Pose turtlesim_pose;

double getDistance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){ // get current x, y, orientation
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta; // current absolute orientation of the turtle
}

void move2goal(turtlesim::Pose goal_pose, double distance_tolerance){

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

	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

void input_xy_Callback(const geometry_msgs::Point::ConstPtr & p){ // get current x, y, orientation
	turtlesim::Pose goal_pose;

        goal_pose.x = p->x;
	goal_pose.y = p->y;
	goal_pose.theta=0;
	//ROS_INFO_STREAM(goal_pose.x << " " << goal_pose.y);
	move2goal(goal_pose, 0.01);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "go2xy_node");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	//bool isForward, clockwise;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
        pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	goal_subscriber = n.subscribe("/goal", 10, input_xy_Callback);
	ros::spinOnce();
/*
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

	ros::spin();
	return 0;
}
