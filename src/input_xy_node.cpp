#include "ros/ros.h"
#include "geometry_msgs/Point.h"
using namespace std;

ros::Publisher xy_pub;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "input_xy_node");
	ros::NodeHandle n;
	geometry_msgs::Point p;
	double x, y;
	
	xy_pub = n.advertise<geometry_msgs::Point>("/goal", 1000); 

	while(ros::ok()) {
	    cout << "------------\nEnter x: ";
	    cin >> x;
	    cout << "Enter y: ";
	    cin >> y;
	    p.x = x;
	    p.y = y;
	    p.z = 0.0;
	    xy_pub.publish(p); 
        }
	ros::spin();
}
