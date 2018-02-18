#include "ros/ros.h" // needed for everything
#include <geometry_msgs/Point.h>

int button_3  = 2;
int button_4 = 3;
int button_5 = 4;
int button_6 = 5;

ros::Publisher pointPub;
int index = 0;
int n = 3;
double points [n][3]; // n points, [x,y,z]

void joy_to_point(const sensor_msgs::Joy::ConstPtr& joy){
    geometry_msgs::Point newPoint;
    if(joy->buttons[button_3]==1){
        index = (index + 1);
        index = index>n ? index-n : index;
        index = index%n;
    }
    if(joy->buttons[button_4]==1){
        index = (index -1);
        index = index<0 ? index + n : index;
        index = index % n;
    }
    if(joy->buttons[button_3]==1 or joy->buttons[button_4]==1){
        newPoint.x = points[index][0];
        newPoint.y = points[index][1];
        newPoint.z = points[index][2];
        pointPub.publish(newPoint);
    }
}

int main(int argc, char **argv){
    // x y z, keeping y zero because I cant rotate the base maybe_data
    // All distances in meters
    points[0][0] = 0.5;
    points[0][1] = 0.0;
    points[0][2] = 0.5;

    points[1][0] = 1.0;
    points[1][1] = 0.0;
    points[1][2] = 1.0;

    points[2][0] = 0.0;
    points[2][1] = 0.0;
    points[2][2] = 0.0;

	//Initialize ROS node
	ros::init(argc,argv,"Point_Generator");
	ros::NodeHandle nh;
    pointPub = nh.advertise<geometry_msgs::Point>("Arm/Goal",1);
    ros::Subscriber joySub = nh.subscribe("arm_joy",1,joy_to_point);
    ros::spin();
    return 0;
}