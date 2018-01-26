#include "ros/ros.h" // needed for everything
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

// array locations of AXES
int side_to_side = 0;
int forward_back = 1;
int clockwise_counterclockwise  = 2;
int slider = 3;
int lr_hat = 4;
int ud_hat = 5;

// array locations of button
int trigger = 0;
int thumb_rest = 1;
int button_3  = 2;
int button_4 = 3;
int button_5 = 4;
int button_6 = 5;
int button_7 = 6;
int button_8 = 7;
int button_9 = 8;
int button_10 = 9;
int button_11 = 10;
int button_12 = 11;

bool rotate_enable = true;
bool lower_enable = true;
bool upper_enable = true;

ros::Publisher anglePub;

double mapValue(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Velocity commands callback function
void joy_to_arm(const sensor_msgs::Joy::ConstPtr& joy){
  geometry_msgs::Twist newMessage;
  if(joy->buttons[button_7]==1){
    rotate_enable = !rotate_enable;
  }
  if(joy->buttons[button_9]==1){
    lower_enable = !lower_enable;
  }
  if(joy->buttons[button_11]==1){
    upper_enable = !upper_enable;
  }
  newMessage.linear.x = 1.0*rotate_enable;
  newMessage.linear.y = 1.0*lower_enable;
  newMessage.linear.z = 1.0*upper_enable;
  newMessage.angular.x = joy->axes[clockwise_counterclockwise]*joy->axes[slider];
  newMessage.angular.y = joy->axes[forward_back]*joy->axes[slider];
  newMessage.angular.z = joy->axes[ud_hat]*joy->axes[slider];
  anglePub.publish(newMessage);

}


int main(int argc, char **argv){
	//Initialize ROS node
	ros::init(argc,argv,"Joy_Teleop");
	ros::NodeHandle nh;
  anglePub = nh.advertise<geometry_msgs::Twist>("Arm/AngleVelocities",10);
  ros::Subscriber joySub = nh.subscribe("joy",10,joy_to_arm);
  ros::spin();
  return 0;
}
