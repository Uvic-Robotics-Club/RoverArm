#include "ros/ros.h" // needed for everything
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <RoverArm/arm_velocity.h>

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
  RoverArm::arm_velocity newMessage;
  if(joy->buttons[button_7]==1){
    rotate_enable = !rotate_enable;
  }
  if(joy->buttons[button_9]==1){
    lower_enable = !lower_enable;
  }
  if(joy->buttons[button_11]==1){
    upper_enable = !upper_enable;
  }
  newMessage.enable.rotate = rotate_enable;
  newMessage.enable.lower = lower_enable;
  newMessage.enable.upper = upper_enable;
  newMessage.joint.rotate = joy->axes[clockwise_counterclockwise]*joy->axes[slider];
  newMessage.joint.lower = joy->axes[forward_back]*joy->axes[slider];
  newMessage.joint.upper = joy->axes[ud_hat]*joy->axes[slider];
  newMessage.joint.gripper = -1*joy->buttons[trigger]+joy->buttons[thumb_rest];
  anglePub.publish(newMessage);

}


int main(int argc, char **argv){
	//Initialize ROS node
	ros::init(argc,argv,"Joy_Teleop");
	ros::NodeHandle nh;
  anglePub = nh.advertise<RoverArm::arm_velocity>("Arm/AngleVelocities",10);
  ros::Subscriber joySub = nh.subscribe("joy",10,joy_to_arm);
  ros::spin();
  return 0;
}
