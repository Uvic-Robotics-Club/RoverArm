#include "ros/ros.h" // needed for everything
#include <sensor_msgs/Joy.h>
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
bool speed_lock = false;
double speed = 0;

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
    if(rotate_enable){ROS_INFO("ROTATE ENABLED");}
    else{ROS_INFO("ROTATE DISABLED");}
  }
  if(joy->buttons[button_9]==1){
    lower_enable = !lower_enable;
    if(lower_enable){ROS_INFO("LOWER ENABLED");}
    else{ROS_INFO("LOWER DISABLED");}
  }
  if(joy->buttons[button_11]==1){
    upper_enable = !upper_enable;
    if(upper_enable){ROS_INFO("UPPER ENABLED");}
    else{ROS_INFO("UPPER DISABLED");}
  }
  if(joy->buttons[button_12]==1){
    speed_lock = !speed_lock;
    if(upper_enable){ROS_INFO_STREAM("SPEED LOCKED TO "<< speed);}
    else{ROS_INFO("SPEED UNLOCKED");}
  }
  if(not speed_lock){
	speed = joy->axes[slider];
  }
  newMessage.enable.rotate = rotate_enable;
  newMessage.enable.lower = lower_enable;
  newMessage.enable.upper = upper_enable;
  newMessage.enable.speed = speed_lock;
  newMessage.joint.rotate = joy->axes[clockwise_counterclockwise]*speed;
  newMessage.joint.lower = joy->axes[forward_back]*speed;
  newMessage.joint.upper = joy->axes[ud_hat]*speed;
  newMessage.joint.gripper = -1*joy->buttons[trigger]+joy->buttons[thumb_rest];
  newMessage.joint.speed = speed;
  anglePub.publish(newMessage);
}


int main(int argc, char **argv){
	//Initialize ROS node
	ros::init(argc,argv,"Joy_Teleop");
	ros::NodeHandle nh;
  anglePub = nh.advertise<RoverArm::arm_velocity>("Arm/AngleVelocities",1);
  ros::Subscriber joySub = nh.subscribe("arm_joy",1,joy_to_arm);
  ros::spin();
  return 0;
}
