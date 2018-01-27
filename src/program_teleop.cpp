#include "ros/ros.h" // needed for everything
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <RoverArm/arm_velocity.h>
#include <RoverArm/point_to_angle.h>

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

ros::ServiceClient client;
void prog_to_arm(const sensor_msgs::Joy::ConstPtr& joy){
  RoverArm::point_to_angle srv;
  srv.request.destination.x = joy->axes[clockwise_counterclockwise];
  srv.request.destination.y = joy->axes[forward_back];
  srv.request.destination.z = joy->axes[side_to_side];
  client.call(srv);
  ROS_INFO_STREAM("BASE ANGLE IS " << srv.response.angles.base_angle << " AND THE INPUT WAS "<< joy->axes[clockwise_counterclockwise]);
}

int main(int argc, char **argv){
	//Initialize ROS node
	ros::init(argc,argv,"Program_Teleop");
	ros::NodeHandle nh;
  ros::Subscriber joySub = nh.subscribe("joy",10,prog_to_arm);
  client = nh.serviceClient<RoverArm::point_to_angle>("test_service");
  ros::spin();
  return 0;
}
