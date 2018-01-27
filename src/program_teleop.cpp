#include "ros/ros.h" // needed for everything
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <RoverArm/arm_velocity.h>
#include <RoverArm/point_to_angle.h>
#include <RoverArm/joint_angles.h>

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
ros::Publisher posPub;

void prog_to_arm(const geometry_msgs::Point::ConstPtr& pt){
  RoverArm::point_to_angle srv;
  srv.request.destination.x = pt->x;
  srv.request.destination.y = pt->y;
  srv.request.destination.z = pt->z;
  client.call(srv);

  RoverArm::joint_angles newMessage;
  newMessage.base_angle = srv.response.angles.base_angle;
  newMessage.lower_angle = srv.response.angles.lower_angle;
  newMessage.upper_angle = srv.response.angles.upper_angle;
  posPub.publish(newMessage);

}

int main(int argc, char **argv){
	//Initialize ROS node
	ros::init(argc,argv,"Program_Teleop");
	ros::NodeHandle nh;
  ros::Subscriber joySub = nh.subscribe("Arm/Goal",10,prog_to_arm);
  posPub = nh.advertise<RoverArm::joint_angles>("Arm/Position",10);
  client = nh.serviceClient<RoverArm::point_to_angle>("arm_ik");
  ros::spin();
  return 0;
}
