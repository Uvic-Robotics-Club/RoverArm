#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>

#define BASE 0
#define LOWER_ELBOW 1
#define UPPER_ELBOW 2
#define GRIPPER 3



std::string usbPort;

uint8_t velocities[] = {0,0,0,0};


void setVelocity(const geometry_msgs::Twist::ConstPtr& newVelocities){
  velocities[BASE] = (uint8_t) newVelocities->angular.x;
  velocities[LOWER_ELBOW] = (uint8_t) newVelocities->angular.y;
  velocities[UPPER_ELBOW] = (uint8_t) newVelocities->angular.z;
}


int main(int argc, char **argv){
	//Initialize ROS node
	ros::init(argc,argv,"DirectMotorControl");
	ros::NodeHandle nh;

  nh.param<std::string>("~USBPORT", usbPort, "/dev/ttyUSB0");
  ros::Subscriber joySub = nh.subscribe("Arm/AngleVelocities",1,setVelocity);
  serial::Serial ser("/dev/ttyUSB0",9600);
  try{
		ser.open();
	}catch(serial::IOException& e){
		ROS_INFO("unable to open port");
		throw std::invalid_argument( "Unable to open the usb port (likely its the wrong usb name or its busy)" );
	}
	if(ser.isOpen()){
		ROS_INFO("Serial Port Initialized");
	}
	else{
		throw std::invalid_argument( "Unable to open the Serial Communication port to the UKART" );
	}
  ros::Rate loop_rate(10);
  while(ros::ok()){
    ser.write(&velocities[0],4);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
