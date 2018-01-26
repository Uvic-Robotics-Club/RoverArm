#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <serial/serial.h>

#define BASE 0
#define LOWER_ELBOW 1
#define UPPER_ELBOW 2
#define GRIPPER 3



std::string usbPort;
serial::Serial ser;
ros::Publisher armPub;
bool output_to_serial = true;

uint8_t velocities[] = {0,0,0,0};

double mapValue(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sendToArm(){
  geometry_msgs::QuaternionStamped diagnostics_msg;
  if(output_to_serial){
    ser.write(&velocities[0],4);
  }

  diagnostics_msg.header.stamp = ros::Time::now();
  diagnostics_msg.header.frame_id = "Arm Diagnostics";
  diagnostics_msg.quaternion.x = velocities[0];
  diagnostics_msg.quaternion.y = velocities[1];
  diagnostics_msg.quaternion.z = velocities[2];
  diagnostics_msg.quaternion.w = velocities[3];
  armPub.publish(diagnostics_msg);
}


void setVelocity(const geometry_msgs::Twist::ConstPtr& newVelocities){
  static int rotate = 0;
  static int lower = 0;
  static int upper = 0;

  rotate = round(newVelocities->angular.x*255.0*newVelocities->linear.x);
  lower = round(newVelocities->angular.y*255.0*newVelocities->linear.y);
  upper = round(newVelocities->angular.z*255.0*newVelocities->linear.z);

  velocities[0] = 0;
  if(rotate>0){
    velocities[1] = 0;
    velocities[2] = (uint8_t)abs(rotate);
  }
  else{
    velocities[1] = (uint8_t)abs(rotate);
    velocities[2] = 0;
  }
  sendToArm();

  velocities[0] = 1;
  if(round(lower)>0){
    velocities[1] = 0;
    velocities[2] = (uint8_t)abs(lower);
  }
  else{
    velocities[1] = (uint8_t)abs(lower);
    velocities[2] = 0;
  }
  sendToArm();

  velocities[0] = 2;
  if(round(upper)>0){
    velocities[1] = 0;
    velocities[2] = (uint8_t)abs(upper);
  }
  else{
    velocities[1] = (uint8_t)abs(upper);
    velocities[2] = 0;
  }
  sendToArm();

  //ROS_INFO("%f,%f,%f,%f", newVelocities->angular.x,newVelocities->angular.y,newVelocities->angular.z,newVelocities->angular.w);

}


int main(int argc, char **argv){
	//Initialize ROS node
	ros::init(argc,argv,"DirectMotorControl");
	ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber joySub = nh.subscribe("Arm/AngleVelocities",1,setVelocity);
  armPub = nh.advertise<geometry_msgs::QuaternionStamped>("Arm/Diagnostics",20);
  //nh.param<std::string>("~USBPORT", usbPort, "/dev/ttyUSB0");

  if (pnh.hasParam("output_type")){
    output_to_serial=false;
  }
  else{
    try{
      serial::Serial ser("/dev/ttyUSB0",9600);
  		ser.open();
  	}catch(serial::IOException& e){
  		ROS_INFO("unable to open port");
  		//throw std::invalid_argument( "Unable to open the usb port (likely its the wrong usb name or its busy)" );
  	}
  	if(ser.isOpen()){
  		ROS_INFO("Serial Port Initialized");
  	}
  	else{
  		//throw std::invalid_argument( "Unable to open the Serial Communication port to the UKART" );
  	}
  }
  ros::Rate loop_rate(100);
  while(ros::ok()){
    //sendToArm();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
