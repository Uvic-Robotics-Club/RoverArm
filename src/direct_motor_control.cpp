#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <RoverArm/arm_velocity.h>
#include <RoverArm/joint_angles.h>
#include <RoverArm/diagnostic.h>
#include <serial/serial.h>
#include <string>
#include <iostream>

#include <time.h>

#define BASE 0
#define LOWER_ELBOW 1
#define UPPER_ELBOW 2
#define GRIPPER 3
 


std::string usbPort;
serial::Serial ser;
ros::Publisher armPub;

ros::Publisher arm_feedback_Pub;
bool output_to_serial = true;
// data to send
// mode 0, value 0, mode 1, value 1, mode 2, value 2, mode 3, value 3
int data_to_send[] = {0,0,0,0,0,0,0,0,0,0};
int rotate = 0;
int lower = 0;
int upper = 0;
int gripper = 0;

clock_t t;
double mapValue(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void feedbackParser(){
	static std::string data_returned = "";
	data_returned = "";
	ser.readline(data_returned);
	data_returned.erase(std::remove(data_returned.begin(), data_returned.end(), '\n'), data_returned.end());
	ROS_ERROR_STREAM("Arduino->" << data_returned << "|");
	//std::vector<double> vect;
	//std::stringstream ss(data_returned);
	//double i;
	//while (ss >> i){
		//vect.push_back(i);
		//if (ss.peek() == ',' or ss.peek()=='\n')
			//ss.ignore();
	//}
	
	//std::string maybe_data = "";
	//char buffer [50];
	//for(int i=0; i < vect.size(); i++){
		//sprintf(buffer,"%f",vect[i]);
		//maybe_data = maybe_data + buffer + " | ";
	//}
	////ROS_ERROR_STREAM("PROGRAM -> " << maybe_data);
	//RoverArm::joint_angles feedbackMessage;
	//feedbackMessage.lower_angle = vect[0];
	//feedbackMessage.upper_angle = vect[1];
	//feedbackMessage.base_angle = vect[2];
	//arm_feedback_Pub.publish(feedbackMessage);
}

void sendToArm(){
	
    RoverArm::diagnostic diagnostics_msg;
    
    while(ser.available()>=1){
		feedbackParser();
		
	}

    if(output_to_serial){
        ser.flush();
        char buffer[60];
        //sprintf(buffer, "M:%i V:%i M:%i V:%i M:%i V:%i M:%i V:%i M:%i V:%i ", data_to_send[0], data_to_send[1], data_to_send[2], data_to_send[3], data_to_send[4], data_to_send[5], data_to_send[6], data_to_send[7]);
        sprintf(buffer, "%i %i ", data_to_send[0],data_to_send[1]);
        ser.write(buffer);
        ser.flush();
    }
    else{
		ROS_INFO_STREAM("Did not try and output the data when OUTPUT TO SERIAL IS " << output_to_serial);
	}
   

  diagnostics_msg.base_mode = data_to_send[0];
  diagnostics_msg.base_angle = data_to_send[1];
  diagnostics_msg.lower_mode = data_to_send[2];
  diagnostics_msg.lower_angle = data_to_send[3];
  diagnostics_msg.upper_mode = data_to_send[4];
  diagnostics_msg.upper_angle = data_to_send[5];
  

  armPub.publish(diagnostics_msg);
}


void setVelocity(const RoverArm::arm_velocity::ConstPtr& newdata_to_send){
  rotate = (int)round(newdata_to_send->joint.rotate*255.0*newdata_to_send->enable.rotate);
  lower =  (int)round(newdata_to_send->joint.lower*255.0*newdata_to_send->enable.lower);
  upper =  (int)round(newdata_to_send->joint.upper*255.0*newdata_to_send->enable.upper);
  gripper =  (int)round(newdata_to_send->joint.gripper*255.0);
 
  //data_to_send[0] = 0; // manual rotate
  //data_to_send[1] = rotate;
  //data_to_send[2] = 1; // manual lower
  //data_to_send[3] = lower;
  //data_to_send[4] = 2; // manual upper
  //data_to_send[5] = upper;
  //data_to_send[6] = 3; // manual gripper
  //data_to_send[7] = gripper;
  data_to_send[0] = (int)round(newdata_to_send->joint.lower*100/newdata_to_send->joint.speed);
  data_to_send[1] = (int)round(newdata_to_send->joint.speed*100);

}

void setPosition(const RoverArm::joint_angles::ConstPtr& newdata_to_send){
  // maping the -2pi -> 2pi that the angle is in to the max and min of a uint16
  rotate = (int)round(newdata_to_send->base_angle);
  lower = (int)round(newdata_to_send->lower_angle);
  upper = (int)round(newdata_to_send->upper_angle);
  data_to_send[2] = 5;
  data_to_send[4] = 6;
  data_to_send[0] = 0;
  data_to_send[1] = 0;
  data_to_send[3] = lower;
  data_to_send[5] = upper;
}


int main(int argc, char **argv){
  t = clock();
  //Initialize ROS node
  ros::init(argc,argv,"DirectMotorControl");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber joySub = nh.subscribe("Arm/AngleVelocities",1,setVelocity);
  ros::Subscriber progSub = nh.subscribe("Arm/Position",1,setPosition);
  armPub = nh.advertise<RoverArm::diagnostic>("Arm/Diagnostics",1);
  arm_feedback_Pub = nh.advertise<RoverArm::joint_angles>("Arm/Feedback",1);
  ROS_INFO("STARTING UP THE DMC");
  //nh.param<std::string>("USBPORT", usbPort, "/dev/ttyACM0");


  //Slight re-write for opening serial port; This method gave me the most stability on my machine.
  if (false and !pnh.hasParam("output_type")){
    output_to_serial=false;
  }
  else{
      try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO("Serial Port initialized");
    }else{
        return -1;
    }
  }

  ROS_INFO("CONNECTED TO THE ARDUINO");
  double frequency = 15;
  double sleep_duration = 1.0/frequency;
  ros::Rate loop_rate(5);
  while(ros::ok()){
	ros::spinOnce();
	if(((float)(clock()-t))/CLOCKS_PER_SEC > sleep_duration){
		sendToArm();
		t = clock();
	}
  
    //loop_rate.sleep();
  }
}
