#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <RoverArm/arm_velocity.h>
#include <RoverArm/joint_angles.h>
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

int data_to_send[] = {0,0,0,0,0};
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
	std::vector<double> vect;
	std::stringstream ss(data_returned);
	double i;
	while (ss >> i){
		vect.push_back(i);
		if (ss.peek() == ',' or ss.peek()=='\n')
			ss.ignore();
	}
	
	std::string maybe_data = "";
	char buffer [50];
	for(int i=0; i < vect.size(); i++){
		sprintf(buffer,"%f",vect[i]);
		maybe_data = maybe_data + buffer + " | ";
	}
	//ROS_ERROR_STREAM("PROGRAM -> " << maybe_data);
	RoverArm::joint_angles feedbackMessage;
	feedbackMessage.base_angle = vect[2];
	feedbackMessage.lower_angle = vect[0];
	feedbackMessage.upper_angle = vect[1];
	arm_feedback_Pub.publish(feedbackMessage);
}

void sendToArm(){
	
    geometry_msgs::QuaternionStamped diagnostics_msg;
    
    while(ser.available()>=1){
		feedbackParser();
		
	}

    if(output_to_serial){
        ser.flush();
        char buffer[30];
        sprintf(buffer, "M:%i V:%i\n", 0, data_to_send[1]);
        ser.write(buffer);
        ser.flush();
        sprintf(buffer, "M:%i V:%i\n", 1, data_to_send[2]);
        ser.write(buffer);
        ser.flush();
        sprintf(buffer, "M:%i V:%i\n", 2, data_to_send[3]);
        ser.write(buffer);
        ser.flush();
        sprintf(buffer, "M:%i V:%i\n", 3, data_to_send[4]);
        ser.write(buffer);
        ser.flush();
        //ser.flush();
        //ROS_ERROR_STREAM("Program -> "<<buffer);
        //data_to_send[0]=0;
        //data_to_send[1]=0;
			
    }
    else{
		ROS_INFO_STREAM("Did not try and output the data when OUTPUT TO SERIAL IS " << output_to_serial);
	}
   

  diagnostics_msg.header.stamp = ros::Time::now();
  diagnostics_msg.quaternion.x = data_to_send[0];
  diagnostics_msg.quaternion.y = data_to_send[1];
  diagnostics_msg.quaternion.z = data_to_send[2];
  diagnostics_msg.quaternion.w = data_to_send[3];

  switch (data_to_send[0]) {
    case 0:
      diagnostics_msg.header.frame_id = "Arm Diagnostics: Rotate Velocity [-,+,0]";
      break;
    case 1:
      diagnostics_msg.header.frame_id = "Arm Diagnostics: Lower Velocity [-,+,0]";
      break;
    case 2:
      diagnostics_msg.header.frame_id = "Arm Diagnostics: Upper Velocity [-,+,0]";
      break;
    case 3:
      diagnostics_msg.header.frame_id = "Arm Diagnostics: Gripper Velocity [-,+,0]";
      break;
    case 4:
      diagnostics_msg.header.frame_id = "Arm Diagnostics: Rotate Position [H,L,0]";
      break;
    case 5:
      diagnostics_msg.header.frame_id = "Arm Diagnostics: Lower Position [H,L,0]";
      break;
    case 6:
      diagnostics_msg.header.frame_id = "Arm Diagnostics: Upper Position [H,L,0]";
      break;
  };
  armPub.publish(diagnostics_msg);
}


void setVelocity(const RoverArm::arm_velocity::ConstPtr& newdata_to_send){
  rotate = (int)round(newdata_to_send->joint.rotate*255.0*newdata_to_send->enable.rotate);
  lower =  (int)round(newdata_to_send->joint.lower*255.0*newdata_to_send->enable.lower);
  upper =  (int)round(newdata_to_send->joint.upper*255.0*newdata_to_send->enable.upper);
  gripper =  (int)round(newdata_to_send->joint.gripper*255.0);
 
  
  char buffer [50];
  sprintf(buffer,"R: %i | LOWER: %i | UPPER: %i", rotate, lower, upper);
  ROS_ERROR_STREAM(buffer);
}

void setPosition(const RoverArm::joint_angles::ConstPtr& newdata_to_send){
  static int rotate = 0;
  static int lower = 0;
  static int upper = 0;
  // maping the -2pi -> 2pi that the angle is in to the max and min of a uint16
  rotate = round(mapValue(newdata_to_send->base_angle,-6.28318,6.28318,0,65536));
  lower = round(mapValue(newdata_to_send->lower_angle,-6.28318,6.28318,0,65536));
  upper = round(mapValue(newdata_to_send->upper_angle,-6.28318,6.28318,0,65536));

  //converting to 8 bit; had problems getting proper behavior when just using a cast.
  uint8_t rotateH = rotate>>8;
  uint8_t rotateL = rotate;

  uint8_t lowerH = lower>>8;
  uint8_t lowerL = lower;

  uint8_t upperH = upper>>8;
  uint8_t upperL = upper;


  // [mode, H, L, nothing]
  data_to_send[0] = 4;
  data_to_send[1] = rotateH;
  data_to_send[2] = rotateL;
  sendToArm();

  data_to_send[0] = 5;
  data_to_send[1] = lowerH;
  data_to_send[2] = lowerL;
  sendToArm();

  data_to_send[0] = 6;
  data_to_send[1] = upperH;
  data_to_send[2] = upperL;
  sendToArm();

}


int main(int argc, char **argv){
  t = clock();
  //Initialize ROS node
  ros::init(argc,argv,"DirectMotorControl");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber joySub = nh.subscribe("Arm/AngleVelocities",1,setVelocity);
  ros::Subscriber progSub = nh.subscribe("Arm/Position",1,setPosition);
  armPub = nh.advertise<geometry_msgs::QuaternionStamped>("Arm/Diagnostics",1);
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
  double frequency = 5;
  double sleep_duration = 1.0/frequency;
  ros::Rate loop_rate(5);
  while(ros::ok()){
	ros::spinOnce();
	
	if(((float)(clock()-t))/CLOCKS_PER_SEC > sleep_duration){
		data_to_send[0] = 0;
		data_to_send[1] = rotate;
		data_to_send[2] = lower;
		data_to_send[3] = upper;
		data_to_send[4] = gripper;
		sendToArm();
		t = clock();
	}
  
    //loop_rate.sleep();
  }
}
