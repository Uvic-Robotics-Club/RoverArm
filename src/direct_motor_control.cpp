#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <RoverArm/arm_velocity.h>
#include <RoverArm/joint_angles.h>
#include <serial/serial.h>

#define BASE 0
#define LOWER_ELBOW 1
#define UPPER_ELBOW 2
#define GRIPPER 3



std::string usbPort;
serial::Serial ser;
ros::Publisher armPub;
bool output_to_serial = true;

int data_to_send[] = {0,0,0,0};

double mapValue(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void sendToArm(){
	
    geometry_msgs::QuaternionStamped diagnostics_msg;

    if(output_to_serial){
        ser.flush();
        char buffer[10];
        sprintf(buffer, "[%i,%i]\n", data_to_send[0], data_to_send[1]);
        ser.write(buffer);
        ser.flush();
        ROS_ERROR_STREAM("Program -> "<<buffer);
        //data_to_send[0]=0;
        //data_to_send[1]=0;
			
    }
    else{
		ROS_ERROR_STREAM("Did not try and output the data when OUTPUT TO SERIAL IS " << output_to_serial);
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
  static int rotate = 0;
  static int lower = 0;
  static int upper = 0;

  rotate = (int)round(newdata_to_send->joint.rotate*255.0*newdata_to_send->enable.rotate);
  lower =  (int)round(newdata_to_send->joint.lower*255.0*newdata_to_send->enable.lower);
  upper =  (int)round(newdata_to_send->joint.upper*255.0*newdata_to_send->enable.upper);
  
  

  // [mode, mag, dir, nothing]
  data_to_send[0] = 0;
  data_to_send[1] = rotate;
  sendToArm();

  data_to_send[0] = 1;
  data_to_send[1] = lower;
  sendToArm();

  data_to_send[0] = 2;
  data_to_send[1] = upper;
  sendToArm();
  
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
  std::string data_returned = "";
  //Initialize ROS node
  ros::init(argc,argv,"DirectMotorControl");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber joySub = nh.subscribe("Arm/AngleVelocities",1,setVelocity);
  ros::Subscriber progSub = nh.subscribe("Arm/Position",1,setPosition);
  armPub = nh.advertise<geometry_msgs::QuaternionStamped>("Arm/Diagnostics",1);
  ROS_INFO("STARTING UP THE DMC");
  //nh.param<std::string>("USBPORT", usbPort, "/dev/ttyACM0");


  //Slight re-write for opening serial port; This method gave me the most stability on my machine.
  if (false and !pnh.hasParam("output_type")){
    output_to_serial=false;
  }
  else{
      try
    {
        ser.setPort("/dev/ttyACM0");
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
  ros::Rate loop_rate(20);
  while(ros::ok()){
	if(ser.available()>=1){
		data_returned = "";
		ser.readline(data_returned);
		ROS_ERROR_STREAM("Arduino->" << data_returned);
	}
    ros::spinOnce();
    loop_rate.sleep();
  }
}
