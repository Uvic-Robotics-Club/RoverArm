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

uint8_t data_to_send[] = {0,0,0,0};

double mapValue(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void sendToArm(){
    geometry_msgs::QuaternionStamped diagnostics_msg;

    if(output_to_serial){
        ser.flush();
        ser.write(&data_to_send[0],4);
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
  }
  armPub.publish(diagnostics_msg);
}


void setVelocity(const RoverArm::arm_velocity::ConstPtr& newdata_to_send){
  static int rotate = 0;
  static int lower = 0;
  static int upper = 0;

  rotate = round(newdata_to_send->joint.rotate*255.0*newdata_to_send->enable.rotate);
  lower = round(newdata_to_send->joint.lower*255.0*newdata_to_send->enable.lower);
  upper = round(newdata_to_send->joint.upper*255.0*newdata_to_send->enable.upper);

  uint8_t rotDir = abs(rotate);

  //I switched this to magnitude and direction just for my testing as it made more sense to me, let me know what you think
  // [mode, mag, dir, nothing]
  data_to_send[0] = 0;
  if(rotate>0){
    data_to_send[1] = rotDir;
    data_to_send[2] = 1;
  }
  else{
    data_to_send[1] = rotDir;
    data_to_send[2] = 0;
  }
  sendToArm();

  data_to_send[0] = 1;
  if(round(lower)>0){
    data_to_send[1] = 0;
    data_to_send[2] = (uint8_t)abs(lower);
  }
  else{
    data_to_send[1] = (uint8_t)abs(lower);
    data_to_send[2] = 0;
  }
  sendToArm();

  data_to_send[0] = 2;
  if(round(upper)>0){
    data_to_send[1] = 0;
    data_to_send[2] = (uint8_t)abs(upper);
  }
  else{
    data_to_send[1] = (uint8_t)abs(upper);
    data_to_send[2] = 0;
  }
  sendToArm();
}

void setPosition(const RoverArm::joint_angles::ConstPtr& newdata_to_send){
  static unsigned int rotate = 0;
  static unsigned int lower = 0;
  static unsigned int upper = 0;
  // maping the -2pi -> 2pi that the angle is in to the max and min of a uint16
  rotate = (unsigned int) round(mapValue(newdata_to_send->base_angle,-6.28318,6.28318,0,65536));
  lower = (unsigned int) round(mapValue(newdata_to_send->lower_angle,-6.28318,6.28318,0,65536));
  upper = (unsigned int) round(mapValue(newdata_to_send->upper_angle,-6.28318,6.28318,0,65536));

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
  //Initialize ROS node
  ros::init(argc,argv,"DirectMotorControl");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber joySub = nh.subscribe("Arm/AngleVelocities",1,setVelocity);
  ros::Subscriber progSub = nh.subscribe("Arm/Position",1,setPosition);
  armPub = nh.advertise<geometry_msgs::QuaternionStamped>("Arm/Diagnostics",20);
  //nh.param<std::string>("USBPORT", usbPort, "/dev/ttyACM0");


  //Slight re-write for opening serial port; This method gave me the most stability on my machine.
  if (!pnh.hasParam("output_type")){
    output_to_serial=false;
  }
  else{
      try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(9600);
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
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
  }


  ros::Rate loop_rate(100);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}
