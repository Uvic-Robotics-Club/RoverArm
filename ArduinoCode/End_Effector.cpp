#include "End_Effector.h"

Gripper::Gripper(int servo_pin)
{
  begin(servo_pin);
}
Gripper::Gripper()
{
  // empty constructor
  // to be used with begin
}

void Gripper::begin(int servo_pin)
{
  // save the pin number
  _servo_pin = servo_pin;
  // set to the maximum open and close
  Mapping(0,180);
  // create a new servo
  _servo = new Servo();
  // attach our pin to the servo object
  _servo->attach(_servo_pin);
}

/*
 * Open the gripper to the predefined min angle
 * 
 * NOTE: THIS MAY HAVE TO BE CHANGED TO MAX ANGLE DURING TESTING
 */
void Gripper::Open()
{
  _servo->write(_min_angle);
}
/*
 * Open the gripper to the predefined max angle
 * 
 * NOTE: THIS MAY HAVE TO BE CHANGED TO MIN ANGLE DURING TESTING
 */
void Gripper::Close()
{
  _servo->write(_max_angle);
}

/*
 * Get the angle that the gripper is current at
 */
double Gripper::GetAngle()
{
  return _servo->read();
}

/*
 * Set the min and max angles that the servo will open and close to
 */
void Gripper::Mapping(int new_min_angle, int new_max_angle)
{
  _min_angle = new_min_angle;
  _max_angle = new_max_angle;
}

