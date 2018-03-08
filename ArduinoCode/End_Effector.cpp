#include "End_Effector.h"

Gripper::Gripper(int servo_pin) : 
_servo_pin(servo_pin)
{
  _servo = new Servo();
  _servo->attach(_servo_pin);
}
Gripper::Gripper()
{
  // empty constructor
  // to be used with begin
}

void Gripper::begin(int servo_pin)
{
  _servo_pin = servo_pin;
  _servo = new Servo();
  _servo->attach(_servo_pin);
}

void Gripper::open_gripper()
{
  _servo->write(_min_angle);
}
void Gripper::close_gripper()
{
  _servo->write(_max_angle);
}

void Gripper::set_min_angle(int new_min_angle)
{
  _min_angle = new_min_angle;
}

void Gripper::set_max_angle(int new_max_angle)
{
  _max_angle = new_max_angle;
}



