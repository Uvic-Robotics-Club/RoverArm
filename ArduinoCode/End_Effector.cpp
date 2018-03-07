#include "End_Effector.h"
#include <Servo.h>

Gripper::Gripper(int servo_pin) : _servo_pin(servo_pin){
  _servo = new Servo();
  _servo->attach(_servo_pin);
}
