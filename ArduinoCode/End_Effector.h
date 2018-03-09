#ifndef End_Effector_h
#define End_Effector_h
#include "Arduino.h"
#include "Servo.h"

class Gripper{
public:
  Gripper(int servo_pin);
  Gripper();
  void begin(int servo_pin);
  void Open();
  void Close();
  void Mapping(int new_min_angle, int new_max_angle);
  double GetAngle();
private:
  Servo* _servo;
  int _servo_pin;
  int _max_angle,_min_angle;
};


#endif
