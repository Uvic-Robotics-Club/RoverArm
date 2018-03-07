#ifndef End_Effector_h
#define End_Effector_h
#include <Servo.h>

class Gripper{
public:
  Gripper(int servo_pin);
  void Open();
  void Close();
private:
  Servo* _servo;
  int _servo_pin;
};


#endif
