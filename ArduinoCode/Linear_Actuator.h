#ifndef Linear_Actuator_h
#define Linear_Actuator_h
#include "PID_v1.h"

class LinearActuator{
public:
  #define MANUAL 0
  #define AUTO 1
  #define VELOCITY 1
  #define POSITION 0

  LinearActuator(int pwm_pin, int dir_pin); // constructor for purely manual control
  LinearActuator(int pwm_pin, int dir_pin, int feedback_pin);
  void manual(int speed);
  void Update();
  void SetOutputLimits(int lower_bound, int upper_bound);
  void Feedback();
  void Output();
  void SetSetpoint(int new_setpoint);
  void EnablePID();
  void DisablePID();


private:
    int _pwm_pin, _dir_pin, _mode;
    int _feed_pin;
    double _setpoint, _input, _output;
    double _kp, _ki, _kd;
    PID _pid;
};


#endif
