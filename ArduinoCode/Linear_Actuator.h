#ifndef Linear_Actuator_h
#define Linear_Actuator_h
#include "PID_v1.h"

class LinearActuator{
public:
#define MANUAL 0
#define AUTO 1
#define VELOCITY 1
#define POSITION 0
#define UPPER 0
#define LOWER 1

  LinearActuator(int pwm_pin, int dir_pin, int lin_number); // constructor for purely manual control
  LinearActuator(int pwm_pin, int dir_pin, int feedback_pin, int lin_number);
  void manual(int speed);
  void Update();
  void SetOutputLimits(int lower_bound, int upper_bound);
  void Feedback();
  void Output();
  void SetSetpoint(int new_setpoint);
  void EnablePID();
  void DisablePID();



  int _pwm_pin, _dir_pin, _mode;
  int _feed_pin, _actuator;
  double _setpoint, _input, _output;
  double _raw_input;
  double _kp, _ki, _kd;
  double _lower_limit, _upper_limit;
private:
  PID _pid;
};


#endif

