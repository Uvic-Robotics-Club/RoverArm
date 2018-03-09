#include "Linear_Actuator.h"

/*
  This mapping function is the same as the arduino map function as outlined
  here (https://www.arduino.cc/reference/en/language/functions/math/map/) but
  is written for double type instead of long.
*/
double my_map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

LinearActuator::LinearActuator(int pwm_pin, int dir_pin, int feedback_pin)
{
  begin(pwm_pin, dir_pin, feedback_pin);
}

LinearActuator::LinearActuator()
{
  // empty construtor
  // for use with begin
}

void LinearActuator::begin(int pwm_pin, int dir_pin, int feedback_pin)
{
  // setting up the pins
  _pwm_pin = pwm_pin;
  _dir_pin = dir_pin;
  _feed_pin = feedback_pin;
  pinMode(_pwm_pin, OUTPUT);
  pinMode(_dir_pin, OUTPUT);
  pinMode(_feed_pin, INPUT);
  /*
    kp, ki, kd stand for Proportional gain, Integral Gain, and Derivative gain
    these are used in the PID to make it respond better.
    For more information on this look here
    (https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops)
  */
  _kp = 15;
  _ki = 1;
  _kd = 0;

  // input, output, and setpoint are used in the PID
  _input = 0;
  _output = 0;
  _setpoint = 0;
  // _pid.SetSampleTime(1); // this is untested
  _pid = new PID(&_input, &_output, &_setpoint, _kp, _ki, _kd, P_ON_E, REVERSE);
  SetOutputLimits(-255, 255);
}

/*
  This function is used to manually set the speed of the motor.
  Speed should be between -255 and 255 however it is constrained to that in the
  output so if you give it something over, its no big deal.
*/
void LinearActuator::Manual(double speed)
{
  _pid->SetMode(MANUAL);
  _output = speed;
  Output();
}


/*
  This function is called all the time, Essentially it makes sure we have the
  latest feedback value and then react accordingly.
*/
void LinearActuator::Update()
{
  Feedback();
  Output();
}

/*
  Set the new setpoint for the PID. If the PID is disabled, this still sets
  but does nothing
*/
void LinearActuator::SetSetpoint(int new_setpoint)
{
  _setpoint = new_setpoint;
}

/*
  Turning on the PID, This will cause it to start moving next time
  output (or update) is calculated
*/
void LinearActuator::EnablePID()
{
  _pid->SetMode(AUTOMATIC);
}
/*
  Turning off the PID, This will force the motor to stop and be set to manual mode
*/
void LinearActuator::DisablePID()
{
  Manual(0);
  _pid->SetMode(MANUAL);
}

// This sets the output of the PID, external use of this should not be needed
void LinearActuator::SetOutputLimits(int lower_bound, int upper_bound)
{
  _pid->SetOutputLimits(lower_bound, upper_bound);
}

// Return if the PID is turned on
bool LinearActuator::PidEnabled()
{
  return _pid->GetMode() == 0 ? false : true;
}

/*
  This reads an analog in and then maps it into an angle.
  This is assuming that the input is linear.
*/
void LinearActuator::Feedback()
{
  _raw_input = analogRead(_feed_pin);
  _input = my_map(_raw_input, _lower_absolute_limit, _upper_absolute_limit,
                  _lower_actual_angle, _upper_actual_angle);
}

/*
  This changes the mapping that is used in the feedback method.
  low raw and high raw correspond with the value that the analog input returns
  low_angle and high_angle correspond with the values that they need to be mapped
  to
*/
void LinearActuator::Mapping(double low_raw, double high_raw, double low_angle, double high_angle)
{
  _lower_absolute_limit = low_raw;
  _upper_absolute_limit = high_raw;
  _lower_actual_angle = low_angle;
  _upper_actual_angle = high_angle;
}

/*
  The output method is the one place where the object goes to make any changes
  to the pins. This includes any digital writes or analog writes.

  By keeping all of the outputs in one place it is easier to implement soft
  limits.

  This is where most of the magic happens
*/
int LinearActuator::Output()
{
  // pid_computed is if the PID actually computed the output or not.
  // only really used in debugging. Could be removed with no harm.
  int pid_computed = 0;

  /*
    If the PID is in manual mode Compute returns right away not calculating
    anything, this means that if you set the output elsewhere it wont be changed
    by calling comput
  */
  pid_computed = _pid->Compute();

  /*
    This block stops you from moving past the soft limit. Essentially it will
    stop in one direction but let you keep going in the other direction.
  */
  if ((_input <= _lower_soft_limit && _output > 0) || (_input >= _upper_soft_limit && _output < 0)) {
    analogWrite(_pwm_pin, 0);
    return -1;
  }

  /*
    With our linear actuators if we have a output of less than 80 they dont move
    and just make a bad noise. This just adds a bit of protection for our ears.
  */
  _output = constrain(_output, -255, 255);
  _output = abs(_output) > 80 ? _output : 0;
  analogWrite(_pwm_pin, abs(_output));

  /*
     This block sets the default direction for the linear actuator.

     I think this one line could replace this block but its untested
     _output>0 ? digitalWrite(_dir_pin,1) : digitalWrite(_dir_pin,0);
  */

  if (_output > 0) {
    digitalWrite(_dir_pin, 1);
  }
  else {
    digitalWrite(_dir_pin, 0);
  }
  return pid_computed;
}

/*
   This allows for soft limits to be set so that the arm will not go past
   certain angles. This is useful for demos to prevent crashing.
*/
void LinearActuator::SetSoftLimits(int lower_bound, int upper_bound)
{
  _upper_soft_limit = upper_bound;
  _lower_soft_limit = lower_bound;
}

void LinearActuator::SetPIDGains(double new_kp, double new_ki, double new_kd)
{
  _kp = new_kp;
  _ki = new_ki;
  _kd = new_kd;
  _pid->SetTunings(_kp, _ki, _kd);
}
/*
   BELOW THIS POINT IS ALL GETTERS
*/
double LinearActuator::GetPGain()
{
  return _kp;
}
double LinearActuator::GetIGain()
{
  return _ki;
}
double LinearActuator::GetDGain()
{
  return _kd;
}
double LinearActuator::GetInput()
{
  return _input;
}
double LinearActuator::GetAngle()
{
  return _input;
}
double LinearActuator::GetRawInput()
{
  return _raw_input;
}
double LinearActuator::GetSetpoint()
{
  return _setpoint;
}
double LinearActuator::GetOutput()
{
  return _output;
}



