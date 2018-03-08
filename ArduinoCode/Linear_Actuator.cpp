#include "Linear_Actuator.h"
#include "PID_v1.h"
#include "Arduino.h"



LinearActuator::LinearActuator(int pwm_pin, int dir_pin, int feedback_pin, int lin_number)
{

  _pwm_pin = pwm_pin;
  _dir_pin = dir_pin;
  _feed_pin = feedback_pin;
  if(lin_number==0){
    _actuator=UPPER;
  }
  else{
    _actuator=LOWER;
  }
  pinMode(_pwm_pin, OUTPUT);
  pinMode(_dir_pin, OUTPUT);
  pinMode(_feed_pin, INPUT);
  _kp = 15;
  _ki = 1;
  _kd = 0;
  _mode = VELOCITY;
  _input = 0;
  _output = 0;
  _setpoint = 0;
  // _pid.SetSampleTime(1);
  _pid = new PID(&_input, &_output, &_setpoint, _kp, _ki, _kd, P_ON_E, REVERSE);
  SetOutputLimits(-255,255);
}

LinearActuator::LinearActuator()
{
  // empty construtor
  // for use with begin 
}

void LinearActuator::begin(int pwm_pin, int dir_pin, int feedback_pin, int lin_number)
{

  _pwm_pin = pwm_pin;
  _dir_pin = dir_pin;
  _feed_pin = feedback_pin;
  if(lin_number==0){
    _actuator=UPPER;
  }
  else{
    _actuator=LOWER;
  }
  pinMode(_pwm_pin, OUTPUT);
  pinMode(_dir_pin, OUTPUT);
  pinMode(_feed_pin, INPUT);
  _kp = 15;
  _ki = 1;
  _kd = 0;
  _mode = VELOCITY;
  _input = 0;
  _output = 0;
  _setpoint = 0;
  // _pid.SetSampleTime(1);
  _pid = new PID(&_input, &_output, &_setpoint, _kp, _ki, _kd, P_ON_E, REVERSE);
  SetOutputLimits(-255,255);
}

void LinearActuator::manual(double speed){
  _mode = VELOCITY;
  _pid->SetMode(MANUAL);
  _output = speed;
  Output(); // once this is tested, try uncommenting this
  /*if ((_input <= _lower_limit && speed > 0)||(_input >= _upper_limit && speed < 0)){
   analogWrite(_pwm_pin,0);
   return;
   }
   analogWrite(_pwm_pin,abs(speed));
   if (speed>0) {
   digitalWrite(_dir_pin,1);
   } else{
   digitalWrite(_dir_pin,0);
   }
   */
}



void LinearActuator::Update(){
  Feedback();
  Output();
}

void LinearActuator::SetSetpoint(int new_setpoint){
  _setpoint = new_setpoint;
}

void LinearActuator::EnablePID(){
  _mode = POSITION;
  _pid->SetMode(AUTOMATIC);
}

void LinearActuator::DisablePID(){
  manual(0);
  _mode = VELOCITY;
  _pid->SetMode(MANUAL);
}

void LinearActuator::SetOutputLimits(int lower_bound, int upper_bound){
  _pid->SetOutputLimits(lower_bound,upper_bound);
}

bool LinearActuator::pidEnabled(){
  return false; //_pid.GetMode()==0 ? false:true;
}

double my_map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void LinearActuator::Feedback(){
  if (_feed_pin != -1 ) {
    _raw_input = analogRead(_feed_pin);
    if( _actuator == LOWER){
      _input = my_map(_raw_input, 86, 300, 93, 40);
    }
    else{
      _input = my_map(_raw_input, 473, 892, 162, 52.5); 
    }
  }
}

int LinearActuator::Output(){
  int temp = 0;
  temp = _pid->Compute();
  //if(_input<_setpoint){
  //  _output-= 1;
  //}
  //if(_input>_setpoint){
  //  _output+= 1;
  //}
  if ((_input <= _lower_limit && _output > 0)||(_input >= _upper_limit && _output < 0)){
    analogWrite(_pwm_pin,0);
    return -1;
  }
  //Serial.print(_actuator);
  // Serial.print(" is setting the output to ");
  //Serial.println(_output);
  _output = abs(_output)>80 ? _output : 0;
  analogWrite(_pwm_pin,abs(_output));

  if (_output>0) {
    digitalWrite(_dir_pin,1);
  } 
  else{
    digitalWrite(_dir_pin,0);
  }
  return temp;
}



