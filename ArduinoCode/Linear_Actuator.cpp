#include "Linear_Actuator.h"
#include "PID_v1.h"
#include "Arduino.h"


LinearActuator::LinearActuator(int pwm_pin, int dir_pin) :
_pid(&_input, &_output, &_setpoint, _kp, _ki, _kd, P_ON_E, DIRECT)
{
    _pwm_pin = pwm_pin;
    _dir_pin = dir_pin;
    _feed_pin = -1;
    pinMode(_pwm_pin, OUTPUT);
    pinMode(_dir_pin, OUTPUT);
    _kp = 0.1;
    _ki = 1;
    _kd = 0;
    _mode = VELOCITY;
     //P_ON_E (Proportional on Error) is the default behavior
}

LinearActuator::LinearActuator(int pwm_pin, int dir_pin, int feedback_pin) :
_pid(&_input, &_output, &_setpoint, _kp, _ki, _kd, P_ON_E, DIRECT)
{
    _pwm_pin = pwm_pin;
    _dir_pin = dir_pin;
    _feed_pin = feedback_pin;
    pinMode(_pwm_pin, OUTPUT);
    pinMode(_dir_pin, OUTPUT);
    pinMode(_feed_pin, INPUT);
    _kp = 0.1;
    _ki = 1;
    _kd = 0;
    _mode = VELOCITY;
}

void LinearActuator::manual(int speed){
    _mode = VELOCITY;
    _pid.SetMode(MANUAL);
    _output = speed;
    //Output(); // once this is tested, try uncommenting this
    analogWrite(_pwm_pin,abs(speed));
    if (speed>0) {
        digitalWrite(_dir_pin,1);
    } else{
        digitalWrite(_dir_pin,0);
    }
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
    _pid.SetMode(AUTOMATIC);
}

void LinearActuator::DisablePID(){
    manual(0);
    _mode = VELOCITY;
    _pid.SetMode(MANUAL);
}

void LinearActuator::SetOutputLimits(int lower_bound, int upper_bound){
    _pid.SetOutputLimits(lower_bound,upper_bound);
}

void LinearActuator::Feedback(){
    if (_feed_pin != -1 ) {
        _input = map(analogRead(_feed_pin), 0, 1024, 15, 50);
    }
}

void LinearActuator::Output(){
    _pid.Compute();
    analogWrite(_pwm_pin,abs(_output));
    if (_output>0) {
        digitalWrite(_dir_pin,1);
    } else{
        digitalWrite(_dir_pin,0);
    }
}