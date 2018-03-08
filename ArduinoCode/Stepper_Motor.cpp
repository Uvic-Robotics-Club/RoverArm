#include "Stepper_Motor.h"

// primary constructor
StepperMotor::StepperMotor(int dir_pin, int step_pin, int steps_per_rotation)
{
  begin(dir_pin, step_pin, steps_per_rotation);
}

StepperMotor::StepperMotor()
{
  // empty constructor
  // make sure to call begin if you use this
}

void StepperMotor::begin(int dir_pin, int step_pin, int steps_per_rotation)
{
  pinMode(_dir_pin, OUTPUT);
  pinMode(_step_pin, OUTPUT);
  _dir_pin = dir_pin;
  _step_pin = step_pin;
  _steps_per_rotation = steps_per_rotation;
  _manual = false;
  _percent_step = 1 / steps_per_rotation;
  stepper = new AccelStepper(AccelStepper::DRIVER, _step_pin, _dir_pin, 0, 0, true);
  // this is 10 rpm
  stepper->setMaxSpeed(1000);
}

// this needs to be called quite quickly
void StepperMotor::Update()
{
  if (not _manual)
  {
    stepper->run();
  }
  else
  {
    stepper->runSpeed();
  }
}
double StepperMotor::GetSpeed()
{
  return stepper->speed();
}

void StepperMotor::Manual(int new_speed)
{
  _manual = true;
  stepper->setSpeed(new_speed);
  stepper->runSpeed();
}

// This is the angle that you want to go to in degrees
void StepperMotor::SetSetpoint(int new_setpoint)
{
  _manual = false;
  double ang_to_step = (((double)new_setpoint) * _percent_angle) * _steps_per_rotation;
  stepper->move((long)new_setpoint);
}

void StepperMotor::Home() {
  stepper->setCurrentPosition(0);
}

void StepperMotor::GoHome() {
  stepper->runToNewPosition(0);
}

double StepperMotor::GetAngle()
{
  return stepper->currentPosition();
}



