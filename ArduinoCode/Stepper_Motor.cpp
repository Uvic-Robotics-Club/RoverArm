#include "Stepper_Motor.h"
#include <AccelStepper.h>

// primary constructor
StepperMotor::StepperMotor(int dir_pin, int step_pin, int steps_per_rotation)
: _dir_pin(dir_pin),
_step_pin(step_pin),
_steps_per_rotation(steps_per_rotation)
{
_manual = false;
_percent_step = 1/steps_per_rotation;
stepper = new AccelStepper(AccelStepper::DRIVER,  _dir_pin, _step_pin, 0,0, true);
// this is 10 rpm
stepper->setMaxSpeed(10);

}

// this needs to be called quite quickly
void StepperMotor::Update()
{
  if(not _manual)
  {
    stepper->run();
  }
  else
  {
    stepper->runSpeed();
  }
}

void StepperMotor::Manual(int speed)
{
  _manual = true;
  stepper->setSpeed(speed);
  stepper->runSpeed();
}

// This is the angle that you want to go to in degrees
void StepperMotor::SetSetpoint(int new_setpoint)
{
  _manual = false;
  double ang_to_step = (((double)new_setpoint)*_percent_angle)*_steps_per_rotation;
  stepper->move((long)ang_to_step);
}

void StepperMotor::Home(){
  stepper->setCurrentPosition(0);
}

void StepperMotor::GoHome(){
  stepper->runToNewPosition(0);
}

double StepperMotor::GetAngle()
{
  return stepper->currentPosition()*_percent_step*360.0f;
}

