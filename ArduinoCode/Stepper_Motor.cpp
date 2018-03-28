#include "Stepper_Motor.h"


StepperMotor::StepperMotor(int dir_pin, int step_pin, int steps_per_rotation, int micro_stepping)
{
  begin(dir_pin, step_pin, steps_per_rotation, micro_stepping);
}

StepperMotor::StepperMotor()
{
  // empty constructor
  // make sure to call begin if you use this
}

void StepperMotor::begin(int dir_pin, int step_pin, int steps_per_rotation, int micro_stepping)
{
  // set up the pins
  _dir_pin = dir_pin;
  _step_pin = step_pin;
  pinMode(_dir_pin, OUTPUT);
  pinMode(_step_pin, OUTPUT);
  MappingSteps(steps_per_rotation, micro_stepping);
  // put it in automatic mode
  _manual = false;
  // create a new stepper object
  stepper = new AccelStepper(AccelStepper::DRIVER, _step_pin, _dir_pin, 0, 0, true);
  // Set the max speed to 10 RPM
  SetMaxSpeed(10);
}

void StepperMotor::MappingSteps(int steps_per_rotation, int micro_stepping)
{
  _steps_per_rotation = steps_per_rotation;
  _micro_stepping = micro_stepping;
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
  
  double our_speed = new_speed; // in rpm
  our_speed /= 60; // to get to rotation per second
  our_speed /= (_steps_per_rotation * _micro_stepping); // to get to steps per second
  stepper->setSpeed(new_speed); // our_speed dosnt work yet
  stepper->runSpeed();
}

// This is the angle that you want to go to in degrees
void StepperMotor::SetSetpoint(int new_setpoint)
{
  _manual = false;
  /*
     This math works out to be

     (steps to go around 1 rev)*(multiplication of steps due to microstepping)*(1/360 or degrees per rev)*(new degrees)

     In units it works out to be

      steps |  unitless | rev    | degrees |
     -------|-----------|--------|---------|   -> final unit is steps
        rev |           | degrees|         |
  */
  double ang_to_step = _steps_per_rotation;
  ang_to_step *= _micro_stepping;
  ang_to_step *= _percent_angle; // this is 1/360. multiplication is faster than division
  ang_to_step *= new_setpoint;

  stepper->move((long)new_setpoint);
}
void StepperMotor::SetMaxSpeed(int rpm)
{
  /*
     SetMaxSpeed takes in a float that is steps per second
     The math below works out to be
     (rotation per min)/(sec per min)/(steps per rotation)

     The units work out to be

     rotation|   min   |   steps  | unitless |
     --------|---------|----------|----------|  -> Final unit is steps per second
       min   | seconds | rotation |          |

  */
  double our_speed = rpm;
  our_speed /= 60; // to get to rotation per second
  our_speed /= (_steps_per_rotation * _micro_stepping); // to get to steps per second
  stepper->setMaxSpeed(100);
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



