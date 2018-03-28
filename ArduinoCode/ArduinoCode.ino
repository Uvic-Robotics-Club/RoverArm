#include "Stepper_Motor.h"
#include "Linear_Actuator.h"
/* COMMENT #1
  All Gripper/End_effector Code will be commented out to prevent issues with lower
  When this is confirmed with Electrical That we can safely switch the DIR and PWM pins then we can uncomment
  only uncomment this if you are sure the servo library will not interfere with the analog write on lower
*/
#include "End_Effector.h"


/*
  This will control 4 joints of the Rover Arm

  Joint 1: Rotate
  this is the base rotation of the arm, using a stepper motor to adjust the angle
  For how to use this class please look in Stepper_Motor.h. It should be self explanitory

  Joint 2: Lower
  this is the lower elbow joint of the arm, using a linear actuator to adjust the angle
  feedback = pot on linear actuator

  Joint 3: Upper
  this is the upper elbow joint of the arm, using a linear actuator to adjust the angle
  feedback = pot on linear actuator

  Joint 4: Gripper (NOT YET IMPLEMENTED)
  this is the end effector, using a stepper motor to open and close
  feedback = force sensor ?
*/

#define VELOCITY 1
#define POSITION 0
#define TIMEOUT 1000


//Stepper motor and corresponding limit switch Settings
#define rotateLimitPin 2 // not in use as of now
#define STEPS_PER_ROTATION 200 // defined by the motor
#define ROTATE_MICRO_STEPPING 4 // defined by the motor driver
#define ROTATE_DIR  4
#define ROTATE_STEP 7


//Lower Joint linear actuator Settings
// pins 10 and 11 for uno and nano
#define LOWER_PWM 11
#define LOWER_DIR 10
#define LOWER_FEEDBACK A0

//Upper Joint linear actuator Settings
// pins 5 and 6 for uno and nano
#define UPPER_PWM 5
#define UPPER_DIR 6
#define UPPER_FEEDBACK A1

// Gripper Joint Servo Settings
#define GRIPPER_PIN 9


// This is used so that if we lose comms then everything will stop
double LastMessageReceived = 0;

// create a stepper motor object
StepperMotor rotate;

// make the lower linear actuator object
LinearActuator lower;
LinearActuator upper;

// make the gripper object
Gripper gripper; // see comment #1

double my2_map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // Serial Setup
  Serial.begin(115200);

  //Rotate Stepper Setup
  // the limit pin is not being used right now
  pinMode(rotateLimitPin, INPUT);
  rotate.begin(ROTATE_DIR, ROTATE_STEP, STEPS_PER_ROTATION, ROTATE_MICRO_STEPPING);

  //Lower Linear Actuator Setup
  // pins setup inside the class
  lower.begin(LOWER_PWM, LOWER_DIR, LOWER_FEEDBACK);
  lower.Mapping(120, 300, 93, 40);
  lower.SetSoftLimits(45, 360);


  //Upper Linear Actuator Setup
  upper.begin(UPPER_PWM, UPPER_DIR, UPPER_FEEDBACK);
  upper.Mapping(330, 684, 162, 52.5);
  upper.SetSoftLimits(50, 135);

  /* see comment #1*/
  gripper.begin(GRIPPER_PIN);
  gripper.Mapping(100, 180);

  lower.Manual(0);
  upper.Manual(0);
  rotate.Manual(0);

}

void loop() {
  // If the last message was over a second ago, turn everything to manual and turn it off.
  if ((millis() - LastMessageReceived) > TIMEOUT) {
    lower.Manual(0);
    upper.Manual(0);
    rotate.Manual(0);
    Serial.println("TIMEOUT");
  }
  // call the update on all joints
  lower.Update();
  upper.Update();
  rotate.Update();
  // report back all the stuff that I need for feedback
  Serial.print(lower.GetAngle());
  Serial.print(",");
  Serial.print(upper.GetAngle());
  Serial.print(",");
  Serial.print(gripper.GetAngle());
  Serial.print(",");
  Serial.println(rotate.GetAngle());

  // this delay is just so that serial event is not called to much (allows data to come in)
  delay(50);
}

void serialEvent() {
  int mode, value;
  while (Serial.available() > 4) {
    if (Serial.find("M:")) {
      mode = Serial.parseInt();
      if (Serial.find("V:")) {
        value = Serial.parseInt();
      }
      else {
        return;
      }
    }
    else {
      return;
    }
    // the program has gotten here then the mode and value are valid.
    LastMessageReceived = millis();
    switch (mode) {
      case 0:
        rotate.Manual(value/10);
        break;
      case 1:
        lower.Manual(value);
        break;
      case 2:
        upper.Manual(value);
        break;
      case 3:
        /* See Comment # 1*/
        if (value > 0) {
          gripper.Open();
        }
        else if (value < 0) {
          gripper.Close();
        }

        break;
      case 4:
        rotate.SetSetpoint(value);
        break;
      case 5:
        lower.SetSetpoint(value);
        lower.EnablePID();
        break;
      case 6:
        upper.SetSetpoint(value);
        upper.EnablePID();
        break;
    };

  } // end of the while loop
}

