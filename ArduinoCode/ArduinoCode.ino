#include "Stepper_Motor.h"
#include "PID_v1.h"
#include "Linear_Actuator.h"
//#include <Servo.h>
//#include <AccelStepper.h>

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
 feedback = force sensor
 */

#define VELOCITY 1
#define POSITION 0
#define TIMEOUT 1000


//Stepper motor and corresponding limit switch Settings
#define rotateLimitPin 2 // not in use as of now
#define STEPS_PER_ROTATION 800
#define ROTATE_DIR  4
#define ROTATE_STEP 7


//Lower Joint linear actuator Settings
// pins 10 and 11 for uno and nano
#define LOWER_PWM 10
#define LOWER_DIR 11
#define LOWER_FEEDBACK A0

//Upper Joint linear actuator Settings
// pins 5 and 6 for uno and nano
#define UPPER_PWM 5
#define UPPER_DIR 6
#define UPPER_FEEDBACK A1



double LastMessageReceived = 0;

StepperMotor rotate;


LinearActuator lower(LOWER_PWM, LOWER_DIR, LOWER_FEEDBACK, LOWER);
double lowerInput, lowerOutput, lowerSetpoint;
PID lowerPID(&lowerInput, &lowerOutput, &lowerSetpoint, lower._kp, lower._ki, lower._kd, P_ON_E, REVERSE);


LinearActuator upper(UPPER_PWM, UPPER_DIR, UPPER_FEEDBACK, UPPER);
double upperInput, upperOutput, upperSetpoint;
PID upperPID(&upperInput, &upperOutput, &upperSetpoint, upper._kp, upper._ki, upper._kd, P_ON_E, REVERSE);

double GripperSetpoint, GripperInput, GripperOutput;
double GripperKp = 0.1, GripperKi = 1, GripperKd = 0;
int GripperMode = VELOCITY;
PID gripper(&GripperInput, &GripperOutput, &GripperSetpoint, GripperKp, GripperKi, GripperKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

void UpdateGripper();
double GripperFeedback();
void OutputGripper();

double my2_map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // Serial Setup
  Serial.begin(115200);

  //Rotate Stepper Setup
  pinMode(rotateLimitPin, INPUT);
  //rotateStepper.setSpeed(50);
  pinMode(ROTATE_DIR, OUTPUT);
  pinMode(ROTATE_STEP, OUTPUT);
  rotate.begin(ROTATE_DIR, ROTATE_STEP, STEPS_PER_ROTATION);

  //Lower Linear Actuator Setup
  // pins setup inside the class
  lower.SetOutputLimits(-255, 255);
  lower._upper_limit = 360;
  lower._lower_limit = 45;
  lowerPID.SetOutputLimits(-255, 255);
  lowerPID.SetMode(1);

  //Upper Linear Actuator Setup
  upper.SetOutputLimits(-255, 255);
  upper._lower_limit = 0;
  upper._upper_limit = 135;
  upperPID.SetOutputLimits(-255, 255);
  upperPID.SetMode(1);


}

void loop() {
  if ((millis() - LastMessageReceived)>TIMEOUT){
    // If the last message was over a second ago, turn everything to manual and turn it off.
    lower.manual(0);
    lowerPID.SetMode(0);
    upper.manual(0);
    lowerPID.SetMode(0);
    rotate.Manual(0);
    Serial.println("TIMEOUT");
  }

  lower.Update();
  upper.Update();
  rotate.Update();
  
  upperInput = upper._input;
  upperOutput = upper._output;
  upperSetpoint = upper._setpoint;
  //upperPID.Compute();
  //upper.manual(upperOutput);

  lowerInput = lower._input;
  lowerOutput = lower._output;
  lowerSetpoint = lower._setpoint;
  //lowerPID.Compute();
  //lower.manual(lowerOutput);

  Serial.print(upper._input);
  Serial.print(",");
  Serial.print(lower._input);
  Serial.print(",");
  Serial.println(rotate.GetAngle());

  delay(50);
}

void serialEvent() {
  int mode, value;
  String mode_string, value_string;
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
      rotate.Manual(value);
      break;
    case 1:
      lower.manual(value);
      lowerPID.SetMode(0);
      break;
    case 2:
      upper.manual(value);
      upperPID.SetMode(0);
      break;
    case 3:
      GripperMode = VELOCITY;
      GripperSetpoint = value;
      GripperOutput = GripperSetpoint;
      gripper.SetMode(MANUAL);
      break;
    case 4:
      rotate.SetSetpoint(value);
      break;
    case 5:
      lower.SetSetpoint(value);
      lowerPID.SetMode(1);
      break;
    case 6:
      upper.SetSetpoint(value);
      upperPID.SetMode(1);
      break;
    };

  }
}




void UpdateGripper() {
  static double LastValue = 0;
  static double LatestValue = 0;
  if (GripperMode == VELOCITY) {
    GripperInput = LatestValue - LastValue;
  }
  else if (GripperMode == POSITION) {
    GripperInput = LatestValue;
  }
  gripper.Compute();
  OutputGripper();

}


double GripperFeedback() {
  // have specific feedback for the gripper joint here
  return 0.0;
}

void OutputGripper() {
  // have specific output for the gripper joint here
}








