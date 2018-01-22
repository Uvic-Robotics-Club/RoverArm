
#include <PID_v1.h>

/*
   This will control 4 joints

   Joint 1: Rotate
      this is the base rotation of the arm, using a stepper motor to adjust the angle
      feedback = switch for homing, then open loop

   Joint 2: Lower
      this is the lower elbow joint of the arm, using a linear actuator to adjust the angle
      feedback = pot on linear actuator

   Joint 3: Upper
      this is the upper elbow joint of the arm, using a linear actuator to adjust the angle
      feedback = pot on linear actuator

   Joint 4: Gripper
      this is the end effector, using a stepper motor to open and close
      feedback = force sensor

*/

#define VELOCITY 1
#define POSITION 0
#define TIMEOUT 1000

double LastMessageReceived = 0;

double RotateSetpoint, RotateInput, RotateOutput;
double RotateKp = 1, RotateKi = 1, RotateKd = 0;
int RotateMode = VELOCITY;
PID rotate(&RotateInput, &RotateOutput, &RotateSetpoint, RotateKp, RotateKi, RotateKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

#define LOWERFEEDBACKPIN A0
double LowerSetpoint, LowerInput, LowerOutput;
double LowerKp = 1, LowerKi = 1, LowerKd = 0;
int LowerMode = VELOCITY;
PID lower(&LowerInput, &LowerOutput, &LowerSetpoint, LowerKp, LowerKi, LowerKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

#define UPPERFEEDBACKPIN A1
double UpperSetpoint, UpperInput, UpperOutput;
double UpperKp = 1, UpperKi = 1, UpperKd = 0;
int UpperMode = VELOCITY;
PID upper(&UpperInput, &UpperOutput, &UpperSetpoint, UpperKp, UpperKi, UpperKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

double GripperSetpoint, GripperInput, GripperOutput;
double GripperKp = 1, GripperKi = 1, GripperKd = 0;
int GripperMode = VELOCITY;
PID gripper(&GripperInput, &GripperOutput, &GripperSetpoint, GripperKp, GripperKi, GripperKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

void UpdateRotate();
void UpdateLower();
void UpdateUpper();
void UpdateGripper();
double RotateFeedback();
double LowerFeedback();
double UpperFeedback();
double GripperFeedback();
void OutputRotate();
void OutputLower();
void OutputUpper();
void OutputGripper();


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  UpdateRotate();
  UpdateLower();
  UpdateUpper();
  UpdateGripper();
}

void serialEvent() {
  unsigned char mode = Serial.read();
  unsigned char first  = Serial.read();
  unsigned char second  = Serial.read();
  unsigned char third  = Serial.read();
  LastMessageReceived = millis();
  switch (mode) {
    case 0:
      RotateMode = VELOCITY;
      RotateSetpoint = -1 * first + second; //[rotate-,rotate+]
      break;
    case 1:
      LowerMode = VELOCITY;
      LowerSetpoint = -1 * first + second; //[lower-,lower+]
      break;
    case 2:
      UpperMode = VELOCITY;
      UpperSetpoint = -1 * first + second; //[upper-,upper+]
      break;
    case 3:
      GripperMode = VELOCITY;
      GripperSetpoint = -1 * first + second; //[gripper-,gripper+]
      break;
    case 4:

      // this might cause an issue trying to left shift a char
      RotateMode = POSITION;
      RotateSetpoint = map(first << 4 + second, 0, 65535, -180, 180);
      break;
    case 5:
      LowerMode = POSITION;
      LowerSetpoint = map(first << 4 + second, 0, 65535, -180, 180);
      break;
    case 6:
      UpperMode = POSITION;
      UpperSetpoint = map(first << 4 + second, 0, 65535, -180, 180);
      break;
  };
}

void UpdateRotate() {
  static double LastValue = 0;
  static double LatestValue = 0;
  LatestValue = RotateFeedback();
  if (RotateMode == VELOCITY) {
    RotateInput = LatestValue - LastValue;
  }
  else if (RotateMode == POSITION) {
    RotateInput = LatestValue;
  }
  rotate.Compute();
  OutputRotate();
}

void UpdateLower() {
  static double LastValue = 0;
  static double LatestValue = 0;
  LatestValue = RotateFeedback();
  if (LowerMode == VELOCITY) {
    LowerInput = LatestValue - LastValue;
  }
  else if (LowerMode == POSITION) {
    LowerInput = LatestValue;
  }
  lower.Compute();
  OutputLower();
}

void UpdateUpper() {
  static double LastValue = 0;
  static double LatestValue = 0;
  LatestValue = RotateFeedback();
  if (UpperMode == VELOCITY) {
    UpperInput = LatestValue - LastValue;
  }
  else if (UpperMode == POSITION) {
    UpperInput = LatestValue;
  }
  upper.Compute();
  OutputUpper();
}

void UpdateGripper() {
  static double LastValue = 0;
  static double LatestValue = 0;
  LatestValue = RotateFeedback();
  if (GripperMode == VELOCITY) {
    GripperInput = LatestValue - LastValue;
  }
  else if (GripperMode == POSITION) {
    GripperInput = LatestValue;
  }
  gripper.Compute();
  OutputGripper();

}

double RotateFeedback() {
  // have specific feedback for the rotation joint here
  return 0.0;
}

double LowerFeedback() {
  /*
   * 0    Lowerbound on ADC input
   * 1024 Upperbound on ADC input
   * 15   Lowerbound on Lower Angle
   * 50   Upperbound on Lower Angle
   */
  return map(analogRead(LOWERFEEDBACKPIN),0,1024,15,50);
}

double UpperFeedback() {
  /*
   * 0    Lowerbound on ADC input
   * 1024 Upperbound on ADC input
   * 30   Lowerbound on Lower Angle
   * 80   Upperbound on Lower Angle
   */
  return map(analogRead(UPPERFEEDBACKPIN),0,1024,30,80);

}
double GripperFeedback() {
  // have specific feedback for the gripper joint here
  return 0.0;
}
void OutputRotate(){
  // have specific output for the rotate joint here
}

void OutputLower(){
  // have specific output for the lower joint here  
}

void OutputUpper(){
  // have specific output for the upper joint here
}

void OutputGripper(){
  // have specific output for the gripper joint here
}

