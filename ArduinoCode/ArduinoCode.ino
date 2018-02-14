#include <Stepper.h>
#include "PID_v1.h"
#include "Linear_Actuator.h"

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

String inputString = "";


//Stepper motor and corresponding limit switch Settings
#define rotateLimitPin 2
#define stepsPerRevolution 800
int rotateCurrentSteps {
  0};
bool rotateZeroSet {
  0};
int rotateLimitState {
  0};
int stepsToRotate;
// pins 4, 7, 8, 9 for uno and nano
Stepper rotateStepper(stepsPerRevolution, 4, 7, 8, 9);

//Lower Joint linear actuator Settings
// pins 10 and 11 for uno and nano
#define LOWER_PWM 11
#define LOWER_DIR 3

//Upper Joint linear actuator Settings
// pins 5 and 6 for uno and nano
#define UPPER_PWM 5
#define UPPER_DIR 6



double LastMessageReceived = 0;

double RotateSetpoint, RotateInput, RotateOutput;
double RotateKp = 1, RotateKi = 1, RotateKd = 0;
int RotateMode = VELOCITY;
PID rotate(&RotateInput, &RotateOutput, &RotateSetpoint, RotateKp, RotateKi, RotateKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

#define LOWERFEEDBACKPIN A0
LinearActuator lower(LOWER_PWM,LOWER_DIR,LOWERFEEDBACKPIN);

#define UPPERFEEDBACKPIN A1
LinearActuator upper(UPPER_PWM,UPPER_DIR,UPPERFEEDBACKPIN);

double GripperSetpoint, GripperInput, GripperOutput;
double GripperKp = 0.1, GripperKi = 1, GripperKd = 0;
int GripperMode = VELOCITY;
PID gripper(&GripperInput, &GripperOutput, &GripperSetpoint, GripperKp, GripperKi, GripperKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

void UpdateRotate();
void UpdateGripper();
double RotateFeedback();
double GripperFeedback();
void OutputRotate();
void OutputGripper();


void setup() {
  // Serial Setup
  Serial.begin(115200);

  //Rotate Stepper Setup
  pinMode(rotateLimitPin, INPUT);
  rotateStepper.setSpeed(60);

  //Lower Linear Actuator Setup
  // pins setup inside the class
  lower.SetOutputLimits(-255,255);

  //Upper Linear Actuator Setup
  upper.SetOutputLimits(-255,255);


}

void loop() {
  //UpdateRotate();
  //UpdateLower();
  //UpdateUpper();
  //UpdateGripper();
  //lower.Update(); // this is empty right now
  //upper.Update(); // this is empty right now
  delay(50);
}

void serialEvent() {
  static int temp = 0;

  int mode, value;
  String mode_string, value_string;

  if(Serial.find("M:")){

    mode = Serial.parseInt();
    if(Serial.find("V:")){
      value = Serial.parseInt();
    }
    else{return;}
  }else{return;}

  if(true or Serial.read() == '\n'){


    LastMessageReceived = millis();

    char buffer[30];
    sprintf(buffer, "M:%i | V: %i ", mode ,value);
    /*
     Serial.print("M: ");
     Serial.print(mode_string);
     Serial.print(" V: ");
     Serial.print(value_string);
     */
    Serial.println(buffer);
    Serial.flush();


    switch (mode) {
    case 0:
      RotateMode = VELOCITY;
      RotateSetpoint = inputString.toInt();
      RotateOutput = RotateSetpoint;
      rotate.SetMode(MANUAL);
      break;
    case 1:
      lower.manual(value);
      break;
    case 2:
      //Serial.print("In case 2, and value 2 is ");
      //Serial.println(value);
      upper.manual(value);
      break;
    case 3:
      GripperMode = VELOCITY;
      GripperSetpoint = inputString.toInt(); //[gripper-,gripper+]
      GripperOutput = GripperSetpoint;
      gripper.SetMode(MANUAL);
      break;
    case 4:
      // this might cause an issue trying to left shift a char
      RotateMode = POSITION;
      //temp = first<<8;
      //temp = temp + second;
      RotateSetpoint = map(inputString.toInt(), 0, 65536, -360, 360);
      rotate.SetMode(AUTOMATIC);
      break;
    case 5:
      lower.EnablePID();
      lower.SetSetpoint(value);
      break;
    case 6:
      upper.EnablePID();
      upper.SetSetpoint(value);
      break;
    };
  }
}

void UpdateRotate() {
  static double LastValue = 0;
  static double LatestValue = 0;
  LatestValue = RotateFeedback();
  if (RotateMode == VELOCITY) {
  }
  else if (RotateMode == POSITION) {
    RotateInput = LatestValue;
    rotate.Compute();
  }

  OutputRotate();
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
  //rotates negative until it hits the limit switch, then roates to 0 and sets zero
  //had some werid behaviour trying to set zero at -180.
  static double current;
  if (!rotateZeroSet) {
    rotateStepper.step(-1);
    rotateLimitState = digitalRead(rotateLimitPin);
    if (rotateLimitState == LOW) {
      delay(100); //delay to reducce jerk of stepper. Smooth velocity curve would be better, or lost step detection.
      rotateStepper.step(400);
      rotateCurrentSteps = 0;
      rotateZeroSet = 1;
    }

  }
  else {
    stepsToRotate = RotateSetpoint * stepsPerRevolution / 360 - rotateCurrentSteps;
    rotateCurrentSteps = rotateCurrentSteps + stepsToRotate;
  }

  current = rotateCurrentSteps / 800 * 360;
  return current;
}




double GripperFeedback() {
  // have specific feedback for the gripper joint here
  return 0.0;
}
void OutputRotate() {
  //rotates stepper number of steps found in feedback
  rotateStepper.step(stepsToRotate);
}


void OutputGripper() {
  // have specific output for the gripper joint here
}

void wholeThing() {
  inputString = "";
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    if (inChar == '\n'){
      return;
    }
    // add it to the inputString:
    inputString += inChar;
  }
}




