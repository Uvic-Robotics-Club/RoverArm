#include "PID_v1.h"
#include <Stepper.h>

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
#define LOWEREXTEND 10
#define LOWERRETRACT 11

//Upper Joint linear actuator Settings
// pins 5 and 6 for uno and nano
#define UPPEREXTEND 5
#define UPPERRETRACT 6



double LastMessageReceived = 0;

double RotateSetpoint, RotateInput, RotateOutput;
double RotateKp = 1, RotateKi = 1, RotateKd = 0;
int RotateMode = VELOCITY;
PID rotate(&RotateInput, &RotateOutput, &RotateSetpoint, RotateKp, RotateKi, RotateKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

#define LOWERFEEDBACKPIN A0
double LowerSetpoint, LowerInput, LowerOutput;
double LowerKp = 0.1, LowerKi = 1, LowerKd = 0;
int LowerMode = VELOCITY;
PID lower(&LowerInput, &LowerOutput, &LowerSetpoint, LowerKp, LowerKi, LowerKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

#define UPPERFEEDBACKPIN A1
double UpperSetpoint, UpperInput, UpperOutput;
double UpperKp = 0.1, UpperKi = 1, UpperKd = 0;
int UpperMode = VELOCITY;
PID upper(&UpperInput, &UpperOutput, &UpperSetpoint, UpperKp, UpperKi, UpperKd, P_ON_E, DIRECT); //P_ON_E (Proportional on Error) is the default behavior

double GripperSetpoint, GripperInput, GripperOutput;
double GripperKp = 0.1, GripperKi = 1, GripperKd = 0;
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
  // Serial Setup
  Serial.begin(115200);

  //Rotate Stepper Setup
  pinMode(rotateLimitPin, INPUT);
  rotateStepper.setSpeed(60);

  //Lower Linear Actuator Setup
  pinMode(LOWEREXTEND, OUTPUT);
  pinMode(LOWERRETRACT, OUTPUT);
  lower.SetOutputLimits(-255, 255);

  //Upper Linear Actuator Setup
  pinMode(UPPEREXTEND, OUTPUT);
  pinMode(UPPERRETRACT, OUTPUT);
  upper.SetOutputLimits(-255,255);


}

void loop() {
  //UpdateRotate();
  //UpdateLower();
  //UpdateUpper();
  //UpdateGripper();
}

void serialEvent() {
  static int temp = 0;

  int mode, value;
  String mode_string, value_string;

  //adding the > 3 made data more reliable. Is it possible to add into the serialEvent function?
  if (Serial.available() >= 8) {
    wholeThing();
    if(inputString.charAt(0) == '[' and inputString.endsWith("]")){
      int comma = inputString.indexOf(',');
      mode_string = inputString.substring(1, comma);
      mode = mode_string.toInt();
    //mode = inputString.toInt();
      value_string = inputString.substring(comma+1,inputString.length()-1);
      value = value_string.toInt();
      //Serial.println(inputString);
      //Serial.print("Mode String: ");
      //Serial.print(mode_string);
      //Serial.print(" | Value String: ");
      //Serial.println(value_string);
      
      
      
    }
    else{
      return;
    }
      
    //wholeThing();
    //value = inputString.toInt();
    LastMessageReceived = millis();
    char buffer[30];
    sprintf(buffer, "M:%i | V: %i ", mode ,value);
    Serial.flush();
    Serial.println(buffer);
    Serial.flush();
  } 
  else {
    return;
  }
  //mode -= '0';

  

  switch (mode) {
  case 0:
    RotateMode = VELOCITY;
    RotateSetpoint = inputString.toInt();
    RotateOutput = RotateSetpoint;
    rotate.SetMode(MANUAL);
    break;
  case 1:
    LowerMode = VELOCITY;
    LowerSetpoint = value;
    Serial.print("I AM IN MODE 1 and Value is ");
    Serial.println(value);
    if(LowerSetpoint>0){
      analogWrite(LOWEREXTEND,abs(LowerSetpoint));
      analogWrite(LOWERRETRACT, 0);
    }
    else{
      analogWrite(LOWERRETRACT,abs(LowerSetpoint));
      analogWrite(LOWEREXTEND, 0);
    }
    LowerOutput = LowerSetpoint;
    lower.SetMode(MANUAL);
    break;
  case 2:
    UpperMode = VELOCITY;
    UpperSetpoint = value; //[upper-,upper+]
    Serial.print("I AM IN MODE 2 and Value is ");
    Serial.println(value);
    if(UpperSetpoint>0){
      analogWrite(UPPEREXTEND,abs(UpperSetpoint));
      analogWrite(UPPERRETRACT, 0);
    }
    else{
      analogWrite(UPPERRETRACT,abs(UpperSetpoint));
      analogWrite(UPPEREXTEND, 0);
    }
    UpperOutput = UpperSetpoint;
    upper.SetMode(MANUAL);
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
    LowerMode = POSITION;
    //temp = first<<8;
    //temp = temp + second;
    LowerSetpoint = map(inputString.toInt(), 0, 65536, -360, 360);
    lower.SetMode(AUTOMATIC);
    break;
  case 6:
    UpperMode = POSITION;
    //temp = first<<8;
    //temp = temp + second;
    UpperSetpoint = map(inputString.toInt(), 0, 65536, -360, 360);
    upper.SetMode(AUTOMATIC);
    break;
  };
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

void UpdateLower() {
  static double LastValue = 0;
  static double LatestValue = 0;
  LatestValue = LowerFeedback();
  if (LowerMode == VELOCITY) {
  }
  else if (LowerMode == POSITION) {
    LowerInput = LatestValue;
    lower.Compute();
  }
  
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

double LowerFeedback() {
  /*
     0    Lowerbound on ADC input
   1024 Upperbound on ADC input
   15   Lowerbound on Lower Angle
   50   Upperbound on Lower Angle
   */
  return map(analogRead(LOWERFEEDBACKPIN), 0, 1024, 15, 50);
}

double UpperFeedback() {
  /*
     0    Lowerbound on ADC input
   1024 Upperbound on ADC input
   30   Lowerbound on Lower Angle
   80   Upperbound on Lower Angle
   */
  return map(analogRead(UPPERFEEDBACKPIN), 0, 1024, 30, 80);

}
double GripperFeedback() {
  // have specific feedback for the gripper joint here
  return 0.0;
}
void OutputRotate() {
  //rotates stepper number of steps found in feedback
  rotateStepper.step(stepsToRotate);
}

void OutputLower() {
  //checks pid for direction and power and sends to H-Bridge. May need to adjust bounds on PID depending on controller.
  //If not working, check pins for both feedback and power to linear actuator are correct first.
  if (LowerOutput < 0) {
    analogWrite(LOWERRETRACT, LowerOutput);
  } 
  else {
    analogWrite(LOWEREXTEND, LowerOutput);
  }

}

void OutputUpper() {
  //Checks pid for direction and power and sends to H-Bridge. May need to adjust bounds on PID depending on controller.
  //If not working, check pins for both feedback and power to linear actuator are correct first.
  if (UpperOutput < 0) {
    analogWrite(UPPERRETRACT, UpperOutput);
  } 
  else {
    analogWrite(UPPEREXTEND, UpperOutput);
  }
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

