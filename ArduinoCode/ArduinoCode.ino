#include <PID_v1.h>
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


//Stepper motor and corresponding limit switch Settings
#define rotateLimitPin 2
#define stepsPerRevolution 800
int rotateCurrentSteps {0};
bool rotateZeroSet {0};
int rotateLimitState {0};
int stepsToRotate;
Stepper rotateStepper(stepsPerRevolution, 9, 10, 11, 12);

//Lower Joint linear actuator Settings
#define LOWEREXTEND 7
#define LOWERRETRACT 8

//Upper Joint linear actuator Settings
#define UPPEREXTEND 5
#define UPPERRETRACT 6



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
  // Serial Setup
  Serial.begin(9600);

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
  UpdateRotate();
  UpdateLower();
  UpdateUpper();
  UpdateGripper();
}

void serialEvent() {

  unsigned char mode, first, second, third;

  //adding the > 3 made data more reliable. Is it possible to add into the serialEvent function?
  if (Serial.available() > 3) {
    mode = Serial.read();
    first  = Serial.read();
    second  = Serial.read();
    third  = Serial.read();
    LastMessageReceived = millis();
  } else return;

  switch (mode) {
    case 0:
      RotateMode = VELOCITY;
      RotateSetpoint = -1 * first + second; //[rotate-,rotate+]
      RotateOutput = RotateSetpoint;
      rotate.SetMode(MANUAL);
      break;
    case 1:
      LowerMode = VELOCITY;
      LowerSetpoint = -1 * first + second; //[lower-,lower+]
      LowerOutput = LowerSetpoint;
      lower.SetMode(MANUAL);
      break;
    case 2:
      UpperMode = VELOCITY;
      UpperSetpoint = -1 * first + second; //[upper-,upper+]
      UpperOutput = UpperSetpoint;
      upper.SetMode(MANUAL);
      break;
    case 3:
      GripperMode = VELOCITY;
      GripperSetpoint = -1 * first + second; //[gripper-,gripper+]
      GripperOutput = GripperSetpoint;
      gripper.SetMode(MANUAL);
      break;
    case 4:
      // this might cause an issue trying to left shift a char
      RotateMode = POSITION;
      RotateSetpoint = (double)map((uint16_t)first << 8 | (uint16_t)second, 0, 65536, -360, 360);
      rotate.SetMode(AUTOMATIC);
      break;
    case 5:
      LowerMode = POSITION;
      LowerSetpoint = (double)map((uint16_t)first << 8 | (uint16_t)second, 0, 65536, -360, 360);
      lower.SetMode(AUTOMATIC);
      break;
    case 6:
      UpperMode = POSITION;
      UpperSetpoint = first;//map(first << 4 + second, 0, 65535, -180, 180);
      upper.SetMode(AUTOMATIC);
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
  LatestValue = LowerFeedback();
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

  } else {
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
  } else {
    analogWrite(LOWEREXTEND, LowerOutput);
  }

}

void OutputUpper() {
    //Checks pid for direction and power and sends to H-Bridge. May need to adjust bounds on PID depending on controller.
    //If not working, check pins for both feedback and power to linear actuator are correct first.
  if (UpperOutput < 0) {
    analogWrite(UPPERRETRACT, UpperOutput);
  } else {
    analogWrite(UPPEREXTEND, UpperOutput);
  }
}

void OutputGripper() {
  // have specific output for the gripper joint here
}
