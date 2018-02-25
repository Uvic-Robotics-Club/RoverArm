#include <Stepper.h>
#include "PID_v1.h"
#include "Linear_Actuator.h"
//#include <Servo.h>    

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
int rotateCurrentSteps {0};
bool rotateZeroSet {0};
int rotateLimitState {0};
int stepsToRotate;
// pins 4, 7, 8, 9 for uno and nano
//Stepper rotateStepper(stepsPerRevolution, 4, 7);
//Servo myservo;

//Lower Joint linear actuator Settings
// pins 10 and 11 for uno and nano
#define LOWER_PWM 10
#define LOWER_DIR 11

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
LinearActuator lower(LOWER_PWM,LOWER_DIR,LOWERFEEDBACKPIN,LOWER);
double lowerInput, lowerOutput, lowerSetpoint;
PID lowerPID(&lowerInput, &lowerOutput, &lowerSetpoint, lower._kp, lower._ki, lower._kd, P_ON_E, REVERSE);

#define UPPERFEEDBACKPIN A1
LinearActuator upper(UPPER_PWM,UPPER_DIR,UPPERFEEDBACKPIN,UPPER);
double upperInput, upperOutput, upperSetpoint;
PID upperPID(&upperInput, &upperOutput, &upperSetpoint, upper._kp, upper._ki, upper._kd, P_ON_E,REVERSE);

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
  pinMode(7, OUTPUT);
  pinMode(4, OUTPUT);

  //Lower Linear Actuator Setup
  // pins setup inside the class
  lower.SetOutputLimits(-255,255);
  lower._upper_limit = 360;
  lower._lower_limit = 45;
  lowerPID.SetOutputLimits(-255,255);
  lowerPID.SetMode(1);

  //Upper Linear Actuator Setup
  upper.SetOutputLimits(-255,255);
  upper._lower_limit = 0;
  upper._upper_limit = 135;
  upperPID.SetOutputLimits(-255,255);
  upperPID.SetMode(1);
  //myservo.attach(9);


}

void loop() {
  lower.Update();
  upper.Update();

  if (RotateSetpoint > 0) {
    // step 1/800 of a revolution:
    //rotateStepper.step(5);
    //RotateSetpoint=-1;
    digitalWrite(7, LOW);
    digitalWrite(4, LOW);
    delay(10);
    digitalWrite(7, HIGH);
    
  }
  else   if (RotateSetpoint < 0) {
    // step 1/800 of a revolution:
    //rotateStepper.step(-5);
    //RotateSetpoint=+1;
    digitalWrite(7, LOW);
    digitalWrite(4, HIGH);
    delay(10);
    digitalWrite(7, HIGH);
  }
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
  Serial.print(upper._output);
  Serial.print(",");
  Serial.println(lower._output);
  
  delay(50);
}

void serialEvent() {
  static int temp = 0;

  int mode, value;
  String mode_string, value_string;
  while(Serial.available()>4){
    if(Serial.find("M:")){
      mode = Serial.parseInt();
      if(Serial.find("V:")){
        value = Serial.parseInt();
      }
      else{
        return;
      }
    }
    else{
      return;
    }

    if(true or Serial.read() == '\n'){
      LastMessageReceived = millis();
      switch (mode) {
      case 0:
        RotateMode = VELOCITY;
        RotateSetpoint = abs(value)>150 ? value: 0;
        RotateOutput = RotateSetpoint;
        rotate.SetMode(MANUAL);
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
        // this might cause an issue trying to left shift a char
        RotateMode = POSITION;
        RotateSetpoint = map(value, -255, 255, -360, 360);
        rotate.SetMode(AUTOMATIC);
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
}

void UpdateRotate() {
  static double LastValue = 0;
  static double LatestValue = 0;
  LatestValue = RotateFeedback();
  if (RotateMode == VELOCITY) {
  }
  else if (RotateMode == POSITION) {
    RotateInput = LatestValue;
    //rotate.Compute();
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
    //rotateStepper.step(-1);
    rotateLimitState = digitalRead(rotateLimitPin);
    if (rotateLimitState == LOW) {
      delay(100); //delay to reducce jerk of stepper. Smooth velocity curve would be better, or lost step detection.
      //rotateStepper.step(400);
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
  //rotateStepper.step(stepsToRotate);
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






