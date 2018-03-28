#ifndef Stepper_Motor_h
#define Stepper_Motor_h
#include <AccelStepper.h>
class StepperMotor {
  public:
    StepperMotor(int dir_pin, int step_pin, int steps_per_rotation, int micro_stepping);
    StepperMotor();
    void begin(int dir_pin, int step_pin, int steps_per_rotation , int micro_stepping);

    int _new_setpoint;
    bool _manual;

    void Update();
    void Manual(int new_speed);
    void MappingSteps(int steps_per_rotation, int micro_stepping);
    void SetSetpoint(int new_setpoint);
    void SetMaxSpeed(int rpm);
    void Home();
    void GoHome();
    double GetAngle();
    double GetSpeed();
    
  private:
    const double _percent_angle = 0.00277777777777f; // this is 1/360
    int _dir_pin, _step_pin;
    int _steps_per_rotation,_micro_stepping;
    AccelStepper* stepper;
};
#endif
