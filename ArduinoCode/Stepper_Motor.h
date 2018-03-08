#ifndef Stepper_Motor_h
#define Stepper_Motor_h
#include <AccelStepper.h>
class StepperMotor {
  public:
    StepperMotor(int dir_pin, int step_pin, int steps_per_rotation);
    StepperMotor();
    void begin(int dir_pin, int step_pin, int steps_per_rotation);


    int _new_setpoint;
    bool _manual;

    void Update();
    void Manual(int new_speed);
    void SetSetpoint(int new_setpoint);

    void Home();
    void GoHome();

    double GetAngle();
    double GetSpeed();


    AccelStepper* stepper;
  private:
    const double _percent_angle = 0.00277777777777f;
    double _percent_step;
    int _dir_pin, _step_pin;
    int _steps_per_rotation;
};
#endif
