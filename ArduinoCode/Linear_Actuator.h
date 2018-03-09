#ifndef Linear_Actuator_h
#define Linear_Actuator_h
#include "PID_v1.h"
#include "Arduino.h"

class LinearActuator {
  public:
#define MANUAL 0
#define AUTO 1
#define VELOCITY 1
#define POSITION 0

    // THIS IS IN ORDER IN THE CPP CLASS
    LinearActuator(int pwm_pin, int dir_pin, int feedback_pin);
    LinearActuator();
    void begin(int pwm_pin, int dir_pin, int feedback_pin);
    void Manual(double speed);
    void Update();
    void SetSetpoint(int new_setpoint);
    void EnablePID();
    void DisablePID();
    // set output limits but this is private
    bool PidEnabled();
    void Feedback();
    void Mapping(double low_raw, double high_raw, double low_angle, double high_angle);
    int Output();
    void SetSoftLimits(int lower_bound, int upper_bound);
    void SetPIDGains(double new_kp, double new_ki, double new_kd);
    double GetPGain();
    double GetIGain();
    double GetDGain();
    double GetInput();
    double GetAngle();
    double GetRawInput();
    double GetSetpoint();
    double GetOutput();

  private:
    void SetOutputLimits(int lower_bound, int upper_bound);
    int _pwm_pin, _dir_pin, _feed_pin; // Pins that we use
    double _setpoint, _input, _output; // used for PID
    double _raw_input; // what analog read feeds into
    double _kp, _ki, _kd; // used for PID
    double _lower_soft_limit, _upper_soft_limit; // used for soft limits
    int _lower_absolute_limit, _upper_absolute_limit; // used for mapping voltage into angle
    int _lower_actual_angle, _upper_actual_angle; // used for mapping voltage into angle
    PID* _pid; // is the PID
};


#endif
