#include "Arduino.h"
#include "pid.h"

diffbot::PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd)
{
    prev_error_ = 0;
}

double diffbot::PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    // Compute the time difference since the last loop iteration
    time_t  current_time = time (NULL);
    dt_ = difftime (last_time_, current_time);

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    proportional_ = error;
    integral_ += error * dt_;
    derivative_ = (error - prev_error_) / dt_;

    if(setpoint == 0 && error == 0)
    {
        integral_ = 0;
        //SBR:
        return 0;
    }

    pid = (kp_ * proportional_); // + (ki_ * integral_) + (kd_ * derivative_);

    prev_error_ = error;
    last_time_ = time (NULL);

    return constrain(pid, min_val_, max_val_);
}

void diffbot::PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}