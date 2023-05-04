#include "pid.h"

#include<chrono> 
#include<thread>

using namespace std;

diffbot::PID::PID(ros::NodeHandle& nh, float min_val, float max_val, float kp, float ki, float kd, bool debug):
    nh_(nh),
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    debug_(debug)
{
    prev_error_ = 0;
    last_time_ = sinceMillis;
}

double diffbot::PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    kp_ = 10.0;
    ki_ = 0.9;
    kd_ = 3.0;

    int current_time = sinceMillis;

    // Compute the time difference since the last loop iteration
    dt_ = (current_time - last_time_) / (double)1000;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    proportional_ = error;
    integral_ += error;
    derivative_ = (error - prev_error_);

    if (setpoint == 0 && error == 0)
    {
        integral_ = 0;
    }


    double vkp = (kp_ * proportional_);
    double vki = (ki_ * integral_) / dt_;
    double vkd = (kd_ * derivative_) * dt_;

    pid = vkp + vki + vkd;
    prev_error_ = error;
    last_time_ = sinceMillis;

    if (debug_)
    {
        String log_msg =
                String("\nPID dt_: <") + String(dt_) 
                + String("> current_time: <") + String(current_time) 
                + String("> last_time_: <") + String(last_time_) 
                + String("> error: <") + String(error)
                + String("> integral_: <") + String(integral_) 
                + String("> derivative_: ") + String(derivative_);        
        nh_.loginfo(log_msg.c_str()); 
        
        log_msg =
            String("\nPID vkp: <") + String(vkp) 
            + String("> vki: <") + String(vki)
            + String("> vkd: <") + String(vkd) 
            + String("> pid: ") + String(pid);  
        nh_.loginfo(log_msg.c_str()); 
    }

    return pid;
}

void diffbot::PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}