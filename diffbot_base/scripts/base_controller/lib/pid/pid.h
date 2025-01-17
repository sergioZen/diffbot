#pragma once
#ifndef PID_H
#define PID_H

#include <ros.h>

using namespace std;

namespace diffbot {

    class PID
    {
    public:
        PID(ros::NodeHandle& nh, float min_val, float max_val, float kp, float ki, float kd, bool debug);
        double compute(float setpoint, float measured_value);
        void updateConstants(float kp, float ki, float kd);

        inline double proportional() { return proportional_; };
        inline double integral() { return integral_; };
        inline double derivative() { return derivative_; };
        inline double prev_error() { return prev_error_; };

        inline int last_time(){ return last_time_; }; 
        inline double dt(){ return dt_; }; 

    private:
        // ROS node handle, which provides the handler for logging
        ros::NodeHandle& nh_;

        float min_val_;
        float max_val_;
        float kp_;
        float ki_;
        float kd_;
        bool debug_;
        double proportional_;
        double integral_;
        double derivative_;
        double prev_error_;

        elapsedMillis sinceMillis;
        int last_time_;
        double dt_;
    };
}

#endif
