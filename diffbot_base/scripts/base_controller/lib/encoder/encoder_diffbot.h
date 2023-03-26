/*
 * Author: Franz Pucher
 */

#ifndef DIFFBOT_ENCODER_H
#define DIFFBOT_ENCODER_H

#define LEFT_ENCODER 1
#define RIGHT_ENCODER 2

#include <Arduino_EncoderShield.h>
#include <ros.h>

namespace diffbot
{
    struct JointState
    {
        double angular_position_;
        double angular_velocity_;
    };

    class Encoder
    {
    public:
        /** \brief Construct a diffbot::Encoder providing access to quadrature encoder ticks and angular joint velocity.
         * 
         * \param nh reference to the main ros::NodeHandle to compute the velocity from time and ticks or angle (s = v * t)
         * \param pin1 Pin of the first Hall effect sensor
         * \param pin2 Pin of the second Hall effect sensor
         * \param encoder_resolution number of tick counts for one full revolution of the wheel (not the motor shaft). Keep track of gear reduction ratio.
         */
        Encoder(ros::NodeHandle& nh, int encoder_resolution, Arduino_EncoderShield encoder_shield);

        /** \brief get revolutions per minute
         *
         * Calculates the wheels revolution per minute using the encoder ticks.
         * 
         * Based on https://github.com/linorobot/linorobot/blob/53bc4abf150632fbc6c28cdfcc207caa0f81d2b1/teensy/firmware/lib/encoder/Encoder.h
         * 
         * \returns revolutions per minute
         */
        int getRPM(int encoder);


        double angularPosition();

        /** \brief Get the measure the angular joint velocity
         *
         * Calculates the angular velocity of the wheel joint using the encoder ticks.
         * 
         * \returns angular wheel joint velocity (rad/s)
         */
        double angularVelocity();


        JointState jointState(int encoder = 0 );

        /** \brief Convert number of encoder ticks to angle in radians 
         *
         * Calculates the current tick count of the encoder to its absolute angle in radians
         * using the \ref encoder_resolution_.
         * 
         * \param ticks tick count from an encoder which is converted to a corresponding absolute angle.
         * 
         * \returns angle corresponding to encoder ticks (rad)
         */
        double ticksToAngle(const int &ticks) const;

        /** \brief Read the current encoder tick count
         * 
         * \returns encoder ticks
         */
        int32_t read(int encoder = 0);

        /** \brief Set the encoder tick count
         * 
         * Mainly used to reset the encoder back to zero.
         * 
         * \param p encoder ticks
         */
        void write(int32_t p);

        /** \brief Setter for encoder resolution
         * 
         * Used to initialize the encoder with a new resolution, e.g. obtained
         * from the ROS parameter server.
         * 
         * \param resolution value to which the encoder tick count shoudl be set
         */
        inline void resolution(int resolution) { encoder_resolution_ = resolution; };

        /** \brief Getter for encoder resolution
         * 
         * Returns the currently set encder resolution.
         */
        inline int resolution() { return encoder_resolution_; };


        JointState joint_state_;

    private:
        // ROS node handle, which provides the current time to compute the angular velocity from the current tick count
        ros::NodeHandle nh_;
        // Number of tick counts for one full revolution of the wheel (not the motor shaft). Keep track of gear reduction ratio.
        int encoder_resolution_;
        // Previous time when the \ref getRPM or \ref angularVelocity method was called to calculated the delta update time.
        ros::Time prev_update_time_;
        // Previous encoder tick count when the \ref getRPM or \ref angularVelocity method was called to calculated the delta tick count.
        int32_t prev_encoder_ticks_;

        uint8_t pin1;
        uint8_t pin2;

        Arduino_EncoderShield encoder_shield_;
    };
}

#endif // DIFFBOT_ENCODER_H