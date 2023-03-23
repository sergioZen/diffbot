/***************************************************************
   Motor driver function definitions - by James Nugen
*************************************************************/

#define RIGHT_MOTOR_BACKWARD 5
#define LEFT_MOTOR_BACKWARD  6
#define RIGHT_MOTOR_FORWARD  9
#define LEFT_MOTOR_FORWARD   10
#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_ENABLE 13

#include <Wire.h>

#include "diffbot_base_config.h"

#include "L298N_MotorShield.h"

#include <motor_controller_interface.h>

namespace diffbot {

    /** \brief Implementation of the MotorControllerIntf for the Adafruit_MotorShield
     * 
     * Implements the abstract setSpeed method from the MotorControllerIntf
     * 
     * The class makes use of the adafruit-stepper-dc-motor-featherwing library.
     * 
     * \note for more details see
     * https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/library-reference
     */
    class L298NMotorController : public MotorControllerIntf<L298N_MotorShield>
    {
        public:
            /** \brief Construct an \ref AdafruitMotorController for a single motor
             * 
             * Specify the motor to control \p motor_num (use 3 or 4 for diffbot).
             * Specify the i2c address to be used. The default address is 0x60 but it
             * can be changed by soldering different address switches on the bottom of
             * the PCB.
             * 
             * \param motor_num Number of the motor to control (one of 1, 2, 3, 4). Diffbot uses motors 3 and 4.
             * \param addr i2c address used to communicate with the motor driver.
             */
            L298NMotorController(uint8_t motor_num, uint8_t addr=0x60);

            /** \brief Initializes the communication with the motor driver
             * 
             * must be called in setup() to initialize the shield. 
             * An optional frequency parameter can be used to specify something other 
             * than the default maximum: 1.6KHz PWM frequency.
             */
            void begin(uint16_t freq = 1600);

            /** \brief Set the speed of the motor \ref pMotor_.
             * 
             * Concrete implementation of the setSpeed interface method
             * to control a single motor connected to the Adafruit_MotorShield.
             * The input parameter \p value ranges from -255 to 255.
             * Inside this method the value is mapped between 0 to 255 and 
             * the sign is used to set the direction.
             * A positive \p value results in the motor spinning forward.
             * Negative \p value rotates the motor backward and
             * a zero value stops the motor (releases the current).
             * 
             * \note for more details see
             * https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/library-reference
             * 
             * \param value positive or negative value to set the direction
             * and speed of the motor.
             */
            void setSpeed(int value) override;
            

        private:
            // Pointer to the motor that is controlled by this motor controller.
            L298N_DCMotor *pMotor_;
    };

}
