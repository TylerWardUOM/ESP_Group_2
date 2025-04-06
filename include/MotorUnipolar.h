#ifndef MOTOR_UNIPOLAR_H
#define MOTOR_UNIPOLAR_H

/**
 * @file MotorUnipolar.h
 * @brief Defines the MotorUnipolar class for controlling a DC motor using PWM.
 */

#include "mbed.h"
#include "Motor.h"

/**
 * @class MotorUnipolar
 * @brief Controls a DC motor using a bipolar signal and PWM.
 */
class MotorUnipolar : public Motor {public:
    /**
     * @brief Constructs a Motor object.
     * @param bipolar Pin for setting motor mode.
     * @param direction Pin for setting motor direction.
     * @param multiplier Scaling factor for speed control.
     * @param pwm Pin for PWM speed control.
     * @param enable Pin to enable or disable the motor.
     */
    MotorUnipolar(PinName bipolar, PinName direction, float multiplier, PinName pwm, PinName enable);

    /**
     * @brief Sets the motor speed.
     * @param speed Speed value ranging from -1.0 (full reverse) to 1.0 (full forward).
     */
    void setSpeed(float speed);


    /**
     * @brief Stops the motor by setting speed to zero.
     */
    void stop();
private:
    DigitalOut _direction; ///< Controls motor direction.
};

#endif // MOTOR_UNIPOLAR_H
