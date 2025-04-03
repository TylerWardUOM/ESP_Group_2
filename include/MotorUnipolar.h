#ifndef MOTOR_H
#define MOTOR_H

/**
 * @file Motor.h
 * @brief Defines the Motor class for controlling a DC motor using PWM.
 */

#include "mbed.h"

/**
 * @class Motor
 * @brief Controls a DC motor using a bipolar signal and PWM.
 */
class Motor {
public:
    /**
     * @brief Constructs a Motor object.
     * @param direction Pin for setting motor direction.
     * @param multiplier Scaling factor for speed control.
     * @param pwm Pin for PWM speed control.
     * @param enable Pin to enable or disable the motor.
     */
    Motor(PinName direction, float multiplier, PinName pwm, PinName enable);

    /**
     * @brief Sets the motor speed.
     * @param speed Speed value ranging from -1.0 (full reverse) to 1.0 (full forward).
     */
    void setSpeed(float speed);

    /**
     * @brief Stops the motor by setting speed to zero.
     */
    void stop();

    /**
     * @brief Disables the motor, cutting off power.
     */
    void disable();

    /**
     * @brief Enables the motor for operation.
     */
    void enable();

    /**
     * @brief Retrieves the current motor speed.
     * @return The speed value (-1.0 to 1.0).
     */
    float getSpeed();

private:
    DigitalOut _direction; ///< Controls motor direction.
    float _multiplier; ///< Multiplier for speed adjustment.
    PwmOut _pwm; ///< PWM output for speed control.
    DigitalOut _enable; ///< Digital output to enable/disable the motor.
    float _speed; ///< Stores the current motor speed.
};

#endif // MOTOR_H
