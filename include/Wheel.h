#ifndef WHEEL_H
#define WHEEL_H

/**
 * @file Wheel.h
 * @brief Defines the Wheel class, which handles motor control and speed regulation.
 */

#include "mbed.h"
#include "Motor.h"
#include "MotorUnipolar.h"
#include "Encoder.h"
#include "Bluetooth.h"

/**
 * @class Wheel
 * @brief Represents a wheel with motor control and speed regulation.
 */
class Wheel {
public:
    /**
     * @brief Constructs a Wheel object.
     * @param motorBipolar Pin for motor bipolar control.
     * @param motorMultiplier Speed multiplier for the motor.
     * @param motorPwm PWM pin for motor speed control.
     * @param motorEnable Pin to enable or disable the motor.
     * @param encoderA Encoder channel A pin.
     * @param encoderB Encoder channel B pin.
     * @param wheelDiameter Diameter of the wheel in meters.
     * @param encoderResolution Number of pulses per revolution.
     * @param maxRpm Maximum RPM of the motor.
     * @param bt Reference to a Bluetooth module for debugging and communication.
     * @param motorDirection Pin for motor direction.
     */
    Wheel(PinName motorBipolar, float motorMultiplier, PinName motorPwm, PinName motorEnable, 
          PinName encoderA, PinName encoderB, float wheelDiameter, int encoderResolution, int maxRpm,
          Bluetooth &bt, PinName motorDirection = NC); // Added the useUnipolar parameter

    ~Wheel();  // Destructor to clean up dynamically allocated motor

    /**
     * @brief Sets the target speed of the wheel.
     * @param rpm Desired speed in revolutions per minute (RPM).
     */
    void setSpeed(int rpm);

    /**
     * @brief Regulates the wheel speed using proportional control.
     */
    void regulateSpeed();

    /**
     * @brief Regulates the wheel speed and logs debug information.
     */
    void regulateSpeedDebug();
  
  
    void regulateSpeedwithBOOST();

    /**
     * @brief Starts automatic speed regulation at a fixed interval.
     * @param interval Time interval (in seconds) for speed regulation updates.
     */
    void startRegulation(float interval);

    /**
     * @brief Stops the automatic speed regulation.
     */
    void stopRegulation();

    /**
     * @brief Enables the motor.
     */
    void enableMotor();

    /**
     * @brief Logs wheel data for debugging purposes via Bluetooth.
     */
    void debugWheelData(int side);

    /**
     * @brief Sends live wheel data for debugging purposes via Bluetooth.
     */
    void live_debugWheelData(int side);

    /**
     * @brief Disables the motor.
     */
    void disableMotor();

    /**
     * @brief Sets the proportional gain (Kp) for speed control.
     * @param kp Proportional gain value.
     */
    void setKp(float kp);

    /**
     * @brief Sets the intergral gain (Ki) for speed control.
     * @param ki Intergral gain value.
     */
    void setKi(float ki);
  
  
    void setThreshold(float new_threshold);
  
    void setBoost(float new_boost);

    void setMaxRpm(int MaxRpm);

    /**
     * @brief Stops the wheel by setting speed to zero.
     */
    void stop();

    /// Motor control instance
    Motor* motor;

    /// Encoder instance for measuring wheel speed and distance
    Encoder encoder;

private:
    int target_rpm; ///< Target speed in RPM
    int set_rpm; ///< Target speed in RPM Taking into account error history
    int max_rpm; ///< Maximum allowable speed in RPM
    float Kp; ///< Proportional gain for speed regulation
    float Ki; ///< Integral gain for speed regulation
    float boost;
    float threshold;
    float actual_rpm;  ///< Measured RPM from encoder
    float error;       ///< Difference between target and actual RPM
    float adjustment;  ///< P correction value
    Ticker ticker; ///< Timer for automatic speed regulation
    Bluetooth &_bt;  ///< Reference to the Bluetooth module for debugging.
    float correction_history[5];  // Store the last 5 Correction values
    float error_history[5];
    int correction_index;       // Circular buffer index
    float persistent_correction;
    float persistent_error;
    float newSpeed;
    float originalSpeed;

};

#endif // WHEEL_H
