#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * @file PIDController.h
 * @class PIDController
 * @brief Implements a simple PID controller.
 *
 * This class provides a basic PID control algorithm.
 */
class PIDController {
public:
    /**
     * @brief Constructs a PID controller with given gains.
     * 
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param scaling_multiplier A scaling factor for the output (default is 1).
     */
    PIDController(float kp, float ki, float kd, float scaling_multiplier = 1);

    /**
     * @brief Computes the PID control output.
     * 
     * This function updates the PID control loop using the current error and time step.
     * 
     * @param error The current error (setpoint - measured value).
     * @param dt Time step in seconds.
     * @return The computed control output.
     */
    float update(float error, float dt);

    /**
     * @brief Resets the internal state of the PID controller.
     *
     * This function clears the integral term and previous error, 
     * which is useful when restarting the control loop.
     */
    void reset();

    // Getters
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    float getScalingMultiplier() const { return _scaling_multiplier; }

    // Setters
    void setKp(float newKp) { kp = newKp; }
    void setKi(float newKi) { ki = newKi; }
    void setKd(float newKd) { kd = newKd; }
    void setScalingMultiplier(float newScalingMultiplier) { _scaling_multiplier = newScalingMultiplier; }

private:
    float _scaling_multiplier; ///< Scaling factor for the output.
    float kp; ///< Proportional gain.
    float ki; ///< Integral gain.
    float kd; ///< Derivative gain.
    float integral; ///< Accumulated integral error.
    float prevError; ///< Previous error value for derivative calculation.
};

#endif // PID_CONTROLLER_H
