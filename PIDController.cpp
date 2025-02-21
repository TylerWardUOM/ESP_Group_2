#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float scaling_multiplier)
        : kp(kp), ki(ki), kd(kd), integral(0.0f), _scaling_multiplier(scaling_multiplier), prevError(0.0f) {}

float PIDController::update(float error, float dt) {
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    return (kp * error + ki * integral + kd * derivative)*_scaling_multiplier;
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
}
