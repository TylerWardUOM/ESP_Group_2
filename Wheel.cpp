#include "Wheel.h"

Wheel::Wheel(PinName motorBipolar, float motorMultiplier, PinName motorPwm, PinName motorEnable, 
             PinName encoderA, PinName encoderB, float wheelDiameter, int encoderResolution, int maxRpm) 
    : motor(motorBipolar, motorMultiplier, motorPwm, motorEnable), 
      encoder(encoderA, encoderB, wheelDiameter, encoderResolution),
      target_rpm(0), max_rpm(maxRpm) {}

void Wheel::setSpeed(int rpm) {
    if (rpm > max_rpm) rpm = max_rpm;
    if (rpm < -max_rpm) rpm = -max_rpm;

    target_rpm = rpm;

    float normalizedSpeed = static_cast<float>(rpm) / max_rpm;
    //Debug Print
    //printf("RPM = %d",rpm);
    printf("normalized speed = %.2f\n", normalizedSpeed);

    motor.setSpeed(normalizedSpeed);
}

void Wheel::regulateSpeed() {
    int actual_rpm = encoder.getSpeed();
    int error = target_rpm - actual_rpm;

    float adjustment = error * 0.1f;  // Tuning factor
    float newSpeed = (motor.getSpeed() + adjustment);

    if (newSpeed > 1.0f) newSpeed = 1.0f;
    if (newSpeed < -1.0f) newSpeed = -1.0f;

    motor.setSpeed(newSpeed);
    //Debug Print
    //printf("Target: %d, Actual: %d, Adjusted Speed: %.2f\n", target_rpm, actual_rpm, newSpeed);
}

void Wheel::startRegulation(float interval) {
    ticker.attach(callback(this, &Wheel::regulateSpeed), interval);
}

void Wheel::stopRegulation() {
    ticker.detach();
}

void Wheel::stop() {
    target_rpm = 0;  // Ensure target speed is reset
    motor.setSpeed(0.0f);  // Stop the motor
}