#include "Wheel.h"

Wheel::Wheel(PinName motorBipolar, float motorMultiplier, PinName motorPwm, PinName motorEnable, 
             PinName encoderA, PinName encoderB, float wheelDiameter, int encoderResolution, int maxRpm,
             Bluetooth& bt) 
    : motor(motorBipolar, motorMultiplier, motorPwm, motorEnable), 
      encoder(encoderA, encoderB, wheelDiameter, encoderResolution),
      target_rpm(0), max_rpm(maxRpm), Kp(0.01), _bt(bt) {}

void Wheel::setSpeed(int rpm) {
    if (rpm > max_rpm) rpm = max_rpm;
    if (rpm < -max_rpm) rpm = -max_rpm;

    target_rpm = rpm;

    float normalizedSpeed = static_cast<float>(rpm) / max_rpm;
    //Debug Print
    //printf("RPM = %d",rpm);
    //printf("normalized speed = %.2f\n", normalizedSpeed);

    motor.setSpeed(normalizedSpeed);
}

void Wheel::setKp(float newKp){
    Kp=newKp;
}

void Wheel::regulateSpeed() {
    actual_rpm = encoder.getSpeed();
    error = target_rpm - actual_rpm;

    adjustment = error/max_rpm * Kp;  // Tuning factor
    float newSpeed = (motor.getSpeed() + adjustment);

    if (newSpeed > 1.0f) newSpeed = 1.0f;
    if (newSpeed < -1.0f) newSpeed = -1.0f;

    motor.setSpeed(newSpeed);
    //Debug Print
    //printf("Target: %d, Actual: %d, Adjusted Speed: %.2f\n", target_rpm, actual_rpm, newSpeed);
}

void Wheel::regulateSpeedDebug() {
    actual_rpm = encoder.getSpeed();
    error = target_rpm - actual_rpm;

    adjustment = error/max_rpm * Kp;  // Tuning factor
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

void Wheel::enableMotor(){
     motor.enable();
}

void Wheel::disableMotor(){
    motor.disable();
}

void Wheel::stop() {
    target_rpm = 0;  // Ensure target speed is reset
    motor.setSpeed(0.0f);  // Stop the motor
}

void Wheel::debugWheelData(int side){
    Bluetooth::MotorDebugData motorData = {side,encoder.getDistance(),actual_rpm,target_rpm,error,adjustment};
    _bt.logDebugData(Bluetooth::MOTOR_DEBUG, &motorData);
}