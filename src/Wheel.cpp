#include "Wheel.h"

Wheel::Wheel(PinName motorBipolar, float motorMultiplier, PinName motorPwm, PinName motorEnable, 
             PinName encoderA, PinName encoderB, float wheelDiameter, int encoderResolution, int maxRpm,
             Bluetooth &bt, PinName motorDirection)  // Make motorDirection optional with default value NC
    : encoder(encoderA, encoderB, wheelDiameter, encoderResolution), 
      target_rpm(0), max_rpm(maxRpm), Kp(0.01), _bt(bt) {

    // Dynamically allocate the correct motor type based on the presence of a motorDirection pin
    if (motorDirection != NC) {  // Check if a motorDirection pin was passed
        // Use Unipolar Motor
        motor = new MotorUnipolar(motorBipolar, motorDirection, motorMultiplier, motorPwm, motorEnable);
    } else {
        // Use Bipolar Motor
        motor = new Motor(motorBipolar, motorMultiplier, motorPwm, motorEnable);
    }
}
// Destructor to free dynamically allocated memory
Wheel::~Wheel() {
    delete motor;  // Free the motor object
}

//Only Directly adjust motor if a large change else let regulate handle it
void Wheel::setSpeed(int rpm) {
    if (rpm > max_rpm) rpm = max_rpm;
    if (rpm < -max_rpm) rpm = -max_rpm;

    if (abs(target_rpm-rpm)>30){
        target_rpm = rpm;

        float normalizedSpeed = static_cast<float>(rpm) / max_rpm;
        motor.setSpeed(normalizedSpeed);
    }
    else{
        target_rpm = rpm;
    }
}

void Wheel::setKp(float newKp){
    Kp=newKp;
}

void Wheel::setThreshold(float new_threshold){
    threshold=new_threshold;
}
void Wheel::setBoost(float new_boost){
    boost=new_boost;
}

void Wheel::regulateSpeed() {
    actual_rpm = encoder.getSpeed();
    error = target_rpm - actual_rpm;

    adjustment = error/max_rpm * Kp;  // Tuning factor
    float newSpeed = (motor.getSpeed() + adjustment);

    if (newSpeed > 1.0f) newSpeed = 1.0f;
    if (newSpeed < -1.0f) newSpeed = -1.0f;

    motor.setSpeed(newSpeed);
}
    
void Wheel::regulateSpeedwithBOOST() {
    actual_rpm = encoder.getSpeed();
    error = target_rpm - actual_rpm;

    adjustment = (error / max_rpm) * Kp;  // Proportional control

    // If the error is large (e.g., struggling uphill), apply a constant boost


    if (error > threshold) {
        adjustment += boost;  
    }

    // Calculate new speed and clamp it
    float newSpeed = motor.getSpeed() + adjustment;
    newSpeed = fmaxf(fminf(newSpeed, 1.0f), -1.0f);

    motor.setSpeed(newSpeed);
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