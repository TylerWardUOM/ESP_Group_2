#include "Wheel.h"

Wheel::Wheel(PinName motorBipolar, float motorMultiplier, PinName motorPwm, PinName motorEnable, 
             PinName encoderA, PinName encoderB, float wheelDiameter, int encoderResolution, int maxRpm,
             Bluetooth &bt, PinName motorDirection)  // Make motorDirection optional with default value NC
    : encoder(encoderA, encoderB, wheelDiameter, encoderResolution), 
      target_rpm(0), max_rpm(maxRpm), Kp(0.05), _bt(bt), Ki(0.15), persistent_correction(0.0), error_index(0), integral(0.0), correction_sum(0.0) {

    // Initialize error history to zero
    for (int i = 0; i < 250; i++) {
        correction_history[i] = 0.0;
    }
    // Dynamically allocate the correct motor type based on the presence of a motorDirection pin
    if (motorDirection != NC) {  // Check if a motorDirection pin was passed
        // Use Unipolar Motor
        motor = new MotorUnipolar(motorBipolar, motorDirection, motorMultiplier, motorPwm, motorEnable);
        printf("uniPolar\n");
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
        

        originalSpeed = (static_cast<float>(rpm) / max_rpm)+(persistent_correction*(1));
        motor->setSpeed(originalSpeed);
    }
    target_rpm = rpm;
}

void Wheel::setKp(float newKp){
    Kp=newKp;
}

void Wheel::setKi(float newKi){
    Ki=newKi;
}

void Wheel::setThreshold(float new_threshold){
    threshold=new_threshold;
}
void Wheel::setBoost(float new_boost){
    boost=new_boost;
}

void Wheel::setDT(float new_dt){
    dt = new_dt;
    integral = 0.0;
    for (int i = 0; i < 250; i++) {
        correction_history[i] = 0.0;
    }
    persistent_correction=0.0;
}

void Wheel::regulateSpeed() {
    actual_rpm = encoder.getSpeed();
    error = target_rpm - actual_rpm;
    

    integral *= 0.7;   // Decay old integral slightly each update
    integral += error * dt;

    float old_correction = correction_history[error_index];  // Value being overwritten
    float new_correction = newSpeed - originalSpeed;         // New correction value

    correction_sum -= old_correction;  // Remove the old value
    correction_sum += new_correction;  // Add the new value

    correction_history[error_index] = new_correction;        // Store the new value
    error_index = (error_index + 1) % 250;

    persistent_correction = correction_sum / 250.0f;  // Fast average

    adjustment = (error / max_rpm) * Kp + (integral/max_rpm)*Ki;  // Tuning factor
    newSpeed = motor->getSpeed() + adjustment;

    if (newSpeed > 1.0f) newSpeed = 1.0f;
    if (newSpeed < -1.0f) newSpeed = -1.0f;

    motor->setSpeed(newSpeed);
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
    newSpeed = motor->getSpeed() + adjustment;
    newSpeed = fmaxf(fminf(newSpeed, 1.0f), -1.0f);

    motor->setSpeed(newSpeed);
}


void Wheel::regulateSpeedDebug() {
    actual_rpm = encoder.getSpeed();
    error = target_rpm - actual_rpm;

    adjustment = error/max_rpm * Kp;  // Tuning factor
    newSpeed = (motor->getSpeed() + adjustment);

    if (newSpeed > 1.0f) newSpeed = 1.0f;
    if (newSpeed < -1.0f) newSpeed = -1.0f;

    motor->setSpeed(newSpeed);
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
     motor->enable();
}

void Wheel::disableMotor(){
    motor->disable();
}

void Wheel::stop() {
    target_rpm = 0;  // Ensure target speed is reset
    motor->setSpeed(0.0f);  // Stop the motor
}

void Wheel::debugWheelData(int side){
    Bluetooth::MotorDebugData motorData = {side,encoder.getDistance(),actual_rpm,target_rpm,error,adjustment,persistent_correction,newSpeed,originalSpeed};
    _bt.logDebugData(Bluetooth::MOTOR_DEBUG, &motorData);
}


void Wheel::live_debugWheelData(int side){
    Bluetooth::MotorDebugData motorData = {side,encoder.getDistance(),actual_rpm,target_rpm,error,adjustment,persistent_correction,newSpeed,originalSpeed};
    _bt.printLiveDebugData(Bluetooth::MOTOR_DEBUG, &motorData);
}


void Wheel::setMaxRpm(int MaxRpm){
    max_rpm=MaxRpm;
}