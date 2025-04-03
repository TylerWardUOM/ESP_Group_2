#include "MotorUnipolar.h"

MotorUnipolar::MotorUnipolar(PinName pwm, PinName enable, PinName dir, float multiplier) 
    : _pwm(pwm), _enable(enable), _dir(dir), _multiplier(multiplier) {

    _pwm.period(0.00005f); // Set PWM period (20 kHz)
    _enable = 0;           // Disable motor driver initially
}

void MotorUnipolar::setSpeed(float speed) {
    _speed = speed;

    // Clamp speed to [-1.0, 1.0]
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    // Set direction based on the sign of speed
    _dir = (speed >= 0) ? 1 : 0;  // Set direction pin (1 for forward, 0 for reverse)

    // Set PWM duty cycle for motor speed
    float duty = fabs(speed);      // Use the absolute value for unipolar mode
    _pwm.write(duty);
}

void MotorUnipolar::enable() {
    _enable = 1;
}

void MotorUnipolar::disable() {
    _enable = 0;
}

void MotorUnipolar::stop() {
    _pwm.write(0.0f);  // Stop motor
}

float MotorUnipolar::getSpeed() {
    return _speed;
}
