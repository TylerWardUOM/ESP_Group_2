#include "Motor.h"

Motor::Motor(PinName bipolar, float multiplier, PinName pwm, PinName enable) 
    : _bipolar(bipolar), _multiplier(multiplier), _pwm(pwm), _enable(enable) { 
    
    _pwm.period(0.00005f); // Set PWM period (20 kHz)
    _enable = 0;         // Enable motor driver
    _bipolar = 1;        // Set to bipolar mode
}

void Motor::setSpeed(float speed) {
    // Clamp speed to [-1.0, 1.0]
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;
    float duty = 0.5f + (speed * 0.5f*_multiplier);
    //Debug print
    //printf("speed = %.2f Duty = %.2f\n",speed,duty);
    _pwm.write(duty);
}

void Motor::enable(){
    _enable = 1;
}

void Motor::disable(){
    _enable = 0;
}

void Motor::stop() {
    _pwm.write(0.5f);  // Stop motor
}

float Motor::getSpeed(){
    return _speed;
}