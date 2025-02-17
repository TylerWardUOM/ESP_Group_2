#include "Motor.h"

Motor::Motor(PinName bipolar, PinName direction, PinName pwm, PinName enable) 
    : _bipolar(bipolar), _direction(direction), _pwm(pwm), _enable(enable) { 
    
    _pwm.period(0.001f); // Set PWM period (1 kHz)
    _enable = 1;         // Enable motor driver
    _bipolar = 1;        // Set to bipolar mode
}

void Motor::setSpeed(float speed) {
    // Clamp speed to [-1.0, 1.0]
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;
    
    // For bipolar control, a 50% duty cycle is stationary.
    // Map speed from [-1.0,1.0] to [0.0,1.0] where 0.0 corresponds to -1.0,
    // 0.5 corresponds to 0.0 (stationary), and 1.0 corresponds to 1.0.
    float duty = 0.5f + (speed * 0.5f);
    _pwm.write(duty);
}


void Motor::stop() {
    _pwm.write(0.5f);  // Stop motor
}
