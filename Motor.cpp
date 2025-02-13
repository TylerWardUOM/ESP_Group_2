#include "Motor.h"

Motor::Motor(PinName bipolar, PinName direction, PinName pwm, PinName enable) 
    : _bipolar(bipolar), _direction(direction), _pwm(pwm), _enable(enable) {
    
    _pwm.period(0.001f); // Set PWM period (1 kHz)
    _enable = 1;         // Enable motor driver
    _bipolar = 1;        // Set to bipolar mode
}

void Motor::setSpeed(float speed) {
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    _direction = (speed >= 0) ? 1 : 0;  // Set direction
    _pwm.write(abs(speed));             // Set PWM duty cycle
}

void Motor::stop() {
    _pwm.write(0.0f);  // Stop motor
}
