#include "MotorUnipolar.h"

MotorUnipolar::MotorUnipolar(PinName bipolar, PinName direction, float multiplier, PinName pwm, PinName enable)
    : Motor(bipolar, multiplier, pwm, enable), 
      _direction(direction){
            _bipolar = 0;        // Dissable bipolar mode
      }

void MotorUnipolar::setSpeed(float speed) {
    _speed = speed;

    // Clamp speed to [-1.0, 1.0]
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    // Set direction based on the sign of speed
    _direction = (speed >= 0) ? 1 : 0;  // Set direction pin (1 for forward, 0 for reverse)

    // Set PWM duty cycle for motor speed
    float duty = fabs(speed);      // Use the absolute value for unipolar mode
    _pwm.write(duty);
}