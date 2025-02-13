#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

class Motor {
public:
    Motor(PinName bipolar, PinName direction, PinName pwm, PinName enable);
    void setSpeed(float speed);  // Speed from -1.0 to 1.0
    void stop();

private:
    DigitalOut _bipolar;
    DigitalOut _direction;
    PwmOut _pwm;
    DigitalOut _enable;
};

#endif
