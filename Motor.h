#ifndef MOTOR_H
#define MOTOR_H

#include "mbed.h"

class Motor {
public:
    Motor(PinName bipolar, float multiplier , PinName pwm, PinName enable);
    void setSpeed(float speed);  // Speed from -1.0 to 1.0
    void stop();
    void disable();
    void enable();

private:
    DigitalOut _bipolar;
    float _multiplier;
    PwmOut _pwm;
    DigitalOut _enable;
};

#endif
