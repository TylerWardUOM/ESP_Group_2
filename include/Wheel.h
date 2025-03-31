#ifndef WHEEL_H
#define WHEEL_H

#include "mbed.h"
#include "Motor.h"
#include "Encoder.h"

class Wheel {
public:
    Wheel(PinName motorBipolar, float motorMultiplier, PinName motorPwm, PinName motorEnable, 
          PinName encoderA, PinName encoderB, float wheelDiameter, int encoderResolution, int maxRpm);

    void setSpeed(int rpm);
    void regulateSpeed();
    void regulateSpeedDebug();
    void regulateSpeedwithBOOST();
    void startRegulation(float interval);
    void stopRegulation();
    void enableMotor();
    void disableMotor();
    void setKp(float kp);
    void stop();
    Motor motor;
    Encoder encoder;

private:
    int target_rpm;
    int max_rpm;
    float Kp;
    Ticker ticker;
};

#endif // WHEEL_H
