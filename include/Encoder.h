#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"

class Encoder {
public:
    Encoder(PinName pinA, PinName pinB, float wheelDiameter, int resolution = 1);
    
    int getRevolutions() const;
    float getDistance() const;
    float getSpeed() const;
    void reset();

private:
    void encoderISR();
    void updateSpeed();

    InterruptIn _channelA, _channelB;
    volatile int _pulses;
    const int _pulsesPerRev;
    const float _wheelCircumference;

    Ticker _speedTicker;
    volatile int _lastPulses;
    volatile float _speed;
    int _resolution;
};

#endif
