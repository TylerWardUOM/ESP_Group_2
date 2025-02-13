#include "Encoder.h"

Encoder::Encoder(PinName pinA, PinName pinB, float wheelDiameter, int resolution)
    : _channelA(pinA), _channelB(pinB), _pulses(0), 
      _pulsesPerRev(256 * resolution),  // Adjust pulses per revolution
      _wheelCircumference(3.1416f * wheelDiameter),
      _lastPulses(0), _speed(0.0f), _resolution(resolution) {
    
    if (resolution == 1) {
        _channelA.rise(callback(this, &Encoder::encoderISR));  // 1× mode
    } else if (resolution == 2) {
        _channelA.rise(callback(this, &Encoder::encoderISR));
        _channelA.fall(callback(this, &Encoder::encoderISR));  // 2× mode 
    } else {  
        _channelA.rise(callback(this, &Encoder::encoderISR));
        _channelA.fall(callback(this, &Encoder::encoderISR));
        _channelB.rise(callback(this, &Encoder::encoderISR));
        _channelB.fall(callback(this, &Encoder::encoderISR));
    }

    _speedTicker.attach(callback(this, &Encoder::updateSpeed), 0.1);
}

void Encoder::encoderISR() {
    int direction = _channelB.read() ? 1 : -1;  // Quadrature direction check
    _pulses += direction;
}

void Encoder::updateSpeed() {
    int pulseDiff = _pulses - _lastPulses;
    _lastPulses = _pulses;

    _speed = (static_cast<float>(pulseDiff) / _pulsesPerRev) * _wheelCircumference / 0.1f;
}

int Encoder::getRevolutions() const {
    return _pulses / _pulsesPerRev;
}

float Encoder::getDistance() const {
    return (static_cast<float>(_pulses) / _pulsesPerRev) * _wheelCircumference;
}

float Encoder::getSpeed() const {
    return _speed;
}

void Encoder::reset() {
    _pulses = 0;
    _lastPulses = 0;
    _speed = 0.0f;
}
