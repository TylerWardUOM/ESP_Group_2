#include "Encoder.h"

Encoder::Encoder(PinName pinA, PinName pinB, float wheelDiameter, int resolution)
    : _channelA(pinA), _channelB(pinB), _pulses(0), 
      _pulsesPerRev(256 * resolution),  // Adjust pulses per revolution
      _wheelCircumference(3.1416f * wheelDiameter),
      _lastPulses(0), _speed(0.0f), _resolution(resolution),
      prevState_(0), currState_(0) {
    
    if (resolution == 1) {
        _channelA.rise(callback(this, &Encoder::encoderISR));  // 1× mode
    } else if (resolution == 2) {
        _channelA.rise(callback(this, &Encoder::encoderISR));
        _channelA.fall(callback(this, &Encoder::encoderISR));  // 2× mode 
    } else if (resolution == 4){  
        _channelA.rise(callback(this, &Encoder::encoderISR));
        _channelA.fall(callback(this, &Encoder::encoderISR));
        _channelB.rise(callback(this, &Encoder::encoderISR));
        _channelB.fall(callback(this, &Encoder::encoderISR));
    }

    _speedTicker.attach(callback(this, &Encoder::updateSpeed), 0.01);
}

void Encoder::encoderISR() {
    int chanA = _channelA.read();  // Read state of channel A
    int chanB = _channelB.read();  // Read state of channel B

    // Update the current state with a 2-bit state based on the A and B channels
    currState_ = (chanA << 1) | chanB;

    // Handle different encoding schemes
    if (_resolution == 2) {
        // X2 Encoding Mode (basic 2-step encoding)
        // Handle transitions for counter-clockwise or clockwise based on A and B states
        if ((prevState_ == 0x3 && currState_ == 0x0) || (prevState_ == 0x0 && currState_ == 0x3)) {
            _pulses--; 
        } else if ((prevState_ == 0x2 && currState_ == 0x1) || (prevState_ == 0x1 && currState_ == 0x2)) {
            _pulses++;  
        }
    } else if (_resolution == 4) {
        // X4 Encoding Mode (advanced quadrature encoding)
        // Only update pulses if the state has changed and it's a valid transition
        if ((currState_ ^ prevState_) != 0x00) {
            // Use XOR to determine direction based on state change
            int change = (prevState_ & 0x02) ^ ((currState_ & 0x01) << 1);
            if (change == 0) {
                _pulses++;  // Reverse direction
            } else {
                _pulses--;  // Forward direction
            }
        }
    }

    // Update the previous state for the next ISR cycle
    prevState_ = currState_;
}

void Encoder::updateSpeed() {
    int pulseDiff = _pulses - _lastPulses;
    _lastPulses = _pulses;
    _speed = (static_cast<float>(pulseDiff) / _pulsesPerRev) / 0.1f; 
    _speed *= 60.0f; 
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
