#ifndef POTENTIOMETER_H  // Prevents multiple inclusions of the header
#define POTENTIOMETER_H

#include "mbed.h"  // For AnalogIn class

class Potentiometer {
private:
    AnalogIn inputSignal;  // The analog input signal
    float VDD;              // Maximum measurable voltage (V)
    float currentSampleNorm; // Most recent normalized sample (0.0 to 1.0)
    float currentSampleVolts; // Most recent sample in volts (0.0 to VDD)

public:
    // Constructor
    Potentiometer(PinName pin, float v);

    // Member functions
    float amplitudeVolts(void);    // Measure the amplitude in volts
    float amplitudeNorm(void);     // Measure the normalized amplitude
    void sample(void);             // Sample the analog voltage
    float getCurrentSampleVolts(void);  // Get the most recent voltage sample
    float getCurrentSampleNorm(void);   // Get the most recent normalized sample
};

#endif // POTENTIOMETER_H
