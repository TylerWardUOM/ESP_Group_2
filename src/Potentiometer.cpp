#include "Potentiometer.h"  // Include the header file for the Potentiometer class

// Constructor to initialize the input signal and VDD value
Potentiometer::Potentiometer(PinName pin, float v) : inputSignal(pin), VDD(v) {}

// Measure the amplitude in volts (scaled by VDD)
float Potentiometer::amplitudeVolts(void) {
    return (inputSignal.read() * VDD);  // Scale the 0.0-1.0 value by VDD to return voltage
}

// Measure the normalized amplitude (0.0 to 1.0)
float Potentiometer::amplitudeNorm(void) {
    return inputSignal.read();  // Return the raw ADC value between 0.0 and 1.0
}

// Sample the potentiometer and store the normalized and voltage values
void Potentiometer::sample(void) {
    currentSampleNorm = inputSignal.read();          // Store the normalized value (0.0 - 1.0)
    currentSampleVolts = currentSampleNorm * VDD;    // Convert to voltage and store
}

// Get the most recent sample in volts
float Potentiometer::getCurrentSampleVolts(void) {
    return currentSampleVolts;  // Return the stored voltage value
}

// Get the most recent normalized sample
float Potentiometer::getCurrentSampleNorm(void) {
    return currentSampleNorm;  // Return the stored normalized value
}
