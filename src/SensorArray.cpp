#include "mbed.h"
#include "SensorArray.h"
#include "Sensors.h"

// Constructor
SensorArray::SensorArray(Sensor _sensors[6], int _thresholdvalue) : thresholdvalue(_thresholdvalue) {
    for (int i = 0; i < 6; i++) {
        sensors[i] = _sensors[i];
    }
}

// Sample analog values from sensors
void SensorArray::sample() {
    for (int i = 0; i < 6; i++) {
        sensorvalues[i] = sensors[i].GetValue();
    }
}

// Convert analog values to digital states
void SensorArray::updateDigitalStates() {
    for (int i = 0; i < 6; i++) {
        digitalStates[i] = (sensorvalues[i] >= thresholdvalue) ? 1 : 0;
    }
}

// Compute error value (relative position of line)
float SensorArray::calculateError() {
    float weightings[6] = {-0.6, -0.4, -0.2, 0.2, 0.4, 0.6}; 
    float error = 0.0;
    int activeCount = 0;

    for (int i = 0; i < 6; i++) {
        if (digitalStates[i] == 1) {
            error += weightings[i];  // Use weights to calculate deviation
            activeCount++;
        }
    }

    if (activeCount > 0) {
        return error; // Return weighted error value
    } else {
        return 0.0; // Return 0 if no line is detected
    }
}


// Get the most recent voltage sample
float SensorArray::getCurrentSampleVolts() {
    return currentSampleVolts;
}

// Get the most recent normalized sample
float SensorArray::getCurrentSampleNorm() {
    return currentSampleNorm;
}
