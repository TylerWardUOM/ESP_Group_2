#include "mbed.h"
#include "SensorArray.h"
#include "Sensor.h"

// Constructor
SensorArray::SensorArray(Sensor** _sensors) {
    for (int i = 0; i < 6; i++) {
        sensors[i] = _sensors[i];  // Store pointers
    }
}

// Sample analog values from sensors
void SensorArray::sample() {
    for (int i = 0; i < 6; i++) {
        sensorvalues[i] = sensors[i]->GetValue();
    }
}

// Convert analog values to digital states
void SensorArray::updateDigitalStates() {
    for (int i = 0; i < 6; i++) {
        digitalStates[i] = sensors[i]->digitalize();
    }
}

// Compute error value (relative position of line)
float SensorArray::getError() {
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
