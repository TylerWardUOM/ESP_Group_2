#include "mbed.h"
#include "SensorArray.h"
#include "Sensor.h"

// Constructor
SensorArray::SensorArray(Sensor** _sensors, Bluetooth &bt): _bt(bt) {
    memcpy(sensors, _sensors, 6 * sizeof(Sensor*));  // Faster than loop assignment
}

// Sample analog values from sensors
void SensorArray::sample() {
    for (int i = 0; i < 6; i++) {
        float val = sensors[i]->GetValue();  
        sensorvalues[i] = val;
        digitalStates[i] = val > 0.4; // Inline digitalization
    }
}

// Compute error value (relative position of line)
float SensorArray::getError() {
    static const int weightings[6] = {-3, -2, -1, 1, 2, 3}; 
    int error = 0, activeCount = 0;

    for (int i = 0; i < 6; i++) {
        if (digitalStates[i]) {
            error += weightings[i];
            activeCount++;
        }
    }

    if (activeCount == 0) {
        return 999.0f; // Special value indicating no line detected
    } else {
        return (float)error / activeCount;
    }
}

// Start the ticker to sample at a fixed interval
void SensorArray::startTicker(float period) {
    sampleTicker.attach(callback(this, &SensorArray::sample), period);
}

// Stop the ticker
void SensorArray::stopTicker() {
    sampleTicker.detach();
}

void SensorArray::debugSensorData() {
    sample();  // Sample the sensor values
    float error = getError();  // Compute error value

    _bt.printLiveSensorData(sensorvalues, 6, error);  // Send data via Bluetooth
}
