#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include "mbed.h"
#include "Sensor.h"

class SensorArray {
public:
    SensorArray(Sensor** _sensors);  // Accepts a pointer to an array

    void sample();
    float getError();
    void startTicker(float period); // Start periodic sampling
    void stopTicker();             // Stop periodic sampling
private:
    Sensor* sensors[6];  // Store pointers instead of objects
    float sensorvalues[6];
    int digitalStates[6];
    Ticker sampleTicker;           // Ticker for periodic sampling
};

#endif
