#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include "mbed.h"
#include "Sensor.h"
#include "Bluetooth.h"
class SensorArray {
public:
    SensorArray(Sensor** _sensors, Bluetooth &bt);  // Accepts a pointer to an array

    void sample();
    float getError();
    void startTicker(float period); // Start periodic sampling
    void stopTicker();             // Stop periodic sampling
    void debugSensorData();
    float* getValues();
    void calibrate();
    void calibrateBlack();
private:
    Sensor* sensors[6];  // Store pointers instead of objects
    float sensorvalues[6];
    int digitalStates[6];
    float calibratedValues[6];   // Stores calibration values
    float blackValues[6];   // Stores calibration black values
    Ticker sampleTicker;           // Ticker for periodic sampling
    Bluetooth &_bt;
};

#endif
