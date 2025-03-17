#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

#include "mbed.h"
#include "Sensors.h"

class SensorArray {
public:
    SensorArray(Sensor _sensors[6], int _thresholdvalue);

    void sample();
    void updateDigitalStates();
    float calculateError();

    float getCurrentSampleVolts();
    float getCurrentSampleNorm();

private:
    Sensor sensors[6];
    int thresholdvalue;
    float sensorvalues[6];
    int digitalStates[6];

    float currentSampleVolts;
    float currentSampleNorm;
};

#endif
