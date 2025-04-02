#include "mbed.h"
#include "SensorArray.h"
#include "Sensor.h"

// Constructor
SensorArray::SensorArray(Sensor** _sensors, Bluetooth &bt): _bt(bt) {
    memcpy(sensors, _sensors, 6 * sizeof(Sensor*));  // Faster than loop assignment
    float defaultCalibratedValues[6] = {2.2596, 2.0807, 2.0566, 2.6416, 1.7092, 1.9590};
    memcpy(calibratedValues, defaultCalibratedValues, sizeof(calibratedValues));
    float defaultBlackValues[6] = {0};
    memcpy(blackValues, defaultBlackValues, sizeof(blackValues));
}


// Calibrate sensor array by reading values over the white surface
void SensorArray::calibrate() {
    float sum[6] = {0};
    float maxReading[6] = {0};  // To store the maximum value for each sensor
    int samples = 100;  // Number of samples for calibration

    // Take multiple samples and determine the max reading for each sensor
    for (int j = 0; j < samples; j++) {
        for (int i = 0; i < 6; i++) {
            float val = sensors[i]->GetValue();
            sum[i] += val;
            if (val > maxReading[i]) {
                maxReading[i] = val;  // Track the maximum value
            }
        }
        wait(0.01);  // Small delay to allow readings to stabilize
    }

    // Normalize the sensor values based on the maximum readings (calibrate over the white line)
    for (int i = 0; i < 6; i++) {
        calibratedValues[i] = maxReading[i];  // Store max value as calibration value
    }
}

// Calibrate sensor array by reading values over the black surface
void SensorArray::calibrateBlack() {
    float minReading[6] = {10000};  // To store the minimum value for each sensor
    int samples = 100;  // Number of samples for calibration

    // Take multiple samples and determine the minimum reading for each sensor (black surface)
    for (int j = 0; j < samples; j++) {
        for (int i = 0; i < 6; i++) {
            float val = sensors[i]->GetValue();
            if (val < minReading[i]) {
                minReading[i] = val;  // Track the minimum value (black surface)
            }
        }
        wait(0.01);  // Small delay to allow readings to stabilize
    }

    // Normalize the sensor values based on the minimum readings (black surface calibration)
    for (int i = 0; i < 6; i++) {
        blackValues[i] = minReading[i];  // Store min value as calibration value
    }
}


// Sample analog values from sensors (after calibration)
void SensorArray::sample() {
    for (int i = 0; i < 6; i++) {
        float val = sensors[i]->GetValue();
        sensorvalues[i] = ((val-blackValues[i]) / calibratedValues[i]);  // Normalize each sensor's value
        digitalStates[i] = sensorvalues[i] > 0.4; // Inline digitalization (thresholding)
    }
}


// Compute error value (relative position of line)
float SensorArray::getError() {
    static const float weightings[6] = {-3.0, -2.0, -1.0, 1.0, 2.0, 3.0}; 
    float error = 0;
    int activeCount = 0;

    for (int i = 0; i < 6; i++) {
        error += (weightings[i]*sensorvalues[i]);
        if (digitalStates[i]) {
            activeCount++;
        }
    }

    if (activeCount == 0) {
        return 999.0f; // Special value indicating no line detected
    } else {
        return error;
    }
}

float* SensorArray::getValues() {
    return sensorvalues; 
}

// Start the ticker to sample at a fixed interval
void SensorArray::startTicker(float period) {
    sampleTicker.attach(callback(this, &SensorArray::sample), period);
}

// Stop the ticker
void SensorArray::stopTicker() {
    sampleTicker.detach();
}

void SensorArray::live_debugSensorData() {
    sample();  // Sample the sensor values
    float error = getError();  // Compute error value

    _bt.printLiveSensorData(sensorvalues, 6, error);  // Send data via Bluetooth
}

void SensorArray::debugSensorData() {
    sample();  // Sample the sensor values
    float error = getError();  // Compute error value
    Bluetooth::SensorDebugData sensorData = {};
    memcpy(sensorData.sensor_values, sensorvalues, sizeof(sensorData.sensor_values));
    sensorData.error=error;
    _bt.logDebugData(Bluetooth::SENSOR_DEBUG, &sensorData);
}