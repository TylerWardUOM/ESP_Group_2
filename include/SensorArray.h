#ifndef SENSOR_ARRAY_H
#define SENSOR_ARRAY_H

/**
 * @file SensorArray.h
 * @brief Defines the SensorArray class for managing multiple sensors.
 */

#include "mbed.h"
#include "Sensor.h"
#include "Bluetooth.h"

/**
 * @class SensorArray
 * @brief Manages an array of sensors, handles calibration, and provides error calculations.
 */
class SensorArray {
public:
    /**
     * @brief Constructs a SensorArray object.
     * @param _sensors Pointer to an array of Sensor objects.
     * @param bt Reference to a Bluetooth module for debugging and communication.
     */
    SensorArray(Sensor** _sensors, Bluetooth &bt);

    /**
     * @brief Samples all sensors and updates their values.
     */
    void sample();

    /**
     * @brief Computes the error value for line-following applications.
     * @return The calculated error value based on sensor readings.
     */
    float getError();

    /**
     * @brief Starts periodic sampling of sensors at a specified interval.
     * @param period The sampling period in seconds.
     */
    void startTicker(float period);

    /**
     * @brief Stops periodic sensor sampling.
     */
    void stopTicker();

    /**
     * @brief Logs sensor data for debugging purposes via Bluetooth.
     */
    void debugSensorData();

    /**
     * @brief Retrieves the latest sensor readings.
     * @return Pointer to an array containing sensor values.
     */
    float* getValues();

    /**
     * @brief Calibrates the sensors over a white line to normalize maximum values.
     */
    void calibrate();

    /**
     * @brief Calibrates the sensors over a black surface to normalize minimum values.
     */
    void calibrateBlack();

private:
    Sensor* sensors[6];  ///< Array of pointers to sensor objects.
    float sensorvalues[6];  ///< Stores the latest sensor readings.
    int digitalStates[6];  ///< Stores digitalized sensor states.
    float calibratedValues[6];  ///< Stores maximum calibration values (white line).
    float blackValues[6];  ///< Stores minimum calibration values (black surface).
    Ticker sampleTicker;  ///< Ticker for periodic sampling.
    Bluetooth &_bt;  ///< Reference to the Bluetooth module for debugging.
};

#endif // SENSOR_ARRAY_H
