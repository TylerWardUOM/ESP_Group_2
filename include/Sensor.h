#ifndef SENSOR_H
#define SENSOR_H

/**
 * @file Sensor.h
 * @brief Defines the Sensor class for reading and controlling sensor values.
 */

#include "mbed.h"

/**
 * @class Sensor
 * @brief Represents a sensor that reads analog values and provides digitalized output.
 */
class Sensor {
public:
    /**
     * @brief Constructs a Sensor object.
     * @param inputPin The pin connected to the sensor output.
     * @param controlPin The pin used to control the sensor LED.
     */
    Sensor(PinName inputPin, PinName controlPin);
    
    /**
     * @brief Samples the sensor and updates the internal value.
     * 
     * The value is scaled to a 3.3V reference.
     */
    void Sample();
    
    /**
     * @brief Gets the raw sensor reading.
     * @return The sensor voltage value.
     */
    float GetValue();
    
    /**
     * @brief Gets the normalized sensor reading.
     * @return The normalized value between 0.0 and 1.0.
     */
    float GetNormalizedValue();
    
    /**
     * @brief Clears the sensor reading and resets the logic state.
     */
    void clearReading();
    
    /**
     * @brief Turns the sensor on by setting the LED control pin high.
     */
    void on();
    
    /**
     * @brief Turns the sensor off by setting the LED control pin low.
     */
    void off();
    
    /**
     * @brief Toggles the sensor control state.
     */
    void toggle();
    
    /**
     * @brief Converts the analog sensor value to a digital output.
     * @return 1 if white (high value), 0 if black (low value).
     */
    int digitalize();

    /// Digital logic state after digitalization (1 = white, 0 = black).
    int Logic;  

private:
    AnalogIn sensor_pin;  ///< Analog input pin for sensor output.
    DigitalOut control;   ///< Digital pin to control the sensor state.
    float value;          ///< Latest sampled sensor reading.
};

#endif // SENSOR_H
