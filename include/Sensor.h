#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"

class Sensor {
public:
    // Constructor to initialize sensor and control pins
    Sensor(PinName inputPin, PinName controlPin);
    
    // Sample the sensor and update the internal value (scaled to 3.3V)
    void Sample();
    
    // Returns the raw reading from the sensor (voltage)
    float GetValue();
    
    // Returns the normalized sensor value (0.0 - 1.0)
    float GetNormalizedValue();
    
    // Clears the sensor reading and resets the logic state
    void clearReading();
    
    // Turns the sensor on by setting the control pin high
    void on();
    
    // Turns the sensor off by setting the control pin low
    void off();
    
    // Toggles the sensor control state
    void toggle();
    
    // Digitalizes the sensor value (returns 1 if white, 0 if black)
    int digitalize();

    int Logic;  // Digital logic state after digitalization

private:
    AnalogIn sensor_pin;  // Analog input pin for sensor output
    DigitalOut control;   // Digital pin to control sensor behavior
    float value;          // Latest sensor reading
};

#endif // SENSOR_H