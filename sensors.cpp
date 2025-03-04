#include "mbed.h"

class Sensor {
private:
    AnalogIn sensor_pin;  // Analog input pin for sensor output
    DigitalOut control; // Digital pin to control sensor behavior
    float value;         // Latest reading from sensor


public:
    int Logic;  // Logic value after digitalizing the sensor reading
    
    // Constructor to initialize sensor pin and darlington transistor pin
    Sensor(PinName inputPin, PinName controlPin) : sensor_pin(inputPin), control(controlPin), value(0.0f) {
        clearReading();  // Clear initial reading
    }

    void Sample() {
        value = sensor_pin.read()*3.3;  // Read the analog value from the sensor
    }
    
    //Return the raw reading from the sensor
    float GetValue() {
        return value;
    }
    
    //Return normalized sensor value
    float GetNormalizedValue() {
        return sensor_pin.read();
    }

    // Clear the sensor reading
    void clearReading() {
        value = 0;
        Logic = 0;
    }
    
    // Turn the sensor on
    void on() {
        control = 1;
    }

    // Turn the sensor off
    void off() {
        control = 0;
    }

    void toggle(){
        if (control == 1){
            off();
        }
        else if (control == 0){
            on();
        }
    }

  
    // Digitalize the sensor value (converts analog reading to 1 for white, 0 for black)
    int digitalize() {
        if (GetValue() > 0.4) {
            Logic = 1;  // 1 means the sensor detects white
        } else {
            Logic = 0;  // 0 means the sensor detects black
        }
        return Logic;
    }
};
