#include "sensors.h"

// Constructor: Initializes sensor and control pins, then clears the initial reading.
Sensor::Sensor(PinName inputPin, PinName controlPin)
    : sensor_pin(inputPin), control(controlPin), value(0.0f)
{
    clearReading();
}

// Samples the sensor by reading the analog value and converting it to a voltage.
void Sensor::Sample() {
    value = sensor_pin.read() * 3.3;
}

// Returns the raw sensor reading (voltage value).
float Sensor::GetValue() {
    return value;
}

// Returns the normalized sensor reading (0.0 - 1.0).
float Sensor::GetNormalizedValue() {
    return sensor_pin.read();
}

// Clears the sensor reading and resets the logic value.
void Sensor::clearReading() {
    value = 0;
    Logic = 0;
}

// Turns the sensor on by setting the control pin high.
void Sensor::on() {
    control = 1;
}

// Turns the sensor off by setting the control pin low.
void Sensor::off() {
    control = 0;
}

// Toggles the sensor control pin between on and off.
void Sensor::toggle() {
    if (control == 1) {
        off();
    } else {
        on();
    }
}

// Digitalizes the sensor reading: returns 1 if the raw value is greater than 0.4, else returns 0.
int Sensor::digitalize() {
    if (GetValue() > 0.4) {
        Logic = 1;  // Sensor detects white
    } else {
        Logic = 0;  // Sensor detects black
    }
    return Logic;
}
