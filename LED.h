#ifndef LED_H  // Prevents multiple inclusions of the header
#define LED_H

#include "mbed.h"  // For DigitalOut class

class LED {
protected:
    DigitalOut outputSignal;  // The DigitalOut object to control the LED
    bool status;              // Variable to store the current state of the LED (on or off)

public:
    // Constructor: Initializes the LED with a pin
    LED(PinName pin);

    // Public member functions
    void on(void);           // Turns the LED on
    void off(void);          // Turns the LED off
    void toggle(void);       // Toggles the LED (on -> off or off -> on)
    bool getStatus(void);    // Returns the current status of the LED (on or off)
};

#endif // LED_H
