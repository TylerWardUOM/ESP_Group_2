#include "LED.h"  // Include the LED header file

// Constructor: Initializes the LED with the provided pin and turns it off
LED::LED(PinName pin) : outputSignal(pin) {
    off();  // Initially turn off the LED
}

// Turns the LED on (active low)
void LED::on(void) {
    outputSignal = 0;  // Set output to 0 (LED is active low)
    status = true;      // Update status to 'on'
}

// Turns the LED off (active low)
void LED::off(void) {
    outputSignal = 1;  // Set output to 1 (LED is active low)
    status = false;     // Update status to 'off'
}

// Toggles the LED between on and off states
void LED::toggle(void) {
    if (status) {  // If the LED is on
        off();     // Turn it off
    } else {       // If the LED is off
        on();      // Turn it on
    }
}

// Returns the current status of the LED (on or off)
bool LED::getStatus(void) {
    return status;  // Return the stored status value (true if on, false if off)
}
