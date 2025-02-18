#include "mbed.h"
#include "C12832.h"
#include "Motor.h"
#include "Encoder.h"
#include "Potentiometer.h"
#include "LED.h"



//made this code no idea if it works havent been able to test it based it off the timer from last semester
//Encoder and Motor functionality in seperate files eg Motor.cpp and the class defnitions in files like Motor.h
//i hope this works
//no pins are correct except for the ones for leds, fire, up and down buttons and the display and potentiometers
//go through and correct the pin assignments
//to calculate if buggy turned 90 degrees i use the distance between the wheels i didnt know that so update the track width below
//also update the wheel diameter 
//and change the encoder resolution if we using multiple signals i dont know
//when you run it the buggy should be in an idle state pressing the joystick up should put it in a state where each potentiometer controls each wheel display should print encoder distance
//when in this state pressing the fire (joystick in) wil return back to the idle state
//when in idle state pressing down will set the buggy to square idle mode in this mode pressing up or down will return to idle but pressing fire will start drawing square
//Takes one second before starting square to give u time 
//hope it works :)


//i use bipolar motor control so it can go backwards and forwards easier this makes turning 90 degrees to make a square easier
//i not sure if this best for final design as bipolar has higher losses so less battery life more heat to deal with  

//if the buggy doesnt turn 90 degrees correctly maybe just change the Turn DIstance value and do trial and error 
//if you need to turn left instead of right replace wich value of speed is negative in updatesquaremovement function and change it to measure the right encoder instead


// Constants for the square movement
#define WHEEL_DIAMETER 0.075f  // 3.7 cm wheels i dont know it but it seems right maybe
#define ENCODER_RESOLUTION 1   // Encoder resolution (1, 2, or 4)
#define TRACK_WIDTH 0.272f       // Distance between wheels (meters) change to correct value
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

// Pin Assignments for Motor Control
Motor leftMotor(PB_7,1.22,PB_14,PA_14);  // (Bipolar1, Dir1, PWM1, Enable)
Motor rightMotor(PB_15,1,PB_13,PA_14); // (Bipolar2, Dir2, PWM2, Enable)

// Encoder Pins
Encoder leftEncoder(PA_13, PB_6, WHEEL_DIAMETER, ENCODER_RESOLUTION);   // Change pins
Encoder rightEncoder(PB_2, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION);

// Potentiometer Pins
Potentiometer potentiometerLeft(A0, 3.3);  // Left potentiometer pin
Potentiometer potentiometerRight(A1, 3.3); // Right potentiometer pin

// LCD and LED Pins
C12832 lcd(D11, D13, D12, D7, D10);  // LCD display
LED redLED(D5);
LED greenLED(D9);
LED blueLED(D8);

// Button Pins
InterruptIn fire(D4);  // Interrupt for the center button on the joystick
InterruptIn up(A2);    // Interrupt for the up button
InterruptIn down(A3);  // Interrupt for the down button

// Motor State Enum
typedef enum {
    idle_mode,
    speed_control_mode,
    square_idle_mode,   // New state for idle in the square pattern
    square_pattern_mode
} MotorState;

MotorState motorState = idle_mode;

int sideCount = 0;
const float SQUARE_DISTANCE = 0.1f;  // 0.5 meters per side

// Movement State Enum for square movement
typedef enum {
    MOVE_FORWARD,   // State for moving forward
    TURN_RIGHT,     // State for turning right
    STOP            // State for stopping
} MovementState;



// Function Prototypes
void stopMotorAndSwitchToIdleMode();
void switchToSpeedControlMode();
void switchToSquareIdleMode();
void switchToSquarePatternMode();
void updateSquareMovement();
void updateLCD();

// Update LCD periodically
Ticker lcdUpdateTicker; 
bool lcdUpdateRequired = false;

void updateLCD() {
    lcdUpdateRequired = true;
}

// LED Control Functions
void setLEDgreen(LED red, LED green, LED blue) {
    red.off();
    green.on();
    blue.off();
}

void setLEDred(LED red, LED green, LED blue) {
    red.on();
    green.off();
    blue.off();
}

void setLEDblue(LED red, LED green, LED blue) {
    red.off();
    green.off();
    blue.on();
}

void turnoffLED(LED red, LED green, LED blue) {
    red.off();
    green.off();
    blue.off();
}

// Square Movement Function
void updateSquareMovement() {
    static float targetDistance = SQUARE_DISTANCE;
    static MovementState state = MOVE_FORWARD;  // Start with moving forward

    // This switch controls the robot's behavior based on current movement state
    switch (state) {
        case MOVE_FORWARD:
            leftMotor.setSpeed(0.3);  // Move forward
            rightMotor.setSpeed(0.3);
            if (leftEncoder.getDistance() >= targetDistance) {
                leftMotor.stop();
                rightMotor.stop();
                leftEncoder.reset();
                rightEncoder.reset();
                targetDistance = TURN_DISTANCE;  // Prepare for turn
                state = TURN_RIGHT;  // Change state to turning right
            }
            break;

        case TURN_RIGHT:
            leftMotor.setSpeed(0.35);  // Rotate left wheel only
            rightMotor.stop(); // stops right wheel
            if (leftEncoder.getDistance() >= targetDistance) {
                leftMotor.stop();
                rightMotor.stop();
                leftEncoder.reset();
                rightEncoder.reset();
                wait(0.5);
                sideCount++;  // Increase side count after a turn

                if (sideCount < 4) {
                    targetDistance = SQUARE_DISTANCE;  // Reset for next side
                    state = MOVE_FORWARD;  // Continue with forward motion
                } else {
                    state = STOP;  // Stop after completing 4 sides
                }
            }
            break;

        case STOP:
            leftMotor.stop();
            rightMotor.stop();

            // After completing the square, reset static variables and exit square pattern mode
            sideCount = 0;  // Reset side count
            targetDistance = SQUARE_DISTANCE;  // Reset target distance
            state = MOVE_FORWARD;  // Reset state for next potential square

            lcd.cls();
            lcd.locate(0, 0);
            lcd.printf("Square Complete, Idle Mode");
            stopMotorAndSwitchToIdleMode();

            break;
    }
}

// Mode Switching Functions
void switchToSpeedControlMode() {
    leftMotor.enable();
    motorState = speed_control_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Speed Control Mode");
    leftEncoder.reset();
    rightEncoder.reset();

    // Detach interrupts for other modes
    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    // Attach interrupts for speed control mode
    fire.fall(callback(&stopMotorAndSwitchToIdleMode));
}

void switchToSquareIdleMode() {
    leftMotor.disable();
    motorState = square_idle_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Square Idle Mode");

    // Detach interrupts for other modes
    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    // Attach interrupts for square_idle_mode
    up.fall(callback(&stopMotorAndSwitchToIdleMode));
    down.fall(callback(&stopMotorAndSwitchToIdleMode));
    fire.fall(callback(&switchToSquarePatternMode));
}

void switchToSquarePatternMode() {
    motorState = square_pattern_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Square Pattern Mode");

    // Detach interrupts for other modes
    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);
    wait(1);
    leftMotor.stop();
    rightMotor.stop();
    leftEncoder.reset();
    rightEncoder.reset();
    leftMotor.enable();
    // No button interrupts in square_pattern_mode
}

void stopMotorAndSwitchToIdleMode() {
    leftMotor.disable();
    leftMotor.stop();
    rightMotor.stop();
    motorState = idle_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Idle Mode");

    // Detach interrupts for idle mode
    up.fall(callback(&switchToSpeedControlMode));
    down.fall(callback(&switchToSquareIdleMode));
    fire.fall(NULL);
}


long map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};
// Main Loop
int main() {
    lcdUpdateTicker.attach(&updateLCD, 0.7);  // Update LCD every 0.7second random number lol maybe change with testing 

    // Attach interrupts for general button presses
    leftMotor.stop();
    rightMotor.stop();
    up.fall(callback(&switchToSpeedControlMode));
    down.fall(callback(&switchToSquareIdleMode));
    fire.fall(NULL);

    while (true) {
        switch (motorState) {
            case idle_mode:
                setLEDred(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.locate(0, 0);
                    lcd.printf("Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case speed_control_mode:
                setLEDgreen(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(25,8);
                    lcd.printf("Speed Control Mode");
                    lcd.locate(8, 17);
                    lcd.printf("L=%.2f R=%.2f", leftEncoder.getDistance(), rightEncoder.getDistance());
                    lcdUpdateRequired = false;
                }
                // Map potentiometer values to motor speed
                potentiometerLeft.sample();
                potentiometerRight.sample();
                float speedRawL = potentiometerLeft.getCurrentSampleNorm();
                float speedRawR = potentiometerRight.getCurrentSampleNorm();
                leftMotor.setSpeed(speedRawL);  // Set motor speed based on potentiometer
                rightMotor.setSpeed(speedRawR);
                break;

            case square_idle_mode:
                setLEDblue(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(0, 0);
                    lcd.printf("Square Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case square_pattern_mode:
                setLEDblue(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(0, 0);
                    lcd.printf("Square Pattern Mode");
                    lcdUpdateRequired = false;
                }
                updateSquareMovement();  // Perform square pattern movement
                break;

            default:
                break;
        }
    }
}
