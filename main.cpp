#include "Wheel.h"
#include "mbed.h"
#include "C12832.h"
#include "Motor.h"
#include "Encoder.h"
#include "Potentiometer.h"
#include "Wheel.h"
#include "PIDController.h"
#include "ControlSystem.h"
#include "RGBled.h"

// Constants for the square movement
#define WHEEL_DIAMETER 0.075f       // wheel diameter in meters
#define ENCODER_RESOLUTION 1        // encoder resolution (1, 2, or 4)
#define TRACK_WIDTH 0.28f          // distance between wheels (meters)
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

Wheel leftWheel(PB_7,1.22,PB_14,PA_14,PA_13,PB_6,WHEEL_DIAMETER,ENCODER_RESOLUTION,100); //change max rpm by testing
Wheel rightWheel(PB_15, 1, PB_13, PA_14,PB_2, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION,100);
// Global instance of ControlSystem
ControlSystem control(leftWheel, rightWheel, TRACK_WIDTH);

// Potentiometer Pins
Potentiometer potentiometerLeft(A0, 3.3);   // Left potentiometer pin
Potentiometer potentiometerRight(A1, 3.3);  // Right potentiometer pin

// LCD and LED Pins
C12832 lcd(D11, D13, D12, D7, D10);  // LCD display
RGBLed LED(D5,D9,D8);
// Button Pins
InterruptIn fire(D4);  // Interrupt for the center button on the joystick
InterruptIn up(A2);    // Interrupt for the up button
InterruptIn down(A3);  // Interrupt for the down button
InterruptIn righ(A5); 
// Motor State Enum
typedef enum {
    idle_mode,
    speed_control_mode,
    square_idle_mode,   // idle state for square pattern
    square_pattern_mode,
    line_mode
} MotorState;

MotorState motorState = idle_mode;

//
// Function Prototypes
//
void stopMotorAndSwitchToIdleMode();
void switchToSpeedControlMode();
void switchToSquareIdleMode();
void switchToSquarePatternMode();
void switchToLineMode();
void updateSquareMovement();
void updateLCD();

// Update LCD periodically using a Ticker
Ticker lcdUpdateTicker; 
bool lcdUpdateRequired = false;
void updateLCD() {
    lcdUpdateRequired = true;
}



// Movement State Enum for square movement
typedef enum {
    MOVE_FORWARD,   // State for moving forward
    TURN_RIGHT,
    TURN_LEFT,
    TURN_AROUND,     // State for turning right
    STOP            // State for stopping
} MovementState;



const float SQUARE_DISTANCE = 0.5f;  // distance for each side in meters
void updateSquareMovement() {
    static MovementState state = MOVE_FORWARD;  // Start with moving forward
    static bool retracing = false;  // Track whether we're retracing
    static int sideCount = 0;  // Track the number of sides completed

    switch (state) {
        case MOVE_FORWARD:
            // Move forward a fixed distance (one side of the square)
            control.moveForward(SQUARE_DISTANCE);  // moveForward is used as in your original logic
            if (control.isMovementComplete()) {  // Wait for the movement to complete
                state = retracing ? TURN_RIGHT : TURN_LEFT;
            }
            break;

        case TURN_LEFT:  // Turn left after moving forward (square pattern)
            control.turn(90);  // Turn 90° left
            if (control.isMovementComplete()) {
                sideCount++;  
                state = (sideCount < 4) ? MOVE_FORWARD : TURN_AROUND;  
            }
            break;

        case TURN_RIGHT:  // Turn right while retracing
            control.turn(-90);  // Turn 90° right (opposite direction for retracing)
            if (control.isMovementComplete()) {
                sideCount++;  // Increment the side count
                state = (sideCount < 4) ? MOVE_FORWARD : STOP;
            }
            break;

        case TURN_AROUND:  // Turn around (180°) after completing the square
            control.turn(180);  // Turn 180° to prepare for retracing
            if (control.isMovementComplete()) {
                retracing = true;  // Start retracing
                sideCount = 0;  // Reset side count for retracing
                state = MOVE_FORWARD;  // Start moving forward again, but retracing
            }
            break;

        case STOP:  // Stop after retracing all sides
            retracing = false;  
            sideCount = 0;  
            state = MOVE_FORWARD;  
            lcd.cls();
            lcd.locate(0, 0);
            lcd.printf("Square Complete, Idle Mode");
            stopMotorAndSwitchToIdleMode();  // Call to stop the motors and switch to idle
            break;
    }
    control.update();
}




//
// Mode Switching Functions
//
void switchToSpeedControlMode() {
    leftWheel.motor.enable();
    motorState = speed_control_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Speed Control Mode");
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    fire.fall(callback(&stopMotorAndSwitchToIdleMode));
}

void switchToSquareIdleMode() {
    leftWheel.motor.disable();
    motorState = square_idle_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Square Idle Mode");

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    up.fall(callback(&stopMotorAndSwitchToIdleMode));
    down.fall(callback(&stopMotorAndSwitchToIdleMode));
    fire.fall(callback(&switchToSquarePatternMode));
}

void switchToSquarePatternMode() {
    motorState = square_pattern_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Square Pattern Mode");

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);
    wait(1);
    leftWheel.stop();
    rightWheel.stop();
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();
    leftWheel.motor.enable();
}

void switchToLineMode(){
    leftWheel.motor.enable();
    motorState = line_mode;
    lcd.cls();
    lcd.locate(0,0);
}

void stopMotorAndSwitchToIdleMode() {
    leftWheel.motor.disable();
    leftWheel.stop();
    rightWheel.stop();
    motorState = idle_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Idle Mode");

    up.fall(callback(&switchToSpeedControlMode));
    down.fall(callback(&switchToSquareIdleMode));
    fire.fall(NULL);
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//
// Main Loop
//
int main() {
    lcdUpdateTicker.attach(&updateLCD, 0.7);
    leftWheel.regulateSpeed();
    rightWheel.regulateSpeed();

    leftWheel.stop();
    rightWheel.stop();
    up.fall(callback(&switchToSpeedControlMode));
    down.fall(callback(&switchToSquareIdleMode));
    fire.fall(NULL);
    float speedRawL;
    float speedRawR;

    while (true) {
        switch (motorState) {
            case idle_mode:
                LED.setRed();
                if (lcdUpdateRequired) {
                    lcd.locate(0, 0);
                    lcd.printf("Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case speed_control_mode:
                LED.setGreen();
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(25,8);
                    lcd.printf("Speed Control Mode");
                    lcd.locate(8, 17);
                    lcd.printf("L=%.2f R=%.2f", leftWheel.encoder.getDistance(), rightWheel.encoder.getDistance());
                    lcdUpdateRequired = false;
                }
                potentiometerLeft.sample();
                potentiometerRight.sample();
                speedRawL = potentiometerLeft.getCurrentSampleNorm();
                speedRawR = potentiometerRight.getCurrentSampleNorm();
                leftWheel.setSpeed(speedRawL);
                rightWheel.setSpeed(speedRawR);
                break;

            case square_idle_mode:
                LED.setBlue();
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(0, 0);
                    lcd.printf("Square Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case square_pattern_mode:
                LED.setWhite();
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(0, 0);
                    lcd.printf("Square Pattern Mode");
                    lcdUpdateRequired = false;
                }
                updateSquareMovement();  // perform square movement with PID control
                break;

            default:
                break;
        }
    }
}
