#include "Wheel.h"
#include "mbed.h"
#include "C12832.h"
#include "Motor.h"
#include "Encoder.h"
#include "Potentiometer.h"
#include "LED.h"
#include "Wheel.h"
#include "PIDController.h"
#include "ControlSystem.h"

// Constants for the square movement
#define WHEEL_DIAMETER 0.075f       // wheel diameter in meters
#define ENCODER_RESOLUTION 1        // encoder resolution (1, 2, or 4)
#define TRACK_WIDTH 0.28f          // distance between wheels (meters)
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

Wheel leftWheel(PB_7,1.22,PB_14,PA_14,PA_13,PB_6,WHEEL_DIAMETER,ENCODER_RESOLUTION,100); //change max rpm by testing
Wheel rightWheel(PB_15, 1, PB_13, PA_14,PB_2, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION,100);

// Potentiometer Pins
Potentiometer potentiometerLeft(A0, 3.3);   // Left potentiometer pin
Potentiometer potentiometerRight(A1, 3.3);  // Right potentiometer pin

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
    square_idle_mode,   // idle state for square pattern
    square_pattern_mode
} MotorState;

MotorState motorState = idle_mode;

int sideCount = 0;
const float SQUARE_DISTANCE = 0.5f;  // distance for each side in meters

// Movement State Enum for square movement
typedef enum {
    MOVE_FORWARD,
    TURN_RIGHT,
    TURN_LEFT,
    TURN_AROUND,
    STOP
} MovementState;


//
// Function Prototypes
//
void stopMotorAndSwitchToIdleMode();
void switchToSpeedControlMode();
void switchToSquareIdleMode();
void switchToSquarePatternMode();
void updateSquareMovement();
void updateLCD();

// Update LCD periodically using a Ticker
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

//
// Square Movement Function (with full PID control)
//
// Global instance of ControlSystem
ControlSystem control(leftWheel, rightWheel, TRACK_WIDTH);

void updateSquareMovement() {
    static int step = 0;
    static bool retracing = false;

    if (control.isMovementComplete()) {
        switch (step) {
            case 0: case 1: case 2: case 3:
                control.moveForward(SQUARE_DISTANCE);
                step++;
                break;
            case 4:
                control.turn(retracing ? -90 : 90);
                step++;
                break;
            case 5: case 6: case 7: case 8:
                control.moveForward(SQUARE_DISTANCE);
                step++;
                break;
            case 9:
                control.turn(retracing ? -90 : 90);
                step++;
                break;
            case 10:
                control.turn(180);
                retracing = true;
                step = 0;
                break;
        }
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
                    lcd.printf("L=%.2f R=%.2f", leftWheel.encoder.getDistance(), rightWheel.encoder.getDistance());
                    lcdUpdateRequired = false;
                }
                potentiometerLeft.sample();
                potentiometerRight.sample();
                float speedRawL = potentiometerLeft.getCurrentSampleNorm();
                float speedRawR = potentiometerRight.getCurrentSampleNorm();
                leftWheel.setSpeed(speedRawL);
                rightWheel.setSpeed(speedRawR);
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
                updateSquareMovement();  // perform square movement with PID control
                break;

            default:
                break;
        }
    }
}
