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

//Constants for Wheel
//if not going correct distance adjust wheel_diameter
#define WHEEL_DIAMETER 0.078f       // wheel diameter in meters
#define ENCODER_RESOLUTION 1        // encoder resolution (1, 2, or 4)
#define MAX_RPM 300


//Constants for Buggy
//If not turning to set angle adjust track width
#define TRACK_WIDTH 0.20f          // distance between wheels (meters)
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

//Constants for Control
//If it doesnt go straight adjust these values
#define KP_FORWARD 2.3f
#define KI_FORWARD 0.6f
#define KD_FORWARD 0
#define SCALING_FORWARD 2.2f

//If turns arent consistent adjust these values
// If turns perfect one direction of square but not the other add a multipler to the turn angle in updatesquaremovement function
#define KP_TURN 2.3f
#define KI_TURN 0.6f
#define KD_TURN 0
#define SCALING_TURN 2.2f

//Try get this to smallest value possible if code stops working properly go back to previous
#define SQUARE_CONTROL_INTERVAL 0.01

//Maybe adjust the left wheel multiplier if you see a consitant drift in one direction
Wheel leftWheel(PB_7,1.10,PB_14,PA_14,PA_13,PB_8,WHEEL_DIAMETER,ENCODER_RESOLUTION,MAX_RPM); //change max rpm by testing
Wheel rightWheel(PB_15, 1, PB_13, PA_14,PB_2, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION,MAX_RPM);
// Global instance of ControlSystem
ControlSystem control(leftWheel, rightWheel, TRACK_WIDTH, KP_FORWARD, KI_FORWARD, KD_FORWARD, SCALING_FORWARD, KP_TURN, KI_TURN, KD_TURN, SCALING_TURN);

// Potentiometer Pins
Potentiometer potentiometerLeft(A0, 3.3);   // Left potentiometer pin
Potentiometer potentiometerRight(A1, 3.3);  // Right potentiometer pin

// LCD and LED Pins
C12832 lcd(D11, D13, D12, D7, D10);  // LCD display


//RGBLed LED(D5,D9,D8);


// Button Pins
InterruptIn fire(D4);  // Interrupt for the center button on the joystick
InterruptIn up(A2);    // Interrupt for the up button
InterruptIn down(A3);  // Interrupt for the down button
InterruptIn right(A5); 
InterruptIn left(A4);
// Motor State Enum
typedef enum {
    idle_mode,
    speed_control_mode,
    square_idle_mode,   // idle state for square pattern
    square_pattern_mode,
    line_menu_mode,
    turn_menu_mode,
    waiting_for_movement
} MotorState;

MotorState motorState = idle_mode;

//
// Function Prototypes
//
void stopMotorAndSwitchToIdleMode();
void switchToSpeedControlMode();
void switchToSquareIdleMode();
void switchToSquarePatternMode();
void switchToLineMenuMode();
void switchToTurnMenuMode();
void switchToLineMode();
void switchToTurnMode();
void updateSquareMovement();
void updateLCD();
long map(long x, long in_min, long in_max, long out_min, long out_max);

// Update LCD periodically using a Ticker
Ticker lcdUpdateTicker; 
bool lcdUpdateRequired = false;
void updateLCD() {
    lcdUpdateRequired = true;
}


Ticker controlticker;



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
    right.fall(NULL);
    left.fall(NULL);

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
    right.fall(NULL);
    left.fall(NULL);

    up.fall(callback(&stopMotorAndSwitchToIdleMode));
    down.fall(callback(&stopMotorAndSwitchToIdleMode));
    fire.fall(callback(&switchToSquarePatternMode));
}

void switchToSquarePatternMode() {
    motorState = waiting_for_movement;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Square Pattern Mode");

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);
    right.fall(NULL);
    left.fall(NULL);
    wait(1);
    leftWheel.stop();
    rightWheel.stop();
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();
    controlticker.attach(callback(&control, &ControlSystem::update), SQUARE_CONTROL_INTERVAL);
    leftWheel.motor.enable();
    control.moveSquare();
}

void switchToLineMenuMode(){
    leftWheel.motor.disable();
    motorState = line_menu_mode;
    lcd.cls();
    lcd.locate(0,0);
    lcd.printf("Line Mode");
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);
    right.fall(NULL);
    left.fall(NULL);

    up.fall(&stopMotorAndSwitchToIdleMode);
    fire.fall(callback(&switchToLineMode));
}

void switchToTurnMenuMode(){
    leftWheel.motor.disable();
    motorState = turn_menu_mode;
    lcd.cls();
    lcd.locate(0,0);
    lcd.printf("Turn Mode");
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);
    right.fall(NULL);
    left.fall(NULL);

    up.fall(&stopMotorAndSwitchToIdleMode);
    fire.fall(callback(&switchToTurnMode));
}


void switchToLineMode(){
    leftWheel.motor.enable();
    potentiometerLeft.sample();
    potentiometerRight.sample();
    float distance = map(potentiometerLeft.getCurrentSampleNorm()* 1000, 0, 1000, 0, 10); //need to map these values
    float speed = map(potentiometerRight.getCurrentSampleNorm()* 1000, 0, 1000, 0, MAX_RPM); // need to map speed
    control.moveForward(distance,speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    motorState = waiting_for_movement;
}

void switchToTurnMode(){
    leftWheel.motor.enable();
    potentiometerLeft.sample();
    potentiometerRight.sample();
    float angle = map(potentiometerLeft.getCurrentSampleNorm()* 1000, 0, 1000, -180, 180); //need to map these values
    float speed = map(potentiometerRight.getCurrentSampleNorm()* 1000, 0, 1000, 0, MAX_RPM); // need to map speed
    control.turn(angle,speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    motorState = waiting_for_movement;
}


void stopMotorAndSwitchToIdleMode() {
    leftWheel.motor.disable();
    controlticker.detach();
    leftWheel.stop();
    rightWheel.stop();
    motorState = idle_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Idle Mode");

    up.fall(callback(&switchToSpeedControlMode));
    fire.fall(callback(&switchToSquareIdleMode));
    right.fall(callback(&switchToLineMenuMode));
    left.fall(callback(&switchToTurnMenuMode));
    down.fall(NULL);
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//
// Main Loop
//
int main() {
    lcdUpdateTicker.attach(&updateLCD, 0.7);
    //leftWheel.startRegulation(0.2);
    //rightWheel.startRegulation(0.2);

    leftWheel.stop();
    rightWheel.stop();
    up.fall(callback(&switchToSpeedControlMode));
    fire.fall(callback(&switchToSquareIdleMode));
    right.fall(callback(&switchToLineMenuMode));
    left.fall(callback(&switchToTurnMenuMode));
    down.fall(NULL);
    float speedRawL;
    float speedRawR;
    float distance;
    float angle;
    float speed;

    while (true) {
        switch (motorState) {
            case idle_mode:
                //LED.setRed();
                if (lcdUpdateRequired) {
                    lcd.locate(0, 0);
                    lcd.printf("Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case speed_control_mode:
                //LED.setGreen();
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(25,8);
                    lcd.printf("Speed Control Mode");
                    lcd.locate(3, 17);
                    lcd.printf("L=%.2frpm R=%.2frpm", leftWheel.encoder.getSpeed(), rightWheel.encoder.getSpeed());
                    lcdUpdateRequired = false;
                }
                potentiometerLeft.sample();
                potentiometerRight.sample();
                speedRawL = map(potentiometerLeft.getCurrentSampleNorm()* 1000, 0, 1000, 0, MAX_RPM);
                speedRawR = map(potentiometerRight.getCurrentSampleNorm()* 1000, 0, 1000, 0, MAX_RPM); // need to map speed
                //Debug Prints
                //printf("Lraw = %.2f Rraw = %.2f\n",speedRawL,speedRawR);
                printf("LCount = %f, Rcount = %f\n",leftWheel.encoder.getRevolutions(),rightWheel.encoder.getRevolutions());
                leftWheel.setSpeed(speedRawL);
                rightWheel.setSpeed(speedRawR);
                break;

            case square_idle_mode:
                //LED.setBlue();
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(0, 0);
                    lcd.printf("Square Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case line_menu_mode:
                //LED.setGreen();
                potentiometerLeft.sample();
                potentiometerRight.sample();
                distance = map(potentiometerLeft.getCurrentSampleNorm()* 1000, 0, 1000, 0, 10); //need to map these values
                speed = map(potentiometerRight.getCurrentSampleNorm()* 1000, 0, 1000, 0, MAX_RPM); // need to map speed
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(25,8);
                    lcd.printf("Line Menu Mode");
                    lcd.locate(8, 17);
                    lcd.printf("Distance=%.2fm Speed=%.2frpm", distance, speed);
                    lcdUpdateRequired = false;
                }
                break;

            case turn_menu_mode:
                //LED.setGreen();
                potentiometerLeft.sample();
                potentiometerRight.sample();
                angle = map(potentiometerLeft.getCurrentSampleNorm()* 1000, 0, 1000, -180, 180); //need to map these values
                speed = map(potentiometerRight.getCurrentSampleNorm()* 1000, 0, 1000, 0, MAX_RPM); // need to map speed
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(25,8);
                    lcd.printf("Turn Menu Mode");
                    lcd.locate(8, 17);
                    lcd.printf("Angle=%.2f degrees Speed=%.2frpm", angle, speed);
                    lcdUpdateRequired = false;
                }
                break;


            case waiting_for_movement:
                //LED.setWhite();
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(0, 0);
                    lcd.printf("Waiting for Movement Completion");
                    lcdUpdateRequired = false;
                }
                if (control.isMovementComplete()){
                    printf("left count =%f Right count =%f\n",leftWheel.encoder.getDistance(),rightWheel.encoder.getDistance());
                    controlticker.detach();
                    stopMotorAndSwitchToIdleMode();
                    break;
                }
                break;    

            default:
                break;
        }
    }
}
