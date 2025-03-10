#include "mbed.h"
#include "C12832.h"
#include "Potentiometer.h"
#include "Wheel.h"
#include "ControlSystem.h"
#include "BuggyModes.h"
#include "BuggyModeEnum.h"
#include "BuggyModeParameters.h"
#include "Bluetooth.h"

//Constants for Wheel
//if not going correct distance adjust wheel_diameter
#define WHEEL_DIAMETER 0.078f       // wheel diameter in meters
#define ENCODER_RESOLUTION 1        // encoder resolution (1, 2, or 4)
#define MAX_RPM 300

//Constants for Buggy
//If not turning to set angle adjust track width
#define TRACK_WIDTH 0.20f          // distance between wheels (meters)
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

//Try get this to smallest value possible if code stops working properly go back to previous
#define SQUARE_CONTROL_INTERVAL 0.01

SquarePatternParams squareParams;
StraightLineParams straightlineParams;
TurnAngleParams turnangleParams;
FollowParams followParams;
BuggyMode buggyMode = idle_mode;


// Bluetooth Instance
Serial btSerial(PA_11,PA_12);
//Serial btSerial(USBTX,USBRX);
Bluetooth bluetooth(btSerial, buggyMode, squareParams, straightlineParams, turnangleParams);

//Maybe adjust the left wheel multiplier if you see a consitant drift in one direction
Wheel leftWheel(PB_7,1.10,PB_14,PA_14,PA_13,PB_8,WHEEL_DIAMETER,ENCODER_RESOLUTION,MAX_RPM); //change max rpm by testing
Wheel rightWheel(PB_15, 1, PB_13, PA_14,PB_2, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION,MAX_RPM);
// Global instance of ControlSystem
ControlSystem control(leftWheel, rightWheel, TRACK_WIDTH, bluetooth);

// Potentiometer Pins
Potentiometer potentiometerLeft(A0, 3.3);   // Left potentiometer pin
Potentiometer potentiometerRight(A1, 3.3);  // Right potentiometer pin

// LCD Pins
C12832 lcd(D11, D13, D12, D7, D10);  // LCD display

//
// Function Prototypes
//
void updateSquareMovement();
void updateLCD();
long map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Update LCD periodically using a Ticker
Ticker lcdUpdateTicker; 
bool lcdUpdateRequired = false;
void updateLCD() {
    lcdUpdateRequired = true;
}



Ticker controlticker;

bool square_flag = false;


int main() {
    lcdUpdateTicker.attach(&updateLCD, 0.7);
    control.stopWheels();
    float speedRawL;
    float speedRawR;

    while (true) {
        bluetooth.processCommand(); // Process Bluetooth commands
        
        switch (buggyMode) {
            case idle_mode:
                lcd.locate(0, 0);
                lcd.printf("Idle Mode");
                control.stopWheels();
                break;

            case speed_control_mode:
                //LED.setGreen();
                if (lcdUpdateRequired) {
                    leftWheel.enableMotor();
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
                //printf("LCount = %f, Rcount = %f\n",leftWheel.encoder.getRevolutions(),rightWheel.encoder.getRevolutions());
                leftWheel.setSpeed(speedRawL);
                rightWheel.setSpeed(speedRawR);
                break;

            case square_idle_mode:
                lcd.locate(0, 0);
                lcd.printf("Square Idle Mode");
                if (bluetooth.shouldStart()){
                    switchToSquarePatternMode(control, buggyMode,controlticker,square_flag,squareParams);
                    break;
                }
                break;

            case line_menu_mode:
                lcd.locate(0, 0);
                lcd.printf("Line Menu Mode");
                if (bluetooth.shouldStart()){
                    switchToLineMode(control,buggyMode,controlticker,straightlineParams);
                    break;
                }
                break;

            case turn_menu_mode:
                lcd.locate(0, 0);
                lcd.printf("Turn Menu Mode");
                if (bluetooth.shouldStart()){
                    switchToTurnMode(control,buggyMode,controlticker, turnangleParams);
                    break;
                }
                break;

            
            case follow_menu_mode:
                lcd.locate(0, 0);
                lcd.printf("Follow Menu Mode");
                if (bluetooth.shouldStart()){
                    switchToFollowMode(control,buggyMode,controlticker, followParams);
                    break;
                }
                break;

            case waiting_for_movement:
                lcd.locate(0, 0);
                lcd.printf("Waiting for Movement");

                if ((square_flag && control.isSquareComplete()) || (!square_flag && control.isMovementComplete())) {
                    controlticker.detach();
                    bluetooth.sendMovementFinished();
                    bluetooth.sendDebugData();
                    stopMotorAndSwitchToIdleMode(control,buggyMode,controlticker,square_flag);
                }
            break;

        }
    }
}
