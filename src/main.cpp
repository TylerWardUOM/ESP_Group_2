#include "mbed.h"
#include "C12832.h"
#include "Potentiometer.h"
#include "Wheel.h"
#include "ControlSystem.h"
#include "BuggyModes.h"
#include "BuggyModeEnum.h"
#include "BuggyModeParameters.h"
#include "Bluetooth.h"
#include "Sensor.h"
#include "SensorArray.h"

//Constants for Wheel
//if not going correct distance adjust wheel_diameter
#define WHEEL_DIAMETER 0.078f       // wheel diameter in meters
#define ENCODER_RESOLUTION 1        // encoder resolution (1, 2, or 4)
#define MAX_RPM 500

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
BangBangParams bangbangParams;
BangBangProportionalParams bangbangproportionalParams;
BuggyMode buggyMode = idle_mode;


// Bluetooth Instance
//Serial btSerial(PA_11,PA_12);
Serial btSerial(USBTX,USBRX);
Bluetooth bluetooth(btSerial, buggyMode, squareParams, straightlineParams, turnangleParams, followParams,bangbangParams,bangbangproportionalParams);

//Maybe adjust the left wheel multiplier if you see a consitant drift in one direction
Wheel leftWheel(PB_7,1.10,PB_14,PA_14,PA_13,PB_8,WHEEL_DIAMETER,ENCODER_RESOLUTION,MAX_RPM); //change max rpm by testing
Wheel rightWheel(PB_15, 1, PB_13, PA_14,PB_2, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION,MAX_RPM);

Sensor sensor1(A0, NC);
Sensor sensor2(A1, NC);
Sensor sensor3(A2, NC);
Sensor sensor4(A3, NC);
Sensor sensor5(A4, NC);
Sensor sensor6(A5, NC);

Sensor* sensors[6] = {&sensor1, &sensor2, &sensor3, &sensor4, &sensor5, &sensor6};

// Pass the array of pointers to SensorArray
SensorArray sensorArray(sensors, bluetooth);

// Global instance of ControlSystem
ControlSystem control(leftWheel, rightWheel, TRACK_WIDTH, bluetooth, sensorArray);

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
Ticker sensorTicker;
Ticker motorTicker;

bool square_flag = false;
bool turn_around_flag = false;

int main() {
    lcdUpdateTicker.attach(&updateLCD, 0.7);
    control.stopWheels();
    float speedRawL;
    float speedRawR;
    float desiredSpeedL;
    float desiredSpeedR;
    mode_t previousMode = buggyMode;  // Store previous mode
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

            case sensor_callibration:
                if (bluetooth.shouldStart()){
                    sensorArray.calibrate();
                    bluetooth.sendMovementFinished();
                    break;
                }
                break;
            
            case sensor_debug_menu:
                wait(1);
                bluetooth.resetDebugData();
                switchToSensorDebug(buggyMode,sensorTicker,sensorArray);
                break;

            case sensor_debug:
                lcd.locate(0, 0);
                lcd.printf("Sensor Debug Mode");
                break;

            case motor_debug_menu:
                wait(1);
                bluetooth.resetDebugData();
                control.enableWheels();
                switchToMotorDebug(buggyMode,motorTicker,control);
                break;

            case motor_debug:
                lcd.locate(0, 0);
                desiredSpeedL=bluetooth.SpeedRequestLeft();
                desiredSpeedR=bluetooth.SpeedRequestRight();
                if (desiredSpeedL!=5000.00){
                    leftWheel.setSpeed(desiredSpeedL);
                    bluetooth.printDebugData("%f\n",(desiredSpeedL));
                }
                if (desiredSpeedR!=5000.00){
                    rightWheel.setSpeed(desiredSpeedR);
                }
                lcd.printf("Motor Debug Mode");
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
                    previousMode=buggyMode;
                    switchToFollowMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,followParams);
                    break;
                }
                break;

            case bang_bang_menu_mode:
                lcd.locate(0, 0);
                lcd.printf("Bang Bang Menu Mode");
                if (bluetooth.shouldStart()){
                    previousMode=buggyMode;
                    switchToBangBangMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,bangbangParams);
                    break;
                }
                break;

            case bang_bang_proportional_menu_mode:
                lcd.locate(0, 0);
                lcd.printf("Bang Bang KP Menu Mode");
                if (bluetooth.shouldStart()){
                    previousMode=buggyMode;
                    switchToBangBangProportionalMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,bangbangproportionalParams);
                    break;
                }
                break;

            case turn_around:
                controlticker.detach();
                sensorTicker.detach();
                motorTicker.detach();
                control.stopWheels();
                turn_around_flag=true;
                switchToTurnAround(control,buggyMode,controlticker,turnangleParams);

            case waiting_for_movement:
                lcd.locate(0, 0);
                lcd.printf("Waiting for Movement");

                if ((square_flag && control.isSquareComplete()) || (!square_flag && control.isMovementComplete())) {
                    if (turn_around_flag){
                        if (previousMode==follow_menu_mode){
                            switchToFollowMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,followParams);
                        }else if(previousMode==bang_bang_menu_mode){
                            switchToBangBangMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,bangbangParams);
                        }else if(previousMode==bang_bang_proportional_menu_mode){
                            switchToBangBangProportionalMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,bangbangproportionalParams);
                        }
                    }else{
                        controlticker.detach();
                        sensorTicker.detach();
                        motorTicker.detach();
                        bluetooth.sendMovementFinished();
                        bluetooth.sendDebugData();
                        stopMotorAndSwitchToIdleMode(control,buggyMode,controlticker,sensorTicker,motorTicker,square_flag);
                    }
                    turn_around_flag=false;
                }
            break;

            case reset:
                controlticker.detach();
                sensorTicker.detach();
                motorTicker.detach();
                stopMotorAndSwitchToIdleMode(control,buggyMode,controlticker,sensorTicker,motorTicker,square_flag);
                break;
        }
    }
}
