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
#include "OneWire_Methods.h"
#include "ds2781.h"

//Constants for Wheel
//if not going correct distance adjust wheel_diameter
#define WHEEL_DIAMETER 0.078f       // wheel diameter in meters
#define ENCODER_RESOLUTION 4        // encoder resolution (1, 2, or 4)
#define MAX_RPM 500

//Constants for Buggy
//If not turning to set angle adjust track width
#define TRACK_WIDTH 0.20f          // distance between wheels (meters)
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

//Try get this to smallest value possible if code stops working properly go back to previous
#define SQUARE_CONTROL_INTERVAL 0.01

//Battery Monitor

DigitalInOut one_wire_pin(PC_3);

SquarePatternParams squareParams;
StraightLineParams straightlineParams;
TurnAngleParams turnangleParams;
FollowParams followParams;
BangBangParams bangbangParams;
BangBangBoostParams bangbangboostParams;
BangBangProportionalParams bangbangproportionalParams;
BuggyMode buggyMode = idle_mode;


// Bluetooth Instance
//Serial btSerial(PA_11,PA_12);
Serial btSerial(USBTX,USBRX);
Bluetooth bluetooth(btSerial, buggyMode, squareParams, straightlineParams, turnangleParams, followParams,bangbangParams,bangbangboostParams,bangbangproportionalParams);

//Maybe adjust the left wheel multiplier if you see a consitant drift in one direction
Wheel leftWheel(PB_7,1.10,PB_14,PA_14,PA_13,PA_15,WHEEL_DIAMETER,ENCODER_RESOLUTION,MAX_RPM,bluetooth/*,PA_3*/); //change max rpm by testing
Wheel rightWheel(PB_15, 1, PB_13, PA_14,PB_2, PA_8, WHEEL_DIAMETER, ENCODER_RESOLUTION,MAX_RPM,bluetooth/*,PA_2*/);

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

//Potentiometer potentiometerLeft(A0, 3.3);   // Left potentiometer pin
//Potentiometer potentiometerRight(A1, 3.3);  // Right potentiometer pin


// LCD Pins
//C12832 lcd(D11, D13, D12, D7, D10);  // LCD display

//
// Function Prototypes
//
void updateSquareMovement();
void updateLCD();
long map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fullBatteryCapacity;
float Rsns= 1000.0;
float getBatteryPercentage() {
    uint32_t accumulatedCurrent = ReadAccumulatedCurrent();  // Get accumulated charge (in nVhr/Rsns)

    // Convert to mAh based on sense resistor (Rsns) and time factor
    float accumulatedCharge_mAh = accumulatedCurrent * (1.526e-9) / Rsns * 1000.0 / 3600.0;

    // Estimate remaining capacity
    float remainingCapacity = fullBatteryCapacity - accumulatedCharge_mAh;

    // Calculate battery percentage
    float batteryPercentage = (remainingCapacity / fullBatteryCapacity) * 100.0;
    batteryPercentage = fmax(0.0, fmin(100.0, batteryPercentage)); // Ensure within 0-100%

    return batteryPercentage;
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

//Debug Tickers
Ticker control_debugTicker;
Ticker sensor_debugTicker;
Ticker motor_debugTicker;

bool square_flag = false;
bool turn_around_flag = false;

int main() {
    lcdUpdateTicker.attach(&updateLCD, 0.7);
    control.stopWheels();
    float speedRawL;
    float speedRawR;
    float desiredSpeedL;
    float desiredSpeedR;
    int VoltageReading, CurrentReading; 
    float Voltage, Current, batteryPercentage; 
    mode_t previousMode = buggyMode;  // Store previous mode
    fullBatteryCapacity=2500; //Estimated 
    while (true) {
        bluetooth.processCommand(); // Process Bluetooth commands
        
        switch (buggyMode) {
            case idle_mode:
                control.stopWheels();
                if (bluetooth.shouldCallibrateWhite()){
                    sensorArray.calibrate();
                    bluetooth.sendCallibrationFinished();//Update to send the callibrated values
                    break;
                }
                if (bluetooth.shouldCallibrateBlack()){
                    sensorArray.calibrateBlack();
                    bluetooth.sendCallibrationFinished();
                    break;
                }
                if (bluetooth.shouldReadBattery()){
                    VoltageReading = ReadVoltage(); 
                    Voltage = VoltageReading*0.00967; 
                    CurrentReading = ReadCurrent(); 
                    Current = CurrentReading/6400.0;
                    batteryPercentage=getBatteryPercentage();
                    bluetooth.sendBatteryInfo(Voltage, Current, batteryPercentage);
                }

                break;

            case speed_control_mode:
                //LED.setGreen();
                if (lcdUpdateRequired) {
                    leftWheel.enableMotor();
                    lcdUpdateRequired = false;
                }
                //potentiometerLeft.sample();
                //potentiometerRight.sample();
                //speedRawL = map(potentiometerLeft.getCurrentSampleNorm()* 1000, 0, 1000, 0, MAX_RPM);
                //speedRawR = map(potentiometerRight.getCurrentSampleNorm()* 1000, 0, 1000, 0, MAX_RPM); // need to map speed
                //Debug Prints
                //printf("Lraw = %.2f Rraw = %.2f\n",speedRawL,speedRawR);
                //printf("LCount = %f, Rcount = %f\n",leftWheel.encoder.getRevolutions(),rightWheel.encoder.getRevolutions());
                leftWheel.setSpeed(speedRawL);
                rightWheel.setSpeed(speedRawR);
                break;

            
            case sensor_debug_menu:
                wait(1);
                bluetooth.resetDebugData();
                switchToSensorDebug(buggyMode,sensorTicker,sensorArray);
                break;

            case sensor_debug:
                break;

            case motor_debug_menu:
                wait(1);
                bluetooth.resetDebugData();
                control.enableWheels();
                switchToMotorDebug(buggyMode,motorTicker,control);
                break;

            case motor_debug:
                desiredSpeedL=bluetooth.SpeedRequestLeft();
                desiredSpeedR=bluetooth.SpeedRequestRight();
                if (desiredSpeedL!=5000.00){
                    leftWheel.setSpeed(desiredSpeedL);
                }
                if (desiredSpeedR!=5000.00){
                    rightWheel.setSpeed(desiredSpeedR);
                }
                break;

            case rc_menu:
                switchToRCMode(control,buggyMode);
                break;
                
            case rc:
                desiredSpeedL=bluetooth.SpeedRequestLeft();
                desiredSpeedR=bluetooth.SpeedRequestRight();
                if (desiredSpeedL!=5000.00){
                    leftWheel.setSpeed(desiredSpeedL);
                }
                if (desiredSpeedR!=5000.00){
                    rightWheel.setSpeed(desiredSpeedR);
                }
                break;

            case square_idle_mode:
                if (bluetooth.shouldStart()){
                    switchToSquarePatternMode(control, buggyMode,controlticker,square_flag,squareParams);
                    break;
                }
                break;

            case line_menu_mode:
                if (bluetooth.shouldStart()){
                    switchToLineMode(control,buggyMode,controlticker,straightlineParams);
                    break;
                }
                break;

            case turn_menu_mode:
                if (bluetooth.shouldStart()){
                    switchToTurnMode(control,buggyMode,controlticker, turnangleParams);
                    break;
                }
                break;

            
            case follow_menu_mode:
                if (bluetooth.shouldStart()){
                    previousMode=buggyMode;
                    switchToFollowMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,followParams);
                    break;
                }
                break;

            case bang_bang_menu_mode:
                if (bluetooth.shouldStart()){
                    previousMode=buggyMode;
                    if (bangbangParams.debugFlag==1.0){
                        motor_debugTicker.attach(callback(&control, &ControlSystem::debugWheels), 0.3);
                        sensor_debugTicker.attach(callback(&sensorArray, &SensorArray::debugSensorData), 0.3);
                    }
                    switchToBangBangMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,bangbangParams);
                    break;
                }
                break;
                
            case bang_bang_boost_menu_mode:
                if (bluetooth.shouldStart()){
                    previousMode=buggyMode;
                    if (bangbangboostParams.debugFlag==1.0){
                        motor_debugTicker.attach(callback(&control, &ControlSystem::debugWheels), 0.3);
                        sensor_debugTicker.attach(callback(&sensorArray, &SensorArray::debugSensorData), 0.3);
                    }
                    switchToBangBangBoostMode(control,sensorArray,buggyMode,controlticker,sensorTicker,motorTicker,bangbangboostParams);
                    break;
                }
                break;
            case bang_bang_proportional_menu_mode:
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
                        sensor_debugTicker.detach();
                        motor_debugTicker.detach();
                        control_debugTicker.detach();
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
