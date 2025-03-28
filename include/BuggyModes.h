#ifndef BUGGY_MODES_H
#define BUGGY_MODES_H


#include "mbed.h"
#include "BuggyModeEnum.h"
#include "BuggyModeParameters.h"
class Bluetooth;
class ControlSystem;
class SensorArray;

// Function declarations for mode switching
void switchToSpeedControlMode(ControlSystem& control, BuggyMode& buggyMode);

void switchToSensorDebug(BuggyMode& buggyMode, Ticker& sensorTicker, SensorArray& SensorArray);
void switchToMotorDebug(BuggyMode& buggyMode, Ticker& motorTicker, ControlSystem& control);


void switchToSquareIdleMode(BuggyMode& buggyMode);
void switchToSquarePatternMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, bool& square_flag, SquarePatternParams& params);
void switchToLineMenuMode(BuggyMode& buggyMode);
void switchToTurnMenuMode(BuggyMode& buggyMode);
void switchToLineMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, StraightLineParams& params);
void switchToTurnMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, TurnAngleParams& params);
void stopMotorAndSwitchToIdleMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker,Ticker& sensorTicker,Ticker& motorTicker, bool& square_flag);
//Switch To Follow Mode Menu
void switchToFollowMenuMode(BuggyMode& buggyMode);
void switchToFollowMode(ControlSystem& control, SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker, FollowParams& params);

void switchToBangBangMenuMode(BuggyMode& buggyMode);
void switchToBangBangMode(ControlSystem& control, SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker, BangBangParams& params);

void switchToBangBangProportionalMenuMode(BuggyMode& buggyMode);
void switchToBangBangProportionalMode(ControlSystem& control, SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker, BangBangProportionalParams& params);

#endif // BUGGY_MODES_H
