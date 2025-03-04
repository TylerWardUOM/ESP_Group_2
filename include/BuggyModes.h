#ifndef BUGGY_MODES_H
#define BUGGY_MODES_H


#include "mbed.h"
#include "BuggyModeEnum.h"
#include "BuggyModeParameters.h"
class Bluetooth;
class ControlSystem;

// Function declarations for mode switching
void switchToSpeedControlMode(ControlSystem& control, BuggyMode& buggyMode);
void switchToSquareIdleMode(BuggyMode& buggyMode);
void switchToSquarePatternMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, bool& square_flag, SquarePatternParams& params);
void switchToLineMenuMode(BuggyMode& buggyMode);
void switchToTurnMenuMode(BuggyMode& buggyMode);
void switchToLineMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, StraightLineParams& params);
void switchToTurnMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, TurnAngleParams& params);
void stopMotorAndSwitchToIdleMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, bool& square_flag);

#endif // BUGGY_MODES_H
