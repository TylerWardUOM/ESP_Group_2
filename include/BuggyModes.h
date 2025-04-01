#ifndef BUGGY_MODES_H
#define BUGGY_MODES_H

/**
 * @file BuggyModes.h
 * @brief Function declarations for controlling buggy movement modes.
 */

#include "mbed.h"
#include "BuggyModeEnum.h"
#include "BuggyModeParameters.h"

class Bluetooth;
class ControlSystem;
class SensorArray;

/**
 * @brief Switches the buggy to speed control mode.
 * @param control Reference to the control system.
 * @param buggyMode Reference to the current buggy mode.
 */
void switchToSpeedControlMode(ControlSystem& control, BuggyMode& buggyMode);

/**
 * @brief Switches the buggy to sensor debug mode.
 * @param buggyMode Reference to the current buggy mode.
 * @param sensorTicker Reference to the sensor ticker.
 * @param sensorArray Reference to the sensor array.
 */
void switchToSensorDebug(BuggyMode& buggyMode, Ticker& sensorTicker, SensorArray& sensorArray);

/**
 * @brief Switches the buggy to motor debug mode.
 * @param buggyMode Reference to the current buggy mode.
 * @param motorTicker Reference to the motor ticker.
 * @param control Reference to the control system.
 */
void switchToMotorDebug(BuggyMode& buggyMode, Ticker& motorTicker, ControlSystem& control);

/**
 * @brief Switches the buggy to square idle mode.
 * @param buggyMode Reference to the current buggy mode.
 */
void switchToSquareIdleMode(BuggyMode& buggyMode);

/**
 * @brief Starts the square movement mode.
 * @param control Reference to the control system.
 * @param buggyMode Reference to the current buggy mode.
 * @param controlticker Reference to the control ticker.
 * @param square_flag Reference to the square movement flag.
 * @param params Reference to the parameters for square movement.
 */
void switchToSquarePatternMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, bool& square_flag, SquarePatternParams& params);

/**
 * @brief Switches the buggy to the line mode menu.
 * @param buggyMode Reference to the current buggy mode.
 */
void switchToLineMenuMode(BuggyMode& buggyMode);

/**
 * @brief Switches the buggy to the turn mode menu.
 * @param buggyMode Reference to the current buggy mode.
 */
void switchToTurnMenuMode(BuggyMode& buggyMode);

/**
 * @brief Switches the buggy to straight-line movement mode.
 * @param control Reference to the control system.
 * @param buggyMode Reference to the current buggy mode.
 * @param controlticker Reference to the control ticker.
 * @param params Reference to the parameters for straight-line movement.
 */
void switchToLineMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, StraightLineParams& params);

/**
 * @brief Switches the buggy to turn mode.
 * @param control Reference to the control system.
 * @param buggyMode Reference to the current buggy mode.
 * @param controlticker Reference to the control ticker.
 * @param params Reference to the parameters for turning.
 */
void switchToTurnMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, TurnAngleParams& params);

/**
 * @brief Switches the buggy to a 180-degree turn-around mode.
 * @param control Reference to the control system.
 * @param buggyMode Reference to the current buggy mode.
 * @param controlticker Reference to the control ticker.
 * @param params Reference to the parameters for turning.
 */
void switchToTurnAround(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, TurnAngleParams& params);

/**
 * @brief Switches the buggy to follow mode.
 * @param control Reference to the control system.
 * @param sensorArray Reference to the sensor array.
 * @param buggyMode Reference to the current buggy mode.
 * @param controlticker Reference to the control ticker.
 * @param sensorTicker Reference to the sensor ticker.
 * @param motorTicker Reference to the motor ticker.
 * @param params Reference to the parameters for follow mode.
 */
void switchToFollowMode(ControlSystem& control, SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker, Ticker& motorTicker, FollowParams& params);

/**
 * @brief Switches the buggy to Bang-Bang mode.
 * @param control Reference to the control system.
 * @param sensorArray Reference to the sensor array.
 * @param buggyMode Reference to the current buggy mode.
 * @param controlticker Reference to the control ticker.
 * @param sensorTicker Reference to the sensor ticker.
 * @param motorTicker Reference to the motor ticker.
 * @param params Reference to the parameters for Bang-Bang mode.
 */
void switchToBangBangMode(ControlSystem& control, SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker, Ticker& motorTicker, BangBangParams& params);

/**
 * @brief Switches the buggy to Bang-Bang proportional mode.
 * @param control Reference to the control system.
 * @param sensorArray Reference to the sensor array.
 * @param buggyMode Reference to the current buggy mode.
 * @param controlticker Reference to the control ticker.
 * @param sensorTicker Reference to the sensor ticker.
 * @param motorTicker Reference to the motor ticker.
 * @param params Reference to the parameters for Bang-Bang proportional mode.
 */
void switchToBangBangProportionalMode(ControlSystem& control, SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker, Ticker& motorTicker, BangBangProportionalParams& params);

/**
 * @brief Stops the motor and switches the buggy to idle mode.
 * @param control Reference to the control system.
 * @param buggyMode Reference to the current buggy mode.
 * @param controlticker Reference to the control ticker.
 * @param sensorTicker Reference to the sensor ticker.
 * @param motorTicker Reference to the motor ticker.
 * @param square_flag Reference to the square movement flag.
 */
void stopMotorAndSwitchToIdleMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker,Ticker& sensorTicker,Ticker& motorTicker, bool& square_flag);

#endif // BUGGY_MODES_H
