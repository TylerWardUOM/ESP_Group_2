#ifndef BUGGY_MODES_H
#define BUGGY_MODES_H


#include "mbed.h"
class Bluetooth;
class ControlSystem;

// Enum for buggy states
enum BuggyMode {
    idle_mode,
    speed_control_mode,
    square_idle_mode,   
    line_menu_mode,
    turn_menu_mode,
    waiting_for_movement
};

const char* getModeName(BuggyMode mode);


// Structs for mode-specific parameters

struct SquarePatternParams {
    float forward_pid_kp;
    float forward_pid_ki;
    float forward_pid_kd;
    float turn_pid_kp;
    float turn_pid_ki;
    float turn_pid_kd;
    float scaling_forward;
    float scaling_turn;
    float left_turn_multiplier;
    float right_turn_multiplier;
    float distance;
    float speed;
};

struct StraightLineParams {
    float forward_pid_kp;
    float forward_pid_ki;
    float forward_pid_kd;
    float scaling_forward;
    float speed;
    float distance;
};

struct TurnAngleParams {
    float turn_pid_kp;
    float turn_pid_ki;
    float turn_pid_kd;
    float scaling_turn;
    float speed;
    float angle;
};

// Declare external global instances (Defined in BuggyModes.cpp)
extern SquarePatternParams squareParams;
extern StraightLineParams straightlineParams;
extern TurnAngleParams turnangleParams;
extern BuggyMode buggyMode;


// Function declarations for mode switching
void switchToSpeedControlMode(ControlSystem& control, BuggyMode& buggyMode);
void switchToSquareIdleMode(BuggyMode& buggyMode);
void switchToSquarePatternMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, bool& square_flag);
void switchToLineMenuMode(BuggyMode& buggyMode);
void switchToTurnMenuMode(BuggyMode& buggyMode);
void switchToLineMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker);
void switchToTurnMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker);
void stopMotorAndSwitchToIdleMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, bool& square_flag);

#endif // BUGGY_MODES_H
