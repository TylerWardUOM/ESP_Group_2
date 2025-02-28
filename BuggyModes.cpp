#include "BuggyModes.h"

// Initialize default parameters
SquarePatternParams squareParams = {
    2.3, 0.6, 0.00,   // pid_kp1, pid_ki1, pid_kd1
    2.3, 0.6, 0.00,   // pid_kp2, pid_ki2, pid_kd2
    2.2,              // scaling_forward
    2.2,              // scaling_turn
    1.1, 1.0,         // left_turn_multiplier, right_turn_multiplier
    0.5,              // distance
    90.0                // speed
};

StraightLineParams straightlineParams = {
    2.3, 0.6, 0.00,   // pid_kp, pid_ki, pid_kd
    2.2,              // scaling_forward
    0.0,              // speed
    0.0             // distance
};

TurnAngleParams turnangleParams = {
    2.3, 0.6, 0.00,   // pid_kp, pid_ki, pid_kd
    2.2,              // scaling_turn
    0.0,              // speed
    0.0              // angle
};

// Function to get mode name as a string
const char* getModeName(BuggyMode mode) {
    switch (mode) {
        case idle_mode: return "IDLE";
        case speed_control_mode: return "SPEED_CONTROL";
        case square_idle_mode: return "SQUARE_IDLE";
        case line_menu_mode: return "LINE_MENU";
        case turn_menu_mode: return "TURN_MENU";
        case waiting_for_movement: return "WAITING_FOR_MOVEMENT";
        default: return "UNKNOWN_MODE";
    }
}

BuggyMode buggyMode = idle_mode;