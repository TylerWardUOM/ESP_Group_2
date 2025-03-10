#include "BuggyModeEnum.h"

// Function to get mode name as a string
const char* getModeName(BuggyMode mode) {
    switch (mode) {
        case idle_mode: return "IDLE";
        case speed_control_mode: return "SPEED_CONTROL";
        case square_idle_mode: return "SQUARE_IDLE";
        case line_menu_mode: return "LINE_MENU";
        case turn_menu_mode: return "TURN_MENU";
        case follow_menu_mode: return "FOLLOW_MENU"
        case waiting_for_movement: return "WAITING_FOR_MOVEMENT";
        default: return "UNKNOWN_MODE";
    }
}