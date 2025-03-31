#include "BuggyModeEnum.h"

// Function to get mode name as a string
const char* getModeName(BuggyMode mode) {
    switch (mode) {
        case idle_mode: return "IDLE";
        case speed_control_mode: return "SPEED_CONTROL";
        case square_idle_mode: return "SQUARE_IDLE";
        case line_menu_mode: return "LINE_MENU";
        case turn_menu_mode: return "TURN_MENU";
        case follow_menu_mode: return "FOLLOW_MENU";
        case bang_bang_menu_mode: return "BANG_BANG_MENU";
        case bang_bang_proportional_menu_mode: return "BANG_PROPORTIONAL_MENU";
        case sensor_debug: return "SENSOR_DEBUG";
        case sensor_debug_menu: return "SENSOR_DEBUG";
        case motor_debug: return "MOTOR_DEBUG";
        case motor_debug_menu: return "MOTOR_DEBUG";
        case waiting_for_movement: return "WAITING_FOR_MOVEMENT";
        case sensor_callibration: return "SENSOR_CALLIBRATION";
        case reset: return "IDLE";
        default: return "UNKNOWN_MODE";
    }
}