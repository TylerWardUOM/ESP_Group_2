#ifndef BUGGY_MODES_ENUM_H
#define BUGGY_MODES_ENUM_H

/**
 * @file BuggyModeEnum.h
 * @brief Defines the BuggyMode enum and provides a function to get mode names.
 */

/**
 * @enum BuggyMode
 * @brief Represents different operational modes of a buggy.
 */
enum BuggyMode {
    idle_mode,                        ///< The buggy is idle.
    reset,                            ///< Resets the buggy.
    speed_control_mode,               ///< Controls the buggy's speed.
    square_idle_mode,                 ///< Idle mode in square movement sequence.
    line_menu_mode,                   ///< Menu mode for line following.
    turn_menu_mode,                   ///< Menu mode for turning settings.
    follow_menu_mode,                 ///< Menu mode for follow behavior.
    bang_bang_menu_mode,              ///< Menu mode for Bang-Bang control.
    bang_bang_proportional_menu_mode, ///< Menu mode for proportional Bang-Bang control.
    sensor_debug,                     ///< Debug mode for sensors.
    sensor_debug_menu,                ///< Menu mode for sensor debugging.
    motor_debug,                      ///< Debug mode for motors.
    motor_debug_menu,                 ///< Menu mode for motor debugging.
    turn_around,                      ///< Turns the buggy around.
    waiting_for_movement              ///< Waiting for movement input.
};

/**
 * @brief Retrieves the name of a given BuggyMode as a string.
 * 
 * @param mode The BuggyMode to get the name for.
 * @return The corresponding name of the mode as a constant character pointer.
 */
const char* getModeName(BuggyMode mode);

#endif // BUGGY_MODES_ENUM_H
