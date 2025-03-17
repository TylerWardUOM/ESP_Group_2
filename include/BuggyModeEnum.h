#ifndef BUGGY_MODES_ENUM_H
#define BUGGY_MODES_ENUM_H

// Enum for buggy states
enum BuggyMode {
    idle_mode,
    speed_control_mode,
    square_idle_mode,   
    line_menu_mode,
    turn_menu_mode,
    waiting_for_movement
};

// Function to get mode name
const char* getModeName(BuggyMode mode);

#endif // BUGGY_MODES_ENUM_H
