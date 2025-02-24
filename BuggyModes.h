#ifndef BUGGY_MODES_H
#define BUGGY_MODES_H

// Enum for buggy states
enum BuggyMode {
    idle_mode,
    speed_control_mode,
    square_idle_mode,   // idle state for square pattern
    square_pattern_mode,
    line_menu_mode,
    turn_menu_mode,
    waiting_for_movement
};

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

#endif // BUGGY_MODES_H
