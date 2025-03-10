#ifndef BUGGY_MODE_PARAMS_H
#define BUGGY_MODE_PARAMS_H

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

    // Constructor
    SquarePatternParams();
};

struct StraightLineParams {
    float forward_pid_kp;
    float forward_pid_ki;
    float forward_pid_kd;
    float scaling_forward;
    float speed;
    float distance;

    // Constructor
    StraightLineParams();
};

struct TurnAngleParams {
    float turn_pid_kp;
    float turn_pid_ki;
    float turn_pid_kd;
    float scaling_turn;
    float speed;
    float angle;

    // Constructor
    TurnAngleParams();
};

#endif // BUGGY_MODE_PARAMS_H
