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

struct FollowParams {
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_scaling;
    float speed;
    float sensorSamplePeriod;   // Sampling period for sensors
    float controlPeriod;        // Control loop execution period
    float motorRegulatePeriod;
    float motor_Kp;

    // Constructor
    FollowParams();
};


struct BangBangParams {
    float baseSpeed;
    float turnSpeedMultiplier;  // Speed multiplier for turning
    float sensorSamplePeriod;   // Sampling period for sensors
    float controlPeriod;        // Control loop execution period
    float motorRegulatePeriod;
    float motor_Kp;

    // Constructor
    BangBangParams();

};

struct BangBangProportionalParams {
    float baseSpeed;
    float kP;                   // Proportional gain
    float maxMultiplier;        // Limits speed changes
    float bangBangThreshold;    // Threshold to switch between bang-bang and proportional
    float sensorSamplePeriod;   // Sampling period for sensors
    float controlPeriod;        // Control loop execution period
    float motorRegulatePeriod;
    float motor_Kp;

    // Constructor
    BangBangProportionalParams();

};

#endif // BUGGY_MODE_PARAMS_H
