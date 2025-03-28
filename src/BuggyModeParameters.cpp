#include "BuggyModeParameters.h"

// Initialize default parameters in constructors
SquarePatternParams::SquarePatternParams()
    : forward_pid_kp(2.3), forward_pid_ki(0.6), forward_pid_kd(0.00),
      turn_pid_kp(2.3), turn_pid_ki(0.6), turn_pid_kd(0.00),
      scaling_forward(2.2), scaling_turn(2.2),
      left_turn_multiplier(1.1), right_turn_multiplier(1.0),
      distance(0.5), speed(90.0) {}

StraightLineParams::StraightLineParams()
    : forward_pid_kp(2.3), forward_pid_ki(0.6), forward_pid_kd(0.00),
      scaling_forward(2.2), speed(0.0), distance(0.0) {}

TurnAngleParams::TurnAngleParams()
    : turn_pid_kp(2.3), turn_pid_ki(0.6), turn_pid_kd(0.00),
      scaling_turn(2.2), speed(150), angle(0.0) {}

FollowParams::FollowParams()
    : pid_kp(2.3), pid_ki(0.6), pid_kd(0.00),
      pid_scaling(2.2), speed(0.0),
      sensorSamplePeriod(0.01), controlPeriod(0.02),
      motorRegulatePeriod(0.1),motor_Kp(0.01){}

BangBangParams::BangBangParams()
    : baseSpeed(250.0), turnSpeedMultiplier(0.3),
      sensorSamplePeriod(0.0005), controlPeriod(0.0002),
      motorRegulatePeriod(0.1),motor_Kp(0.01) {}

BangBangProportionalParams::BangBangProportionalParams()
    : baseSpeed(1.0), kP(1.5), maxMultiplier(2.0),
      bangBangThreshold(0.2), sensorSamplePeriod(0.01), controlPeriod(0.02),
      motorRegulatePeriod(0.1),motor_Kp(0.01){}
