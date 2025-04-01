#ifndef BUGGY_MODE_PARAMS_H
#define BUGGY_MODE_PARAMS_H

/**
 * @file BuggyModeParameters.h
 * @brief Defines parameter structures for various buggy movement modes.
 */

/**
 * @struct SquarePatternParams
 * @brief Parameters for executing a square movement pattern.
 */
struct SquarePatternParams {
    float forward_pid_kp; ///< Proportional gain for forward PID control.
    float forward_pid_ki; ///< Integral gain for forward PID control.
    float forward_pid_kd; ///< Derivative gain for forward PID control.
    float turn_pid_kp;    ///< Proportional gain for turn PID control.
    float turn_pid_ki;    ///< Integral gain for turn PID control.
    float turn_pid_kd;    ///< Derivative gain for turn PID control.
    float scaling_forward; ///< Scaling factor for forward movement.
    float scaling_turn; ///< Scaling factor for turning.
    float left_turn_multiplier; ///< Multiplier for left turns.
    float right_turn_multiplier; ///< Multiplier for right turns.
    float distance; ///< Distance to travel per segment.
    float speed; ///< Speed of movement.

    /**
     * @brief Constructor for SquarePatternParams.
     */
    SquarePatternParams();
};

/**
 * @struct StraightLineParams
 * @brief Parameters for executing a straight-line movement.
 */
struct StraightLineParams {
    float forward_pid_kp; ///< Proportional gain for forward PID control.
    float forward_pid_ki; ///< Integral gain for forward PID control.
    float forward_pid_kd; ///< Derivative gain for forward PID control.
    float scaling_forward; ///< Scaling factor for forward movement.
    float speed; ///< Speed of movement.
    float distance; ///< Distance to travel.

    /**
     * @brief Constructor for StraightLineParams.
     */
    StraightLineParams();
};

/**
 * @struct TurnAngleParams
 * @brief Parameters for executing a turn by a specific angle.
 */
struct TurnAngleParams {
    float turn_pid_kp; ///< Proportional gain for turn PID control.
    float turn_pid_ki; ///< Integral gain for turn PID control.
    float turn_pid_kd; ///< Derivative gain for turn PID control.
    float scaling_turn; ///< Scaling factor for turning.
    float speed; ///< Speed of turning.
    float angle; ///< Angle to turn.

    /**
     * @brief Constructor for TurnAngleParams.
     */
    TurnAngleParams();
};

/**
 * @struct FollowParams
 * @brief Parameters for following a line with PID.
 */
struct FollowParams {
    float pid_kp; ///< Proportional gain for path-following PID control.
    float pid_ki; ///< Integral gain for path-following PID control.
    float pid_kd; ///< Derivative gain for path-following PID control.
    float pid_scaling; ///< Scaling factor for PID control.
    float speed; ///< Speed of movement.
    float sensorSamplePeriod; ///< Sampling period for sensors.
    float controlPeriod; ///< Control loop execution period.
    float motorRegulatePeriod; ///< Motor regulation period.
    float motor_Kp; ///< Motor proportional control gain.

    /**
     * @brief Constructor for FollowParams.
     */
    FollowParams();
};

/**
 * @struct BangBangParams
 * @brief Parameters for Bang-Bang for Line control.
 */
struct BangBangParams {
    float baseSpeed; ///< Base movement speed.
    float turnSpeedMultiplier; ///< Speed multiplier for turning.
    float bangBangThreshold; ///< Threshold to switch between Bang-Bang states.
    float sensorSamplePeriod; ///< Sampling period for sensors.
    float controlPeriod; ///< Control loop execution period.
    float motorRegulatePeriod; ///< Motor regulation period.
    float motor_Kp; ///< Motor proportional control gain.

    /**
     * @brief Constructor for BangBangParams.
     */
    BangBangParams();
};

/**
 * @struct BangBangProportionalParams
 * @brief Parameters for Bang-Bang Line control with proportional adjustments.
 */
struct BangBangProportionalParams {
    float baseSpeed; ///< Base movement speed.
    float kP; ///< Proportional gain for control adjustments.
    float maxMultiplier; ///< Maximum limit for speed adjustments.
    float bangBangThreshold; ///< Threshold to switch between Bang-Bang and proportional control.
    float sensorSamplePeriod; ///< Sampling period for sensors.
    float controlPeriod; ///< Control loop execution period.
    float motorRegulatePeriod; ///< Motor regulation period.
    float motor_Kp; ///< Motor proportional control gain.

    /**
     * @brief Constructor for BangBangProportionalParams.
     */
    BangBangProportionalParams();
};

#endif // BUGGY_MODE_PARAMS_H
