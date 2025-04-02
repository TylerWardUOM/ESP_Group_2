#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

/**
 * @file ControlSystem.h
 * @brief Defines the ControlSystem class for managing the buggy's movement and control logic.
 */

#include "mbed.h"
#include "Wheel.h"
#include "PIDController.h"
#include "Timer.h"
#include "BuggyModes.h"
#include "Bluetooth.h"
#include "SensorArray.h"

/**
 * @class ControlSystem
 * @brief Handles movement, control modes, and sensor-based navigation for the buggy.
 */
class ControlSystem {
public:
    /**
     * @brief Constructs a ControlSystem object.
     * @param leftWheel Reference to the left wheel.
     * @param rightWheel Reference to the right wheel.
     * @param track_width Distance between the wheels.
     * @param bluetooth Reference to the Bluetooth communication module.
     * @param sensorArray Reference to the sensor array.
     */
    ControlSystem(Wheel& leftWheel, Wheel& rightWheel, float track_width, Bluetooth& bluetooth, SensorArray& sensorArray);

    /**
     * @brief Moves the buggy forward by a specified distance and speed.
     * @param distance Distance to travel in meters.
     * @param speed Speed of movement.
     */
    void moveForward(float distance, float speed);

    /**
     * @brief Rotates the buggy by a specified angle.
     * @param angle Angle to turn in degrees.
     * @param speed Speed of the turn.
     */
    void turn(float angle, float speed);

    /**
     * @brief Engages line-following mode using PID control.
     * @param params Parameters for the follow mode.
     */
    void follow(const FollowParams& params);

    /**
     * @brief Engages Bang-Bang control mode.
     * @param params Parameters for Bang-Bang mode.
     */
    void setBangBangMode(const BangBangParams& params);
  
    void setBangBangBoostMode(const BangBangBoostParams& params);

    /**
     * @brief Engages Bang-Bang control with proportional adjustments.
     * @param params Parameters for Bang-Bang proportional mode.
     */
    void setBangBangProportionalMode(const BangBangProportionalParams& params);

    /**
     * @brief Updates the control system state.
     */
    void update();

    /**
     * @brief Executes a square movement pattern.
     * @param distance Distance per segment.
     * @param speed Movement speed.
     * @param left_turn_multiplier Speed multiplier for left turns.
     * @param right_turn_multiplier Speed multiplier for right turns.
     */
    void moveSquare(float distance, float speed, float left_turn_multiplier, float right_turn_multiplier);

    /**
     * @brief Checks if movement is complete.
     * @return True if movement is complete, false otherwise.
     */
    bool isMovementComplete();

    /**
     * @brief Checks if the square movement pattern is complete.
     * @return True if square movement is complete, false otherwise.
     */
    bool isSquareComplete();

    /**
     * @brief Disables the wheels by setting motor enable pin.
     */
    void disableWheels();

    /**
     * @brief Enables the wheels by setting motor enable pin.
     */
    void enableWheels();

    /**
     * @brief Stops the wheels immediately.
     */
    void stopWheels();

    /**
     * @brief Sets the wheel proportional gain (Kp) for motor control.
     * @param wheelKp Proportional gain value.
     */
    void setWheelKp(float wheelKp);
  
    void setWheelParams(float wheelKp, float wheel_threshold, float wheel_boost);

    void regulateWheelSpeedwithBOOST();

    /**
     * @brief Calls the wheel control functions to regulate speed using feedback control.
     */
    void regulateWheelSpeed();

    /**
     * @brief Calls the wheel control functions to regulate speed and logs debug data.
     */
    void debugRegulateWheelSpeed();

    /**
     * @brief Calls wheel debug logging functions.
     */
    void debugWheels();
    
    /**
     * @brief Sets PID parameters based on the current movement mode.
     * @param squareParams Parameters for square movement (nullable).
     * @param straightlineParams Parameters for straight-line movement (nullable).
     * @param turnangleParams Parameters for turning movement (nullable).
     * @param followParams Parameters for follow mode (nullable).
     */
    void setModePIDParameters(const SquarePatternParams *squareParams, const StraightLineParams *straightlineParams, const TurnAngleParams *turnangleParams, const FollowParams *followParams);

    /// PID Controllers
    PIDController pidForward; ///< PID controller for forward movement.
    PIDController pidTurn; ///< PID controller for turning.
    PIDController pidLine; ///< PID controller for line following.

private:
    /**
     * @enum State
     * @brief Represents the movement states of the buggy.
     */
    enum State { IDLE, MOVE_FORWARD, TURNING, MOVING_SQUARE, FOLLOW_LINE, BANG_BANG_PROPORTIONAL, BANG_BANG };
    
    /**
     * @enum SquareMovementState
     * @brief Represents different states in the square movement sequence.
     */
    enum SquareMovementState { IDLE_SQUARE, MOVE_FORWARD_SQUARE, TURN_LEFT_SQUARE, TURN_RIGHT_SQUARE, TURN_AROUND_SQUARE, STOP_SQUARE };
    
    float basespeed; ///< Base movement speed.
    Wheel& leftWheel; ///< Reference to the left wheel.
    Wheel& rightWheel; ///< Reference to the right wheel.
    Timer pidTimer; ///< Timer for PID control updates.
    Bluetooth& bluetooth; ///< Reference to the Bluetooth module.
    SensorArray& sensorArray; ///< Reference to the sensor array.
    
    State state; ///< Current movement state.
    SquareMovementState squareState; ///< Current square movement state.
    float targetDistance; ///< Target distance for movement.
    int turnDirection; ///< Direction of the turn.
    bool movementCompleted; ///< Flag for movement completion.
    bool squareCompleted; ///< Flag for square movement completion.
    float _track_width; ///< Width of the buggy track.

    int sideCount; ///< Count of completed sides in a square movement pattern.
    bool retracing; ///< Flag indicating if the buggy is retracing a path.
    bool moving; ///< Flag indicating if movement is active.
    
    // Square Movement Parameters
    float square_distance; ///< Distance per square segment.
    float square_speed; ///< Speed during square movement.
    float square_left_turn_multiplier; ///< Multiplier for left turns.
    float square_right_turn_multiplier; ///< Multiplier for right turns.
    
    float turnSpeedMultiplier; ///< Speed multiplier for turning.
    float bangBangThreshold; ///< Threshold for Bang-Bang control.
    float kP; ///< Proportional gain for Bang-Bang proportional control.
    float maxMultiplier; ///< Maximum speed multiplier.

    /**
     * @brief Resets wheel encoders.
     */
    void resetEncoders();

    /**
     * @brief Processes forward movement calculations.
     * @param dt Time step for calculations.
     */
    void processForwardMovement(float dt);

    /**
     * @brief Processes turning calculations.
     * @param dt Time step for calculations.
     */
    void processTurning(float dt);

    /**
     * @brief Processes square movement logic.
     */
    void processSquare();

    /**
     * @brief Constrains a value between a minimum and maximum.
     * @param value The value to constrain.
     * @param minVal Minimum allowable value.
     * @param maxVal Maximum allowable value.
     * @return The constrained value.
     */
    float constrain(float value, float minVal, float maxVal);

    /**
     * @brief Processes the follow-line logic.
     * @param dt Time step for calculations.
     */
    void processFollowLine(float dt);

    /**
     * @brief Processes Bang-Bang proportional control logic.
     */
    void processBangBangProportional();

    /**
     * @brief Processes Bang-Bang control logic.
     */
    void processBangBang();

};

#endif // CONTROL_SYSTEM_H
