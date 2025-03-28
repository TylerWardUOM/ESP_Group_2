#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "mbed.h"
#include "Wheel.h"
#include "PIDController.h"
#include "Timer.h"
#include "BuggyModes.h"  // Include to access the global variables
#include "Bluetooth.h"
#include "SensorArray.h"


class ControlSystem {
public:
    ControlSystem(Wheel& leftWheel, Wheel& rightWheel, float track_width, Bluetooth& bluetooth, SensorArray& sensorArray);
    void moveForward(float distance, float speed);
    void turn(float angle, float speed);
    void follow(const FollowParams& params);
    void setBangBangMode(const BangBangParams& params);
    void setBangBangProportionalMode(const BangBangProportionalParams& params);
    void update();
    void moveSquare(float distance, float speed, float left_turn_multiplier, float right_turn_multiplier);
    bool isMovementComplete();
    bool isSquareComplete();
    void disableWheels();
    void stopWheels();
    void setWheelKp(float wheelKp);
    void regulateWheelSpeed();
    void debugRegulateWheelSpeed();
    void setModePIDParameters(const SquarePatternParams *squareParams, const StraightLineParams *straightlineParams, const TurnAngleParams *turnangleParams, const FollowParams *followParams);
    PIDController pidForward;
    PIDController pidTurn;
    PIDController pidLine;
  
private:
    enum State { IDLE, MOVE_FORWARD, TURNING, MOVING_SQUARE, FOLLOW_LINE,BANG_BANG_PROPORTIONAL,BANG_BANG};
    enum SquareMovementState {IDLE_SQUARE, MOVE_FORWARD_SQUARE, TURN_LEFT_SQUARE, TURN_RIGHT_SQUARE, TURN_AROUND_SQUARE, STOP_SQUARE};
    float basespeed;
    Wheel& leftWheel;
    Wheel& rightWheel;
    Timer pidTimer;
    Bluetooth& bluetooth;
    SensorArray& sensorArray;

    State state;
    SquareMovementState squareState;
    float targetDistance;
    int turnDirection;
    bool movementCompleted;
    bool squareCompleted;
    float _track_width;

    int sideCount;
    bool retracing;
    bool moving;
    //Square Parrameters     
    float square_distance;
    float square_speed;
    float square_left_turn_multiplier;
    float square_right_turn_multiplier;

    float turnSpeedMultiplier;
    float bangBangThreshold;
    float kP;
    float maxMultiplier;

    void resetEncoders();
    void processForwardMovement(float dt);
    void processTurning(float dt);
    void processSquare();
    float constrain(float value, float minVal, float maxVal);
    void enableWheels();
    void processFollowLine(float dt);
    void processBangBangProportional();
    void processBangBang();
};

#endif // CONTROL_SYSTEM_H
