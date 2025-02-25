#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "mbed.h"
#include "Wheel.h"
#include "PIDController.h"
#include "Timer.h"
#include "BuggyModes.h"  // Include to access the global variables
#include "Bluetooth.h"


class ControlSystem {
public:
    ControlSystem(Wheel& leftWheel, Wheel& rightWheel, float track_width, float kp_forward, float ki_forward, float kd_forward, float scaling_forward, float kp_turn, float ki_turn, float kd_turn, float scaling_turn,Bluetooth* bluetooth);
    void moveForward(float distance, float speed);
    void turn(float angle, float speed);
    void update();
    void moveSquare();
    bool isMovementComplete();
    PIDController pidForward;
    PIDController pidTurn;

private:
    enum State { IDLE, MOVE_FORWARD, TURNING, MOVING_SQUARE};
    enum SquareMovementState {IDLE_SQUARE, MOVE_FORWARD_SQUARE, TURN_LEFT_SQUARE, TURN_RIGHT_SQUARE, TURN_AROUND_SQUARE, STOP_SQUARE};
    float basespeed;
    Wheel& leftWheel;
    Wheel& rightWheel;
    Timer pidTimer;
    Bluetooth* bluetooth;

    State state;
    SquareMovementState squareState;
    float targetDistance;
    int turnDirection;
    bool movementCompleted;
    float _track_width;

    int sideCount;
    bool retracing;
    bool moving;
    float square_distance;

    void processForwardMovement(float dt);
    void processTurning(float dt);
    void processSquare();
    void stopWheels();
    float constrain(float value, float minVal, float maxVal);
};

#endif // CONTROL_SYSTEM_H
