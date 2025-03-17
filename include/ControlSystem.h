#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "mbed.h"
#include "Wheel.h"
#include "PIDController.h"
#include "Timer.h"
#include "BuggyModes.h"  // Include to access the global variables
#include "Bluetooth.h"
#include "SensoryArray.h"


class ControlSystem {
public:
    ControlSystem(Wheel& leftWheel, Wheel& rightWheel, float track_width, Bluetooth& bluetooth, SensorArray& sensorArray);
    void moveForward(float distance, float speed);
    void turn(float angle, float speed);
    void update();
    void moveSquare();
    bool isMovementComplete();
    bool isSquareComplete();
    PIDController pidForward;
    PIDController pidTurn;
    PIDController pidLine;
  

private:
    enum State { IDLE, MOVE_FORWARD, TURNING, MOVING_SQUARE};
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
    float square_distance;

    void processForwardMovement(float dt);
    void processTurning(float dt);
    void processSquare();
    void stopWheels();
    float constrain(float value, float minVal, float maxVal);
};

#endif // CONTROL_SYSTEM_H
