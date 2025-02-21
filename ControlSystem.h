#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "Wheel.h"
#include "PIDController.h"
#include "Timer.h"

class ControlSystem {
public:
    ControlSystem(Wheel& leftWheel, Wheel& rightWheel, float track_width, float kp_forward, float ki_forward, float kd_forward, float scaling_forward, float kp_turn, float ki_turn, float kd_turn, float scaling_turn);
    void moveForward(float distance, float speed);
    void turn(float angle, float speed);
    void update();
    bool isMovementComplete();

private:
    enum State { IDLE, MOVE_FORWARD, TURNING };
    float basespeed;
    Wheel& leftWheel;
    Wheel& rightWheel;
    PIDController pidForward;
    PIDController pidTurn;
    Timer pidTimer;

    State state;
    float targetDistance;
    int turnDirection;
    bool movementCompleted;
    float _track_width;

    void processForwardMovement(float dt);
    void processTurning(float dt);
    void stopWheels();
    float constrain(float value, float minVal, float maxVal);
};

#endif // CONTROL_SYSTEM_H
