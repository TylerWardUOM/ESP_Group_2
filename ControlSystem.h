#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "Wheel.h"
#include "PIDController.h"
#include "Timer.h"

class ControlSystem {
public:
    ControlSystem(Wheel& leftWheel, Wheel& rightWheel, float track_width);
    void moveForward(float distance);
    void turn(float angle);
    void update();
    bool isMovementComplete();

private:
    enum State { IDLE, MOVE_FORWARD, TURNING };
    
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
