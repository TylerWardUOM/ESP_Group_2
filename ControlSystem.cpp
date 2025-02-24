#include "ControlSystem.h"

ControlSystem::ControlSystem(Wheel& leftWheel, Wheel& rightWheel, float track_width, float kp_forward, float ki_forward, float kd_forward, float scaling_forward, float kp_turn, float ki_turn, float kd_turn, float scaling_turn)
    : leftWheel(leftWheel), rightWheel(rightWheel),
      pidForward(kp_forward, ki_forward, kd_forward, scaling_forward),
      pidTurn(kp_turn, ki_turn, kd_turn, scaling_turn),
      state(IDLE), targetDistance(0.0f), turnDirection(0), movementCompleted(true), _track_width(track_width) {}

void ControlSystem::moveForward(float distance, float speed) {
    targetDistance = distance;
    state = MOVE_FORWARD;
    movementCompleted = false;
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();
    basespeed = speed;
}

void ControlSystem::turn(float angle, float speed) {
    turnDirection = (angle > 0) ? 1 : -1;
    targetDistance = (_track_width / 2.0f) * (fabs(angle) * 3.14159f / 180.0f);
    state = TURNING;
    movementCompleted = false;
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();
    basespeed = speed;
}

void ControlSystem::update() {
    static bool timerStarted = false;
    if (!timerStarted) {
        pidTimer.start();
        timerStarted = true;
    }

    float currentTime = pidTimer.read();
    static float lastTime = currentTime;
    float dt = currentTime - lastTime;
    if (dt <= 0.0f) dt = 0.01f;
    lastTime = currentTime;

    switch (state) {
        case MOVE_FORWARD:
            processForwardMovement(dt);
            break;
        case TURNING:
            processTurning(dt);
            break;
        case IDLE:
            stopWheels();
            break;
    }
}

bool ControlSystem::isMovementComplete() {
    return movementCompleted;
}

void ControlSystem::processForwardMovement(float dt) {
    float leftDistance = leftWheel.encoder.getDistance();
    float rightDistance = rightWheel.encoder.getDistance();
    //printf("Left Distance = %f,Right Distance = %f\n",leftDistance,rightDistance);

    float error = rightDistance - leftDistance;
    float pidOutput = pidForward.update(error, dt);
    float multiplier = 1.0f - (pidOutput);

    // Replacing constrain with if-statements
    if (multiplier < 0.6f) {
        multiplier = 0.6f;
    } else if (multiplier > 1.4f) {
        multiplier = 1.4f;
    }

    leftWheel.setSpeed(basespeed);
    rightWheel.setSpeed(basespeed * multiplier);

    if (((leftDistance + rightDistance) / 2.0f) >= targetDistance) {
        stopWheels();
        pidForward.reset();
        movementCompleted = true;
        state = IDLE;
    }
}

void ControlSystem::processTurning(float dt) {
    float leftDistance = fabs(leftWheel.encoder.getDistance());
    float rightDistance = fabs(rightWheel.encoder.getDistance());
    float avgDistance = (leftDistance + rightDistance) / 2.0f;

    float error = targetDistance - avgDistance;
    float pidOutput = pidTurn.update(error, dt);
    float multiplier = 1.0f - (pidOutput);

    // Replacing constrain with if-statements
    if (multiplier < 0.7f) {
        multiplier = 0.7f;
    } else if (multiplier > 1.3f) {
        multiplier = 1.3f;
    }

    float baseTurnSpeed = 0.2f;
    leftWheel.setSpeed(-turnDirection * basespeed * multiplier);
    rightWheel.setSpeed(turnDirection * basespeed);

    if (fabs(error) < 0.02f || avgDistance >= targetDistance) {
        stopWheels();
        pidTurn.reset();
        movementCompleted = true;
        state = IDLE;
    }
}

void ControlSystem::stopWheels() {
    leftWheel.stop();
    rightWheel.stop();
    basespeed = 0;
}
