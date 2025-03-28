#include "ControlSystem.h"
#include "BuggyModes.h"  // Include to access the global variables

//add square state to the header
//add a process square movment function based of the current square function 
//work out how to passparameters into update square worried that passing every run will memory issue
ControlSystem::ControlSystem(Wheel& leftWheel, Wheel& rightWheel,
    float track_width, Bluetooth& bluetooth, SensorArray& sensorArray)
    : leftWheel(leftWheel), rightWheel(rightWheel),
      pidForward(0, 0, 0, 1),
      pidTurn(0, 0, 0, 1),
      pidLine(0, 0, 0, 1),
      state(IDLE), targetDistance(0.0f), turnDirection(0), 
      movementCompleted(true), _track_width(track_width),squareState(IDLE_SQUARE),
      bluetooth(bluetooth),
      squareCompleted(true),square_distance(0),square_speed(0),square_left_turn_multiplier(1),square_right_turn_multiplier(1),
      sensorArray(sensorArray),turnSpeedMultiplier(0.0f){}

void ControlSystem::moveForward(float distance, float speed) {
    //Ensure Motors Enabled
    enableWheels();
    //Set Distance
    targetDistance = distance;
    //Set buggy state and flag
    state = MOVE_FORWARD;
    movementCompleted = false;
    //Reset Encoders
    resetEncoders();
    //Set Speed
    basespeed = speed;
    //Reset Debug if needed
    if (squareCompleted){
        bluetooth.resetDebugData();
    }
}

void ControlSystem::turn(float angle, float speed) {
    //Ensure Wheels Enabled
    enableWheels();
    //Set turn distance
    turnDirection = (angle > 0) ? 1 : -1;
    targetDistance = (_track_width / 2.0f) * (fabs(angle) * 3.14159f / 180.0f);
    //Set buggy state and flag
    state = TURNING;
    movementCompleted = false;
    //Stop wheels
    //stopWheels(); //might be needed
    //reset encoder values
    resetEncoders();
    //Set speed
    basespeed = speed;
    //Reset Debug if needed
    if (squareCompleted){
        bluetooth.resetDebugData();
    }}

void ControlSystem::follow(const FollowParams& params) {
    enableWheels();
    resetEncoders();
    basespeed = params.speed;
    state = FOLLOW_LINE;       // Set the state to FOLLOW_LINE
    movementCompleted = false;
    pidLine.reset();
}

void ControlSystem::moveSquare(float distance, float speed, float left_turn_multiplier, float right_turn_multiplier) {
    //reset debug data
    if (squareCompleted) { 
        bluetooth.resetDebugData();
    }
    //Set States & flags
    state = MOVING_SQUARE;
    squareState = MOVE_FORWARD_SQUARE;
    squareCompleted = false;
    sideCount = 0;
    retracing = false;
    moving = false;
    //Ensure Wheels stopped
    stopWheels();
    //Ensure Wheels Enabled
    enableWheels();
    //Reset Encoders
    resetEncoders();
    //Set speed distance and turn multiplers
    square_distance = distance;
    square_speed = speed;
    square_left_turn_multiplier = left_turn_multiplier;
    square_right_turn_multiplier = right_turn_multiplier;
    //Reset Debug
    bluetooth.resetDebugData();
}



void ControlSystem::setBangBangMode(const BangBangParams& params) {
    // Ensure wheels are enabled before setting control mode
    enableWheels();
    // Store the given parameters
    basespeed = params.baseSpeed;
    turnSpeedMultiplier = params.turnSpeedMultiplier;

    // Set control state
    state = BANG_BANG;
    movementCompleted = false;

    // Reset encoders (if needed for tracking)
    resetEncoders();
    bluetooth.resetDebugData();
}

void ControlSystem::setBangBangProportionalMode(const BangBangProportionalParams& params) {
    // Ensure wheels are enabled before setting control mode
    enableWheels();

    // Store the given parameters
    basespeed = params.baseSpeed;
    bangBangThreshold = params.bangBangThreshold;
    kP = params.kP;
    maxMultiplier = params.maxMultiplier;

    // Set control state
    state = BANG_BANG_PROPORTIONAL;
    movementCompleted = false;

    // Reset encoders (if needed for tracking)
    resetEncoders();
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
        case MOVING_SQUARE:
            processSquare();
            break;
        case IDLE:
            stopWheels();
            pidTimer.stop();
            pidTimer.reset(); 
            timerStarted = false;
            break;
        case FOLLOW_LINE:
             processFollowLine(dt);
             break;
        case BANG_BANG_PROPORTIONAL:
            processBangBangProportional();
            break;
        case BANG_BANG:
            processBangBang();
            break;
    }
}

bool ControlSystem::isMovementComplete() {
    return movementCompleted;
}

bool ControlSystem::isSquareComplete(){
    return squareCompleted;
}

void ControlSystem::setModePIDParameters(const SquarePatternParams* squareParams, 
                                      const StraightLineParams* straightlineParams, 
                                      const TurnAngleParams* turnangleParams,
                                      const FollowParams* followParams) {
    // Apply Square Pattern parameters if provided
    if (squareParams) {
        //Forward PID constants
        pidForward.setKp(squareParams->forward_pid_kp);
        pidForward.setKi(squareParams->forward_pid_ki);
        pidForward.setKd(squareParams->forward_pid_kd);
        pidForward.setScalingMultiplier(squareParams->scaling_forward);
        
        //Turn Pid Constants
        pidTurn.setKp(squareParams->turn_pid_kp);
        pidTurn.setKi(squareParams->turn_pid_ki);
        pidTurn.setKd(squareParams->turn_pid_kd);
        pidTurn.setScalingMultiplier(squareParams->scaling_turn);
    }

    // Apply Straight Line parameters if provided
    if (straightlineParams) {
        pidForward.setKp(straightlineParams->forward_pid_kp);
        pidForward.setKi(straightlineParams->forward_pid_ki);
        pidForward.setKd(straightlineParams->forward_pid_kd);
        pidForward.setScalingMultiplier(straightlineParams->scaling_forward);
    }

    // Apply Turn Angle parameters if provided
    if (turnangleParams) {
        pidTurn.setKp(turnangleParams->turn_pid_kp);
        pidTurn.setKi(turnangleParams->turn_pid_ki);
        pidTurn.setKd(turnangleParams->turn_pid_kd);
        pidTurn.setScalingMultiplier(turnangleParams->scaling_turn);
    }

    if (followParams) {
        pidLine.setKp(followParams->pid_kp);
        pidLine.setKi(followParams->pid_ki);
        pidLine.setKd(followParams->pid_kd);
        pidLine.setScalingMultiplier(followParams->pid_scaling);
    }
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
    bluetooth.logDebugData(leftDistance,rightDistance,error,pidOutput,multiplier);
    if (((leftDistance + rightDistance) / 2.0f) >= targetDistance) {
        stopWheels();
        pidForward.reset();
        movementCompleted = true;
        if (squareCompleted){
            state = IDLE;
        }
        else{
            state = MOVING_SQUARE;
        }
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
    bluetooth.logDebugData(leftDistance,rightDistance,error,pidOutput,multiplier);
    if (fabs(error) < 0.02f || avgDistance >= targetDistance) {
        stopWheels();
        pidTurn.reset();
        movementCompleted = true;
        if (squareCompleted){
            state = IDLE;
        }
        else{
            state = MOVING_SQUARE;
        }
    }
}

void ControlSystem::processSquare() {
    switch (squareState) {
        case MOVE_FORWARD_SQUARE:
            // Handle square movement logic here
            if (!moving) {
                // Move forward one side of the square
                moveForward(square_distance, square_speed);
                moving = true;
            }
            if (isMovementComplete()) {  // Wait for the movement to complete
                wait(1);
                // Determine whether to turn left or right
                squareState = retracing ? TURN_RIGHT_SQUARE : TURN_LEFT_SQUARE;
                moving = false;
            }
            break;

        case TURN_LEFT_SQUARE:
            // Turn left after moving forward (square pattern)
            if (!moving) {
                turn(90*square_left_turn_multiplier, square_speed);  // Turn 90° left
                moving = true;
            }
            if (isMovementComplete()) {
                sideCount++;  // Increment side count after a turn
                moving = false;
                squareState = (sideCount < 4) ? MOVE_FORWARD_SQUARE : TURN_AROUND_SQUARE;
            }
            break;

        case TURN_RIGHT_SQUARE:
            // Turn right while retracing
            if (!moving) {
                turn(-90*square_right_turn_multiplier, square_speed);  // Turn 90° right (opposite direction for retracing)
                moving = true;
            }
            if (isMovementComplete()) {
                sideCount++;  // Increment side count after a turn
                moving = false;
                squareState = (sideCount < 4) ? MOVE_FORWARD_SQUARE : STOP_SQUARE;
            }
            break;

        case TURN_AROUND_SQUARE:
            // Turn around (90) after completing the square
            if (!moving) {
                turn(90*square_left_turn_multiplier, square_speed);  // Turn 180° to prepare for retracing
                moving = true;
            }
            if (isMovementComplete()) {
                retracing = true;  // Start retracing
                sideCount = 0;  // Reset side count for retracing
                squareState = MOVE_FORWARD_SQUARE;  // Start moving forward again, but retracing
                moving = false;
            }
            break;

        case STOP_SQUARE:
            // Stop after retracing all sides
            if (isMovementComplete()){
                retracing = false;
                moving = false;
                sideCount = 0;
                squareCompleted = true;
                state = IDLE;
                squareState = IDLE_SQUARE;
                stopWheels();
            }
            break;
    }
}




void ControlSystem::processFollowLine(float dt) {
    // Implement line following algorithm here
    
    //for debugging
    float* sensorValues = sensorArray.getValues();  // Get pointer to array
    float leftDistance = fabs(leftWheel.encoder.getDistance());
    float rightDistance = fabs(rightWheel.encoder.getDistance());


    float error = sensorArray.getError();

    if (error>=900){
        stopWheels();
        movementCompleted = true;
        state = IDLE;
    };
    float pidOutput = pidLine.update(error, dt);  //kp,ki,kd need test after td2
    float multiplier = 1.0f - (pidOutput);
    
    leftWheel.setSpeed(basespeed * multiplier);
    rightWheel.setSpeed(basespeed);
    //For Debugging
    bluetooth.logDebugData(leftDistance,rightDistance,error,pidOutput,multiplier,sensorValues);
}


void ControlSystem::processBangBang() {
    float error = sensorArray.getError();  // Get error from sensor array
    static int lostCount = 0;
    if (error>=900){
        if (lostCount>1000){
            stopWheels();
            movementCompleted = true;
            state = IDLE;
            lostCount = 0;
        }
        lostCount++;
    }else{
    if (error > 0.6) {
        // Turn right sharply
        leftWheel.setSpeed(basespeed);
        rightWheel.setSpeed(basespeed * turnSpeedMultiplier);  // Slow down right wheel
    } 
    else if (error < 0.6) {
        // Turn left sharply
        leftWheel.setSpeed(basespeed * turnSpeedMultiplier);  // Slow down left wheel
        rightWheel.setSpeed(basespeed);
    } 
    else {
        // Drive straight
        leftWheel.setSpeed(basespeed);
        rightWheel.setSpeed(basespeed);
    }
    lostCount=0;
    }
}

void ControlSystem::processBangBangProportional() {
    float error = sensorArray.getError();  // Get error from sensor array

    if (error>=900){
        stopWheels();
        movementCompleted = true;
        state = IDLE;
    }

    if (fabs(error) > bangBangThreshold) {
        // Use bang-bang for large errors
        if (error > 0) {
            leftWheel.setSpeed(basespeed);
            rightWheel.setSpeed(basespeed * 0.5);
        } else {
            leftWheel.setSpeed(basespeed * 0.5);
            rightWheel.setSpeed(basespeed);
        }
    } else {
        // Use proportional control for small errors
        float multiplier = 1.0f + kP * error;

        // Clamp the multiplier
        if (multiplier > maxMultiplier) multiplier = maxMultiplier;
        if (multiplier < 1.0f / maxMultiplier) multiplier = 1.0f / maxMultiplier;

        leftWheel.setSpeed(basespeed);
        rightWheel.setSpeed(basespeed * multiplier);
    }
}


void ControlSystem::stopWheels() {
    leftWheel.stop();
    rightWheel.stop();
    basespeed = 0;
    disableWheels(); //Disable for safety if causes issues remove untested
}

void ControlSystem::enableWheels(){
    leftWheel.enableMotor();
}
void ControlSystem::disableWheels(){
    leftWheel.disableMotor();
    rightWheel.disableMotor();
}

void ControlSystem::resetEncoders(){
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();
}

void ControlSystem::regulateWheelSpeed(){
    leftWheel.regulateSpeed();
    rightWheel.regulateSpeed();
}

void ControlSystem::debugRegulateWheelSpeed(){
    static int counter = 0;
    leftWheel.regulateSpeed();
    rightWheel.regulateSpeedDebug();
    if (counter==150){
        bluetooth.printMotorDebugData(leftWheel.encoder.getSpeed(),rightWheel.encoder.getSpeed());
        counter=0;
    }
    counter++;
}

void ControlSystem::setWheelKp(float wheelKp){
    leftWheel.setKp(wheelKp);
    rightWheel.setKp(wheelKp);
}