#include "BuggyModes.h"
#include "ControlSystem.h"
#include "Bluetooth.h"

// Initialize default parameters
SquarePatternParams squareParams = {
    2.3, 0.6, 0.00,   // pid_kp1, pid_ki1, pid_kd1
    2.3, 0.6, 0.00,   // pid_kp2, pid_ki2, pid_kd2
    2.2,              // scaling_forward
    2.2,              // scaling_turn
    1.1, 1.0,         // left_turn_multiplier, right_turn_multiplier
    0.5,              // distance
    90.0                // speed
};

StraightLineParams straightlineParams = {
    2.3, 0.6, 0.00,   // pid_kp, pid_ki, pid_kd
    2.2,              // scaling_forward
    0.0,              // speed
    0.0             // distance
};

TurnAngleParams turnangleParams = {
    2.3, 0.6, 0.00,   // pid_kp, pid_ki, pid_kd
    2.2,              // scaling_turn
    0.0,              // speed
    0.0              // angle
};

// Function to get mode name as a string
const char* getModeName(BuggyMode mode) {
    switch (mode) {
        case idle_mode: return "IDLE";
        case speed_control_mode: return "SPEED_CONTROL";
        case square_idle_mode: return "SQUARE_IDLE";
        case line_menu_mode: return "LINE_MENU";
        case turn_menu_mode: return "TURN_MENU";
        case waiting_for_movement: return "WAITING_FOR_MOVEMENT";
        default: return "UNKNOWN_MODE";
    }
}

BuggyMode buggyMode = idle_mode;

void switchToSpeedControlMode(ControlSystem& control, BuggyMode& buggyMode) {
    //Needs to be adjusted to use controlSystem
    //leftWheel.motor.enable();
    buggyMode = speed_control_mode;
    //leftWheel.encoder.reset();
    //rightWheel.encoder.reset();
}

void switchToSquareIdleMode(BuggyMode& buggyMode) {
    //leftWheel.motor.disable();
    buggyMode = square_idle_mode;
}

//Function to start the square movement
void switchToSquarePatternMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, bool& square_flag) {
    //Wait
    wait(1);//could maybe remove this
    //Set Mode and square flag
    square_flag = true;
    buggyMode = waiting_for_movement;
    //Set Relevant Parameters
    control.setModePIDParameters(&squareParams,NULL,NULL);
    //Begin Control
    controlticker.attach(callback(&control, &ControlSystem::update), 0.1);//add interval to square params
    //Begin Movement
    control.moveSquare(squareParams.distance,squareParams.speed,squareParams.left_turn_multiplier,squareParams.right_turn_multiplier);
}

void switchToLineMenuMode(BuggyMode& buggyMode) {
    buggyMode = line_menu_mode;
}

void switchToTurnMenuMode(BuggyMode& buggyMode) {
    buggyMode = turn_menu_mode;
}

//Function to switch the buggy to line mode
void switchToLineMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker){
    //Set Relevant PID Parameters
    control.setModePIDParameters(NULL,&straightlineParams,NULL); //I want nullptr but outdated c++
    //Begin Control
    control.moveForward(straightlineParams.distance,straightlineParams.speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

//Function to switch the buggy to turn mode
void switchToTurnMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker){
    //Set Relevant PID Parameters
    control.setModePIDParameters(NULL,NULL,&turnangleParams);
    //Begin Control
    control.turn(turnangleParams.angle,turnangleParams.speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

void stopMotorAndSwitchToIdleMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker,bool& square_flag) {
    //Set buggy mode
    buggyMode = idle_mode;
    //Detach control ticker
    controlticker.detach();
    //Reset square_flag
    square_flag = false;
}