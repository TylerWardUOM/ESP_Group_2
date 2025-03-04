#include "BuggyModes.h"
#include "ControlSystem.h"
#include "Bluetooth.h"


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
void switchToSquarePatternMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, bool& square_flag, SquarePatternParams& params) {
    //Wait
    wait(1);//could maybe remove this
    //Set Mode and square flag
    square_flag = true;
    buggyMode = waiting_for_movement;
    //Set Relevant Parameters
    control.setModePIDParameters(&params,NULL,NULL);
    //Begin Control
    controlticker.attach(callback(&control, &ControlSystem::update), 0.1);//add interval to square params
    //Begin Movement
    control.moveSquare(params.distance,params.speed,params.left_turn_multiplier,params.right_turn_multiplier);
}

void switchToLineMenuMode(BuggyMode& buggyMode) {
    buggyMode = line_menu_mode;
}

void switchToTurnMenuMode(BuggyMode& buggyMode) {
    buggyMode = turn_menu_mode;
}

//Function to switch the buggy to line mode
void switchToLineMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, StraightLineParams& params){
    //Set Relevant PID Parameters
    control.setModePIDParameters(NULL,&params,NULL); //I want nullptr but outdated c++
    //Begin Control
    control.moveForward(params.distance,params.speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

//Function to switch the buggy to turn mode
void switchToTurnMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, TurnAngleParams& params){
    //Set Relevant PID Parameters
    control.setModePIDParameters(NULL,NULL,&params);
    //Begin Control
    control.turn(params.angle,params.speed);
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