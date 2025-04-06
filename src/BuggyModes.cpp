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
    control.setModePIDParameters(&params,NULL,NULL,NULL);
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
void switchToFollowMenuMode(BuggyMode& buggyMode) {
    buggyMode = follow_menu_mode;
}

void switchToBangBangMenuMode(BuggyMode& buggyMode) {
    buggyMode = bang_bang_menu_mode;
}

void switchToBangBangBoostMenuMode(BuggyMode& buggyMode) {
    buggyMode = bang_bang_boost_menu_mode;
}

void switchToBangBangProportionalMenuMode(BuggyMode& buggyMode) {
    buggyMode = bang_bang_proportional_menu_mode;
}


void switchToSensorDebug(BuggyMode& buggyMode, Ticker& sensorTicker, SensorArray& sensorArray){
    wait(1);
    sensorTicker.attach(callback(&sensorArray, &SensorArray::live_debugSensorData), 0.2);
    buggyMode = sensor_debug;
}

void switchToMotorDebug(BuggyMode& buggyMode, Ticker& motorTicker, ControlSystem& control){
    motorTicker.attach(callback(&control, &ControlSystem::debugRegulateWheelSpeed), 0.01);
    buggyMode = motor_debug;
}


//Function to switch the buggy to line mode
void switchToLineMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, StraightLineParams& params){
    //Set Relevant PID Parameters
    control.setModePIDParameters(NULL,&params,NULL,NULL); //I want nullptr but outdated c++
    //Begin Control
    control.moveForward(params.distance,params.speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

//Function to switch the buggy to turn mode
void switchToTurnMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, TurnAngleParams& params){
    //Set Relevant PID Parameters
    control.setModePIDParameters(NULL,NULL,&params,NULL);
    //Begin Control
    control.turn(params.angle,params.speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

//Function to switch the buggy to turn around 180
void switchToTurnAround(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker, TurnAngleParams& params){
    //Set Relevant PID Parameters
    control.setModePIDParameters(NULL,NULL,&params,NULL);
    //Begin Control
    control.turn(180,params.speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

//Function to switch the buggy to turn mode
void switchToFollowMode(ControlSystem& control,  SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker,Ticker& motorTicker, FollowParams& params){
    //Set Relevant PID Parameters
    control.setWheelKp(params.motor_Kp);
    control.setModePIDParameters(NULL,NULL,NULL,&params);
    //Begin Control
    control.follow(params);
    controlticker.attach(callback(&control, &ControlSystem::update), params.controlPeriod);
    sensorTicker.attach(callback(&sensorArray, &SensorArray::sample), params.sensorSamplePeriod);
    motorTicker.attach(callback(&control, &ControlSystem::regulateWheelSpeed), params.motorRegulatePeriod);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

void switchToBangBangMode(ControlSystem& control,  SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker,Ticker& motorTicker, BangBangParams& params){
    control.setWheelKp(params.motor_Kp);
    control.setWheelKi(params.motor_Ki);
    control.setBangBangMode(params);
    controlticker.attach(callback(&control, &ControlSystem::update), params.controlPeriod);
    sensorTicker.attach(callback(&sensorArray, &SensorArray::sample), params.sensorSamplePeriod);
    motorTicker.attach(callback(&control, &ControlSystem::regulateWheelSpeed), params.motorRegulatePeriod);
    buggyMode = waiting_for_movement;
}

void switchToBangBangBoostMode(ControlSystem& control,  SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker,Ticker& motorTicker, BangBangBoostParams& params){
    control.setWheelParams(params.motor_Kp,params.motor_threshold,params.motor_boost,params.motor_Ki);
    control.setBangBangBoostMode(params);
    controlticker.attach(callback(&control, &ControlSystem::update), params.controlPeriod);
    sensorTicker.attach(callback(&sensorArray, &SensorArray::sample), params.sensorSamplePeriod);
    motorTicker.attach(callback(&control, &ControlSystem::regulateWheelSpeedwithBOOST), params.motorRegulatePeriod);
    buggyMode = waiting_for_movement;
}

void switchToBangBangProportionalMode(ControlSystem& control, SensorArray& sensorArray, BuggyMode& buggyMode, Ticker& controlticker, Ticker& sensorTicker,Ticker& motorTicker, BangBangProportionalParams& params){
    control.setWheelParams(params.motor_Kp,params.motor_threshold,params.motor_boost);
    control.setBangBangProportionalMode(params);
    controlticker.attach(callback(&control, &ControlSystem::update), params.controlPeriod);
    sensorTicker.attach(callback(&sensorArray, &SensorArray::sample), params.sensorSamplePeriod);
    motorTicker.attach(callback(&control, &ControlSystem::regulateWheelSpeed), params.motorRegulatePeriod);
    buggyMode = waiting_for_movement;
}

void stopMotorAndSwitchToIdleMode(ControlSystem& control, BuggyMode& buggyMode, Ticker& controlticker,Ticker& sensorTicker,Ticker& motorTicker,bool& square_flag) {
    //Set buggy mode
    buggyMode = idle_mode;
    //Detach control ticker
    control.stopWheels();
    controlticker.detach();
    sensorTicker.detach();
    motorTicker.detach();
    //Reset square_flag
    square_flag = false;
}

void switchToRCMode(ControlSystem& control, BuggyMode& buggyMode){
    buggyMode=rc;
    control.enableWheels();
}
