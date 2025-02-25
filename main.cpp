#include "Wheel.h"
#include "mbed.h"
#include "C12832.h"
#include "Motor.h"
#include "Encoder.h"
#include "Potentiometer.h"
#include "Wheel.h"
#include "PIDController.h"
#include "ControlSystem.h"
#include "RGBled.h"
#include "BuggyModes.h"
#include "Bluetooth.h"

//Constants for Wheel
//if not going correct distance adjust wheel_diameter
#define WHEEL_DIAMETER 0.078f       // wheel diameter in meters
#define ENCODER_RESOLUTION 1        // encoder resolution (1, 2, or 4)
#define MAX_RPM 300


//Constants for Buggy
//If not turning to set angle adjust track width
#define TRACK_WIDTH 0.20f          // distance between wheels (meters)
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

//Constants for Control
//If it doesnt go straight adjust these values
#define KP_FORWARD 2.3f
#define KI_FORWARD 0.6f
#define KD_FORWARD 0
#define SCALING_FORWARD 2.2f

//If turns arent consistent adjust these values
// If turns perfect one direction of square but not the other add a multipler to the turn angle in updatesquaremovement function
#define KP_TURN 2.3f
#define KI_TURN 0.6f
#define KD_TURN 0
#define SCALING_TURN 2.2f

//Try get this to smallest value possible if code stops working properly go back to previous
#define SQUARE_CONTROL_INTERVAL 0.01

//Maybe adjust the left wheel multiplier if you see a consitant drift in one direction
Wheel leftWheel(PB_7,1.10,PB_14,PA_14,PA_13,PB_8,WHEEL_DIAMETER,ENCODER_RESOLUTION,MAX_RPM); //change max rpm by testing
Wheel rightWheel(PB_15, 1, PB_13, PA_14,PB_2, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION,MAX_RPM);
// Global instance of ControlSystem
ControlSystem control(leftWheel, rightWheel, TRACK_WIDTH, KP_FORWARD, KI_FORWARD, KD_FORWARD, SCALING_FORWARD, KP_TURN, KI_TURN, KD_TURN, SCALING_TURN);

// Potentiometer Pins
Potentiometer potentiometerLeft(A0, 3.3);   // Left potentiometer pin
Potentiometer potentiometerRight(A1, 3.3);  // Right potentiometer pin

// LCD and LED Pins
C12832 lcd(D11, D13, D12, D7, D10);  // LCD display


//RGBLed LED(D5,D9,D8);


// Button Pins
InterruptIn fire(D4);  // Interrupt for the center button on the joystick
InterruptIn up(A2);    // Interrupt for the up button
InterruptIn down(A3);  // Interrupt for the down button
InterruptIn right(A5); 
InterruptIn left(A4);

//
// Function Prototypes
//
void stopMotorAndSwitchToIdleMode();
void switchToSpeedControlMode();
void switchToSquareIdleMode();
void switchToSquarePatternMode();
void switchToLineMenuMode();
void switchToTurnMenuMode();
void switchToLineMode();
void switchToTurnMode();
void updateSquareMovement();
void updateLCD();
long map(long x, long in_min, long in_max, long out_min, long out_max);

// Update LCD periodically using a Ticker
Ticker lcdUpdateTicker; 
bool lcdUpdateRequired = false;
void updateLCD() {
    lcdUpdateRequired = true;
}


Ticker controlticker;


// Movement State Enum for square movement
typedef enum {
    MOVE_FORWARD,   // State for moving forward
    TURN_RIGHT,
    TURN_LEFT,
    TURN_AROUND,     // State for turning right
    STOP            // State for stopping
} MovementState;



float SQUARE_DISTANCE = 0.5f;  // distance for each side in meters

void updateSquareMovement() {
    static MovementState state = MOVE_FORWARD;  // Start with moving forward
    static bool retracing = false;  // Track whether we're retracing
    static int sideCount = 0;  // Track the number of sides completed
    static float basespeed = 0.30f * MAX_RPM;
    static bool moving = false;  // Corrected variable name

    switch (state) {
        case MOVE_FORWARD:
            // Move forward a fixed distance (one side of the square)
            if (!moving) {
                control.moveForward(SQUARE_DISTANCE, basespeed);  // Move forward one side
                moving = true;
            }
            if (control.isMovementComplete()) {  // Wait for the movement to complete
                wait(1);
                state = retracing ? TURN_RIGHT : TURN_LEFT;  // Decide direction after moving forward
                moving = false;
            }
            break;

        case TURN_LEFT:  // Turn left after moving forward (square pattern)
            if (!moving) {
                control.turn(90*1.0f, basespeed);  // Turn 90° left
                moving = true;
            }
            if (control.isMovementComplete()) {
                wait(1);
                sideCount++;  // Increment side count after a turn
                moving = false;
                state = (sideCount < 4) ? MOVE_FORWARD : TURN_AROUND;  // Move forward until all sides are completed, then turn around
            }
            break;

        case TURN_RIGHT:  // Turn right while retracing
            if (!moving) {
                control.turn(-90*1.0f, basespeed);  // Turn 90° right (opposite direction for retracing)
                moving = true;
            }
            if (control.isMovementComplete()) {
                wait(1);
                sideCount++;  // Increment side count after a turn
                moving = false;
                state = (sideCount < 4) ? MOVE_FORWARD : STOP;  // Move forward or stop if all sides are retraced
            }
            break;

        case TURN_AROUND:  // Turn around (180°) after completing the square
            if (!moving) {
                control.turn(90*1.0f, basespeed);  // Turn 180° to prepare for retracing
                moving = true;
            }
            if (control.isMovementComplete()) {
                wait(1);
                retracing = true;  // Start retracing
                sideCount = 0;  // Reset side count for retracing
                state = MOVE_FORWARD;  // Start moving forward again, but retracing
                moving = false;
            }
            break;

        case STOP:  // Stop after retracing all sides
            retracing = false;
            moving=false;
            sideCount = 0;
            state = MOVE_FORWARD;
            lcd.cls();
            lcd.locate(0, 0);
            lcd.printf("Square Complete, Idle Mode");
            stopMotorAndSwitchToIdleMode();  // Call to stop the motors and switch to idle mode
            break;
    }
}




// Bluetooth Instance
Serial btSerial(USBTX,USBRX); //temp to test functionality
Bluetooth bluetooth(btSerial, &buggyMode, &squareParams, &straightlineParams, &turnangleParams);

void switchToSpeedControlMode() {
    leftWheel.motor.enable();
    buggyMode = speed_control_mode;
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();
}

void switchToSquareIdleMode() {
    leftWheel.motor.disable();
    buggyMode = square_idle_mode;
}

void switchToSquarePatternMode() {
    //Set Mode State
    buggyMode = waiting_for_movement;
    //Reset Encoders
    leftWheel.encoder.reset();
    rightWheel.encoder.reset();
    //Set Relevant Parameters
    //Still need to set the left and right turn multipliers
    control.pidForward.setKp(squareParams.forward_pid_kp);
    control.pidForward.setKi(squareParams.forward_pid_ki);
    control.pidForward.setKd(squareParams.forward_pid_kd);
    control.pidForward.setScalingMultiplier(squareParams.scaling_forward);
    control.pidTurn.setKp(squareParams.turn_pid_kp);
    control.pidTurn.setKi(squareParams.turn_pid_ki);
    control.pidTurn.setKd(squareParams.turn_pid_kd);
    control.pidTurn.setScalingMultiplier(squareParams.scaling_turn);
    SQUARE_DISTANCE = squareParams.distance;
    //Begin Control
    controlticker.attach(callback(&control, &ControlSystem::update), SQUARE_CONTROL_INTERVAL);
    //Enable Motor
    leftWheel.motor.enable();
}

void switchToLineMenuMode() {
    buggyMode = line_menu_mode;
    lcd.cls();
    lcd.locate(0,0);
    lcd.printf("Line Mode");
}

void switchToTurnMenuMode() {
    buggyMode = turn_menu_mode;
    lcd.cls();
    lcd.locate(0,0);
    lcd.printf("Turn Mode");
}

void switchToLineMode(){
    leftWheel.motor.enable();
    //Set Relevant Parameters
    control.pidForward.setKp(straightlineParams.forward_pid_kp);
    control.pidForward.setKi(straightlineParams.forward_pid_ki);
    control.pidForward.setKd(straightlineParams.forward_pid_kd);
    control.pidForward.setScalingMultiplier(straightlineParams.scaling_forward);
    float distance = straightlineParams.distance;
    float speed = straightlineParams.speed;
    //Begin Control
    control.moveForward(distance,speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

void switchToTurnMode(){
    //Enable Motor
    leftWheel.motor.enable();
    //Set Relevant Parameters
    control.pidTurn.setKp(turnangleParams.turn_pid_kp);
    control.pidTurn.setKi(turnangleParams.turn_pid_ki);
    control.pidTurn.setKd(turnangleParams.turn_pid_kd);
    control.pidTurn.setScalingMultiplier(turnangleParams.scaling_turn);
    float angle = turnangleParams.angle;
    float speed = turnangleParams.speed;
    //Begin Control
    control.turn(angle,speed);
    controlticker.attach(callback(&control, &ControlSystem::update), 0.05);
    //Update Mode state
    buggyMode = waiting_for_movement;
}

void stopMotorAndSwitchToIdleMode() {
    leftWheel.motor.disable();
    controlticker.detach();
    leftWheel.stop();
    rightWheel.stop();
    buggyMode = idle_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Idle Mode");
}

int main() {
    lcdUpdateTicker.attach(&updateLCD, 0.7);
    leftWheel.stop();
    rightWheel.stop();
    
    while (true) {
        bluetooth.processCommand(); // Process Bluetooth commands
        
        switch (buggyMode) {
            case idle_mode:
                lcd.locate(0, 0);
                lcd.printf("Idle Mode");
                break;

            case speed_control_mode:
                lcd.locate(0, 0);
                lcd.printf("Speed Control Mode");
                break;

            case square_idle_mode:
                lcd.locate(0, 0);
                lcd.printf("Square Idle Mode");
                if (bluetooth.shouldStart()){
                    switchToSquarePatternMode();
                    break;
                }
                break;

            case line_menu_mode:
                lcd.locate(0, 0);
                lcd.printf("Line Menu Mode");
                if (bluetooth.shouldStart()){
                    switchToLineMode();
                    break;
                }
                break;

            case turn_menu_mode:
                lcd.locate(0, 0);
                lcd.printf("Turn Menu Mode");
                if (bluetooth.shouldStart()){
                    switchToTurnMode();
                    break;
                }
                break;

            case waiting_for_movement:
                lcd.locate(0, 0);
                lcd.printf("Waiting for Movement");
                if (control.isMovementComplete()) {
                    controlticker.detach();
                    bluetooth.sendMovementFinished();
                    bluetooth.sendDebugData();
                    stopMotorAndSwitchToIdleMode();
                }
                break;
        }
    }
}
