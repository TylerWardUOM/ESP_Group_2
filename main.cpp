#include "mbed.h"
#include "C12832.h"
#include "Motor.h"
#include "Encoder.h"
#include "Potentiometer.h"
#include "LED.h"

// Constants for the square movement
#define WHEEL_DIAMETER 0.075f       // wheel diameter in meters
#define ENCODER_RESOLUTION 1        // encoder resolution (1, 2, or 4)
#define TRACK_WIDTH 0.28f          // distance between wheels (meters)
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

// Pin Assignments for Motor Control
Motor leftMotor(PB_7, 1.22, PB_14, PA_14);    // (Bipolar1, Dir1, PWM1, Enable)
Motor rightMotor(PB_15, 1, PB_13, PA_14);      // (Bipolar2, Dir2, PWM2, Enable)

// Encoder Pins
Encoder leftEncoder(PA_13, PB_6, WHEEL_DIAMETER, ENCODER_RESOLUTION);
Encoder rightEncoder(PB_2, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION);

// Potentiometer Pins
Potentiometer potentiometerLeft(A0, 3.3);   // Left potentiometer pin
Potentiometer potentiometerRight(A1, 3.3);  // Right potentiometer pin

// LCD and LED Pins
C12832 lcd(D11, D13, D12, D7, D10);  // LCD display
LED redLED(D5);
LED greenLED(D9);
LED blueLED(D8);

// Button Pins
InterruptIn fire(D4);  // Interrupt for the center button on the joystick
InterruptIn up(A2);    // Interrupt for the up button
InterruptIn down(A3);  // Interrupt for the down button

// Motor State Enum
typedef enum {
    idle_mode,
    speed_control_mode,
    square_idle_mode,   // idle state for square pattern
    square_pattern_mode
} MotorState;

MotorState motorState = idle_mode;

int sideCount = 0;
const float SQUARE_DISTANCE = 0.5f;  // distance for each side in meters

// Movement State Enum for square movement
typedef enum {
    MOVE_FORWARD,
    TURN_RIGHT,
    TURN_LEFT,
    TURN_AROUND,
    STOP
} MovementState;

//
// PID Controller Class
//
class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd), integral(0.0f), prevError(0.0f) {}

    float update(float error, float dt) {
        integral += error * dt;
        float derivative = (error - prevError) / dt;
        prevError = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void reset() {
        integral = 0.0f;
        prevError = 0.0f;
    }

private:
    float kp, ki, kd;
    float integral;
    float prevError;
};

// Global PID Controllers for different states
// (Tune these parameters as needed)
PIDController pidForward(0.6f, 0.03f, 0.13f);   // used during straight movement
PIDController pidTurn(0.4f, 0.0f, 0.0f);         // used during turning

// Timer for PID dt calculations
Timer pidTimer;

//
// Function Prototypes
//
void stopMotorAndSwitchToIdleMode();
void switchToSpeedControlMode();
void switchToSquareIdleMode();
void switchToSquarePatternMode();
void updateSquareMovement();
void updateLCD();

// Update LCD periodically using a Ticker
Ticker lcdUpdateTicker; 
bool lcdUpdateRequired = false;
void updateLCD() {
    lcdUpdateRequired = true;
}

// LED Control Functions
void setLEDgreen(LED red, LED green, LED blue) {
    red.off();
    green.on();
    blue.off();
}

void setLEDred(LED red, LED green, LED blue) {
    red.on();
    green.off();
    blue.off();
}

void setLEDblue(LED red, LED green, LED blue) {
    red.off();
    green.off();
    blue.on();
}

void turnoffLED(LED red, LED green, LED blue) {
    red.off();
    green.off();
    blue.off();
}

//
// Square Movement Function (with full PID control)
//
#define MULTIPLIER_SCALING_FACTOR 2.2f  // Adjust this value for tuning

void updateSquareMovement() {
    static float targetDistance = SQUARE_DISTANCE;
    static MovementState state = MOVE_FORWARD;
    static bool retracing = false;  // Indicates if we are in the retrace phase

    // Initialize and update PID timer for dt calculation
    static bool timerStarted = false;
    if (!timerStarted) {
        pidTimer.start();
        timerStarted = true;
    }
    float currentTime = pidTimer.read(); // seconds
    static float lastTime = currentTime;
    float dt = currentTime - lastTime;
    if (dt <= 0.0f) dt = 0.01f; // prevent division by zero
    lastTime = currentTime;

    switch (state) {
        case MOVE_FORWARD: {
            float baseSpeed = 0.28f;  // Lowered base speed for forward movement
            float leftDistance = leftEncoder.getDistance();
            float rightDistance = rightEncoder.getDistance();

            // Compute the error (positive means left moved more than right)
            float error = rightDistance - leftDistance;
            float pidOutput = pidForward.update(error, dt);

            // Apply scaling factor to PID output
            float multiplier = 1.0f - (pidOutput * MULTIPLIER_SCALING_FACTOR);
            if (multiplier < 0.55f) multiplier = 0.6f;
            if (multiplier > 1.5f) multiplier = 1.4f;

            float leftSpeed = baseSpeed;
            float rightSpeed = baseSpeed * multiplier;

            leftMotor.setSpeed(leftSpeed);
            rightMotor.setSpeed(rightSpeed);

            printf("MOVE_FORWARD | L: %.2f | R: %.2f | Multiplier: %.2f | Error: %.2f\n",
                leftDistance, rightDistance, multiplier, error);

            if (((leftDistance + rightDistance) / 2.0f) >= targetDistance) {
                leftMotor.stop();
                rightMotor.stop();
                leftEncoder.reset();
                rightEncoder.reset();
                pidForward.reset();
                targetDistance = TURN_DISTANCE;
                state = retracing ? TURN_RIGHT : TURN_LEFT;
                wait(0.5);
            }
            break;
        }

        case TURN_LEFT: {
            float leftDistance = fabs(leftEncoder.getDistance());
            float rightDistance = fabs(rightEncoder.getDistance());
            float avgDistance = (leftDistance + rightDistance) / 2.0f;

            float error = (targetDistance / 2) - avgDistance;
            float pidOutput = pidTurn.update(error, dt);

            // Apply scaling factor to PID output
            float multiplier = 1.0f - (pidOutput * MULTIPLIER_SCALING_FACTOR);
            if (multiplier < 0.7f) multiplier = 0.7f;
            if (multiplier > 1.3f) multiplier = 1.3f;

            float baseTurnSpeed = 0.2f;  // Lowered base turn speed
            float leftSpeed = -baseTurnSpeed * multiplier;
            float rightSpeed = baseTurnSpeed;

            leftMotor.setSpeed(leftSpeed);
            rightMotor.setSpeed(rightSpeed);

            printf("TURN_LEFT | Target: %.2f | L: %.2f | R: %.2f | Multiplier: %.2f | Error: %.2f\n",
                targetDistance, leftDistance, rightDistance, multiplier, error);

            if (fabs(error) < 0.02f || avgDistance >= targetDistance) {
                leftMotor.stop();
                rightMotor.stop();
                leftEncoder.reset();
                rightEncoder.reset();
                wait(0.5);
                pidTurn.reset();

                sideCount++;
                if (sideCount < 4) {
                    targetDistance = SQUARE_DISTANCE;
                    state = MOVE_FORWARD;
                } else {
                    state = TURN_AROUND;
                }
            }
            break;
        }

        case TURN_RIGHT: {
            float leftDistance = fabs(leftEncoder.getDistance());
            float rightDistance = fabs(rightEncoder.getDistance());
            float avgDistance = (leftDistance + rightDistance) / 2.0f;

            float error = (targetDistance/2) - avgDistance;
            float pidOutput = pidTurn.update(error, dt);

            // Apply scaling factor to PID output
            float multiplier = 1.0f - (pidOutput * MULTIPLIER_SCALING_FACTOR);
            if (multiplier < 0.7f) multiplier = 0.7f;
            if (multiplier > 1.3f) multiplier = 1.3f;

            float baseTurnSpeed = 0.2f;  // Lowered base turn speed
            float leftSpeed = baseTurnSpeed;
            float rightSpeed = -baseTurnSpeed * multiplier;

            leftMotor.setSpeed(leftSpeed);
            rightMotor.setSpeed(rightSpeed);

            printf("TURN_RIGHT | Target: %.2f | L: %.2f | R: %.2f | Multiplier: %.2f | Error: %.2f\n",
                targetDistance, leftDistance, rightDistance, multiplier, error);

            if (fabs(error) < 0.02f || avgDistance >= targetDistance) {
                leftMotor.stop();
                rightMotor.stop();
                leftEncoder.reset();
                rightEncoder.reset();
                wait(0.5);
                pidTurn.reset();

                sideCount++;
                if (sideCount < 4) {
                    targetDistance = SQUARE_DISTANCE;
                    state = MOVE_FORWARD;
                } else {
                    state = STOP;
                }
            }
            break;
        }

        case TURN_AROUND: {
            float leftDistance = fabs(leftEncoder.getDistance());
            float rightDistance = fabs(rightEncoder.getDistance());
            float avgDistance = (leftDistance + rightDistance) / 2.0f;

            float targetTurn = TURN_DISTANCE / 2;
            float error = targetTurn - avgDistance;
            float turnSpeed = pidTurn.update(error, dt);

            if (turnSpeed > 0.35f) turnSpeed = 0.35f;
            if (turnSpeed < 0.25f) turnSpeed = 0.25f;

            leftMotor.setSpeed(-turnSpeed);
            rightMotor.setSpeed(turnSpeed);

            printf("TURN_AROUND | Target: %.2f | L: %.2f | R: %.2f | Avg: %.2f | Error: %.2f | Speed: %.2f\n",
                targetTurn, leftDistance, rightDistance, avgDistance, error, turnSpeed);

            if (fabs(error) < 0.02f || avgDistance >= targetTurn) {
                leftMotor.stop();
                rightMotor.stop();
                leftEncoder.reset();
                rightEncoder.reset();
                wait(1);
                pidTurn.reset();

                retracing = true;
                sideCount = 0;
                targetDistance = SQUARE_DISTANCE;
                state = MOVE_FORWARD;
            }
            break;
        }

        case STOP: {
            leftMotor.stop();
            rightMotor.stop();
            retracing = false;
            sideCount = 0;
            targetDistance = SQUARE_DISTANCE;
            state = MOVE_FORWARD;

            lcd.cls();
            lcd.locate(0, 0);
            lcd.printf("Square Complete, Idle Mode");
            stopMotorAndSwitchToIdleMode();
            break;
        }
    }
}



//
// Mode Switching Functions
//
void switchToSpeedControlMode() {
    leftMotor.enable();
    motorState = speed_control_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Speed Control Mode");
    leftEncoder.reset();
    rightEncoder.reset();

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    fire.fall(callback(&stopMotorAndSwitchToIdleMode));
}

void switchToSquareIdleMode() {
    leftMotor.disable();
    motorState = square_idle_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Square Idle Mode");

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    up.fall(callback(&stopMotorAndSwitchToIdleMode));
    down.fall(callback(&stopMotorAndSwitchToIdleMode));
    fire.fall(callback(&switchToSquarePatternMode));
}

void switchToSquarePatternMode() {
    motorState = square_pattern_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Square Pattern Mode");

    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);
    wait(1);
    leftMotor.stop();
    rightMotor.stop();
    leftEncoder.reset();
    rightEncoder.reset();
    leftMotor.enable();
}

void stopMotorAndSwitchToIdleMode() {
    leftMotor.disable();
    leftMotor.stop();
    rightMotor.stop();
    motorState = idle_mode;
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("Idle Mode");

    up.fall(callback(&switchToSpeedControlMode));
    down.fall(callback(&switchToSquareIdleMode));
    fire.fall(NULL);
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//
// Main Loop
//
int main() {
    lcdUpdateTicker.attach(&updateLCD, 0.7);

    leftMotor.stop();
    rightMotor.stop();
    up.fall(callback(&switchToSpeedControlMode));
    down.fall(callback(&switchToSquareIdleMode));
    fire.fall(NULL);

    while (true) {
        switch (motorState) {
            case idle_mode:
                setLEDred(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.locate(0, 0);
                    lcd.printf("Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case speed_control_mode:
                setLEDgreen(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(25,8);
                    lcd.printf("Speed Control Mode");
                    lcd.locate(8, 17);
                    lcd.printf("L=%.2f R=%.2f", leftEncoder.getDistance(), rightEncoder.getDistance());
                    lcdUpdateRequired = false;
                }
                potentiometerLeft.sample();
                potentiometerRight.sample();
                float speedRawL = potentiometerLeft.getCurrentSampleNorm();
                float speedRawR = potentiometerRight.getCurrentSampleNorm();
                leftMotor.setSpeed(speedRawL);
                rightMotor.setSpeed(speedRawR);
                break;

            case square_idle_mode:
                setLEDblue(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(0, 0);
                    lcd.printf("Square Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case square_pattern_mode:
                setLEDblue(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.locate(0, 0);
                    lcd.printf("Square Pattern Mode");
                    lcdUpdateRequired = false;
                }
                updateSquareMovement();  // perform square movement with PID control
                break;

            default:
                break;
        }
    }
}
