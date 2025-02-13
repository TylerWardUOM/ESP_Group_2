#include "mbed.h"
#include "C12832.h"
#include "Motor.h"
#include "Encoder.h"
#include "Potentiometer.h"
#include "LED.h"

// Constants for the square movement
#define WHEEL_DIAMETER 0.037f  // 3.7 cm wheels i dont know it but it seems right maybe
#define ENCODER_RESOLUTION 1   // Encoder resolution (1, 2, or 4)
#define TRACK_WIDTH 0.2f       // Distance between wheels (meters) change to correct value
#define TURN_DISTANCE (3.1416f * TRACK_WIDTH / 4)  // 90-degree turn distance

// Pin Assignments for Motor Control
Motor leftMotor(PA_1, PA_2, PA_3, PA_7);  // (Bipolar1, Dir1, PWM1, Enable)
Motor rightMotor(PA_4, PA_5, PA_6, PA_7); // (Bipolar2, Dir2, PWM2, Enable)

// Encoder Pins
Encoder leftEncoder(PB_5, PB_6, WHEEL_DIAMETER, ENCODER_RESOLUTION);   // Change pins
Encoder rightEncoder(PB_7, PB_8, WHEEL_DIAMETER, ENCODER_RESOLUTION);

// Potentiometer Pins
Potentiometer potentiometerLeft(A0, 3.3);  // Left potentiometer pin
Potentiometer potentiometerRight(A1, 3.3); // Right potentiometer pin

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
    square_idle_mode,   // New state for idle in the square pattern
    square_pattern_mode
} MotorState;

MotorState motorState = idle_mode;

int sideCount = 0;
const float SQUARE_DISTANCE = 0.5f;  // 0.5 meters per side

// Movement State Enum for square movement
typedef enum {
    MOVE_FORWARD,   // State for moving forward
    TURN_RIGHT,     // State for turning right
    STOP            // State for stopping
} MovementState;



// Function Prototypes
void stopMotorAndSwitchToIdleMode();
void switchToSpeedControlMode();
void switchToSquareIdleMode();
void switchToSquarePatternMode();
void updateSquareMovement();
void updateLCD();

// Update LCD periodically
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

// Square Movement Function
void updateSquareMovement() {
    static float targetDistance = SQUARE_DISTANCE;
    static MovementState state = MOVE_FORWARD;  // Start with moving forward

    // This switch controls the robot's behavior based on current movement state
    switch (state) {
        case MOVE_FORWARD:
            leftMotor.setSpeed(0.5);  // Move forward
            rightMotor.setSpeed(0.5);
            if (leftEncoder.getDistance() >= targetDistance) {
                leftMotor.stop();
                rightMotor.stop();
                leftEncoder.reset();
                rightEncoder.reset();
                targetDistance = TURN_DISTANCE;  // Prepare for turn
                state = TURN_RIGHT;  // Change state to turning right
            }
            break;

        case TURN_RIGHT:
            leftMotor.setSpeed(0.5);  // Rotate left wheel only
            rightMotor.setSpeed(-0.5); // Reverse right wheel
            if (leftEncoder.getDistance() >= targetDistance) {
                leftMotor.stop();
                rightMotor.stop();
                leftEncoder.reset();
                rightEncoder.reset();
                sideCount++;  // Increase side count after a turn

                if (sideCount < 4) {
                    targetDistance = SQUARE_DISTANCE;  // Reset for next side
                    state = MOVE_FORWARD;  // Continue with forward motion
                } else {
                    state = STOP;  // Stop after completing 4 sides
                }
            }
            break;

        case STOP:
            leftMotor.stop();
            rightMotor.stop();
            break;
    }
}

// Mode Switching Functions
void switchToSpeedControlMode() {
    motorState = speed_control_mode;
    lcd.cls();
    lcd.printf("Speed Control Mode");

    // Detach interrupts for other modes
    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    // Attach interrupts for speed control mode
    fire.fall(callback(&stopMotorAndSwitchToIdleMode));
}

void switchToSquareIdleMode() {
    motorState = square_idle_mode;
    lcd.cls();
    lcd.printf("Square Idle Mode");

    // Detach interrupts for other modes
    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    // Attach interrupts for square_idle_mode
    up.fall(callback(&stopMotorAndSwitchToIdleMode));
    down.fall(callback(&stopMotorAndSwitchToIdleMode));
    fire.fall(callback(&switchToSquarePatternMode));
}

void switchToSquarePatternMode() {
    motorState = square_pattern_mode;
    lcd.cls();
    lcd.printf("Square Pattern Mode");

    // Detach interrupts for other modes
    up.fall(NULL);
    down.fall(NULL);
    fire.fall(NULL);

    // No button interrupts in square_pattern_mode
}

void stopMotorAndSwitchToIdleMode() {
    leftMotor.stop();
    rightMotor.stop();
    motorState = idle_mode;
    lcd.cls();
    lcd.printf("Idle Mode");

    // Detach interrupts for idle mode
    up.fall(callback(&switchToSpeedControlMode));
    down.fall(callback(&switchToSquareIdleMode));
    fire.fall(NULL);
}

// Main Loop
int main() {
    lcdUpdateTicker.attach(&updateLCD, 1);  // Update LCD every second

    // Attach interrupts for general button presses
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
                    lcd.printf("Speed Control Mode");
                    lcdUpdateRequired = false;
                }
                // Map potentiometer values to motor speed
                float speedRaw = potentiometerLeft.getCurrentSampleNorm();
                leftMotor.setSpeed(speedRaw);  // Set motor speed based on potentiometer
                rightMotor.setSpeed(speedRaw);
                break;

            case square_idle_mode:
                setLEDblue(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.printf("Square Idle Mode");
                    lcdUpdateRequired = false;
                }
                break;

            case square_pattern_mode:
                setLEDblue(redLED, greenLED, blueLED);
                if (lcdUpdateRequired) {
                    lcd.cls();
                    lcd.printf("Square Pattern Mode");
                    lcdUpdateRequired = false;
                }
                updateSquareMovement();  // Perform square pattern movement
                break;

            default:
                break;
        }
    }
}
