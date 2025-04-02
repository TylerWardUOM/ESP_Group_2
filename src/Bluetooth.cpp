#include "Bluetooth.h"
#include "BuggyModes.h"
#include <cstring>
#include <cstdio>
#include "mbed.h"

// Define the timer
Timer debugTimer;

//Add a message when movement starts so website knows to wait and then a movement complete message that could open debug mode
//Debug mode puts data in table and also displays parameters used somewhere
//maybe make website save parameters in cahce or something
//debug should be time followed by other variables
Bluetooth::Bluetooth(Serial &serial, BuggyMode &mode,
                     SquarePatternParams &sqParams,
                     StraightLineParams &slParams,
                     TurnAngleParams &taParams,
                     FollowParams &flParams,
                     BangBangParams &bbParams,
                     BangBangProportionalParams &bbpParams)
    : _serial(serial), _currentMode(mode),
      _sqParams(sqParams), _slParams(slParams), _taParams(taParams),
      _flParams(flParams), _bbParams(bbParams),_bbpParams(bbpParams) {
    _serial.attach(callback(this, &Bluetooth::rx_interrupt), Serial::RxIrq);
    debugTimer.start();
}

//Process Recived Commands
void Bluetooth::processCommand() {

    while (tail != head) {
        char c = rx_buffer[tail];
        tail = (tail + 1) % BUFFER_SIZE;
        parseCommand(c);
    }
}


void Bluetooth::sendAvailableParameters() {
    _serial.printf("PARAMETERS:\n");

    switch (_currentMode) {
        case square_idle_mode:
            _serial.printf("forward_kp=%.7f, forward_ki=%.7f, forward_kd=%.7f\n",
                           _sqParams.forward_pid_kp, _sqParams.forward_pid_ki, _sqParams.forward_pid_kd);
            _serial.printf("turn_kp=%.7f, turn_ki=%.7f, turn_kd=%.7f\n",
                           _sqParams.turn_pid_kp, _sqParams.turn_pid_ki, _sqParams.turn_pid_kd);
            _serial.printf("scaling_forward=%.7f, scaling_turn=%.7f\n",
                           _sqParams.scaling_forward, _sqParams.scaling_turn);
            _serial.printf("left_turn_multiplier=%.7f, right_turn_multiplier=%.7f\n",
                           _sqParams.left_turn_multiplier, _sqParams.right_turn_multiplier);
            _serial.printf("distance=%.7f\n", _sqParams.distance);
            break;

        case line_menu_mode:
            _serial.printf("forward_kp=%.7f, forward_ki=%.7f, forward_kd=%.7f\n",
                           _slParams.forward_pid_kp, _slParams.forward_pid_ki, _slParams.forward_pid_kd);
            _serial.printf("scaling_forward=%.7f\n", _slParams.scaling_forward);
            _serial.printf("speed=%.7f\n", _slParams.speed);
            _serial.printf("distance=%.7f\n", _slParams.distance);
            break;

        case turn_menu_mode:
            _serial.printf("turn_kp=%.7f, turn_ki=%.7f, turn_kd=%.7f\n",
                           _taParams.turn_pid_kp, _taParams.turn_pid_ki, _taParams.turn_pid_kd);
            _serial.printf("scaling_turn=%.7f\n", _taParams.scaling_turn);
            _serial.printf("speed=%.7f\n", _taParams.speed);
            _serial.printf("angle=%.7f\n", _taParams.angle);
            break;

        
        case follow_menu_mode:
            _serial.printf("kp=%.7f, ki=%.7f, kd=%.7f\n",
                           _flParams.pid_kp, _flParams.pid_ki, _flParams.pid_kd);
            _serial.printf("pid_scaling=%.7f\n", _flParams.pid_scaling);
            _serial.printf("speed=%.7f\n", _flParams.speed);
            _serial.printf("sensorPeriod=%.7f\n", _flParams.sensorSamplePeriod);
            _serial.printf("controlPeriod=%.7f\n", _flParams.controlPeriod);
            _serial.printf("motorRegulatePeriod=%.7f\n", _flParams.motorRegulatePeriod);
            _serial.printf("motorKp=%.7f\n", _flParams.motor_Kp);
            break;

        case bang_bang_menu_mode:
            _serial.printf("baseSpeed=%.7f\n", _bbParams.baseSpeed);
            _serial.printf("turnSpeedMultiplier=%.7f\n", _bbParams.turnSpeedMultiplier);
            _serial.printf("bangBangThreshold=%.7f\n", _bbParams.bangBangThreshold);
            _serial.printf("sensorPeriod=%.7f\n", _bbParams.sensorSamplePeriod);
            _serial.printf("controlPeriod=%.7f\n", _bbParams.controlPeriod);
            _serial.printf("motorRegulatePeriod=%.7f\n", _bbParams.motorRegulatePeriod);
            _serial.printf("motorKp=%.7f\n", _bbParams.motor_Kp);
            break;

        case bang_bang_proportional_menu_mode:
            _serial.printf("baseSpeed=%.7f\n", _bbpParams.baseSpeed);
            _serial.printf("kP=%.7f\n", _bbpParams.kP);
            _serial.printf("maxMultiplier=%.7f\n", _bbpParams.maxMultiplier);
            _serial.printf("bangBangThreshold=%.7f\n", _bbpParams.bangBangThreshold);
            _serial.printf("sensorPeriod=%.7f\n", _bbpParams.sensorSamplePeriod);
            _serial.printf("controlPeriod=%.7f\n", _bbpParams.controlPeriod);
            _serial.printf("motorRegulatePeriod=%.7f\n", _bbpParams.motorRegulatePeriod);
            _serial.printf("motorKp=%.7f\n", _bbpParams.motor_Kp);
            break;



        default:
            _serial.printf("No parameters available for this mode.\n");
            break;
    }

    _serial.printf("PARAMETERS_DONE\n");
}

void Bluetooth::sendDebugData() {
    _serial.printf("DEBUG DATA:\n");

    for (int i = 0; i < debug_index; i++) {
        DebugEntry entry = debug_data_buffer[i];

        // Print main debug info
        _serial.printf("%lu ms: Left_Distance=%f, Right_Distance=%f, Error=%f, PID_out=%f, Multiplier=%f\n",
                       entry.timestamp, entry.left_distance, entry.right_distance, entry.error, entry.pid_output, entry.multiplier);

        // Print sensor values (formatted inline)
        _serial.printf("Sensors: [");
        for (int j = 0; j < 6; j++) {
            _serial.printf("%f", entry.sensor_values[j]);
            if (j < 5) {
                _serial.printf(", "); // Add commas between values
            }
        }
        _serial.printf("]\n"); // Close sensor array output
    }

    debug_index = 0; // Reset binary buffer after sending

    _serial.printf("DEBUG_END\n");
}


// UART Interrupt Handler (store data in circular buffer)
void Bluetooth::rx_interrupt() {
    while (_serial.readable()) {
        char c = _serial.getc();
        rx_buffer[head] = c;
        head = (head + 1) % BUFFER_SIZE;
    }
}

// Parse incoming command
void Bluetooth::parseCommand(char c) {
    static char cmd_buffer[32];
    static uint8_t cmd_index = 0;

    if (c == '\n') {
        cmd_buffer[cmd_index] = '\0'; // Null-terminate
        handleCommand(cmd_buffer);
        cmd_index = 0; // Reset
    } else if (cmd_index < sizeof(cmd_buffer) - 1) {
        cmd_buffer[cmd_index++] = c;
    }
}

// Handle full command
void Bluetooth::handleCommand(const char *cmd) {
    if (strcmp(cmd, "GO") == 0) {
        if (_currentMode != idle_mode) {
            _serial.printf("STARTING MOVEMENT\n");
            go_flag = true;
        }
    } else if (strcmp(cmd, "SET_MODE:SQUARE_IDLE") == 0) {
        _currentMode = square_idle_mode;
        _serial.printf("MODE_CHANGED:SQUARE_IDLE\n");
    } else if (strcmp(cmd, "SET_MODE:STRAIGHT_LINE") == 0) {
        _currentMode = line_menu_mode;
        _serial.printf("MODE_CHANGED:STRAIGHT_LINE\n");
    } else if (strcmp(cmd, "SET_MODE:IDLE") == 0) {
        _currentMode = reset;
        _serial.printf("MODE_CHANGED:IDLE\n");
    } else if (strcmp(cmd, "SET_MODE:TURN_ANGLE") == 0) {
        _currentMode = turn_menu_mode;
        _serial.printf("MODE_CHANGED:TURN_ANGLE\n");
    }else if (strcmp(cmd, "SET_MODE:SPEED_CONTROL") == 0) {
        _currentMode = speed_control_mode;
        _serial.printf("MODE_CHANGED:SPEED_CONTROL\n");
    }else if (strcmp(cmd, "SET_MODE:FOLLOW_LINE") == 0) {
        _currentMode = follow_menu_mode;
        _serial.printf("MODE_CHANGED:FOLLOW_LINE\n");
    } else if (strcmp(cmd, "SET_MODE:BANG_BANG") == 0) {
        _currentMode = bang_bang_menu_mode;
        _serial.printf("MODE_CHANGED:BANG_BANG\n");
    }else if (strcmp(cmd, "SET_MODE:BANG_BANG_PROPORTIONAL") == 0) {
        _currentMode = bang_bang_proportional_menu_mode;
        _serial.printf("MODE_CHANGED:BANG_BANG_PROPORTIONAL\n");
    }else if (strcmp(cmd, "SET_MODE:SENSOR_DEBUG") == 0) {
        _currentMode = sensor_debug_menu;
        _serial.printf("MODE_CHANGED:SENSOR_DEBUG\n");
    }else if (strcmp(cmd, "SET_MODE:MOTOR_DEBUG") == 0) {
        _currentMode = motor_debug_menu;
        _serial.printf("MODE_CHANGED:MOTOR_DEBUG\n");
    }else if (strcmp(cmd, "SET_MODE:RC") == 0) {
        _currentMode = rc_menu;
        _serial.printf("MODE_CHANGED:RC\n");
    }else if (strcmp(cmd, "CALLIBRATE_WHITE") == 0) {
        callibrateWhite_flag=true;
        _serial.printf("CALLIBRATING_WHITE\n");
    }else if (strcmp(cmd, "CALLIBRATE_BLACK") == 0) {
        callibrateBlack_flag=true;
        _serial.printf("CALLIBRATING_BLACK\n");
    }else if (strcmp(cmd, "BATTERY") == 0) {
        readBattery_flag=true;
        _serial.printf("SENDING_BATTERY\n");
    }else if (strcmp(cmd, "PARAMETER") == 0) {
        sendAvailableParameters();  // Return current mode parameters
    } else if (strcmp(cmd, "STATE") == 0){
        sendCurrentMode();
    } else if (strncmp(cmd, "PARAM:", 6) == 0) {
        updateParameter(cmd + 6);   // Update a parameter dynamically
    }else if (strncmp(cmd, "SET_SPEED:", 10) == 0) {
        updateSpeed(cmd + 10);   // Update a parameter dynamically
    }else if (strcmp(cmd, "STOP") == 0) {
        _currentMode = reset;
        _serial.printf("MODE_CHANGED:IDLE\n");
    }else if (strcmp(cmd, "TURN_AROUND") == 0) {
        _currentMode = turn_around;
        _serial.printf("TURNING_AROUND\n");
    }
}



// Update parameters dynamically based on mode
void Bluetooth::updateParameter(const char *paramStr) {
    char key[32];
    float value;
    if (sscanf(paramStr, "%[^=]=%f", key, &value) == 2) {
        switch (_currentMode) {
            case square_idle_mode:
                if (strcmp(key, "forward_kp") == 0) _sqParams.forward_pid_kp = value;
                else if (strcmp(key, "forward_ki") == 0) _sqParams.forward_pid_ki = value;
                else if (strcmp(key, "forward_kd") == 0) _sqParams.forward_pid_kd = value;
                else if (strcmp(key, "turn_kp") == 0) _sqParams.turn_pid_kp = value;
                else if (strcmp(key, "turn_ki") == 0) _sqParams.turn_pid_ki = value;
                else if (strcmp(key, "turn_kd") == 0) _sqParams.turn_pid_kd = value;
                else if (strcmp(key, "scaling_forward") == 0) _sqParams.scaling_forward = value;
                else if (strcmp(key, "scaling_turn") == 0) _sqParams.scaling_turn = value;
                else if (strcmp(key, "left_turn_multiplier") == 0) _sqParams.left_turn_multiplier = value;
                else if (strcmp(key, "right_turn_multiplier") == 0) _sqParams.right_turn_multiplier = value;
                else if (strcmp(key, "distance") == 0) _sqParams.distance = value;
                break;

            case line_menu_mode:
                if (strcmp(key, "forward_kp") == 0) _slParams.forward_pid_kp = value;
                else if (strcmp(key, "forward_ki") == 0) _slParams.forward_pid_ki = value;
                else if (strcmp(key, "forward_kd") == 0) _slParams.forward_pid_kd = value;
                else if (strcmp(key, "scaling_forward") == 0) _slParams.scaling_forward = value;
                else if (strcmp(key, "speed") == 0) _slParams.speed = value;
                else if (strcmp(key, "distance") == 0) _slParams.distance = value;
                break;

            case turn_menu_mode:
                if (strcmp(key, "turn_kp") == 0) _taParams.turn_pid_kp = value;
                else if (strcmp(key, "turn_ki") == 0) _taParams.turn_pid_ki = value;
                else if (strcmp(key, "turn_kd") == 0) _taParams.turn_pid_kd = value;
                else if (strcmp(key, "scaling_turn") == 0) _taParams.scaling_turn = value;
                else if (strcmp(key, "speed") == 0) _taParams.speed = value;
                else if (strcmp(key, "angle") == 0) _taParams.angle = value;
                break;

            case follow_menu_mode:
                if (strcmp(key, "kp") == 0) _flParams.pid_kp = value;
                else if (strcmp(key, "ki") == 0) _flParams.pid_ki = value;
                else if (strcmp(key, "kd") == 0) _flParams.pid_kd = value;
                else if (strcmp(key, "pid_scaling") == 0) _flParams.pid_scaling = value;
                else if (strcmp(key, "speed") == 0) _flParams.speed = value;
                else if (strcmp(key, "sensorPeriod") == 0) _flParams.sensorSamplePeriod = value;
                else if (strcmp(key, "controlPeriod") == 0) _flParams.controlPeriod = value;
                else if (strcmp(key, "motorRegulatePeriod") == 0) _flParams.motorRegulatePeriod = value;
                else if (strcmp(key, "motorKp") == 0) _flParams.motor_Kp = value;
                break;

            case bang_bang_menu_mode:
                if (strcmp(key, "baseSpeed") == 0) _bbParams.baseSpeed = value;
                else if (strcmp(key, "turnSpeedMultiplier") == 0) _bbParams.turnSpeedMultiplier = value;
                else if (strcmp(key, "bangBangThreshold") == 0) _bbParams.bangBangThreshold = value;
                else if (strcmp(key, "sensorPeriod") == 0) _bbParams.sensorSamplePeriod = value;
                else if (strcmp(key, "controlPeriod") == 0) _bbParams.controlPeriod = value;
                else if (strcmp(key, "motorRegulatePeriod") == 0) _bbParams.motorRegulatePeriod = value;
                else if (strcmp(key, "motorKp") == 0) _bbParams.motor_Kp = value;
                break;

            case bang_bang_proportional_menu_mode:
                if (strcmp(key, "baseSpeed") == 0) _bbpParams.baseSpeed = value;
                else if (strcmp(key, "kP") == 0) _bbpParams.kP = value;
                else if (strcmp(key, "maxMultiplier") == 0) _bbpParams.maxMultiplier = value;
                else if (strcmp(key, "bangBangThreshold") == 0) _bbpParams.bangBangThreshold = value;
                else if (strcmp(key, "sensorPeriod") == 0) _bbpParams.sensorSamplePeriod = value;
                else if (strcmp(key, "controlPeriod") == 0) _bbpParams.controlPeriod = value;
                else if (strcmp(key, "motorRegulatePeriod") == 0) _bbpParams.motorRegulatePeriod = value;
                else if (strcmp(key, "motorKp") == 0) _bbpParams.motor_Kp = value;
                break;

        }
        _serial.printf("Updated: %s = %.7f\n", key, value);
    }
}

bool Bluetooth::shouldStart() {
    if (go_flag) {
        go_flag = false; // Reset flag
        return true;
    }
    return false;
}

bool Bluetooth::shouldCallibrateWhite() {
    if (callibrateWhite_flag) {
        callibrateWhite_flag = false; // Reset flag
        return true;
    }
    return false;
}

bool Bluetooth::shouldCallibrateBlack() {
    if (callibrateBlack_flag) {
        callibrateBlack_flag = false; // Reset flag
        return true;
    }
    return false;
}

void Bluetooth::sendMovementFinished(){
    _serial.printf("MOVEMENT FINISHED\n");
}

void Bluetooth::sendCallibrationFinished(){
    _serial.printf("CALLIBRATION FINISHED\n");
}

void Bluetooth::sendTestingFinished(){
    _serial.printf("TESTING FINISHED\n");
}

void Bluetooth::sendCurrentMode() {
    // Call the helper function to get the mode name
    const char* modeName = getModeName(_currentMode);
    
    // Send the mode name over Bluetooth
    _serial.printf("MODE:%s\n", modeName);
}



void Bluetooth::logDebugData(float leftDistance, float rightDistance, float error, float pidOutput, float multiplier, float* sensorValues) {
    if (debug_index >= MAX_ENTRIES) {
        return;  // Avoid buffer overflow
    }

    // Get reference to the next available entry (avoids redundant indexing)
    DebugEntry& entry = debug_data_buffer[debug_index++];

    // Assign basic data
    entry.timestamp = debugTimer.read_ms();
    entry.left_distance = leftDistance;
    entry.right_distance = rightDistance;
    entry.error = error;
    entry.pid_output = pidOutput;
    entry.multiplier = multiplier;

    // Only copy sensor values if provided
    if (sensorValues) {
        memcpy(entry.sensor_values, sensorValues, 6 * sizeof(float));
    } else {
        memset(entry.sensor_values, 0, 6 * sizeof(float)); // Default to 0 if no values provided
    }
}




void Bluetooth::resetDebugData() {
    // Reset the debug timer
    debugTimer.reset();

    // Clear the binary debug buffer by resetting the index
    debug_index = 0;

    printDebugData("Resetting Debug Data");
}


void Bluetooth::printDebugData(const char* format, ...) {
    // Get elapsed time in milliseconds
    uint32_t timeElapsed = debugTimer.read_ms();

    // Print timestamp
    _serial.printf("%lu ms: ", timeElapsed);

    // Handle formatted arguments
    va_list args;
    va_start(args, format);
    _serial.vprintf(format, args);
    va_end(args);

    // Print newline for formatting
    _serial.printf("\n");
};

void Bluetooth::printLiveSensorData(float sensorValues[], int numSensors, float error) {
    // Get elapsed time in milliseconds
    uint32_t timeElapsed = debugTimer.read_ms();

    // Print timestamp
    _serial.printf("SENSOR DATA: ");
    _serial.printf("%lu ", timeElapsed);
    // Print sensor values
    for (int i = 0; i < numSensors; i++) {
        _serial.printf("%.4f ", sensorValues[i]);
    }

    // Print error value
    _serial.printf("| ERROR: %.4f\n", error);
};

void Bluetooth::printMotorDebugData(float leftSpeed, float rightSpeed) {
    // Get elapsed time in milliseconds
    uint32_t timeElapsed = debugTimer.read_ms();

    // Print motor debug data
    _serial.printf("MOTOR DATA: ");
    _serial.printf("%lu ", timeElapsed);
    _serial.printf("| Left Speed: %.4f ", leftSpeed);
    _serial.printf("| Right Speed: %.4f\n", rightSpeed);
}


float Bluetooth::SpeedRequestLeft(){
    float output = 5000.00;
    if (desiredSpeedL!=5000.00){
        output = desiredSpeedL;
        desiredSpeedL = 5000.00; //5000 acting as null value
    }
    return output;
}

float Bluetooth::SpeedRequestRight(){
    float output = 5000.00;
    if (desiredSpeedR!=5000.00){
        output = desiredSpeedR;
        desiredSpeedR = 5000.00; //5000 acting as null value
    }
    return output;
}

// Update speed based on SET_SPEED:left=x or SET_SPEED:right=x
void Bluetooth::updateSpeed(const char* speedStr) {
    char key[32];
    float value;
    // Parse the input to extract key (e.g., "left" or "right") and value (e.g., 50)
    if (sscanf(speedStr, "%[^=]=%f", key, &value) == 2) {
        // Check if the key is "left" or "right" and set the corresponding desired speed
        if (strcmp(key, "left") == 0) {
            desiredSpeedL = value;  // Set desired left speed
           _serial.printf("Desired left Speed set to: %.4f\n", desiredSpeedL);
        } else if (strcmp(key, "right") == 0) {
            desiredSpeedR = value;  // Set desired right speed
            _serial.printf("Desired right Speed set to: %.4f\n", desiredSpeedR);
        } else {
            _serial.printf("Invalid speed parameter: %s\n", key);
        }
    } else {
        printf("Invalid SET_SPEED command format\n");
    }
}

void Bluetooth::sendBatteryInfo(float voltage, float current, float batteryPercentage){
    _serial.printf("BATTERY: Voltage: %.2f, Current: %.3f, Battery: %.2f\n", voltage, current, batteryPercentage);
}

bool Bluetooth::shouldReadBattery() {
    if (readBattery_flag) {
        readBattery_flag = false; // Reset flag
        return true;
    }
    return false;
}