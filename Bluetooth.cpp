#include "Bluetooth.h"
#include "BuggyModes.h"
#include <cstring>
#include <cstdio>

//Add a message when movement starts so website knows to wait and then a movement complete message that could open debug mode
//Debug mode puts data in table and also displays parameters used somewhere
//maybe make website save parameters in cahce or something
//debug should be time followed by other variables
Bluetooth::Bluetooth(Serial &serial, BuggyMode *mode,
                     SquarePatternParams *sqParams,
                     StraightLineParams *slParams,
                     TurnAngleParams *taParams)
    : _serial(serial), _currentMode(mode),
      _sqParams(sqParams), _slParams(slParams), _taParams(taParams) {
    _serial.attach(callback(this, &Bluetooth::rx_interrupt), Serial::RxIrq);
}

//Process Recived Commands
void Bluetooth::processCommand() {
    // Ignore commands if the buggy is in a movement state
    if (*_currentMode == waiting_for_movement) {
        return; // Exit without processing commands
    }

    // Process commands normally if not in a movement state
    while (tail != head) {
        char c = rx_buffer[tail];
        tail = (tail + 1) % BUFFER_SIZE;
        parseCommand(c);
    }
}


// Send available parameters based on the current mode
void Bluetooth::sendAvailableParameters() {
    _serial.printf("PARAMETERS:\n");

    switch (*_currentMode) {
        case square_idle_mode:
            _serial.printf("forward_kp=%.2f, forward_ki=%.2f, forward_kd=%.2f\n",
                           _sqParams->forward_pid_kp, _sqParams->forward_pid_ki, _sqParams->forward_pid_kd);
            _serial.printf("turn_kp=%.2f, turn_ki=%.2f, turn_kd=%.2f\n",
                           _sqParams->turn_pid_kp, _sqParams->turn_pid_ki, _sqParams->turn_pid_kd);
            _serial.printf("scaling_forward=%.2f, scaling_turn=%.2f\n",
                           _sqParams->scaling_forward, _sqParams->scaling_turn);
            _serial.printf("left_turn_multiplier=%.2f, right_turn_multiplier=%.2f\n",
                           _sqParams->left_turn_multiplier, _sqParams->right_turn_multiplier);
            _serial.printf("distance=%.2f\n", _sqParams->distance);
            break;

        case line_menu_mode:
            _serial.printf("forward_kp=%.2f, forward_ki=%.2f, forward_kd=%.2f\n",
                           _slParams->forward_pid_kp, _slParams->forward_pid_ki, _slParams->forward_pid_kd);
            _serial.printf("scaling_forward=%.2f\n", _slParams->scaling_forward);
            _serial.printf("speed=%.2f\n", _slParams->speed);
            _serial.printf("distance=%.2f\n", _slParams->distance);
            break;

        case turn_menu_mode:
            _serial.printf("turn_kp=%.2f, turn_ki=%.2f, turn_kd=%.2f\n",
                           _taParams->turn_pid_kp, _taParams->turn_pid_ki, _taParams->turn_pid_kd);
            _serial.printf("scaling_turn=%.2f\n", _taParams->scaling_turn);
            _serial.printf("speed=%.2f\n", _taParams->speed);
            _serial.printf("angle=%.2f\n", _taParams->angle);
            break;

        default:
            _serial.printf("No parameters available for this mode.\n");
            break;
    }

    _serial.printf("PARAMETERS_DONE\n");
}

// Send debug data after control mode ends
void Bluetooth::sendDebugData() {
    _serial.printf("DEBUG DATA:\n%s\n", debug_buffer);
    debug_index = 0; // Reset buffer
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
        if (*_currentMode != idle_mode) {
            _serial.printf("STARTING MOVEMENT\n");
            go_flag = true;
        }
    } else if (strcmp(cmd, "SET_MODE:SQUARE_IDLE") == 0) {
        *_currentMode = square_idle_mode;
    } else if (strcmp(cmd, "SET_MODE:STRAIGHT_LINE") == 0) {
        *_currentMode = line_menu_mode;
    } else if (strcmp(cmd, "SET_MODE:IDLE") == 0) {
        *_currentMode = idle_mode;
    } else if (strcmp(cmd, "SET_MODE:TURN_ANGLE") == 0) {
        *_currentMode = turn_menu_mode;
    } else if (strcmp(cmd, "PARAMETER") == 0) {
        sendAvailableParameters();  // Return current mode parameters
    } else if (strcmp(cmd, "STATE") == 0){
        sendCurrentMode();
    } else if (strncmp(cmd, "PARAM:", 6) == 0) {
        updateParameter(cmd + 6);   // Update a parameter dynamically
    }
}


// Update parameters dynamically based on mode
void Bluetooth::updateParameter(const char *paramStr) {
    char key[32];
    float value;
    if (sscanf(paramStr, "%[^=]=%f", key, &value) == 2) {
        switch (*_currentMode) {
            case square_idle_mode:
                if (strcmp(key, "forward_kp") == 0) _sqParams->forward_pid_kp = value;
                else if (strcmp(key, "forward_ki") == 0) _sqParams->forward_pid_ki = value;
                else if (strcmp(key, "forward_kd") == 0) _sqParams->forward_pid_kd = value;
                else if (strcmp(key, "turn_kp") == 0) _sqParams->turn_pid_kp = value;
                else if (strcmp(key, "turn_ki") == 0) _sqParams->turn_pid_ki = value;
                else if (strcmp(key, "turn_kd") == 0) _sqParams->turn_pid_kd = value;
                else if (strcmp(key, "scaling_forward") == 0) _sqParams->scaling_forward = value;
                else if (strcmp(key, "scaling_turn") == 0) _sqParams->scaling_turn = value;
                else if (strcmp(key, "left_turn_multiplier") == 0) _sqParams->left_turn_multiplier = value;
                else if (strcmp(key, "right_turn_multiplier") == 0) _sqParams->right_turn_multiplier = value;
                else if (strcmp(key, "distance") == 0) _sqParams->distance = value;
                break;

            case line_menu_mode:
                if (strcmp(key, "forward_kp") == 0) _slParams->forward_pid_kp = value;
                else if (strcmp(key, "forward_ki") == 0) _slParams->forward_pid_ki = value;
                else if (strcmp(key, "forward_kd") == 0) _slParams->forward_pid_kd = value;
                else if (strcmp(key, "scaling_forward") == 0) _slParams->scaling_forward = value;
                else if (strcmp(key, "speed") == 0) _slParams->speed = value;
                else if (strcmp(key, "distance") == 0) _slParams->distance = value;
                break;

            case turn_menu_mode:
                if (strcmp(key, "turn_kp") == 0) _taParams->turn_pid_kp = value;
                else if (strcmp(key, "turn_ki") == 0) _taParams->turn_pid_ki = value;
                else if (strcmp(key, "turn_kd") == 0) _taParams->turn_pid_kd = value;
                else if (strcmp(key, "scaling_turn") == 0) _taParams->scaling_turn = value;
                else if (strcmp(key, "speed") == 0) _taParams->speed = value;
                else if (strcmp(key, "angle") == 0) _taParams->angle = value;
                break;
        }

        _serial.printf("Updated: %s = %.2f\n", key, value);
    }
}

bool Bluetooth::shouldStart() {
    if (go_flag) {
        go_flag = false; // Reset flag
        return true;
    }
    return false;
}

void Bluetooth::sendMovementFinished(){
    _serial.printf("MOVEMENT FINISHED\n");
}

void Bluetooth::sendCurrentMode() {
    // Call the helper function to get the mode name
    const char* modeName = getModeName(*_currentMode);
    
    // Send the mode name over Bluetooth
    _serial.printf("MODE:%s\n", modeName);
}
