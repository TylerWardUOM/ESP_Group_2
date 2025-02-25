#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "mbed.h"
#include "BuggyModes.h"

class Bluetooth {
public:
    // Constructor declaration
    Bluetooth(Serial &serial, BuggyMode *mode, 
              SquarePatternParams *sqParams, 
              StraightLineParams *slParams, 
              TurnAngleParams *taParams);
    void processCommand();
    void sendAvailableParameters();
    void sendDebugData();
    bool shouldStart();
    void sendMovementFinished();

private:
    Serial &_serial;
    BuggyMode *_currentMode;  // Pointer to external buggy state
    SquarePatternParams *_sqParams; // Pointer to square pattern parameters
    StraightLineParams *_slParams;  // Pointer to straight line parameters
    TurnAngleParams *_taParams;     // Pointer to turn angle parameters

    bool go_flag = false; 

    static const int BUFFER_SIZE = 128;
    char rx_buffer[BUFFER_SIZE];
    volatile uint16_t head = 0, tail = 0;

    static const int DEBUG_BUFFER_SIZE = 512;
    char debug_buffer[DEBUG_BUFFER_SIZE];
    uint16_t debug_index = 0;

    void rx_interrupt();
    void parseCommand(char c);
    void handleCommand(const char *cmd);
    void updateParameter(const char *paramStr);
    void startControlMode();
    void sendCurrentMode();
};

#endif // BLUETOOTH_H
