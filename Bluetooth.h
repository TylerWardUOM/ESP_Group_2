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
    void logDebugData(float leftDistance, float rightDistance, float error, float pidOutput, float multiplier);
    void resetDebugData();
    void printDebugData(const char* format, ...);

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

    void rx_interrupt();
    void parseCommand(char c);
    void handleCommand(const char *cmd);
    void updateParameter(const char *paramStr);
    void startControlMode();
    void sendCurrentMode();

    struct DebugEntry {
        uint32_t timestamp;
        float left_distance;
        float right_distance;
        float error;
        float pid_output;
        float multiplier;
    };

    #define DEBUG_RAM_ALLOCATION  (98304 - 16312) / 2  // Half of available RAM
    #define MAX_ENTRIES  (DEBUG_RAM_ALLOCATION / sizeof(DebugEntry))

    DebugEntry debug_data_buffer[MAX_ENTRIES];
    int debug_index = 0;
};

#endif // BLUETOOTH_H
