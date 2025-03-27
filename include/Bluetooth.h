#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "mbed.h"
#include "BuggyModes.h"
#include "BuggyModeEnum.h"
#include "BuggyModeParameters.h"

/**
 * @class Bluetooth
 * @brief Handles Bluetooth communication for controlling the buggy.
 *
 * This class processes incoming Bluetooth commands, sends debug data, 
 * and manages parameter updates for different buggy movement modes.
 */
class Bluetooth {
public:
    /**
     * @brief Constructs a Bluetooth object.
     * @param serial Reference to a Serial object for Bluetooth communication.
     * @param mode Pointer to the current Buggy mode.
     * @param sqParams Pointer to Square Pattern movement parameters.
     * @param slParams Pointer to Straight Line movement parameters.
     * @param taParams Pointer to Turn Angle movement parameters.
     * @param flParams Pointer to Follow Line movement parameters.
     * @param bbParams Pointer to Bang Bang movement parameters.
     * @param bbpParams Pointer to Bang Bang Proportional movement parameters.
     */
    Bluetooth(Serial &serial, BuggyMode &mode, 
              SquarePatternParams &sqParams, 
              StraightLineParams &slParams, 
              TurnAngleParams &taParams,
              FollowParams &flParams,
              BangBangParams &bbParams,
              BangBangProportionalParams &bbpParams);

    /**
     * @brief Processes received commands from the buffer.
     *
     * Reads each character from the circular buffer and forwards it to \ref parseCommand.
     * Commands are only processed if the buggy is not in a movement state.
     */
    void processCommand();

    /**
     * @brief Sends the current set of available parameters over Bluetooth.
     */
    void sendAvailableParameters();

    /**
     * @brief Sends debug data from debug buffer over Bluetooth.
     */
    void sendDebugData();

    /**
     * @brief Check to see if the buggy should start movement.
     * @return True if the start command has been received, false otherwise.
     */
    bool shouldStart();

    /**
     * @brief Sends a notification that the movement has finished.
     */
    void sendMovementFinished();

    /**
     * @brief Logs debug data for tracking buggy movement and control performance.
     * @param leftDistance Distance measured by the left wheel encoder.
     * @param rightDistance Distance measured by the right wheel encoder.
     * @param error Current positional error.
     * @param pidOutput PID controller output value.
     * @param multiplier Control multiplier applied.
     */
    void logDebugData(float leftDistance, float rightDistance, float error, float pidOutput, float multiplier,float* sensorValues = NULL);

    /**
     * @brief Resets the debug data buffer.
     */
    void resetDebugData();

    /**
     * @brief Prints debug data immediatly to ble in a formatted manner.
     * @param format C-style format string.
     * @param ... Variable arguments for formatted output.
     */
    void printDebugData(const char* format, ...);

    /**
     * @brief Prints live sensor data immediately to Bluetooth in a formatted manner.
     * 
     * This function sends real-time sensor values and the computed error over Bluetooth.
     * It helps in debugging by providing immediate feedback on sensor readings.
     * 
     * @param sensorValues Array of sensor readings.
     * @param numSensors Number of sensors in the array.
     * @param error The computed error value representing the line position.
     */
    void printLiveSensorData(float sensorValues[], int numSensors, float error);

private:
    Serial &_serial;  ///< Reference to the Serial interface for Bluetooth communication.
    BuggyMode &_currentMode;  ///< Pointer to the current buggy mode.
    SquarePatternParams &_sqParams; ///< Pointer to square pattern movement parameters.
    StraightLineParams &_slParams;  ///< Pointer to straight-line movement parameters.
    TurnAngleParams &_taParams;     ///< Pointer to turn angle movement parameters.
    FollowParams &_flParams;    ///< Pointer to follow line  movement parameters.
    BangBangParams &_bbParams;  ///< Pointer to bang bang movement parameters.
    BangBangProportionalParams &_bbpParams; ///< Pointer to bang bang proportional movement parameters.

    bool go_flag = false; ///< Flag indicating whether the buggy should start.

    static const int BUFFER_SIZE = 128; ///< Size of the receive buffer.
    char rx_buffer[BUFFER_SIZE]; ///< Buffer for storing received commands.
    volatile uint16_t head = 0, tail = 0; ///< Circular buffer head and tail indices.

    /**
     * @brief Interrupt service routine for receiving Bluetooth data.
     */
    void rx_interrupt();

    /**
     * @brief Parses an incoming command character by character.
     *
     * This function accumulates characters in a buffer until it detects a newline (`\n`).
     * Once a full command is formed, it is sent to \ref handleCommand for processing.
     *
     * @param c The received character to be processed.
     */
    void parseCommand(char c);

    /**
     * @brief Handles a fully received command.
     *
     * Once a complete command string is received by \ref parseCommand, this function 
     * processes it and executes the corresponding action.
     *
     * @param cmd The full command string to be handled.
     */
    void handleCommand(const char *cmd);

    /**
     * @brief Updates parameters based on received command strings.
     * @param paramStr The parameter string received from Bluetooth.
     */
    void updateParameter(const char *paramStr);

    /**
     * @brief Starts the control mode for the buggy.
     */
    void startControlMode();

    /**
     * @brief Sends the current buggy mode over Bluetooth.
     */
    void sendCurrentMode();

    /**
     * @struct DebugEntry
     * @brief Stores a single debug entry for movement logging.
     */
    struct DebugEntry {
        uint32_t timestamp; ///< Timestamp of the recorded data entry.
        float left_distance; ///< Distance recorded by the left wheel encoder.
        float right_distance; ///< Distance recorded by the right wheel encoder.
        float error; ///< Positional error value.
        float pid_output; ///< PID controller output.
        float multiplier; ///< Applied control multiplier.
        float sensor_values[6]; ///< Array to store sensor values.
    };

    #define DEBUG_RAM_ALLOCATION  (98304 - 16312) / 2  ///< Half of available RAM allocated for debugging.
    #define MAX_ENTRIES  (DEBUG_RAM_ALLOCATION / sizeof(DebugEntry)) ///< Maximum number of debug entries.

    DebugEntry debug_data_buffer[MAX_ENTRIES]; ///< Buffer for storing debug data entries.
    int debug_index = 0; ///< Current index in the debug data buffer.
};

#endif // BLUETOOTH_H
