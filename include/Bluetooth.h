#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "mbed.h"
#include "BuggyModes.h"
#include "BuggyModeEnum.h"
#include "BuggyModeParameters.h"

/**
 * @file Bluetooth.h
 * @class Bluetooth
 * @brief Handles Bluetooth communication for controlling the buggy.
 *
 * This class processes incoming Bluetooth commands, sends debug data, 
 * and manages parameter updates for different buggy movement modes.
 */
class Bluetooth {
public:
   /**
     * @enum DebugType
     * @brief Defines different types of debugging information.
     */
    enum DebugType {
        MOTOR_DEBUG,   /**< Debugging information related to motor control. */
        SENSOR_DEBUG,  /**< Debugging information related to sensor readings. */
        CONTROL_DEBUG, /**< Debugging information related to control system. */
        SQUARE_DEBUG   /**< Debugging information related to square movement pattern. */
    };

    /**
     * @struct MotorDebugData
     * @brief Stores motor-related debug information.
     */
    struct MotorDebugData {
        int side;        /**< 0 for left, 1 for right motor. */
        float distance;  /**< Distance traveled by the motor. */
        float speed;     /**< Current speed of the motor. */
        float set_speed; /**< Desired speed of the motor. */
        float error;     /**< Error in speed tracking. */
        float adjustment; /**< Adjustments applied for error correction. */
        float persistent_error;
        float set_rpm;
    };

    /**
     * @struct SensorDebugData
     * @brief Stores sensor-related debug information.
     */
    struct SensorDebugData {
        float sensor_values[6]; /**< Array storing sensor readings. */
        float error;            /**< Computed sensor error. */
    };

    /**
     * @struct ControlDebugData
     * @brief Stores control-related debug information.
     */
    struct ControlDebugData {
        float pid_output; /**< PID controller output. */
        float multiplier; /**< Multiplier for control adjustments. */
    };

    /**
     * @struct SquareDebugData
     * @brief Stores debug information related to square movement pattern.
     */
    struct SquareDebugData {
        float left_distance;  /**< Distance covered by the left wheel. */
        float right_distance; /**< Distance covered by the right wheel. */
        float error;          /**< Error in maintaining the path. */
        float pid_output;     /**< PID controller output for correction. */
        float multiplier;     /**< Multiplier applied for adjustments. */
    };

    /**
     * @union DebugDataUnion
     * @brief A union to store different types of debug data.
     */
    union DebugDataUnion {
        MotorDebugData motor;   /**< Motor debug data. */
        SensorDebugData sensor; /**< Sensor debug data. */
        ControlDebugData control; /**< Control system debug data. */
        SquareDebugData square; /**< Square movement debug data. */
    };

    /**
     * @struct DebugEntry
     * @brief Stores a debug entry containing type, timestamp, and data.
     */
    struct DebugEntry {
        DebugType type;      /**< Type of debug data. */
        uint32_t timestamp;  /**< Timestamp of the debug entry. */
        DebugDataUnion data; /**< Union containing debug data. */
    };

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
              BangBangBoostParams &bbbParams,
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
     * @brief Sends debug data from the debug buffer over Bluetooth.
     * 
     * This function iterates through the `debug_data_buffer` and sends formatted 
     * debug information via `_serial`. Each entry in the buffer contains a timestamp 
     * and a specific type of debug data (Motor, Sensor, Control, or Square).
     * 
     * The function then resets the debug buffer after transmission.
     * 
     * @note The output format varies depending on the debug type.
     * 
     * **Output Format:**
     * ```
     * DEBUG DATA:
     * T:<timestamp> <DATA_TYPE>:<comma-separated values>
     * DEBUG_END
     * ```
     * 
     * **Debug Data Types & Their Fields:**
     * - **MOTOR_DEBUG**  
     *   ```
     *   MOTOR:<side>,<distance>,<speed>,<set_speed>,<error>,<adjustment>
     *   ```
     *   - `side` (int): Motor side (left or right)
     *   - `distance` (float): Distance traveled
     *   - `speed` (float): Current speed
     *   - `set_speed` (float): Target speed
     *   - `error` (float): Speed error
     *   - `adjustment` (float): Adjustment applied for correction
     * 
     * - **SENSOR_DEBUG**  
     *   ```
     *   SENSOR:<error>,<sensor_1>,<sensor_2>,<sensor_3>,<sensor_4>,<sensor_5>,<sensor_6>
     *   ```
     *   - `error` (float): Sensor error
     *   - `sensor_values[6]` (float[6]): Values from individual sensors
     * 
     * - **CONTROL_DEBUG**  
     *   ```
     *   CONTROL:<pid_output>,<multiplier>
     *   ```
     *   - `pid_output` (float): PID controller output
     *   - `multiplier` (float): Control multiplier
     * 
     * - **SQUARE_DEBUG**  
     *   ```
     *   SQUARE:<left_distance>,<right_distance>,<error>,<pid_output>,<multiplier>
     *   ```
     *   - `left_distance` (float): Left wheel distance
     *   - `right_distance` (float): Right wheel distance
     *   - `error` (float): Square alignment error
     *   - `pid_output` (float): PID correction value
     *   - `multiplier` (float): Correction multiplier
     * 
     */
    void sendDebugData();

    /**
     * @brief Check to see if the buggy should start movement.
     * @return True if the start command has been received, false otherwise.
     */
    bool shouldStart();

    /**
     * @brief Check if the buggy should callibrate white.
     * @return True if Callibrate Command recieved, false otherwise.
     */
    bool shouldCallibrateWhite();

    /**
     * @brief Check if the buggy should callibrate black.
     * @return True if Callibrate Command recieved, false otherwise.
     */
    bool shouldCallibrateBlack();

        /**
     * @brief Check if the buggy should read battery info.
     * @return True if battery Command recieved, false otherwise.
     */
    bool shouldReadBattery();

    /**
     * @brief Sends a notification that the movement has finished.
     */
    void sendMovementFinished();

        /**
     * @brief Sends a notification that the callibration has finished.
     */
    void sendCallibrationFinished();

        /**
     * @brief Sends a notification that the testing has finished.
     */
    void sendTestingFinished();

    /**
     * @brief Logs debug data for tracking buggy movement and control performance.
     * @param type The type of debug data being logged.
     * @param data Pointer to a DebugDataUnion structure containing debug information.
     */
    void logDebugData(DebugType type, const void* data);

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


    void printLiveDebugData(DebugType type, void* data);

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

    /**
     * @brief Prints live motor debug data immediately to Bluetooth in a formatted manner.
     * 
     * This function sends real-time motor speed values over Bluetooth, allowing for 
     * debugging of motor behavior and performance in real-time.
     * It helps in analyzing how motor speeds change during operation.
     * 
     * @param leftSpeed The speed of the left motor.
     * @param rightSpeed The speed of the right motor.
     */
    void printMotorDebugData(float leftSpeed, float rightSpeed);

    /**
     * @brief Returns desired left motor speed
     * 
     */
    float SpeedRequestLeft();

    /**
     * @brief Returns desired right motor speed
     * 
    */
    float SpeedRequestRight();

            /**
     * @brief Check if the buggy should be sending motor Debugging.
     * @return True if should Debug False if Shoudlnt.
     */
    bool shouldDebugMotor();


    /**
     * @brief Sends battery information for monitoring.
     * 
     * @param voltage The battery voltage in volts.
     * @param current The battery current in amperes.
     * @param batteryPercentage The remaining battery percentage (0-100%).
     */
    void sendBatteryInfo(float voltage, float current, float batteryPercentage);


private:
    Serial &_serial;  ///< Reference to the Serial interface for Bluetooth communication.
    BuggyMode &_currentMode;  ///< Pointer to the current buggy mode.
    SquarePatternParams &_sqParams; ///< Pointer to square pattern movement parameters.
    StraightLineParams &_slParams;  ///< Pointer to straight-line movement parameters.
    TurnAngleParams &_taParams;     ///< Pointer to turn angle movement parameters.
    FollowParams &_flParams;    ///< Pointer to follow line  movement parameters.
    BangBangParams &_bbParams;  ///< Pointer to bang bang movement parameters.
    BangBangBoostParams &_bbbParams; ///< Pointer to bang bang boost movement parameters.
    BangBangProportionalParams &_bbpParams; ///< Pointer to bang bang proportional movement parameters.

    bool go_flag = false; ///< Flag indicating whether the buggy should start.
    bool callibrateWhite_flag; ///<Flag indicating wether the buggy should callibrate sensors over white.
    bool callibrateBlack_flag; ///<Flag indicating wether the buggy should callibrate sensors over black.
    bool debugMotor_flag; ///<Flag indicating wether the buggy be sending motor debugging.
    bool readBattery_flag; ///<Flag indicating wether the buggy should read battery info.

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
     * @brief Updates desired speeds based on received command strings.
     * @param speedStr The speed string received from Bluetooth.
     */
    void updateSpeed(const char *speedStr);

    /**
     * @brief Starts the control mode for the buggy.
     */
    void startControlMode();

    /**
     * @brief Sends the current buggy mode over Bluetooth.
     */
    void sendCurrentMode();


    #define DEBUG_RAM_ALLOCATION  (98304 - 16312) / 2  ///< Half of available RAM allocated for debugging.
    #define MAX_ENTRIES  (DEBUG_RAM_ALLOCATION / sizeof(DebugEntry)) ///< Maximum number of debug entries.

    DebugEntry debug_data_buffer[MAX_ENTRIES]; ///< Buffer for storing debug data entries.
    int debug_index = 0; ///< Current index in the debug data buffer.
    Timer debugTimer; ///< Timer for debug log time stamps.

    float desiredSpeedL = 0.0;
    float desiredSpeedR = 0.0;
};

#endif // BLUETOOTH_H
