#ifndef ENCODER_H
#define ENCODER_H

/**
 * @file Encoder.h
 * @brief Defines the Encoder class for measuring wheel rotation, speed, and distance.
 */

#include "mbed.h"

/**
 * @class Encoder
 * @brief Measures wheel rotations using an incremental encoder and calculates speed and distance.
 */
class Encoder {
public:
    /**
     * @brief Constructs an Encoder object.
     * @param pinA Encoder channel A pin.
     * @param pinB Encoder channel B pin.
     * @param wheelDiameter Diameter of the wheel in meters.
     * @param resolution Pulses per revolution of the encoder (default is 1).
     */
    Encoder(PinName pinA, PinName pinB, float wheelDiameter, int resolution = 1);
    
    /**
     * @brief Retrieves the number of wheel revolutions.
     * @return The total number of revolutions since the last reset.
     */
    int getRevolutions() const;

    /**
     * @brief Computes the total distance traveled based on encoder pulses.
     * @return Distance traveled in meters.
     */
    float getDistance() const;

    /**
     * @brief Retrieves the current speed of the wheel.
     * @return The speed in meters per second.
     */
    float getSpeed() const;

    /**
     * @brief Resets the encoder count and clears stored values.
     */
    void reset();

private:
    /**
     * @brief Interrupt service routine for handling encoder pulses.
     */
    void encoderISR();

    /**
     * @brief Updates the calculated speed based on encoder pulses.
     */
    void updateSpeed();

    InterruptIn _channelA;  ///< Interrupt input for encoder channel A.
    InterruptIn _channelB;  ///< Interrupt input for encoder channel B.
    volatile int _pulses;  ///< Stores the number of pulses counted.
    const int _pulsesPerRev;  ///< Number of pulses per wheel revolution.
    const float _wheelCircumference;  ///< Computed wheel circumference based on diameter.

    Ticker _speedTicker;  ///< Ticker for updating speed calculation.
    volatile int _lastPulses;  ///< Stores the last recorded pulse count for speed calculation.
    volatile float _speed;  ///< Stores the computed speed of the wheel.
    int _resolution;  ///< Encoder resolution in pulses per revolution.
};

#endif // ENCODER_H
