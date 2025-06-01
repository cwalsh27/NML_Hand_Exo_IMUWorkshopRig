/**
 * @file nml_hand_exo.h
 * @brief API header for the NML Hand Exoskeleton device.
 *
 * This file declares the NMLHandExo class and helper utilities for managing
 * and controlling the exoskeleton using Dynamixel servos.
 */
#ifndef NML_HAND_EXO_H
#define NML_HAND_EXO_H

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;


/// @brief Verbose output toggle for debugging.
extern bool VERBOSE;

/// @brief Debug serial stream used for logging.
extern Stream& DEBUG_SERIAL;

/// @brief Pointer to the debug stream used for conditional logging.
extern Stream* debugStream;

/// @brief Debugging print helper function.
/// @param msg The message to print.
void debugPrint(const String& msg);

/// @brief Class to manage the NML Hand Exoskeleton, providing initialization, motor control, and telemetry.
class NMLHandExo {
  public:
    /// @brief Constructor.
    /// @param ids Pointer to array of motor IDs.
    /// @param numMotors Number of motors in the device.
    /// @param jointLimits Pointer to 2D array defining [min, max] joint limits in degrees.
    /// @param homeState Optional array of home positions (in degrees). Defaults to zero offsets.
    NMLHandExo(const uint8_t* ids, uint8_t numMotors, const float jointLimits[][2], const float* homeState = nullptr);

    // -----------------------------------------------------------
    // Utility functions
    // -----------------------------------------------------------

    /// @brief Initialize the serial port for Dynamixel communication.
    /// @param baud Baud rate to initialize.
    void initializeSerial(int baud);

    /// @brief Initialize all motors: disables torque, sets position mode, then re-enables torque.
    void initializeMotors();

    /// @brief Get the motor ID from a user-supplied token (either name or ID as a string).
    /// @param token The token string (e.g. "WRIST" or "1").
    /// @return The motor ID or -1 if not found.
    int getMotorID(const String& token);

    /// @brief Get the index of a motor in the internal arrays from its ID.
    /// @param id The motor ID.
    /// @return The index in the motor array or -1 if not found.
    int getIndexById(uint8_t id);

    /// @brief Get the motor ID by name.
    /// @param name Name of the motor (e.g. "WRIST").
    /// @return The motor ID or -1 if not found.
    int getMotorIDByName(const String& name);

    /// @brief Get the motor name from its ID.
    /// @param id The motor ID.
    /// @return The name of the motor.
    String getNameByMotorID(uint8_t id);

    /// @brief Convert a relative angle (degrees) to Dynamixel tick counts.
    /// @param angle_deg Angle in degrees.
    /// @param index Index of the motor.
    /// @return Ticks equivalent to the angle.
    int angleToTicks(float angle_deg, int index);

    /// @brief Calibrate the zero offset for a motor by reading its current position.
    /// @param id Motor ID.
    void setZeroOffset(uint8_t id);

    /// @brief Get the stored zero offset for a motor.
    /// @param id Motor ID.
    /// @return Zero offset in degrees.
    float getZeroOffset(uint8_t id);

    /// @brief Reset zero offsets for all motors using their current positions.
    void resetAllZeros();

    /// @brief Get a string summarizing the device information.
    /// @return Information string.
    String getDeviceInfo();

    // -----------------------------------------------------------
    // Position commands
    // -----------------------------------------------------------

    /// @brief Get the relative angle (degrees) of a motor.
    /// @param id Motor ID.
    /// @return Relative angle in degrees.
    float getRelativeAngle(uint8_t id);

    /// @brief Command the motor to a relative angle.
    /// @param id Motor ID.
    /// @param angleDeg Relative angle in degrees.
    void setRelativeAngle(uint8_t id, float angleDeg);

    /// @brief Get the absolute angle (degrees) of a motor.
    /// @param id Motor ID.
    /// @return Absolute angle in degrees.
    float getAbsoluteAngle(uint8_t id);

    /// @brief Command the motor to an absolute angle.
    /// @param id Motor ID.
    /// @param angleDeg Absolute angle in degrees.
    void setAbsoluteAngle(uint8_t id, float angleDeg);

    /// @brief Get the stored zero angle of a motor.
    /// @param id Motor ID.
    /// @return Zero angle in degrees.
    float getZeroAngle(uint8_t id);

    /// @brief Home the motor to its stored zero position.
    /// @param id Motor ID.
    void setHome(uint8_t id);

    /// @brief Home all motors to their stored zero positions.
    void homeAllMotors();

    /// @brief Command the motor to an angle by ID.
    /// @param id Motor ID.
    /// @param angleDeg Angle in degrees.
    void setAngleById(uint8_t id, float angleDeg);

    /// @brief Command a motor using an alias (name) and angle.
    /// @param alias Motor name (e.g. "WRIST").
    /// @param angleDeg Angle in degrees.
    void setAngleByAlias(const String& alias, float angleDeg);

    // -----------------------------------------------------------
    // Torque commands
    // -----------------------------------------------------------

    /// @brief Enable or disable torque for a motor.
    /// @param id Motor ID.
    /// @param enable True to enable torque, false to disable.
    void enableTorque(uint8_t id, bool enable);

    /// @brief Get the current draw from a motor.
    /// @param id Motor ID.
    /// @return Raw current value.
    int16_t getCurrent(uint8_t id);

    /// @brief Get the calculated torque in N·m for a motor.
    /// @param id Motor ID.
    /// @return Torque in Newton-meters.
    float getTorque(uint8_t id);

    // -----------------------------------------------------------
    // Velocity commands
    // -----------------------------------------------------------

    /// @brief Set the velocity limit of a motor.
    /// @param id Motor ID.
    /// @param vel Velocity limit.
    void setVelocityLimit(uint8_t id, uint32_t vel);

    /// @brief Get the velocity limit of a motor.
    /// @param id Motor ID.
    /// @return Velocity limit.
    uint32_t getVelocityLimit(uint8_t id);

    // -----------------------------------------------------------
    // Acceleration commands
    // -----------------------------------------------------------

    /// @brief Set the acceleration limit of a motor.
    /// @param id Motor ID.
    /// @param acc Acceleration limit.
    void setAccelerationLimit(uint8_t id, uint32_t acc);

    /// @brief Get the acceleration limit of a motor.
    /// @param id Motor ID.
    /// @return Acceleration limit.
    uint32_t getAccelerationLimit(uint8_t id);

    // -----------------------------------------------------------
    // Motor-specific commands
    // -----------------------------------------------------------

    /// @brief Reboot a motor.
    /// @param id Motor ID.
    void rebootMotor(uint8_t id);

    /// @brief Ping a motor to verify communication.
    /// @param id Motor ID.
    void getMotorInfo(uint8_t id);

    /// @brief Set the baud rate of a motor.
    /// @param id Motor ID.
    /// @param baudrate New baud rate.
    void setBaudRate(uint8_t id, uint32_t baudrate);

    /// @brief Get the baud rate of a motor.
    /// @param id Motor ID.
    /// @return Baud rate.
    uint32_t getBaudRate(uint8_t id);

    /// @brief Set the LED state of a motor.
    /// @param id Motor ID.
    /// @param state True for on, false for off.
    void setMotorLED(uint8_t id, bool state);

    /// @brief Set the LED state of all motors.
    /// @param state True for on, false for off.
    void setAllMotorLED(bool state);

    /// @brief Current software version.
    static constexpr const char* VERSION = "1.2.1";

  private:
    /// @brief Dynamixel2Arduino object for motor communication.
    Dynamixel2Arduino dxl_;              // Handle to Dynamixel object

    /// @brief Pointer to array of motor IDs.
    const uint8_t* ids_;                 // List of motor IDs passed by the user

    /// @brief Number of motors configured.
    uint8_t numMotors_;                  // Number of motors being used, should be detected by the length of motor ids passed

    /// @brief Pointer to 2D array of joint limits [min, max] for each motor.
    const float (*jointLimits_)[2];      // Pointer to 2D array of joint limits

    /// @brief Array of zero offsets for each motor.
    float* zeroOffsets_;                 // Track absolute zero positions
};

#endif