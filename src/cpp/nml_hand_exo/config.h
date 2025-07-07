/**
 * @file config.h
 * @brief Header file for custom definitions and configs for the NML Hand Exoskeleton project.
 *
 */
#pragma once
#include <Arduino.h>


/// @brief DYNAMIXEL protocol version used.
constexpr float DXL_PROTOCOL_VERSION = 2.0;

/// @brief Ticks per revolution for the Dynamixel servos.
constexpr int PULSE_RESOLUTION = 4096;

/// @brief Torque constant for XL330 servos, in N*m/mA.
constexpr float XL330_TORQUE_CONSTANT = 0.00038; // Nm / mA

/// @brief Default current limit for Dynamixel servos.
constexpr int MOTOR_CURRENT_LIMIT = 200;

/// @brief Debounce duration for mode switch button in milliseconds.
constexpr int BUTTON_DEBOUNCE_DURATION = 50; // ms debounce for physical button


// @brief Debug serial stream used for logging.
//extern Stream& DEBUG_SERIAL;

// @brief Pointer to the debug stream used for conditional logging.
//constexpr Stream* debugStream;


// Direction control pin - define per board type
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
constexpr int DXL_DIR_PIN = 2;
#define DEBUG_SERIAL Serial

#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
constexpr int DXL_DIR_PIN = 2;
#elif defined(ARDUINO_OpenCM904)
constexpr int DXL_DIR_PIN = 22;
#elif defined(ARDUINO_OpenCR)
constexpr int DXL_DIR_PIN = 84;
#elif defined(ARDUINO_OpenRB)
constexpr int DXL_DIR_PIN = -1;
#else
constexpr int DXL_DIR_PIN = 2;
#endif

// ======================================= USER CONFIGURATION ============================================
// =======================================================================================================

// Define servo IDs
constexpr uint8_t WRIST_ID  = 0;
constexpr uint8_t RING_ID   = 1;
constexpr uint8_t PINKY_ID  = 2;
constexpr uint8_t INDEX_ID  = 3;
constexpr uint8_t MIDDLE_ID = 4;
constexpr uint8_t THUMB_ID  = 5;

constexpr int STATUS_LED_PIN = LED_BUILTIN;

// Define input pins for buttons
constexpr int MODESWITCH_PIN = 12;
constexpr int GESTURE_STATE_PIN = 11;
constexpr int CYCLE_GESTURE_PIN = 10;

// Assign the phsyical joint limits for each motor after assembly on exo (these are found experimentally)
constexpr float jointLimits[6][2] = {
  {0, 300}, 
  {187, 217}, 
  {162, 186}, 
  {278, 300}, 
  {140, 172}, 
  {0, 300}
};

// Assign a home position using the absolute poition for each motor (these are also found experimentally)
constexpr float homeStates[] = {
  0.0,
  195.0,
  170.0,
  276.0,
  168.0,
  0.0
};

constexpr bool DEFAULT_VERBOSE = true;

// Baud rates for serial devices
constexpr long DEBUG_BAUD_RATE = 57600;
constexpr long BLE_BAUD_RATE = 9600;
constexpr long DYNAMIXEL_BAUD_RATE = 57600;

// Gesture Library parameters
constexpr long N_GESTURES = 3;
constexpr long MAX_STATES_PER_GESTURE = 5;
constexpr long NUM_JOINTS = 6;


// =======================================================================================================
// =======================================================================================================
