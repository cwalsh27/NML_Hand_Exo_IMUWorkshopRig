/**
 * @file config.h
 * @brief Header file for custom definitions and configs for the NML Hand Exoskeleton project.
 *
 */
#pragma once
#include <Arduino.h>


// ======================================= PIN CONFIGURATION =============================================
// =======================================================================================================

// Pin definitions for mode switching and gesture control
//constexpr int CYCLE_GESTURE_PIN = 0;
constexpr int MODESWITCH_PIN = 1;
constexpr int GESTURE_PINCH_BUTTON_PIN = 2;
constexpr int GESTURE_KEYGRIP_BUTTON_PIN = 3;
constexpr int GESTURE_GRASP_BUTTON_PIN = 4;
constexpr int CYCLE_GESTURE_STATE_PIN = 5;

// I2C Pins (default)
//constexpr int SDL = 11;
//constexpr int SCA = 12;

// Serial port pins (also separated on board)
//constexpr int DYNAMIXEL_TX_PIN = 14; // TX pin for Dynamixel communication
//constexpr int DYNAMIXEL_RX_PIN = 13; // RX pin for Dynamixel communication

/// @brief Pin definition for the status LED (built-in LED on most boards)
constexpr int STATUS_LED_PIN = LED_BUILTIN;



// ======================================= USER CONFIGURATION ============================================
// =======================================================================================================

/// @brief Command delimiter for parsing commands from the serial input.
constexpr char* COMMAND_DELIMITER = ";";

/// @brief Default exo mode on startup
constexpr char* DEFAULT_EXO_MODE = "gesture_fixed"; // Available modes are "free", "gesture_fixed", and "gesture_continuous"

/// @brief Verbose output toggle for debugging.
constexpr bool DEFAULT_VERBOSE = false;

// Define servo IDs
constexpr uint8_t WRIST_ID  = 0;
constexpr uint8_t RING_ID   = 4; // 1
constexpr uint8_t PINKY_ID  = 5; // 2
constexpr uint8_t INDEX_ID  = 2; // 3
constexpr uint8_t MIDDLE_ID = 3; // 4
constexpr uint8_t THUMB_ID  = 1; // 5

/// @brief Motor ID Array (ordered by internal mapping you use)
constexpr uint8_t MOTOR_IDS[] = {  WRIST_ID, THUMB_ID, INDEX_ID, MIDDLE_ID, RING_ID, PINKY_ID};

/// @brief Motor name Array (must match the order above)
constexpr char* MOTOR_NAMES[] = {  "wrist", "thumb", "index", "middle", "ring", "pinky" };

/// @brief Assign the physical joint limits for each motor after assembly on exo (these are found experimentally)
constexpr float jointLimits[6][2] = {
  {189, 284}, //189, 284   // open -> close
  {126, 147}, //126, 147
  {242, 302}, //242, 302
  {142, 195}, //142, 195
  {158, 226}, //158, 226
  {138, 214} //214, 138
};

// Assign a home position using the absolute position for each motor (these are also found experimentally)
/// @brief Home states for each motor in degrees.
constexpr float HOME_STATES[] = {  208.0,  147.0,  268.0,  167.0,  185.0,  183.0 };

/// @brief Default baud rate for the debug serial connection.
constexpr long DEBUG_BAUD_RATE = 115200;

/// @brief Default baud rates for BLE communication.
constexpr long COMMAND_BAUD_RATE = 115200;

/// @brief Default baud rate for Dynamixel communication.
constexpr long DYNAMIXEL_BAUD_RATE = 115200;

/// @brief Total number of gesture contained in the library
constexpr int N_GESTURES = 3;

/// @brief Maximum number of gesture buttons that can be configured
constexpr int MAX_GESTURE_BUTTONS = 6; // Maximum number of gesture buttons that can be configured

/// @brief Maximum number of states configurable per gesture
constexpr long MAX_STATES_PER_GESTURE = 5;

/// @brief Default current limit for Dynamixel servos.
constexpr int MOTOR_CURRENT_LIMIT = 200;

/// @brief Debounce duration for mode switch button in milliseconds.
constexpr int BUTTON_DEBOUNCE_DURATION = 50; // ms debounce for physical button

/// @brief DYNAMIXEL protocol version used.
constexpr float DXL_PROTOCOL_VERSION = 2.0;

/// @brief Ticks per revolution for the Dynamixel servos.
constexpr int PULSE_RESOLUTION = 4096;

/// @brief Torque constant for XL330 servos, in N*m/mA.
constexpr float XL330_TORQUE_CONSTANT = 0.00038; // Nm / mA

// =======================================================================================================
// =======================================================================================================


/// @brief Number of motors in the system.
constexpr int N_MOTORS = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

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

