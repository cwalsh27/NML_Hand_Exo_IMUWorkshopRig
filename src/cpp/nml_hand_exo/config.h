/**
 * @file config.h
 * @brief Header file for custom definitions and configs for the NML Hand Exoskeleton project.
 *
 */
#pragma once
#include <Arduino.h>

// ========= Board specific configuration ===================
/// @brief Serial port for Dynamixel communication.
/// @brief Pin assignment for the Dynamixel direction control pin.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  //#include <SoftwareSerial.h>
  //SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL Serial1
  #define COMMAND_SERIAL Serial2
  //#define BLE_SERIAL soft_serial
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define COMMAND_SERIAL Serial1
  //#define DEBUG_SERIAL Serial1
  //#define DEBUG_SERIAL SerialUSB
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL   Serial1
  #define COMMAND_SERIAL Serial2
  //#define DEBUG_SERIAL SerialUSB
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DEBUG_SERIAL Serial
  #define COMMAND_SERIAL Serial2
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DEBUG_SERIAL Serial
  #define COMMAND_SERIAL Serial2
  #define DXL_SERIAL   Serial3
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL Serial1
  #define COMMAND_SERIAL Serial2
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL   Serial1
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

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
//constexpr int STATUS_LED_PIN = LED_BUILTIN;
constexpr int STATUS_LED_PIN = 0;



// ======================================= USER CONFIGURATION ============================================
// =======================================================================================================

/// @brief Command delimiter for parsing commands from the serial input.
//constexpr char* COMMAND_DELIMITER = ";";
constexpr const char* COMMAND_DELIMITER = ";";

/// @brief Default exo mode on startup
constexpr const char* DEFAULT_EXO_MODE = "gesture_fixed"; // Available modes are "free", "gesture_fixed", and "gesture_continuous"

/// @brief Verbose output toggle for debugging.
constexpr bool DEFAULT_VERBOSE = true;

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
constexpr const char* MOTOR_NAMES[] = {  "wrist", "thumb", "index", "middle", "ring", "pinky" };

/// @brief Assign the physical joint limits for each motor after assembly on exo (these are found experimentally)
constexpr float jointLimits[6][2] = {
  {-189, 2840}, //189, 284   // open -> close
  {-126, 1470}, //126, 147
  {-242, 3020}, //242, 302
  {-142, 1950}, //142, 195
  {-158, 2260}, //158, 226
  {-138, 2140} //214, 138
};

// Assign a home position using the absolute position for each motor when the hand is fully open 
// Note that these are found experimentally
/// @brief Home states for each motor in degrees.
constexpr float HOME_STATES[] = {  208.5,  180.0,  240.5,  189.0,  154.5,  188.5 };

/// @brief Default baud rate for the debug serial connection.
constexpr long DEBUG_BAUD_RATE = 57600;

/// @brief Default baud rates for BLE communication.
constexpr long COMMAND_BAUD_RATE = 57600;

/// @brief Default baud rate for Dynamixel communication.
constexpr long DYNAMIXEL_BAUD_RATE = 57600;

/// @brief Total number of gesture contained in the library
constexpr int N_GESTURES = 7;

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

