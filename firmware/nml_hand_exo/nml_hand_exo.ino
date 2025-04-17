#include "nml_hand_exo.h"  // Include the helper functions
#include <Dynamixel2Arduino.h>

#define DEBUG_SERIAL Serial

// Define servo IDs
#define WRIST_ID 1
#define THUMB_ID 2
#define INDEX_ID 3
#define MIDDLE_ID 4
#define RING_ID 5
#define PINKY_ID 6

// Global definitions
bool verbose = false;
const int numMotors = 6;
uint8_t servo_ids[] = {WRIST_ID, THUMB_ID, INDEX_ID, MIDDLE_ID, RING_ID, PINKY_ID};

// Define the programatical joint limits here
const int jointLimits_[6][2] = {
  {0, 500},   // CCW 0 - 500
  {500, 1000}, // DOWN - UP
  {650, 900}, // DOWN - UP
  {0,   300}, // UP - DOWN
  {200, 600}, // UP - DOWN
  {120, 880}
};

// Serial input handling
String inputString = "";

// Exo device class
NMLHandExo exo(servo_ids, numMotors, jointLimits_);

void setup() {
  DEBUG_SERIAL.begin(115200);  // High-speed Serial Monitor
  exo.initializeSerial(1000000);    // XL-320 communication baud rate

  // Initialize motors by setting output mode to position. At rest the motor positions upon calibration were found:
  // Wrist: 500, CCW 0-1028, 90 degrees
  // Thumb: 900, 
  // Index: 800
  // Middle: 130
  // Ring: 500
  // Pinky: 
  exo.initializeMotors();
  exo.resetAllZeros();  // Calibrate zero for all motors based on current position


  // Flash the LED on all the motors to let the user know we are ready
  exo.setAllMotorLED(true);
  delay(500);
  exo.setAllMotorLED(false);
  delay(500);
  if (verbose) DEBUG_SERIAL.println("System calibrated. Ready to receive servo commands.");  
}

void loop() {

  // Get the position of motor 1
  //exo.getDynamixelInfo(1);
  //delay(500);

  while (DEBUG_SERIAL.available()) {
    char inChar = (char)DEBUG_SERIAL.read();
    if (inChar == '\n') {
      parseMessage(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}

void parseMessage(String message) {
  if (verbose) DEBUG_SERIAL.println("Parsing: " + message);
  int idx = 0;

  while ((idx = message.indexOf(';')) != -1) {
    String token = message.substring(0, idx);
    message = message.substring(idx + 1);

    token.trim();  // Remove whitespace

    // ------------------ set_joints ------------------
    if (token.startsWith("set_joints:")) {
      String values = token.substring(token.indexOf(':') + 1);
      values.replace("[", "");
      values.replace("]", "");
      values.trim();

      int pos[6] = {0};  // Default to 0
      int i = 0;
      int commaIdx = -1;

      while ((commaIdx = values.indexOf(',')) != -1 && i < 6) {
        pos[i++] = values.substring(0, commaIdx).toInt();
        values = values.substring(commaIdx + 1);
      }
      if (i < 6 && values.length() > 0) pos[i++] = values.toInt();

      for (int j = 0; j < i; j++) {
        exo.setPositionById(j + 1, pos[j]);  // IDs assumed 1–6
      }

    // ------------------ set_joint ------------------
    } 
    else if (token.startsWith("set_joint:")) {
      int first = token.indexOf(':');
      int second = token.indexOf(':', first + 1);

      if (first != -1 && second != -1) {
        String target = token.substring(first + 1, second);
        int position = token.substring(second + 1).toInt();

        // By ID
        if (target.toInt() != 0 || target == "0") {
          exo.setPositionById(target.toInt(), position);
        }
        // By Name
        else {
          target.toUpperCase();
          if (target == "THUMB") exo.setPositionById(2, position);
          else if (target == "INDEX") exo.setPositionById(3, position);
          else if (target == "MIDDLE") exo.setPositionById(4, position);
          else if (target == "RING") exo.setPositionById(5, position);
          else if (target == "PINKY") exo.setPositionById(6, position);
          else if (target == "WRIST") exo.setPositionById(1, position);
        }
      }

    // -------------- get_joint ----------------
    } 
    else if (token.startsWith("get_joint:")) {
      String jointStr = token.substring(token.indexOf(':') + 1);
      jointStr.trim();

      int value = -1;

      // Try numeric ID first
      if (jointStr.toInt() != 0 || jointStr == "0") {
        value = exo.getPositionById(jointStr.toInt());
      } else {
        value = exo.getPositionByAlias(jointStr);
      }

      if (value != -1) {
        if (verbose) {
          DEBUG_SERIAL.print("Position of ");
          DEBUG_SERIAL.print(jointStr);
          DEBUG_SERIAL.print(": ");
          DEBUG_SERIAL.println(value);
        }
      } else {
        if (verbose) DEBUG_SERIAL.println("Unknown joint: " + jointStr);
      }

    // ------------------ LED ------------------
    } 
    else if (token.startsWith("LED:") || token.startsWith("led:")) {
      int first = token.indexOf(':');
      int second = token.indexOf(':', first + 1);

      if (second != -1) {
        String target = token.substring(first + 1, second);
        String stateStr = token.substring(second + 1);
        bool state = (stateStr == "ON" || stateStr == "on");

        if (target == "ALL") {
          exo.setAllMotorLED(state);
        } else {
          uint8_t id = target.toInt();
          exo.setMotorLED(id, state);
        }
      }

    } 
    else if (token.startsWith("REBOOT:") || token.startsWith("reboot:")) {
      String idStr = token.substring(7);
      uint8_t id = idStr.toInt();
      exo.rebootMotor(id);
      if (verbose) DEBUG_SERIAL.println("Rebooted motor ID: " + String(id));

    } 
    else if (token.startsWith("set_angle:")) {
      // Example: set_angle:1:45;  or set_angle:INDEX:90;
      int first = token.indexOf(':');
      int second = token.indexOf(':', first + 1);
      String target = token.substring(first + 1, second);
      float angle = token.substring(second + 1).toFloat();

      if (target.toInt() != 0 || target == "0") {
        exo.setAngleById(target.toInt(), angle);
      } else {
        exo.setAngleByAlias(target, angle);
      }
    }
    else if (token.startsWith("calibrate_zero:")) {
      String idStr = token.substring(token.indexOf(':') + 1);
      if (idStr == "ALL") exo.resetAllZeros();
      else exo.calibrateZero(idStr.toInt());
    }
    else if (token.startsWith("get_angle:")) {
      String idStr = token.substring(token.indexOf(':') + 1);
      uint8_t id = idStr.toInt();
      float rel = exo.getRelativeAngle(id);
      if (verbose) {
        DEBUG_SERIAL.print("Relative angle for motor ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(": ");
        DEBUG_SERIAL.println(rel);
      }
    }
    else {
      if (verbose) DEBUG_SERIAL.println("Unknown command: " + token);
    }
  }
}
