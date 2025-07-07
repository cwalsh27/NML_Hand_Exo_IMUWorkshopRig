/**
 * @file utils.cpp
 * @brief A cpp file for supporting functions
 *
 */
#include <Arduino.h>
#include "nml_hand_exo.h"
#include "gesture_controller.h"

void debugPrint(const String& msg) {
  #if defined(DEBUG_SERIAL)
    DEBUG_SERIAL.println(msg);
  #else
    Serial.println(msg);  // fallback
  #endif
}


void flashPin(int pin, int durationMs, int repetitions) {
  for (int i = 0; i < repetitions; ++i) {
    digitalWrite(pin, HIGH);
    delay(durationMs);
    digitalWrite(pin, LOW);
    delay(durationMs);
  }
}

String getArg(const String line, const int index, char delimiter = ':') {
  int fromIndex = 0;
  for (int i = 0; i < index; ++i) {
    fromIndex = line.indexOf(delimiter, fromIndex);
    if (fromIndex == -1) return "";
    fromIndex += 1;
  }
  int toIndex = line.indexOf(delimiter, fromIndex);
  if (toIndex == -1) toIndex = line.length();
  return line.substring(fromIndex, toIndex);
}

int getArgMotorID(NMLHandExo& exo, const String& line, const int index) {
  return exo.getMotorID(getArg(line, index));
}

void parseMessage(NMLHandExo& exo, GestureController& gc, const String token) {

  token.trim();        // Remove any trailing white space
  token.toLowerCase(); // Set all characters to lowercase

  String cmd = getArg(token, 0, ':'); // Get the command part before the first colon
  int id = -1; // Default to -1 if not found
  int val = 0; // Default value for commands that require a value

  // ========== Supported high-level commands ==========
  if (cmd == "enable") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) exo.enableTorque(id, true);

  } else if (cmd == "disable") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) exo.enableTorque(id, false);

  } else if (cmd == "get_baud") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      uint32_t baud = exo.getBaudRate(id);  
      debugPrint("Motor " + String(id) + " baud: " + String(baud));
    }
  } else if (cmd == "set_baud") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setBaudRate(id, val);

  } else if (cmd == "get_vel") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      uint32_t vel = exo.getVelocityLimit(id);
      debugPrint("Motor " + String(id) + " velocity: " + String(vel));
    }

  } else if (cmd == "set_vel") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setVelocityLimit(id, val);

  } else if (cmd == "get_acc") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      int acc = exo.getAccelerationLimit(id);
      debugPrint("Motor " + String(id) + " acceleration: " + String(acc));
    }

  } else if (cmd == "set_acc") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setAccelerationLimit(id, val);

  } else if (cmd == "get_angle") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      float rel = exo.getRelativeAngle(id);
      debugPrint("Motor " + String(id) + " relative angle: " + String(rel, 2));
    }

  } else if (cmd == "set_angle") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setRelativeAngle(id, val);

  } else if (cmd == "get_absangle") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      float abs = exo.getAbsoluteAngle(id);
      debugPrint("Motor " + String(id) + " absolute angle: " + String(abs, 2));
    }

  } else if (cmd == "set_absangle") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setAbsoluteAngle(id, val);

  } else if (cmd == "get_zero") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      float zero = exo.getZeroAngle(id);
      debugPrint("Motor " + String(id) + " zero angle: " + String(zero, 2));
    }

  } else if (cmd == "set_zero") {
    String target = getArg(token, 1);
    if (target == "all") {
      exo.resetAllZeros();
    } else {
      id = exo.getMotorID(target);
      if (id != -1) exo.setZeroOffset(id);
    }

  } else if (cmd == "get_current") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      float current_mA = exo.getCurrent(id) * 2.69f;
      debugPrint("Motor " + String(id) + " current: " + String(current_mA, 2) + " mA");
    }

  } else if (cmd == "set_current_lim") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setCurrentLimit(id, val);

  } else if (cmd == "get_torque") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      float torque = exo.getTorque(id);
      //debugPrint("Motor " + String(id) + " torque: " + String(torque, 4) + " N·m");
      debugPrint("Motor " + String(id) + " torque: " + String(torque, 4) + " N·m");
    }

  } else if (cmd == "get_motor_limits") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      debugPrint("Motor " + String(id) + " limits: " + exo.getMotorLimits(id));
    }

  } else if (cmd == "set_upper_limit") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setMotorUpperBound(id, val);

  } else if (cmd == "set_lower_limit") {
    id = getArgMotorID(exo, token, 1);
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setMotorLowerBound(id, val);

  } else if (cmd == "set_motor_limits") {
    id = getArgMotorID(exo, token, 1);
    String limitsStr = getArg(token, 2);
    int colonIndex = limitsStr.indexOf(':');
    if (colonIndex != -1) {
      float lowerLimit = limitsStr.substring(0, colonIndex).toFloat();
      float upperLimit = limitsStr.substring(colonIndex + 1).toFloat();
      if (id != -1) exo.setMotorLimits(id, lowerLimit, upperLimit);
    } else {
      //debugPrint("Invalid limits format. Use 'lower:upper'");
      debugPrint("Invalid limits format. Use 'lower:upper'");
    }

  } else if (cmd == "led") {
    String target = getArg(token, 1);
    String stateStr = getArg(token, 2);
    bool state = (stateStr == "on" || stateStr == "1");

    target.trim(); target.toUpperCase();
    if (target == "ALL") {
      exo.setAllMotorLED(state);
    } else {
      id = exo.getMotorID(target);
      if (id != -1) exo.setMotorLED(id, state);
    }

  } else if (cmd == "debug") {
    String stateStr = getArg(token, 1);
    VERBOSE = (stateStr == "on" || stateStr == "1");
    debugPrint(" Debug state: " + String(VERBOSE ? "true" : "false"));

  } else if (cmd == "reboot") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) exo.rebootMotor(id);

  } else if (cmd == "home") {
    String target = getArg(token, 1);
    if (target == "all") {
      exo.homeAllMotors();
    } else {
      id = exo.getMotorID(target);
      if (id != -1) exo.setHome(id);
    }

  } else if (cmd == "get_motor_mode") {
    id = getArgMotorID(exo, token, 1);
    if (id != -1) {
      String mode = exo.getMotorControlMode(id);
      debugPrint("Motor " + String(id) + " mode: " + mode);
    } else {
      String mode = exo.getMotorControlMode();
      debugPrint("Motor " + String(id) + " mode: " + mode);
    }

  } else if (cmd == "set_motor_mode") {
    String modeStr = getArg(token, 1);
    if (modeStr == "position" || modeStr == "current_position" || modeStr == "velocity") {
      exo.setMotorControlMode(modeStr);

    } else {
      debugPrint("Invalid motor mode. Use 'position', 'current_position', or 'velocity'.");
    }

  } else if (cmd == "get_exo_mode") {
    String mode = exo.getExoOperatingMode();
    debugPrint("Exo device mode: " + mode);

  } else if (cmd == "set_exo_mode") {
    String modeStr = getArg(token, 1);
    exo.setExoOperatingMode(modeStr);
    
  } else if (cmd == "get_gesture") {
    String current_gesture = gc.getCurrentGesture();
    debugPrint("Current gesture: " + current_gesture);

  } else if (cmd == "gesture_list") {
    debugPrint("Ccommand not supported yet");

  } else if (cmd == "set_gesture") { 
    String gestureStr = getArg(token, 1);
    String stateStr = getArg(token, 2);
    gc.executeGesture(gestureStr, stateStr);
  
  } else if (cmd == "calibrate_exo") {
    String mode = getArg(token, 1);
    bool timed = (mode == "timed");
    float duration = getArg(token, 2).toFloat();
    if (duration <=0 ) duration = 10;
    exo.beginCalibration(timed, duration);

  } else if (cmd == "version") {
    debugPrint("Exo Device Version: " + String(NMLHandExo::VERSION));

  } else if (cmd == "info") {
    debugPrint("Device Info: ");
    exo.printDeviceInfo(*debugStream); 
  
  } else if (token == "help") {
    debugPrint(F(" ================================== List of commands ======================================"));
    debugPrint(F(" led              |  ID/NAME/ALL:ON/OFF  | // Turn motor LED on/off"));
    debugPrint(F(" help             |                      | // Display available commands"));
    debugPrint(F(" home             |  ID/NAME/ALL         | // Set specific motor (or all) to home position"));
    debugPrint(F(" info             |                      | // Gets information about exo device. Returns string of metadata with comma delimiters"));
    debugPrint(F(" debug            |  ON/OFF              | // Set verbose output on/off"));
    debugPrint(F(" reboot           |  ID/NAME             | // Reboot motor"));
    debugPrint(F(" version          |                      | // Get current software version"));
    debugPrint(F(" enable           |  ID/NAME             | // Enable torque for motor"));
    debugPrint(F(" disable          |  ID/NAME             | // Disable torque for motor"));
    debugPrint(F(" get_baud         |  ID/NAME             | // Get baud rate for motor"));
    debugPrint(F(" set_baud         |  ID/NAME:VALUE       | // Set baud rate for motor"));
    debugPrint(F(" get_vel          |  ID/NAME             | // Get current velocity profile for motor"));
    debugPrint(F(" set_vel          |  ID/NAME:VALUE       | // Set velocity profile for motor"));
    debugPrint(F(" get_acc          |  ID/NAME             | // Get current acceleration profile for motor"));
    debugPrint(F(" set_acc          |  ID/NAME:VALUE       | // Set acceleration limit for motor"));
    debugPrint(F(" get_zero         |  ID                  | // Get stored zero position"));
    debugPrint(F(" set_zero         |  ID/NAME             | // Set current position as new zero angle"));
    debugPrint(F(" get_angle        |  ID/NAME             | // Get relative motor angle"));
    debugPrint(F(" set_angle        |  ID/NAME:ANGLE       | // Set motor angle"));
    debugPrint(F(" get_absangle     |  ID/NAME             | // Get absolute motor angle"));
    debugPrint(F(" set_absangle     |  ID/NAME:ANGLE       | // Set absolute motor angle"));
    debugPrint(F(" get_torque       |  ID/NAME             | // Get torque output reading from motor"));
    debugPrint(F(" get_current      |  ID/NAME             | // Get current draw from motor"));
    debugPrint(F(" set_current_lim  |  ID/NAME:VAL         | // Set current draw limit for motor"));
    debugPrint(F(" set_current_lim  |  ID/NAME:VAL         | // Set current draw limit for motor"));
    debugPrint(F(" get_motor_limits |  ID/NAME             | // Get motor limits (upper and lower bounds)"));
    debugPrint(F(" set_upperpos_lim |  ID/NAME:ANGLE       | // Set the absolute upper bound position limit for the motor"));
    debugPrint(F(" set_lowerpos_lim |  ID/NAME:ANGLE       | // Set the absolute lower bound position limit for the motor"));
    debugPrint(F(" get_motor_mode   |                      | // Get motor control mode"));
    debugPrint(F(" set_motor_mode   |  VALUE               | // Set motor control mode ('POSITION', 'CURRENT_POSITION', 'VELOCITY')"));
    debugPrint(F(" get_exo_mode     |                      | // Get exo device operation mode"));
    debugPrint(F(" set_exo_mode     |  VALUE               | // Set exo device operation mode (FREE', 'GESTURE_FIXED', 'GESTURE_CONTINUOUS')"));
    debugPrint(F(" gesture_list     |                      | // Get gestures in library"));
    debugPrint(F(" set_gesture      |  NAME:VALUE          | // Set exo gesture"));
    debugPrint(F(" get_gesture      |                      | // Get exo gesture"));  
    debugPrint(F(" calibrate_exo    |  VALUE:VALUE         | // start the calibration routine for the exo"));
    debugPrint(F(" =========================================================================================="));
  } else {
    debugPrint("Unknown command: " + token);
  }
}

