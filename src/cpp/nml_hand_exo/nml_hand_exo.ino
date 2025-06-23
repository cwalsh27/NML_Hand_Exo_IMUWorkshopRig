/*
MIT License

Copyright (c) 2025 Jonathan Shulgach

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "nml_hand_exo.h"
#define DEBUG_SERIAL Serial
#define STATUS_LED 0


// ======================================= USER CONFIGURATION ============================================
// =======================================================================================================
// Define servo IDs
#define WRIST_ID 0
#define RING_ID 1
#define PINKY_ID 2
#define INDEX_ID 3
#define MIDDLE_ID 4
#define THUMB_ID 5

// Assign the motor arrangement as a list of ids (For now manyg sure the id order is increasing)
uint8_t ids[] = { WRIST_ID, RING_ID, PINKY_ID, INDEX_ID, MIDDLE_ID, THUMB_ID };


// Assign the phsyical joint limits for each motor, relative to home position (these are found experimentally)
// Measure these everytime a motor is reinstalled
const float jointLimits[6][2] = { 
  {0, 300}, //{-720, 720},   // WRIST,  Home ??
  {187, 217}, //{-45, 90},   // RING ,  Home: 195
  {162, 186}, //{-10, 90},   // PINKY,  Home: 170
  {278, 300}, //{-10, 90},   // INDEX,  Home: 276
  {140, 172}, //{-10, 90},   // MIDDLE, Home: 168
  {0, 300}, //{-10, 90}      // THUMB,  Home ??
};

// Assign a home position using the absolute poition for each motor (these are found experimentally)
const float homeStates[] = { 0.0, 195.0, 170.0, 276.0, 168.0, 0.0 };


// =======================================================================================================
// =======================================================================================================

uint8_t n_motors = sizeof(ids)/sizeof(ids[0]);

// Create the exo device with the motor parameters and id values
NMLHandExo exo(ids, n_motors, jointLimits, homeStates);

// Incoming message buffer
String inputString = "";

// Setup function
void setup() {

  // Enable/disable debugging output. More useful if debugging with the Arduino serial monitor and testing commands
  VERBOSE = false;

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);  // initially off

  DEBUG_SERIAL.begin(57600);    // Setting a default baud rate of 57600
  exo.initializeSerial(57600);
  exo.initializeMotors();       // Initialize motors and set them to "current position" mode
  // exo.resetAllZeros();       // (Optional) Defines the current position of the motors as the home position.

  // Flash LEDs to let user know system ready to go 
  digitalWrite(STATUS_LED, HIGH);
  exo.setAllMotorLED(true); 
  delay(250);
  digitalWrite(STATUS_LED, LOW);
  exo.setAllMotorLED(false); 
  delay(250);
  digitalWrite(STATUS_LED, HIGH);
  exo.setAllMotorLED(true); 
  delay(250);
  digitalWrite(STATUS_LED, LOW);
  exo.setAllMotorLED(false); 
  delay(250);
  //digitalWrite(STATUS_LED, HIGH);

  debugPrint("Exo device ready to receive commands. Ready to receive servo commands.");
}


// Main
void loop() {
  if (DEBUG_SERIAL.available() > 0) {
    while (DEBUG_SERIAL.available()) {
      char inChar = (char)DEBUG_SERIAL.read();
      if (inChar == '\n') {

        // End of line, process the input string
        debugPrint("Received: " + inputString);
        parseMessage(inputString);
        inputString = "";
      } else {
        inputString += inChar;
      }
    }

    digitalWrite(STATUS_LED, HIGH);

  } else {
    //pass
  }
}

String getArg(String line, int index, char delimiter = ':') {
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

int getArgMotorID(const String& line, int index) {
  // Helper function to get the motor ID from a command line with the exo function
  return exo.getMotorID(getArg(line, index));
}

void parseMessage(String token) {

  token.trim();        // Remove any trailing white space
  token.toLowerCase(); // Set all characters to lowercase

  String cmd = getArg(token, 0, ':'); // Get the command part before the first colon
  int id = -1; // Default to -1 if not found
  int val = 0; // Default value for commands that require a value

  // ========== Supported high-level commands ==========
  if (cmd == "enable") {
    int id = getArgMotorID(token,1);
    if (id != -1) exo.enableTorque(id, true);

  } else if (cmd == "disable") {
    int id = getArgMotorID(token,1);
    if (id != -1) exo.enableTorque(id, false);

  } else if (cmd == "get_baud") {
    int id = getArgMotorID(token,1);
    if (id != -1) {
      uint32_t baud = exo.getBaudRate(id);  
      debugPrint("Motor " + String(id) + " baud: " + String(baud));
    }
  } else if (cmd == "set_baud") {
    id = exo.getMotorID(getArg(token, 1));
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setBaudRate(id, val);

  } else if (cmd == "get_vel") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      uint32_t vel = exo.getVelocityLimit(id);
      DEBUG_SERIAL.println("Motor " + String(id) + " velocity: " + String(vel));
    }

  } else if (cmd == "set_vel") {
    id = exo.getMotorID(getArg(token, 1));
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setVelocityLimit(id, val);

  } else if (cmd == "get_acc") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      int acc = exo.getAccelerationLimit(id);
      DEBUG_SERIAL.println("Motor " + String(id) + " acceleration: " + String(acc));
    }

  } else if (cmd == "set_acc") {
    id = exo.getMotorID(getArg(token, 1));
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setAccelerationLimit(id, val);

  } else if (cmd == "get_angle") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      float rel = exo.getRelativeAngle(id);
      DEBUG_SERIAL.println("Motor " + String(id) + " relative angle: " + String(rel, 2));
    }

  } else if (cmd == "set_angle") {
    id = exo.getMotorID(getArg(token, 1));
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setRelativeAngle(id, val);

  } else if (cmd == "get_absangle") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      float abs = exo.getAbsoluteAngle(id);
      DEBUG_SERIAL.println("Motor " + String(id) + " absolute angle: " + String(abs, 2));
    }

  } else if (cmd == "set_absangle") {
    id = exo.getMotorID(getArg(token, 1));
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setAbsoluteAngle(id, val);

  } else if (cmd == "get_zero") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      float zero = exo.getZeroAngle(id);
      DEBUG_SERIAL.println("Motor " + String(id) + " zero angle: " + String(zero, 2));
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
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      float current_mA = exo.getCurrent(id) * 2.69f;
      DEBUG_SERIAL.println("Motor " + String(id) + " current: " + String(current_mA, 2) + " mA");
    }

  } else if (cmd == "set_current_lim") {
    id = exo.getMotorID(getArg(token, 1));
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setCurrentLimit(id, val);

  } else if (cmd == "get_torque") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      float torque = exo.getTorque(id);
      DEBUG_SERIAL.println("Motor " + String(id) + " torque: " + String(torque, 4) + " N·m");
    }

  } else if (cmd == "get_motor_limits") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) {
      DEBUG_SERIAL.println("Motor " + String(id) + " limits: " + exo.getMotorLimits(id));
    }

  } else if (cmd == "set_upper_limit") {
    id = exo.getMotorID(getArg(token, 1));
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setMotorUpperBound(id, val);

  } else if (cmd == "set_lower_limit") {
    id = exo.getMotorID(getArg(token, 1));
    val = getArg(token, 2).toInt();
    if (id != -1) exo.setMotorLowerBound(id, val);

  } else if (cmd == "set_motor_limits") {
    id = exo.getMotorID(getArg(token, 1));
    String limitsStr = getArg(token, 2);
    int colonIndex = limitsStr.indexOf(':');
    if (colonIndex != -1) {
      float lowerLimit = limitsStr.substring(0, colonIndex).toFloat();
      float upperLimit = limitsStr.substring(colonIndex + 1).toFloat();
      if (id != -1) exo.setMotorLimits(id, lowerLimit, upperLimit);
    } else {
      DEBUG_SERIAL.println("Invalid limits format. Use 'lower:upper'");
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
    DEBUG_SERIAL.println(" Debug state: " + String(VERBOSE ? "true" : "false"));

  } else if (cmd == "reboot") {
    id = exo.getMotorID(getArg(token, 1));
    if (id != -1) exo.rebootMotor(id);

  } else if (cmd == "home") {
    String target = getArg(token, 1);
    if (target == "all") {
      exo.homeAllMotors();
    } else {
      id = exo.getMotorID(target);
      if (id != -1) exo.setHome(id);
    }
  } else if (cmd == "version") {
    DEBUG_SERIAL.println("Exo Device Version: " + String(NMLHandExo::VERSION));

  } else if (cmd == "info") {
    DEBUG_SERIAL.println("Device Info: " + exo.getDeviceInfo());

  } else if (token == "help") {
    DEBUG_SERIAL.println(" ========================================= List of commands ================================================");
    DEBUG_SERIAL.println(" led              |  ID/NAME/ALL:ON/OFF  | // Turn motor LED on/off");
    DEBUG_SERIAL.println(" help             |                      | // Display available commands");
    DEBUG_SERIAL.println(" home             |  ID/NAME/ALL         | // Set specific motor (or all) to home position");
    DEBUG_SERIAL.println(" info             |                      | // Gets information about exo device. Returns string of metadata with comma delimiters");
    DEBUG_SERIAL.println(" debug            |  ON/OFF              | // Set verbose output on/off");
    DEBUG_SERIAL.println(" reboot           |  ID/NAME             | // Reboot motor");
    DEBUG_SERIAL.println(" version          |                      | // Get current software version");
    DEBUG_SERIAL.println(" enable           |  ID/NAME             | // Enable torque for motor");
    DEBUG_SERIAL.println(" disable          |  ID/NAME             | // Disable torque for motor");
    DEBUG_SERIAL.println(" get_baud         |  ID/NAME             | // Get baud rate for motor");
    DEBUG_SERIAL.println(" set_baud         |  ID/NAME:VALUE       | // Set baud rate for motor");
    DEBUG_SERIAL.println(" get_vel          |  ID/NAME             | // Get current velocity profile for motor");
    DEBUG_SERIAL.println(" set_vel          |  ID/NAME:VALUE       | // Set velocity profile for motor");
    DEBUG_SERIAL.println(" get_acc          |  ID/NAME             | // Get current acceleration profile for motor");
    DEBUG_SERIAL.println(" set_acc          |  ID/NAME:VALUE       | // Set acceleration limit for motor");
    DEBUG_SERIAL.println(" get_zero         |  ID                  | // Get stored zero position");
    DEBUG_SERIAL.println(" set_zero         |  ID/NAME             | // Set current position as new zero angle");
    DEBUG_SERIAL.println(" get_angle        |  ID/NAME             | // Get relative motor angle");
    DEBUG_SERIAL.println(" set_angle        |  ID/NAME:ANGLE       | // Set motor angle");
    DEBUG_SERIAL.println(" get_absangle     |  ID/NAME             | // Get absolute motor angle");
    DEBUG_SERIAL.println(" set_absangle     |  ID/NAME:ANGLE       | // Set absolute motor angle");
    DEBUG_SERIAL.println(" get_torque       |  ID/NAME             | // Get torque output reading from motor");
    DEBUG_SERIAL.println(" get_current      |  ID/NAME             | // Get current draw from motor");
    DEBUG_SERIAL.println(" set_current_lim  |  ID/NAME:VAL         | // Set current draw limit for motor");
    DEBUG_SERIAL.println(" set_current_lim  |  ID/NAME:VAL         | // Set current draw limit for motor");
    DEBUG_SERIAL.println(" get_motor_limits |  ID/NAME             | // Get motor limits (upper and lower bounds)");
    DEBUG_SERIAL.println(" set_upperpos_lim |  ID/NAME:ANGLE       | // Set the absolute upperbound position limit for the motor");
    DEBUG_SERIAL.println(" set_lowerpos_lim |  ID/NAME:ANGLE       | // Set the absolute lowerbound position limit for the motor");
    DEBUG_SERIAL.println(" ===================================================================================================");
  } else {
    debugPrint("Unknown command: " + token);
  }
}
