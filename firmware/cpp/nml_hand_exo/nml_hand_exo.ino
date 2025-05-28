#include "nml_hand_exo.h"

#define DEBUG_SERIAL Serial

bool verbose = true;

// Define servo IDs
#define WRIST_ID 1
#define THUMB_ID 2
#define INDEX_ID 3
#define MIDDLE_ID 4
#define RING_ID 5
#define PINKY_ID 6

const int numMotors = 1;
uint8_t servo_ids[] = { WRIST_ID };
const int jointLimits_[6][2] = { {0, 500} };
const int motor_baud_[] = { 57600 };  // Add more entries for additional motors

NMLHandExo exo(servo_ids, numMotors, jointLimits_, motor_baud_);
String inputString = "";

void debugPrint(const String& msg) {
  if (verbose) DEBUG_SERIAL.println("[DEBUG] " + msg);
}

int getMotorIDByName(const String& name) {
  String n = name;
  n.toUpperCase();
  if (n == "WRIST") return 1;
  if (n == "THUMB") return 2;
  if (n == "INDEX") return 3;
  if (n == "MIDDLE") return 4;
  if (n == "RING") return 5;
  if (n == "PINKY") return 6;
  return -1;
}

void setup() {
  DEBUG_SERIAL.begin(57600);
  exo.initializeSerial(servo_ids[0], motor_baud_[0]);
  exo.initializeMotors();
  exo.resetAllZeros();

  exo.setAllMotorLED(true); delay(250);
  exo.setAllMotorLED(false); delay(250);
  exo.setAllMotorLED(true); delay(250);
  exo.setAllMotorLED(false); delay(250);

  debugPrint("System calibrated. Ready to receive servo commands.");
}

void loop() {
  while (DEBUG_SERIAL.available()) {
    char inChar = (char)DEBUG_SERIAL.read();
    if (inChar == '\n') {
      inputString.trim();
      parseMessage(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}

void parseMessage(String token) {
  debugPrint("Received: " + token);

  auto parseTargetAndValue = [](const String& token) -> std::pair<int, int> {
    int first = token.indexOf(':');
    int second = token.indexOf(':', first + 1);
    if (second == -1) return {-1, -1};

    String target = token.substring(first + 1, second);
    int value = token.substring(second + 1).toInt();

    int id = target.toInt();
    if (id == 0 && target != "0") {
      id = getMotorIDByName(target);
    }
    return {id, value};
  };

  if (token.startsWith("enable:")) {
    String target = token.substring(7);
    int id = target.toInt();
    if (id == 0 && target != "0") id = getMotorIDByName(target);
    if (id != -1) {
      exo.setActiveBaud(id);
      exo.enableTorque(id, true);
    }

  } else if (token.startsWith("disable:")) {
    String target = token.substring(8);
    int id = target.toInt();
    if (id == 0 && target != "0") id = getMotorIDByName(target);
    if (id != -1) {
      exo.setActiveBaud(id);
      exo.enableTorque(id, false);
    }

  } else if (token.startsWith("set_baud:")) {
    auto [id, val] = parseTargetAndValue(token);
    if (id != -1) {
      exo.setActiveBaud(id);
      exo.setBaudRate(id, val);
    }

  } else if (token.startsWith("set_vel:")) {
    auto [id, val] = parseTargetAndValue(token);
    if (id != -1) {
      exo.setActiveBaud(id);
      exo.setVelocityLimit(id, val);
    }

  } else if (token.startsWith("set_acc:")) {
    auto [id, val] = parseTargetAndValue(token);
    if (id != -1) {
      exo.setActiveBaud(id);
      exo.setAccelerationLimit(id, val);
    }

  } else if (token.startsWith("set_angle:")) {
    auto [id, angle] = parseTargetAndValue(token);
    if (id != -1) {
      exo.setActiveBaud(id);
      exo.setAngleById(id, angle);
    }

  } else if (token.startsWith("get_angle:")) {
    String target = token.substring(token.indexOf(':') + 1);
    int id = target.toInt();
    if (id == 0 && target != "0") id = getMotorIDByName(target);
    if (id != -1) {
      float rel = exo.getRelativeAngle(id);
      if (verbose) {
        DEBUG_SERIAL.print("Relative angle for motor ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(": ");
        DEBUG_SERIAL.println(rel);
      }
    }

  } else if (token.startsWith("led:") || token.startsWith("LED:")) {
    int first = token.indexOf(':');
    int second = token.indexOf(':', first + 1);
    if (second != -1) {
      String target = token.substring(first + 1, second);
      String stateStr = token.substring(second + 1);
      bool state = (stateStr == "ON" || stateStr == "on");
      if (target == "ALL") {
        exo.setAllMotorLED(state);
      } else {
        int id = target.toInt();
        if (id == 0 && target != "0") id = getMotorIDByName(target);
        if (id != -1) exo.setMotorLED(id, state);
      }
    }

  } else if (token.startsWith("reboot:")) {
    String target = token.substring(7);
    int id = target.toInt();
    if (id == 0 && target != "0") id = getMotorIDByName(target);
    if (id != -1) exo.rebootMotor(id);

  } else if (token.startsWith("calibrate_zero:")) {
    String target = token.substring(token.indexOf(':') + 1);
    if (target == "ALL") exo.resetAllZeros();
    else {
      int id = target.toInt();
      if (id == 0 && target != "0") id = getMotorIDByName(target);
      if (id != -1) exo.calibrateZero(id);
    }

  } else if (token == "help") {
    DEBUG_SERIAL.println("================================= List of commands =============================================");
    DEBUG_SERIAL.println(" help        |                  | // Display available commands");
    DEBUG_SERIAL.println(" enable      |  ID/NAME         | // Enable torque for motor");
    DEBUG_SERIAL.println(" disable     |  ID/NAME         | // Disable torque for motor");
    DEBUG_SERIAL.println(" set_baud    |  ID/NAME:VALUE   | // Set baud rate for motor");
    DEBUG_SERIAL.println(" set_vel     |  ID/NAME:VALUE   | // Set velocity limit for motor");
    DEBUG_SERIAL.println(" set_acc     |  ID/NAME:VALUE   | // Set acceleration limit for motor");
    DEBUG_SERIAL.println(" set_angle   |  ID/NAME:ANGLE   | // Set motor angle");
    DEBUG_SERIAL.println(" get_angle   |  ID/NAME         | // Get relative motor angle");
    DEBUG_SERIAL.println(" led         |  ID/NAME:ON/OFF  | // Turn motor LED on/off");
    DEBUG_SERIAL.println(" reboot      |  ID/NAME         | // Reboot motor");
    DEBUG_SERIAL.println(" calibrate_zero | ID/NAME/ALL   | // Set current position as zero");
    DEBUG_SERIAL.println(" ================================================================================================");
  } else {
    if (verbose) DEBUG_SERIAL.println("Unknown command: " + token);
  }
}
