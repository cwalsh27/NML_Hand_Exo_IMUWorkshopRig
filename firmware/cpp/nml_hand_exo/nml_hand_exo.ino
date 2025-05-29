#include "nml_hand_exo.h"
#define DEBUG_SERIAL Serial

// Define servo IDs
#define WRIST_ID 1
#define THUMB_ID 2
#define INDEX_ID 3
#define MIDDLE_ID 4
#define RING_ID 5
#define PINKY_ID 6

// Assign the motor arrangement as a list of ids
uint8_t ids[] = { 
  WRIST_ID,
  THUMB_ID,
  INDEX_ID,
  MIDDLE_ID,
  RING_ID,
  PINKY_ID
};

// Assign the phsyical joint limits for each motor (need to determine this experimentally)
const int jointLimits_[6][2] = { 
  {-90, 90},  // WRIST
  {-45, 90},  // THUMB
  {-10, 90},  // INDEX
  {-10, 90},  // MIDDLE
  {-10, 90},  // RING
  {-10, 90}   // PINKY
};

NMLHandExo exo(ids, sizeof(ids)/sizeof(ids[0]), jointLimits_);

// Incoming message buffer
String inputString = "";


// Helper function to parse the motor id and value 
int parseTargetAndValue(const String& token, int& id, int& value) {
  int first = token.indexOf(':');
  int second = token.indexOf(':', first + 1);
  if (second == -1) return -1;

  String target = token.substring(first + 1, second);
  String valStr = token.substring(second + 1);

  id = target.toInt();
  if (id == 0 && target != "0") {
    id = exo.getMotorID(target);
  }

  value = valStr.toInt();
  return 0;
}

// Setup function
void setup() {

  VERBOSE = false;

  DEBUG_SERIAL.begin(57600);
  exo.initializeSerial(57600);
  exo.initializeMotors();
  exo.resetAllZeros();

  // Flash LEDs to let user know system ready to go 
  exo.setAllMotorLED(true); delay(250);
  exo.setAllMotorLED(false); delay(250);
  exo.setAllMotorLED(true); delay(250);
  exo.setAllMotorLED(false); delay(250);

  debugPrint("System calibrated. Ready to receive servo commands.");
}


// Main
void loop() {
  while (DEBUG_SERIAL.available()) {
    char inChar = (char)DEBUG_SERIAL.read();
    if (inChar == '\n') {
      
      inputString.trim();        // Remove any trailing white space
      inputString.toLowerCase(); // Set all characters to lowercase

      debugPrint("Received: " + inputString);
      parseMessage(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}

void parseMessage(String token) {

  // ========== Supported high-level commands ==========
  if (token.startsWith("enable:")) {
    int id = exo.getMotorID(token.substring(7));
    if (id != -1) exo.enableTorque(id, true);

  } else if (token.startsWith("disable:")) {
    int id = exo.getMotorID(token.substring(8));
    if (id != -1) exo.enableTorque(id, false);

  } else if (token.startsWith("set_baud:")) {
    int id, val;
    if (parseTargetAndValue(token, id, val) == 0 && id != -1){
      exo.setBaudRate(id, val);
    }
  } else if (token.startsWith("get_baud:")) {
    int id = exo.getMotorID(token.substring(8));
    if (id != -1) {
      uint32_t baud = exo.getBaudRate(id);  
      debugPrint("Motor " + String(id) + " baud: " + String(baud));
    }
  } else if (token.startsWith("set_vel:")) {
    int id, val;
    if (parseTargetAndValue(token, id, val) == 0 && id != -1){
      exo.setVelocityLimit(id, val);
    }

  } else if (token.startsWith("get_vel:")) {
    int id = exo.getMotorID(token.substring(9));
    if (id != -1) {
      uint32_t vel = exo.getVelocityLimit(id);
      debugPrint("Motor " + String(id) + " velocity: " + String(vel));
    }

  } else if (token.startsWith("set_acc:")) {
    int id, val;
    if (parseTargetAndValue(token, id, val) == 0 && id != -1) {
      exo.setAccelerationLimit(id, val);
    }

  } else if (token.startsWith("get_acc:")) {
    int id = exo.getMotorID(token.substring(9));
    if (id != -1){
      int acc = exo.getAccelerationLimit(id);
      debugPrint("Motor " + String(id) + " acceleration: " + String(acc));
    }

  } else if (token.startsWith("set_angle:")) {
    int id, val;
    if (parseTargetAndValue(token, id, val) == 0 && id != -1) {
      exo.setAngleById(id, val);
    }

  } else if (token.startsWith("get_angle:")) {
    int id = exo.getMotorID(token.substring(10));
    if (id != -1){ 
      float rel = exo.getRelativeAngle(id);
      debugPrint("Motor " + String(id) + " relative angle: " + String(rel, 2));
    }

  } else if (token.startsWith("get_current:")) {
    int id = exo.getMotorID(token.substring(12));
    if (id != -1) {
      float current_mA = exo.getCurrent(id) * 2.69f;
      debugPrint("Motor " + String(id) + " current: " + String(current_mA, 2) + " mA");
    }

  } else if (token.startsWith("get_torque:")) {
    int id = exo.getMotorID(token.substring(11));
    if (id != -1){
      float torque = exo.getTorque(id);
      debugPrint("Motor " + String(id) + " torque: " + String(torque, 4) + " N·m");
    }

  } else if (token.startsWith("led:")) {
  int first = token.indexOf(':');
  int second = token.indexOf(':', first + 1);
  if (second != -1) {
    String target = token.substring(first + 1, second);
    String stateStr = token.substring(second + 1);
    bool state = (stateStr == "ON" || stateStr == "on" || stateStr == "1");

    target.trim();
    target.toUpperCase();

    if (target == "ALL") {
      exo.setAllMotorLED(state);
    } else {
      int id = exo.getMotorID(target);
      if (id != -1) exo.setMotorLED(id, state); else debugPrint("Invalid LED target: " + target);
    }
  }

  } else if (token.startsWith("reboot:")) {
    int id = exo.getMotorID(token.substring(7));
    exo.rebootMotor(id);

  } else if (token.startsWith("set_zero:")) {
    String target = token.substring(15);
    if (target == "all") exo.resetAllZeros();
    else {
      int id = exo.getMotorID(target);
      if (id != -1) exo.calibrateZero(id);
    }

  } else if (token == "help") {
    DEBUG_SERIAL.println(" ================================= List of commands =============================================");
    DEBUG_SERIAL.println(" help        |                      | // Display available commands");
    DEBUG_SERIAL.println(" enable      |  ID/NAME             | // Enable torque for motor");
    DEBUG_SERIAL.println(" disable     |  ID/NAME             | // Disable torque for motor");
    DEBUG_SERIAL.println(" set_baud    |  ID/NAME:VALUE       | // Set baud rate for motor");
    DEBUG_SERIAL.println(" get_baud    |  ID/NAME             | // Get baud rate for motor");
    DEBUG_SERIAL.println(" set_vel     |  ID/NAME:VALUE       | // Set velocity profile for motor");
    DEBUG_SERIAL.println(" get_vel     |  ID/NAME             | // Get current velocity profile for motor");
    DEBUG_SERIAL.println(" set_acc     |  ID/NAME:VALUE       | // Set acceleration limit for motor");
    DEBUG_SERIAL.println(" get_acc     |  ID/NAME             | // Get current acceleration profile for motor");
    DEBUG_SERIAL.println(" set_angle   |  ID/NAME:ANGLE       | // Set motor angle");
    DEBUG_SERIAL.println(" get_angle   |  ID/NAME             | // Get relative motor angle");
    DEBUG_SERIAL.println(" get_torque  |  ID/NAME             | // Get torque output reading from motor");
    DEBUG_SERIAL.println(" get_current |  ID/NAME             | // Get current draw from motor");
    DEBUG_SERIAL.println(" led         |  ID/NAME/ALL:ON/OFF  | // Turn motor LED on/off");
    DEBUG_SERIAL.println(" reboot      |  ID/NAME             | // Reboot motor");
    DEBUG_SERIAL.println(" set_zero    |  ID/NAME/ALL         | // Set current position as new zero angle");
    DEBUG_SERIAL.println(" get_zero    |  ID                  | // Get stored zero position");
    DEBUG_SERIAL.println(" ================================================================================================");
  } else {
    debugPrint("Unknown command: " + token);
  }
}
