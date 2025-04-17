#include "nml_hand_exo.h"
#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  //#define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  //#define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  //#define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  //#define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  //#define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  //#define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  //#define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

const float DXL_PROTOCOL_VERSION = 2.0;

NMLHandExo::NMLHandExo(const uint8_t* ids, int motorCount, const int jointLimits[][2])
  : dxl_(DXL_SERIAL, DXL_DIR_PIN), ids_(ids), motorCount_(motorCount), jointLimits_(jointLimits) {}


void NMLHandExo::initializeSerial(int baud) {
  // Initialize serial communication with DYNAMIXEL hardware using the specified baudrate. Has to match hardware
  dxl_.begin(baud);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}
void NMLHandExo::initializeMotors() {
  for (int i = 0; i < motorCount_; i++) {
    uint8_t id = ids_[i];
    dxl_.torqueOff(id);
    dxl_.setOperatingMode(id, OP_POSITION);
    dxl_.torqueOn(id);
  }
}
void NMLHandExo::setPositionById(uint8_t id, int position) {
  int index = getIndexById(id);
  if (index == -1) return;

  position = constrain(position, jointLimits_[index][0], jointLimits_[index][1]);
  dxl_.setGoalPosition(id, position);
}
int NMLHandExo::getPositionById(uint8_t id) {
  return dxl_.getPresentPosition(id);
}
void NMLHandExo::setPositionByName(const String& name, int position) {
  if (name.startsWith("F")) {
    int fingerNum = name.substring(1).toInt();
    if (fingerNum >= 1 && fingerNum <= 5) {
      setPositionById(fingerNum + 1, position);  // F1 → ID 2
    }
  } else if (name == "W") {
    setPositionById(1, position);  // WRIST_ID is 1
  }
}
void NMLHandExo::setPositionByAlias(const String& alias, int position) {
  String name = alias;
  name.toUpperCase();
  if (name == "THUMB") setPositionById(2, position);
  else if (name == "INDEX") setPositionById(3, position);
  else if (name == "MIDDLE") setPositionById(4, position);
  else if (name == "RING") setPositionById(5, position);
  else if (name == "PINKY") setPositionById(6, position);
  else if (name == "WRIST") setPositionById(1, position);
}
int NMLHandExo::getPositionByAlias(const String& alias) {
  String name = alias;
  name.toUpperCase();

  if (name == "THUMB") return getPositionById(2);
  if (name == "INDEX") return getPositionById(3);
  if (name == "MIDDLE") return getPositionById(4);
  if (name == "RING") return getPositionById(5);
  if (name == "PINKY") return getPositionById(6);
  if (name == "WRIST") return getPositionById(1);

  return -1;  // Invalid name
}
int NMLHandExo::getIndexById(uint8_t id) {
  for (int i = 0; i < motorCount_; i++) {
    if (ids_[i] == id) return i;
  }
  return -1;
}
void NMLHandExo::getDynamixelInfo(uint8_t id) {
  // Get DYNAMIXEL information, should print to the terminal
  dxl_.ping(id);
}
void NMLHandExo::setMotorLED(uint8_t id, bool state) {
  // Sets specified motor LED to the specified state
  if (state) {
    dxl_.ledOn(id);
  } else {
    dxl_.ledOff(id);
  }
}
void NMLHandExo::setAllMotorLED(bool state) {
  // Sets the state of all motor LEDs to the specified state
  for (int i = 0; i < motorCount_; i++) {
    uint8_t id = ids_[i];
    setMotorLED(id, state);
  }
}
void NMLHandExo::printAllPositions() {
  for (int i = 0; i < motorCount_; i++) {
    uint8_t id = ids_[i];
    int pos = getPositionById(id);
    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" position: ");
    Serial.println(pos);
  }
}
void NMLHandExo::rebootMotor(uint8_t id) {
  dxl_.reboot(id);
}
void NMLHandExo::setAngleById(uint8_t id, float angleDeg) {
  int pos = round((angleDeg / 300.0f) * 1023.0f);
  setPositionById(id, pos);
}
void NMLHandExo::setAngleByAlias(const String& alias, float angleDeg) {
  String name = alias;
  name.toUpperCase();
  if (name == "WRIST") setAngleById(1, angleDeg);
  else if (name == "THUMB") setAngleById(2, angleDeg);
  else if (name == "INDEX") setAngleById(3, angleDeg);
  else if (name == "MIDDLE") setAngleById(4, angleDeg);
  else if (name == "RING") setAngleById(5, angleDeg);
  else if (name == "PINKY") setAngleById(6, angleDeg);
}
void NMLHandExo::calibrateZero(uint8_t id) {
  int index = getIndexById(id);
  if (index != -1) {
    zeroOffsets_[index] = getPositionById(id);
  }
}
void NMLHandExo::resetAllZeros() {
  for (int i = 0; i < motorCount_; i++) {
    zeroOffsets_[i] = getPositionById(ids_[i]);
  }
}
float NMLHandExo::getRelativeAngle(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return -1;

  int pos = getPositionById(id);
  int delta = pos - zeroOffsets_[index];
  float degrees = (delta / 1023.0f) * 300.0f;
  return degrees;
}

