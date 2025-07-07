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

#include "config.h"
#include "utils.h"
#include "nml_hand_exo.h"
#include "gesture_controller.h"

#define DEBUG_SERIAL Serial
#define BLE_SERIAL Serial2

uint8_t ids[] = { WRIST_ID, RING_ID, PINKY_ID, INDEX_ID, MIDDLE_ID, THUMB_ID };
uint8_t n_motors = sizeof(ids)/sizeof(ids[0]);

// Create the exo device with the motor parameters and id values
NMLHandExo exo(ids, n_motors, jointLimits, homeStates);
GestureController gc(exo);  // pass exo reference

void setup() {

  // LEDs for command/connection feedback
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);  // initially off

  // Serial connections
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);    // Setting a default baud rate of 57600
  while (!DEBUG_SERIAL);
  BLE_SERIAL.begin(BLE_BAUD_RATE);     // (Optional) Establish port with TX/RX pins for incomming serial data/commands

  // Setup exo
  exo.initializeSerial(DYNAMIXEL_BAUD_RATE);
  exo.initializeMotors();       // Initialize motors and set them to "current position" mode
  // exo.resetAllZeros();       // (Optional) Defines the current position of the motors as the home position
  exo.setModeSwitchButton(MODESWITCH_PIN);

  // Setup gesture controller
  gc.setCycleGestureButton(CYCLE_GESTURE_PIN);
  gc.setGestureStateSwitchButton(GESTURE_STATE_PIN);

  // Flash LEDs to let user know system ready to go
  flashPin(STATUS_LED_PIN, 100, 4);
  debugPrint(F("Exo device ready to receive commands"));
}

void loop() {

  // Handle data from the debug connection
  if (DEBUG_SERIAL.available() > 0) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    String input = DEBUG_SERIAL.readStringUntil('\n');
    input.trim();
    debugPrint("Received: " + input);
    parseMessage(exo, gc, input);
  }

  // Handle data from the BLE/command connection
  if (BLE_SERIAL.available() > 0) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    String input = BLE_SERIAL.readStringUntil('\n');
    input.trim();
    debugPrint("Received: " + input);
    parseMessage(exo, gc, input);
  }

  // Update the exo state, including checking for button pressed, mode switching, and internal routines
  exo.update();

  // Check for any updates needed with the gesture controller
  gc.update();
}
