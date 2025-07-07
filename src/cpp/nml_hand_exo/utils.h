/**
 * @file utils.h
 * @brief Header file for suporting function definitions
 *
 */
#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "nml_hand_exo.h"
#include "gesture_controller.h"

void debugPrint(const String& msg);

void flashPin(int pin, int durationMs, int repetitions);

String getArg(const String line, const int index, char delimiter = ':');

int getArgMotorID(NMLHandExo& exo, const String& line, const int index);

void parseMessage(NMLHandExo& exo, GestureController& gc, const String token);

#endif