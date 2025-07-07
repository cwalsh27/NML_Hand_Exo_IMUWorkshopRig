/**
 * @file gesture_library.h
 * @brief Header file for the gesture library used in the NML Hand Exoskeleton project.
 *
 */
#ifndef GESTURE_LIBRARY_H
#define GESTURE_LIBRARY_H
#include "config.h"

struct GestureState {
    const char* name;              // e.g., "a", "open", "close"
    float jointAngles[NUM_JOINTS]; // joint angles for each state
};

struct GestureMap {
    const char* name;              // e.g., "pinch", "open"
    GestureState states[MAX_STATES_PER_GESTURE];
    int numStates;
    //const char* defaultState;              // e.g., "pinch", "open"
};

extern GestureMap gestureLibrary[N_GESTURES];
extern int gestureLibrarySize;

inline int findGestureIndex(const String& gestureName) {
    for (int i = 0; i < gestureLibrarySize; i++) {
        if (gestureName.equalsIgnoreCase(gestureLibrary[i].name)) {
            return i;
        }
    }
    return -1;
}

inline int findStateIndex(const GestureMap& gesture, const String& stateName) {
    for (int i = 0; i < gesture.numStates; i++) {
        if (stateName.equalsIgnoreCase(gesture.states[i].name)) {
            return i;
        }
    }
    return -1;
}

#endif
