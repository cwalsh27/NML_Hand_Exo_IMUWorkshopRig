#include "gesture_library.h"

GestureMap gestureLibrary[N_GESTURES] = {
    // This is where you can create your own gestures and their states
    // Each gesture is a map of states, where each state has a vector of joint angles
    {"grasp", {{"open", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
               {"close", {180.0, 180.0, 180.0, 180.0, 180.0, 180.0}}
              }, 2},
    {"point", {{"a", {0.0, 0.0, 0.0, 0.0, 0.0, 90.0}}}, 1},

    // You can add multiple states to a gesture, especially if it is being used for manipulation or grasping
    {"pinch", {{"open", {10.0, 20.0, 30.0, 40.0, 50.0, 60.0}}, 
               {"close", {15.0, 25.0, 35.0, 45.0, 55.0, 65.0}}
              }, 2}
};
int gestureLibrarySize = 3;
