#ifndef NML_HAND_EXO_H
#define NML_HAND_EXO_H

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

class NMLHandExo {
  public:
    NMLHandExo(const uint8_t* ids, int motorCount, const int jointLimits[][2]);
    void initializeMotors();
    void initializeSerial(int baud);
    void setPositionById(uint8_t id, int position);
    void setPositionByName(const String& name, int position);
    void setAllMotorLED(bool state);
    void setMotorLED(uint8_t id, bool state);
    void getDynamixelInfo(uint8_t id);
    int getPositionById(uint8_t id);
    void printAllPositions();
    void setPositionByAlias(const String& alias, int position);
    int getPositionByAlias(const String& alias);
    void rebootMotor(uint8_t id);
    void setAngleById(uint8_t id, float angleDeg);
    void setAngleByAlias(const String& alias, float angleDeg);
    void calibrateZero(uint8_t id);  // Set current position as zero
    void resetAllZeros();
    float getRelativeAngle(uint8_t id);

    static constexpr const char* VERSION = "1.2.0";

  private:
    Dynamixel2Arduino dxl_;
    const uint8_t* ids_;
    int motorCount_;
    //const int jointLimits_[6][2] = {
    //   {0, 500}, 
    //   {500, 700}, // DOWN - UP
    //   {650, 820}, // DOWN - UP
    //   {0,   150}, // UP - DOWN
    //   {200, 600}, // UP - DOWN
    //   {120, 880}
    // };
    const int (*jointLimits_)[2];  // Pointer to 2D array of joint limits
    int getIndexById(uint8_t id);
    int zeroOffsets_[6] = {0};  // per-motor calibration offsets
};

#endif