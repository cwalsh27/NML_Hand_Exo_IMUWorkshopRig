#ifndef NML_HAND_EXO_H
#define NML_HAND_EXO_H

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

// Declare externally-defined debug printer
extern bool VERBOSE;
//extern HardwareSerial& DEBUG_SERIAL;
extern Stream& DEBUG_SERIAL;
extern Stream* debugStream;

// Helper function
void debugPrint(const String& msg);


class NMLHandExo {
  public:
    NMLHandExo(const uint8_t* ids, int motorCount, const int jointLimits[][2]);
    void initializeMotors();
    void initializeSerial(int baud);
    int angleToTicks(float angle_deg, int index);
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
    void getMotorInfo(uint8_t id);  // will just ping for now
    void setAngleById(uint8_t id, float angleDeg);
    int getIndexById(uint8_t id);
    void setAngleByAlias(const String& alias, float angleDeg);
    void calibrateZero(uint8_t id);  // Set current position as zero
    void resetAllZeros();
    void enableTorque(uint8_t id, bool enable);
    int16_t getCurrent(uint8_t id);
    float getTorque(uint8_t id);
    void setBaudRate(uint8_t id, uint32_t baudrate);
    void setVelocityLimit(uint8_t id, uint32_t vel);
    uint32_t getVelocityLimit(uint8_t id);
    void setAccelerationLimit(uint8_t id, uint32_t acc);
    uint32_t getAccelerationLimit(uint8_t id);
    uint32_t getBaudRate(uint8_t id);
    float getRelativeAngle(uint8_t id);

    static constexpr const char* VERSION = "1.2.0";

  private:
    Dynamixel2Arduino dxl_;
    const uint8_t* ids_;
    int motorCount_;
    const int (*jointLimits_)[2];  // Pointer to 2D array of joint limits
    int zeroOffsets_[6] = {0};     // Track absolute zero positions

    // Motor tick limits must be defined after motor installation, before usage to find bounds
    // const int (*jointLimits_)[2];  // Pointer to 2D array of joint limits
    
};

#endif