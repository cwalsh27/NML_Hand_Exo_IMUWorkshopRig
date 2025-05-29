#ifndef NML_HAND_EXO_H
#define NML_HAND_EXO_H

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

// Declare externally-defined debug printer
extern bool VERBOSE;

extern Stream& DEBUG_SERIAL;
extern Stream* debugStream;

// Helper function for printing and debugging
void debugPrint(const String& msg);


class NMLHandExo {
  public:    
    NMLHandExo(const uint8_t* ids, int numMotors, const int jointLimits[][2]);

    // Utility functions
    void initializeSerial(int baud);
    void initializeMotors();
    int getMotorID(const String& token);
    int getIndexById(uint8_t id);
    int getMotorIDByName(const String& name);
    int angleToTicks(float angle_deg, int index);
    void calibrateZero(uint8_t id); 
    void resetAllZeros();

    //  Position comands 
    // void printAllPositions(); TO-DO: COmand to get teh current joint state of exo device, maybe "getJointState()"
    float getRelativeAngle(uint8_t id);
    float getAbsoluteAngle(uint8_t id);
    float getZeroOffset(uint8_t id);
    void setAngleById(uint8_t id, float angleDeg);
    void setAngleByAlias(const String& alias, float angleDeg);

    // Torque commands
    void enableTorque(uint8_t id, bool enable);
    int16_t getCurrent(uint8_t id);
    float getTorque(uint8_t id);

    // Velocity commands
    void setVelocityLimit(uint8_t id, uint32_t vel);
    uint32_t getVelocityLimit(uint8_t id);

    // Acceleration commands
    void setAccelerationLimit(uint8_t id, uint32_t acc);
    uint32_t getAccelerationLimit(uint8_t id);

    // Motor-specific commands
    void rebootMotor(uint8_t id);
    void getMotorInfo(uint8_t id);  // will just ping for now
    void setBaudRate(uint8_t id, uint32_t baudrate);
    uint32_t getBaudRate(uint8_t id);
    void setMotorLED(uint8_t id, bool state);
    void setAllMotorLED(bool state);

    // User-accessible parameters
    static constexpr const char* VERSION = "1.2.1";

  private:
  // Internal parameters specific to the class
    Dynamixel2Arduino dxl_;         // Handle to Dynamicel object
    const uint8_t* ids_;            // List of motor IDs passed by the user          
    int numMotors_;                // Number of motors being used, should be detected by the length of motor ids passed
    const int (*jointLimits_)[2];   // Pointer to 2D array of joint limits
    int zeroOffsets_[6] = {0};      // Track absolute zero positions

    // Motor tick limits must be defined after motor installation, before usage to find bounds
    
};

#endif