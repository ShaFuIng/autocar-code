#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm100_v1\\Orin_Interface_v1\\Drive.h"
#pragma once
#include "Config.h"
struct MotorState {
  uint8_t en, a, b;
  bool invert;
  int8_t dir, wantedDir;
  int speed, target, pending;
};
extern MotorState motorA, motorB;
extern uint16_t steeringUs;
extern bool pcaHealthy;
extern uint32_t i2cErrors;
void drivePinsBegin();
bool driveBegin();
void driveStop();
bool driveSet(int throttle, int steer);
void driveTick();

extern bool steeringEnabled;
extern bool steeringOutputKnown;
void servoOff();
void servoCalibrationCommand(char command);

const char *servoCalibrationPulse(uint32_t generation, uint16_t pulseUs);

struct ServoProfile { uint16_t minimum, center, maximum; uint32_t revision; };
extern ServoProfile servoProfile;
extern bool servoProfileSynced, servoReturning;
const char *servoConfigure(uint32_t generation, uint32_t revision, uint16_t minimum, uint16_t center, uint16_t maximum);
const char *servoReturnToCenter(uint32_t generation);

bool driveTestSet(int right, int left);
bool driveManualSet(int right,int left,uint16_t pulse);

size_t formatDriveRecovery(char *out,size_t size);
