#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm100_v1\\Orin_Interface_v1\\Safety.h"
#pragma once
#include "Config.h"
enum class ControlMode : uint8_t { ORIN, BLE };
enum class RunState : uint8_t { LOCKED, READY, ARMED, MOTOR_TEST, MANUAL, FAULT };
struct SafetyState {
  ControlMode mode;
  RunState state;
  bool neutralRequired;
  uint32_t lastMoveMs;
  uint32_t generation;
  const char *reason;
};
extern SafetyState safety;
const char *stateName();
const char *modeName();
void safetyBegin();
void safetyTick();
bool estopActive();   // estop-debounce-r1: LOW must persist ESTOP_CONFIRM_US before it counts
void safetyStop(const char *reason, bool fault = false);
const char *safetyArm(uint32_t generation);
const char *safetyCalibrationEnable(uint32_t generation);
const char *safetyMove(ControlMode source, uint32_t generation, int throttle, int steer);
const char *safetyMode(ControlMode mode);
