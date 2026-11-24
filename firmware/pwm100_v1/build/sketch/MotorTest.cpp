#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm100_v1\\Orin_Interface_v1\\MotorTest.cpp"
#include "MotorTest.h"
#include "Drive.h"
#include "Safety.h"
#include "Telemetry.h"
bool motorTestStarted = false;
uint32_t motorTestStartMs = 0, motorTestDurationMs = 0;
int motorTestRight = 0, motorTestLeft = 0;
void motorTestCancel() { motorTestStarted=false; motorTestRight=motorTestLeft=0; }
static void finish() {
  driveStop();
  motorTestCancel();
  safety.state=RunState::READY;
  safety.reason="MOTOR_TEST_COMPLETE";
  safety.neutralRequired=true;
  safety.lastMoveMs=millis();
  ++safety.generation; // Old test packets cannot restart the wheels.
  eventLog("MOTOR_TEST","COMPLETE");
}
const char *motorTestArm(uint32_t generation,uint32_t durationMs) {
  safetyTick();
  if (!SERVO_CALIBRATION_MODE) return "BENCH_ONLY";
  if (estopActive()) return "ESTOP_ACTIVE";
  if (!pcaHealthy) return "PCA_FAULT";
  if (!servoProfileSynced) return "SERVO_PROFILE_REQUIRED";
  if (generation!=safety.generation) return "STALE_GENERATION";
  if (safety.mode!=ControlMode::ORIN || (safety.state!=RunState::READY && !(WEB_TIMED_TEST_START && safety.state==RunState::LOCKED))) return "PHYSICAL_START_REQUIRED";
  if (servoReturning) return "STEERING_BUSY";
  if (durationMs<500 || durationMs>30000) return "BAD_DURATION";
  driveStop();motorTestCancel();
  motorTestDurationMs=durationMs;
  safety.state=RunState::MOTOR_TEST;
  safety.neutralRequired=true;
  safety.lastMoveMs=millis();
  safety.reason="MOTOR_TEST_ARMED";
  eventLog("MOTOR_TEST","ARMED_NO_MOTION");
  return nullptr;
}
const char *motorTestMove(uint32_t generation,int right,int left) {
  safetyTick();
  if (generation!=safety.generation) return "STALE_GENERATION";
  if (safety.state!=RunState::MOTOR_TEST) return "TEST_NOT_ARMED";
  if (estopActive()) return "ESTOP_ACTIVE";
  if (!pcaHealthy) return "PCA_FAULT";
  if (right < -(int)MAX_THROTTLE_PERCENT || right > MAX_THROTTLE_PERCENT || left < -(int)MAX_THROTTLE_PERCENT || left > MAX_THROTTLE_PERCENT) return "PWM_LIMIT_50";
  if (safety.neutralRequired) {
    if (right || left) return "NEUTRAL_REQUIRED";
    safety.neutralRequired=false;
  } else if (!motorTestStarted) {motorTestStarted=true;motorTestStartMs=millis();}
  if (!driveTestSet(right,left)) {safetyStop("PCA_WRITE_FAILED",true);return "PCA_FAULT";}
  motorTestRight=right;motorTestLeft=left;
  safety.lastMoveMs=millis();
  return nullptr;
}
const char *motorTestEnd(uint32_t generation) {
  safetyTick();
  if (generation!=safety.generation) return "STALE_GENERATION";
  if (safety.state!=RunState::MOTOR_TEST) return "TEST_NOT_ARMED";
  finish();return nullptr;
}
void motorTestTick() {
  if (safety.state!=RunState::MOTOR_TEST) return;
  uint32_t now=millis();
  if (now-safety.lastMoveMs>COMMAND_TIMEOUT_MS) {safetyStop("COMMAND_TIMEOUT");return;}
  if (motorTestStarted && now-motorTestStartMs>=motorTestDurationMs) finish();
}
