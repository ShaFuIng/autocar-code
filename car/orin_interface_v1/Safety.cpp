#include "Safety.h"
#include "Drive.h"
#include "MotorTest.h"
#include "ManualDrive.h"
#include "BleRemote.h"
#include "Telemetry.h"
SafetyState safety = {ControlMode::ORIN, RunState::LOCKED, true, 0, 0, "BOOT_LOCK"};
static bool startRaw = false, startStable = false;
static bool releaseSeen = false;
static uint32_t startChanged;
const char *stateName() {
  switch(safety.state) {
    case RunState::LOCKED: return "LOCKED";
    case RunState::READY: return "READY";
    case RunState::ARMED: return "ARMED";
    case RunState::MOTOR_TEST: return "MOTOR_TEST";
    case RunState::MANUAL: return "MANUAL";
    default: return "FAULT";
  }
}
const char *modeName() { return safety.mode == ControlMode::ORIN ? "ORIN" : "BLE"; }
void safetyBegin() {
  pinMode(PIN_START, INPUT_PULLUP); pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_START_LED, OUTPUT); digitalWrite(PIN_START_LED, LOW);
  startRaw = startStable = digitalRead(PIN_START) == LOW;
  releaseSeen = !startRaw;
  startChanged = millis();
}
void safetyStop(const char *reason, bool fault) {
  motorTestCancel();
  manualCancel();
  driveStop();
  if (SERVO_CALIBRATION_MODE) servoOff();
  safety.state = (fault || !pcaHealthy) ? RunState::FAULT : RunState::LOCKED;
  safety.neutralRequired = true;
  safety.reason = reason;
  ++safety.generation; // Invalidate commands buffered before the stop.
  releaseSeen = false; // Require release and a new physical start press.
  bleDiscardPending();
  digitalWrite(PIN_START_LED, LOW);
  eventLog("STOP", reason);
}
void safetyTick() {
  uint32_t now = millis();
  bool estop = digitalRead(PIN_ESTOP) == LOW;
  if (estop) {
    releaseSeen = false;
    if (strcmp(safety.reason, "PHYSICAL_ESTOP") != 0 && safety.state != RunState::FAULT)
      safetyStop("PHYSICAL_ESTOP");
  }
  bool raw = digitalRead(PIN_START) == LOW;
  if (raw != startRaw) { startRaw = raw; startChanged = now; }
  if (now - startChanged >= START_DEBOUNCE_MS) {
    if (!raw && !estop) releaseSeen = true;
    if (raw != startStable) {
      startStable = raw;
      if (raw && releaseSeen && !estop && pcaHealthy && safety.state == RunState::LOCKED) {
        releaseSeen = false;
        ++safety.generation;
        safety.state = (!SERVO_CALIBRATION_MODE && safety.mode == ControlMode::BLE) ? RunState::ARMED : RunState::READY;
        safety.reason = "PHYSICAL_START";
        safety.neutralRequired = true;
        safety.lastMoveMs = now;
        bleDiscardPending(); // Phone packets queued before authorization cannot move.
        eventLog("START", safety.reason);
      }
    }
  }
  if (safety.state == RunState::ARMED && now - safety.lastMoveMs > COMMAND_TIMEOUT_MS)
    safetyStop("COMMAND_TIMEOUT");
  if (!SERVO_CALIBRATION_MODE && safety.state == RunState::READY && now - safety.lastMoveMs > 30000)
    safetyStop("START_AUTH_EXPIRED");
  motorTestTick();
  manualTick();
  digitalWrite(PIN_START_LED, (safety.state == RunState::ARMED || safety.state == RunState::MOTOR_TEST || safety.state == RunState::MANUAL) ? HIGH : LOW);
}
const char *safetyArm(uint32_t generation) {
  if (SERVO_CALIBRATION_MODE) return "SERVO_CALIBRATION_ONLY";
  if (!servoProfileSynced) return "SERVO_PROFILE_REQUIRED";
  if (digitalRead(PIN_ESTOP) == LOW) return "ESTOP_ACTIVE";
  if (!pcaHealthy) return "PCA_FAULT";
  if (safety.mode != ControlMode::ORIN) return "WRONG_MODE";
  if (generation != safety.generation) return "STALE_GENERATION";
  if (safety.state != RunState::READY) return "PHYSICAL_START_REQUIRED";
  safety.state = RunState::ARMED;
  safety.neutralRequired = true;
  safety.lastMoveMs = millis();
  safety.reason = "ORIN_ARM";
  eventLog("ARM", safety.reason);
  return nullptr;
}
const char *safetyMove(ControlMode source, uint32_t generation, int throttle, int steer) {
  safetyTick();
  if (SERVO_CALIBRATION_MODE) return "SERVO_CALIBRATION_ONLY";
  if (!servoProfileSynced) return "SERVO_PROFILE_REQUIRED";
  if (digitalRead(PIN_ESTOP) == LOW) return "ESTOP_ACTIVE";
  if (!pcaHealthy) return "PCA_FAULT";
  if (source != safety.mode) return "WRONG_MODE";
  if (generation != safety.generation) return "STALE_GENERATION";
  if (safety.state != RunState::ARMED) return "NOT_ARMED";
  if (throttle < -100 || throttle > 100 || steer < -100 || steer > 100) return "RANGE";
  if (safety.neutralRequired && (throttle != 0 || steer != 0)) return "NEUTRAL_REQUIRED";
  if (!driveSet(throttle, steer)) { safetyStop("PCA_WRITE_FAILED", true); return "PCA_FAULT"; }
  safety.neutralRequired = false;
  safety.lastMoveMs = millis();
  return nullptr;
}
const char *safetyMode(ControlMode mode) {
  if (SERVO_CALIBRATION_MODE && mode == ControlMode::BLE) return "SERVO_CALIBRATION_ONLY";
  if (safety.state == RunState::ARMED || safety.state == RunState::MOTOR_TEST || safety.state == RunState::MANUAL || safety.state == RunState::READY) return "STOP_FIRST";
  if (safety.state == RunState::FAULT) return "PCA_FAULT";
  safetyStop("MODE_CHANGE");
  if (!bleSetEnabled(mode == ControlMode::BLE)) return "BLE_INIT_FAILED";
  safety.mode = mode;
  eventLog("MODE", modeName());
  return nullptr;
}
