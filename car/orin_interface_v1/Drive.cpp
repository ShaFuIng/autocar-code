#include "Drive.h"
#include "Safety.h"
#include "ManualDrive.h"
#include "Telemetry.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
MotorState motorA = {MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_INVERT, 0,0,0,0,0};
MotorState motorB = {MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_INVERT, 0,0,0,0,0};
uint16_t steeringUs = 0;
bool steeringEnabled = false;
ServoProfile servoProfile = {SERVO_MIN_US, SERVO_CENTER_US, SERVO_MAX_US, 0};
bool servoProfileSynced = false, servoReturning = false;
static uint8_t prescale;
static uint32_t returnGeneration, returnAt;
static uint16_t returnTarget;
static uint32_t pulseCount(uint16_t us) {
  return (uint32_t)((float)us * PCA_OSC_HZ /
    (1000000.0f * (prescale + 1)) + 0.5f);
}
static void returnTick();
bool pcaHealthy = false;
uint32_t i2cErrors = 0;
static Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);

static uint32_t rampAt, probeAt;

static bool output(uint8_t ch, uint16_t on, uint16_t off) {
  if (!pcaHealthy) return false;
  if (pwm.setPWM(ch, on, off) == 0) return true;
  ++i2cErrors;
  pcaHealthy = false;
  // Remove drive enable independently of the failed I2C bus.
  analogWrite(MOTOR_A_EN, 0);
  analogWrite(MOTOR_B_EN, 0);
  return false;
}
static bool direction(MotorState &m, int8_t dir) {
  if (m.invert) dir = -dir;
  // Disable both first so a reversal never briefly selects both high.
  if (!output(m.a, 0, 4096) || !output(m.b, 0, 4096)) return false;
  if (dir > 0) return output(m.a, 4096, 0);
  if (dir < 0) return output(m.b, 4096, 0);
  return true;
}
void drivePinsBegin() {
  pinMode(MOTOR_A_EN, OUTPUT); pinMode(MOTOR_B_EN, OUTPUT);
  analogWrite(MOTOR_A_EN, 0); analogWrite(MOTOR_B_EN, 0);
}
bool driveBegin() {
  Wire.begin();
  Wire.setWireTimeout(3000, true); // Bound transactions on a missing/stuck PCA bus.
  if (!pwm.begin()) { ++i2cErrors; return false; }
  pcaHealthy = true;
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(50);
  prescale = pwm.readPrescale();
  if (prescale == 0 || prescale == 255) { pcaHealthy = false; ++i2cErrors; return false; }
  driveStop();
  servoOff();
  rampAt = probeAt = millis();
  return pcaHealthy;
}
void driveStop() {
  analogWrite(MOTOR_A_EN, 0); analogWrite(MOTOR_B_EN, 0);
  motorA.speed = motorA.target = motorA.pending = 0;
  motorB.speed = motorB.target = motorB.pending = 0;
  motorA.dir = motorA.wantedDir = motorB.dir = motorB.wantedDir = 0;
  direction(motorA, 0); direction(motorB, 0);
  // Hold steering on stop: no sudden automatic steering movement.
}
static void command(MotorState &m, int8_t dir, int speed) {
  m.wantedDir = dir; m.pending = speed;
  m.target = (m.dir == dir) ? speed : 0;
}
bool driveTestSet(int right,int left) {
  if (!pcaHealthy || (safety.state!=RunState::MOTOR_TEST && safety.state!=RunState::MANUAL)) return false;
  command(motorA,right>0?1:right<0?-1:0,abs(right)*255/100);
  command(motorB,left>0?1:left<0?-1:0,abs(left)*255/100);
  return true;
}
bool driveManualSet(int right,int left,uint16_t pulse){
  if(safety.state!=RunState::MANUAL || !servoProfileSynced || pulse<servoProfile.minimum || pulse>servoProfile.maximum)return false;
  if(!right&&!left)driveStop(); // Key release removes PWM immediately, rather than coasting down a ramp.
  else if(!driveTestSet(right,left))return false;
  if(!steeringEnabled){
    if(pulse!=servoProfile.center || !output(SERVO_CH,0,pulseCount(pulse)))return false;
    steeringUs=pulse;steeringEnabled=true;
  }
  servoReturning=false;manualSteerTarget=pulse;return true;
}
bool driveSet(int throttle, int steer) {
  if (!pcaHealthy || SERVO_CALIBRATION_MODE || !servoProfileSynced) return false;
  steer = constrain(steer, -100, 100);
  throttle = constrain(throttle, -(int)MAX_THROTTLE_PERCENT, (int)MAX_THROTTLE_PERCENT);
  int8_t dir = throttle > 0 ? 1 : throttle < 0 ? -1 : 0;
  int speed = abs(throttle) * 255 / 100;
  command(motorA, dir, speed); command(motorB, dir, speed);
  uint16_t wantedUs = steer >= 0
    ? servoProfile.center + (long)steer * (servoProfile.maximum - servoProfile.center) / 100
    : servoProfile.center + (long)steer * (servoProfile.center - servoProfile.minimum) / 100;
  if (wantedUs == steeringUs) return true;
  uint16_t count = (uint16_t)((float)wantedUs * PCA_OSC_HZ /
                           (1000000.0f * (prescale + 1)) + 0.5f);
  if (!output(SERVO_CH, 0, count)) return false;
  steeringUs = wantedUs;
  steeringEnabled = true;
  return true;
}
static void ramp(MotorState &m) {
  if (m.speed != m.target) {
    m.speed += m.speed < m.target ? RAMP_STEP : -RAMP_STEP;
    if (abs(m.speed - m.target) < RAMP_STEP) m.speed = m.target;
    m.speed = constrain(m.speed, 0, 255);
    analogWrite(m.en, m.speed);
  } else if (m.speed == 0 && m.dir != m.wantedDir) {
    if (direction(m, m.wantedDir)) {
      m.dir = m.wantedDir;
      m.target = m.dir ? m.pending : 0;
    }
  }
}
void driveTick() {
  uint32_t now = millis();
  if (pcaHealthy && now - probeAt >= 1000) {
    probeAt = now;
    Wire.beginTransmission(PCA_ADDR);
    if (Wire.endTransmission() != 0) { ++i2cErrors; pcaHealthy = false; }
  }
  if (!pcaHealthy) {
    if (safety.state != RunState::FAULT) safetyStop("PCA_IO_FAILED", true);
    return;
  }
  if (now - rampAt >= RAMP_MS) {
    rampAt = now;
    if (safety.state == RunState::ARMED || safety.state == RunState::MOTOR_TEST || safety.state == RunState::MANUAL) { ramp(motorA); if (pcaHealthy) ramp(motorB); }
  }
  if(safety.state==RunState::MANUAL && steeringEnabled && now-returnAt>=120){
    returnAt=now;
    int delta=constrain((int)manualSteerTarget-(int)steeringUs,-10,10);
    if(delta && output(SERVO_CH,0,pulseCount((uint16_t)((int)steeringUs+delta))))steeringUs=(uint16_t)((int)steeringUs+delta);
  }
  if (SERVO_CALIBRATION_MODE) returnTick();
  if (!pcaHealthy) safetyStop("PCA_WRITE_FAILED", true);
}

void servoOff() {
  output(SERVO_CH, 0, 4096);
  steeringEnabled = false;
  servoReturning = false;
}
const char *servoCalibrationPulse(uint32_t generation, uint16_t pulseUs) {
  static uint32_t lastSetMs = 0;
  if (!SERVO_CALIBRATION_MODE) return "CALIBRATION_ONLY";
  if (digitalRead(PIN_ESTOP) == LOW) return "ESTOP_ACTIVE";
  if (!pcaHealthy) return "PCA_FAULT";
  if (generation != safety.generation) return "STALE_GENERATION";
  if (safety.state != RunState::READY) return "PHYSICAL_START_REQUIRED";
  if (!servoProfileSynced) return "SERVO_PROFILE_REQUIRED";
  if (pulseUs < servoProfile.minimum || pulseUs > servoProfile.maximum) return "SERVO_LIMIT";
  if (pulseUs == 0) return "INVALID_PULSE";
  uint32_t count = (uint32_t)((float)pulseUs * PCA_OSC_HZ /
    (1000000.0f * (prescale + 1)) + 0.5f);
  if (count == 0 || count >= 4096) return "PWM_COUNTER_RANGE";
  if (!steeringEnabled && pulseUs != servoProfile.center) return "INITIALIZE_CENTER_FIRST";
  if (steeringEnabled && abs((int)pulseUs - (int)steeringUs) > 100) return "STEP_TOO_LARGE";
  if (steeringEnabled && millis() - lastSetMs < 100) return "RATE_LIMIT";
  driveStop();
  if (!output(SERVO_CH, 0, count)) { safetyStop("PCA_WRITE_FAILED", true); return "PCA_FAULT"; }
  lastSetMs = millis();
  steeringUs = pulseUs;
  steeringEnabled = true;
  servoReturning = false;
  safety.lastMoveMs = millis();
  eventLog("SERVO_CAL", "PULSE_SET_NOT_MEASURED_ANGLE");
  telemetryStatus();
  return nullptr;
}
void servoCalibrationCommand(char command) {
  if (!SERVO_CALIBRATION_MODE) return;
  if (command == 'o') { safetyStop("SERVO_CAL_OFF"); return; }
  if (command != 'i' && !steeringEnabled) { eventLog("SERVO_CAL_REJECT", "INITIALIZE_FIRST"); return; }
  if (command == 'i' && steeringEnabled) { eventLog("SERVO_CAL_REJECT", "ALREADY_ENABLED"); return; }
  long wanted = command == 'i' ? servoProfile.center :
    (long)steeringUs + (command == '+' ? SERVO_TEST_STEP_US : -(int)SERVO_TEST_STEP_US);
  const char *error = wanted <= 0 || wanted > 65535
    ? "INVALID_PULSE" : servoCalibrationPulse(safety.generation, (uint16_t)wanted);
  if (error) eventLog("SERVO_CAL_REJECT", error);
}

const char *servoConfigure(uint32_t generation, uint32_t revision, uint16_t minimum, uint16_t center, uint16_t maximum) {
  if (!SERVO_CALIBRATION_MODE) return "CALIBRATION_ONLY";
  if (generation != safety.generation) return "STALE_GENERATION";
  if (safety.state == RunState::ARMED || safety.state == RunState::MOTOR_TEST || safety.state == RunState::MANUAL || safety.state == RunState::FAULT) return "STOP_FIRST";
  if (!revision || !minimum || !pulseCount(minimum) || minimum >= center || center >= maximum || pulseCount(maximum) >= 4096) return "BAD_PROFILE";
  if (steeringEnabled && (steeringUs < minimum || steeringUs > maximum)) return "CURRENT_OUTSIDE_LIMITS";
  servoProfile = {minimum, center, maximum, revision};
  servoProfileSynced = true;
  servoReturning = false;
  eventLog("SERVO_PROFILE", "CONFIGURED_NO_MOVEMENT");
  telemetryStatus();
  return nullptr;
}
const char *servoReturnToCenter(uint32_t generation) {
  if (!SERVO_CALIBRATION_MODE || !servoProfileSynced) return "SERVO_PROFILE_REQUIRED";
  if (generation != safety.generation) return "STALE_GENERATION";
  if (digitalRead(PIN_ESTOP) == LOW) return "ESTOP_ACTIVE";
  if (!pcaHealthy) return "PCA_FAULT";
  if (safety.state != RunState::READY) return "PHYSICAL_START_REQUIRED";
  if (!steeringEnabled) return "INITIALIZE_CENTER_FIRST";
  if (steeringUs < servoProfile.minimum || steeringUs > servoProfile.maximum) return "SERVO_LIMIT";
  returnGeneration = generation;
  returnTarget = servoProfile.center;
  returnAt = millis();
  servoReturning = steeringUs != returnTarget;
  eventLog("SERVO_CENTER", "RETURN_REQUESTED");
  telemetryStatus();
  return nullptr;
}
static void returnTick() {
  if (!servoReturning) return;
  if (safety.state != RunState::READY || safety.generation != returnGeneration || !steeringEnabled || digitalRead(PIN_ESTOP) == LOW) {
    servoReturning = false; return;
  }
  if (millis() - returnAt < 120) return;
  returnAt = millis();
  int difference = (int)returnTarget - (int)steeringUs;
  int step = constrain(difference, -10, 10);
  const char *error = servoCalibrationPulse(returnGeneration, (uint16_t)((int)steeringUs + step));
  if (error) { servoReturning = false; eventLog("SERVO_CENTER_ERROR",error); return; }
  servoReturning = steeringUs != returnTarget;
  if (!servoReturning) { eventLog("SERVO_CENTER", "COMMAND_REACHED_CENTER"); telemetryStatus(); }
}
