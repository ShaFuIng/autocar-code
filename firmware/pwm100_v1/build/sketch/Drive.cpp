#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm100_v1\\Orin_Interface_v1\\Drive.cpp"
#include "Drive.h"
#include "FaultDiagnostics.h"
#include "Safety.h"
#include "ManualDrive.h"
#include "Telemetry.h"
#include <Wire.h>
#include <stdio.h>
#include <Adafruit_PWMServoDriver.h>
MotorState motorA = {MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_INVERT, 0,0,0,0,0};
MotorState motorB = {MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_INVERT, 0,0,0,0,0};
uint16_t steeringUs = 0;
bool steeringEnabled = false;
bool steeringOutputKnown = false;
ServoProfile servoProfile = {SERVO_MIN_US, SERVO_CENTER_US, SERVO_MAX_US, 0};
bool servoProfileSynced = false, servoReturning = false;
static uint8_t prescale;
static bool bootProfilePending = true;
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
static uint8_t timedEnd(){transactionBegin();uint8_t c=Wire.endTransmission();transactionEnd();return c;}
static int timedRead(){transactionBegin();int n=Wire.requestFrom((uint8_t)PCA_ADDR,(uint8_t)1);transactionEnd();return n;}

// Experimental bounded servo BUSERR recovery; never clears an existing FAULT.
static const uint32_t RETRY_US=60000, WINDOW_MS=1000, WINDOW_PAUSE_US=120000;
struct RetryState { bool active; uint8_t attempts; uint16_t on,off,pulse; uint32_t start,generation; };
static RetryState retry={};
static uint16_t requestedPulse=0;
static uint32_t episodeMs[I2C_EPISODE_LIMIT]={},episodePause[I2C_EPISODE_LIMIT]={};
static uint8_t episodeCount=0;
static uint32_t retryRevision=0,recoveredCount=0;
static const char *retryResult="idle";
static uint32_t lastPause=0;
size_t formatDriveRecovery(char *out,size_t size){
 return snprintf(out,size,"{\"type\":\"i2c_retry\",\"version\":4,\"revision\":%lu,\"active\":%s,\"result\":\"%s\",\"attempts\":%u,\"pause_us\":%lu,\"started_us\":%lu,\"recovered_count\":%lu,\"generation\":%lu}",(unsigned long)retryRevision,retry.active?"true":"false",retryResult,retry.attempts,(unsigned long)lastPause,(unsigned long)retry.start,(unsigned long)recoveredCount,(unsigned long)retry.generation);
}
static void retryNote(const char *reason){retryResult=reason;++retryRevision;eventLog("I2C_RETRY",reason);}
static void cancelRetry(){if(retry.active){lastPause=micros()-retry.start;retry.active=false;retryNote("cancelled_no_resume");}}
static void retryFault(const char *reason){lastPause=micros()-retry.start;retry.active=false;retryNote(reason);pcaHealthy=false;safetyStop(reason,true);}
static bool pureBuserr(){return lastIoFailure.code==4 && lastIoFailure.detailAvailable && lastIoFailure.causeBits==1 && !lastIoFailure.timeout;}
static bool recoverableIo(){return pureBuserr() || (lastIoFailure.timeout && (lastIoFailure.code==5 || lastIoFailure.code==0) && (!lastIoFailure.detailAvailable || lastIoFailure.causeBits==0));}
static bool beginRetry(uint8_t ch,uint16_t on,uint16_t off){
 if(ch!=SERVO_CH || !requestedPulse || !steeringEnabled || !servoProfileSynced || safety.state!=RunState::MANUAL || !recoverableIo() || lastIoFailure.duringStop[0] || estopActive())return false;
 uint8_t kept=0;
 for(uint8_t i=0;i<episodeCount;++i)if(millis()-episodeMs[i]<WINDOW_MS){episodeMs[kept]=episodeMs[i];episodePause[kept++]=episodePause[i];}
 episodeCount=kept;
 if(episodeCount>=I2C_EPISODE_LIMIT){retryNote("episode_limit");return false;}   // i2c-episode10-r1
 retry={true,0,on,off,requestedPulse,micros(),safety.generation};
 episodeMs[episodeCount]=millis();episodePause[episodeCount++]=0;
 retryNote("enable_suspended");return true;
}
static bool retryAllowed(){
 safetyTick(); // ESTOP, command timeout and cancellation retain priority.
 if(!retry.active)return false;
 if(safety.state!=RunState::MANUAL || safety.generation!=retry.generation || !steeringEnabled || !pcaHealthy){cancelRetry();safetyStop("I2C_RETRY_CANCELLED");return false;}
 if(micros()-retry.start>RETRY_US-10000){retryFault("I2C_RETRY_DEADLINE");return false;}
 return true;
}
static void retryTick(){
 if(!retryAllowed())return;
 if(retry.attempts>=3){retryFault("I2C_RETRY_EXHAUSTED");return;}
 ++retry.attempts;
 Wire.clearWireTimeoutFlag();Wire.beginTransmission(PCA_ADDR);Wire.write((uint8_t)(0x06+4*SERVO_CH));
 Wire.write((uint8_t)retry.on);Wire.write((uint8_t)(retry.on>>8));Wire.write((uint8_t)retry.off);Wire.write((uint8_t)(retry.off>>8));
 uint8_t code=timedEnd();
 if(code || Wire.getWireTimeoutFlag()){recordIoFailure("RETRY_WRITE",SERVO_CH,code,"WIRE",Wire.getWireTimeoutFlag());++i2cErrors;if(!recoverableIo() || retry.attempts>=3)retryFault("I2C_RETRY_WRITE_FAILED");return;}
 if(!retryAllowed())return;
 Wire.clearWireTimeoutFlag();Wire.beginTransmission(PCA_ADDR);Wire.write((uint8_t)(0x06+4*SERVO_CH));code=timedEnd();
 if(code || Wire.getWireTimeoutFlag()){recordIoFailure("RETRY_SELECT",SERVO_CH,code,"WIRE",Wire.getWireTimeoutFlag());++i2cErrors;if(!recoverableIo() || retry.attempts>=3)retryFault("I2C_RETRY_SELECT_FAILED");return;}
 if(!retryAllowed())return;
 Wire.clearWireTimeoutFlag();transactionBegin();int count=Wire.requestFrom((uint8_t)PCA_ADDR,(uint8_t)4);transactionEnd();
 if(count!=4 || Wire.getWireTimeoutFlag()){recordIoFailure("RETRY_READ",SERVO_CH,count,"READ_COUNT",Wire.getWireTimeoutFlag());++i2cErrors;if(!Wire.getWireTimeoutFlag() || retry.attempts>=3)retryFault("I2C_RETRY_READ_FAILED");return;}
 uint16_t on=Wire.read();on|=(uint16_t)Wire.read()<<8;uint16_t off=Wire.read();off|=(uint16_t)Wire.read()<<8;
 if(on!=retry.on || off!=retry.off){recordIoFailure("RETRY_VERIFY",SERVO_CH,-1,"REGISTER_VALUE",false);++i2cErrors;retryFault("I2C_RETRY_MISMATCH");return;}
 safetyTick();if(!retry.active)return;
 if(safety.state!=RunState::MANUAL || safety.generation!=retry.generation){cancelRetry();safetyStop("I2C_RETRY_CANCELLED");return;}
 lastPause=micros()-retry.start;uint32_t total=lastPause;
 for(uint8_t i=0;i+1<episodeCount;++i)total+=episodePause[i];
 if(lastPause>RETRY_US || total>WINDOW_PAUSE_US){retryFault("I2C_RETRY_TIME_LIMIT");return;}
 episodePause[episodeCount-1]=lastPause;steeringUs=retry.pulse;steeringOutputKnown=true;
 retry.active=false;++recoveredCount;retryNote("verified_resumed");
 analogWrite(MOTOR_A_EN,motorA.speed);analogWrite(MOTOR_B_EN,motorB.speed);
}
// Only register pointer selection/read: never re-enable outputs or clear the fault.
static void recoveryObservation(){
 static uint32_t at=0;
 if(safety.state!=RunState::FAULT || !firstIoFailure.valid || i2cRecoveryConfirmed() || millis()-at<250)return;
 at=millis();Wire.clearWireTimeoutFlag();Wire.beginTransmission(PCA_ADDR);Wire.write((uint8_t)0x00);
 if(timedEnd()!=0)return;
 if(timedRead()!=1)return;
 Wire.read();confirmI2cRecovery("MODE1_READ_ACK");
}


static bool output(uint8_t ch, uint16_t on, uint16_t off) {
  if (!pcaHealthy) return false;
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(PCA_ADDR);
  Wire.write((uint8_t)(0x06+4*ch));
  Wire.write((uint8_t)on);Wire.write((uint8_t)(on>>8));
  Wire.write((uint8_t)off);Wire.write((uint8_t)(off>>8));
  const uint8_t code=timedEnd();
  if (code == 0) {
    if (ch == SERVO_CH) steeringOutputKnown = true;
    return true;
  }
  recordIoFailure("PWM_WRITE",ch,code,"WIRE",Wire.getWireTimeoutFlag());
  steeringOutputKnown = false;
  ++i2cErrors;
  // Remove drive enable independently of the failed I2C bus.
  analogWrite(MOTOR_A_EN, 0);
  analogWrite(MOTOR_B_EN, 0);
  if(!beginRetry(ch,on,off))pcaHealthy=false;
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
// Direct single-register write works before library begin(), without auto-increment.
// Confirm CH0 FULL_OFF before any reset/frequency change. ACK/readback is not a waveform measurement.
static bool forceServoOff() {
  steeringOutputKnown = false;
  const uint8_t offHigh = 0x09 + 4 * SERVO_CH;
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(offHigh); Wire.write((uint8_t)0x10);
  uint8_t code=timedEnd();
  bool ok=code==0;
  if(!ok)recordIoFailure("OFF_WRITE",SERVO_CH,code,"WIRE",Wire.getWireTimeoutFlag());
  if(ok) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(PCA_ADDR);Wire.write(offHigh);
    code=timedEnd();ok=code==0;
    if(!ok)recordIoFailure("OFF_READ_SELECT",SERVO_CH,code,"WIRE",Wire.getWireTimeoutFlag());
    if(ok) {
      Wire.clearWireTimeoutFlag();
      int count=timedRead();ok=count==1;
      if(!ok)recordIoFailure("OFF_READ",SERVO_CH,count,"READ_COUNT",Wire.getWireTimeoutFlag());
    }
    if(ok) {
      int value=Wire.read();ok=(value & 0x10)!=0;
      if(!ok)recordIoFailure("OFF_VERIFY",SERVO_CH,value,"REGISTER_VALUE",Wire.getWireTimeoutFlag());
    }
  }
  steeringEnabled = false; // Stop software updates even when physical output is unknown.
  servoReturning = false;
  steeringOutputKnown = ok;
  if(ok)confirmI2cRecovery("SERVO_OFF_READBACK");
  if (!ok) {
    ++i2cErrors; pcaHealthy = false;
    analogWrite(MOTOR_A_EN, 0); analogWrite(MOTOR_B_EN, 0);
  }
  return ok;
}
static bool initializePca() {
  Wire.begin();
  Wire.setWireTimeout(10000, true); // Bound transactions on a missing/stuck PCA bus.
  if (!forceServoOff()) return false;
  Wire.clearWireTimeoutFlag();
  if (!pwm.begin()) { recordIoFailure("LIB_BEGIN",-1,-1,"LIBRARY_BOOL",Wire.getWireTimeoutFlag()); ++i2cErrors; return false; }
  pcaHealthy = true;
  // Keep CH0 off while configuring the PWM clock, then explicitly center below.
  servoOff();
  if (!pcaHealthy) return false;
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(50);
  // Reassert full-off immediately after changing the PWM frequency.
  servoOff();
  if (!pcaHealthy) return false;
  Wire.clearWireTimeoutFlag();
  prescale = pwm.readPrescale();
  if (prescale == 0 || prescale == 255) { recordIoFailure("PRESCALE_READ",-1,prescale,"REGISTER_VALUE",Wire.getWireTimeoutFlag()); pcaHealthy = false; ++i2cErrors; return false; }
  driveStop();
  return pcaHealthy;
}

static void bootWait(uint16_t duration) {
  const uint32_t started = millis();
  while (millis() - started < duration) {
    // No command dispatch during boot; EN stays zero, even with START held.
    analogWrite(MOTOR_A_EN, 0);
    analogWrite(MOTOR_B_EN, 0);
    telemetryTick();
    delay(1);
  }
}

bool driveBegin() {
  drivePinsBegin();
  pcaHealthy = false;
  Wire.begin();
  Wire.setWireTimeout(10000, true);
  forceServoOff(); // Earliest bus access, before the power-settle wait.
  bootWait(PCA_BOOT_WAIT_MS);
  for (uint8_t attempt = 1; attempt <= PCA_BOOT_ATTEMPTS; ++attempt) {
    char detail[24];
    snprintf(detail, sizeof(detail), "%u/%u", attempt, PCA_BOOT_ATTEMPTS);
    eventLog("PCA_INIT_ATTEMPT", detail);
    if (initializePca()) {
      eventLog("PCA_INIT_READY", "boot_retry_v1");
      break;
    }
    // Best-effort off; a broken I2C connection cannot confirm physical output.
    forceServoOff();
    steeringEnabled = false;
    steeringUs = 0;
    pcaHealthy = false;
    eventLog("PCA_INIT_RETRY_FAILED", steeringOutputKnown ? "servo_off_confirmed" : "servo_off_unconfirmed");
    if (attempt < PCA_BOOT_ATTEMPTS) bootWait(PCA_BOOT_RETRY_MS);
  }
  if (!pcaHealthy) return false;
  // Wait for the first validated profile from Orin; never use a fixed boot pulse.
  eventLog("BOOT_CENTER", "WAITING_ORIN_PROFILE");
  rampAt = probeAt = millis();
  return pcaHealthy;
}
void driveStop() {
  cancelRetry();
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
  if(retry.active){retryTick();return;}
  uint32_t now = millis();
  if (pcaHealthy && now - probeAt >= 1000) {
    probeAt = now;
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(PCA_ADDR);
    uint8_t code=timedEnd();
    if (code != 0) { recordIoFailure("PROBE",-1,code,"WIRE",Wire.getWireTimeoutFlag()); ++i2cErrors; pcaHealthy = false; steeringOutputKnown = false; }
  }
  if (!pcaHealthy) {
    recoveryObservation();
    if (safety.state != RunState::FAULT) safetyStop("PCA_IO_FAILED", true);
    return;
  }
  if (now - rampAt >= RAMP_MS) {
    rampAt = now;
    if (safety.state == RunState::ARMED || safety.state == RunState::MOTOR_TEST || safety.state == RunState::MANUAL) { ramp(motorA); if (pcaHealthy) ramp(motorB); }
  }
  if(safety.state==RunState::MANUAL && steeringEnabled && servoProfileSynced &&
     manualSteerTarget>=servoProfile.minimum && manualSteerTarget<=servoProfile.maximum && now-returnAt>=20){
    returnAt=now;
    int delta=constrain((int)manualSteerTarget-(int)steeringUs,-(int)MANUAL_STEER_STEP_US,(int)MANUAL_STEER_STEP_US);   // steer-slew-r1: 40 us / 20 ms = 2000 us/s (was 20 = 1000 us/s)
    requestedPulse=(uint16_t)((int)steeringUs+delta);
    if(delta && output(SERVO_CH,0,pulseCount(requestedPulse)))steeringUs=requestedPulse;
    requestedPulse=0;
  }
  if (SERVO_CALIBRATION_MODE) returnTick();
  if (!pcaHealthy) safetyStop("PCA_WRITE_FAILED", true);
}

void servoOff() {
  cancelRetry();
  forceServoOff(); // Attempt even after pcaHealthy became false; never clear the fault latch.
}
const char *servoCalibrationPulse(uint32_t generation, uint16_t pulseUs) {
  static uint32_t lastSetMs = 0;
  if (!SERVO_CALIBRATION_MODE) return "CALIBRATION_ONLY";
  if (estopActive()) return "ESTOP_ACTIVE";
  if (!pcaHealthy) return "PCA_FAULT";
  if (generation != safety.generation) return "STALE_GENERATION";
  if (safety.state != RunState::READY) return "PHYSICAL_START_REQUIRED";
  if (!servoProfileSynced) return "SERVO_PROFILE_REQUIRED";
  if (pulseUs < servoProfile.minimum || pulseUs > servoProfile.maximum) return "SERVO_LIMIT";
  if (pulseUs == 0) return "INVALID_PULSE";
  uint32_t count = (uint32_t)((float)pulseUs * PCA_OSC_HZ /
    (1000000.0f * (prescale + 1)) + 0.5f);
  if (count == 0 || count >= 4096) return "PWM_COUNTER_RANGE";
  // First calibration pulse is explicitly chosen by the user; position is unknown while off.
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
  if (!revision || minimum < SERVO_MIN_US || maximum > SERVO_MAX_US || !pulseCount(minimum) || minimum >= center || center >= maximum || pulseCount(maximum) >= 4096) return "BAD_PROFILE";
  if (steeringEnabled && (steeringUs < minimum || steeringUs > maximum)) return "CURRENT_OUTSIDE_LIMITS";
  servoProfile = {minimum, center, maximum, revision};
  servoProfileSynced = true;
  servoReturning = false;
  eventLog("SERVO_PROFILE", "CONFIGURED_NO_MOVEMENT");
  if (bootProfilePending) {
    bootProfilePending = false; // Only the first accepted profile after MCU boot.
    if (pcaHealthy && !estopActive() &&
        safety.state == RunState::LOCKED && strcmp(safety.reason, "BOOT_LOCK") == 0) {
      if (!output(SERVO_CH, 0, pulseCount(center))) {
        safetyStop("PCA_BOOT_CENTER_FAILED", true);
        return "PCA_FAULT";
      }
      steeringUs = center;
      steeringEnabled = true;
      eventLog("BOOT_CENTER", "ORIN_SAVED_PROFILE");
    } else {
      eventLog("BOOT_CENTER", "SKIPPED_STOP_OR_ESTOP");
    }
  }
  telemetryStatus();
  return nullptr;
}
const char *servoReturnToCenter(uint32_t generation) {
  if (!SERVO_CALIBRATION_MODE || !servoProfileSynced) return "SERVO_PROFILE_REQUIRED";
  if (generation != safety.generation) return "STALE_GENERATION";
  if (estopActive()) return "ESTOP_ACTIVE";
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
  if (safety.state != RunState::READY || safety.generation != returnGeneration || !steeringEnabled || estopActive()) {
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
