#include "Telemetry.h"
#include "Safety.h"
#include "Drive.h"
#include "MotorTest.h"
#include "Encoder.h"
#include "BleRemote.h"
#include <stdio.h>
#include <string.h>
uint32_t rxGood=0, rxBad=0, rxRejected=0, txDropped=0, maxLoopUs=0;
// Independent queues: unplugged USB never holds up Orin or the control loop.
struct TxQueue { char bytes[2048]; uint16_t head=0, tail=0; };
static TxQueue orin, usb;
static uint32_t statusAt, lastLoop;
static uint16_t freeSpace(const TxQueue &q) { return (q.tail-q.head-1)&2047; }
static bool enqueue(TxQueue &q,const char *line) {
  size_t n=strlen(line);
  if (n+1>freeSpace(q)) return false;
  for(size_t i=0;i<n;i++) { q.bytes[q.head]=line[i]; q.head=(q.head+1)&2047; }
  q.bytes[q.head]='\n'; q.head=(q.head+1)&2047;
  return true;
}
static void emit(const char *line) {
  if (!enqueue(orin,line)) ++txDropped;
  // USB is optional and may drop independently.
  enqueue(usb,line);
}
void telemetryBegin() { lastLoop=micros(); statusAt=millis(); }
void eventLog(const char *event,const char *detail) {
  char line[256];
  snprintf(line,sizeof(line),"{\"type\":\"event\",\"ms\":%lu,\"event\":\"%s\",\"detail\":\"%s\",\"generation\":%lu}",
    (unsigned long)millis(),event,detail,(unsigned long)safety.generation);
  emit(line);
}
void replyLog(uint32_t seq,const char *error) {
  char line[240];
  snprintf(line,sizeof(line),"{\"type\":\"ack\",\"seq\":%lu,\"ok\":%s,\"reason\":\"%s\",\"generation\":%lu}",
    (unsigned long)seq,error ? "false":"true",error ? error:"OK",(unsigned long)safety.generation);
  emit(line);
}
void telemetryStatus() {
  char line[1800];
  const bool active=safety.state==RunState::ARMED || safety.state==RunState::MOTOR_TEST || safety.state==RunState::MANUAL;
  snprintf(line,sizeof(line),
    "{\"type\":\"status\",\"fw\":\"%s\",\"ms\":%lu,\"mode\":\"%s\",\"state\":\"%s\","
    "\"reason\":\"%s\",\"generation\":%lu,\"neutral_required\":%s,\"cmd_age_ms\":%ld,"
    "\"pins\":{\"D10\":%d,\"D11\":%d,\"D12_command\":%d},"
    "\"motor_a\":{\"pwm_command\":%d,\"target\":%d,\"dir\":%d,\"wanted_dir\":%d},"
    "\"motor_b\":{\"pwm_command\":%d,\"target\":%d,\"dir\":%d,\"wanted_dir\":%d},"
    "\"servo_calibration\":%s,\"steer_output_enabled\":%s,\"servo_profile_valid\":%s,\"steer_us_command\":%u,\"throttle_limit_pct\":%u,\"pca_ok\":%s,\"i2c_errors\":%lu,"
    "\"servo_profile\":{\"synced\":%s,\"revision\":%lu,\"min_us\":%u,\"center_us\":%u,\"max_us\":%u},\"servo_returning\":%s,"
    "\"ble_enabled\":%s,\"ble_connected\":%s,\"ble_bad_frames\":%lu,"
    "\"motor_test\":{\"active\":%s,\"started\":%s,\"start_ms\":%lu,\"duration_ms\":%lu,\"right_pct\":%d,\"left_pct\":%d},\"enc_sample_ms\":%lu,"
    "\"enc_enabled\":%s,\"enc_a\":{\"forward\":%lu,\"reverse\":%lu,\"delta\":%ld,\"edges\":%lu,\"invalid\":%lu,\"seen\":%s,\"age_ms\":%ld,\"ab\":%u},"
    "\"enc_b\":{\"forward\":%lu,\"reverse\":%lu,\"delta\":%ld,\"edges\":%lu,\"invalid\":%lu,\"seen\":%s,\"age_ms\":%ld,\"ab\":%u},"
    "\"voltage_v\":null,\"current_a\":null,\"speed_mps\":null,\"rx_good\":%lu,\"rx_bad\":%lu,\"rx_rejected\":%lu,\"tx_dropped\":%lu,\"max_loop_us\":%lu}",
    FW_VERSION,(unsigned long)millis(),modeName(),stateName(),safety.reason,(unsigned long)safety.generation,
    safety.neutralRequired ? "true":"false",active ? (long)(millis()-safety.lastMoveMs) : -1L,
    digitalRead(PIN_START),digitalRead(PIN_ESTOP),active ? 1:0,
    motorA.speed,motorA.target,motorA.dir,motorA.wantedDir,
    motorB.speed,motorB.target,motorB.dir,motorB.wantedDir,
    SERVO_CALIBRATION_MODE ? "true":"false",steeringEnabled ? "true":"false",servoProfileSynced ? "true":"false",
    steeringUs,MAX_THROTTLE_PERCENT,pcaHealthy ? "true":"false",(unsigned long)i2cErrors,
    servoProfileSynced ? "true":"false",(unsigned long)servoProfile.revision,servoProfile.minimum,servoProfile.center,servoProfile.maximum,servoReturning ? "true":"false",
    bleEnabled ? "true":"false",bleConnected ? "true":"false",(unsigned long)bleBadFrames,
    safety.state==RunState::MOTOR_TEST ? "true":"false",motorTestStarted ? "true":"false",(unsigned long)motorTestStartMs,(unsigned long)motorTestDurationMs,motorTestRight,motorTestLeft,(unsigned long)encoderSampleMs,
    ENCODERS_ENABLED ? "true":"false",
    (unsigned long)encA.forward,(unsigned long)encA.reverse,(long)encA.delta,(unsigned long)encA.edges,(unsigned long)encA.invalid,
    encA.seen ? "true":"false",encA.seen ? (long)encA.lastEdgeAge : -1L,encA.levels,
    (unsigned long)encB.forward,(unsigned long)encB.reverse,(long)encB.delta,(unsigned long)encB.edges,(unsigned long)encB.invalid,
    encB.seen ? "true":"false",encB.seen ? (long)encB.lastEdgeAge : -1L,encB.levels,
    (unsigned long)rxGood,(unsigned long)rxBad,(unsigned long)rxRejected,(unsigned long)txDropped,(unsigned long)maxLoopUs);
  emit(line);
}
void telemetryTick() {
  uint32_t nowUs=micros(),elapsed=nowUs-lastLoop; lastLoop=nowUs;
  if(elapsed>maxLoopUs) maxLoopUs=elapsed;
  if(millis()-statusAt>=TELEMETRY_MS) {
    statusAt=millis();
    // Leave room for acknowledgements/events under command flooding.
    if(freeSpace(orin)>1600) telemetryStatus(); else ++txDropped;
  }
  for(uint8_t i=0;i<32 && orin.tail!=orin.head && Serial1.availableForWrite()>0;i++) {
    Serial1.write((uint8_t)orin.bytes[orin.tail]); orin.tail=(orin.tail+1)&2047;
  }
  if (Serial) {
    for(uint8_t i=0;i<32 && usb.tail!=usb.head && Serial.availableForWrite()>0;i++) {
      Serial.write((uint8_t)usb.bytes[usb.tail]); usb.tail=(usb.tail+1)&2047;
    }
  } else usb.tail=usb.head;
}
