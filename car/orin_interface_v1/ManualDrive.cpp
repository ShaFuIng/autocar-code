#include "ManualDrive.h"
#include "Drive.h"
#include "Safety.h"
#include "Encoder.h"
#include "Telemetry.h"
int manualRight=0,manualLeft=0;
uint16_t manualSteerTarget=0;
static int lastDirection=0;
static uint32_t lastNonzeroMs=0;
void manualCancel(){manualRight=manualLeft=0;manualSteerTarget=0;lastDirection=0;}
const char *manualArm(uint32_t generation){
  safetyTick();
  if(generation!=safety.generation)return "STALE_GENERATION";
  if(safety.mode!=ControlMode::ORIN || safety.state!=RunState::READY)return "PHYSICAL_START_REQUIRED";
  if(digitalRead(PIN_ESTOP)==LOW)return "ESTOP_ACTIVE";
  if(!pcaHealthy || !servoProfileSynced)return "PROFILE_OR_PCA_FAULT";
  if(servoReturning)return "STEERING_BUSY";
  driveStop();manualCancel();
  safety.state=RunState::MANUAL;safety.reason="MANUAL_ARMED_NO_MOTION";
  safety.neutralRequired=true;safety.lastMoveMs=millis();
  eventLog("MANUAL","ARMED_NO_MOTION");return nullptr;
}
const char *manualMove(uint32_t generation,int right,int left,uint16_t pulse){
  safetyTick();
  if(generation!=safety.generation)return "STALE_GENERATION";
  if(safety.state!=RunState::MANUAL)return "MANUAL_NOT_ARMED";
  if(digitalRead(PIN_ESTOP)==LOW)return "ESTOP_ACTIVE";
  if(!pcaHealthy || !servoProfileSynced)return "PROFILE_OR_PCA_FAULT";
  if(abs(right)>MAX_THROTTLE_PERCENT || abs(left)>MAX_THROTTLE_PERCENT)return "PWM_LIMIT_25";
  if((right>0&&left<0)||(right<0&&left>0))return "MIXED_DIRECTION";
  if(pulse<servoProfile.minimum || pulse>servoProfile.maximum)return "SERVO_LIMIT";
  if(safety.neutralRequired && (right || left || pulse!=servoProfile.center))return "NEUTRAL_REQUIRED";
  int dir=right? (right>0?1:-1):left?(left>0?1:-1):0;
  if(dir && lastDirection && dir!=lastDirection &&
     (millis()-lastNonzeroMs<500 || motorA.speed || motorB.speed || encA.delta || encB.delta))return "DIRECTION_CHANGE_WAIT";
  if(!driveManualSet(right,left,pulse)){safetyStop("PCA_WRITE_FAILED",true);return "PCA_FAULT";}
  if(dir){lastDirection=dir;lastNonzeroMs=millis();}
  manualRight=right;manualLeft=left;manualSteerTarget=pulse;
  safety.neutralRequired=false;safety.lastMoveMs=millis();return nullptr;
}
const char *manualEnd(uint32_t generation){
  if(generation!=safety.generation)return "STALE_GENERATION";
  if(safety.state!=RunState::MANUAL)return "MANUAL_NOT_ARMED";
  safetyStop("MANUAL_END");return nullptr;
}
void manualTick(){
  if(safety.state==RunState::MANUAL && millis()-safety.lastMoveMs>COMMAND_TIMEOUT_MS)safetyStop("COMMAND_TIMEOUT");
}
