#include "ManualDrive.h"
#include "Drive.h"
#include "Safety.h"
#include "Encoder.h"
#include "Telemetry.h"
int manualRight=0,manualLeft=0;
uint16_t manualSteerTarget=0;
// mixed-direction-r1 (user 2026-11-06): each wheel keeps its own last direction; one wheel forward and the other reverse is
// allowed. A wheel that changes direction must have been at zero for 500 ms with no motor speed and no encoder counts.
static int lastDirRight=0,lastDirLeft=0;
static uint32_t lastNonzeroRightMs=0,lastNonzeroLeftMs=0;
void manualCancel(){manualRight=manualLeft=0;manualSteerTarget=0;lastDirRight=lastDirLeft=0;}
const char *manualArm(uint32_t generation){
  safetyTick();
  if(generation!=safety.generation)return "STALE_GENERATION";
  if(safety.mode!=ControlMode::ORIN || (safety.state!=RunState::READY && !(WEB_MANUAL_START && safety.state==RunState::LOCKED)))return "PHYSICAL_START_REQUIRED";
  if(estopActive())return "ESTOP_ACTIVE";
  if(!pcaHealthy || !servoProfileSynced)return "PROFILE_OR_PCA_FAULT";
  if(servoReturning)return "STEERING_BUSY";
  driveStop();manualCancel();
  // Preserve an active output until the first validated manual command arrives.
  manualSteerTarget=steeringEnabled ? steeringUs : servoProfile.center;
  safety.state=RunState::MANUAL;safety.reason="MANUAL_ARMED_NO_MOTION";
  safety.neutralRequired=true;safety.lastMoveMs=millis();
  eventLog("MANUAL","ARMED_NO_MOTION");return nullptr;
}
const char *manualMove(uint32_t generation,int right,int left,uint16_t pulse){
  safetyTick();
  if(generation!=safety.generation)return "STALE_GENERATION";
  if(safety.state!=RunState::MANUAL)return "MANUAL_NOT_ARMED";
  if(estopActive())return "ESTOP_ACTIVE";
  if(!pcaHealthy || !servoProfileSynced)return "PROFILE_OR_PCA_FAULT";
  if(abs(right)>MAX_THROTTLE_PERCENT || abs(left)>MAX_THROTTLE_PERCENT)return "PWM_LIMIT_50";
  if(pulse<servoProfile.minimum || pulse>servoProfile.maximum)return "SERVO_LIMIT";
  if(safety.neutralRequired && (right || left || pulse!=servoProfile.center))return "NEUTRAL_REQUIRED";
  int dirR=right>0?1:right<0?-1:0, dirL=left>0?1:left<0?-1:0;
  if(dirR && lastDirRight && dirR!=lastDirRight && (millis()-lastNonzeroRightMs<500 || motorA.speed || encA.delta))return "DIRECTION_CHANGE_WAIT";
  if(dirL && lastDirLeft && dirL!=lastDirLeft && (millis()-lastNonzeroLeftMs<500 || motorB.speed || encB.delta))return "DIRECTION_CHANGE_WAIT";
  if(!driveManualSet(right,left,pulse)){safetyStop("PCA_WRITE_FAILED",true);return "PCA_FAULT";}
  if(dirR){lastDirRight=dirR;lastNonzeroRightMs=millis();}
  if(dirL){lastDirLeft=dirL;lastNonzeroLeftMs=millis();}
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
