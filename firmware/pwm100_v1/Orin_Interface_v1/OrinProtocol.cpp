#include "OrinProtocol.h"
#include "Safety.h"
#include "Drive.h"
#include "MotorTest.h"
#include "ManualDrive.h"
#include "Telemetry.h"
#include "BleRemote.h"
#include <stdlib.h>
#include <string.h>
struct Receiver { char line[128]; uint8_t used=0; bool collecting=false; uint32_t started=0; };
static Receiver orinRx, usbRx;
static uint32_t lastSeq;
static bool haveSeq=false;
static uint16_t crc16(const char *p) {
  uint16_t crc=0xFFFF;
  while(*p) {
    crc^=(uint16_t)(uint8_t)*p++<<8;
    for(uint8_t i=0;i<8;i++) crc=(crc&0x8000) ? (crc<<1)^0x1021 : crc<<1;
  }
  return crc;
}
static bool number(const char *p, uint32_t &value) {
  if(!p || !*p) return false;
  uint32_t n=0;
  while(*p) {
    if(*p<'0'||*p>'9') return false;
    uint8_t digit=*p++-'0';
    if(n>(UINT32_MAX-digit)/10) return false;
    n=n*10+digit;
  }
  value=n; return true;
}
static bool percent(const char *p,int &value) {
  if(!p) return false;
  bool negative=*p=='-'; if(negative) ++p;
  uint32_t n; if(!number(p,n)||n>100) return false;
  value=negative ? -(int)n : (int)n; return true;
}
static void frame(char *line) {
  char *star=strchr(line,'*');
  if(!star||strlen(star+1)!=4) { ++rxBad; return; }
  for(char *p=star+1;*p;p++) if(!((*p>='0'&&*p<='9')||(*p>='A'&&*p<='F')||(*p>='a'&&*p<='f'))) { ++rxBad; return; }
  uint16_t supplied=(uint16_t)strtoul(star+1,nullptr,16);
  *star=0;
  if(crc16(line)!=supplied) { ++rxBad; return; }
  char *parts[8]; uint8_t count=1; parts[0]=line;
  for(char *p=line;*p;p++) if(*p==',') {
    if(count==8) { ++rxBad; return; }
    *p=0; parts[count++]=p+1;
  }
  uint32_t seq;
  if(count<2||!number(parts[0],seq)) { ++rxBad; return; }
  ++rxGood;
  const char *error=nullptr;
  bool stop=(strcmp(parts[1],"STOP")==0 && count==2);
  bool sync=(strcmp(parts[1],"SYNC")==0 && count==2);
  if(stop) {
    // A valid STOP is honored even if duplicated or out of order.
    safetyStop("REMOTE_STOP");
  } else if(sync) {
    if(safety.state==RunState::ARMED || safety.state==RunState::MOTOR_TEST || safety.state==RunState::MANUAL) error="STOP_FIRST";
    else { lastSeq=seq; haveSeq=true; ++safety.generation; }
  } else if(!haveSeq) error="SYNC_REQUIRED";
  else if((int32_t)(seq-lastSeq)<=0) error="STALE_SEQUENCE";
  else {
    lastSeq=seq;
    if(strcmp(parts[1],"STATUS")==0 && count==2) telemetryStatus();
    else if(strcmp(parts[1],"ARM")==0 && count==3) {
      uint32_t gen;
      error=number(parts[2],gen) ? safetyArm(gen) : "BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"MOVE")==0 && count==5) {
      uint32_t gen; int throttle,steer;
      error=number(parts[2],gen)&&percent(parts[3],throttle)&&percent(parts[4],steer)
        ? safetyMove(ControlMode::ORIN,gen,throttle,steer) : "BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"CAL_ENABLE")==0 && count==3) {
      uint32_t gen;error=number(parts[2],gen)?safetyCalibrationEnable(gen):"BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"SERVO")==0 && count==4) {
      uint32_t gen, pulse;
      error=number(parts[2],gen)&&number(parts[3],pulse)&&pulse<=65535
        ? servoCalibrationPulse(gen,(uint16_t)pulse) : "BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"MANUAL_ARM")==0 && count==3) {
      uint32_t gen;error=number(parts[2],gen)?manualArm(gen):"BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"MANUAL")==0 && count==6) {
      uint32_t gen,pulse;int right,left;
      error=number(parts[2],gen)&&percent(parts[3],right)&&percent(parts[4],left)&&number(parts[5],pulse)&&pulse<=65535
        ? manualMove(gen,right,left,(uint16_t)pulse):"BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"MANUAL_END")==0 && count==3) {
      uint32_t gen;error=number(parts[2],gen)?manualEnd(gen):"BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"TEST_ARM")==0 && count==4) {
      uint32_t gen,duration;
      error=number(parts[2],gen)&&number(parts[3],duration) ? motorTestArm(gen,duration) : "BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"TEST")==0 && count==5) {
      uint32_t gen;int right,left;
      error=number(parts[2],gen)&&percent(parts[3],right)&&percent(parts[4],left) ? motorTestMove(gen,right,left) : "BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"TEST_END")==0 && count==3) {
      uint32_t gen;
      error=number(parts[2],gen) ? motorTestEnd(gen) : "BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"PROFILE")==0 && count==7) {
      uint32_t gen,revision,minimum,center,maximum;
      error=number(parts[2],gen)&&number(parts[3],revision)&&number(parts[4],minimum)&&number(parts[5],center)&&number(parts[6],maximum)
        &&minimum<=65535&&center<=65535&&maximum<=65535
        ? servoConfigure(gen,revision,(uint16_t)minimum,(uint16_t)center,(uint16_t)maximum) : "BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"CENTER")==0 && count==3) {
      uint32_t gen;
      error=number(parts[2],gen) ? servoReturnToCenter(gen) : "BAD_ARGUMENTS";
    } else if(strcmp(parts[1],"MODE")==0 && count==3) {
      if(strcmp(parts[2],"ORIN")==0) error=safetyMode(ControlMode::ORIN);
      else if(strcmp(parts[2],"BLE")==0) error=safetyMode(ControlMode::BLE);
      else error="BAD_MODE";
    } else error="BAD_COMMAND";
  }
  if(error) ++rxRejected;
  replyLog(seq,error);
}
static void poll(Stream &port,Receiver &r,bool usb) {
  if(r.collecting && millis()-r.started>RX_FRAME_TIMEOUT_MS) {
    r.collecting=false; r.used=0; ++rxBad;
  }
  for(uint8_t budget=0;budget<32&&port.available();++budget) {
    char c=port.read();
    if(c=='$') { r.collecting=true; r.used=0; r.started=millis(); continue; }
    if(!r.collecting) {
      // Local diagnostics do not need a CRC; UART always requires framing.
      if(usb&&c=='?') telemetryStatus();
      if(usb&&c=='!') safetyStop("USB_STOP");
      if(usb && (c=='i'||c=='+'||c=='-'||c=='o')) servoCalibrationCommand(c);
      continue;
    }
    if(c=='\r') continue;
    if(c=='\n') {
      r.line[r.used]=0; r.collecting=false;
      frame(r.line); r.used=0;
    } else if(c<32||c>126||r.used>=sizeof(r.line)-1) {
      r.collecting=false; r.used=0; ++rxBad;
    } else r.line[r.used++]=c;
  }
}
void protocolTick() { poll(Serial1,orinRx,false); poll(Serial,usbRx,true); }
