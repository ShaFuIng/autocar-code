#include "BleRemote.h"
#include "Safety.h"
#include "Telemetry.h"
#include "BM7701-00-1.h"
bool bleEnabled=false, bleConnected=false, bleInitOK=false;
uint32_t bleBadFrames=0;
static BM7701_00_1 module(&Serial3);
static uint8_t phase=0, length=0, used=0, data[255];
static uint32_t receivedAt;

void bleDiscardPending() {
  phase=used=0;
  // Hardware RX ring is bounded; consume only the bytes already present.
  int n=Serial3.available();
  while (n-- > 0) Serial3.read();
}
void bleBegin() {
  delay(60); // Module power-up requirement, before any motion authorization.
  module.begin(BAUD_115200);
  // A powered module may remember advertising settings from the old firmware.
  bool off=module.setAdvCtrl(DISABLE);
  eventLog("BLE", off ? "DISABLED" : "DISABLE_UNCONFIRMED_CONTROL_BLOCKED");
}
bool bleSetEnabled(bool enabled) {
  // Called only with drive output stopped. Library configuration can block.
  bleEnabled=false; bleConnected=false;
  bleDiscardPending();
  if (!enabled) {
    bool ok=module.setAdvCtrl(DISABLE);
    eventLog("BLE", ok ? "DISABLED" : "DISABLE_UNCONFIRMED_CONTROL_BLOCKED");
    return true; // Ownership gating still prevents all BLE driving.
  }
  uint8_t address[6]={0x11,0x22,0x33,0x44,0x55,0x66};
  uint8_t name[]={'B','M','C','_','C','A','R'};
  uint8_t adv[]={0x02,0x01,0x06}, scan[]={0x03,0x02,0x0f,0x18};
  bleInitOK = module.setAddress(address) && module.setName(sizeof(name),name)
    && module.setAdvIntv(160,160,7) && module.setAdvData(APPEND_NAME,sizeof(adv),adv)
    && module.setScanData(sizeof(scan),scan) && module.setTXpower(0x0F)
    && module.setCrystalOffset(0x04) && module.setFeature(FEATURE_DIR,AUTO_SEND_SATUS)
    && module.setAdvCtrl(ENABLE);
  bleEnabled=bleInitOK;
  bleDiscardPending();
  return bleInitOK;
}
static void handleFrame() {
  if (length >= 4 && data[0]==0 && data[1]==0) {
    bool connected=(data[3]&1)!=0;
    if (connected!=bleConnected) {
      bleConnected=connected;
      eventLog("BLE", connected ? "CONNECTED" : "DISCONNECTED");
      if (!connected && safety.mode==ControlMode::BLE) safetyStop("BLE_DISCONNECTED");
    }
  } else if (length==6 && data[0]==0 && data[1]==0xF2 && data[2]==0xFF && data[3]=='M') {
    if (!bleEnabled || safety.mode!=ControlMode::BLE || !bleConnected) return;
    int t=(int)data[4]-128, s=(int)data[5]-128;
    t=abs(t)<=6 ? 0 : constrain(t*100/127,-100,100);
    s=abs(s)<=6 ? 0 : constrain(s*100/127,-100,100);
    const char *error=safetyMove(ControlMode::BLE,safety.generation,t,s);
    // Avoid flooding while phone streams into a locked system.
    static uint32_t rejectionAt;
    if (error && millis()-rejectionAt>=1000) {
      rejectionAt=millis(); eventLog("BLE_REJECT",error);
    }
  } else ++bleBadFrames;
}
void bleTick() {
  if (phase && millis()-receivedAt>RX_FRAME_TIMEOUT_MS) {
    phase=used=0; ++bleBadFrames;
  }
  for (uint8_t budget=0; budget<32 && Serial3.available(); ++budget) {
    uint8_t ch=Serial3.read();
    receivedAt=millis();
    if (phase==0) { if(ch==0x78) phase=1; }
    else if (phase==1) {
      length=ch; used=0;
      if (length<3) { phase=0; ++bleBadFrames; } else phase=2;
    } else {
      data[used++]=ch;
      if (used==length) { phase=0; handleFrame(); }
    }
  }
}
