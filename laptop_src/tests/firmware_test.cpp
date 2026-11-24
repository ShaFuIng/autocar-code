// Executes the actual firmware modules with mocked GPIO/UART/I2C hardware.
#include "Arduino.h"
#include "Wire.h"
#include "Safety.h"
#include "Drive.h"
#include "MotorTest.h"
#include "ManualDrive.h"
#include "Encoder.h"
#include "BleRemote.h"
#include "OrinProtocol.h"
#include "Telemetry.h"
#include <cassert>
#include <cstdio>
#include <iostream>
uint32_t fakeMs=0, fakeServoWrites=0;
int fakeBeginCalls=0, fakeBeginFailures=0, fakeWriteFailures=0, fakePrescaleFailures=0;
uint16_t fakePcaOff[16]={};
uint32_t fakeIrqMask=0,fakeSerialFlushes=0;
int pinLevels[64],pwmLevels[64];
void (*callbacks[64])()={};
bool fakeI2cGood=true;
Stream Serial,Serial1,Serial3;
TwoWire Wire;
static uint32_t sequence=0;
static std::string frame(uint32_t seq,const std::string &command) {
  std::string body=std::to_string(seq)+","+command;
  uint16_t crc=0xFFFF;
  for(unsigned char c:body) {
    crc^=(uint16_t)c<<8;
    for(int i=0;i<8;i++)crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;
  }
  char suffix[16];snprintf(suffix,sizeof(suffix),"*%04X\n",crc);
  return "$"+body+suffix;
}
static void send(const std::string &command) {
  Serial1.feed(frame(++sequence,command));
  while(Serial1.available())protocolTick();
  for(int i=0;i<100;i++)telemetryTick();
}
static void tick(uint32_t ms) {fakeMs+=ms;safetyTick();driveTick();}
static void start() {
  pinLevels[PIN_START]=HIGH; safetyTick();tick(31);
  pinLevels[PIN_START]=LOW;safetyTick();tick(31);
  pinLevels[PIN_START]=HIGH;safetyTick();tick(31);
}
static std::string gen(){return std::to_string(safety.generation);}
static void arm(){start();assert(safety.state==RunState::READY);send("ARM,"+gen());assert(safety.state==RunState::ARMED);send("MOVE,"+gen()+",0,0");}
static void drive(){send("MOVE,"+gen()+",100,10");for(int i=0;i<20;i++)tick(5);assert(motorA.speed==63);assert(motorB.speed==63);}
static void bleFrame(std::initializer_list<uint8_t> data) {
  Serial3.rx.push_back(0x78);Serial3.rx.push_back((uint8_t)data.size());
  for(auto c:data)Serial3.rx.push_back(c);
  while(Serial3.available())bleTick();
  for(int i=0;i<100;i++)telemetryTick();
}
int main(int argc, char **argv) {
  for(auto &p:pinLevels)p=HIGH;
  if(argc>1 && std::string(argv[1]).find("pca-boot-")==0) {
    const std::string scenario=argv[1];
    if(scenario=="pca-boot-recover" || scenario=="pca-boot-estop")fakeBeginFailures=2;
    if(scenario=="pca-boot-exhaust")fakeBeginFailures=99;
    if(scenario=="pca-boot-write")fakeWriteFailures=1;
    if(scenario=="pca-boot-prescale")fakePrescaleFailures=1;
    if(scenario=="pca-boot-estop")pinLevels[PIN_ESTOP]=LOW;
    pinLevels[PIN_START]=LOW; // A held start must not authorize boot motion.
    safetyBegin();drivePinsBegin();telemetryBegin();
    const bool ok=driveBegin();
    if(!ok)safetyStop("PCA_INIT_FAILED",true);
    assert(pwmLevels[8]==0 && pwmLevels[9]==0);
    assert(fakeBeginCalls<=PCA_BOOT_ATTEMPTS);
    assert(fakeMs==PCA_BOOT_WAIT_MS+(fakeBeginCalls-1)*PCA_BOOT_RETRY_MS);
    if(scenario=="pca-boot-exhaust") {
      assert(!ok && !pcaHealthy && !steeringEnabled && safety.state==RunState::FAULT);
      fakeI2cGood=true;fakeBeginFailures=0;tick(2000);
      assert(fakeBeginCalls==3 && safety.state==RunState::FAULT); // No runtime recovery.
    } else {
      assert(ok && pcaHealthy && safety.state==RunState::LOCKED);
      assert(i2cErrors==(uint32_t)(fakeBeginCalls-1));
      assert(steeringEnabled==(pinLevels[PIN_ESTOP]!=LOW));
      if(steeringEnabled)assert(steeringUs==1360);
      fakeI2cGood=false;tick(1001);
      assert(safety.state==RunState::FAULT && pwmLevels[8]==0 && pwmLevels[9]==0);
      const int calls=fakeBeginCalls;fakeI2cGood=true;tick(1001);
      assert(fakeBeginCalls==calls && safety.state==RunState::FAULT);
    }
    std::cout << scenario << " PASS\n";return 0;
  }
  if(argc>1 && std::string(argv[1])=="boot-estop")pinLevels[PIN_ESTOP]=LOW;
  if(argc>1 && std::string(argv[1])=="boot-i2c-fault")fakeI2cGood=false;
  if(!fakeI2cGood){safetyBegin();drivePinsBegin();assert(!driveBegin());assert(!steeringEnabled&&pwmLevels[8]==0&&pwmLevels[9]==0);return 0;}
  safetyBegin();drivePinsBegin();telemetryBegin();assert(driveBegin());encoderBegin();bleBegin();
  assert(safety.state==RunState::LOCKED&&!bleEnabled);
  assert(motorA.target==0&&motorB.target==0&&pwmLevels[8]==0&&pwmLevels[9]==0);
  if(pinLevels[PIN_ESTOP]==LOW){assert(!steeringEnabled&&fakePcaOff[0]==4096);return 0;}
  assert(steeringEnabled&&steeringUs==1360&&fakePcaOff[0]>0&&fakePcaOff[0]<4096);
  tick(1000);assert(steeringEnabled&&steeringUs==1360&&pwmLevels[8]==0&&pwmLevels[9]==0);
  if(argc>1 && std::string(argv[1])=="web-calibration"){
    send("SYNC");send("PROFILE,"+gen()+",1,500,1360,2500");
    auto writes=fakeServoWrites;auto old=safety.generation;
    send("CAL_ENABLE,"+gen());
    assert(safety.state==RunState::READY && safety.generation==old+1);
    assert(fakeServoWrites==writes && motorA.target==0 && motorB.target==0);
    assert(safetyCalibrationEnable(old)!=nullptr);
    safetyStop("TEST_STOP");writes=fakeServoWrites;
    assert(safetyCalibrationEnable(safety.generation)==nullptr);
    assert(fakeServoWrites==writes && motorA.speed==0 && motorB.speed==0);
    safetyStop("TEST_STOP");pinLevels[PIN_ESTOP]=LOW;
    assert(safetyCalibrationEnable(safety.generation)!=nullptr);
    pinLevels[PIN_ESTOP]=HIGH;safetyStop("TEST_FAULT",true);
    assert(safetyCalibrationEnable(safety.generation)!=nullptr);
    std::cout<<"WEB CALIBRATION PASS: explicit enable, no servo writes, zero motors, generation, ESTOP, FAULT\n";
    return 0;
  }
  if(argc>1 && std::string(argv[1])=="web-start"){
    assert(WEB_MANUAL_START); send("SYNC");
    send("PROFILE,"+gen()+",1,500,1360,2500");
    assert(safety.state==RunState::LOCKED);
    send("MANUAL_ARM,"+gen());
    assert(safety.state==RunState::MANUAL && safety.neutralRequired);
    assert(motorA.target==0 && motorB.target==0);
    assert(manualMove(safety.generation,10,10,1360)!=nullptr);
    assert(manualMove(safety.generation,0,0,1360)==nullptr);
    tick(501);assert(safety.state==RunState::LOCKED);
    uint32_t old=safety.generation;
    safetyStop("TEST_STOP");
    assert(manualArm(old)!=nullptr);
    pinLevels[PIN_ESTOP]=LOW;
    assert(manualArm(safety.generation)!=nullptr);
    pinLevels[PIN_ESTOP]=HIGH;
    safetyStop("TEST_FAULT",true);
    assert(manualArm(safety.generation)!=nullptr);
    assert(motorA.target==0 && motorB.target==0);
    std::cout<<"WEB START PASS: zero power, neutral, stale generation, ESTOP, FAULT, watchdog\n";
    return 0;
  }
  if (SERVO_CALIBRATION_MODE) {
    send("STOP");steeringUs=0; // Remaining tests exercise explicit re-enable after stop.
    assert(!steeringEnabled && steeringUs==0 && !servoProfileSynced && fakePcaOff[0]==4096);
    send("SYNC");start();send("SERVO,"+gen()+",1460");assert(!steeringEnabled);
    send("STOP");auto writes=fakeServoWrites;
    send("PROFILE,"+gen()+",1,1010,1460,1810");
    assert(servoProfileSynced && servoProfile.center==1460 && fakeServoWrites==writes && !steeringEnabled);
    send("PROFILE,"+gen()+",2,1460,1460,1810");assert(servoProfile.revision==1);
    send("PROFILE,"+gen()+",2,1010,1460,65535");assert(servoProfile.revision==1);
    send("PROFILE,"+std::to_string(safety.generation-1)+",2,1010,1470,1810");assert(servoProfile.center==1460);
    send("PROFILE,"+gen()+",2,499,1460,2500");assert(servoProfile.revision==1);
    send("PROFILE,"+gen()+",2,500,1460,2501");assert(servoProfile.revision==1);
    start();send("ARM,"+gen());assert(safety.state==RunState::READY);
    send("MOVE,"+gen()+",25,0");assert(motorA.target==0 && motorB.target==0);
    send("SERVO,"+gen()+",1340");assert(steeringEnabled && steeringUs==1340);
    tick(120);send("SERVO,"+gen()+",1470");assert(steeringUs==1340); // Large subsequent jump still rejected.
    tick(120);send("SERVO,"+gen()+",1440");assert(steeringUs==1440);tick(120);
    send("SERVO,"+gen()+",1460");assert(steeringEnabled && steeringUs==1460);
    tick(120);send("SERVO,"+gen()+",1470");assert(steeringUs==1470);
    send("SERVO,"+gen()+",1480");assert(steeringUs==1470); // Rate limit.
    tick(120);send("SERVO,"+gen()+",1480");assert(steeringUs==1480);
    tick(120);send("SERVO,"+gen()+",1811");assert(steeringUs==1480);
    send("SERVO,"+gen()+",1009");assert(steeringUs==1480);
    send("SERVO,"+gen()+",1590");assert(steeringUs==1480); // >100 step.
    for(int pulse:{1580,1680,1780,1810}){tick(120);send("SERVO,"+gen()+","+std::to_string(pulse));assert(steeringUs==pulse);}
    send("CENTER,"+gen());assert(servoReturning && steeringUs==1810);
    tick(50);assert(steeringUs==1810);tick(70);assert(steeringUs==1800);
    for(int i=0;i<40&&servoReturning;i++){auto before=steeringUs;tick(120);assert(abs((int)steeringUs-before)<=10);}
    assert(!servoReturning && steeringUs==1460);
    writes=fakeServoWrites;
    send("PROFILE,"+gen()+",2,1010,1470,1810");assert(servoProfile.center==1470 && steeringUs==1460 && fakeServoWrites==writes);
    send("PROFILE,"+gen()+",3,1480,1500,1810");assert(servoProfile.revision==2);
    send("CENTER,"+gen());tick(120);assert(steeringUs==1470&&!servoReturning);
    tick(120);send("SERVO,"+gen()+",1570");send("CENTER,"+gen());tick(120);assert(servoReturning);
    send("STOP");auto stopped=steeringUs;tick(500);assert(!steeringEnabled&&!servoReturning&&steeringUs==stopped);
    start();send("SERVO,"+gen()+",1470");assert(steeringEnabled);
    tick(120);send("SERVO,"+gen()+",1570");send("CENTER,"+gen());
    pinLevels[PIN_ESTOP]=LOW;tick(120);assert(!steeringEnabled&&!servoReturning&&safety.state==RunState::LOCKED);
    pinLevels[PIN_ESTOP]=HIGH;start();send("SERVO,"+gen()+",1470");tick(30001);assert(steeringEnabled&&safety.state==RunState::READY);
    tick(120);send("SERVO,"+gen()+",1570");send("CENTER,"+gen());send("SYNC");tick(120);assert(!servoReturning&&steeringUs==1570);
    tick(120);send("CENTER,"+gen());tick(120);tick(120);tick(100);send("SERVO,"+gen()+",1560");assert(!servoReturning);auto manual=steeringUs;tick(300);assert(steeringUs==manual);
    assert(motorA.speed==0 && motorB.speed==0 && pwmLevels[8]==0 && pwmLevels[9]==0);
    send("STOP");send("MODE,BLE");assert(!bleEnabled);
    start();send("TEST_ARM,"+gen()+",5000");assert(safety.state==RunState::MOTOR_TEST&&motorA.target==0);
    send("TEST,"+gen()+",10,10");assert(motorA.target==0&&safety.neutralRequired);
    send("TEST,"+gen()+",0,0");assert(!safety.neutralRequired);
    auto heldSteer=steeringUs;auto heldEnabled=steeringEnabled;
    send("SERVO,"+gen()+",1480");assert(steeringUs==heldSteer);
    send("PROFILE,"+gen()+",9,1010,1470,1810");assert(servoProfile.revision==2);
    send("TEST,"+gen()+",51,10");assert(motorA.target==0);
    send("TEST,"+gen()+",10,15");for(int i=0;i<20;i++)tick(5);
    assert(motorA.speed==25&&motorB.speed==38&&steeringUs==heldSteer&&steeringEnabled==heldEnabled);
    auto lease=safety.lastMoveMs;Serial1.feed(frame(sequence,"TEST,"+gen()+",10,15"));while(Serial1.available())protocolTick();assert(safety.lastMoveMs==lease);
    tick(501);assert(safety.state==RunState::LOCKED&&motorA.speed==0&&motorB.speed==0);
    start();send("TEST_ARM,"+gen()+",600");send("TEST,"+gen()+",0,0");send("TEST,"+gen()+",15,0");
    for(int i=0;i<20;i++){tick(5);}
    assert(motorA.speed==38&&motorB.speed==0);
    tick(200);send("TEST,"+gen()+",15,0");tick(200);send("TEST,"+gen()+",15,0");tick(100);
    assert(safety.state==RunState::READY&&motorA.speed==0&&motorB.speed==0&&strcmp(safety.reason,"MOTOR_TEST_COMPLETE")==0);
    send("TEST,"+gen()+",15,15");assert(motorA.target==0);
    send("TEST_ARM,"+gen()+",5000");send("TEST,"+gen()+",0,0");send("TEST,"+gen()+",-10,-15");for(int i=0;i<20;i++)tick(5);assert(motorA.dir==-1&&motorB.dir==-1);
    pinLevels[PIN_ESTOP]=LOW;tick(1);assert(safety.state==RunState::LOCKED&&motorA.speed==0&&motorB.speed==0);
    pinLevels[PIN_ESTOP]=HIGH;start();send("TEST_ARM,"+gen()+",5000");send("TEST,"+gen()+",0,0");send("TEST,"+gen()+",10,10");tick(100);send("TEST_END,"+gen());assert(safety.state==RunState::READY&&motorA.speed==0);
    send("TEST_ARM,"+gen()+",5000");send("TEST,"+gen()+",0,0");send("TEST,"+gen()+",10,10");send("STOP");assert(safety.state==RunState::LOCKED&&motorA.speed==0);
    start();send("MANUAL_ARM,"+gen());assert(safety.state==RunState::MANUAL&&!steeringEnabled&&motorA.target==0);
    send("MANUAL,"+gen()+",10,15,1470");assert(safety.neutralRequired&&motorA.target==0);
    send("MANUAL,"+gen()+",0,0,1470");assert(!safety.neutralRequired&&steeringEnabled);
    send("MANUAL,"+gen()+",10,15,1570");
    for(int i=0;i<20;i++){auto before=steeringUs;tick(5);assert(abs((int)steeringUs-before)<=20);}
    assert(motorA.speed==25&&motorB.speed==38);tick(20);assert(steeringUs==1570);
    send("TEST_ARM,"+gen()+",5000");assert(safety.state==RunState::MANUAL);
    send("PROFILE,"+gen()+",99,1010,1470,1810");assert(servoProfile.revision==2);
    send("SYNC");assert(safety.state==RunState::MANUAL);
    send("MANUAL,"+gen()+",51,15,1570");assert(manualRight==10);
    send("MANUAL,"+gen()+",10,-15,1570");assert(manualLeft==15);
    send("MANUAL,"+gen()+",10,15,1811");assert(manualSteerTarget==1570);
    send("MANUAL,"+gen()+",0,0,1470");assert(motorA.speed==0&&motorB.speed==0);
    send("MANUAL,"+gen()+",-10,-15,1470");assert(manualRight==0);
    for(int i=0;i<3;i++){tick(200);send("MANUAL,"+gen()+",0,0,1470");}
    send("MANUAL,"+gen()+",-10,-15,1470");assert(manualRight==-10&&manualLeft==-15);
    auto manualLease=safety.lastMoveMs;
    Serial1.feed(frame(sequence,"MANUAL,"+gen()+",10,15,1570"));while(Serial1.available())protocolTick();assert(safety.lastMoveMs==manualLease);
    tick(501);assert(safety.state==RunState::LOCKED&&motorA.speed==0&&!steeringEnabled);
    start();send("MANUAL_ARM,"+gen());send("MANUAL,"+gen()+",0,0,1470");send("MANUAL,"+gen()+",10,10,1570");
    pinLevels[PIN_ESTOP]=LOW;tick(1);assert(safety.state==RunState::LOCKED&&motorA.speed==0&&!steeringEnabled);
    pinLevels[PIN_ESTOP]=HIGH;
    start();fakeI2cGood=false;send("SERVO,"+gen()+",1470");assert(!steeringEnabled&&safety.state==RunState::FAULT&&pwmLevels[8]==0);
    for(int i=0;i<300;i++)telemetryTick();
    rxGood=rxBad=rxRejected=txDropped=UINT32_MAX;i2cErrors=bleBadFrames=maxLoopUs=UINT32_MAX;
    encA.forward=encA.reverse=encA.edges=encA.invalid=UINT32_MAX;encB=encA;encA.delta=encB.delta=INT32_MIN;
    telemetryStatus();for(int i=0;i<300;i++)telemetryTick();
    std::cout<<Serial1.tx;
    std::cerr<<"Servo and motor bench behavior tests passed\n";
    return 0;
  }

  send("MOVE,0,100,0");assert(motorA.target==0);
  send("SYNC");
  send("ARM,"+gen());assert(safety.state==RunState::LOCKED);
  start();send("ARM,"+gen());
  send("MOVE,"+gen()+",20,0");assert(motorA.target==0&&safety.neutralRequired);
  send("MOVE,"+gen()+",0,0");drive();
  // Status does not refresh the motion lease.
  tick(300);send("STATUS");tick(101);
  assert(safety.state==RunState::LOCKED&&motorA.speed==0&&pwmLevels[8]==0);
  auto oldGen=gen();
  arm();send("MOVE,"+oldGen+",10,0");assert(motorA.target==0);
  drive();
  // Held START cannot release an ESTOP, even via UART ARM.
  pinLevels[PIN_START]=LOW;safetyTick();tick(31);
  pinLevels[PIN_ESTOP]=LOW;safetyTick();
  assert(safety.state==RunState::LOCKED&&motorA.speed==0);
  send("ARM,"+gen());assert(safety.state==RunState::LOCKED);
  pinLevels[PIN_ESTOP]=HIGH;tick(100);assert(safety.state==RunState::LOCKED);
  arm();drive();
  // Reverse ramps to zero before changing direction.
  send("MOVE,"+gen()+",-20,0");tick(5);assert(motorA.dir==1&&motorA.target==0);
  for(int i=0;i<25;i++)tick(5);
  assert(motorA.dir==-1&&motorA.speed==51);
  // Old sequence cannot change outputs or extend lease.
  auto lastMove=safety.lastMoveMs;
  Serial1.feed(frame(sequence,"MOVE,"+gen()+",20,0"));while(Serial1.available())protocolTick();
  assert(safety.lastMoveMs==lastMove&&motorA.wantedDir==-1);
  // Malformed CRC, missing newline, excessive length and split frames.
  Serial1.feed("$999999,MOVE,0,100,0*0000\n");while(Serial1.available())protocolTick();
  assert(motorA.wantedDir==-1);
  Serial1.feed("$123,MOVE");protocolTick();tick(101);protocolTick();
  Serial1.feed("$"+std::string(200,'A')+"\n");while(Serial1.available())protocolTick();
  assert(rxBad>=3);
  auto stop=frame(0,"STOP");Serial1.feed(stop.substr(0,5));protocolTick();
  Serial1.feed(stop.substr(5));protocolTick();assert(safety.state==RunState::LOCKED);
  // Valid STOP is effective even if sequence is older.
  send("MODE,BLE");assert(safety.mode==ControlMode::BLE&&bleEnabled);
  bleFrame({0,0,0,1});start();assert(safety.state==RunState::ARMED);
  bleFrame({0,0xF2,0xFF,'M',255,128});assert(motorA.target==0);
  bleFrame({0,0xF2,0xFF,'M',128,128});
  bleFrame({0,0xF2,0xFF,'M',255,128});for(int i=0;i<20;i++)tick(5);
  assert(motorA.speed==63);
  send("MOVE,"+gen()+",-20,0");assert(motorA.wantedDir==1); // Wrong owner.
  bleFrame({0,0,0,0});assert(safety.state==RunState::LOCKED&&motorA.speed==0);
  send("MODE,ORIN");assert(!bleEnabled);
  // Partial BLE packet never waits for its missing bytes.
  Serial3.feed(std::string("\x78\x06\x00",3));bleTick();tick(101);bleTick();assert(bleBadFrames>0);
  // Encoder detects transitions but zero observations remain unknown.
  tick(200);encoderTick();assert(!encA.seen);
  for(int state:{2,0,1,3}) {
    pinLevels[4]=(state>>1)&1;pinLevels[5]=state&1;callbacks[4]();
  }
  tick(200);encoderTick();assert(encA.seen&&encA.edges==4&&encA.invalid==0&&abs(encA.delta)==4);
  pinLevels[4]=0;pinLevels[5]=0;callbacks[4]();tick(200);encoderTick();assert(encA.invalid==1);
  // HT32 snapshots must not invoke serial flushing while IRQs are masked.
  assert(fakeSerialFlushes==0&&fakeIrqMask==0);
  fakeIrqMask=1;fakeMs+=200;auto beforeSnapshot=fakeMs;encoderTick();
  assert(fakeIrqMask==1&&fakeMs==beforeSnapshot&&fakeSerialFlushes==0);
  fakeIrqMask=0;
  // Unplugged / full output sinks never block control processing.
  Serial1.capacity=Serial.capacity=0;
  for(int i=0;i<100;i++){tick(10);telemetryTick();}
  assert(txDropped>0);
  Serial1.capacity=Serial.capacity=128;
  for(int i=0;i<300;i++)telemetryTick();
  // I2C failure removes PWM and latches a fault until reset.
  arm();drive();fakeI2cGood=false;send("MOVE,"+gen()+",20,20");
  assert(safety.state==RunState::FAULT&&pwmLevels[8]==0&&pwmLevels[9]==0);
  fakeI2cGood=true;start();assert(safety.state==RunState::FAULT);
  // Output records must remain complete JSON even at max counters.
  rxGood=rxBad=rxRejected=txDropped=UINT32_MAX;
  i2cErrors=bleBadFrames=maxLoopUs=UINT32_MAX;
  encA.forward=encA.reverse=encA.edges=encA.invalid=UINT32_MAX;
  encB=encA;encA.delta=encB.delta=INT32_MIN;
  telemetryStatus();for(int i=0;i<300;i++)telemetryTick();
  std::cout<<Serial1.tx;
  std::cerr<<"Firmware behavior tests passed\n";
}

