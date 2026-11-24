#pragma once
#include "Wire.h"
extern uint32_t fakeServoWrites;
extern int fakeBeginCalls, fakeBeginFailures, fakeWriteFailures, fakePrescaleFailures;
extern uint16_t fakePcaOff[16];
class Adafruit_PWMServoDriver {
 public:
 Adafruit_PWMServoDriver(int,TwoWire&){}
 bool begin(){++fakeBeginCalls;return fakeI2cGood && fakeBeginCalls>fakeBeginFailures;}
 void setOscillatorFrequency(uint32_t){}
 void setPWMFreq(int){}
 uint8_t readPrescale(){if(fakePrescaleFailures>0){--fakePrescaleFailures;return 255;}return 125;}
 uint8_t setPWM(uint8_t ch,uint16_t,uint16_t off){if(fakeWriteFailures>0){--fakeWriteFailures;return 1;}if(fakeI2cGood){fakePcaOff[ch]=off;if(ch==0)++fakeServoWrites;}return fakeI2cGood?0:1;}
};
