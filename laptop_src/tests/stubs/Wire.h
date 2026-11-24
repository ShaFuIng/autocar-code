#pragma once
#include "Arduino.h"
extern bool fakeI2cGood;
struct TwoWire {
 void begin(){}
 void setWireTimeout(uint32_t,bool){}
 void beginTransmission(int){}
 int endTransmission(){return fakeI2cGood?0:4;}
};
extern TwoWire Wire;
