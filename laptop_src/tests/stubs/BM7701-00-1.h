#pragma once
#include "Arduino.h"
#define BAUD_115200 115200
#define DISABLE false
#define ENABLE true
#define APPEND_NAME 0
#define FEATURE_DIR 0
#define AUTO_SEND_SATUS 128
class BM7701_00_1 {
 public:
 explicit BM7701_00_1(Stream*){}
 void begin(int){}
 bool setAdvCtrl(bool){return true;}
 bool setAddress(uint8_t*){return true;}
 bool setName(int,uint8_t*){return true;}
 bool setAdvIntv(int,int,int){return true;}
 bool setAdvData(int,int,uint8_t*){return true;}
 bool setScanData(int,uint8_t*){return true;}
 bool setTXpower(int){return true;}
 bool setCrystalOffset(int){return true;}
 bool setFeature(int,int){return true;}
};
