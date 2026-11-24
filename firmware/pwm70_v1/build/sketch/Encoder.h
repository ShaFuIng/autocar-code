#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm70_v1\\Orin_Interface_v1\\Encoder.h"
#pragma once
#include "Config.h"
struct EncoderSnapshot {
  uint32_t forward, reverse, edges, invalid;
  int32_t delta;
  uint32_t lastEdgeAge;
  bool seen;
  uint8_t levels;
};
extern EncoderSnapshot encA, encB;
void encoderBegin();
void encoderTick();

extern uint32_t encoderSampleMs;
