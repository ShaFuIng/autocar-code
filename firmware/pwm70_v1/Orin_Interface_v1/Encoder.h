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
