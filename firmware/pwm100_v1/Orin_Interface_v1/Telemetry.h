#pragma once
#include "Config.h"
extern uint32_t rxGood, rxBad, rxRejected, txDropped;
extern uint32_t maxLoopUs;
void telemetryBegin();
void telemetryTick();
void telemetryStatus();
void eventLog(const char *event, const char *detail);
void replyLog(uint32_t seq, const char *error);
