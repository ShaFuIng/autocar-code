#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm100_v1\\Orin_Interface_v1\\Telemetry.h"
#pragma once
#include "Config.h"
extern uint32_t rxGood, rxBad, rxRejected, txDropped;
extern uint32_t maxLoopUs;
void telemetryBegin();
void telemetryTick();
void telemetryStatus();
void eventLog(const char *event, const char *detail);
void replyLog(uint32_t seq, const char *error);
