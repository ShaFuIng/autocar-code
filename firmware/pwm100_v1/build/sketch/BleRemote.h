#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm100_v1\\Orin_Interface_v1\\BleRemote.h"
#pragma once
#include "Config.h"
extern bool bleEnabled, bleConnected, bleInitOK;
extern uint32_t bleBadFrames;
void bleBegin();
bool bleSetEnabled(bool enabled);
void bleTick();
void bleDiscardPending();
