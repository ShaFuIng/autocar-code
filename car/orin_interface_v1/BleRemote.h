#pragma once
#include "Config.h"
extern bool bleEnabled, bleConnected, bleInitOK;
extern uint32_t bleBadFrames;
void bleBegin();
bool bleSetEnabled(bool enabled);
void bleTick();
void bleDiscardPending();
