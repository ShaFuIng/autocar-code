#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm70_v1\\Orin_Interface_v1\\ManualDrive.h"
#pragma once
#include "Config.h"
extern int manualRight, manualLeft;
extern uint16_t manualSteerTarget;
const char *manualArm(uint32_t generation);
const char *manualMove(uint32_t generation,int right,int left,uint16_t pulse);
const char *manualEnd(uint32_t generation);
void manualTick();
void manualCancel();
