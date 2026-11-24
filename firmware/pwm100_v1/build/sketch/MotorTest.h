#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm100_v1\\Orin_Interface_v1\\MotorTest.h"
#pragma once
#include "Config.h"
extern bool motorTestStarted;
extern uint32_t motorTestStartMs, motorTestDurationMs;
extern int motorTestRight, motorTestLeft;
const char *motorTestArm(uint32_t generation, uint32_t durationMs);
const char *motorTestMove(uint32_t generation, int right, int left);
const char *motorTestEnd(uint32_t generation);
void motorTestCancel();
void motorTestTick();
