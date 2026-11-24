#pragma once
#include <Arduino.h>
constexpr const char *FW_VERSION = "orin-v1.4.0-manual";
constexpr uint8_t PIN_START = 10, PIN_ESTOP = 11, PIN_START_LED = 12;
constexpr uint8_t MOTOR_A_EN = 8, MOTOR_B_EN = 9;
// Preserve the working firmware mapping, even where the old drawing differs.
constexpr uint8_t MOTOR_A_IN1 = 4, MOTOR_A_IN2 = 5;
constexpr uint8_t MOTOR_B_IN1 = 6, MOTOR_B_IN2 = 7;
constexpr bool MOTOR_A_INVERT = true, MOTOR_B_INVERT = false;
constexpr uint8_t PCA_ADDR = 0x40, SERVO_CH = 0;
constexpr uint32_t PCA_OSC_HZ = 25880000UL;
// No boot movement; timed TEST and physically authorized MANUAL modes are explicit.
#ifndef ORIN_SERVO_CALIBRATION
#define ORIN_SERVO_CALIBRATION 1
#endif
constexpr bool SERVO_CALIBRATION_MODE = ORIN_SERVO_CALIBRATION != 0;
// Fill measured values before changing ORIN_SERVO_CALIBRATION to 0.
constexpr uint16_t SERVO_MIN_US = 1010, SERVO_CENTER_US = 1460, SERVO_MAX_US = 1810;
// Profile limits are enforced in both calibration and manual driving.
constexpr uint16_t SERVO_TEST_REFERENCE_US = 1460; // Explicit USB 'i' only.
constexpr uint16_t SERVO_TEST_STEP_US = 10;
constexpr bool SERVO_PROFILE_VALID = SERVO_MIN_US > 0 &&
  SERVO_MIN_US < SERVO_CENTER_US && SERVO_CENTER_US < SERVO_MAX_US &&
  SERVO_MAX_US <= 65535;
constexpr uint16_t COMMAND_TIMEOUT_MS = 500, START_DEBOUNCE_MS = 30;
constexpr uint8_t MAX_THROTTLE_PERCENT = 25; // First bring-up limit for both sources.
constexpr uint8_t RAMP_STEP = 5;
constexpr uint16_t RAMP_MS = 5, TELEMETRY_MS = 200;
// Inferred from drawing; wheel identity and polarity MUST be checked on vehicle.
constexpr bool ENCODERS_ENABLED = true;
constexpr uint8_t ENC_A_A = 4, ENC_A_B = 5, ENC_B_A = 6, ENC_B_B = 7;
constexpr int8_t ENC_A_SIGN = 1, ENC_B_SIGN = 1;
constexpr uint16_t RX_FRAME_TIMEOUT_MS = 100;
