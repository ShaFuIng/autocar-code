#pragma once
#include <Arduino.h>
constexpr const char *FW_VERSION = "orin-v1.4.1-manual";
// Temporary field mode: explicit web MANUAL_ARM may authorize from LOCKED.
// Boot never starts motors; FAULT and active ESTOP remain blocked.
constexpr bool WEB_MANUAL_START = true;
constexpr bool WEB_TIMED_TEST_START = true;
constexpr uint8_t PIN_START = 10, PIN_ESTOP = 11, PIN_START_LED = 12;
constexpr uint32_t ESTOP_CONFIRM_US = 10000;   // estop-debounce-r1: a shorter LOW on D11 is a glitch (logged, no stop)
constexpr uint8_t MOTOR_A_EN = 8, MOTOR_B_EN = 9;
// Preserve the working firmware mapping, even where the old drawing differs.
constexpr uint8_t MOTOR_A_IN1 = 4, MOTOR_A_IN2 = 5;
constexpr uint8_t MOTOR_B_IN1 = 6, MOTOR_B_IN2 = 7;
constexpr bool MOTOR_A_INVERT = true, MOTOR_B_INVERT = false;
constexpr uint8_t PCA_ADDR = 0x40, SERVO_CH = 0;
// Experimental startup tolerance, not a voltage-stability measurement.
constexpr uint16_t PCA_BOOT_WAIT_MS = 1000, PCA_BOOT_RETRY_MS = 500;
constexpr uint8_t PCA_BOOT_ATTEMPTS = 3;
constexpr uint32_t PCA_OSC_HZ = 25880000UL;
// Boot steering waits for Orin saved profile; motors remain locked.
#ifndef ORIN_SERVO_CALIBRATION
#define ORIN_SERVO_CALIBRATION 1
#endif
constexpr bool SERVO_CALIBRATION_MODE = ORIN_SERVO_CALIBRATION != 0;
// Fill measured values before changing ORIN_SERVO_CALIBRATION to 0.
// TD-8135MG command envelope; linkage limits must be calibrated within it.
constexpr uint16_t SERVO_MIN_US = 500, SERVO_CENTER_US = 1500, SERVO_MAX_US = 2500;

// Profile limits are enforced in both calibration and manual driving.
constexpr uint16_t SERVO_TEST_REFERENCE_US = 1500; // Explicit USB 'i' only.
constexpr uint16_t SERVO_TEST_STEP_US = 100;
constexpr bool SERVO_PROFILE_VALID = SERVO_MIN_US > 0 &&
  SERVO_MIN_US < SERVO_CENTER_US && SERVO_CENTER_US < SERVO_MAX_US &&
  SERVO_MAX_US <= 65535;
constexpr uint16_t COMMAND_TIMEOUT_MS = 500, START_DEBOUNCE_MS = 30;
constexpr uint8_t MAX_THROTTLE_PERCENT = 70; // 2026-11-16 pwm70-r1: firmware ceiling raised 50 -> 70 (user request); the Orin side still caps at 50 until its json is changed. Experiment ceiling; not a hardware rating.
constexpr uint8_t RAMP_STEP = 5;
constexpr uint16_t MANUAL_STEER_STEP_US = 40;
constexpr uint8_t I2C_EPISODE_LIMIT = 10;   // i2c-episode10-r1 (2026-11-08): servo I2C retry episodes allowed per 1 s window (was 3)   // steer-slew-r1 (2026-11-08 step test: 20 us / 20 ms limited steering to 1000 us/s)
constexpr uint16_t RAMP_MS = 5, TELEMETRY_MS = 200;
// Inferred from drawing; wheel identity and polarity MUST be checked on vehicle.
constexpr bool ENCODERS_ENABLED = true;
constexpr uint8_t ENC_A_A = 4, ENC_A_B = 5, ENC_B_A = 6, ENC_B_B = 7;
constexpr int8_t ENC_A_SIGN = 1, ENC_B_SIGN = 1;
constexpr uint16_t RX_FRAME_TIMEOUT_MS = 100;
