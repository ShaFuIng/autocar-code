#include <Arduino.h>
#line 1 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm70_v1\\Orin_Interface_v1\\Orin_Interface_v1.ino"
// BMduino Orin interface v1. Original Serial1 fix remains untouched.
#include "Config.h"
#include "Drive.h"
#include "Safety.h"
#include "Encoder.h"
#include "BleRemote.h"
#include "OrinProtocol.h"
#include "Telemetry.h"

#line 10 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm70_v1\\Orin_Interface_v1\\Orin_Interface_v1.ino"
void setup();
#line 22 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm70_v1\\Orin_Interface_v1\\Orin_Interface_v1.ino"
void loop();
#line 10 "D:\\OneDrive\\Desktop\\自走車\\firmware_builds\\pwm70_v1\\Orin_Interface_v1\\Orin_Interface_v1.ino"
void setup() {
  safetyBegin();             // Establish local stop inputs first.
  drivePinsBegin();          // PWM zero before UART/I2C/module initialization.
  Serial.begin(115200);
  Serial1.begin(115200);
  telemetryBegin();
  if (!driveBegin()) safetyStop("PCA_INIT_FAILED", true);
  encoderBegin();
  bleBegin();                // Advertising disabled; motors remain locked at boot.
  eventLog("BOOT", FW_VERSION);
}

void loop() {
  safetyTick();
  protocolTick();
  safetyTick();
  bleTick();
  safetyTick();
  driveTick();
  encoderTick();
  telemetryTick();
}

