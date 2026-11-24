#!/bin/sh
# Record-only MPU6050 logger; one instance. Not part of vehicle control.
cd /home/jetson/orin_interface_v1/imu || exit 1
mkdir -p ../logs/imu
if pgrep -f "[p]ython3 imu_logger.py" >/dev/null; then echo already-running; exit 0; fi
setsid nohup python3 imu_logger.py --out ../logs/imu >> ../logs/imu/logger.out 2>&1 < /dev/null &
echo started
