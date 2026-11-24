#!/bin/bash
set -euo pipefail
test "$(id -u)" = 0 || { echo 'Run with sudo bash.'; exit 1; }
systemctl disable --now orin-clock-maintain.timer
systemctl disable orin-clock-restore.service
systemctl stop orin-clock-maintain.service
echo 'Clock guard disabled. Saved time, code and backups preserved; vehicle services unchanged.'
