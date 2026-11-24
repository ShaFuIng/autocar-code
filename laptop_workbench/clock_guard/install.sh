#!/bin/bash
set -euo pipefail
export LC_ALL=C
test "$(id -u)" = 0 || { echo 'Run with sudo bash.'; exit 1; }
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
test "$(timedatectl show -p NTPSynchronized --value)" = yes || { echo 'NTP must be synchronized before installation.'; exit 1; }
python3 -c 'import time; assert 1735689600 < time.time() < 4102444800'
python3 -m py_compile clock_guard.py
backup="/var/lib/orin-clock-guard/install-backup-$(date +%Y%m%dT%H%M%S)"
mkdir -p "$backup"
for unit in orin-clock-restore.service orin-clock-maintain.service orin-clock-maintain.timer; do
  if test -f "/etc/systemd/system/$unit"; then cp -a "/etc/systemd/system/$unit" "$backup/"; fi
done
if test -d /usr/local/lib/orin-clock-guard; then cp -a /usr/local/lib/orin-clock-guard "$backup/code"; fi
install -d -m 755 /usr/local/lib/orin-clock-guard
install -m 644 clock_guard.py /usr/local/lib/orin-clock-guard/clock_guard.py
for unit in orin-clock-restore.service orin-clock-maintain.service orin-clock-maintain.timer; do
  install -m 644 "$unit" "/etc/systemd/system/$unit"
done
systemd-analyze verify /etc/systemd/system/orin-clock-restore.service /etc/systemd/system/orin-clock-maintain.service /etc/systemd/system/orin-clock-maintain.timer
# Seed only from the currently synchronized clock; no clock change is performed.
python3 - <<'PY'
import json, os, time
from pathlib import Path
p = Path('/var/lib/orin-clock-guard/trusted.json')
tmp = p.with_suffix('.tmp')
with tmp.open('w') as f:
    json.dump({'epoch': time.time(), 'source': 'NTP synchronized at installation'}, f)
    f.flush(); os.fsync(f.fileno())
os.replace(tmp, p)
PY
systemctl daemon-reload
systemctl enable orin-clock-restore.service
systemctl enable --now orin-clock-maintain.timer
systemctl start orin-clock-maintain.service
python3 /usr/local/lib/orin-clock-guard/clock_guard.py --check
systemctl is-enabled orin-clock-restore.service orin-clock-maintain.timer
systemctl is-active orin-clock-maintain.timer
echo 'Installed. No reboot or vehicle-service restart. Cold-boot verification remains pending.'
