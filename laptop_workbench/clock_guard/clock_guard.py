#!/usr/bin/python3
"""Restore a trusted time floor after late RTC initialization; never roll back time.

No UART, vehicle API, model, or vehicle service operations.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

STATE = Path('/var/lib/orin-clock-guard')
RUNTIME = Path('/run/orin-clock-guard')
MIN_EPOCH = 1735689600  # 2025-01-01 UTC; reject 1970 and uninitialized files.
MAX_EPOCH = 4102444800  # 2100-01-01 UTC
PROFILE = 'Orin-ttuwifi5g'

def run(args):
    return subprocess.run(args, capture_output=True, text=True, timeout=35,
                          env={**os.environ, 'LC_ALL': 'C'})

def valid(value):
    return isinstance(value, (int, float)) and MIN_EPOCH <= value < MAX_EPOCH

def decision(now, candidates):
    floor = max((x for x in candidates if valid(x)), default=None)
    return floor, floor is not None and now < floor - 5

def atomic(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    with tmp.open('w') as stream:
        stream.write(json.dumps(value, indent=2) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)

def trusted_times():
    values = []
    try:
        values.append(json.loads((STATE/'trusted.json').read_text())['epoch'])
    except (OSError, ValueError, KeyError, TypeError):
        pass
    try:
        values.append(Path('/var/lib/systemd/timesync/clock').stat().st_mtime)
    except OSError:
        pass
    return values

def rtc_ready():
    p = Path('/sys/class/rtc/rtc0')
    try:
        return 'nvvrs-pseq-rtc' in (p/'name').read_text() and (p/'hctosys').read_text().strip() == '1'
    except OSError:
        return False

def wait_rtc():
    # Bound the boot delay; the maintenance timer handles a later device arrival.
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        if rtc_ready():
            time.sleep(2)  # Allow the driver's subsequent clock-setting step to finish.
            return True
        time.sleep(.25)
    return False

def retry_wifi():
    """Retry only the saved school profile, never disconnect another network."""
    # Retry exhaustion survives wall-clock jumps but is reset on reboot (/run).
    path = RUNTIME/'wifi-retry.json'
    try:
        record = json.loads(path.read_text())
    except (OSError, ValueError):
        record = {'attempts': 0, 'last_monotonic': -1000}
    now = time.monotonic()
    if record['attempts'] >= 3 or now - record['last_monotonic'] < 120:
        return 'retry limit/cooldown'
    devices = run(['nmcli', '-t', '-f', 'DEVICE,STATE', 'device', 'status'])
    if devices.returncode or 'wlP1p1s0:disconnected' not in devices.stdout.splitlines():
        return 'wireless not disconnected; unchanged'
    autoconnect = run(['nmcli', '-g', 'connection.autoconnect', 'connection', 'show', PROFILE])
    if autoconnect.returncode or autoconnect.stdout.strip() != 'yes':
        return 'saved profile unavailable or autoconnect disabled; unchanged'
    atomic(path, {'attempts': record['attempts'] + 1, 'last_monotonic': now})
    result = run(['nmcli', '--wait', '20', 'connection', 'up', PROFILE])
    return 'activation success' if result.returncode == 0 else 'activation failed; see NetworkManager journal'

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boot', action='store_true')
    parser.add_argument('--check', action='store_true', help='Read-only: no time, file, or network changes')
    args = parser.parse_args()
    rtc = wait_rtc() if args.boot and not args.check else rtc_ready()
    now = time.time()
    floor, restore = decision(now, trusted_times())
    report = {'epoch_before': now, 'trusted_floor': floor, 'rtc_ready': rtc,
              'would_restore': restore, 'check_only': args.check}
    if args.check:
        print(json.dumps(report)); return
    if floor is None:
        report['error'] = 'No trusted saved time; manual calibration required'
    elif restore:
        time.clock_settime(time.CLOCK_REALTIME, floor)
        report['restored'] = True
        # Persist until successful synchronization, including across timer invocations.
        atomic(RUNTIME/'recovery.json', {'pending': True})
    # Do not activate D-Bus services while holding NetworkManager's boot ordering.
    sync = None if args.boot else run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'])
    synced = sync is not None and sync.returncode == 0 and sync.stdout.strip() == 'yes'
    current = time.time()
    if synced and valid(current) and not restore:
        atomic(STATE/'trusted.json', {'epoch': current, 'source': 'NTPSynchronized=yes'})
        atomic(RUNTIME/'recovery.json', {'pending': False})
    # At boot NetworkManager has not started: never invoke it from the ordered boot unit.
    pending = False
    try:
        pending = json.loads((RUNTIME/'recovery.json').read_text()).get('pending', False)
    except (OSError, ValueError):
        pass
    if not args.boot and pending and valid(current) and not synced:
        report['wifi'] = retry_wifi()
    report.update(epoch_after=current, ntp_synchronized=synced, recovery_pending=pending)
    atomic(STATE/'status.json', report)
    print(json.dumps(report))

if __name__ == '__main__':
    main()
